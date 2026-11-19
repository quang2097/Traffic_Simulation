#include "traffic/Simulation.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace traffic {
const char* stopReasonName(StopReason reason) {
    switch(reason) {
    case StopReason::Moving: return "Moving";
    case StopReason::Starting: return "Starting / observing stop";
    case StopReason::Arrived: return "At destination";
    case StopReason::Signal: return "Legal stop: red / yellow signal";
    case StopReason::Vehicle: return "Legal stop: vehicle ahead";
    case StopReason::Road: return "Legal stop: road boundary / obstruction";
    case StopReason::Yielding: return "Planned reverse yield";
    case StopReason::Deadlock: return "Circular vehicle blockage: coordinated yield";
    case StopReason::ManeuverYield: return "Legal stop: yielding to negotiated turn";
    case StopReason::BlockedExit: return "Legal stop: waiting for clear turn exit";
    default: return "Unexplained stop: fleet recovery required";
    }
}
Footprint Simulation::stopProbe(const Entity& e) const {
    auto from=e.footprint(),to=from;
    to.center=from.center+Vec2{std::cos(e.heading),std::sin(e.heading)}*.25;
    if(e.next<e.path.size() && e.pathHeadings.size()==e.path.size()) {
        double remaining=.25;
        // The next dense curve sample may be only millimeters away. Walk the
        // full probe distance so a real blockage is not labeled unexplained.
        for(std::size_t i=e.next;i<e.path.size();++i) {
            auto delta=e.path[i]-to.center;double distance=length(delta);
            if(distance<1e-9) continue;
            double fraction=std::min(1.,remaining/distance);
            to.center=to.center+delta*fraction;
            to.heading+=angleDelta(to.heading,e.pathHeadings[i])*fraction;
            remaining-=distance*fraction;
            if(remaining<1e-9) break;
        }
    }
    return to;
}
StopReason Simulation::stopReason(const Entity& e) const {
    if(e.reversing) return StopReason::Yielding;
    if(e.speed>stoppedSpeedThreshold) return StopReason::Moving;
    if(e.waitingForJunction) return StopReason::BlockedExit;
    if(e.turnExitBlocked) return e.deadlocked?StopReason::Deadlock:StopReason::BlockedExit;
    if(maneuverYieldTo(e) && !e.clearingTurn) return StopReason::ManeuverYield;
    if(length(e.goal-e.center)<.25) {
        if(e.inOpenJunction && e.networkStage+1<e.networkRoute.size()) return e.currentStopSeconds>trafficRecoverySettings_.stopSeconds?StopReason::Unexplained:StopReason::Starting;
        return StopReason::Arrived;
    }
    // Verify current geometry and phases, never trust cached waiting flags.
    auto from=e.footprint(),to=stopProbe(e);
    if(blockedByLight(from,to,e)) return StopReason::Signal;
    if(e.signalLight>=0 && e.signalLight<static_cast<int>(lights_.size()) &&
       lightStops(lights_[e.signalLight],e) && length(e.center-e.signalTarget)<.3) return StopReason::Signal;
    if(e.signalLight>=0 && e.signalLight<static_cast<int>(lights_.size()) &&
       !lightStops(lights_[e.signalLight],e) && e.next>=e.path.size()) return StopReason::Starting;
    double pad; auto sweep=geometry::swept(from,to,pad);
    for(const auto& other:entities_) if(other.id!=e.id) {
        double gap=std::min(e.preferredDistance,other.preferredDistance);
        // Existing undersized gaps must be allowed to separate in recovery.
        if(!geometry::motionClearance(from,to,other.footprint(),gap,true)) return e.deadlocked?StopReason::Deadlock:StopReason::Vehicle;
    }
    if(!geometry::fits(map_,sweep,pad)) return StopReason::Road;
    if(e.currentStopSeconds<trafficRecoverySettings_.stopSeconds) return StopReason::Starting;
    return StopReason::Unexplained;
}
void Simulation::setTrafficRecoverySettings(TrafficRecoverySettings settings) {
    if(!std::isfinite(settings.stopSeconds)) settings.stopSeconds=1;
    settings.stopSeconds=std::clamp(settings.stopSeconds,.1,30.0);
    if(!std::isfinite(settings.radius)) settings.radius=25;
    settings.radius=std::clamp(settings.radius,5.,250.);
    trafficRecoverySettings_=settings;
}
void Simulation::planStoppedTraffic(bool force) {
    if(recoveryPreview_ || (!force && (!trafficRecoverySettings_.enabled || seconds_<nextTrafficRecoveryCheck_))) return;
    nextTrafficRecoveryCheck_=seconds_+.25;
    std::vector<std::size_t> seeds,group;
    // A physical blockage is ordinary only if its wait chain can make progress.
    // Circular waits with no moving or signal-controlled root need negotiation.
    std::vector<StopReason> reasons; std::vector<int> blocker(entities_.size(),-1);
    for(auto& e:entities_) { e.deadlocked=false; reasons.push_back(stopReason(e)); }
    for(std::size_t i=0;i<entities_.size();++i) if((reasons[i]==StopReason::Vehicle || reasons[i]==StopReason::BlockedExit) &&
        entities_[i].currentStopSeconds>=std::max(1.0,trafficRecoverySettings_.stopSeconds*2)) {
        if(reasons[i]==StopReason::BlockedExit) {
            for(std::size_t j=0;j<entities_.size();++j) if(entities_[j].id==entities_[i].exitWaitFor) { blocker[i]=static_cast<int>(j); break; }
            continue;
        }
        auto from=entities_[i].footprint(),to=stopProbe(entities_[i]);
        for(std::size_t j=0;j<entities_.size();++j) if(i!=j &&
            !geometry::motionClearance(from,to,entities_[j].footprint(),std::min(entities_[i].preferredDistance,entities_[j].preferredDistance),true)) {
            blocker[i]=static_cast<int>(j); break;
        }
    }
    for(std::size_t i=0;i<entities_.size();++i) {
        std::vector<int> chain; int node=static_cast<int>(i);
        while(node>=0) {
            auto repeated=std::find(chain.begin(),chain.end(),node);
            if(repeated!=chain.end()) {
                for(auto at=repeated;at!=chain.end();++at) entities_[*at].deadlocked=true;
                break;
            }
            chain.push_back(node); node=blocker[node];
        }
    }
    auto eligible=[&](const Entity& e) {
        if(flow_.predictiveJunctions && !e.networkRoute.empty()) return false;
        if(!e.laneRoute.empty()) return false;
        auto reason=stopReason(e);
        bool crossingConflict=e.inOpenJunction && reason==StopReason::Vehicle && e.currentStopSeconds>=trafficRecoverySettings_.stopSeconds;
        return (e.deadlocked || reason==StopReason::Unexplained || crossingConflict) &&
            (force || seconds_>=e.trafficRecoveryAfter);
    };
    for(std::size_t i=0;i<entities_.size();++i) if(eligible(entities_[i])) seeds.push_back(i);
    if(seeds.empty()) return;
    // Resolve independent knots separately. Requiring a single maneuver to
    // release two unrelated jams can reject a perfectly good local solution.
    auto root=*std::max_element(seeds.begin(),seeds.end(),[&](auto a,auto b) {
        return entities_[a].currentStopSeconds<entities_[b].currentStopSeconds;
    });
    std::vector<bool> connected(entities_.size(),false); connected[root]=true;
    for(auto i:seeds) if(!entities_[i].deadlocked && length(entities_[i].center-entities_[root].center)<6) connected[i]=true;
    for(std::size_t pass=0;pass<entities_.size();++pass) {
        bool changed=false;
        for(std::size_t i=0;i<blocker.size();++i) if(blocker[i]>=0 && connected[i]!=connected[blocker[i]]) {
            connected[i]=connected[blocker[i]]=true; changed=true;
        }
        if(!changed) break;
    }
    seeds.erase(std::remove_if(seeds.begin(),seeds.end(),[&](auto i){ return !connected[i]; }),seeds.end());
    const auto start=std::chrono::steady_clock::now(); ++trafficRecoveryPasses_;
    group=seeds;
    // Reconcile the local crossing neighborhood together, including moving
    // traffic. Keep legal signal queues and guided roads on their own routes.
    for(std::size_t i=0;i<entities_.size();++i) {
        const auto& e=entities_[i];
        if(!e.inOpenJunction || !e.laneRoute.empty() || e.signalLight!=-1 || e.reversing || reasons[i]==StopReason::Signal) continue;
        if(length(e.center-entities_[root].center)<=trafficRecoverySettings_.radius && std::find(group.begin(),group.end(),i)==group.end()) group.push_back(i);
    }
    // Include stationary vehicles behind a deadlock when they need to back up
    // together to make room. Never commandeer a moving actor or a real signal slot.
    for(std::size_t at=0;at<group.size();++at) {
        auto i=group[at]; if(!entities_[i].deadlocked && at<seeds.size()) continue;
        auto from=entities_[i].footprint(),to=from;
        to.center=to.center-Vec2{std::cos(from.heading),std::sin(from.heading)}*.75;
        for(std::size_t j=0;j<entities_.size();++j) if(j!=i && entities_[j].laneRoute.empty() && entities_[j].speed<=stoppedSpeedThreshold &&
            (reasons[j]==StopReason::Vehicle || reasons[j]==StopReason::BlockedExit) && std::find(group.begin(),group.end(),j)==group.end() &&
            !geometry::motionClearance(from,to,entities_[j].footprint(),std::min(entities_[i].preferredDistance,entities_[j].preferredDistance),true)) group.push_back(j);
    }
    // Map the entire fleet to its original endpoints in one private world.
    // Working committed curves and legal signal approaches remain in place;
    // their existing routes participate in every rollout as moving obstacles.
    Simulation mapped=*this; mapped.recoveryPreview_=true; lastFleetRouteCount_=0;
    std::vector<Entity> routes=entities_;
    for(std::size_t i=0;i<entities_.size();++i) {
        if(std::find(group.begin(),group.end(),i)==group.end()) {
            // The live route already maps a working vehicle to its endpoint.
            // Reuse it in the fleet rollout instead of repeating terrain A*.
            ++lastFleetRouteCount_; continue;
        }
        auto path=mapped.plan(entities_[i],0,entities_[i].inOpenJunction);
        if(!path.empty() && mapped.installPath(routes[i],std::move(path),std::max(4.0,entities_[i].length*2)))
            ++lastFleetRouteCount_;
        else if(entities_[i].next<entities_[i].path.size() || entities_[i].signalLight>=0 || entities_[i].reversing)
            ++lastFleetRouteCount_; // Retain a valid existing approach / exit.
    }
    double bestScore=.15; std::vector<Entity> best;
    int reverseChoices=static_cast<int>(std::min<std::size_t>(group.size(),6));
    for(int variant=0;variant<3+reverseChoices;++variant) {
        Simulation trial=*this; trial.recoveryPreview_=true;
        for(auto i:group) {
            auto& e=trial.entities_[i]; e.reservedTurn.clear(); e.reservedApproach.clear();
            e.turnExitBlocked=false; e.exitWaitFor=0;
            e.turnPadding=0; e.reservationEnd=0;
        }
        bool valid=true;
        for(auto i:group) {
            auto& e=trial.entities_[i];
            if(variant==1) {
                if(routes[i].next>=routes[i].path.size()) { valid=false; break; }
                e=routes[i];
            }
            if(e.signalLight!=-1) {
                // An unexplained stop may carry a stale signal slot. Reissue
                // its endpoint; real red crossings are still checked per step.
                if(routes[i].next>=routes[i].path.size()) { valid=false; break; }
                e=routes[i]; e.signalLight=-1; e.signalExit={};
            }
            e.cooperativeRecovery=true; e.waitingForManeuver=false;
            e.reservationEnd=e.path.size();
            for(const auto& turn:e.maneuvers) if(turn.end>=e.next) { e.reservationEnd=turn.end; break; }
            e.replanAfter=seconds_+2; e.localSteerAfter=seconds_+2;
            e.trafficRecoveryAfter=seconds_+(urbanTraffic_.enabled?1.5:3);
        }
        if(!valid) continue;
        if(variant>=2 && variant<2+reverseChoices && !trial.beginRecovery(trial.entities_[group[variant-2]],true)) continue;
        if(variant==2+reverseChoices) {
            // Plan a rear-first convoy against virtual final poses, then preview
            // from the original poses. Physical motion never uses virtual bodies.
            Simulation staging=trial; bool yielded=false;
            std::vector<bool> planned(group.size(),false);
            for(std::size_t pass=0;pass<group.size();++pass) {
                bool changed=false;
                for(std::size_t k=0;k<group.size();++k) if(!planned[k]) {
                    auto i=group[k]; auto& e=staging.entities_[i];
                    if(!staging.beginRecovery(e,true)) continue;
                    trial.entities_[i]=e;
                    trial.entities_[i].reservedTurn.clear(); trial.entities_[i].reservedApproach.clear();
                    e.center=e.yieldTarget; e.reservedTurn.clear(); e.reservedApproach.clear();
                    planned[k]=true; changed=yielded=true;
                }
                if(!changed) break;
            }
            if(!yielded) continue;
        }
        const auto proposal=trial.entities_;
        for(int step=0;step<(variant>=2?400:150);++step) trial.tick(.01);
        double score=std::numeric_limits<double>::infinity();
        for(auto i:seeds) {
            const auto& original=entities_[i];
            auto found=std::find_if(trial.entities_.begin(),trial.entities_.end(),[&](const Entity& e){ return e.id==original.id; });
            double progress=found==trial.entities_.end()?1.0:
                found->reversing?0.0:length(found->center-original.center);
            score=std::min(score,progress); // Every unexplained root must improve.
        }
        if(score>bestScore) { bestScore=score; best=proposal; }
        if(variant<2 && !best.empty()) break; // Prefer forward repair to needless reversing.
    }
    for(auto i:group) entities_[i].trafficRecoveryAfter=seconds_+(urbanTraffic_.enabled?1.5:3);
    if(!best.empty()) {
        // Only initial plans are committed: rollout poses, clocks, statistics and RNG stay private.
        for(auto i:group) {
            if(best[i].reversing && !entities_[i].reversing) ++recoveries_;
            best[i].speed=entities_[i].speed;
            entities_[i]=std::move(best[i]);
        }
        ++trafficRecoveryRepairs_; ++replans_;
    }
    lastTrafficRecoveryMilliseconds_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    totalTrafficRecoverySeconds_+=lastTrafficRecoveryMilliseconds_/1000;
}
}
