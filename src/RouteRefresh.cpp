#include "traffic/Simulation.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <utility>

namespace traffic {
bool Simulation::refreshMovingRoute() {
    if(!routing_.rtsStyle || !routing_.refreshMovingRoutes) return false;
    Entity* selected=nullptr;
    for(auto& e:entities_) {
        if(flow_.predictiveJunctions && !e.networkRoute.empty()) continue;
        if(!e.laneRoute.empty()) continue;
        // Native trips already have a crossing committed at spawn. Keep that
        // decision through clear free space; refresh only an avoidance detour.
        if(e.inOpenJunction && !e.usingLocalDetour) continue;
        if(e.inOpenJunction && (e.goal.x-e.center.x)*e.exitDirection.x+(e.goal.y-e.center.y)*e.exitDirection.y<e.length*2+2) continue;
        double interval=e.usingLocalDetour?std::min(routing_.movingRefreshInterval,routing_.localInterval):routing_.movingRefreshInterval;
        if(e.movingSinceRefresh+1e-9<interval || e.speed<=stoppedSpeedThreshold ||
           e.reversing || e.tugActive || e.turnExitBlocked || ownsNegotiatedTurn(e) || maneuverYieldTo(e) || e.signalLight!=-1 || e.waitingForLight || e.signalQueue ||
           (!e.reservedTurn.empty() && !e.usingLocalDetour) || e.next>=e.path.size()) continue;
        bool redApproach=false;
        for(const auto& light:lights_) if(lightStops(light,e)) {
            Vec2 previous=e.center;
            for(std::size_t i=e.next;i<e.path.size();++i) {
                if(geometry::intersectsLine({previous,e.path[i]},light.a,light.b)) { redApproach=true; break; }
                previous=e.path[i];
            }
            if(redApproach) break;
        }
        if(redApproach && !e.usingLocalDetour) continue;
        // Oldest accumulated moving time first prevents high-ID units starving
        // when the requested refresh rate exceeds the one-plan/substep budget.
        if(!selected || e.movingSinceRefresh>selected->movingSinceRefresh) selected=&e;
    }
    if(!selected) return false;
    auto& e=*selected;
    e.movingSinceRefresh=0; ++destinationRefreshAttempts_;
    // Reissue the terrain route with no random edge noise. Let normal RTS
    // local avoidance handle temporary vehicles instead of making a global
    // detour around an instantaneous traffic snapshot on every refresh.
    // installPath builds from the current pose and leaves the old route intact
    // if its curved connector cannot fit. Neither call changes goal or speed.
    // Zero-noise replans must not perturb the future spawn/route random stream.
    auto spawnRandomState=random_;
    auto guide=plan(e,0,false);
    random_=spawnRandomState;
    if(guide.empty()) return true;
    Entity candidate=e;
    if(!installPath(candidate,std::move(guide),e.usingLocalDetour?std::max(4.0,e.length*2):0)) return true;
    auto remainingDistance=[](const Entity& entity) {
        double distance=0; Vec2 previous=entity.center;
        for(std::size_t i=entity.next;i<entity.path.size();++i) {
            distance+=length(entity.path[i]-previous); previous=entity.path[i];
        }
        return distance;
    };
    double oldDistance=remainingDistance(e),newDistance=remainingDistance(candidate);
    // A repeated move command should not churn an already good route because
    // of small sampling differences. Replace only a meaningfully shorter path.
    double improvement=e.usingLocalDetour?.05:std::max(.5,oldDistance*.05);
    if(oldDistance-newDistance>improvement) {
        if(e.usingLocalDetour) {
            // Validate the return and immediate following corridor. Distant
            // traffic is handled by normal following control after merging.
            // Include the next motion interval beyond the nominal horizon:
            // a legal boundary contact alone does not establish a clear merge.
            double horizon=std::max({routing_.lookAhead,e.length*2,e.speed*e.speed/(4*e.acceleration)+e.length})+std::max(.25,e.speed*.1);
            auto from=e.footprint(); double checked=0;
            for(std::size_t i=candidate.next;i<candidate.path.size() && checked<horizon;++i) {
                double distance=length(candidate.path[i]-from.center);
                double fraction=distance>0?std::min(1.0,(horizon-checked)/distance):1;
                Footprint to{from.center+(candidate.path[i]-from.center)*fraction,
                    from.heading+angleDelta(from.heading,candidate.pathHeadings[i])*fraction,e.length,e.width};
                if(!feasible(from,to,e,true) || blockedByLight(from,to,e)) return true;
                checked+=distance*fraction; from=to;
            }
        }
        // A new connector starting at the current pose must be admissible now;
        // otherwise retain the moving detour rather than stop midway through it.
        if(!candidate.maneuvers.empty() && candidate.maneuvers.front().begin==0 &&
           !reserveManeuver(candidate,candidate.maneuvers.front())) return true;
        e=std::move(candidate); ++replans_;
        e.replanFailures=0;
    }
    ++destinationRefreshes_; ++e.destinationRefreshes;
    return true;
}
}
