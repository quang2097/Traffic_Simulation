#include "traffic/Simulation.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace traffic {
namespace {
double dot(Vec2 a,Vec2 b) { return a.x*b.x+a.y*b.y; }
}
void Simulation::setTurnNegotiationSettings(TurnNegotiationSettings s) {
    if(!std::isfinite(s.creepSpeed) || s.creepSpeed<.2 || s.creepSpeed>4 ||
       !std::isfinite(s.lookAhead) || s.lookAhead<3 || s.lookAhead>30 ||
       !std::isfinite(s.noProgressTimeout) || s.noProgressTimeout<.5 || s.noProgressTimeout>10)
        throw std::invalid_argument("Invalid turn negotiation settings");
    turnNegotiation_=s; nextTurnCheck_=0;
    if(!s.enabled) { turnClaims_.clear(); for(auto& e:entities_) { e.yieldToVehicle=0; e.clearingTurn=false; e.turnRequestSince=-1; e.turnExitBlocked=false; e.exitWaitFor=0; } }
}
const Entity* Simulation::turnOwner(const TurnClaim& c) const {
    for(const auto& e:entities_) if(e.id==c.owner) {
        if(e.routeRevision!=c.revision || e.reversing || e.signalLight>=0 ||
           e.next>c.clearIndex || (e.next==c.clearIndex && e.segmentProgress>=c.clearProgress-1e-8)) return nullptr;
        return &e;
    }
    return nullptr;
}
bool Simulation::ownsNegotiatedTurn(const Entity& e) const {
    if(!turnNegotiation_.enabled) return false;
    for(const auto& c:turnClaims_) if(c.owner==e.id && turnOwner(c)) return true;
    return false;
}
std::uint64_t Simulation::maneuverYieldTo(const Entity& e) const {
    if(!turnNegotiation_.enabled || !e.yieldToVehicle) return 0;
    for(const auto& c:turnClaims_) if(c.owner==e.yieldToVehicle && turnOwner(c)) return c.owner;
    return 0;
}
bool Simulation::sameTurnStream(const Entity& e,const TurnClaim& c) const {
    const auto* owner=turnOwner(c); if(!owner) return false;
    if(!owner->laneRoute.empty() && e.laneRoute==owner->laneRoute) return true;
    Vec2 forward{std::cos(c.entryHeading),std::sin(c.entryHeading)},side{-forward.y,forward.x};
    if(dot(forward,{std::cos(e.heading),std::sin(e.heading)})<=.85 ||
       std::abs(dot(e.center-c.origin,side))>=.5*(e.width+owner->width)+std::min(e.preferredDistance,owner->preferredDistance)) return false;
    if(e.tugActive && length(e.tugVelocity)>.1 && dot(e.tugVelocity,forward)/length(e.tugVelocity)<.85) return false;
    if(e.pathHeadings.size()!=e.path.size()) return false;
    // Sharing the incoming lane is insufficient: a straight-through follower
    // must yield to the rear swing of a vehicle turning out of that lane.
    double outgoing=owner->pathHeadings[c.clearIndex];
    for(const auto& m:e.maneuvers) if(m.end>=e.next && m.end<e.path.size() &&
        length(e.path[m.begin]-c.origin)<2 && std::cos(angleDelta(e.pathHeadings[m.end],outgoing))>.95) return true;
    return false;
}
bool Simulation::insideTurn(const Footprint& f,const Entity& e,const TurnClaim& c) const {
    auto owner=turnOwner(c); if(!owner) return false;
    auto body=corners(f); double gap=std::min(e.preferredDistance,owner->preferredDistance);
    for(const auto& p:c.pieces) if(geometry::violatesClearance(body,p.polygon,gap+p.padding+.02)) return true;
    return false;
}
std::vector<Simulation::TurnPiece> Simulation::turnPieces(const Entity& e,std::size_t end,double limit) const {
    std::vector<TurnPiece> pieces; if(e.next>=e.path.size() || e.pathHeadings.size()!=e.path.size()) return pieces;
    auto from=e.footprint(); std::vector<Vec2> cloud=corners(from);
    double total=0,start=0,padding=0;
    for(std::size_t i=e.next;i<e.path.size() && i<=end && total<limit;++i) {
        Footprint initial=from,target{e.path[i],e.pathHeadings[i],e.length,e.width};
        double distance=length(target.center-initial.center),take=std::min(distance,limit-total);
        bool clipped=take<distance; int steps=std::max(1,static_cast<int>(std::ceil(take/.5)));
        for(int step=1;step<=steps;++step) {
            double fraction=distance>1e-10?(take/distance)*step/steps:1;
            auto to=initial; to.center=initial.center+(target.center-initial.center)*fraction;
            to.heading=initial.heading+angleDelta(initial.heading,target.heading)*fraction;
            double pad; auto sweep=geometry::swept(from,to,pad); padding=std::max(padding,pad);
            cloud.insert(cloud.end(),sweep.begin(),sweep.end()); total+=length(to.center-from.center); from=to;
            bool last=step==steps && (clipped || i==end || i+1==e.path.size());
            if(total-start>=.5-1e-9 || last) {
                pieces.push_back({geometry::convexHull(std::move(cloud)),padding,start});
                cloud=corners(from); padding=0; start=total;
            }
        }
        if(clipped) break;
    }
    return pieces;
}
bool Simulation::turnYieldBlocks(const Footprint& from,const Footprint& to,const Entity& e) const {
    if(!turnNegotiation_.enabled) return false;
    for(const auto& c:turnClaims_) {
        if(c.owner==e.id || !turnOwner(c)) continue;
        if(e.id==nextId_) {
            // A new spawn is not an existing occupant entitled to clear.
            if(insideTurn(from,e,c) || insideTurn(to,e,c)) return true;
            continue;
        }
        if(sameTurnStream(e,c) || insideTurn(from,e,c)) continue;
        if(insideTurn(to,e,c)) return true;
        double pad; auto sweep=geometry::swept(from,to,pad); auto owner=turnOwner(c);
        for(const auto& p:c.pieces) if(geometry::violatesClearance(sweep,p.polygon,std::min(e.preferredDistance,owner->preferredDistance)+pad+p.padding+.02)) return true;
    }
    return false;
}
double Simulation::negotiatedSpeed(const Entity& e,double requested) const {
    if(!turnNegotiation_.enabled) return requested;
    if(ownsNegotiatedTurn(e)) return std::min(e.desiredSpeed,turnNegotiation_.creepSpeed);
    if(!maneuverYieldTo(e)) return requested;
    auto pieces=turnPieces(e,e.path.size(),std::max(turnNegotiation_.lookAhead,e.speed*e.speed/(4*e.acceleration)+e.length*2));
    for(const auto& c:turnClaims_) if(c.owner==e.yieldToVehicle && !insideTurn(e.footprint(),e,c)) {
        auto owner=turnOwner(c); if(!owner) continue;
        for(const auto& a:pieces) for(const auto& b:c.pieces)
            if(geometry::violatesClearance(a.polygon,b.polygon,std::min(e.preferredDistance,owner->preferredDistance)+a.padding+b.padding+.02))
                return std::min(requested,std::sqrt(4*e.acceleration*std::max(0.0,a.distance-.15)));
    }
    return requested;
}
void Simulation::updateTurnNegotiation() {
    if(!turnNegotiation_.enabled) return;
    turnClaims_.erase(std::remove_if(turnClaims_.begin(),turnClaims_.end(),[&](TurnClaim& c) {
        auto owner=turnOwner(c); bool stale=!owner;
        if(owner) {
            if(length(owner->center-c.anchor)>.05) { c.anchor=owner->center; c.lastProgress=seconds_; }
            stale=seconds_-c.lastProgress>turnNegotiation_.noProgressTimeout || seconds_-c.created>45;
        }
        if(stale) for(auto& e:entities_) if(e.id==c.owner) { e.turnRequestSince=-1; e.turnRetryAfter=seconds_+1; }
        return stale;
    }),turnClaims_.end());
    if(seconds_<nextTurnCheck_) return; nextTurnCheck_=seconds_+.1;
    for(auto& e:entities_) { e.yieldToVehicle=0; e.clearingTurn=false; e.turnExitBlocked=false; e.exitWaitFor=0; }
    struct Request { std::size_t entity; TurnClaim claim; };
    std::vector<Request> requests;
    for(std::size_t k=0;k<entities_.size();++k) {
        auto& e=entities_[k];
        if(flow_.predictiveJunctions && !e.networkRoute.empty()) continue;
        if(e.reversing || e.cooperativeRecovery || e.tugActive || e.signalLight>=0 || e.next>=e.path.size() || e.pathHeadings.size()!=e.path.size() || ownsNegotiatedTurn(e) || seconds_<e.turnRetryAfter) continue;
        const PathManeuver* turn=nullptr;
        for(const auto& m:e.maneuvers) if(m.end>=e.next) { turn=&m; break; }
        if(!turn || turn->end>=e.path.size() || std::abs(angleDelta(e.pathHeadings[turn->begin],e.pathHeadings[turn->end]))<.35) { e.turnRequestSince=-1; continue; }
        double approach=0; Vec2 previous=e.center;
        for(std::size_t i=e.next;i<=turn->begin;++i) { approach+=length(e.path[i]-previous); previous=e.path[i]; }
        if(approach>std::max({3.0,e.length*1.1,e.speed*.7})) { e.turnRequestSince=-1; continue; }
        TurnClaim c; c.owner=e.id; c.revision=e.routeRevision; c.entryHeading=e.pathHeadings[turn->begin]; c.origin=e.path[turn->begin];
        c.anchor=e.center; c.lastProgress=c.created=seconds_;
        double distance=0; previous=e.center;
        for(std::size_t i=e.next;i<=turn->end;++i) { distance+=length(e.path[i]-previous); previous=e.path[i]; }
        double tail=e.length+e.preferredDistance;
        double arcDistance=distance;
        bool beforeArc=e.next<=turn->begin || (e.next==turn->begin+1 && e.segmentProgress<1e-8);
        c.clearIndex=turn->end; c.clearProgress=length(e.path[turn->end]-e.path[turn->end-1]);
        Footprint exit{e.path[turn->end],e.pathHeadings[turn->end],e.length,e.width};
        for(std::size_t i=turn->end+1;i<e.path.size() && tail>1e-8;++i) {
            double segment=length(e.path[i]-previous),take=std::min(segment,tail),fraction=segment>1e-10?take/segment:1;
            distance+=take; tail-=take; c.clearIndex=i; c.clearProgress=take;
            exit.center=previous+(e.path[i]-previous)*fraction;
            exit.heading=e.pathHeadings[i-1]+angleDelta(e.pathHeadings[i-1],e.pathHeadings[i])*fraction;
            previous=e.path[i];
        }
        c.pieces=turnPieces(e,c.clearIndex,distance);
        bool blocked=c.pieces.empty(),conflict=false;
        auto last=e.footprint();
        for(std::size_t i=e.next;i<=c.clearIndex && !blocked;++i) {
            Footprint to=i==c.clearIndex?exit:Footprint{e.path[i],e.pathHeadings[i],e.length,e.width};
            if(blockedByLight(last,to,e)) blocked=true;
            last=to;
        }
        for(const auto& other:entities_) if(other.id!=e.id && !blocked) {
            double gap=std::min(e.preferredDistance,other.preferredDistance);
            Vec2 exitForward{std::cos(exit.heading),std::sin(exit.heading)};
            bool queuedAhead=other.speed<=stoppedSpeedThreshold &&
                dot(exitForward,{std::cos(other.heading),std::sin(other.heading)})>.8 &&
                dot(other.center-e.path[turn->end],exitForward)>.25*(e.length+other.length);
            if(queuedAhead && geometry::violatesClearance(corners(exit),corners(other.footprint()),gap+.1)) {
                blocked=true; e.turnExitBlocked=beforeArc; e.blockedTurnEntry=turn->begin; e.exitWaitFor=other.id; break;
            }
            if(queuedAhead) for(const auto& p:c.pieces)
                if(p.distance>=arcDistance && geometry::violatesClearance(p.polygon,corners(other.footprint()),gap+p.padding+.1)) {
                    blocked=true; e.turnExitBlocked=beforeArc; e.blockedTurnEntry=turn->begin; e.exitWaitFor=other.id; break;
                }
            if(blocked) break;
            bool same=sameTurnStream(other,c);
            if(same) continue;
            auto otherPath=turnPieces(other,other.path.size(),turnNegotiation_.lookAhead);
            for(const auto& a:c.pieces) {
                if(!other.reservedTurn.empty() && (geometry::violatesClearance(a.polygon,other.reservedTurn,gap+a.padding+other.turnPadding) ||
                    geometry::violatesClearance(a.polygon,other.reservedApproach,gap+a.padding+other.turnPadding))) { blocked=true; break; }
                for(const auto& b:otherPath) if(geometry::violatesClearance(a.polygon,b.polygon,gap+a.padding+b.padding+.02)) { conflict=true; break; }
            }
        }
        if(e.turnExitBlocked) { e.reservedTurn.clear(); e.reservedApproach.clear(); }
        if(blocked || !conflict) { e.turnRequestSince=-1; continue; }
        if(e.turnRequestSince<0) e.turnRequestSince=seconds_;
        requests.push_back({k,std::move(c)});
    }
    std::stable_sort(requests.begin(),requests.end(),[&](const Request& a,const Request& b) {
        return entities_[a.entity].turnRequestSince<entities_[b.entity].turnRequestSince;
    });
    for(auto& request:requests) {
        auto& e=entities_[request.entity]; bool free=true;
        for(const auto& c:turnClaims_) {
            auto owner=turnOwner(c); if(!owner) continue;
            for(const auto& a:c.pieces) for(const auto& b:request.claim.pieces)
                if(geometry::violatesClearance(a.polygon,b.polygon,std::min(e.preferredDistance,owner->preferredDistance)+a.padding+b.padding+.02)) free=false;
        }
        if(!free) continue;
        e.reservedTurn.clear(); e.reservedApproach.clear();
        turnClaims_.push_back(std::move(request.claim)); ++negotiatedTurns_;
    }
    for(auto& e:entities_) if(!ownsNegotiatedTurn(e)) {
        auto pieces=turnPieces(e,e.path.size(),turnNegotiation_.lookAhead);
        for(const auto& c:turnClaims_) {
            auto owner=turnOwner(c); if(!owner || sameTurnStream(e,c)) continue;
            bool intersects=insideTurn(e.footprint(),e,c);
            for(const auto& a:pieces) for(const auto& b:c.pieces)
                if(geometry::violatesClearance(a.polygon,b.polygon,std::min(e.preferredDistance,owner->preferredDistance)+a.padding+b.padding+.02)) intersects=true;
            if(intersects) { e.yieldToVehicle=c.owner; e.clearingTurn=insideTurn(e.footprint(),e,c); break; }
        }
    }
    // A queue can extend beyond the geometric look-ahead. Propagate only a
    // live owner's yield through nearby bodies actually blocking its followers.
    // This prevents a legitimate rear queue from being mistaken for a free-road
    // standstill and starting a tug or an unnecessary go-around.
    for(std::size_t pass=0;pass<entities_.size();++pass) {
        bool changed=false;
        for(auto& e:entities_) if(!ownsNegotiatedTurn(e) && !maneuverYieldTo(e)) {
            auto probe=turnPieces(e,e.path.size(),.25);
            for(const auto& other:entities_) {
                auto owner=maneuverYieldTo(other);
                if(e.id==other.id || !owner || e.id==owner || other.speed>1) continue;
                double gap=std::min(e.preferredDistance,other.preferredDistance);
                if(length(e.center-other.center)>.5*(std::hypot(e.length,e.width)+std::hypot(other.length,other.width))+gap+.5) continue;
                bool blocked=false;
                for(const auto& p:probe) if(geometry::violatesClearance(p.polygon,corners(other.footprint()),gap+p.padding+.1)) { blocked=true; break; }
                if(!blocked) continue;
                e.yieldToVehicle=owner; e.clearingTurn=false; changed=true; break;
            }
        }
        if(!changed) break;
    }
}
}
