#include "traffic/Simulation.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace traffic {
void Simulation::setGridlockEscapeSettings(GridlockEscapeSettings settings) {
    if(!std::isfinite(settings.afterSeconds) || settings.afterSeconds<1 || settings.afterSeconds>3600 ||
       !std::isfinite(settings.overlapAfterSeconds) || settings.overlapAfterSeconds<0 || settings.overlapAfterSeconds>3600 ||
       !std::isfinite(settings.speed) || settings.speed<.1 || settings.speed>5)
        throw std::invalid_argument("Invalid gridlock escape settings");
    gridlockEscape_=settings;
    // Retained API fields from the earlier overlap policy are ignored. This
    // policy only expires stale reservations; physical bodies always remain.
    for(auto& e:entities_) e.escapeOverlap=false;
}
void Simulation::updateGridlockEscapes() {
    if(recoveryPreview_) return;
    bool active=false;
    for(auto& e:entities_) if(e.escapingGridlock) {
        bool clear=std::all_of(entities_.begin(),entities_.end(),[&](const Entity& other) {
            return e.id==other.id || clearance(e.footprint(),other.footprint())+1e-6>=movementGap(e,other);
        });
        if(length(e.center-e.escapeAnchor)>=std::max(3.,e.length+1) && clear) {
            e.escapingGridlock=e.escapeOverlap=false; e.cooperativeRecovery=false;
            continue;
        }
        active=true;
        e.waitingForJunction=false; e.turnExitBlocked=false; e.yieldToVehicle=0;
        e.reservedTurn.clear(); e.reservedApproach.clear(); e.sublaneReservation.clear();
        e.junctionRetryAfter=seconds_;
    }
    if(active || !gridlockEscape_.enabled) return;
    Entity* selected=nullptr;
    for(auto& e:entities_) {
        if(e.currentStopSeconds+1e-9<gridlockEscape_.afterSeconds || e.desiredSpeed<=stoppedSpeedThreshold ||
           e.priorityYielding || e.reversing || e.waitingForLight || e.signalQueue || blockedByLight(e.footprint(),stopProbe(e),e)) continue;
        if(e.signalLight>=0 && e.signalLight<static_cast<int>(lights_.size()) && lightStops(lights_[e.signalLight],e)) continue;
        if(e.next>=e.path.size() && e.networkStage+1>=e.networkRoute.size()) continue;
        if(!selected || e.currentStopSeconds>selected->currentStopSeconds ||
           (e.currentStopSeconds==selected->currentStopSeconds && e.id<selected->id)) selected=&e;
    }
    if(!selected) return;
    auto& e=*selected; e.escapingGridlock=true; e.escapeOverlap=false;
    e.escapeStarted=seconds_; e.escapeAnchor=e.center; e.cooperativeRecovery=true;
    e.waitingForJunction=false; e.turnExitBlocked=false; e.yieldToVehicle=0;
    e.reservedTurn.clear(); e.reservedApproach.clear(); e.sublaneReservation.clear();
    e.junctionRetryAfter=seconds_;
    ++gridlockEscapes_;
    if(e.inOpenJunction) ++freeGridlockEscapes_;
    else if(e.internalJunction>=0) ++internalGridlockEscapes_;
    else ++laneGridlockEscapes_;
}
void Simulation::recordEscapeOverlaps() {
    std::vector<std::pair<std::uint64_t,std::uint64_t>> now;
    for(const auto& e:entities_) if(e.escapeOverlap) for(const auto& other:entities_) if(e.id!=other.id) {
        std::pair<std::uint64_t,std::uint64_t> pair{std::min(e.id,other.id),std::max(e.id,other.id)};
        if(std::find(now.begin(),now.end(),pair)!=now.end()) continue;
        if(!geometry::violatesClearance(corners(e.footprint()),corners(other.footprint()),0)) continue;
        now.emplace_back(pair.first,pair.second);
        if(std::find(previousEscapeOverlaps_.begin(),previousEscapeOverlaps_.end(),now.back())==previousEscapeOverlaps_.end()) ++escapeOverlapPairs_;
    }
    previousEscapeOverlaps_=std::move(now);
}
bool Simulation::laneChangeAllowed(int from,int to,Vec2 start,Vec2 end,double bodyLength) const {
    if(from<0 || to<0 || from>=static_cast<int>(lanes_.size()) || to>=static_cast<int>(lanes_.size())) return false;
    if(from==to) return true;
    auto station=[](const DirectedLane& lane,Vec2 point) {
        double best=1e100,result=0,distance=0;
        for(std::size_t i=1;i<lane.points.size();++i) {
            auto delta=lane.points[i]-lane.points[i-1]; double span=length(delta);
            double t=std::clamp(((point.x-lane.points[i-1].x)*delta.x+(point.y-lane.points[i-1].y)*delta.y)/std::max(1e-12,span*span),0.,1.);
            double gap=length(point-(lane.points[i-1]+delta*t));
            if(gap<best) {best=gap;result=distance+t*span;} distance+=span;
        }
        return result;
    };
    for(int id:{from,to}) {
        double a=station(lanes_[id],start),b=station(lanes_[id],end);
        double low=std::min(a,b)-bodyLength*.5,high=std::max(a,b)+bodyLength*.5;
        for(const auto& section:lanes_[id].changeSections)
            if(high>=section.begin-1e-8 && low<=section.end+1e-8 &&
                ((id==from && !section.allowExit) || (id==to && !section.allowEnter))) return false;
    }
    return true;
}
}
