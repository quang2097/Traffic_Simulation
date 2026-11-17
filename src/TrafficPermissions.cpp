#include "traffic/Simulation.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace traffic {
namespace {
double cross(Vec2 a,Vec2 b) { return a.x*b.y-a.y*b.x; }
bool crossing(Vec2 from,Vec2 to,Vec2 a,Vec2 b,double& fraction) {
    Vec2 d=to-from,line=b-a; double divisor=cross(d,line);
    if(std::abs(divisor)<1e-10) return false;
    fraction=cross(a-from,line)/divisor; double along=cross(a-from,d)/divisor;
    return fraction>=-1e-9 && fraction<=1+1e-9 && along>=-1e-9 && along<=1+1e-9;
}
}
bool Simulation::routePermitsRed(const Entity& e,const TrafficLight& light) const {
    if(!light.permissive) return false;
    bool passedStop=false,allowed=false; Vec2 previous=e.center;
    auto segment=[&](Vec2 next) {
        double stop=0,exit=0;
        bool stopCross=crossing(previous,next,light.a,light.b,stop);
        bool exitCross=crossing(previous,next,light.allowedA,light.allowedB,exit);
        if(exitCross && (passedStop || (stopCross && exit>stop+1e-8))) allowed=true;
        passedStop=passedStop || stopCross; previous=next;
    };
    for(std::size_t i=e.next;i<e.path.size() && !allowed;++i) segment(e.path[i]);
    // Queue slots are temporary goals. Their cached departure is the vehicle's
    // preferred route beyond the stop, and must participate in the decision.
    for(std::size_t i=0;i<e.signalExit.points.size() && !allowed;++i) segment(e.signalExit.points[i]);
    // Lane phases are temporary goals too. Follow the committed trip, including
    // its precomputed free crossing and return lane, before deciding permission.
    for(std::size_t stage=e.networkStage+1;stage<e.networkRoute.size() && !allowed;++stage) {
        const auto& lane=lanes_[e.networkRoute[stage]];
        const auto& points=stage<e.junctionPlans.size() && !e.junctionPlans[stage].points.empty()
            ?e.junctionPlans[stage].points:lane.points;
        // A crossing plan includes its merge into the following lane. Trim
        // that lane's consumed prefix rather than drawing a backwards segment.
        std::size_t begin=0;
        if(lane.openJunction<0 && !points.empty()) {
            double nearest=1e100;
            for(std::size_t i=1;i<points.size();++i) {
                auto d=points[i]-points[i-1]; double n=d.x*d.x+d.y*d.y;
                double t=n>0?std::clamp(((previous-points[i-1]).x*d.x+(previous-points[i-1]).y*d.y)/n,0.,1.):0;
                double distance=length(points[i-1]+d*t-previous);
                if(distance<nearest) { nearest=distance; begin=i; }
            }
        }
        for(std::size_t i=begin;i<points.size() && !allowed;++i) segment(points[i]);
    }
    return allowed;
}
bool Simulation::lightStops(const TrafficLight& light,const Entity& e) const {
    return light.phase(seconds_)!=LightPhase::Green && !routePermitsRed(e,light);
}
std::array<double,vehicleKindCount> Simulation::spawnPercentages() const {
    std::array<double,vehicleKindCount> result{}; double total=0;
    for(const auto& profile:profiles_) total+=profile.weight;
    if(total>0) for(int i=0;i<vehicleKindCount;++i) result[i]=profiles_[i].weight*100/total;
    return result;
}
void Simulation::setSpawnPercentages(const std::array<double,vehicleKindCount>& percentages) {
    double total=0;
    for(double value:percentages) {
        if(!std::isfinite(value) || value<0 || value>100) throw std::invalid_argument("Spawn percentages must be between 0 and 100");
        total+=value;
    }
    if(std::abs(total-100)>.01) throw std::invalid_argument("Spawn percentages must total 100%");
    for(int i=0;i<vehicleKindCount;++i) profiles_[i].weight=percentages[i]*100/total;
}
}
