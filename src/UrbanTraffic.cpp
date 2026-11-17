#include "traffic/Simulation.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace traffic {
std::array<VehicleProfile,vehicleKindCount> vietnameseCityProfiles() {
    // Tunable demonstration assumptions, not a calibrated model of every city.
    return {{ {{1.8,.10},{.6,.04},{.35,.07},{4,.6,1.2,.2},3},
              {{2,.12},{.7,.05},{.35,.08},{7,1,2,.3},80},
              {{4.4,.3},{1.8,.08},{1,.15},{6,1,1.6,.25},14},
              {{8,.7},{2.4,.1},{1.4,.2},{4.5,.7,.9,.15},1},
              {{10,.7},{2.4,.1},{1.4,.2},{4.5,.7,.9,.15},2} }};
}
void Simulation::setUrbanTrafficSettings(UrbanTrafficSettings s) {
    for(double value:{s.motorbikeHeadway,s.otherHeadway,s.predictionSeconds,s.filterSpeed,s.lateralAcceleration})
        if(!std::isfinite(value)) throw std::invalid_argument("Urban settings must be finite");
    if(s.motorbikeHeadway<.2 || s.motorbikeHeadway>3 || s.otherHeadway<.2 || s.otherHeadway>3 ||
       s.predictionSeconds<.2 || s.predictionSeconds>3 || s.filterSpeed<.5 || s.filterSpeed>8 ||
       s.lateralAcceleration<.5 || s.lateralAcceleration>4)
        throw std::invalid_argument("Urban settings outside supported range");
    urbanTraffic_=s;
}
double Simulation::urbanSpeedLimit(const Entity& e,double requested,bool& brakingForBend,double* selectedBraking) const {
    brakingForBend=false;
    if(!urbanTraffic_.enabled) return requested;
    if(e.usingLocalDetour && (e.kind==VehicleKind::Motorbike || e.kind==VehicleKind::Bike))
        requested=std::min(requested,urbanTraffic_.filterSpeed);
    int source=!e.networkRoute.empty()?e.networkRoute.front():!e.laneRoute.empty()?e.laneRoute.front():-1;
    const auto* controls=source>=0 && lanes_[source].explicitTurns?&lanes_[source].turnControls:nullptr;
    double braking=laneTurnDeceleration(e);
    if(controls && !controls->empty()) for(const auto& control:*controls) braking=std::min(braking,control.braking);
    if(selectedBraking) *selectedBraking=braking;
    const double beforeBends=requested;
    double ahead=0; Vec2 previous=e.center;
    const double horizon=std::max(8.0,e.speed*e.speed/(2*braking)+e.length);
    for(std::size_t i=e.next;i<e.path.size() && i<e.pathHeadings.size() && ahead<horizon;++i) {
        double segment=length(e.path[i]-previous);
        double previousHeading=i==e.next?e.heading:e.pathHeadings[i-1];
        double curvature=std::abs(angleDelta(previousHeading,e.pathHeadings[i]))/std::max(.001,segment);
        if(curvature>1e-5) {
            double localBraking=braking;
            if(controls) {
                const TurnControl* marked=nullptr; double nearest=1e100;
                for(const auto& control:*controls) { double distance=length(e.path[i]-control.point); if(distance<=control.radius && distance<nearest) { marked=&control; nearest=distance; } }
                if(!marked) { ahead+=segment; previous=e.path[i]; continue; }
                localBraking=marked->braking;
            }
            double bendSpeed=std::sqrt(urbanTraffic_.lateralAcceleration/curvature);
            double target=std::sqrt(bendSpeed*bendSpeed+2*localBraking*ahead);
            if(target<requested) { requested=target; if(selectedBraking) *selectedBraking=localBraking; }
        }
        ahead+=segment; previous=e.path[i];
    }
    const double bendTarget=requested;
    brakingForBend=bendTarget<beforeBends-1e-8;
    Vec2 forward{std::cos(e.heading),std::sin(e.heading)};
    for(const auto& other:entities_) if(other.id!=e.id && other.speed>.1 && e.id>other.id) {
        Vec2 direction{std::cos(other.heading),std::sin(other.heading)};
        double alignment=forward.x*direction.x+forward.y*direction.y;
        if(std::abs(alignment)>.7) continue; // Following handles aligned traffic.
        double seconds=urbanTraffic_.predictionSeconds;
        if(length(other.center-e.center)>(e.speed+other.speed)*seconds+e.length+other.length) continue;
        Vec2 relative=direction*other.speed-forward*std::min(requested,e.speed+e.acceleration*seconds);
        Vec2 offset=other.center-e.center;
        double relativeSquared=relative.x*relative.x+relative.y*relative.y;
        if(relativeSquared<1e-8) continue;
        double closest=-(offset.x*relative.x+offset.y*relative.y)/relativeSquared;
        if(closest<=0 || closest>seconds) continue; // Diverging or beyond prediction horizon.
        double bound=.5*std::hypot(e.length,e.width)+.5*std::hypot(other.length,other.width)+std::min(e.preferredDistance,other.preferredDistance);
        if(length(offset+relative*closest)>bound) continue;
        auto last=e.footprint(),otherLast=other.footprint();
        for(int step=1;step<=6;++step) {
            double t=seconds*step/6;
            auto pose=e.footprint(),obstacle=other.footprint();
            // Predict near-term velocity, never teleport or reserve future bodies.
            pose.center=pose.center+forward*(std::min(requested,e.speed+e.acceleration*t)*t);
            obstacle.center=obstacle.center+direction*(other.speed*t);
            double p,q; auto a=geometry::swept(last,pose,p),b=geometry::swept(otherLast,obstacle,q);
            if(geometry::violatesClearance(a,b,std::min(e.preferredDistance,other.preferredDistance)+p+q)) {
                requested=std::min(requested,std::max(0.0,(length(pose.center-e.center)-.3)/(seconds+.2)));
                break;
            }
            last=pose; otherLast=obstacle;
        }
    }
    if(requested<bendTarget-1e-8) brakingForBend=false;
    return requested;
}
}
