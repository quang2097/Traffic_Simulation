#include "traffic/Simulation.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace traffic {
namespace {
double cross(Vec2 a,Vec2 b) { return a.x*b.y-a.y*b.x; }
double dot(Vec2 a,Vec2 b) { return a.x*b.x+a.y*b.y; }
double front(const Footprint& box,Vec2 origin,Vec2 forward) {
    double value=-std::numeric_limits<double>::infinity();
    for(Vec2 p:corners(box)) value=std::max(value,dot(p-origin,forward));
    return value;
}
std::vector<Footprint> remainingRoute(const Entity& e) {
    std::vector<Footprint> result{e.footprint()};
    for(std::size_t i=e.next;i<e.path.size();++i) result.push_back({e.path[i],e.pathHeadings[i],e.length,e.width});
    for(std::size_t i=0;i<e.signalExit.points.size();++i)
        if(length(e.signalExit.points[i]-result.back().center)>1e-8)
            result.push_back({e.signalExit.points[i],e.signalExit.headings[i],e.length,e.width});
    return result;
}
void markCurves(MotionPath& path,double bodyLength,double bodyWidth) {
    for(std::size_t i=1;i<path.points.size();) {
        if(std::abs(angleDelta(path.headings[i-1],path.headings[i]))<1e-7) { ++i; continue; }
        std::size_t begin=i-1;
        while(i+1<path.points.size() && std::abs(angleDelta(path.headings[i],path.headings[i+1]))>=1e-7) ++i;
        PathManeuver turn; turn.begin=begin; turn.end=i;
        std::vector<Vec2> envelope;
        for(std::size_t j=begin;j<=i;++j) {
            auto box=corners({path.points[j],path.headings[j],bodyLength,bodyWidth});
            envelope.insert(envelope.end(),box.begin(),box.end());
            if(j>begin) turn.padding=std::max(turn.padding,.5*std::hypot(bodyLength,bodyWidth)*(1-std::cos(angleDelta(path.headings[j-1],path.headings[j])*.5)));
        }
        turn.sweep=geometry::convexHull(std::move(envelope)); path.turns.push_back(std::move(turn)); ++i;
    }
}
}
int Simulation::closestStoppingLight(const Entity& e,double* routeDistance,Vec2* crossingPoint) const {
    if(std::none_of(lights_.begin(),lights_.end(),[&](const TrafficLight& light) { return light.phase(seconds_)!=LightPhase::Green; })) {
        if(routeDistance) *routeDistance=std::numeric_limits<double>::infinity();
        return -1;
    }
    int selected=-1; double nearest=std::numeric_limits<double>::infinity(); Vec2 point;
    auto route=remainingRoute(e);
    // Look through the lane's temporary endpoint to its committed crossing,
    // so native vehicles brake before the light without acquiring RTS slots.
    if(flow_.predictiveJunctions && !e.networkRoute.empty() && !e.inOpenJunction &&
       e.networkStage+1<e.junctionPlans.size()) {
        const auto& next=e.junctionPlans[e.networkStage+1];
        for(std::size_t i=0;i<next.points.size();++i) if(length(next.points[i]-route.back().center)>1e-8)
            route.push_back({next.points[i],next.headings[i],e.length,e.width});
    }
    for(std::size_t k=0;k<lights_.size();++k) {
        const auto& light=lights_[k];
        if(light.phase(seconds_)==LightPhase::Green || routePermitsRed(e,light) || geometry::intersectsLine(corners(e.footprint()),light.a,light.b)) continue;
        double walked=0; Vec2 line=light.b-light.a;
        for(std::size_t i=1;i<route.size();++i) {
            Vec2 start=route[i-1].center,delta=route[i].center-start;
            double divisor=cross(delta,line),segment=length(delta);
            if(std::abs(divisor)>1e-10) {
                double t=cross(light.a-start,line)/divisor,u=cross(light.a-start,delta)/divisor;
                if(t>=0 && t<=1 && u>=0 && u<=1 && light.controlsDirection(delta)) {
                    if(walked+segment*t<nearest) { nearest=walked+segment*t; selected=static_cast<int>(k); point=start+delta*t; }
                    break;
                }
            }
            walked+=segment;
        }
    }
    if(routeDistance) *routeDistance=nearest;
    if(crossingPoint && selected>=0) *crossingPoint=point;
    return selected;
}
bool Simulation::followRouteToSignal(Entity& e,int selected,bool queueBehind) {
    const auto& light=lights_[selected]; auto route=remainingRoute(e);
    Vec2 tangent=(light.b-light.a)*(1/length(light.b-light.a)),normal{-tangent.y,tangent.x};
    Vec2 forward=normal*(dot(e.center-light.a,normal)>0?-1.:1.);
    MotionPath approach,exit;
    approach.points={e.center}; approach.headings={e.heading};
    bool stopped=false; Footprint target; std::size_t stopSegment=0;
    for(std::size_t i=1;i<route.size();++i) {
        auto from=route[i-1],to=route[i];
        if(front(from,light.a,forward)<-.15 && front(to,light.a,forward)>=-.15) {
            double low=0,high=1;
            for(int n=0;n<40;++n) {
                double t=(low+high)*.5; auto pose=from;
                pose.center=from.center+(to.center-from.center)*t;
                pose.heading=from.heading+angleDelta(from.heading,to.heading)*t;
                if(front(pose,light.a,forward)<-.15) low=t; else high=t;
            }
            target=from; target.center=from.center+(to.center-from.center)*low;
            target.heading=from.heading+angleDelta(from.heading,to.heading)*low;
            if(!feasible(from,target,e,true)) return false;
            double along=dot(target.center-light.a,tangent);
            if(along>=0 && along<=length(light.b-light.a)) { stopped=true; stopSegment=i; }
        }
        if(stopped) to=target;
        if(!map_.fits(to) || blockedByLight(from,to,e)) return false;
        if(!feasible(from,to,e,true)) {
            if(!queueBehind || !planningSignalBatch_) return false;
            double pad; auto sweep=geometry::swept(from,to,pad); bool plannedQueue=false;
            for(const auto& other:entities_) if(other.id!=e.id && other.signalLight>=0) {
                double gap=movementGap(e,other)+pad+.04;
                if(geometry::closerThan(sweep,corners({other.signalTarget,other.signalStopHeading,other.length,other.width}),gap)) { plannedQueue=true; break; }
            }
            if(!plannedQueue) return false;
            double low=0,high=1;
            for(int n=0;n<40;++n) {
                double t=(low+high)*.5; auto probe=from;
                probe.center=from.center+(to.center-from.center)*t;
                probe.heading=from.heading+angleDelta(from.heading,to.heading)*t;
                if(feasible(from,probe,e,true)) low=t; else high=t;
            }
            target=from; target.center=from.center+(to.center-from.center)*low;
            target.heading=from.heading+angleDelta(from.heading,to.heading)*low;
            to=target; stopped=true; stopSegment=i;
        }
        approach.points.push_back(to.center); approach.headings.push_back(to.heading);
        if(stopped) break;
    }
    if(!stopped || (e.signalLight>=0 && dot(target.center-e.center,forward)<.25)) return false;
    exit.points={target.center}; exit.headings={target.heading};
    for(std::size_t i=stopSegment;i<route.size();++i) if(length(route[i].center-exit.points.back())>1e-8) {
        exit.points.push_back(route[i].center); exit.headings.push_back(route[i].heading);
    }
    markCurves(approach,e.length,e.width); markCurves(exit,e.length,e.width);
    Entity candidate=e;
    candidate.path=std::move(approach.points); candidate.pathHeadings=std::move(approach.headings); candidate.maneuvers=std::move(approach.turns);
    candidate.next=1; candidate.segmentProgress=0; candidate.reservedTurn.clear(); candidate.reservedApproach.clear(); candidate.reservationEnd=0;
    candidate.usingLocalDetour=false; candidate.following=false; candidate.signalQueue=false; candidate.maneuverQueue=false;
    candidate.signalLight=selected; candidate.stopTargetLight=selected; candidate.signalTarget=target.center; candidate.signalStopHeading=target.heading;
    candidate.signalForward=forward; candidate.signalA=light.a; candidate.signalB=light.b; candidate.signalExit=std::move(exit); candidate.signalRemaining=0;
    for(std::size_t i=1;i<candidate.path.size();++i) candidate.signalRemaining+=length(candidate.path[i]-candidate.path[i-1]);
    if(!planningSignalBatch_ && !candidate.maneuvers.empty() && !reserveManeuver(candidate,candidate.maneuvers.front())) return false;
    candidate.progressAnchor=candidate.center; candidate.noProgressSeconds=0;
    e=std::move(candidate); ++signalPlacements_; return true;
}
}
