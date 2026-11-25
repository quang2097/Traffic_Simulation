#include "traffic/Simulation.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace traffic {
namespace {
double dot(Vec2 a,Vec2 b) { return a.x*b.x+a.y*b.y; }
double front(const Footprint& box,Vec2 origin,Vec2 forward) {
    double result=-std::numeric_limits<double>::infinity();
    for(Vec2 p:corners(box)) result=std::max(result,dot(p-origin,forward));
    return result;
}
}
bool Simulation::spreadAtSignal(Entity& e) {
    // Native lanes and crossings own their committed path and sublane slots.
    // The legacy free-mask spreader can replace that path with an RTS detour.
    if(flow_.predictiveJunctions && !e.networkRoute.empty()) return false;
    if(e.signalLight==-2 || e.reversing) return false;
    if(e.signalLight>=0 && e.next<e.path.size()) return false;
    double nearest=0; Vec2 intersection;
    int selected=closestStoppingLight(e,&nearest,&intersection); e.stopTargetLight=selected;
    if(selected<0) return false;
    planningSignalLight_=selected;
    Vec2 line=lights_[selected].b-lights_[selected].a;
    Vec2 queueNormal{-line.y/length(line),line.x/length(line)};
    planningSignalForward_=queueNormal*(dot(e.center-lights_[selected].a,queueNormal)>0?-1.:1.);
    double horizon=std::max({routing_.lookAhead,e.length*3,8.0,e.speed*e.speed/(4*e.acceleration)+e.length});
    if(!planningSignalBatch_ && nearest>horizon) return false;
    if(followRouteToSignal(e,selected)) return true;
    if(!e.laneRoute.empty() || !routing_.spreadAtSignals) return planningSignalBatch_ && followRouteToSignal(e,selected,true);
    if(!e.reservedTurn.empty() && e.speed>.05) return false;
    const auto& light=lights_[selected];
    Vec2 tangent=(light.b-light.a)*(1/length(light.b-light.a));
    Vec2 normal{-tangent.y,tangent.x};
    Vec2 forward=normal*(dot(e.center-light.a,normal)>0?-1.0:1.0);
    Vec2 heading{std::cos(e.heading),std::sin(e.heading)};
    if(dot(heading,forward)<.25 || front(e.footprint(),light.a,forward)>-1e-8) return false;
    double lineLength=length(light.b-light.a),margin=e.width*.5+.12;
    if(lineLength<=margin*2) return false;
    double preferred=std::clamp(dot(intersection-light.a,tangent),margin,lineLength-margin);
    double spacing=std::max(.35,e.width*.5+.15);
    std::vector<double> positions{preferred};
    for(int i=1;i<=40;++i) {
        if(preferred+i*spacing<=lineLength-margin) positions.push_back(preferred+i*spacing);
        if(preferred-i*spacing>=margin) positions.push_back(preferred-i*spacing);
    }
    // Include exact edges of claimed slots. A narrow bike should be able to
    // use a fitting gap even when its center falls between the regular probes.
    for(const auto& other:entities_) if(other.id!=e.id && other.signalLight==selected && dot(other.signalForward,forward)>.5) {
        double halfWidth=0;
        for(Vec2 p:corners({other.signalTarget,other.signalStopHeading,other.length,other.width}))
            halfWidth=std::max(halfWidth,std::abs(dot(p-other.signalTarget,tangent)));
        double offset=halfWidth+e.width*.5+movementGap(e,other)+.06;
        double center=dot(other.signalTarget-light.a,tangent);
        for(double p:{center-offset,center+offset}) if(p>=margin && p<=lineLength-margin) positions.push_back(p);
    }
    std::sort(positions.begin(),positions.end(),[&](double a,double b) {
        double da=std::abs(a-preferred),db=std::abs(b-preferred);
        return da==db?a<b:da<db;
    });
    positions.erase(std::unique(positions.begin(),positions.end(),[](double a,double b) { return std::abs(a-b)<1e-6; }),positions.end());
    double stopHeading=std::atan2(forward.y,forward.x);
    // Front row first. If all usable front positions are occupied, fill rows
    // behind them, checking actual heterogeneous box sizes rather than lanes.
    int rows=planningSignalBatch_?std::min<int>(128,static_cast<int>(entities_.size())+1):3;
    for(int row=0;row<rows;++row) {
    if(row==1 && planningSignalBatch_ && followRouteToSignal(e,selected,true)) return true;
    for(double lateral:positions) {
        Vec2 target=light.a+tangent*lateral-forward*(e.length*.5+.15+row*(e.length+std::max(.25,carFollowing_.enabled?e.minGap:e.preferredDistance)+.3));
        double h=stopHeading;
        bool alreadyStopped=row==0 && e.speed<.1 && std::abs(lateral-preferred)<.01 &&
            front(e.footprint(),light.a,forward)>-.35;
        if(alreadyStopped) { target=e.center; h=e.heading; }
        if(e.signalLight>=0 && dot(target-e.center,forward)<.25) continue;
        if(dot(target-e.center,forward)<-.02) continue;
        Footprint slot{target,h,e.length,e.width};
        if(!map_.fits(slot) || front(slot,light.a,forward)>-1e-8) continue;
        bool available=true;
        for(const auto& other:entities_) if(other.id!=e.id) {
            double gap=movementGap(e,other)+.04;
            bool vacating=planningSignalBatch_ && other.signalLight==selected && dot(other.signalForward,forward)>.5 &&
                other.next<other.path.size() && length(other.signalTarget-other.center)>.25;
            if(!vacating && geometry::closerThan(corners(slot),corners(other.footprint()),gap)) { available=false; break; }
            if(other.signalLight>=0 &&
               geometry::closerThan(corners(slot),corners({other.signalTarget,other.signalStopHeading,other.length,other.width}),gap)) {
                available=false; break;
            }
        }
        if(!available) continue;
        MotionPath approach;
        approach.points.push_back(e.center); approach.headings.push_back(e.heading);
        double distance=length(target-e.center);
        if(distance>1e-7) {
            // The long final handle makes lateral movement happen early,
            // before reaching the rear bumper of a front-row vehicle.
            double along=std::max(.1,dot(target-e.center,forward));
            Vec2 c1=e.center+heading*std::min(along*.25,std::max(1.0,e.length));
            Vec2 c2=target-forward*std::min(along*.75,std::max(3.0,e.length*2));
            double controlLength=length(c1-e.center)+length(c2-c1)+length(target-c2);
            int samples=std::max(100,static_cast<int>(std::ceil(controlLength*3/.05)));
            Footprint from=e.footprint();
            std::vector<Vec2> envelope=corners(from); double maxPad=0;
            for(int i=1;i<=samples;++i) {
                double t=static_cast<double>(i)/samples,s=1-t;
                Vec2 point=e.center*(s*s*s)+c1*(3*s*s*t)+c2*(3*s*t*t)+target*(t*t*t);
                Vec2 derivative=(c1-e.center)*(3*s*s)+(c2-c1)*(6*s*t)+(target-c2)*(3*t*t);
                if(length(derivative)<1e-9) { available=false; break; }
                Footprint to{point,std::atan2(derivative.y,derivative.x),e.length,e.width};
                double pad; auto sweep=geometry::swept(from,to,pad);
                for(Vec2 p:sweep) if(dot(p-light.a,forward)+pad>=-1e-8) { available=false; break; }
                if(!available || !feasible(from,to,e,true) || blockedByLight(from,to,e)) { available=false; break; }
                approach.points.push_back(point); approach.headings.push_back(to.heading);
                auto box=corners(to); envelope.insert(envelope.end(),box.begin(),box.end());
                maxPad=std::max(maxPad,pad); from=to;
            }
            if(!available) continue;
            approach.turns.push_back({0,approach.points.size()-1,geometry::convexHull(std::move(envelope)),maxPad});
        }
        Entity exit=e; exit.center=target; exit.heading=h;
        Vec2 departure=target+forward*std::max(4.0,e.length*2+2);
        Footprint departureBox{departure,h,e.length,e.width};
        bool parallelExit=feasible(exit.footprint(),departureBox,exit,false);
        std::vector<Vec2> guide;
        if(parallelExit) {
            Entity after=exit; after.center=departure;
            auto tail=plan(after,0,false);
            if(!tail.empty()) { guide={target,departure}; guide.insert(guide.end(),tail.begin()+1,tail.end()); }
        }
        if(guide.empty()) guide=plan(exit,0,false);
        if(guide.empty()) continue;
        auto outgoing=curvePath(map_,exit.footprint(),guide);
        if(outgoing.points.empty()) continue;
        Entity candidate=e;
        candidate.path=std::move(approach.points); candidate.pathHeadings=std::move(approach.headings); candidate.maneuvers=std::move(approach.turns);
        candidate.next=1; candidate.segmentProgress=0; candidate.reservedTurn.clear(); candidate.reservedApproach.clear();
        candidate.usingLocalDetour=false; candidate.following=false; candidate.signalQueue=false; candidate.maneuverQueue=false;
        candidate.signalLight=selected; candidate.signalTarget=target; candidate.signalForward=forward;
        candidate.signalA=light.a; candidate.signalB=light.b; candidate.signalStopHeading=h;
        candidate.signalExit=std::move(outgoing); candidate.signalRemaining=0;
        for(std::size_t i=1;i<candidate.path.size();++i) candidate.signalRemaining+=length(candidate.path[i]-candidate.path[i-1]);
        if(!planningSignalBatch_ && !candidate.maneuvers.empty() && !reserveManeuver(candidate,candidate.maneuvers.front())) continue;
        candidate.progressAnchor=candidate.center; candidate.noProgressSeconds=0;
        e=std::move(candidate); ++signalPlacements_; return true;
    }
    }
    return planningSignalBatch_ && followRouteToSignal(e,selected,true);
}
bool Simulation::releaseSignalSlot(Entity& e) {
    bool installed=false;
    if(!e.signalExit.points.empty()) {
        if(e.next<e.path.size() && length(e.path.back()-e.signalExit.points.front())<1e-6) {
            std::size_t offset=e.path.size()-1;
            e.path.insert(e.path.end(),e.signalExit.points.begin()+1,e.signalExit.points.end());
            e.pathHeadings.insert(e.pathHeadings.end(),e.signalExit.headings.begin()+1,e.signalExit.headings.end());
            for(auto turn:e.signalExit.turns) { turn.begin+=offset; turn.end+=offset; e.maneuvers.push_back(std::move(turn)); }
        } else {
            e.path=std::move(e.signalExit.points); e.pathHeadings=std::move(e.signalExit.headings); e.maneuvers=std::move(e.signalExit.turns);
            e.next=1; e.segmentProgress=0; e.reservedTurn.clear(); e.reservedApproach.clear();
        }
        e.usingLocalDetour=false;
        installed=true;
    }
    if(!installed) {
        auto guide=plan(e,0,false);
        installed=!guide.empty() && installPath(e,std::move(guide));
    }
    if(!installed) return false;
    e.signalLight=-1; e.stopTargetLight=-1; e.signalExit={}; e.signalRemaining=0;
    e.waitingForLight=false; e.signalQueue=false; e.noProgressSeconds=0; e.progressAnchor=e.center;
    return true;
}
}
