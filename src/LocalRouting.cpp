#include "traffic/Simulation.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace traffic {
namespace {
double dot(Vec2 a,Vec2 b) { return a.x*b.x+a.y*b.y; }
Vec2 unit(Vec2 p) { double n=length(p); return n>1e-9?p*(1/n):Vec2{1,0}; }
double extent(const Entity& e,Vec2 axis) {
    double result=0;
    for(Vec2 p:corners(e.footprint())) result=std::max(result,std::abs(dot(p-e.center,axis)));
    return result;
}
}
double Simulation::followingSpeed(Entity& e) const {
    if(carFollowing_.enabled) return carFollowingSpeed(e);
    e.following=false;
    e.signalQueue=false; e.maneuverQueue=false;
    if(e.escapingGridlock) return e.desiredSpeed;
    if((!routing_.rtsStyle && e.signalLight<0 && e.laneRoute.empty()) || (e.usingLocalDetour && !urbanTraffic_.enabled)) return e.desiredSpeed;
    Vec2 forward{std::cos(e.heading),std::sin(e.heading)},side{-forward.y,forward.x};
    double target=e.desiredSpeed;
    for(const auto& other:entities_) if(other.id!=e.id) {
        if(e.signalLight>=0 && other.signalLight==e.signalLight && dot(e.signalForward,other.signalForward)>.5) {
            Vec2 lateral{-e.signalForward.y,e.signalForward.x};
            auto halfWidth=[&](const Entity& unit) {
                Vec2 direction{std::cos(unit.signalStopHeading),std::sin(unit.signalStopHeading)};
                return std::abs(dot(direction,lateral))*unit.length*.5+
                    std::abs(dot({-direction.y,direction.x},lateral))*unit.width*.5;
            };
            double gap=movementGap(e,other);
            if(std::abs(dot(other.signalTarget-e.signalTarget,lateral))>=halfWidth(e)+halfWidth(other)+gap) continue;
        }
        Vec2 delta=other.center-e.center;
        double along=dot(delta,forward);
        if(along<=0 || along>std::max(routing_.lookAhead,e.length*3)) continue;
        // An angled neighbor beside us is not a longitudinal leader. Without
        // this gate, two converging headings can both "follow" each other.
        Vec2 heading{std::cos(other.heading),std::sin(other.heading)};
        double alignment=dot(forward,heading);
        bool packedLane=flow_.sublanes && !e.networkRoute.empty();
        if(urbanTraffic_.enabled && along<.25*(e.length+other.length) && (!packedLane || alignment<.98)) continue;
        if(packedLane && alignment>.98 && along<.1 && e.id<other.id) continue;
        if(alignment<.6) continue;
        double required=movementGap(e,other);
        double steeringMargin=packedLane?.02:0;
        if(std::abs(dot(delta,side))>=e.width*.5+extent(other,side)+required+steeringMargin) continue;
        double gap=along-e.length*.5-extent(other,forward);
        double safe=std::max(0.0,other.speed+(gap-required)/1.25);
        if(urbanTraffic_.enabled) {
            double headway=(e.kind==VehicleKind::Bike || e.kind==VehicleKind::Motorbike)?urbanTraffic_.motorbikeHeadway:urbanTraffic_.otherHeadway;
            double braking=2*e.acceleration,bt=braking*headway;
            // A dynamic braking allowance above the unchanged hard minimum gap.
            safe=std::max(0.0,std::sqrt(bt*bt+other.speed*other.speed+2*braking*std::max(0.0,gap-required))-bt);
        }
        if(safe<target) {
            target=safe; e.following=true;
            e.signalQueue=other.waitingForLight || other.signalQueue;
            e.maneuverQueue=other.waitingForManeuver || other.reversing || other.maneuverQueue;
        }
    }
    return target;
}
bool Simulation::localDetour(Entity& e) {
    if(!routing_.rtsStyle || e.next>=e.path.size() || !e.reservedTurn.empty() || e.reversing || e.usingLocalDetour) return false;
    Vec2 forward=unit(e.path[e.next]-e.center),side{-forward.y,forward.x};
    // Keep a committed bend intact. Local patches are attached to the straight
    // corridor before it, then smoothed from the current vehicle heading.
    double horizon=std::min(std::max(routing_.lookAhead,e.length*2.5),length(e.path[e.next]-e.center));
    if(horizon<2) return false;
    auto ahead=e.footprint(); ahead.center=e.center+forward*horizon;
    if(blockedByLight(e.footprint(),ahead,e)) return false;
    double padding; auto corridor=geometry::swept(e.footprint(),ahead,padding);
    const Entity* blocker=nullptr; double nearest=std::numeric_limits<double>::infinity();
    for(const auto& other:entities_) if(other.id!=e.id) {
        double along=dot(other.center-e.center,forward);
        if(along<=.1 || along>horizon+other.length || along>=nearest) continue;
        double gap=movementGap(e,other);
        if(geometry::closerThan(corridor,corners(other.footprint()),gap+padding+1e-6)) { blocker=&other; nearest=along; }
    }
    bool filtering=urbanTraffic_.enabled && (e.kind==VehicleKind::Motorbike || e.kind==VehicleKind::Bike);
    if(!blocker || (blocker->waitingForLight && !filtering) || blocker->waitingForManeuver || blocker->reversing || !blocker->reservedTurn.empty()) return false;
    Vec2 blockerForward{std::cos(blocker->heading),std::sin(blocker->heading)};
    if(blocker->speed>=e.desiredSpeed*.7 && dot(blockerForward,forward)>.6) return false; // Follow a moving queue.
    if(dot(blockerForward,forward)>.6 && blocker->blockedSeconds<.5 && blocker->currentStopSeconds<.5) {
        double leaderForecast=std::min(blocker->desiredSpeed,blocker->speed+blocker->acceleration*1.5);
        double ownForecast=std::min(e.desiredSpeed,e.speed+e.acceleration*1.5);
        if(leaderForecast>=ownForecast*.85 || blocker->desiredSpeed>=e.desiredSpeed*.9) return false;
    }
    double halfAlong=extent(*blocker,forward),halfAcross=extent(*blocker,side);
    double required=movementGap(e,*blocker);
    double lead=std::max(e.length,2.0);
    double firstDistance=std::max(lead,nearest-halfAlong-lead);
    double passDistance=nearest+halfAlong+lead;
    double mergeDistance=passDistance+std::max(3.0,e.length*2);
    // Locate a continuous rejoin point, including the middle of a long segment.
    Vec2 previous=e.center,rejoin=e.goal,exitDirection=forward;
    std::size_t suffix=e.path.size(); double walked=0;
    for(std::size_t i=e.next;i<e.path.size();++i) {
        double segment=length(e.path[i]-previous);
        if(walked+segment>=mergeDistance && segment>1e-8) {
            exitDirection=unit(e.path[i]-previous);
            rejoin=previous+(e.path[i]-previous)*((mergeDistance-walked)/segment); suffix=i; break;
        }
        walked+=segment; previous=e.path[i];
    }
    if(dot(rejoin-e.center,forward)<passDistance+.5) return false;
    for(int sign:{e.detourSide,-e.detourSide}) for(double extra:{1.0,1.4}) {
        double offset=(halfAcross+e.width*.5+required+(filtering?.12:.35))*extra;
        // Offset relative to the obstacle center, not just the old path center,
        // so a truck already shifted sideways still leaves the requested gap.
        double lateral=dot(blocker->center-e.center,side)+sign*offset;
        Vec2 first=e.center+forward*firstDistance+side*lateral;
        Vec2 pass=e.center+forward*passDistance+side*lateral;
        std::vector<Vec2> guide{e.center,first,pass,rejoin};
        Vec2 last=rejoin; double lastHeading=std::atan2(exitDirection.y,exitDirection.x);
        for(std::size_t i=suffix;i<e.path.size();++i) {
            if(i+1==e.path.size() || length(e.path[i]-last)>3 || std::abs(angleDelta(lastHeading,e.pathHeadings[i]))>.2) {
                if(length(e.path[i]-last)>1e-6) guide.push_back(e.path[i]);
                last=e.path[i]; lastHeading=e.pathHeadings[i];
            }
        }
        if(length(guide.back()-e.goal)>1e-6) guide.push_back(e.goal);
        Entity candidate=e;
        if(!installPath(candidate,std::move(guide))) continue;
        // Validate the complete local maneuver now; farther route sections are
        // governed by normal movement admission when the vehicle reaches them.
        double localLength=length(first-e.center)+length(pass-first)+length(rejoin-pass)+e.length*2;
        double checked=0; bool clear=true;
        auto from=e.footprint();
        for(std::size_t i=1;i<candidate.path.size() && checked<localLength;++i) {
            Footprint to{candidate.path[i],candidate.pathHeadings[i],e.length,e.width};
            if(!feasible(from,to,e,true) || blockedByLight(from,to,e)) { clear=false; break; }
            checked+=length(to.center-from.center); from=to;
        }
        if(!clear) continue;
        if(!candidate.maneuvers.empty() && !reserveManeuver(candidate,candidate.maneuvers.front())) continue;
        // The cap is applied while filtering, not to the vehicle's sampled cruise speed.
        candidate.usingLocalDetour=true; candidate.detourSide=sign;
        candidate.detourMerge=rejoin; candidate.detourExitDirection=exitDirection;
        candidate.localSteerAfter=seconds_+routing_.localInterval;
        e=std::move(candidate); ++localDetours_; ++replans_; return true;
    }
    return false;
}
}
