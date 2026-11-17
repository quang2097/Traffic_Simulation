#include "traffic/Simulation.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace traffic {
namespace {
double dot(Vec2 a,Vec2 b) { return a.x*b.x+a.y*b.y; }
double cross(Vec2 a,Vec2 b) { return a.x*b.y-a.y*b.x; }
Vec2 unit(Vec2 v) { double n=length(v); return n>1e-10?v*(1/n):Vec2{1,0}; }
Vec2 capped(Vec2 v,double limit) { double n=length(v); return n>limit?v*(limit/n):v; }
double inertia(const Entity& e) { return e.massKg*(e.length*e.length+e.width*e.width)/12; }
Vec2 closest(Vec2 p,Vec2 a,Vec2 b) {
    auto d=b-a; return a+d*std::clamp(dot(p-a,d)/std::max(1e-12,dot(d,d)),0.0,1.0);
}
struct Contact { Vec2 point,normal; double distance=1e100; };
Contact contact(const Footprint& a,const Footprint& b) {
    auto ca=corners(a),cb=corners(b); Contact best; Vec2 points,normals; int count=0;
    auto consider=[&](Vec2 p,Vec2 q,double distance) {
        if(distance<best.distance-1e-7) { best.distance=distance; points={}; normals={}; count=0; }
        if(std::abs(distance-best.distance)>1e-7) return;
        points=points+p; normals=normals+(distance>1e-10?(p-q)*(1/distance):unit(a.center-b.center)); ++count;
    };
    for(int i=0;i<4;++i) for(int j=0;j<4;++j) {
        auto q=closest(ca[i],cb[j],cb[(j+1)%4]); double d=length(ca[i]-q);
        consider(ca[i],q,d);
        auto p=closest(cb[i],ca[j],ca[(j+1)%4]); d=length(p-cb[i]);
        consider(p,cb[i],d);
    }
    if(count) { best.point=points*(1.0/count); best.normal=unit(normals); }
    return best;
}
}
double defaultVehicleMass(VehicleKind kind) {
    constexpr double kg[]{100,200,1400,10000,12000};
    int i=static_cast<int>(kind); return i>=0 && i<vehicleKindCount?kg[i]:1400;
}
void Simulation::setTugSettings(TugSettings s) {
    for(double v:{s.afterStopSeconds,s.strengthNewtons,s.damping,s.maximumSpeed,s.lookAhead,s.arrivalRadius})
        if(!std::isfinite(v)) throw std::invalid_argument("Tug settings must be finite");
    if(s.afterStopSeconds<.1 || s.afterStopSeconds>10 || s.strengthNewtons<10 || s.strengthNewtons>20000 ||
       s.damping<.1 || s.damping>10 || s.maximumSpeed<.1 || s.maximumSpeed>10 || s.lookAhead<1 || s.lookAhead>15 ||
       s.arrivalRadius<.1 || s.arrivalRadius>2) throw std::invalid_argument("Invalid tug settings");
    auto reconnected=entities_;
    if(!s.enabled) for(auto& e:reconnected) if(e.tugActive) {
        // Blend back onto a trajectory anchored at the actual pose.
        std::vector<Vec2> guide{e.center};
        for(std::size_t i=e.next;i<e.path.size();++i) guide.push_back(e.path[i]);
        if(guide.size()<2) guide.push_back(e.goal);
        if(!installPath(e,std::move(guide),std::max(4.0,e.length*2)))
            throw std::runtime_error("Cannot disable tug until its current pose can reconnect to the road route");
        e.tugActive=false; e.angularVelocity=0;
        e.speed=std::max(0.0,dot(e.tugVelocity,Vec2{std::cos(e.heading),std::sin(e.heading)}));
    }
    if(!s.enabled) entities_=std::move(reconnected);
    tugSettings_=s;
}
bool Simulation::setEntityMass(std::uint64_t id,double kg) {
    if(!std::isfinite(kg) || kg<20 || kg>100000) throw std::invalid_argument("Mass must be 20-100000 kg");
    for(auto& e:entities_) if(e.id==id) { e.massKg=kg; return true; }
    return false;
}
void Simulation::activateTug(Entity& e) {
    if(!e.laneRoute.empty()) return;
    if(ownsNegotiatedTurn(e)) return;
    if(e.tugActive) return;
    e.tugActive=true; e.tugSeconds=0; e.tugStart=e.center;
    e.tugVelocity=Vec2{std::cos(e.heading),std::sin(e.heading)}*e.speed; e.angularVelocity=0;
    e.reservedTurn.clear(); e.reservedApproach.clear();
    if(e.path.size()<2) {
        e.path={e.center,e.goal}; e.pathHeadings={e.heading,std::atan2(e.goal.y-e.center.y,e.goal.x-e.center.x)};
        e.next=1; e.segmentProgress=0;
    }
}
bool Simulation::advanceTug(Entity& e,double dt) {
    if(e.reversing || ownsNegotiatedTurn(e)) return false;
    auto reason=stopReason(e);
    if(reason==StopReason::Signal || reason==StopReason::ManeuverYield || reason==StopReason::BlockedExit) {
        if(e.tugActive) { e.speed=0; e.tugVelocity={}; e.angularVelocity=0; return true; }
        return false;
    }
    if(!e.tugActive && !tugSettings_.alwaysOn &&
       (seconds_<e.tugSuppressedUntil || e.cooperativeRecovery || reason==StopReason::Vehicle || reason==StopReason::Road || reason==StopReason::Arrived)) return false;
    if(!e.tugActive && !tugSettings_.alwaysOn && e.currentStopSeconds<tugSettings_.afterStopSeconds) return false;
    activateTug(e); e.tugSeconds+=dt;
    if(!tugSettings_.alwaysOn && e.tugSeconds>1) {
        Entity candidate=e; std::vector<Vec2> guide{e.center};
        for(std::size_t i=e.next;i<e.path.size();++i) guide.push_back(e.path[i]);
        if(guide.size()<2) guide.push_back(e.goal);
        if(installPath(candidate,std::move(guide),std::max(4.0,e.length*2))) {
            e=std::move(candidate);
            e.speed=std::max(0.0,dot(e.tugVelocity,Vec2{std::cos(e.heading),std::sin(e.heading)}));
            e.tugActive=false; e.tugVelocity={}; e.angularVelocity=0;
            e.tugSuppressedUntil=seconds_+2; e.replanAfter=seconds_;
            return false;
        }
    }
    auto from=e.footprint(); Vec2 forward{std::cos(e.heading),std::sin(e.heading)};
    // Project onto nearby guide segments. Passing a waypoint advances guidance;
    // physical motion is never snapped to the projection or to a waypoint.
    double nearest=1e100,walked=0; std::size_t segment=e.next; double projected=0;
    Vec2 previous=e.center;
    for(std::size_t i=e.next;i<e.path.size() && walked<20;++i) {
        Vec2 start=i?e.path[i-1]:e.center,d=e.path[i]-start;
        double t=std::clamp(dot(e.center-start,d)/std::max(1e-12,dot(d,d)),0.0,1.0);
        double distance=length(e.center-(start+d*t));
        if(distance<nearest) { nearest=distance; segment=i; projected=t*length(d); }
        walked+=length(e.path[i]-previous); previous=e.path[i];
    }
    if(segment<e.path.size()) { e.next=segment; e.segmentProgress=projected; }
    Vec2 destination=e.goal;
    bool signal=e.signalLight>=0 && e.signalLight<static_cast<int>(lights_.size()) && lightStops(lights_[e.signalLight],e);
    if(signal) destination=e.signalTarget;
    double radius=signal?.12:tugSettings_.arrivalRadius;
    if(length(e.center-destination)<radius) {
        e.speed=0; e.tugVelocity={}; e.angularVelocity=0;
        e.next=e.path.size(); e.signalRemaining=0; return true;
    }
    Vec2 target=destination; previous=e.center; walked=0;
    double look=std::max(tugSettings_.lookAhead,e.length*.75)+e.speed*.3;
    for(std::size_t i=e.next;i<e.path.size();++i) {
        double distance=length(e.path[i]-previous);
        if(walked+distance>=look && distance>1e-10) { target=previous+(e.path[i]-previous)*((look-walked)/distance); break; }
        walked+=distance; previous=e.path[i];
    }
    Vec2 nose=e.center+forward*(e.length*.5);
    if(length(target-destination)<1e-8) {
        Vec2 approach=e.path.size()>1?unit(e.path.back()-e.path[e.path.size()-2]):unit(destination-e.center);
        target=target+approach*(e.length*.5);
    }
    const double strength=tugSettings_.strengthNewtons*std::sqrt(e.massKg/200.0);
    Vec2 delta=target-nose;
    Vec2 force=delta*(strength/std::sqrt(dot(delta,delta)+1));
    double torque=cross(nose-e.center,force);
    auto addForce=[&](Vec2 f,Vec2 point) { force=force+f; torque+=cross(point-e.center,f); };
    if(dot(unit(delta),forward)<-.5 && std::abs(cross(unit(delta),forward))<.1)
        addForce(Vec2{-forward.y,forward.x}*(strength*.15*e.detourSide),nose);
    for(const auto& other:entities_) if(other.id!=e.id) {
        double gap=std::min(e.preferredDistance,other.preferredDistance),range=gap+.8;
        if(length(e.center-other.center)>.5*std::hypot(e.length,e.width)+.5*std::hypot(other.length,other.width)+range) continue;
        auto c=contact(from,other.footprint()); if(c.distance>=range) continue;
        double pressure=std::clamp((range-c.distance)/.8,0.0,2.0);
        double dominance=std::clamp(std::sqrt(other.massKg/e.massKg),.5,2.0);
        addForce(c.normal*(strength*pressure*1.5*dominance),c.point);
        if(dot(other.center-e.center,forward)>0)
            addForce(Vec2{-forward.y,forward.x}*(strength*.45*pressure*e.detourSide),nose);
    }
    for(int i=0;i<8;++i) {
        double a=i*3.141592653589793/4; Vec2 direction{std::cos(a),std::sin(a)};
        auto probe=from; probe.center=probe.center+direction*.4;
        if(!map_.fits(probe)) addForce(direction*(-strength*.65),nose);
    }
    Vec2 acceleration=capped(force*(1/e.massKg),2*e.acceleration);
    e.tugVelocity=(e.tugVelocity+acceleration*dt)*(1/(1+tugSettings_.damping*dt));
    Vec2 lateral{-forward.y,forward.x};
    e.tugVelocity=e.tugVelocity-lateral*(dot(e.tugVelocity,lateral)*std::min(1.0,dt*2));
    double limit=negotiatedSpeed(e,std::min(tugSettings_.maximumSpeed,e.desiredSpeed));
    e.tugVelocity=capped(e.tugVelocity,limit);
    e.angularVelocity=std::clamp((e.angularVelocity+std::clamp(torque/inertia(e),-2.0,2.0)*dt)/(1+3*dt),-1.0,1.0);
    auto pose=[&](double fraction) { auto to=from; to.center=to.center+e.tugVelocity*(dt*fraction); to.heading+=e.angularVelocity*dt*fraction; return to; };
    auto trial=pose(1);
    for(auto& other:entities_) if(other.id!=e.id &&
        !geometry::motionClearance(from,trial,other.footprint(),std::min(e.preferredDistance,other.preferredDistance),true)) {
        auto c=contact(from,other.footprint()); Vec2 r=c.point-e.center,s=c.point-other.center;
        auto otherReason=stopReason(other);
        bool anchored=otherReason==StopReason::Signal || otherReason==StopReason::ManeuverYield || otherReason==StopReason::BlockedExit;
        Vec2 ov=other.tugActive?other.tugVelocity:Vec2{std::cos(other.heading),std::sin(other.heading)}*other.speed;
        Vec2 relative=e.tugVelocity+Vec2{-r.y,r.x}*e.angularVelocity-ov-Vec2{-s.y,s.x}*other.angularVelocity;
        double incoming=dot(relative,c.normal);
        if(incoming>=0) continue;
        bool freeBody=other.laneRoute.empty() && !ownsNegotiatedTurn(other) && (other.tugActive || tugSettings_.alwaysOn);
        Vec2 otherForward{std::cos(other.heading),std::sin(other.heading)};
        double constrained=dot(c.normal,otherForward);
        double invOther=anchored?0:(freeBody?1:constrained*constrained)/other.massKg;
        double rn=cross(r,c.normal),sn=cross(s,c.normal);
        double impulse=-incoming/(1/e.massKg+invOther+rn*rn/inertia(e)+(anchored || !freeBody?0:sn*sn/inertia(other)));
        e.tugVelocity=e.tugVelocity+c.normal*(impulse/e.massKg); e.angularVelocity+=rn*impulse/inertia(e);
        if(!anchored && freeBody) {
            activateTug(other);
            other.tugVelocity=capped(other.tugVelocity-c.normal*(impulse/other.massKg),std::min(tugSettings_.maximumSpeed,other.desiredSpeed));
            other.angularVelocity=std::clamp(other.angularVelocity-sn*impulse/inertia(other),-1.0,1.0);
        } else if(!anchored) {
            // Route-guided tires constrain lateral motion and yaw; longitudinal
            // momentum still responds to the finite effective mass.
            other.speed=std::clamp(other.speed-constrained*impulse/other.massKg,0.0,other.desiredSpeed);
        }
        ++tugContacts_;
    }
    e.tugVelocity=capped(e.tugVelocity,limit); e.angularVelocity=std::clamp(e.angularVelocity,-1.0,1.0);
    Entity mover; mover.id=e.id; mover.length=e.length; mover.width=e.width;
    mover.center=e.center; mover.heading=e.heading;
    mover.tugActive=e.tugActive; mover.tugVelocity=e.tugVelocity;
    mover.preferredDistance=e.preferredDistance; mover.cooperativeRecovery=true;
    auto permitted=[&](const Footprint& to) {
        if(blockedByLight(from,to,e) || !feasible(from,to,mover,true,false)) return false;
        double pad; auto sweep=geometry::swept(from,to,pad);
        for(const auto& light:lights_) if(lightStops(light,e) && !geometry::intersectsLine(corners(from),light.a,light.b) &&
            geometry::distance(sweep,{light.a,light.b})<.1+pad &&
            geometry::distance(corners(to),{light.a,light.b})<=geometry::distance(corners(from),{light.a,light.b})+1e-10) return false;
        return true;
    };
    bool moved=false;
    for(double fraction:{1.0,.5,.25,.125}) {
        auto to=pose(fraction);
        if(!permitted(to)) continue;
        e.center=to.center; e.heading=to.heading;
        moved=length(e.center-from.center)>1e-10 || std::abs(angleDelta(from.heading,e.heading))>1e-10;
        e.tugVelocity=e.tugVelocity*fraction; e.angularVelocity*=fraction; break;
    }
    if(!moved) { e.tugVelocity=e.tugVelocity*.5; e.angularVelocity*=.5; }
    e.speed=moved?length(e.tugVelocity):0; e.waitingForManeuver=false;
    e.blockedSeconds=moved?0:e.blockedSeconds+dt;
    if(!tugSettings_.alwaysOn && e.tugSeconds>1.5 && length(e.center-e.tugStart)>2 &&
       std::abs(e.angularVelocity)<.25 && dot(unit(e.tugVelocity),Vec2{std::cos(e.heading),std::sin(e.heading)})>.95) {
        std::vector<Vec2> guide{e.center};
        for(std::size_t i=e.next;i<e.path.size();++i) guide.push_back(e.path[i]);
        if(guide.size()<2) guide.push_back(e.goal);
        if(installPath(e,std::move(guide),std::max(4.0,e.length*2))) { e.tugActive=false; e.angularVelocity=0; }
    }
    return true;
}
}
