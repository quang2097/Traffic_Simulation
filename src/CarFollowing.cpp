#include "traffic/Simulation.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace traffic {
double Simulation::brakingDistance(double speed,double deceleration,double step) {
    // Sum the semi-implicit Euler braking trajectory, after each speed update.
    double reduction=deceleration*step;
    if(speed<=0) return 0;
    double n=std::floor(speed/reduction);
    return std::max(0.,step*(n*speed-reduction*n*(n+1)*.5));
}
double Simulation::safeFollowingSpeed(double gap,double leaderSpeed,double leaderDeceleration,double deceleration,double tau,double step) {
    // Independent inversion of SUMO's discrete stopping-distance inequality:
    // v*tau + brakeDistance(v,b) <= gap + brakeDistance(vLeader,max(b,bLeader)).
    // gap is already bumper-to-bumper distance minus the follower's minGap.
    double available=gap+brakingDistance(leaderSpeed,std::max(deceleration,leaderDeceleration),step);
    if(available<=0) return 0;
    double low=0,high=std::sqrt(2*deceleration*available)+deceleration*step+available/std::max(tau,step);
    for(int i=0;i<38;++i) {
        double mid=(low+high)*.5;
        if(mid*tau+brakingDistance(mid,deceleration,step)<=available-1e-9) low=mid; else high=mid;
    }
    return low;
}
void Simulation::prepareDrivingSnapshot(double dt) {
    drivingStep_=dt;drivingSnapshot_.clear();
    if(!carFollowing_.enabled) return;
    drivingSnapshot_.reserve(entities_.size());
    for(const auto& e:entities_) {
        DrivingObservation observation{e.id,e.footprint(),e.speed,e.deceleration,e.waitingForLight || e.signalQueue,e.waitingForManeuver || e.maneuverQueue};
        if(e.networkStage<e.networkRoute.size())observation.lane=e.networkRoute[e.networkStage];
        if(e.onJunction() && e.networkStage+1<e.networkRoute.size())observation.exit=e.networkRoute[e.networkStage+1];
        if(e.onJunction())observation.junction=e.activeJunction;
        drivingSnapshot_.push_back(observation);
        if(e.changingLanes && e.laneChangeOrigin){
            auto claim=observation;claim.body=e.laneChangeOrigin->footprint();claim.speed=0;
            claim.signal=claim.maneuver=false;
            if(e.laneChangeOrigin->networkStage<e.laneChangeOrigin->networkRoute.size())claim.lane=e.laneChangeOrigin->networkRoute[e.laneChangeOrigin->networkStage];
            drivingSnapshot_.push_back(claim);
        }
    }
}
void Simulation::prepareApproachSpeeds() {
    if(!carFollowing_.enabled || !flow_.predictiveJunctions) return;
    for(auto& e:entities_) {
        if(e.onJunction() || e.networkStage+1>=e.networkRoute.size()) {e.approachBlocked=false;continue;}
        int link=e.networkRoute[e.networkStage+1];
        if(lanes_[link].openJunction<0 && lanes_[link].internalJunction<0) {e.approachBlocked=false;continue;}
        double remaining=0;Vec2 previous=e.center;
        for(std::size_t i=e.next;i<e.path.size();++i){remaining+=length(e.path[i]-previous);previous=e.path[i];}
        if(remaining>std::max(10.,e.speed*e.speed/(2*e.deceleration)+e.speed*.5+e.length)) {e.approachBlocked=false;continue;}
        if(seconds_<e.approachCheckAfter)continue;
        e.approachCheckAfter=seconds_+.2;
        Entity atStop=e;
        if(e.path.empty() || e.pathHeadings.empty())continue;
        atStop.center=e.path.back();atStop.heading=e.pathHeadings.back();atStop.next=atStop.path.size();
        Entity crossing=atStop;
        bool feasible=installNetworkStage(crossing,e.networkStage+1);
        if(feasible)atStop.admissionPlan={crossing.path,crossing.pathHeadings,crossing.maneuvers};
        e.approachBlocked=!feasible || !junctionAdmission(atStop,link,crossing.path);
    }
}
double Simulation::carFollowingSpeed(Entity& e) const {
    e.following=false;e.signalQueue=false;e.maneuverQueue=false;e.gapCompressed=false;
    double cap=e.desiredSpeed;
    if(e.escapeOverlap) return std::min(cap,gridlockEscape_.speed);
    if(e.approachBlocked && !e.onJunction()) {
        double remaining=0;Vec2 previous=e.center;
        for(std::size_t i=e.next;i<e.path.size();++i){remaining+=length(e.path[i]-previous);previous=e.path[i];}
        cap=std::min(cap,safeFollowingSpeed(remaining,0,e.deceleration,e.deceleration,drivingStep_,drivingStep_));
    }
    auto consider=[&](const DrivingObservation& other) {
        if(other.id==e.id) return;
        bool routeLeader=true;
        if(!e.networkRoute.empty() && other.lane>=0) {
            routeLeader=false;
            for(std::size_t stage=e.networkStage;stage<std::min(e.networkRoute.size(),e.networkStage+3);++stage)if(e.networkRoute[stage]==other.lane)routeLeader=true;
            if(e.onJunction() && other.junction==e.activeJunction && e.networkStage+1<e.networkRoute.size() && other.exit==e.networkRoute[e.networkStage+1])routeLeader=true;
        }
        double horizon=std::max(15.,e.speed*e.tau+brakingDistance(e.speed,e.deceleration,drivingStep_)+e.length+other.body.length);
        if(length(other.body.center-e.center)>horizon+other.body.width) return;
        Vec2 previous=e.center;double travelled=0,best=std::numeric_limits<double>::infinity(),leaderSpeed=0;
        for(std::size_t i=e.next;i<e.path.size() && travelled<horizon;++i) {
            Vec2 delta=e.path[i]-previous;double span=length(delta);
            if(span<1e-9){previous=e.path[i];continue;}
            Vec2 forward=delta*(1/span),side{-forward.y,forward.x},offset=other.body.center-previous;
            double along=offset.x*forward.x+offset.y*forward.y;
            Vec2 otherForward{std::cos(other.body.heading),std::sin(other.body.heading)};
            double alignment=forward.x*otherForward.x+forward.y*otherForward.y;
            double lengthExtent=std::abs(alignment)*other.body.length*.5+std::abs(forward.x*(-otherForward.y)+forward.y*otherForward.x)*other.body.width*.5;
            double widthExtent=std::abs(side.x*otherForward.x+side.y*otherForward.y)*other.body.length*.5+std::abs(alignment)*other.body.width*.5;
            if(along>=-lengthExtent && along<=span+lengthExtent && travelled+along>.02 &&
                std::abs(offset.x*side.x+offset.y*side.y)<e.width*.5+widthExtent+.005) {
                // Crossing traffic is handled by junction admission. A stopped
                // body actually on this path is still a stationary obstacle.
                if((alignment>.5 && routeLeader) || other.speed<=stoppedSpeedThreshold) {
                    double gap=travelled+along-e.length*.5-lengthExtent;
                    if(gap<best){best=gap;leaderSpeed=std::max(0.,other.speed*alignment);}
                }
            }
            travelled+=span;previous=e.path[i];
        }
        if(!std::isfinite(best)) return;
        double safe=safeFollowingSpeed(best-e.minGap,leaderSpeed,other.deceleration,e.deceleration,std::max(e.tau,drivingStep_),drivingStep_);
        if(best<e.minGap-1e-6 && best>=-1e-6) e.gapCompressed=true;
        if(safe<cap) {cap=safe;e.following=true;e.signalQueue=other.signal;e.maneuverQueue=other.maneuver;}
    };
    if(!drivingSnapshot_.empty()) for(const auto& other:drivingSnapshot_) consider(other);
    else for(const auto& other:entities_) consider({other.id,other.footprint(),other.speed,other.deceleration,other.waitingForLight || other.signalQueue,other.waitingForManeuver || other.maneuverQueue});
    return cap;
}
double Simulation::clearMovementFraction(const Footprint& from,const Footprint& to,const Entity& e,bool reservations) const {
    auto clear=[&](double t) {
        Footprint pose{from.center+(to.center-from.center)*t,from.heading+angleDelta(from.heading,to.heading)*t,e.length,e.width};
        return !blockedByLight(from,pose,e) && feasible(from,pose,e,true,reservations);
    };
    if(clear(1)) return 1;
    double low=0,high=1;
    for(int i=0;i<18;++i){double mid=(low+high)*.5;if(clear(mid))low=mid;else high=mid;}
    return low;
}
void Simulation::recordDrivingStatistics() {
    if(recoveryPreview_) return;
    for(auto& e:entities_) {
        if(e.gapCompressed && !e.wasGapCompressed)++compressedGapEvents_;
        if(e.emergencyBraking && !e.wasEmergencyBraking)++emergencyBrakeEvents_;
        if(e.contactLimited && !e.wasContactLimited)++contactLimitEvents_;
        e.wasGapCompressed=e.gapCompressed;e.wasEmergencyBraking=e.emergencyBraking;e.wasContactLimited=e.contactLimited;
    }
    std::vector<std::pair<std::uint64_t,std::uint64_t>> now;
    for(std::size_t i=0;i<entities_.size();++i)for(std::size_t j=i+1;j<entities_.size();++j) {
        const auto& a=entities_[i];const auto& b=entities_[j];
        if(length(a.center-b.center)>.5*(std::hypot(a.length,a.width)+std::hypot(b.length,b.width)))continue;
        if(!geometry::violatesClearance(corners(a.footprint()),corners(b.footprint()),0))continue;
        std::pair<std::uint64_t,std::uint64_t> pair{std::min(a.id,b.id),std::max(a.id,b.id)};now.push_back(pair);
        if(std::find(previousPhysicalCollisions_.begin(),previousPhysicalCollisions_.end(),pair)==previousPhysicalCollisions_.end())++physicalCollisionEvents_;
    }
    previousPhysicalCollisions_=std::move(now);
}
}
