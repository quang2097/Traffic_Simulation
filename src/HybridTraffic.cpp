#include "traffic/Simulation.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <cmath>
namespace traffic {
namespace {
double dot(Vec2 a,Vec2 b) { return a.x*b.x+a.y*b.y; }
Vec2 unit(Vec2 v) { return v*(1/std::max(1e-9,length(v))); }
double closest(Vec2 a,Vec2 b,Vec2 c,Vec2 d,double& t,double& u) {
    auto r=b-a,s=d-c,w=a-c; double aa=dot(r,r),bb=dot(r,s),cc=dot(s,s),dd=dot(r,w),ee=dot(s,w),den=aa*cc-bb*bb;
    t=den>1e-9?std::clamp((bb*ee-cc*dd)/den,0.,1.):0;
    u=std::clamp((bb*t+ee)/std::max(1e-9,cc),0.,1.); t=std::clamp((bb*u-dd)/std::max(1e-9,aa),0.,1.);
    return length((a+r*t)-(c+s*u));
}
std::pair<std::size_t,Vec2> project(const std::vector<Vec2>& points,Vec2 p) {
    double best=1e100; std::pair<std::size_t,Vec2> result{1,points.front()};
    for(std::size_t i=1;i<points.size();++i) { auto d=points[i]-points[i-1]; auto q=points[i-1]+d*std::clamp(dot(p-points[i-1],d)/std::max(1e-9,dot(d,d)),0.,1.); double distance=length(q-p); if(distance<best) { best=distance; result={i,q}; } }
    return result;
}
Vec2 along(const std::vector<Vec2>& points,double distance,Vec2& direction) {
    for(std::size_t i=1;i<points.size();++i) { auto d=points[i]-points[i-1]; double n=length(d); direction=unit(d); if(n>=distance) return points[i-1]+direction*distance; distance-=n; }
    return points.back();
}
// Build one forward-only curve rather than refitting hundreds of sampled
// guide points (which can introduce loops near a tight exit).
MotionPath crossingPath(const Map& map,const std::vector<DirectedLane>& lanes,const Entity& e,const Entity& target,double buffer,bool unrestricted=false) {
    Vec2 forward{std::cos(e.heading),std::sin(e.heading)},out=target.exitDirection;
    double turn=std::abs(angleDelta(e.heading,std::atan2(out.y,out.x)));
    double lead=!e.junctionEntryCleared && (turn>.05 || std::abs(e.lateralOffset)>.01)?e.length*1.5+1:0;
    // Stop lines on neighboring lanes may be staggered. Clear the farthest
    // nearby parallel row with the rear bumper before starting any turn.
    if(lead>0) for(const auto& lane:lanes) if(lane.roadGroup>=0 && lane.openJunction<0 &&
        length(lane.points.back()-target.junctionCenter)<=target.junctionRadius+1) {
        auto direction=unit(lane.points.back()-lane.points[lane.points.size()-2]);
        Vec2 side{-forward.y,forward.x};auto delta=lane.points.back()-e.center;
        if(dot(direction,forward)>.95 && std::abs(dot(delta,side))<(lane.width+e.width+e.length)*.5+buffer+.5)
            lead=std::max(lead,dot(delta,forward)+e.length*.5+.2);
    }
    Vec2 a=e.center+forward*lead,end=target.goal;
    if(turn>3.0) end=end+forward*std::max(0.,dot(a-end,forward));
    double distance=length(end-a),best=1e100;
    MotionPath chosen;
    for(double factor:{.25,.4,.55,.75,1.,1.4,2.}) {
        double handle=std::max(e.length*1.2,distance*factor);
        Vec2 b=a+forward*handle,c=end-out*handle;
        MotionPath trial;trial.points={e.center};trial.headings={e.heading};
        auto previous=e.footprint(); bool valid=true; double total=0,rotation=0;
        auto append=[&](Vec2 p,double h) {
            double d=length(p-previous.center),angle=std::abs(angleDelta(previous.heading,h));
            if(d<1e-9) return angle<.001;
            if(angle/d>1/std::max(1.,e.length*.65)) return false;
            auto pose=previous;pose.center=p;pose.heading=h;double pad;auto swept=geometry::swept(previous,pose,pad);
            if(!geometry::fits(map,swept,pad)) return false;
            // Incoming road corridors may be crossed, but never followed upstream.
            for(const auto& lane:lanes) if(!unrestricted && lane.roadGroup>=0 && lane.openJunction<0) {
                for(std::size_t i=1;i<lane.points.size();++i) {
                    auto delta=lane.points[i]-lane.points[i-1];double n=length(delta);if(n<1e-8)continue;auto direction=delta*(1/n);
                    // A free crossing may traverse the junction, not cut across
                    // another incoming queue behind its stop line. Even a nearly
                    // perpendicular car can otherwise clip a parked row with its
                    // nose. The initial straight prefix may clear its own row.
                    bool incoming=length(lane.points.back()-target.junctionCenter)<=target.junctionRadius+1;
                    auto entryDirection=unit(lane.points.back()-lane.points[lane.points.size()-2]);
                    Vec2 bodyForward{std::cos(h),std::sin(h)},bodySide{-std::sin(h),std::cos(h)};
                    bool clearingRow=dot(entryDirection,forward)>.95 && std::abs(angleDelta(e.heading,h))<.002;
                    if(incoming && !clearingRow) {
                        Vec2 side{-direction.y,direction.x};
                        double halfAlong=std::abs(dot(bodyForward,direction))*e.length*.5+std::abs(dot(bodySide,direction))*e.width*.5;
                        double halfAcross=std::abs(dot(bodyForward,side))*e.length*.5+std::abs(dot(bodySide,side))*e.width*.5;
                        double projection=dot(p-lane.points[i-1],direction);
                        double wa=lane.pointWidths.size()==lane.points.size()?lane.pointWidths[i-1]:lane.width;
                        double wb=lane.pointWidths.size()==lane.points.size()?lane.pointWidths[i]:lane.width;
                        if(projection>=-halfAlong-.08 && projection<=n+halfAlong+.08 &&
                           std::abs(dot(p-lane.points[i-1],side))<std::max(wa,wb)*.5+halfAcross+.08) {
                            geometry::Polygon corridor{lane.points[i-1]+side*(wa*.5),lane.points[i]+side*(wb*.5),
                                lane.points[i]-side*(wb*.5),lane.points[i-1]-side*(wa*.5)};
                            if(geometry::violatesClearance(corners(pose),corridor,.08))return false;
                        }
                    }
                    if(dot(direction,bodyForward)>=-.25)continue;
                    double along=dot(p-lane.points[i-1],direction);
                    double mouth=0;
                    if(i+1==lane.points.size() && length(lane.points.back()-target.junctionCenter)<=target.junctionRadius+1)
                        mouth=target.internalJunction>=0?.5:e.length+buffer+.5;
                    // A queue's nose extends beyond the centerline endpoint.
                    // Protect that mouth as well as the lane interior, or a
                    // crossing can stop nose-to-nose just outside the road.
                    if(along<0 || along>n+mouth)continue;
                    double width=lane.pointWidths.size()==lane.points.size()?std::max(lane.pointWidths[i-1],lane.pointWidths[i]):lane.width;
                    Vec2 sideways{-direction.y,direction.x};
                    double across=std::abs(dot(bodyForward,sideways))*e.length*.5+std::abs(dot(bodySide,sideways))*e.width*.5;
                    // An angled car's nose occupies much more than its width.
                    // Keep the full rotated body out of an opposing approach.
                    if(std::abs(dot(p-lane.points[i-1],sideways))<width*.5+across+.1) return false;
                }
            }
            total+=d;rotation+=angle;trial.points.push_back(p);trial.headings.push_back(h);previous=pose;return true;
        };
        if(lead>0 && !append(a,e.heading)) continue;
        int samples=std::max(40,static_cast<int>(std::ceil((length(b-a)+length(c-b)+length(end-c))/.2)));
        for(int i=1;i<=samples && valid;++i) {double t=static_cast<double>(i)/samples,u=1-t;auto tangent=(b-a)*(3*u*u)+(c-b)*(6*u*t)+(end-c)*(3*t*t);if(length(tangent)<1e-8){valid=false;break;}valid=append(a*(u*u*u)+b*(3*u*u*t)+c*(3*u*t*t)+end*(t*t*t),std::atan2(tangent.y,tangent.x));}
        if(valid && length(end-target.goal)>1e-8) valid=append(target.goal,std::atan2(out.y,out.x));
        if(valid && rotation<=turn+.45 && total<best) {best=total;chosen=std::move(trial);}
    }
    return chosen;
}
}
bool Simulation::prepareJunctionPlans(Entity& e) {
    const auto spawnRandom=random_;
    e.junctionPlans.assign(e.networkRoute.size(),{});
    e.admissionPlan={};
    for(std::size_t stage=e.networkStage+1;stage<e.networkRoute.size();++stage) {
        if(lanes_[e.networkRoute[stage]].openJunction<0 && (!carFollowing_.enabled || lanes_[e.networkRoute[stage]].internalJunction<0)) continue;
        const auto& approach=lanes_[e.networkRoute[stage-1]].points;
        Entity preview=e;
        preview.center=approach.back();
        auto direction=approach.back()-approach[approach.size()-2];
        if(flow_.sublanes && stage==e.networkStage+1 && lanes_[e.networkRoute[stage-1]].roadGroup>=0) {
            auto forward=unit(direction); preview.center=preview.center+Vec2{-forward.y,forward.x}*e.lateralOffset;
        }
        preview.heading=std::atan2(direction.y,direction.x);
        // A curved/tapered sublane merge can finish at a different tangent
        // from the raw centerline. Validate the exact future handoff pose.
        if(stage==e.networkStage+1 && !e.inOpenJunction && e.signalLight==-1 &&
           !e.path.empty() && e.pathHeadings.size()==e.path.size()) {
            preview.center=e.path.back(); preview.heading=e.pathHeadings.back();
        }
        if(!installNetworkStage(preview,stage)) { random_=spawnRandom; return false; }
        auto previous=preview.footprint();
        for(std::size_t i=1;i<preview.path.size();++i) {
            Footprint pose{preview.path[i],preview.pathHeadings[i],e.length,e.width};
            if(!feasible(previous,pose,preview,false)) { random_=spawnRandom; return false; }
            previous=pose;
        }
        e.junctionPlans[stage]={preview.path,preview.pathHeadings,preview.maneuvers};
    }
    random_=spawnRandom;
    return true;
}
bool Simulation::installNetworkStage(Entity& e,std::size_t stage) {
    if(stage>=e.networkRoute.size()) return false;
    const auto& lane=lanes_[e.networkRoute[stage]];
    Entity candidate=e; candidate.networkStage=stage; candidate.waitingForJunction=false; candidate.turnExitBlocked=false; candidate.exitWaitFor=0;
    candidate.internalJunction=lane.internalJunction;
    if(lane.internalJunction>=0) { candidate.activeJunction=lane.internalJunction; candidate.junctionEnteredAt=seconds_; }
    candidate.changingLanes=false; candidate.changingFrom=candidate.changingTo=-1;
    candidate.reservedTurn.clear(); candidate.reservedApproach.clear(); candidate.usingLocalDetour=false;
    std::vector<Vec2> guide{e.center};
    if(lane.openJunction>=0) {
        if(stage+1>=e.networkRoute.size() && !lane.destination) return false;
        candidate.inOpenJunction=true; candidate.activeJunction=lane.openJunction;
        candidate.junctionCenter=lane.junctionCenter; candidate.junctionRadius=lane.junctionRadius;
        candidate.laneRoute.clear(); candidate.junctionEnteredAt=seconds_;
        if(stage+1<e.networkRoute.size()) {
            const auto& exit=lanes_[e.networkRoute[stage+1]];
            candidate.goal=along(exit.points,std::min(e.length*1.5+2.,length(exit.points.back()-exit.points.front())*.35),candidate.exitDirection);
        } else { candidate.goal=lane.points.back(); candidate.exitDirection=unit(lane.points.back()-lane.points.front()); }
        Vec2 forward{std::cos(e.heading),std::sin(e.heading)};
        if(stage+1<e.networkRoute.size() && (std::abs(e.lateralOffset)>.01 || dot(forward,candidate.exitDirection)<.98)) {
            // Clear the whole incoming row before steering across its stripes.
            // Otherwise a later arrival can park beside a turning vehicle's
            // rear swing and permanently block both the turn and admission.
            auto a=e.center+forward*(e.length*1.5+1);
            guide.push_back(a);
            auto end=lanes_[e.networkRoute[stage+1]].points.front();
            double depth=std::max(e.length*2+routeBuffer(e),length(end-a)*.5);
            auto b=a+forward*depth,c=end-candidate.exitDirection*depth;
            int samples=std::max(20,static_cast<int>(std::ceil(depth*4)));
            for(int i=1;i<=samples;++i) {
                double t=static_cast<double>(i)/samples,s=1-t;
                guide.push_back(a*(s*s*s)+b*(3*s*s*t)+c*(3*s*t*t)+end*(t*t*t));
            }
        } else {
            auto offset=e.center-lane.points.front();
            for(std::size_t i=0;i<lane.points.size();++i) {
                double t=static_cast<double>(i)/std::max<std::size_t>(1,lane.points.size()-1);
                auto point=lane.points[i]+offset*((1-t)*(1-t));
                if(length(point-guide.back())>.05) guide.push_back(point);
            }
        }
        guide.push_back(candidate.goal);
    } else {
        if(e.inOpenJunction || (e.internalJunction>=0 && !carFollowing_.enabled)) candidate.lateralOffset=0;
        candidate.inOpenJunction=false; candidate.activeJunction=lane.internalJunction;
        candidate.laneRoute={e.networkRoute[stage]};
        if(stage) candidate.laneRoute.push_back(e.networkRoute[stage-1]);
        if(stage+1<e.networkRoute.size()) candidate.laneRoute.push_back(e.networkRoute[stage+1]);
        auto nearest=project(lane.points,e.center);
        for(std::size_t i=nearest.first;i<lane.points.size();++i) if(length(guide.back()-lane.points[i])>.05) guide.push_back(lane.points[i]);
        // An approach ends at its stop line with the front bumper, not the
        // center. A bus parked center-on-node otherwise projects half its body
        // into the crossing and traps a shorter neighbor that has begun turning.
        if(stage+1==e.networkRoute.size() || lanes_[e.networkRoute[stage+1]].openJunction>=0 || lanes_[e.networkRoute[stage+1]].internalJunction>=0) {
            double margin=e.length*.5+.2;
            while(guide.size()>1) { auto d=guide.back()-guide[guide.size()-2]; double n=length(d); if(n>margin+.1) { guide.back()=guide.back()-unit(d)*margin; break; } margin-=n; guide.pop_back(); }
        }
        if(guide.size()<2) return false;
        if(flow_.sublanes && std::abs(candidate.lateralOffset)>.01 && lane.roadGroup>=0) {
            double tail=0;
            bool freeExit=stage+1==e.networkRoute.size() || lanes_[e.networkRoute[stage+1]].openJunction>=0 || lanes_[e.networkRoute[stage+1]].internalJunction>=0;
            for(std::size_t i=guide.size()-1;i>0;--i) {
                auto at=project(lane.points,guide[i]);
                auto direction=unit(lane.points[at.first]-lane.points[at.first-1]); Vec2 side{-direction.y,direction.x};
                double offset=candidate.lateralOffset*(freeExit?1.:std::clamp(tail/(e.length*2+2),0.,1.));
                double limit=std::max(0.,(laneWidthAt(e.networkRoute[stage],guide[i])-e.width)*.5-.08);
                auto original=guide[i]; guide[i]=guide[i]+side*std::clamp(offset,-limit,limit);
                tail+=length(original-guide[i-1]);
            }
        }
        candidate.goal=guide.back();
        if(carFollowing_.enabled && lane.internalJunction>=0 && stage+1<e.networkRoute.size()) {
            candidate.junctionCenter=lane.junctionCenter;candidate.junctionRadius=lane.junctionRadius;
            const auto& exit=lanes_[e.networkRoute[stage+1]];
            candidate.goal=along(exit.points,e.length*.5+1,candidate.exitDirection);
            double limit=std::max(0.,(exit.width-e.width)*.5-.08);
            candidate.lateralOffset=std::clamp(e.lateralOffset,-limit,limit);
            candidate.goal=candidate.goal+Vec2{-candidate.exitDirection.y,candidate.exitDirection.x}*candidate.lateralOffset;
        }
    }
    const MotionPath* ready=nullptr;
    if(lane.openJunction>=0 || (carFollowing_.enabled && lane.internalJunction>=0)) {
        if(!e.admissionPlan.points.empty()) ready=&e.admissionPlan;
        else if(stage<e.junctionPlans.size() && !e.junctionPlans[stage].points.empty()) ready=&e.junctionPlans[stage];
    }
    // A path cursor must start at the actual pose. A "close enough" cached
    // anchor behind the bumper causes the first step to move backwards into
    // the following queue, even though the intended route is unobstructed.
    if(ready && length(ready->points.front()-e.center)<1e-7 && std::abs(angleDelta(e.heading,ready->headings.front()))<1e-7) {
        candidate.path=ready->points; candidate.pathHeadings=ready->headings; candidate.maneuvers=ready->turns;
        candidate.path.front()=e.center; candidate.pathHeadings.front()=e.heading;
        candidate.next=1; candidate.segmentProgress=0; ++candidate.routeRevision;
        candidate.reservationEnd=0; candidate.following=false; candidate.cooperativeRecovery=false;
    } else if((lane.openJunction>=0 || (carFollowing_.enabled && lane.internalJunction>=0)) && stage+1<e.networkRoute.size()) {
        auto motion=crossingPath(map_,lanes_,e,candidate,routeBuffer(e),lane.semanticTransfer);
        if(motion.points.empty()) {
            if(!lane.semanticTransfer)return false;
            auto alternative=plan(candidate,0,false);
            if(alternative.empty() || !installPath(candidate,std::move(alternative),std::max(4.,e.length*2)))return false;
            motion={candidate.path,candidate.pathHeadings,candidate.maneuvers};
        }
        candidate.path=std::move(motion.points); candidate.pathHeadings=std::move(motion.headings); candidate.maneuvers.clear();
        candidate.next=1;candidate.segmentProgress=0;++candidate.routeRevision;
        candidate.reservationEnd=0;candidate.following=false;candidate.cooperativeRecovery=false;
    } else if(!installPath(candidate,std::move(guide),std::max(4.,e.length*2))) {
        if(lane.openJunction<0) return false;
        auto alternative=plan(candidate,0,false);
        if(alternative.empty() || !installPath(candidate,std::move(alternative),std::max(4.,e.length*2))) return false;
    }
    candidate.admissionPlan={};
    e=std::move(candidate); return true;
}
bool Simulation::junctionAdmission(const Entity& e,int linkId,const std::vector<Vec2>& movement) const {
    if(e.priorityYielding)return false;
    if(e.escapingGridlock) return true;
    if(flow_.predictiveJunctions) return predictiveAdmission(e,linkId,movement);
    const auto& link=lanes_[linkId];
    if(e.networkStage+2>=e.networkRoute.size() && !link.destination) return false;
    int exitId=e.networkStage+2<e.networkRoute.size()?e.networkRoute[e.networkStage+2]:-1;
    Vec2 direction,merge;
    if(exitId>=0) merge=along(lanes_[exitId].points,e.length*1.5+2,direction);
    else { merge=link.points.back(); direction=unit(merge-link.points.front()); }
    Footprint slot{merge,std::atan2(direction.y,direction.x),e.length*2+2,e.width};
    for(const auto& other:entities_) if(other.id!=e.id) {
        double gap=movementGap(e,other);
        if(geometry::violatesClearance(corners(slot),corners(other.footprint()),gap+.2)) return false;
        int junction=link.openJunction>=0?link.openJunction:link.internalJunction;
        if(!other.onJunction() || (other.activeJunction!=junction && !link.semanticTransfer && !(other.networkStage<other.networkRoute.size() && lanes_[other.networkRoute[other.networkStage]].semanticTransfer)))continue;
        if(other.currentStopSeconds>2) return false;
        // Admit concurrent disjoint movements. Crossing corridors wait outside,
        // so a stopped exit cannot create a circular wait inside the junction.
        if(exitId>=0 && other.networkStage+1<other.networkRoute.size() && other.networkRoute[other.networkStage+1]==exitId) return false;
        Vec2 previous=other.center; double otherDistance=0;
        double radius=.5*(std::hypot(e.length,e.width)+std::hypot(other.length,other.width))+gap+.5;
        double ownSpeed=std::clamp(e.desiredSpeed,2.,5.),otherSpeed=std::clamp(other.speed,2.,5.);
        for(std::size_t i=other.next;i<other.path.size();++i) {
            double ownDistance=0,segment=length(other.path[i]-previous);
            for(std::size_t j=1;j<movement.size();++j) {
                double distance=length(movement[j]-movement[j-1]),ownFraction=0,otherFraction=0;
                double gapDistance=closest(movement[j-1],movement[j],previous,other.path[i],ownFraction,otherFraction);
                double ownEta=(ownDistance+distance*ownFraction)/ownSpeed,otherEta=(otherDistance+segment*otherFraction)/otherSpeed;
                if(std::abs(ownEta-otherEta)<2+radius/std::min(ownSpeed,otherSpeed) && gapDistance<radius) return false;
                ownDistance+=distance;
            }
            otherDistance+=segment;
            previous=other.path[i];
        }
    }
    return true;
}
bool Simulation::advanceNetworkStage(Entity& e) {
    if(e.signalLight!=-1 || e.reversing) { e.junctionStatus="Signal queue or reversing"; return false; }
    if(e.networkRoute.empty() || e.networkStage+1>=e.networkRoute.size()) return false;
    int next=e.networkRoute[e.networkStage+1];
    if(e.next<e.path.size()) {
        // A tiny final alignment at the stop line can demand an impossible
        // rotation between two packed neighbors. Rebuild the crossing from
        // the actual pose rather than insisting on that artificial waypoint.
        if(e.currentStopSeconds<2 || (lanes_[next].openJunction<0 && lanes_[next].internalJunction<0) || stopReason(e)==StopReason::Signal) return false;
        // A rigid body's steering clearance scales with its size. A fixed
        // half-meter tolerance still strands a leading bike about 0.7 m short.
        double handoffDistance=std::clamp(e.length*.5,.5,2.);
        double remaining=0;auto previous=e.center;
        for(std::size_t i=e.next;i<e.path.size();++i){remaining+=length(e.path[i]-previous);previous=e.path[i];if(remaining>handoffDistance)return false;}
    }
    if(lanes_[next].openJunction>=0 || lanes_[next].internalJunction>=0) {
        if(seconds_<e.junctionRetryAfter) return false;
        Entity candidate=e;
        if(!installNetworkStage(candidate,e.networkStage+1)) { e.junctionStatus="No feasible crossing from current pose"; e.junctionRetryAfter=seconds_+1; e.waitingForJunction=true; e.turnExitBlocked=true; e.speed=0; return false; }
        // Cache this exact motion for retries and for the admission forecast.
        e.admissionPlan={candidate.path,candidate.pathHeadings,candidate.maneuvers};
        // A clear first step is insufficient: a long straight lead can stop
        // against an adjacent approach queue before its turn begins. Validate
        // the entire committed crossing against current road bodies so that
        // those queues can enter before we occupy their departure corridor.
        if(flow_.predictiveJunctions && !e.escapingGridlock && candidate.path.size()>1) {
            bool blocked=false;
            for(const auto& other:entities_) if(other.id!=e.id && !other.onJunction()) {
                double gap=movementGap(e,other);
                auto from=e.footprint();
                for(std::size_t i=1;i<candidate.path.size();++i) {
                    auto to=from;to.center=candidate.path[i];to.heading=candidate.pathHeadings[i];
                    double reach=.5*std::hypot(e.length,e.width)+.5*std::hypot(other.length,other.width)+gap+length(to.center-from.center);
                    if(length(other.center-from.center)<=reach &&
                       !geometry::motionClearance(from,to,other.footprint(),gap,e.cooperativeRecovery)){blocked=true;break;}
                    from=to;
                }
                if(blocked)break;
            }
            if(blocked) {
                e.junctionStatus="Crossing corridor occupied by road traffic";e.junctionRetryAfter=seconds_+.15;
                e.waitingForJunction=e.next>=e.path.size();e.turnExitBlocked=e.waitingForJunction;e.speed=0;return false;
            }
        }
        if(!junctionAdmission(e,next,candidate.path)) {
            e.junctionStatus="Crossing conflict or occupied exit buffer";
            e.admissionPlan={candidate.path,candidate.pathHeadings,candidate.maneuvers};
            e.junctionRetryAfter=seconds_+.15;
            e.waitingForJunction=true; e.turnExitBlocked=true; e.speed=0; return false;
        }
        e=std::move(candidate); e.junctionStatus.clear(); ++junctionAdmissions_; return true;
    }
    bool leaving=e.onJunction();
    if(!installNetworkStage(e,e.networkStage+1)) { e.speed=0; return false; }
    if(leaving) ++e.junctionCrossings;
    return true;
}
void Simulation::updateLaneChanges() {
    if(recoveryPreview_) return;
    for(auto& e:entities_) {
        if(e.reservesSublaneGap()) continue;
        if(e.changingLanes) {
            if(dot(e.center-e.laneChangeEnd,e.laneChangeDirection)>e.length*.5) {
                Entity settled=e;settled.changingLanes=false;settled.changingFrom=settled.changingTo=-1;
                settled.laneRoute={e.networkRoute[e.networkStage]};
                if(e.networkStage+1<e.networkRoute.size())settled.laneRoute.push_back(e.networkRoute[e.networkStage+1]);
                if(!fitsLaneRoute(e.footprint(),settled)) {e.laneChangeStatus="Finishing alignment with target lane";continue;}
                e.changingLanes=false; e.changingFrom=e.changingTo=-1;
                e.laneRoute={e.networkRoute[e.networkStage]}; if(e.networkStage+1<e.networkRoute.size()) e.laneRoute.push_back(e.networkRoute[e.networkStage+1]);
            } else continue;
        }
        if(e.networkRoute.empty()) { e.laneChangeStatus="legacy route / no generated passing lane"; continue; }
        if(e.escapingGridlock) { e.laneChangeStatus="clearing gridlock"; continue; }
        if(e.onJunction()) { e.laneChangeStatus=e.inOpenJunction?"free-space junction navigation":"internal junction connector"; continue; }
        if(e.waitingForJunction || e.signalLight!=-1 || e.reversing || seconds_<e.laneChangeAfter) continue;
        e.laneChangeAfter=seconds_+.4;
        int id=e.networkRoute[e.networkStage]; const auto& lane=lanes_[id];
        if(lane.roadGroup<0) continue;
        Vec2 forward{std::cos(e.heading),std::sin(e.heading)},side{-forward.y,forward.x};
        auto expectedSpeed=[&](int laneId,Vec2 origin,Vec2 direction) {
            double result=std::min(e.desiredSpeed,lanes_[laneId].speedLimit); Vec2 lateral{-direction.y,direction.x};
            for(const auto& actor:entities_) if(actor.id!=e.id) {
                auto delta=actor.center-origin; double ahead=dot(delta,direction);
                if(ahead<=0 || ahead>std::max(30.,e.speed*5) || dot(direction,{std::cos(actor.heading),std::sin(actor.heading)})<.7) continue;
                double gap=movementGap(e,actor);
                if(std::abs(dot(delta,lateral))>(e.width+actor.width)*.5+gap) continue;
                double space=ahead-(e.length+actor.length)*.5-gap;
                // Compare the lanes before the leader has already forced hard
                // braking. This is a planning forecast, not a weaker gap check.
                space+=std::min(actor.speed,actor.desiredSpeed)*3-std::min(e.desiredSpeed,lanes_[laneId].speedLimit)*3;
                double headway=(e.kind==VehicleKind::Bike || e.kind==VehicleKind::Motorbike)?urbanTraffic_.motorbikeHeadway:urbanTraffic_.otherHeadway;
                double braking=2*e.acceleration,bt=braking*headway;
                result=std::min(result,std::max(0.,std::sqrt(bt*bt+actor.speed*actor.speed+2*braking*std::max(0.,space))-bt));
            }
            return result;
        };
        double currentSpeed=expectedSpeed(id,e.center,forward);
        auto here=project(lane.points,e.center); double remaining=length(lane.points[here.first]-here.second);
        for(std::size_t i=here.first+1;i<lane.points.size();++i) remaining+=length(lane.points[i]-lane.points[i-1]);
        int neighbors[2]={-1,-1}; double closest[2]={1e100,1e100};
        for(std::size_t target=0;target<lanes_.size();++target) {
            const auto& other=lanes_[target]; if(static_cast<int>(target)==id || other.roadGroup<0 || other.openJunction>=0) continue;
            auto at=project(other.points,e.center); auto direction=unit(other.points[at.first]-other.points[at.first-1]);
            double distance=length(at.second-e.center),across=dot(at.second-e.center,side);
            if(dot(direction,forward)<.95 || distance<.2 || distance>(laneWidthAt(id,e.center)+laneWidthAt(static_cast<int>(target),at.second))*.75+1) continue;
            int which=across>0?1:0; if(distance<closest[which]) { closest[which]=distance; neighbors[which]=static_cast<int>(target); }
        }
        struct Choice { int lane; double score; std::string reason; }; std::vector<Choice> choices;
        bool preparingLeft=false;
        if(e.networkStage+2<e.networkRoute.size()) { const auto& exit=lanes_[e.networkRoute[e.networkStage+2]]; preparingLeft=dot(unit(exit.points[1]-exit.points[0]),side)<-.25 && remaining<std::max(35.,e.desiredSpeed*8); }
        for(int target:neighbors) if(target>=0) {
            const auto& other=lanes_[target]; auto at=project(other.points,e.center); auto direction=unit(other.points[at.first]-other.points[at.first-1]);
            double speed=expectedSpeed(target,at.second,direction),score=0; std::string reason;
            if(speed-currentSpeed>.5) { score=speed-currentSpeed; reason="speed gain"; }
            if(e.networkStage+2<e.networkRoute.size() && remaining<std::max(35.,e.desiredSpeed*8)) {
                auto exit=lanes_[e.networkRoute[e.networkStage+2]].points.front();
                double improvement=length(lane.points.back()-exit)-length(other.points.back()-exit);
                if(improvement>.75 && 5+improvement*.25>score) { score=5+improvement*.25; reason="strategic exit positioning"; }
            }
            if(!preparingLeft && dot(at.second-e.center,side)>.2 && e.speed>3 && speed>=currentSpeed-.3 && currentSpeed>=e.desiredSpeed*.8 && score<.35) { score=.35; reason="keep right"; }
            if(score>.2) choices.push_back({target,score,reason});
        }
        std::sort(choices.begin(),choices.end(),[](const auto& a,const auto& b){return a.score>b.score;});
        e.laneChangeStatus=choices.empty()?(neighbors[0]<0 && neighbors[1]<0?"no same-direction adjacent lane":"no useful speed/route gain"):"evaluating target lane";
        for(const auto& choice:choices) {
            std::size_t target=static_cast<std::size_t>(choice.lane); ++laneChangeAttempts_;
            const auto& otherLane=lanes_[target];
            if(static_cast<int>(target)==id || otherLane.roadGroup<0 || otherLane.openJunction>=0) continue;
            auto at=project(otherLane.points,e.center); double lateral=length(at.second-e.center);
            bool reservedExit=false;
            for(const auto& owner:entities_) if(owner.onJunction() && owner.networkStage+1<owner.networkRoute.size() && owner.networkRoute[owner.networkStage+1]==static_cast<int>(target) && length(at.second-otherLane.points.front())<e.length*4+10) reservedExit=true;
            if(reservedExit) { e.laneChangeStatus="junction exit buffer reserved"; continue; }
            Vec2 direction=unit(otherLane.points[at.first]-otherLane.points[at.first-1]);
            if(dot(direction,forward)<.95) continue;
            if(laneWidthAt(static_cast<int>(target),at.second)<e.width+.1) { e.laneChangeStatus="target lane too narrow"; continue; }
            // Keep the selected destination/turn. A sibling must reach the same
            // outgoing lane through a compatible internal movement.
            int replacement=-1;
            if(e.networkStage+1<e.networkRoute.size() && lanes_[e.networkRoute[e.networkStage+1]].roadGroup>=0) {
                // Signal stations can join physical lanes directly, without an
                // intervening junction link. Do not move into a sibling that
                // cannot continue into the committed lane beyond that station.
                int continuation=e.networkRoute[e.networkStage+1];
                if(std::find(otherLane.successors.begin(),otherLane.successors.end(),continuation)==otherLane.successors.end()) {
                    e.laneChangeStatus="target cannot reach next road lane"; continue;
                }
            } else if(e.networkStage+2<e.networkRoute.size()) {
                int destination=e.networkRoute[e.networkStage+2];
                for(int link:otherLane.successors) if(std::find(lanes_[link].successors.begin(),lanes_[link].successors.end(),destination)!=lanes_[link].successors.end()) { replacement=link; break; }
                if(replacement<0) { e.laneChangeStatus="target cannot reach chosen exit"; continue; }
            }
            if(e.networkStage+1==e.networkRoute.size() && length(otherLane.points.back()-lane.points.back())>std::max(5.,lane.width+otherLane.width)) continue;
            double mergeDistance=std::max({8.,e.length*3,e.speed*2.5});
            if(remaining<mergeDistance+e.length+1) { e.laneChangeStatus="not enough distance before junction"; continue; }
            Vec2 merge=at.second; std::size_t mergeIndex=at.first; double left=mergeDistance;
            for(;mergeIndex<otherLane.points.size();++mergeIndex) {
                auto delta=otherLane.points[mergeIndex]-merge; double n=length(delta); direction=unit(delta);
                if(n>=left) { merge=merge+direction*left; left=0; break; }
                left-=n; merge=otherLane.points[mergeIndex];
            }
            if(left>1e-6 || mergeIndex>=otherLane.points.size()) { e.laneChangeStatus="target lane ends before merge"; continue; }
            if(!laneChangeAllowed(id,static_cast<int>(target),e.center,merge,std::hypot(e.length,e.width))) { e.laneChangeStatus="lane change prohibited in this road section"; continue; }
            bool gap=true;
            for(auto& actor:entities_) if(actor.id!=e.id) {
                Vec2 targetSide{-direction.y,direction.x}; double along=dot(actor.center-at.second,direction),across=std::abs(dot(actor.center-at.second,targetSide));
                double preferred=movementGap(e,actor);
                if(actor.changingLanes && geometry::distance({e.center,merge},{actor.center,actor.laneChangeEnd})<(e.width+actor.width)*.5+preferred+.5) gap=false;
                if(along<0 && across<(e.width+actor.width)*.5+preferred) {
                    double rearGap=-along-(e.length+actor.length)*.5;
                    double required=preferred+actor.speed*.9+std::max(0.,actor.speed-e.speed)*2.5;
                    if(carFollowing_.enabled) required=actor.minGap+actor.speed*actor.tau+
                        std::max(0.,brakingDistance(actor.speed,actor.deceleration,drivingStep_)-brakingDistance(e.speed,std::max(actor.deceleration,e.deceleration),drivingStep_));
                    if(rearGap<required) {
                        gap=false;
                        if(actor.speed>1.5) {
                            if(actor.cooperativeFor!=e.id || seconds_>=actor.cooperativeUntil) ++cooperativeGapRequests_;
                            actor.cooperativeFor=e.id; actor.cooperativeLane=static_cast<int>(target); actor.cooperativeUntil=seconds_+1.2;
                            actor.cooperativeSpeedCap=std::max(1.5,std::min(actor.speed,e.speed*.8));
                        }
                    }
                }
                Vec2 predicted=actor.center+Vec2{std::cos(actor.heading),std::sin(actor.heading)}*(actor.speed*1.5);
                Footprint future{predicted,actor.heading,actor.length,actor.width};
                Footprint slot{merge,std::atan2(direction.y,direction.x),e.length+2*std::max(2.,e.speed*.7),e.width};
                if(geometry::violatesClearance(corners(slot),corners(future),movementGap(e,actor))) { gap=false; }
            }
            if(!gap) { e.laneChangeStatus="waiting for safe target gap (cooperation requested)"; continue; }
            Entity candidate=e; candidate.networkRoute[candidate.networkStage]=static_cast<int>(target); candidate.lateralOffset=0;
            candidate.changingLanes=true; candidate.changingFrom=id; candidate.changingTo=static_cast<int>(target); candidate.laneChangeEnd=merge; candidate.laneChangeDirection=direction;
            if(replacement>=0) candidate.networkRoute[candidate.networkStage+1]=replacement;
            candidate.laneRoute={id,static_cast<int>(target)};
            if(replacement>=0) candidate.laneRoute.push_back(replacement);
            std::vector<Vec2> guide{merge};
            for(std::size_t i=mergeIndex;i<otherLane.points.size();++i) guide.push_back(otherLane.points[i]);
            if(e.networkStage+1==e.networkRoute.size()) {
                double margin=e.length*.5+.2;
                while(guide.size()>1) {
                    auto segment=guide.back()-guide[guide.size()-2]; double n=length(segment);
                    if(n>margin+.1) { guide.back()=guide.back()-unit(segment)*margin; break; }
                    margin-=n; guide.pop_back();
                }
            }
            auto tail=curvePath(map_,{merge,std::atan2(direction.y,direction.x),e.length,e.width},guide);
            if(tail.points.empty()) { e.laneChangeStatus="merge continuation does not fit mask"; continue; }
            candidate.path={e.center}; candidate.pathHeadings={e.heading}; candidate.maneuvers.clear();
            Vec2 a=e.center,b=e.center+forward*(mergeDistance*.45),c=merge-direction*(mergeDistance*.45);
            int samples=std::max(30,static_cast<int>(std::ceil(mergeDistance/.1)));
            for(int i=1;i<=samples;++i) {
                double t=static_cast<double>(i)/samples,s=1-t;
                candidate.path.push_back(a*(s*s*s)+b*(3*s*s*t)+c*(3*s*t*t)+merge*(t*t*t));
                auto tangent=(b-a)*(3*s*s)+(c-b)*(6*s*t)+(merge-c)*(3*t*t);
                candidate.pathHeadings.push_back(std::atan2(tangent.y,tangent.x));
            }
            std::size_t shift=candidate.path.size()-1;
            for(std::size_t i=1;i<tail.points.size();++i) { candidate.path.push_back(tail.points[i]); candidate.pathHeadings.push_back(tail.headings[i]); }
            for(auto turn:tail.turns) { turn.begin+=shift; turn.end+=shift; candidate.maneuvers.push_back(std::move(turn)); }
            candidate.next=1; candidate.segmentProgress=0; ++candidate.routeRevision;
            candidate.reservedTurn.clear(); candidate.reservedApproach.clear();
            bool safe=true; auto previous=e.footprint(); double checked=0;
            for(std::size_t i=1;i<candidate.path.size() && checked<mergeDistance+e.length;++i) {
                Footprint pose{candidate.path[i],candidate.pathHeadings[i],e.length,e.width};
                if(!feasible(previous,pose,candidate,true)) { safe=false; break; }
                checked+=length(pose.center-previous.center); previous=pose;
            }
            if(!safe) { e.laneChangeStatus="swept merge clearance blocked"; continue; }
            Entity settled=candidate; settled.changingLanes=false; settled.laneRoute={static_cast<int>(target)};
            if(replacement>=0) settled.laneRoute.push_back(replacement);
            previous={merge,std::atan2(direction.y,direction.x),e.length,e.width};
            for(std::size_t i=shift+1;i<candidate.path.size();++i) {
                Footprint pose{candidate.path[i],candidate.pathHeadings[i],e.length,e.width};
                if(!feasible(previous,pose,settled,false)) { safe=false; break; }
                previous=pose;
            }
            if(!safe) { e.laneChangeStatus="target corridor is too tight farther ahead"; continue; }
            if(!prepareJunctionPlans(candidate)) { e.laneChangeStatus="target lane has no feasible junction crossing"; continue; }
            candidate.goal=candidate.path.back(); candidate.laneChangeAfter=seconds_+3; ++candidate.laneChanges; candidate.laneChangeStatus="changing: "+choice.reason;
            for(auto& actor:entities_) if(actor.cooperativeFor==e.id && actor.cooperativeLane!=static_cast<int>(target)) actor.cooperativeUntil=seconds_;
            ++totalLaneChanges_; e=std::move(candidate); break;
        }
    }
}
}
