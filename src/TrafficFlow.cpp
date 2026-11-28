#include "traffic/Simulation.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace traffic {
namespace {
double dot(Vec2 a,Vec2 b) { return a.x*b.x+a.y*b.y; }
Vec2 unit(Vec2 p) { return p*(1/std::max(1e-9,length(p))); }
struct Track {
    std::vector<Vec2> p;
    std::vector<double> s,h;
    explicit Track(const Entity& e,double limit=std::numeric_limits<double>::infinity()) : p{e.center},s{0},h{e.heading} {
        for(std::size_t i=e.next;i<e.path.size();++i) {
            double d=length(e.path[i]-p.back());
            if(d<1e-8) continue;
            s.push_back(s.back()+d); p.push_back(e.path[i]); h.push_back(e.pathHeadings[i]);
            if(s.back()>=limit) break;
        }
    }
    Footprint at(const Entity& e,double distance) const {
        if(p.size()==1 || distance>=s.back()) return {p.back(),h.back(),e.length,e.width};
        auto i=static_cast<std::size_t>(std::upper_bound(s.begin(),s.end(),distance)-s.begin());
        i=std::clamp<std::size_t>(i,1,s.size()-1);
        double t=(distance-s[i-1])/(s[i]-s[i-1]);
        return {p[i-1]+(p[i]-p[i-1])*t,h[i-1]+angleDelta(h[i-1],h[i])*t,e.length,e.width};
    }
};
double travel(double v,double a,double cap,double time) {
    double t=std::min(time,std::max(0.,(cap-v)/a));
    return v*t+.5*a*t*t+cap*(time-t);
}
bool conflict(const Footprint& a,const Footprint& b,double gap) {
    auto delta=a.center-b.center;
    double radius=.5*(std::sqrt(a.length*a.length+a.width*a.width)+std::sqrt(b.length*b.length+b.width*b.width))+gap;
    if(dot(delta,delta)>radius*radius) return false;
    return geometry::violatesClearance(corners(a),corners(b),gap);
}
}
void Simulation::setTrafficFlowSettings(TrafficFlowSettings settings) {
    if(!std::isfinite(settings.priorityWaitSeconds) || settings.priorityWaitSeconds<1 || settings.priorityWaitSeconds>3600)throw std::invalid_argument("Invalid priority yield timeout");
    if(!std::isfinite(settings.lateralResolution) || settings.lateralResolution<.1 || settings.lateralResolution>2 || !std::isfinite(settings.lateralGap) || settings.lateralGap<0 || settings.lateralGap>2) throw std::invalid_argument("Invalid sublane settings");
    flow_=settings;
    if(flow_.predictiveJunctions) {
        turnClaims_.clear();
        for(auto& e:entities_) if(!e.networkRoute.empty()) { e.reservedTurn.clear(); e.reservedApproach.clear(); e.yieldToVehicle=0; e.tugActive=false; }
    }
}
void Simulation::saveTrafficLights(const std::string& file) const {
    std::ofstream out(file); if(!out) throw std::runtime_error("Cannot save signal links");
    out<<std::setprecision(17)<<"TRAFFIC_SIGNALS 6 "<<lights_.size()<<'\n';
    for(const auto& l:lights_) {
        out<<l.id<<' '<<l.a.x<<' '<<l.a.y<<' '<<l.b.x<<' '<<l.b.y<<' '<<l.greenSeconds<<' '<<l.yellowSeconds<<' '<<l.redSeconds<<' '<<l.offsetSeconds<<' '<<l.enabled<<' '<<l.permissive<<' '<<l.allowedA.x<<' '<<l.allowedA.y<<' '<<l.allowedB.x<<' '<<l.allowedB.y<<' '
           <<l.queueWindowSeconds<<' '<<l.queueThresholdPercent<<' '<<l.queueDetectors.size()<<' '<<l.controlledDirection().x<<' '<<l.controlledDirection().y<<' '<<l.anchorNode<<' '
           <<l.completeJunctionOnGreen<<' '<<l.completionJunction<<' '<<l.completionCenter.x<<' '<<l.completionCenter.y<<' '<<l.completionRadius<<' '<<l.trSeconds<<' '<<l.trJunction<<' '<<l.trGroup<<'\n';
        for(const auto& line:l.queueDetectors) out<<line.id<<' '<<line.a.x<<' '<<line.a.y<<' '<<line.b.x<<' '<<line.b.y<<'\n';
    }
    if(!out) throw std::runtime_error("Cannot write signal links");
}
void Simulation::loadTrafficLights(const std::string& file) {
    std::ifstream in(file); std::string magic; int version; std::size_t count;
    if(!(in>>magic>>version>>count) || magic!="TRAFFIC_SIGNALS" || version<1 || version>6 || count>10000) throw std::runtime_error("Invalid signal links file");
    std::vector<TrafficLight> next(count);
    std::size_t totalDetectors=0;
    for(auto& l:next) {
        if(!(in>>l.id>>l.a.x>>l.a.y>>l.b.x>>l.b.y>>l.greenSeconds>>l.yellowSeconds>>l.redSeconds>>l.offsetSeconds>>l.enabled>>l.permissive>>l.allowedA.x>>l.allowedA.y>>l.allowedB.x>>l.allowedB.y)) throw std::runtime_error("Incomplete signal links file");
        // Version 1 incorrectly offered destination behavior. Read and discard
        // its flag so existing maps retain their links without early despawns.
        bool obsoleteDestination=false;
        if(version==1 && !(in>>obsoleteDestination)) throw std::runtime_error("Incomplete signal links file");
        if(version>=3) {
            std::size_t detectors;
            if(!(in>>l.queueWindowSeconds>>l.queueThresholdPercent>>detectors) || detectors>10000 || (totalDetectors+=detectors)>100000)
                throw std::runtime_error("Invalid queue detector configuration");
            if(version>=4 && !(in>>l.outDirection.x>>l.outDirection.y>>l.anchorNode))throw std::runtime_error("Invalid directional signal");
            if(version>=5 && !(in>>l.completeJunctionOnGreen>>l.completionJunction>>l.completionCenter.x>>l.completionCenter.y>>l.completionRadius))throw std::runtime_error("Invalid green junction completion setting");
            if(version>=6 && !(in>>l.trSeconds>>l.trJunction>>l.trGroup))throw std::runtime_error("Invalid normal-light Tr setting");
            l.queueDetectors.resize(detectors);
            for(auto& line:l.queueDetectors)
                if(!(in>>line.id>>line.a.x>>line.a.y>>line.b.x>>line.b.y)) throw std::runtime_error("Incomplete queue detector line");
        }
    }
    for(std::size_t i=0;i<next.size();++i) for(std::size_t j=0;j<i;++j) if(next[i].id && next[i].id==next[j].id) throw std::runtime_error("Duplicate signal identity");
    setTrafficLights(std::move(next));
}
bool Simulation::parallelDeparture(const Entity& a,const Entity& b) const {
    bool sameOrigin=(a.spawnLane>=0 && a.spawnLane==b.spawnLane) || (a.spawnNode>=0 && a.spawnNode==b.spawnNode);
    if(!sameOrigin || a.reversing || b.reversing || b.waitingForLight || b.desiredSpeed<=stoppedSpeedThreshold) return false;
    auto approach=[&](const Entity& actor) {
        if(actor.networkStage>=actor.networkRoute.size() || (actor.onJunction() && actor.networkStage==0)) return -1;
        return actor.networkRoute[actor.networkStage-(actor.onJunction()?1:0)];
    };
    int left=approach(a),right=approach(b);
    if(left<0 || right<0 || left>=static_cast<int>(lanes_.size()) || right>=static_cast<int>(lanes_.size())) return false;
    if(left!=right && (lanes_[left].roadGroup<0 || lanes_[left].roadGroup!=lanes_[right].roadGroup ||
        lanes_[left].travelDirection!=lanes_[right].travelDirection)) return false;
    Vec2 forward{std::cos(a.heading),std::sin(a.heading)},side{-forward.y,forward.x};
    if(dot(forward,{std::cos(b.heading),std::sin(b.heading)})<.98) return false;
    auto offset=b.center-a.center;
    double lateral=std::abs(dot(offset,side)),gap=movementGap(a,b);
    // Only companions beside each other, never a leader directly ahead or a
    // vehicle from the same source returning on a different approach.
    return std::abs(dot(offset,forward))<std::max(4.,a.length+b.length) &&
        lateral>=(a.width+b.width)*.5+gap-.01 && lateral<std::max(8.,lanes_[left].width+lanes_[right].width);
}
bool Simulation::predictiveAdmission(const Entity& e,int linkId,const std::vector<Vec2>& movement) const {
    // Use the same body tangents that the committed crossing will execute.
    Entity candidate=e;candidate.path=movement;candidate.next=1;
    bool exact=e.admissionPlan.points.size()==movement.size() && e.admissionPlan.headings.size()==movement.size();
    for(std::size_t i=0;exact && i<movement.size();++i)
        exact=e.admissionPlan.points[i].x==movement[i].x && e.admissionPlan.points[i].y==movement[i].y;
    if(exact) candidate.pathHeadings=e.admissionPlan.headings;
    else {
        candidate.pathHeadings.resize(movement.size());
        for(std::size_t i=1;i<movement.size();++i){auto d=movement[i]-movement[i-1];candidate.pathHeadings[i]=std::atan2(d.y,d.x);}
    }
    candidate.pathHeadings[0]=e.heading;
    Track own(candidate);
    int exit=e.networkStage+2<e.networkRoute.size()?e.networkRoute[e.networkStage+2]:-1;
    // Keep the merge buffer clear before committing to the free area.
    if(exit>=0) {
        const auto& lane=lanes_[exit]; Vec2 forward=unit(lane.points[1]-lane.points[0]);
        Footprint buffer{lane.points[0]+forward*(e.length+1),std::atan2(forward.y,forward.x),e.length*2+2,e.width};
        for(const auto& other:entities_) if(other.id!=e.id && !other.onJunction() && conflict(buffer,other.footprint(),movementGap(e,other))) return false;
    }
    double cap=std::min(e.desiredSpeed,lanes_[linkId].speedLimit);
    double horizon=std::min(60.,own.s.back()/std::max(1.,cap)+cap/e.acceleration+2);
    const double interval=flow_.compactAdmission?.1:.25;
    int junction=lanes_[linkId].openJunction>=0?lanes_[linkId].openJunction:lanes_[linkId].internalJunction;
    for(const auto& other:entities_) if(other.id!=e.id && other.onJunction() && (other.activeJunction==junction || lanes_[linkId].semanticTransfer || (other.networkStage<other.networkRoute.size() && lanes_[other.networkRoute[other.networkStage]].semanticTransfer))) {
        if(carFollowing_.enabled && lanes_[linkId].internalJunction>=0) {
            Vec2 forward{std::cos(e.heading),std::sin(e.heading)},offset=other.center-e.center;
            bool sameConnection=other.networkStage>0 && other.networkRoute[other.networkStage]==linkId &&
                other.networkRoute[other.networkStage-1]==e.networkRoute[e.networkStage];
            bool followingStream=sameConnection && dot(offset,forward)>(e.length+other.length)*.5+e.minGap;
            if(!followingStream) {
                // Connection conflicts are resolved before entry. Collinear
                // followers use car-following; separate lateral stripes may
                // proceed together when their actual swept corridors clear.
                struct Piece {geometry::Polygon polygon;double pad,x0=1e100,y0=1e100,x1=-1e100,y1=-1e100;};
                auto pieces=[](const Track& track,const Entity& actor) {
                    std::vector<Piece> result;auto from=actor.footprint();
                    for(double d=0;d<track.s.back();d+=1.) {
                        auto to=track.at(actor,std::min(track.s.back(),d+1.));Piece piece;piece.polygon=geometry::swept(from,to,piece.pad);
                        for(auto p:piece.polygon){piece.x0=std::min(piece.x0,p.x);piece.y0=std::min(piece.y0,p.y);piece.x1=std::max(piece.x1,p.x);piece.y1=std::max(piece.y1,p.y);}
                        result.push_back(std::move(piece));from=to;
                    }
                    return result;
                };
                auto ours=pieces(own,e),theirs=pieces(Track(other),other);
                for(const auto& a:ours)for(const auto& b:theirs) {
                    double pad=a.pad+b.pad+.02;
                    if(a.x1+pad<b.x0 || b.x1+pad<a.x0 || a.y1+pad<b.y0 || b.y1+pad<a.y0)continue;
                    if(geometry::violatesClearance(a.polygon,b.polygon,pad))return false;
                }
                continue;
            }
        }
        // A U-turn occupies the approach's opposing swept corridor. Reserve
        // its remaining corridor while disjoint streams continue.
        // ETA-only forecasts cannot safely arbitrate this opposing maneuver.
        auto reverses=[&](int entry,int target) {
            if(entry<0 || target<0) return false;
            const auto& points=lanes_[entry].points;
            const auto& outgoing=lanes_[target].points;
            return dot(unit(points.back()-points[points.size()-2]),unit(outgoing[1]-outgoing[0]))<-.8;
        };
        int otherExit=other.networkStage+1<other.networkRoute.size()?other.networkRoute[other.networkStage+1]:-1;
        if(reverses(e.networkRoute[e.networkStage],exit) ||
           (other.networkStage>0 && reverses(other.networkRoute[other.networkStage-1],otherExit))) {
            Track occupied(other);
            double gap=movementGap(e,other)+1.05;
            for(double a=0;a<=own.s.back()+1;a+=1) for(double b=0;b<=occupied.s.back()+1;b+=1)
                if(conflict(own.at(e,a),occupied.at(other,b),gap)) return false;
            continue;
        }
        if(other.currentStopSeconds>1) {
            if(!flow_.compactAdmission) return false;
            if(other.speed<=stoppedSpeedThreshold && seconds_-other.junctionEnteredAt>1) {
                // A stopped crossing only closes its occupied corridor. Keep
                // disjoint approaches moving without assuming it can accelerate.
                // Continuous sweeps need no extra sampled-pose cushion. A
                // legally adjacent vehicle must be allowed to move away.
                auto previous=e.footprint();
                for(std::size_t i=1;i<own.p.size();++i) {
                    Footprint pose{own.p[i],own.h[i],e.length,e.width};
                    if(!geometry::motionClearance(previous,pose,other.footprint(),movementGap(e,other)))return false;
                    previous=pose;
                }
                continue;
            }
        }
        Track track(other); double otherCap=std::min(other.desiredSpeed,lanes_[other.networkRoute[other.networkStage]].speedLimit);
        if(carFollowing_.enabled) {Entity forecast=other;otherCap=std::min(otherCap,carFollowingSpeed(forecast));}
        bool companion=parallelDeparture(e,other);
        double gap=movementGap(e,other)+.2;
        for(double t=0;t<horizon;t+=interval) {
            auto a=own.at(e,travel(e.speed,e.acceleration,cap,t));
            double distance=travel(other.speed,other.acceleration,otherCap,t);
            if(carFollowing_.enabled && other.speed>otherCap) {
                double slowing=std::min(t,(other.speed-otherCap)/other.deceleration);
                distance=other.speed*slowing-.5*other.deceleration*slowing*slowing+otherCap*(t-slowing);
            }
            auto b=track.at(other,distance);
            // The outgoing road takes over after the cached merge point.
            if(distance>track.s.back()) {
                Vec2 direction{std::cos(b.heading),std::sin(b.heading)};
                b.center=b.center+direction*(distance-track.s.back());
            }
            // Retain the original swept-time cushion even when sampling more
            // finely: acceleration forecasts do not account for later braking.
            double temporalMargin=.125*(cap+otherCap),forecastGap=gap;
            Vec2 ownForward{std::cos(a.heading),std::sin(a.heading)},otherForward{std::cos(b.heading),std::sin(b.heading)};
            if(companion && dot(ownForward,otherForward)>.98) {
                // A common translation cannot close a lateral gap. Applying
                // the sum-of-speeds cushion to a parallel departure serialized
                // the approach. Keep the whole forecast, using relative motion
                // while aligned; crossing/merging paths still have to clear.
                auto velocity=ownForward*std::min(cap,e.speed+e.acceleration*t)-otherForward*std::min(otherCap,other.speed+other.acceleration*t);
                temporalMargin=interval*.5*length(velocity);
                forecastGap=movementGap(e,other)+.02;
            }
            if(conflict(a,b,forecastGap+temporalMargin)) return false;
        }
    }
    return true;
}
bool Simulation::advancePredictiveVehicle(Entity& e,double dt) {
    if(e.next>=e.path.size()) return true;
    e.reservedTurn.clear(); e.reservedApproach.clear(); e.waitingForManeuver=false; e.tugActive=false;
    double cap=std::min(e.desiredSpeed,lanes_[e.networkRoute[e.networkStage]].speedLimit);
    double stopping=std::max(8.,e.speed*e.speed/(4*e.acceleration)+e.speed*.5+e.length);
    Track own(e,stopping+1);
    const auto& controls=lanes_[e.networkRoute.front()].turnControls;
    double bendBraking=2*e.acceleration;
    for(std::size_t i=1;i<own.p.size();++i) for(const auto& turn:controls) if(length(own.p[i]-turn.point)<=turn.radius) {
        double curvature=std::abs(angleDelta(own.h[i-1],own.h[i]))/std::max(.001,own.s[i]-own.s[i-1]);
        if(curvature>1e-5) {
            double target=std::sqrt(urbanTraffic_.lateralAcceleration/curvature+2*turn.braking*own.s[i-1]);
            if(target<cap) { cap=target; bendBraking=turn.braking; }
        }
    }
    // Match lane-bound red-light braking while preserving the crossing curve.
    double signalDistance=0;
    if(closestStoppingLight(e,&signalDistance)>=0)
        cap=std::min(cap,std::sqrt(4*e.acceleration*std::max(0.,signalDistance-e.length*.5-.12)));
    double bendCap=cap;
    // Braking-aware path following. Projection onto the committed trajectory
    // recognizes leaders around bends instead of testing only today's heading.
    if(carFollowing_.enabled) cap=std::min(cap,carFollowingSpeed(e));
    for(const auto& other:entities_) if(!carFollowing_.enabled && !e.escapingGridlock && other.id!=e.id && length(other.center-e.center)<std::max(20.,e.speed*3+e.length+other.length)) {
        double gap=movementGap(e,other);
        for(double d=.25;d<std::min(own.s.back(),stopping);d+=.5) {
            auto pose=own.at(e,d);
            // An additional lateral cushion makes legally packed neighbors
            // become phantom leaders. The actual smaller preferred gap is
            // already enforced by the swept movement check.
            if(!conflict(pose,other.footprint(),gap)) continue;
            Vec2 direction{std::cos(pose.heading),std::sin(pose.heading)};
            double alignment=dot(direction,{std::cos(other.heading),std::sin(other.heading)});
            if(alignment<.6 && other.speed>.1) {
                Track crossing(other,std::max(10.,other.speed*3));
                auto future=crossing.at(other,other.speed*d/std::max(1.,e.speed));
                if(!conflict(pose,future,gap+.2)) continue;
            }
            double leaderSpeed=std::max(0.,other.speed*dot(direction,{std::cos(other.heading),std::sin(other.heading)}));
            double braking=2*e.acceleration,headway=(e.width<1.2?.45:.9),bt=braking*headway;
            cap=std::min(cap,std::max(0.,std::sqrt(bt*bt+leaderSpeed*leaderSpeed+2*braking*std::max(0.,d-.4))-bt));
            break;
        }
    }
    if(e.escapingGridlock) cap=std::min(cap,gridlockEscape_.speed);
    double braking=cap<bendCap-1e-8?2*e.acceleration:bendBraking;
    double previousSpeed=e.speed;
    double newSpeed=e.speed<cap?std::min(cap,e.speed+e.acceleration*dt):std::max(cap,e.speed-braking*dt);
    if(carFollowing_.enabled) newSpeed=std::max(std::min(cap,e.speed+e.acceleration*dt),std::max(0.,e.speed-e.emergencyDeceleration*dt));
    e.emergencyBraking=carFollowing_.enabled && previousSpeed-newSpeed>e.deceleration*dt+1e-6;
    double distance=carFollowing_.enabled?newSpeed*dt:(e.speed+newSpeed)*.5*dt;
    double movedThisTick=0;
    bool clear=true;
    while(distance>1e-12 && e.next<e.path.size()) {
        double segment=length(e.path[e.next]-e.path[e.next-1]);
        if(segment<1e-9) { ++e.next; e.segmentProgress=0; continue; }
        double move=std::min(distance,segment-e.segmentProgress),t=(e.segmentProgress+move)/segment;
        auto from=e.footprint(),to=from;
        to.center=e.path[e.next-1]+(e.path[e.next]-e.path[e.next-1])*t;
        to.heading=e.pathHeadings[e.next-1]+angleDelta(e.pathHeadings[e.next-1],e.pathHeadings[e.next])*t;
        e.waitingForLight=blockedByLight(from,to,e);
        if(length(to.center-from.center)>move+1e-7) {clear=false;break;}
        double fraction=carFollowing_.enabled?clearMovementFraction(from,to,e,false):(!e.waitingForLight && feasible(from,to,e,true,false)?1:0);
        if(fraction<1) {e.contactLimited=!e.waitingForLight;to.center=from.center+(to.center-from.center)*fraction;to.heading=from.heading+angleDelta(from.heading,to.heading)*fraction;move*=fraction;clear=false;}
        e.center=to.center; e.heading=to.heading; e.segmentProgress+=move; distance-=move;
        movedThisTick+=move;
        if(e.signalLight!=-1) e.signalRemaining=std::max(0.,e.signalRemaining-move);
        if(e.segmentProgress>=segment-1e-9) { ++e.next; e.segmentProgress=0; }
        if(!clear)break;
    }
    e.speed=clear?newSpeed:carFollowing_.enabled?std::min(newSpeed,movedThisTick/dt):0;
    if(carFollowing_.enabled && previousSpeed-e.speed>e.deceleration*dt+1e-6)e.emergencyBraking=true;
    e.blockedSeconds=clear?0:e.blockedSeconds+dt;
    return true;
}

void Simulation::updateSublanes() {
    if(!flow_.sublanes || recoveryPreview_) return;
    for(auto& e:entities_) {
        if(e.reservesSublaneGap()) continue;
        e.sublaneReservation.clear();
        if(e.escapingGridlock || e.networkRoute.empty() || e.onJunction() || e.changingLanes || e.signalLight!=-1 || e.reversing || seconds_<e.sublaneAfter) continue;
        e.sublaneAfter=seconds_+.5;
        int laneId=e.networkRoute[e.networkStage]; const auto& lane=lanes_[laneId];
        bool junctionExit=e.networkStage+1<e.networkRoute.size() && (lanes_[e.networkRoute[e.networkStage+1]].openJunction>=0 || lanes_[e.networkRoute[e.networkStage+1]].internalJunction>=0);
        if(lane.roadGroup<0 || e.next>=e.path.size()) continue;
        Vec2 forward{std::cos(e.heading),std::sin(e.heading)},side{-forward.y,forward.x};
        double width=laneWidthAt(laneId,e.center),bestSpeed=followingSpeed(e),bestOffset=0;
        if(bestSpeed>=e.desiredSpeed*.95) { e.sublaneStatus="free ahead"; continue; }
        double nearest=1e100,currentOffset=0;
        for(std::size_t i=1;i<lane.points.size();++i) {
            auto d=lane.points[i]-lane.points[i-1]; auto q=lane.points[i-1]+d*std::clamp(dot(e.center-lane.points[i-1],d)/std::max(1e-9,dot(d,d)),0.,1.);
            if(length(q-e.center)<nearest) { nearest=length(q-e.center); currentOffset=dot(e.center-q,side); }
        }
        double limit=(width-e.width)*.5-(junctionExit?.08:.2);
        for(double offset=-limit;offset<=limit+.01;offset+=flow_.lateralResolution) {
            Entity candidate=e; candidate.center=e.center+side*(offset-currentOffset);
            double speed=followingSpeed(candidate);
            if(speed>bestSpeed+.5) { bestSpeed=speed; bestOffset=offset-currentOffset; }
        }
        if(std::abs(bestOffset)<.15) { e.sublaneStatus="no faster lateral gap"; continue; }
        // Shift continuously over a forward distance, never insert through a
        // blocked gap. The full swept connector and tapered corridor must fit.
        double mergeDistance=junctionExit?std::max(4.,e.speed*1.5+std::abs(bestOffset)*3.):std::max({e.length*2.,e.speed*.6,std::abs(bestOffset)*3.});
        Track track(e); if(track.s.back()<mergeDistance+e.length*2) continue;
        bool freeExit=e.networkStage+1==e.networkRoute.size() || lanes_[e.networkRoute[e.networkStage+1]].openJunction>=0 || lanes_[e.networkRoute[e.networkStage+1]].internalJunction>=0;
        std::vector<Vec2> guide{e.center};
        for(double d=.4;d<track.s.back();d+=.4) {
            auto pose=track.at(e,d); Vec2 lateral{-std::sin(pose.heading),std::cos(pose.heading)};
            double t=std::min(1.,d/mergeDistance); double shift=bestOffset*t*t*(3-2*t);
            // Rejoin the road's exit so its precomputed crossing stays valid.
            if(!freeExit) shift*=std::clamp((track.s.back()-d)/(e.length*2+2),0.,1.);
            guide.push_back(pose.center+lateral*shift);
        }
        Vec2 endSide{-std::sin(track.h.back()),std::cos(track.h.back())};
        guide.push_back(e.goal+(freeExit?endSide*bestOffset:Vec2{})); Entity candidate=e;
        if(!installPath(candidate,std::move(guide),std::max(4.,e.length*2))) { e.sublaneStatus="connector outside mask"; continue; }
        bool clear=true; auto previous=e.footprint(); double distance=0;
        candidate.sublaneReservation.clear();
        for(std::size_t i=1;i<candidate.path.size();++i) {
            Footprint pose{candidate.path[i],candidate.pathHeadings[i],e.length,e.width};
            bool traffic=distance<mergeDistance+e.length;
            if(!feasible(previous,pose,candidate,false,false)) { clear=false; e.sublaneStatus="target corridor too narrow"; break; }
            double segment=length(pose.center-previous.center);
            if(traffic) {
                double take=std::min(segment,mergeDistance+e.length-distance),f=take/std::max(1e-9,segment); auto end=pose;
                end.center=previous.center+(pose.center-previous.center)*f;
                end.heading=previous.heading+angleDelta(previous.heading,pose.heading)*f;
                if(!feasible(previous,end,candidate,true,false) || blockedByLight(previous,end,candidate)) { clear=false; e.sublaneStatus="swept connector blocked"; break; }
                PathManeuver piece; piece.begin=i-1; piece.end=i;
                piece.sweep=geometry::swept(previous,end,piece.padding);
                candidate.sublaneReservation.push_back(std::move(piece));
                candidate.sublaneReservationEnd=i; candidate.sublaneReservationProgress=take;
            }
            distance+=segment; previous=pose;
        }
        if(clear) {
            for(const auto& other:entities_) if(other.id!=e.id) {
                double gap=(carFollowing_.enabled?std::max({e.lateralMinGap,other.lateralMinGap,flow_.lateralGap}):std::max(movementGap(e,other),flow_.lateralGap))+.02;
                for(const auto& piece:candidate.sublaneReservation) {
                    if(geometry::violatesClearance(piece.sweep,corners(other.footprint()),gap+piece.padding)) clear=false;
                    if(other.reservesSublaneGap()) for(const auto& occupied:other.sublaneReservation) if(occupied.end>=other.next &&
                        geometry::violatesClearance(piece.sweep,occupied.sweep,gap+piece.padding+occupied.padding)) clear=false;
                    if(!clear) break;
                }
                if(!clear) { e.sublaneStatus="waiting for an uncontested lateral gap"; break; }
            }
        }
        if(clear) {
            candidate.goal=candidate.path.back(); candidate.lateralOffset=currentOffset+bestOffset;
            if(!prepareJunctionPlans(candidate)) continue;
            candidate.sublaneAfter=seconds_+2; e=std::move(candidate);
            e.sublaneReservationRevision=e.routeRevision;
            e.sublaneStatus="moving into reachable side gap";
        }
    }
}
}
