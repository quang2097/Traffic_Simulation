#include "traffic/Simulation.hpp"
#include "traffic/SumoBackend.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <fstream>
#include <iomanip>
#include <queue>
namespace traffic {
void Simulation::setSourceDestinationPolicy(int node,bool exclude) {
    if(node<0) throw std::invalid_argument("Invalid source node");
    for(auto& lane:lanes_) if(lane.sourceNode==node) lane.excludeOwnDestination=exclude;
}
void Simulation::setTurnControls(std::vector<TurnControl> controls) {
    for(const auto& c:controls) if(!std::isfinite(c.point.x) || !std::isfinite(c.point.y) || !std::isfinite(c.radius) || c.radius<.5 || c.radius>500 || !std::isfinite(c.braking) || c.braking<.1 || c.braking>20) throw std::invalid_argument("Invalid turn control");
    for(auto& lane:lanes_) { lane.explicitTurns=true; lane.turnControls=controls; }
}
double Simulation::laneWidthAt(int id,Vec2 point) const {
    const auto& lane=lanes_[id]; double best=1e100,width=lane.width;
    for(std::size_t i=1;i<lane.points.size();++i) {
        auto d=lane.points[i]-lane.points[i-1]; double t=std::clamp((d.x*(point.x-lane.points[i-1].x)+d.y*(point.y-lane.points[i-1].y))/std::max(1e-12,d.x*d.x+d.y*d.y),0.,1.);
        double distance=length(point-(lane.points[i-1]+d*t));
        if(distance<best) { best=distance; if(lane.pointWidths.size()==lane.points.size()) width=lane.pointWidths[i-1]*(1-t)+lane.pointWidths[i]*t; }
    }
    return width;
}
bool Simulation::spawnRandomOnLane(int lane) {
    double weights[vehicleKindCount],total=0;
    for(int i=0;i<vehicleKindCount;++i) total+=(weights[i]=profiles_[i].weight);
    if(total<=0) return false;
    return spawnOnLane(lane,static_cast<VehicleKind>(std::discrete_distribution<int>(weights,weights+vehicleKindCount)(random_)));
}
void Simulation::setPointBraking(Vec2 point,double value) {
    if(!std::isfinite(value) || value<.1 || value>20) throw std::invalid_argument("Invalid point braking");
    for(auto& lane:lanes_) {
        for(auto& control:lane.turnControls) if(length(point-control.point)<.01) control.braking=value;
        lane.pointBraking.resize(lane.points.size(),0);
        for(std::size_t i=0;i<lane.points.size();++i) if(length(point-lane.points[i])<.01) lane.pointBraking[i]=value;
    }
}
double Simulation::laneMergeSpeed(const Entity& e,double requested) const {
    for(int id:e.laneRoute) {
        if(lanes_[id].openJunction>=0 || (flow_.predictiveJunctions && lanes_[id].internalJunction>=0)) continue;
        const auto junction=lanes_[id].points.front();
        auto delta=junction-e.center;
        const double distance=length(delta),ahead=delta.x*std::cos(e.heading)+delta.y*std::sin(e.heading);
        // Leave room for the winning vehicle's rear swing, not just its nose.
        const double hold=e.length*1.5+routeBuffer(e)+e.width+.5;
        if(ahead<=0 || distance>std::max(12.0,e.speed*e.speed/(4*e.acceleration)+hold+5)) continue;
        for(const auto& other:entities_) {
            if(other.id==e.id || other.laneRoute.empty() || other.laneRoute==e.laneRoute ||
               std::find(other.laneRoute.begin(),other.laneRoute.end(),id)==other.laneRoute.end()) continue;
            auto offset=junction-other.center; double d=length(offset);
            double forward=offset.x*std::cos(other.heading)+offset.y*std::sin(other.heading);
            if(forward<-(other.length*.5+movementGap(e,other)+.5) || d>12) continue;
            // Closest approach wins; stable ID breaks near ties. An occupant
            // clearing the junction retains priority until its tail has passed.
            bool priority=forward<=0 || d<distance-.5 || (std::abs(d-distance)<=.5 && other.id<e.id);
            if(priority) requested=std::min(requested,std::sqrt(4*e.acceleration*std::max(0.0,distance-hold)));
        }
    }
    return requested;
}
void Simulation::setLaneTurnDeceleration(int lane,double value) {
    if(lane<0 || lane>=static_cast<int>(lanes_.size()) || !std::isfinite(value) || value<.1 || value>20) throw std::invalid_argument("Turn deceleration must be 0.1-20 m/s^2");
    lanes_[lane].turnDeceleration=value;
}
double Simulation::laneTurnDeceleration(const Entity& e) const {
    double best=1e100,result=2*e.acceleration;
    for(int id:e.laneRoute) {
        const auto& lane=lanes_[id];
        for(std::size_t i=1;i<lane.points.size();++i) {
            auto d=lane.points[i]-lane.points[i-1];
            double t=std::clamp(((e.center.x-lane.points[i-1].x)*d.x+(e.center.y-lane.points[i-1].y)*d.y)/std::max(1e-12,d.x*d.x+d.y*d.y),0.0,1.0);
            double distance=length(e.center-(lane.points[i-1]+d*t));
            if(distance<best) {
                best=distance; result=lane.turnDeceleration;
                auto point=t<.5?i-1:i;
                if(point<lane.pointBraking.size() && lane.pointBraking[point]>0) result=lane.pointBraking[point];
            }
        }
    }
    return result;
}
namespace {
double dot(Vec2 a,Vec2 b) { return a.x*b.x+a.y*b.y; }
double distance(Vec2 p,Vec2 a,Vec2 b) { auto d=b-a; return length(p-(a+d*std::clamp(dot(p-a,d)/std::max(1e-12,dot(d,d)),0.0,1.0))); }
}
void Simulation::setLanes(std::vector<DirectedLane> lanes) {
    if(!entities_.empty()) throw std::invalid_argument("Reset traffic before editing lanes");
    for(auto& lane:lanes) {
        for(const auto& c:lane.turnControls) if(!std::isfinite(c.point.x) || !std::isfinite(c.point.y) || !std::isfinite(c.radius) || c.radius<.5 || c.radius>500 || !std::isfinite(c.braking) || c.braking<.1 || c.braking>20) throw std::invalid_argument("Invalid turn control");
        if(lane.pointWidths.empty()) lane.pointWidths.resize(lane.points.size(),lane.width);
        if(lane.pointWidths.size()!=lane.points.size()) throw std::invalid_argument("Invalid lane width profile");
        for(double width:lane.pointWidths) if(!std::isfinite(width) || width<.5 || width>15) throw std::invalid_argument("Invalid tapered lane width");
        if(!std::isfinite(lane.junctionRadius) || lane.junctionRadius<0 || lane.junctionRadius>500 || ((lane.openJunction>=0 || lane.internalJunction>=0) && lane.junctionRadius<=0)) throw std::invalid_argument("Invalid junction geometry");
        for(const auto& rule:lane.changeSections) if(!std::isfinite(rule.begin) || !std::isfinite(rule.end) || rule.begin<0 || rule.end<rule.begin) throw std::invalid_argument("Invalid lane change section");
        if(!std::isfinite(lane.turnSmoothing) || lane.turnSmoothing<.2 || lane.turnSmoothing>4) throw std::invalid_argument("Invalid turn smoothing");
        if(!std::isfinite(lane.spawnRate) || lane.spawnRate<0 || lane.spawnRate>100) throw std::invalid_argument("Invalid lane spawn rate");
        if(lane.sourceNode< -1 || lane.destinationNode< -1 || (lane.excludeOwnDestination && lane.sourceNode<0)) throw std::invalid_argument("Invalid origin destination policy");
        if(lane.pointBraking.empty()) lane.pointBraking.resize(lane.points.size(),0);
        if(lane.pointBraking.size()!=lane.points.size()) throw std::invalid_argument("Invalid point settings");
        for(auto v:lane.pointBraking) if(!std::isfinite(v) || v<0 || v>20 || (v>0 && v<.1)) throw std::invalid_argument("Invalid point braking");
        if(!std::isfinite(lane.turnDeceleration) || lane.turnDeceleration<.1 || lane.turnDeceleration>20) throw std::invalid_argument("Invalid turn deceleration");
        if(lane.points.size()<2 || !std::isfinite(lane.width) || lane.width<.5 || lane.width>15 || !std::isfinite(lane.speedLimit) || lane.speedLimit<.1 || lane.speedLimit>50) throw std::invalid_argument("Invalid directed lane");
        for(std::size_t i=0;i<lane.points.size();++i) {
            auto p=lane.points[i]; if(!std::isfinite(p.x) || !std::isfinite(p.y) || p.x<0 || p.y<0 || p.x>=map_.width*map_.metersPerPixel || p.y>=map_.height*map_.metersPerPixel || (lane.openJunction<0 && !map_.walkable(map_.pixel(p))) || (i && length(p-lane.points[i-1])<.05)) throw std::invalid_argument("Lane points must lie on roads and be distinct");
        }
        for(int to:lane.successors) if(to<0 || to>=static_cast<int>(lanes.size()) || lanes[to].points.empty() || length(lane.points.back()-lanes[to].points.front())>1) throw std::invalid_argument("Lane connection endpoints must meet");
    }
    // Split at shared drawn vertices so a branch can leave or join mid-lane.
    std::vector<DirectedLane> split;
    std::vector<int> first(lanes.size()),last(lanes.size());
    for(std::size_t k=0;k<lanes.size();++k) {
        first[k]=static_cast<int>(split.size()); std::size_t start=0;
        for(std::size_t i=1;i<lanes[k].points.size();++i) {
            bool junction=i+1==lanes[k].points.size();
            for(std::size_t j=0;lanes[k].inferConnections && j<lanes.size() && !junction;++j) if(j!=k)
                for(auto p:lanes[j].points) if(length(p-lanes[k].points[i])<.01) { junction=true; break; }
            if(!junction) continue;
            auto segment=lanes[k]; segment.points.assign(lanes[k].points.begin()+start,lanes[k].points.begin()+i+1);
            segment.pointBraking.assign(lanes[k].pointBraking.begin()+start,lanes[k].pointBraking.begin()+i+1);
            segment.pointWidths.assign(lanes[k].pointWidths.begin()+start,lanes[k].pointWidths.begin()+i+1);
            segment.entry=start==0 && lanes[k].entry; segment.successors.clear();
            if(i+1<lanes[k].points.size()) { segment.destination=false; segment.allowTripEnd=false; segment.destinationNode=-1; }
            if(start) split.back().successors.push_back(static_cast<int>(split.size()));
            split.push_back(std::move(segment)); start=i;
        }
        last[k]=static_cast<int>(split.size())-1;
    }
    for(std::size_t k=0;k<lanes.size();++k) for(int to:lanes[k].successors) split[last[k]].successors.push_back(first[to]);
    lanes=std::move(split);
    // Endpoint connections are directed; never infer an immediate U-turn.
    for(std::size_t i=0;i<lanes.size();++i) if(lanes[i].inferConnections) for(std::size_t j=0;j<lanes.size();++j) if(i!=j && length(lanes[i].points.back()-lanes[j].points.front())<.5) {
        auto a=lanes[i].points.back()-lanes[i].points[lanes[i].points.size()-2],b=lanes[j].points[1]-lanes[j].points[0];
        if(dot(a,b)>-.5*length(a)*length(b) && std::find(lanes[i].successors.begin(),lanes[i].successors.end(),static_cast<int>(j))==lanes[i].successors.end()) lanes[i].successors.push_back(static_cast<int>(j));
    }
    lanes_=std::move(lanes);
}
void Simulation::saveLanes(const std::string& file) const {
    std::ofstream out(file); out<<std::setprecision(17)<<"TRAFFIC_LANES 9 "<<lanes_.size()<<'\n';
    for(const auto& lane:lanes_) {
        out<<lane.width<<' '<<lane.speedLimit<<' '<<lane.entry<<' '<<lane.points.size()<<' '<<lane.successors.size()<<'\n';
        out<<lane.turnDeceleration<<' '<<lane.spawnRate<<' '<<lane.turnSmoothing<<' '<<lane.inferConnections<<'\n';
        out<<lane.roadGroup<<' '<<lane.travelDirection<<' '<<lane.openJunction<<' '<<lane.junctionCenter.x<<' '<<lane.junctionCenter.y<<' '<<lane.junctionRadius<<'\n';
        out<<lane.allowTripEnd<<' '<<lane.destination<<'\n';
        out<<lane.explicitTurns<<' '<<lane.turnControls.size()<<'\n';
        for(const auto& c:lane.turnControls) out<<c.point.x<<' '<<c.point.y<<' '<<c.radius<<' '<<c.braking<<'\n';
        out<<lane.sourceNode<<' '<<lane.destinationNode<<' '<<lane.excludeOwnDestination<<'\n';
        out<<lane.internalJunction<<' '<<lane.changeSections.size()<<'\n';
        for(const auto& rule:lane.changeSections) out<<rule.begin<<' '<<rule.end<<' '<<rule.allowEnter<<' '<<rule.allowExit<<'\n';
        for(std::size_t i=0;i<lane.points.size();++i) out<<lane.points[i].x<<' '<<lane.points[i].y<<' '<<lane.pointBraking[i]<<' '<<lane.pointWidths[i]<<'\n';
        for(auto id:lane.successors) out<<id<<' '; out<<'\n';
    }
    if(!out) throw std::runtime_error("Cannot save lane network");
}
void Simulation::loadLanes(const std::string& file) {
    std::ifstream in(file); std::string magic; int version; std::size_t count;
    if(!(in>>magic>>version>>count) || magic!="TRAFFIC_LANES" || version<1 || version>9 || count>4096) throw std::runtime_error("Invalid lane network file");
    std::vector<DirectedLane> lanes(count); std::size_t total=0;
    for(auto& lane:lanes) {
        std::size_t points,links;
        if(!(in>>lane.width>>lane.speedLimit>>lane.entry>>points>>links) || points<2 || points>65536 || links>count || (total+=points)>200000) throw std::runtime_error("Invalid lane network data");
        if(version>=2 && !(in>>lane.turnDeceleration)) throw std::runtime_error("Invalid lane turn deceleration");
        if(version>=3 && !(in>>lane.spawnRate)) throw std::runtime_error("Invalid spawn rate");
        if(version>=4 && !(in>>lane.turnSmoothing>>lane.inferConnections)) throw std::runtime_error("Invalid lane smoothing");
        if(version>=5 && !(in>>lane.roadGroup>>lane.travelDirection>>lane.openJunction>>lane.junctionCenter.x>>lane.junctionCenter.y>>lane.junctionRadius)) throw std::runtime_error("Invalid hybrid lane metadata");
        if(version>=6 && !(in>>lane.allowTripEnd>>lane.destination)) throw std::runtime_error("Invalid trip destination metadata");
        if(version>=7) {
            std::size_t controls; if(!(in>>lane.explicitTurns>>controls) || controls>200000) throw std::runtime_error("Invalid turn-control count");
            lane.turnControls.resize(controls); for(auto& c:lane.turnControls) if(!(in>>c.point.x>>c.point.y>>c.radius>>c.braking)) throw std::runtime_error("Invalid turn control");
        }
        if(version>=8 && !(in>>lane.sourceNode>>lane.destinationNode>>lane.excludeOwnDestination)) throw std::runtime_error("Invalid origin destination metadata");
        if(version>=9) {
            std::size_t sections;
            if(!(in>>lane.internalJunction>>sections) || lane.internalJunction< -1 || sections>10000) throw std::runtime_error("Invalid internal junction metadata");
            lane.changeSections.resize(sections);
            for(auto& rule:lane.changeSections) if(!(in>>rule.begin>>rule.end>>rule.allowEnter>>rule.allowExit)) throw std::runtime_error("Invalid lane change section");
        }
        lane.points.resize(points); lane.successors.resize(links);
        lane.pointBraking.resize(points,0);
        lane.pointWidths.resize(points,lane.width);
        for(std::size_t i=0;i<points;++i) {
            auto& p=lane.points[i];
            if(!(in>>p.x>>p.y) || (version>=3 && !(in>>lane.pointBraking[i]))) throw std::runtime_error("Invalid lane point");
            if(version>=5 && !(in>>lane.pointWidths[i])) throw std::runtime_error("Invalid lane width sample");
        }
        for(auto& id:lane.successors) if(!(in>>id)) throw std::runtime_error("Invalid lane connection");
    }
    setLanes(std::move(lanes));
}
bool Simulation::fitsLaneRoute(const Footprint& f,const Entity& e) const {
    if(e.internalJunction>=0 && e.networkStage<e.networkRoute.size()) {
        const auto& link=lanes_[e.networkRoute[e.networkStage]];
        bool inside=true;
        for(auto p:corners(f)) if(length(p-link.junctionCenter)>link.junctionRadius+e.length*.5) {inside=false;break;}
        if(inside) return true;
    }
    if(e.changingLanes && e.changingFrom>=0 && e.changingTo>=0) {
        Vec2 axis=e.laneChangeDirection,side{-axis.y,axis.x}; double low=1e100,high=-1e100;
        for(int id:{e.changingFrom,e.changingTo}) {
            const auto& lane=lanes_[id]; double best=1e100; Vec2 nearest;
            for(std::size_t i=1;i<lane.points.size();++i) { auto d=lane.points[i]-lane.points[i-1]; double t=std::clamp(((f.center.x-lane.points[i-1].x)*d.x+(f.center.y-lane.points[i-1].y)*d.y)/std::max(1e-9,d.x*d.x+d.y*d.y),0.,1.); auto q=lane.points[i-1]+d*t; if(length(q-f.center)<best) {best=length(q-f.center);nearest=q;} }
            double across=nearest.x*side.x+nearest.y*side.y,width=laneWidthAt(id,nearest);
            low=std::min(low,across-width*.5); high=std::max(high,across+width*.5);
        }
        for(auto point:corners(f)) {double across=point.x*side.x+point.y*side.y; if(across<low-1e-7 || across>high+1e-7) return false;}
        return true;
    }
    if(e.inOpenJunction) {
        for(auto p:corners(f)) {
            bool inside=length(p-e.junctionCenter)<=e.junctionRadius+e.length+2;
            if(!inside && e.networkStage+1<e.networkRoute.size()) {
                const auto& exit=lanes_[e.networkRoute[e.networkStage+1]];
                for(std::size_t i=1;i<exit.points.size();++i) {
                    double funnel=length(p-exit.points.front())<e.length*3+5?e.length:0;
                    if(distance(p,exit.points[i-1],exit.points[i])<=laneWidthAt(e.networkRoute[e.networkStage+1],p)*.5+funnel) { inside=true; break; }
                }
            }
            if(!inside) return false;
        }
        return true;
    }
    if(e.laneRoute.empty()) return true;
    for(auto p:corners(f)) {
        bool inside=false;
        for(int id:e.laneRoute) {
            const auto& lane=lanes_[id];
            if(lane.openJunction>=0 && length(p-lane.junctionCenter)<=lane.junctionRadius+e.length+2) inside=true;
            for(std::size_t i=1;!inside && i<lane.points.size();++i) {
                auto d=lane.points[i]-lane.points[i-1]; double t=std::clamp((d.x*(p.x-lane.points[i-1].x)+d.y*(p.y-lane.points[i-1].y))/std::max(1e-12,d.x*d.x+d.y*d.y),0.,1.);
                double width=lane.pointWidths.size()==lane.points.size()?lane.pointWidths[i-1]*(1-t)+lane.pointWidths[i]*t:lane.width;
                // A rigid body sweeps outside a curved junction centreline.
                // This allowance belongs to internal links, never road lanes.
                if(!e.networkRoute.empty() && lane.roadGroup<0 && !lane.entry && lane.openJunction<0) width+=e.length;
                // Nominal lane edges are not physical walls. Half a mask
                // pixel (at most 5 cm) bridges sampling scallops on curved,
                // tapered roads; the mask, dividers and pair gaps stay hard.
                double edgeTolerance=flow_.predictiveJunctions && !e.networkRoute.empty() && lane.roadGroup>=0?
                    std::min(.05,map_.metersPerPixel*.5):0.;
                if(length(p-(lane.points[i-1]+d*t))<=width*.5+edgeTolerance+1e-7) inside=true;
            }
            if(inside) break;
        }
        if(!inside) return false;
    }
    return true;
}
bool Simulation::spawnOnLane(int id,VehicleKind kind,int destinationLane) {
#ifdef _WIN32
    if(sumoBackend_) return sumoBackend_->spawn(id,kind,destinationLane);
#endif
    if(id<0 || id>=static_cast<int>(lanes_.size())) return false;
    for(const auto& other:entities_) if(other.onJunction() && other.networkStage+1<other.networkRoute.size() && other.networkRoute[other.networkStage+1]==id) return false;
    const auto& p=profile(kind); Entity e; e.id=nextId_; e.kind=kind;
    e.spawnLane=id; e.spawnNode=lanes_[id].sourceNode;
    e.length=sample(p.length,.2,30); e.width=sample(p.width,.15,5); e.preferredDistance=sample(p.distance,0,20); e.massKg=defaultVehicleMass(kind); sampleMotion(e);
    if(lanes_[id].width<e.width+.05) return false;
    std::vector<double> costs(lanes_.size(),1e100); std::vector<int> parent(lanes_.size(),-1);
    using Item=std::pair<double,int>; std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
    costs[id]=0; queue.push({0,id}); int sink=-1; std::vector<int> exits;
    while(!queue.empty()) {
        auto [cost,node]=queue.top(); queue.pop(); if(cost!=costs[node]) continue;
        bool returnsToSource=lanes_[id].excludeOwnDestination && lanes_[id].sourceNode>=0 && lanes_[node].destinationNode==lanes_[id].sourceNode;
        if(!returnsToSource && ((destinationLane>=0 && node==destinationLane) || (destinationLane<0 && (lanes_[node].destination || (lanes_[node].successors.empty() && lanes_[node].allowTripEnd))))) exits.push_back(node);
        if(lanes_[node].successors.empty()) continue;
        for(int next:lanes_[node].successors) {
            if(lanes_[next].width<e.width+.05) continue;
            double distance=0; const auto& lane=lanes_[next]; for(std::size_t i=1;i<lane.points.size();++i) distance+=length(lane.points[i]-lane.points[i-1]);
            // Destinations remain random; the route to the selected destination is shortest by distance.
            double candidate=cost+distance;
            if(candidate<costs[next]) { costs[next]=candidate; parent[next]=node; queue.push({candidate,next}); }
        }
    }
    if(exits.empty()) return false;
    sink=exits[std::uniform_int_distribution<std::size_t>(0,exits.size()-1)(random_)];
    for(int node=sink;node>=0;node=parent[node]) e.laneRoute.push_back(node);
    std::reverse(e.laneRoute.begin(),e.laneRoute.end()); std::vector<Vec2> guide;
    if(std::any_of(e.laneRoute.begin(),e.laneRoute.end(),[&](int lane){return lanes_[lane].openJunction>=0 || lanes_[lane].roadGroup>=0;})) {
        e.networkRoute=e.laneRoute;
        const auto& start=lanes_[id].points; auto direction=start[1]-start[0]; direction=direction*(1/std::max(1e-9,length(direction)));
        e.center=start.front()+direction*(e.length*.5+.2); e.heading=std::atan2(direction.y,direction.x); e.progressAnchor=e.center;
        if(flow_.sublanes) {
            Vec2 middle=e.center,side{-direction.y,direction.x};
            double limit=(laneWidthAt(id,e.center)-e.width)*.5-.08;
            auto insertable=[&]() {
                if(!feasible(e.footprint(),e.footprint(),e,true,false)) return false;
                // Exact preferred-gap contact leaves no room for the tiny
                // heading changes of a continuous lane/junction handoff.
                for(const auto& other:entities_) if(clearance(e.footprint(),other.footprint())<(carFollowing_.enabled?std::max({e.lateralMinGap,other.lateralMinGap,flow_.lateralGap}):std::max(movementGap(e,other),flow_.lateralGap))+.02) return false;
                return true;
            };
            bool placed=insertable();
            for(double offset=flow_.lateralResolution;!placed && offset<=limit+.01;offset+=flow_.lateralResolution) for(double sign:{-1.,1.}) {
                e.center=middle+side*(offset*sign); e.lateralOffset=offset*sign;
                if(insertable()) { placed=true; break; }
            }
            if(!placed) return false;
        }
        if(!installNetworkStage(e,0) || !feasible(e.footprint(),e.footprint(),e,true)) return false;
        if(!prepareJunctionPlans(e)) return false;
        auto previous=e.footprint();
        for(std::size_t i=1;i<e.path.size();++i) { Footprint pose{e.path[i],e.pathHeadings[i],e.length,e.width}; if(!feasible(previous,pose,e,false)) return false; previous=pose; }
        entities_.push_back(std::move(e)); ++nextId_; return true;
    }
    for(int current:e.laneRoute) {
        const auto& lane=lanes_[current]; e.desiredSpeed=std::min(e.desiredSpeed,lane.speedLimit);
        for(auto point:lane.points) if(guide.empty() || length(point-guide.back())>.01) guide.push_back(point);
    }
    auto trim=[&](double amount) {
        while(guide.size()>1) { double d=length(guide[1]-guide[0]); if(d>amount) { guide[0]=guide[0]+(guide[1]-guide[0])*(amount/d); return true; } amount-=d; guide.erase(guide.begin()); }
        return false;
    };
    double margin=e.length*.5+.2;
    if(!trim(margin)) return false;
    std::reverse(guide.begin(),guide.end()); if(!trim(margin)) return false; std::reverse(guide.begin(),guide.end());
    e.center=guide.front(); e.goal=guide.back(); e.progressAnchor=e.center;
    e.heading=std::atan2(guide[1].y-e.center.y,guide[1].x-e.center.x);
    for(const auto& other:entities_) for(int a:e.laneRoute) for(int b:other.laneRoute) if(a!=b) {
        const auto& left=lanes_[a]; const auto& right=lanes_[b];
        for(std::size_t i=1;i<left.points.size();++i) for(std::size_t j=1;j<right.points.size();++j) {
            auto u=left.points[i]-left.points[i-1],v=right.points[j]-right.points[j-1];
            if(dot(u,v)>-.8*length(u)*length(v)) continue;
            double required=.5*(e.width+other.width)+movementGap(e,other)+.05;
            if(geometry::distance({left.points[i-1],left.points[i]},{right.points[j-1],right.points[j]})<required) return false;
        }
    }
    if(!feasible(e.footprint(),e.footprint(),e,true)) return false;
    bool gentle=true;
    for(std::size_t i=1;i+1<guide.size();++i) {
        auto a=guide[i]-guide[i-1],b=guide[i+1]-guide[i];
        if(std::abs(angleDelta(std::atan2(a.y,a.x),std::atan2(b.y,b.x)))>.15) gentle=false;
    }
    if(gentle) {
        // Smooth centerline tangents avoid a tiny, tight circular fillet at
        // every sample of an otherwise gently curving road.
        e.path=guide;
        for(std::size_t i=0;i<guide.size();++i) {
            auto a=i?guide[i]-guide[i-1]:guide[1]-guide[0];
            auto b=i+1<guide.size()?guide[i+1]-guide[i]:a;
            double h=std::atan2(a.y,a.x),k=std::atan2(b.y,b.x);
            e.pathHeadings.push_back(i?h+angleDelta(h,k)*.5:e.heading);
        }
        e.next=1; ++e.routeRevision;
        auto previous=e.footprint();
        for(std::size_t i=1;i<e.path.size();++i) {
            Footprint pose{e.path[i],e.pathHeadings[i],e.length,e.width};
            if(!feasible(previous,pose,e,false)) return false; previous=pose;
        }
    } else if(!installPath(e,std::move(guide))) return false;
    for(std::size_t i=0;i<e.path.size();++i) if(!fitsLaneRoute({e.path[i],e.pathHeadings[i],e.length,e.width},e)) return false;
    e.laneEnvelope.push_back(e.footprint());
    for(std::size_t i=1;i<e.path.size();++i) {
        int steps=std::max(1,static_cast<int>(std::ceil(length(e.path[i]-e.path[i-1]))));
        for(int step=1;step<=steps;++step) {
            double t=static_cast<double>(step)/steps;
            e.laneEnvelope.push_back({e.path[i-1]+(e.path[i]-e.path[i-1])*t,e.pathHeadings[i-1]+angleDelta(e.pathHeadings[i-1],e.pathHeadings[i])*t,e.length,e.width});
        }
    }
    // A pair wider than the available passing clearance would otherwise meet
    // nose-to-nose and wait forever. Retain the full envelope across red queues.
    for(const auto& other:entities_) for(const auto& a:e.laneEnvelope) for(const auto& b:other.laneEnvelope) {
        if(std::cos(angleDelta(a.heading,b.heading))>-.5) continue;
        if(length(a.center-b.center)>.5*(std::hypot(e.length,e.width)+std::hypot(other.length,other.width))+movementGap(e,other)+.15) continue;
        if(geometry::violatesClearance(corners(a),corners(b),movementGap(e,other)+.15)) return false;
    }
    // Start at rest, already tangent to the lane. A blocked insertion is deferred.
    entities_.push_back(std::move(e)); ++nextId_; return true;
}
}
