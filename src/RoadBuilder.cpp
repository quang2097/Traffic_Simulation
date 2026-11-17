#include "traffic/NodeGraph.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace traffic {
namespace {
Vec2 unit(Vec2 v) { return v*(1/std::max(1e-9,length(v))); }
std::vector<Vec2> offsetPath(std::vector<Vec2> p,const std::vector<double>& offsets) {
    auto source=p;
    for(std::size_t i=0;i<p.size();++i) {
        auto a=unit(source[i?i:1]-source[i?i-1:0]);
        auto b=i+1<p.size()?unit(source[i+1]-source[i]):a;
        auto n=unit(Vec2{-a.y-b.y,a.x+b.x});
        p[i]=p[i]+n*(offsets[i]/std::max(.5,n.x*(-b.y)+n.y*b.x));
    }
    return p;
}
void trimStart(std::vector<Vec2>& p,double amount) {
    while(p.size()>1 && amount>1e-8) { double d=length(p[1]-p[0]); if(d>amount) { p[0]=p[0]+unit(p[1]-p[0])*amount; return; } amount-=d; p.erase(p.begin()); }
}
}
CompiledRoads compileRoads(const NodeGraph& graph) {
    CompiledRoads result;
    struct Port { int road,from,to,lane; Vec2 in,out; };
    std::vector<Port> ports;
    std::vector<int> degree(graph.nodes.size(),0);
    for(const auto& road:graph.connections) { if(road.from<0 || road.to<0 || road.from>=static_cast<int>(degree.size()) || road.to>=static_cast<int>(degree.size())) throw std::invalid_argument("Invalid road endpoint"); ++degree[road.from]; ++degree[road.to]; }
    for(std::size_t r=0;r<graph.connections.size();++r) {
        const auto& road=graph.connections[r];
        if(!road.shapeWidths.empty() && road.shapeWidths.size()!=(road.shape.empty()?2:road.shape.size())) throw std::invalid_argument("Every shape point needs a width");
        for(auto width:road.shapeWidths) if(!std::isfinite(width) || width<.5 || width>15) throw std::invalid_argument("Invalid shape-point width");
        if(road.laneCount<1 || road.laneCount>4) throw std::invalid_argument("Invalid lane count");
        auto shape=road.shape;
        if(shape.empty()) shape={graph.nodes[road.from].position,graph.nodes[road.to].position};
        shape.front()=graph.nodes[road.from].position; shape.back()=graph.nodes[road.to].position;
        if(shape.size()<2) throw std::invalid_argument("Road shape needs two points");
        double distance=0; for(std::size_t i=1;i<shape.size();++i) { double d=length(shape[i]-shape[i-1]); if(d<.05) throw std::invalid_argument("Road shape has duplicate points"); distance+=d; }
        for(int direction=0;direction<(road.roadType==2?2:1);++direction) {
          int count=direction && road.reverseLaneCount>0?road.reverseLaneCount:road.laneCount;
          const auto& widths=direction?road.reverseWidths:road.forwardWidths;
          double total=0; for(int i=0;i<count;++i) total+=i<static_cast<int>(widths.size())?widths[i]:road.width;
          double previous=0;
          for(int index=0;index<count;++index) {
            double width=index<static_cast<int>(widths.size())?widths[index]:road.width;
            if(!std::isfinite(width) || width<.5 || width>15) throw std::invalid_argument("Invalid individual lane width");
            double offset=road.roadType==2?previous+width*.65:previous+width*.5-total*.5;
            previous+=width;
            std::vector<double> offsets(shape.size()); double covered=0;
            double taper=road.endWidth>0?road.endWidth/road.width:1;
            for(std::size_t i=0;i<shape.size();++i) {
                if(i) covered+=length(shape[i]-shape[i-1]);
                double ratio=road.shapeWidths.size()==shape.size()?road.shapeWidths[i]/road.width:1+(taper-1)*covered/distance;
                offsets[i]=road.lateralOffset+(direction?-offset:offset)*ratio;
            }
            auto fullPath=offsetPath(shape,offsets); std::vector<double> fullWidths(shape.size());
            for(std::size_t i=0;i<shape.size();++i) {
                double ratio=road.shapeWidths.size()==shape.size()?road.shapeWidths[i]/road.width:1;
                if(road.shapeWidths.empty()) { double at=0; for(std::size_t j=1;j<=i;++j) at+=length(shape[j]-shape[j-1]); ratio=1+(taper-1)*at/distance; }
                fullWidths[i]=width*ratio;
            }
            auto p=fullPath;
            int from=direction?road.to:road.from,to=direction?road.from:road.to;
            if(direction) { std::reverse(p.begin(),p.end()); std::reverse(fullPath.begin(),fullPath.end()); std::reverse(fullWidths.begin(),fullWidths.end()); }
            Vec2 in=unit(p[1]-p[0]),out=unit(p.back()-p[p.size()-2]);
            double setback=std::min(distance*.2,std::max(8.0,road.width*road.laneCount*1.5));
            if(degree[from]>1) trimStart(p,std::min(distance*.4,std::max(setback,graph.nodes[from].openRadius)));
            std::reverse(p.begin(),p.end()); if(degree[to]>1) trimStart(p,std::min(distance*.4,std::max(setback,graph.nodes[to].openRadius))); std::reverse(p.begin(),p.end());
            if(p.size()<2) throw std::invalid_argument("Road is too short for its junctions");
            DirectedLane lane; lane.points=std::move(p); lane.width=std::min(width,width*taper); lane.speedLimit=road.speed; lane.turnSmoothing=road.smoothing; lane.inferConnections=false;
            lane.roadGroup=static_cast<int>(r); lane.travelDirection=direction;
            for(auto point:lane.points) {
                double best=1e100,value=width;
                for(std::size_t i=1;i<fullPath.size();++i) {
                    auto d=fullPath[i]-fullPath[i-1]; double t=std::clamp(((point.x-fullPath[i-1].x)*d.x+(point.y-fullPath[i-1].y)*d.y)/std::max(1e-9,d.x*d.x+d.y*d.y),0.,1.);
                    double gap=length(point-(fullPath[i-1]+d*t)); if(gap<best) { best=gap; value=fullWidths[i-1]*(1-t)+fullWidths[i]*t; }
                }
                lane.pointWidths.push_back(value);
            }
            lane.width=*std::min_element(lane.pointWidths.begin(),lane.pointWidths.end());
            lane.destination=graph.nodes[to].destination;
            lane.sourceNode=from; lane.destinationNode=to; lane.excludeOwnDestination=graph.nodes[from].excludeOwnDestination;
            lane.allowTripEnd=!graph.explicitDestinations || lane.destination;
            lane.entry=graph.nodes[from].spawn; lane.spawnRate=graph.nodes[from].rate; lane.turnDeceleration=graph.nodes[to].braking;
            lane.pointBraking.resize(lane.points.size(),lane.turnDeceleration);
            int id=static_cast<int>(result.lanes.size()); result.lanes.push_back(std::move(lane)); result.originNodes.push_back(from);
            ports.push_back({static_cast<int>(r),from,to,id,in,out});
        }
    }
    }
    auto junctionAt=[&](int node) {
        if(graph.nodes[node].openRadius>0) return node;
        int best=node; double distance=1e100;
        for(std::size_t i=0;i<graph.nodes.size();++i) if(graph.nodes[i].openRadius>0) {
            double d=length(graph.nodes[node].position-graph.nodes[i].position);
            if(d<=graph.nodes[i].openRadius && d<distance) { distance=d; best=static_cast<int>(i); }
        }
        return best;
    };
    for(const auto& port:ports) if(graph.nodes[junctionAt(port.to)].openRadius>0 && !graph.nodes[port.to].destination) result.lanes[port.lane].allowTripEnd=false;
    for(const auto& incoming:ports) for(const auto& outgoing:ports) {
        int junction=junctionAt(incoming.to);
        if(junction!=junctionAt(outgoing.from) || incoming.lane==outgoing.lane) continue;
        bool open=graph.nodes[junction].openRadius>0;
        if(!open && (incoming.road==outgoing.road || incoming.out.x*outgoing.in.x+incoming.out.y*outgoing.in.y<-.8)) continue;
        auto start=result.lanes[incoming.lane].points.back(),end=result.lanes[outgoing.lane].points.front();
        if(length(end-start)<.05) { result.lanes[incoming.lane].successors.push_back(outgoing.lane); continue; }
        double handle=length(end-start)*std::clamp(graph.connections[incoming.road].smoothing*.25,.1,.8);
        auto b=start+incoming.out*handle,c=end-outgoing.in*handle;
        double incomingWidth=result.lanes[incoming.lane].pointWidths.back(),outgoingWidth=result.lanes[outgoing.lane].pointWidths.front();
        DirectedLane link; link.width=std::min(incomingWidth,outgoingWidth);
        link.speedLimit=std::min(result.lanes[incoming.lane].speedLimit,result.lanes[outgoing.lane].speedLimit);
        if(open) {
            link.openJunction=junction; link.junctionCenter=graph.nodes[junction].position; link.junctionRadius=graph.nodes[junction].openRadius;
        }
        link.entry=false; link.inferConnections=false; link.allowTripEnd=false; link.turnDeceleration=graph.nodes[incoming.to].braking;
        int samples=std::max(12,static_cast<int>(std::ceil((length(b-start)+length(c-b)+length(end-c))/.8)));
        for(int i=0;i<=samples;++i) {
            double t=static_cast<double>(i)/samples,s=1-t;
            auto point=start*(s*s*s)+b*(3*s*s*t)+c*(3*s*t*t)+end*(t*t*t);
            if(link.points.empty() || length(point-link.points.back())>=.06) link.points.push_back(point);
        }
        if(length(link.points.back()-end)<.06) link.points.pop_back();
        link.points.push_back(end);
        for(std::size_t i=0;i<link.points.size();++i) link.pointWidths.push_back(incomingWidth+(outgoingWidth-incomingWidth)*i/std::max<std::size_t>(1,link.points.size()-1));
        link.successors={outgoing.lane}; result.lanes[incoming.lane].successors.push_back(static_cast<int>(result.lanes.size()));
        result.lanes.push_back(std::move(link)); result.originNodes.push_back(-1);
    }
    for(const auto& incoming:ports) {
        int junction=junctionAt(incoming.to); if(graph.nodes[junction].openRadius<=0) continue;
        for(std::size_t node=0;node<graph.nodes.size();++node) {
            if(!graph.nodes[node].destination || junctionAt(static_cast<int>(node))!=junction) continue;
            bool hasRoad=false; for(const auto& road:graph.connections) if(road.from==static_cast<int>(node) || road.to==static_cast<int>(node)) hasRoad=true;
            if(hasRoad) continue;
            auto start=result.lanes[incoming.lane].points.back(),end=graph.nodes[node].position;
            if(length(end-start)<.05) { result.lanes[incoming.lane].destination=true; result.lanes[incoming.lane].allowTripEnd=true; result.lanes[incoming.lane].destinationNode=static_cast<int>(node); continue; }
            DirectedLane link; link.points={start,end}; link.width=result.lanes[incoming.lane].width; link.entry=false; link.inferConnections=false; link.destinationNode=static_cast<int>(node);
            link.openJunction=junction; link.junctionCenter=graph.nodes[junction].position; link.junctionRadius=graph.nodes[junction].openRadius; link.destination=true;
            result.lanes[incoming.lane].successors.push_back(static_cast<int>(result.lanes.size())); result.lanes.push_back(std::move(link)); result.originNodes.push_back(-1);
        }
    }
    std::vector<TurnControl> controls;
    for(const auto& node:graph.nodes) if(node.turn) controls.push_back({node.position,node.turnRadius,node.braking});
    for(auto& lane:result.lanes) { lane.explicitTurns=true; lane.turnControls=controls; }
    return result;
}
void validateRoadMask(const std::vector<DirectedLane>& lanes,const Map& map) {
    for(std::size_t lane=0;lane<lanes.size();++lane) if(lanes[lane].openJunction<0) for(std::size_t i=1;i<lanes[lane].points.size();++i) {
        auto a=lanes[lane].points[i-1],b=lanes[lane].points[i],d=b-a;
        double distance=length(d); if(distance<1e-9) continue; Vec2 side{-d.y/distance,d.x/distance};
        bool profile=lanes[lane].pointWidths.size()==lanes[lane].points.size();
        double first=profile?lanes[lane].pointWidths[i-1]:lanes[lane].width,last=profile?lanes[lane].pointWidths[i]:lanes[lane].width;
        geometry::Polygon quad{a+side*(first*.5),b+side*(last*.5),b-side*(last*.5),a-side*(first*.5)};
        if(!geometry::fits(map,quad)) throw std::invalid_argument("Lane "+std::to_string(lane)+" leaves the road mask; add shape points or reduce lane count/width");
    }
}
}
