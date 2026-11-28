#include "traffic/NodeGraph.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace traffic {
int connectedSignalJunction(const NodeGraph& graph,const TrafficLight& light) {
    int start=light.anchorNode;
    if(start<0 || start>=static_cast<int>(graph.nodes.size())) {
        double nearest=2;
        for(std::size_t n=0;n<graph.nodes.size();++n){double d=length(graph.nodes[n].position-(light.a+light.b)*.5);if(d<nearest){nearest=d;start=static_cast<int>(n);}}
    }
    if(start<0 || start>=static_cast<int>(graph.nodes.size()))return -1;
    std::vector<int> degree(graph.nodes.size(),0);
    for(const auto& road:graph.connections){++degree[road.from];++degree[road.to];}
    auto junction=[&](int n){return degree[n]>=3 || graph.nodes[n].openRadius>0 || graph.nodes[n].internalRadius>0;};
    int shared=graph.junctionAt(start);if(junction(shared))return shared;
    std::vector<int> queue{start};std::vector<bool> visited(graph.nodes.size(),false);visited[start]=true;
    auto forward=light.controlledDirection();
    for(std::size_t at=0;at<queue.size();++at){int n=queue[at];
        if(n!=start && junction(n))return n;
        for(const auto& road:graph.connections){int next=road.from==n?road.to:road.roadType==2 && road.to==n?road.from:-1;
            if(next<0 || visited[next])continue;
            auto delta=graph.nodes[next].position-graph.nodes[n].position;
            if(n==start && delta.x*forward.x+delta.y*forward.y<=1e-9)continue;
            visited[next]=true;queue.push_back(next);
        }
    }
    return -1;
}
Vec2 TrafficLight::controlledDirection() const {
    Vec2 d=outDirection;if(length(d)<1e-9)d={b.y-a.y,a.x-b.x};
    return d*(1/std::max(1e-9,length(d)));
}
bool TrafficLight::controlsDirection(Vec2 travel) const {
    auto d=controlledDirection();return d.x*travel.x+d.y*travel.y>1e-9;
}
bool TrafficLight::controlsMovement(Vec2 from,Vec2 to) const {
    auto d=controlledDirection();
    auto offset=from-(a+b)*.5;
    // A stop line governs the approaching front, never a departing vehicle's
    // rear sweep. This matters when an IN vehicle curves away beside the line.
    return offset.x*d.x+offset.y*d.y<=1e-8 && controlsDirection(to-from);
}
RoadSignalGeometry roadSignalAtNode(const NodeGraph& graph,int road,int node){
    const auto& r=graph.connections.at(road);int direction=r.to==node?0:r.from==node && r.roadType==2?1:-1;
    if(direction<0)throw std::invalid_argument("Select a road arriving at this node");
    auto compiled=compileRoads(graph);Vec2 forward{};int count=0;
    for(const auto& lane:compiled.lanes)if(lane.roadGroup==road && lane.travelDirection==direction){auto d=lane.points.back()-lane.points[lane.points.size()-2];forward=forward+d*(1/std::max(1e-9,length(d)));++count;}
    if(!count)throw std::invalid_argument("This carriageway has no lanes");
    forward=forward*(1/std::max(1e-9,length(forward)));Vec2 side{-forward.y,forward.x},base=graph.nodes.at(node).position;
    double low=1e100,high=-1e100,along=0;
    for(const auto& lane:compiled.lanes)if(lane.roadGroup==road){auto p=(lane.travelDirection==direction?lane.points.back():lane.points.front())-base;double offset=p.x*side.x+p.y*side.y;double width=lane.pointWidths.empty()?lane.width:lane.pointWidths.back();low=std::min(low,offset-width*.5);high=std::max(high,offset+width*.5);if(lane.travelDirection==direction)along+=p.x*forward.x+p.y*forward.y;}
    base=base+forward*(along/count);return {base+side*low,base+side*high,forward};
}
}
