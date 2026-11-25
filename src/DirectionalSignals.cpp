#include "traffic/NodeGraph.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace traffic {
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
