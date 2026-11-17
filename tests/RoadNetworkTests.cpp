#include "traffic/NodeGraph.hpp"
#include <iostream>
#include <stdexcept>
#include <algorithm>
using namespace traffic;
void check(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
int main() {
 try {
    NodeGraph simple; simple.addNode({20,50}); simple.addNode({80,50}); simple.connect(0,1,3.5,8);
    simple.connections[0].roadType=2; simple.connections[0].laneCount=2;
    std::vector<int> selection; check(simple.extendSelection(selection,1) && simple.extendSelection(selection,0),"two-way road could not be selected in reverse");
    auto compiled=compileRoads(simple); check(compiled.lanes.size()==4,"two-way road did not produce two lanes per direction");
    check(compiled.lanes[0].points.front().y>50 && compiled.lanes[2].points.front().y<50,"two-way lanes are not on the right side");
    check(!compiled.lanes[0].entry && !compiled.lanes[2].entry,"road drawing enabled spawning automatically");
    simple.save("road-roundtrip.nodes"); auto restored=NodeGraph::load("road-roundtrip.nodes");
    check(restored.connections[0].roadType==2 && restored.connections[0].laneCount==2,"road direction/count did not persist");
    auto map=Map::fromPng(std::string(TRAFFIC_TEST_ROOT)+"/examples/Wassup5.png",.73);
    auto graph=NodeGraph::load(std::string(TRAFFIC_TEST_ROOT)+"/examples/Wassup5.nodes");
    auto network=compileRoads(graph);
    int bad=0;
    for(std::size_t i=0;i<network.lanes.size();++i) for(auto point:network.lanes[i].points) if(!map.contains(map.pixel(point)) || !map.walkable(map.pixel(point))) {
        if(bad++<8) std::cerr<<"Off-mask lane "<<i<<" origin "<<network.originNodes[i]<<" pixel "<<point.x/.73<<","<<point.y/.73<<'\n';
    }
    check(bad==0,"Wassup5 lane centerlines leave the road mask");
    validateRoadMask(network.lanes,map);
    Simulation sim(map,42); sim.setLanes(network.lanes);
    check(sim.lanes().size()==network.lanes.size(),"compilation changed explicit road/lane indexing");
    check(std::count_if(graph.connections.begin(),graph.connections.end(),[](const auto& r){return r.roadType==2;})==4,"unpaired roads were not two-way");
    std::cout<<"Wassup5 compiled "<<graph.connections.size()<<" roads into "<<network.lanes.size()<<" lanes and internal movements\n";
 } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
