#include "traffic/NodeGraph.hpp"
#include <iostream>
#include <algorithm>
#include <stdexcept>
using namespace traffic;
int main(int argc,char**) {
 try {
    std::cout<<std::unitbuf;
    auto map=Map::fromPng(std::string(TRAFFIC_TEST_ROOT)+"/examples/Wassup5.png",.73);
    auto graph=NodeGraph::load(std::string(TRAFFIC_TEST_ROOT)+"/examples/Wassup5.nodes"); graph.nodes[0].openRadius=160;
    auto roads=compileRoads(graph); validateRoadMask(roads.lanes,map);
    Simulation sim(map,42); sim.setTrafficLights({}); sim.setLanes(roads.lanes); sim.setStuckVehicleSettings({false,30});
    if(argc>1) sim.setTrafficFlowSettings({true,true,.4,.15});
    sim.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{8,0,2,0},1});
    auto urban=sim.urbanTrafficSettings(); urban.enabled=true; sim.setUrbanTrafficSettings(urban); sim.setTrafficRecoverySettings({true,.6});
    int entered=0; bool free=false; double maxStop=0;
    for(int origin:{2,8,9}) {
        auto it=std::find(roads.originNodes.begin(),roads.originNodes.end(),origin);
        if(it==roads.originNodes.end() || !sim.spawnOnLane(static_cast<int>(it-roads.originNodes.begin()),VehicleKind::Car)) throw std::runtime_error("native Wassup5 spawn failed");
        ++entered;
    }
    for(int step=0;step<18000 && !sim.entities().empty();++step) {
        sim.tick(.05);
        if(step%2000==0) std::cout<<"Wassup native time="<<sim.seconds()<<" active="<<sim.entities().size()<<" completed="<<sim.completed()<<'\n';
        for(const auto& e:sim.entities()) {
            free|=e.inOpenJunction; if(e.inOpenJunction) maxStop=std::max(maxStop,e.currentStopSeconds);
            if(!sim.map().fits(e.footprint())) throw std::runtime_error("native Wassup5 vehicle left mask");
            for(const auto& o:sim.entities()) if(e.id<o.id && clearance(e.footprint(),o.footprint())<.5-1e-5) throw std::runtime_error("native Wassup5 collision");
        }
    }
    for(const auto& e:sim.entities()) std::cerr<<"Wassup remaining "<<e.id<<" stage="<<e.networkStage<<" free="<<e.inOpenJunction<<" xy="<<e.center.x<<","<<e.center.y<<" reason="<<stopReasonName(sim.stopReason(e))<<'\n';
    if(!free || sim.completed()!=static_cast<unsigned>(entered) || sim.removedStuckVehicles()!=0) throw std::runtime_error("native Wassup5 traffic failed to drain without deletion");
    std::cout<<"Native Wassup5: "<<entered<<" arrivals; max stationary time inside="<<maxStop<<" seconds\n";
 } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
