#include "traffic/NodeGraph.hpp"
#include "traffic/Demo.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <stdexcept>
using namespace traffic;
void need(bool b,const char* m) { if(!b) throw std::runtime_error(m); }
void safe(const Simulation& s) {
    for(const auto& e:s.entities()) {
        need(s.map().fits(e.footprint()),"body left mask");
        for(const auto& o:s.entities()) if(e.id<o.id) need(!geometry::violatesClearance(corners(e.footprint()),corners(o.footprint()),0),"packing caused physical body overlap");
    }
}
int main() {
 try {
    NodeGraph g; g.addNode({10,20}); g.addNode({90,20}); g.connect(0,1,3.5,7); g.connections[0].roadType=1;
    Simulation s(Map(100,40,std::vector<Tile>(4000,Tile::Road)),42); s.setLanes(g.lanes()); s.setTrafficFlowSettings({true,true,.4,.15}); s.setStuckVehicleSettings({false,30});
    s.setProfile(VehicleKind::Motorbike,{{2,0},{.7,0},{.35,0},{7,0,2,0},1});
    for(int i=0;i<3;++i) need(s.spawnOnLane(0,VehicleKind::Motorbike),"three bikes must share a sufficiently wide lane entrance");
    safe(s); need(!s.spawnOnLane(0,VehicleKind::Motorbike),"a blocked entrance must not teleport another bike through occupants");
    for(int i=0;i<4000 && !s.entities().empty();++i) { s.tick(.01); safe(s); }
    need(s.completed()==3 && s.removedStuckVehicles()==0,"packed bikes must reach the terminal without forced centering or deletion");
    NodeGraph passing; passing.addNode({10,25}); passing.addNode({190,25}); passing.connect(0,1,4.6,8); passing.connections[0].roadType=1;
    Simulation squeeze(Map(200,50,std::vector<Tile>(10000,Tile::Road)),42); squeeze.setLanes(passing.lanes()); squeeze.setTrafficFlowSettings({true,true,.4,.15}); squeeze.setStuckVehicleSettings({false,30});
    squeeze.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.35,0},{2,0,2,0},1}); squeeze.setProfile(VehicleKind::Motorbike,{{2,0},{.7,0},{.35,0},{7,0,2,0},1});
    need(squeeze.spawnOnLane(0,VehicleKind::Car),"sublane leader spawn"); squeeze.tick(10);
    need(squeeze.spawnOnLane(0,VehicleKind::Motorbike),"sublane follower spawn");
    auto bike=squeeze.entities().back().id; bool passed=false;
    for(int step=0;step<5000 && !passed;++step) {
        squeeze.tick(.01); safe(squeeze);
        for(const auto& e:squeeze.entities()) if(e.id==bike) passed|=e.center.x>squeeze.entities().front().center.x+4;
    }
    if(!passed) for(const auto& e:squeeze.entities()) std::cerr<<"squeeze "<<e.id<<" xy="<<e.center.x<<','<<e.center.y<<" speed="<<e.speed<<" offset="<<e.lateralOffset<<" reason="<<e.sublaneStatus<<'\n';
    need(passed,"motorbike must continuously reach and use a side gap to pass within one lane");

    Simulation gates(Map(100,40,std::vector<Tile>(4000,Tile::Road)),42); gates.setLanes(g.lanes());
    TrafficLight a{{30,15},{30,25},.1,0,100,1,true}; a.permissive=true; a.allowedA={60,17}; a.allowedB={60,23};
    TrafficLight b{{20,5},{20,10},4,1,6,2,true}; b.permissive=true; b.allowedA={80,5}; b.allowedB={80,10};
    gates.setTrafficLights({a,b}); auto idA=gates.trafficLights()[0].id,idB=gates.trafficLights()[1].id;
    Simulation coincident(gates.map(),42); coincident.setTrafficLights({a,a});
    need(coincident.trafficLights()[0].id!=coincident.trafficLights()[1].id,"coincident stop lines must still retain independent links");
    Simulation parallel(gates.map(),42); parallel.setLanes(g.lanes()); auto along=a; along.allowedA={60,20}; along.allowedB={80,20}; parallel.setTrafficLights({along});
    need(parallel.spawnOnLane(0,VehicleKind::Car),"permission geometry must not restrict reachable trip destinations");
    need(!parallel.routePermitsRed(parallel.entities()[0],parallel.trafficLights()[0]),"driving along a linked line must not grant crossing permission");
    need(idA && idB && idA!=idB,"independent lights require distinct identities");
    gates.saveTrafficLights("flow-roundtrip.signals");
    Simulation loaded(gates.map(),42); loaded.setLanes(g.lanes()); loaded.loadTrafficLights("flow-roundtrip.signals");
    need(loaded.trafficLights().size()==2 && loaded.trafficLights()[0].id==idA && loaded.trafficLights()[1].id==idB && loaded.trafficLights()[1].allowedA.x==80,"saved exits must remain attached to their own light");
    loaded.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{7,0,2,0},1});
    auto reordered=loaded.trafficLights(); std::reverse(reordered.begin(),reordered.end()); loaded.setTrafficLights(reordered);
    need(loaded.spawnOnLane(0,VehicleKind::Car),"linked permission must retain the lane destination");
    auto tripGoal=loaded.entities()[0].goal; auto tripId=loaded.entities()[0].id;
    auto lights=loaded.trafficLights(); lights.erase(lights.begin()); loaded.setTrafficLights(lights);
    need(loaded.trafficLights()[0].id==idA,"removing another light must preserve permission association");
    auto light=loaded.trafficLights()[0]; light.allowedA.x=65; light.allowedB.x=65; loaded.setTrafficLights({light});
    need(length(loaded.entities()[0].goal-tripGoal)<1e-9,"editing a permission must not change a trip destination");
    need(loaded.routePermitsRed(loaded.entities()[0],light),"linked route must permit passing red");
    for(int i=0;i<1100;++i) { loaded.tick(.01); safe(loaded); }
    need(loaded.completed()==0 && loaded.entities().size()==1 && loaded.entities()[0].id==tripId && loaded.entities()[0].center.x>70,"vehicle must survive crossing its allowed exit during red");
    for(int i=0;i<1000 && !loaded.entities().empty();++i) loaded.tick(.01);
    need(loaded.completed()==1,"trip must finish only at its actual lane destination");
    {
        std::ofstream old("flow-v1.signals");
        old<<"TRAFFIC_SIGNALS 1 1\n77 30 15 30 25 .1 0 100 1 1 1 60 17 60 23 1\n";
    }
    loaded.loadTrafficLights("flow-v1.signals");
    need(loaded.trafficLights()[0].id==77 && loaded.trafficLights()[0].allowedA.x==60,"version 1 must retain the correct permission link");
    need(loaded.spawnOnLane(0,VehicleKind::Car),"version 1 destination flag must not restrict spawning");
    for(int i=0;i<1100;++i) loaded.tick(.01);
    need(loaded.entities().size()==1 && loaded.entities()[0].center.x>70 && loaded.completed()==1,"obsolete saved destination flag must never cause early completion");
    auto demoGraph=makeJunctionDemoGraph(); auto roads=compileRoads(demoGraph); auto demoMap=makeJunctionDemoMap(); validateRoadMask(roads.lanes,demoMap);
    Simulation demo(demoMap,42); demo.setLanes(roads.lanes); demo.setTrafficFlowSettings({true,true,.4,.15}); demo.setStuckVehicleSettings({false,30});
    demo.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{7,0,2,0},1});
    demo.setProfile(VehicleKind::Motorbike,{{2,0},{.7,0},{.35,0},{7,0,2,0},1});
    int admitted=0;
    for(int step=0;step<1000;++step) {
        if(step<160 && step%40==0) for(int node=1;node<=4;++node) {
            auto lane=std::find(roads.originNodes.begin(),roads.originNodes.end(),node);
            admitted+=demo.spawnOnLane(static_cast<int>(lane-roads.originNodes.begin()),step%80==0?VehicleKind::Car:VehicleKind::Motorbike);
        }
        demo.tick(.1); safe(demo);
    }
    need(admitted>=12 && demo.completed()==static_cast<unsigned>(admitted),"native startup demo must drain mixed traffic including randomly selected U-turns");
    std::cout<<"Continuous packing, permission-only exits, true trip completion and signal migration passed\n";
 } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
