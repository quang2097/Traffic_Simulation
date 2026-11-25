#include "traffic/NodeGraph.hpp"
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <stdexcept>
using namespace traffic;
void need(bool value,const char* why){if(!value)throw std::runtime_error(why);}
bool reaches(const std::vector<DirectedLane>& lanes,int a,int b){std::vector<int> todo{a};std::vector<bool> seen(lanes.size());seen[a]=true;for(std::size_t i=0;i<todo.size();++i){if(todo[i]==b)return true;for(int n:lanes[todo[i]].successors)if(!seen[n]){seen[n]=true;todo.push_back(n);}}return false;}
int road(const std::vector<DirectedLane>& lanes,int a,int b){for(std::size_t i=0;i<lanes.size();++i)if(lanes[i].roadGroup>=0 && lanes[i].sourceNode==a && lanes[i].destinationNode==b)return static_cast<int>(i);throw std::runtime_error("missing road");}
int main(){try{
    for(bool prohibited:{false,true}) {
        NodeGraph passing;passing.addNode({10,25});passing.addNode({190,25});passing.connect(0,1,4,10);passing.connections[0].roadType=1;passing.connections[0].laneCount=2;
        for(int x=10;x<=190;++x)passing.connections[0].shape.push_back({static_cast<double>(x),25});
        if(prohibited)passing.connections[0].changeSections={{0,180,-1,-1,false,false}};
        Simulation traffic(Map(200,50,std::vector<Tile>(10000,Tile::Road)),7);traffic.setLanes(passing.lanes());traffic.setTrafficFlowSettings({true,true,.4,.15});traffic.setStuckVehicleSettings({false,30,false,60});
        traffic.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{8,0,2,0},1});
        auto urban=traffic.urbanTrafficSettings();urban.enabled=true;traffic.setUrbanTrafficSettings(urban);
        need(traffic.spawnOnLane(1,VehicleKind::Car),"restriction leader spawn");
        auto& leader=const_cast<std::vector<Entity>&>(traffic.entities())[0];leader.center.x=55;leader.desiredSpeed=leader.speed=2;leader.path={leader.center,leader.goal};leader.pathHeadings={0,0};leader.next=1;leader.segmentProgress=0;leader.laneChangeAfter=1e9;
        need(traffic.spawnOnLane(1,VehicleKind::Car),"restriction follower spawn");
        auto& follower=const_cast<std::vector<Entity>&>(traffic.entities())[1];follower.center.x=35;follower.speed=6;follower.path={follower.center,follower.goal};follower.pathHeadings={0,0};follower.next=1;follower.segmentProgress=0;
        for(int step=0;step<1500;++step)traffic.tick(.01);
        need(prohibited?traffic.totalLaneChanges()==0:traffic.totalLaneChanges()>0,"lane-change executor ignored restriction or blocked an unrestricted pass");
    }
    NodeGraph g;g.addNode({10,40});g.addNode({190,40});g.connect(0,1,3.5,6);g.connections[0].roadType=2;g.connections[0].laneCount=2;
    g.connections[0].changeSections={{40,80,0,1,false,true},{20,40,1,1,false,false}};
    Simulation s(Map(210,90,std::vector<Tile>(18900,Tile::Road)),42);s.setLanes(g.lanes());
    need(!s.laneChangeAllowed(0,1,{60,40},{85,40},4),"restricted target lane accepted entry");
    need(s.laneChangeAllowed(1,0,{60,40},{85,40},4),"one-sided restriction must retain allowed exit");
    need(s.laneChangeAllowed(0,1,{20,40},{30,40},4) && s.laneChangeAllowed(0,1,{110,40},{130,40},4),"unrestricted lengths must allow lane changes");
    need(!s.laneChangeAllowed(0,1,{40,40},{60,40},4),"maneuver spanning section boundary was allowed");
    need(!s.laneChangeAllowed(2,3,{35,40},{45,40},4) && s.laneChangeAllowed(2,3,{140,40},{120,40},4),"reverse section must use authored from-node station");
    s.saveLanes("section-roundtrip.lanes");Simulation loaded(s.map(),7);loaded.loadLanes("section-roundtrip.lanes");
    need(!loaded.laneChangeAllowed(0,1,{60,40},{85,40},4),"lane file lost restrictions");std::remove("section-roundtrip.lanes");
    g.save("section-roundtrip.nodes");auto copy=NodeGraph::load("section-roundtrip.nodes");std::remove("section-roundtrip.nodes");
    need(copy.connections[0].changeSections.size()==2,"node file lost road sections");
    copy.connections[0].changeSections.push_back({90,110,0,0,false,false});
    int middle=copy.addUTurnOpening(0,85,105);need(copy.nodes[middle].allowUTurn && copy.nodes[middle].openRadius==0 && copy.nodes[middle].internalRadius==10,"mid-road opening was not lane guided");
    auto lanes=copy.lanes();int incoming=road(lanes,0,middle),straight=road(lanes,middle,1),opposite=road(lanes,middle,0);
    need(reaches(lanes,incoming,straight) && reaches(lanes,incoming,opposite),"opening lost optional straight or U-turn connection");
    need(copy.connections[0].changeSections.size()==3 && copy.connections[1].changeSections.size()==1,"split lost spanning lane restrictions");
    need(copy.connections[0].changeSections.back().end==95 && copy.connections[1].changeSections[0].begin==0 && copy.connections[1].changeSections[0].end==15,"split did not restation the downstream section");
    need(std::any_of(lanes.begin(),lanes.end(),[&](const DirectedLane& lane){return lane.internalJunction==middle;}),"radius-zero opening lacks internal connectors");
    copy.save("opening-roundtrip.nodes");auto saved=NodeGraph::load("opening-roundtrip.nodes");std::remove("opening-roundtrip.nodes");
    need(saved.nodes[middle].internalRadius==10 && saved.nodes[middle].allowUTurn,"opening extent or U-turn permission lost on save");
    for(int destination:{straight,opposite}) {
        Simulation trip(s.map(),42);trip.setLanes(lanes);trip.setTrafficFlowSettings({true,true,.4,.15});trip.setStuckVehicleSettings({false,30,false,60});
        trip.setProfile(VehicleKind::Car,{{2,0},{1,0},{.3,0},{5,0,2,0},1});
        need(trip.spawnOnLane(incoming,VehicleKind::Car,destination),"opening trip insertion");
        for(int i=0;i<15000 && !trip.entities().empty();++i)trip.tick(.01);
        need(trip.completed()==1 && trip.removedStuckVehicles()==0,"opening straight/U-turn trip did not finish");
    }
    std::cout<<"Directional lengthwise lane restrictions, split/save/load and optional mid-road U-turn/straight routes passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
