#include "traffic/Simulation.hpp"
#include "traffic/SimulationResults.hpp"
#include "traffic/NodeGraph.hpp"
#include "traffic/SumoBackend.hpp"
#include <iostream>
#include <stdexcept>
using namespace traffic;
void need(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
void nativeTests(){
    Simulation s(Map(100,60,std::vector<Tile>(6000,Tile::Road)),42);s.setCollisionWorkers(1);
    s.setProfile(VehicleKind::Car,{{2,0},{1,0},{.5,0},{3,0,3,0},1});
    s.setStuckVehicleSettings({false,30,false,60});s.setTrafficRecoverySettings({false,1,25});
    auto routing=s.routingSettings();routing.rtsStyle=routing.spreadAtSignals=routing.refreshMovingRoutes=false;s.setRoutingSettings(routing);
    need(s.spawnAt({25,20},{60,20},VehicleKind::Car,0),"Leader spawn failed");auto first=s.entities().back().id;
    const_cast<std::vector<Entity>&>(s.entities()).back().desiredSpeed=0;
    need(s.spawnAt({18,20},{60,20},VehicleKind::Car,0),"Follower spawn failed");auto follower=s.entities().back().id;
    s.tick(5);auto before=s.stopStatistics();auto time=s.seconds();
    need(!s.deleteVehicle(999999) && !s.completed(),"Unknown ID changed accounting");
    need(s.deleteVehicle(first),"Selected leader was not deleted");
    need(s.seconds()==time && s.entities().size()==1 && s.entities().front().id==follower,"Manual deletion changed time or another vehicle");
    need(s.completed()==1 && s.manuallyCompleted()==1 && !s.completedOnGreen() && !s.removedStuckVehicles(),"Manual deletion was not counted as exactly one completion");
    auto after=s.stopStatistics();need(after.vehicles==before.vehicles && after.totalSeconds==before.totalSeconds,"Manual deletion lost stopped-time statistics");
    need(!s.deleteVehicle(first) && s.completed()==1,"Repeated deletion double-counted a completed trip");
    for(int i=0;i<200 && !s.entities().empty();++i)s.tick(.1);
    need(s.completed()==2 && s.manuallyCompleted()==1 && s.entities().empty(),"Surviving traffic could not resume after deletion");
    auto result=captureSimulationResult(s,{});bool manual=false,natural=false;
    for(std::size_t i=0;i<result.columns.size();++i){if(result.columns[i]=="manually_completed_trips")manual=result.values[i]=="1";if(result.columns[i]=="destination_completed_trips")natural=result.values[i]=="1";}
    need(manual && natural,"CSV classified manual deletion as a destination arrival");
}
#ifdef _WIN32
int sumoTest(const std::string& mode){
    bool open=mode!="sumo",guard=mode=="guard";
    Map map(220,80,std::vector<Tile>(17600,Tile::Road));NodeGraph graph;graph.explicitDestinations=true;
    for(Vec2 p:std::vector<Vec2>{{10,40},{100,40},{210,40}})graph.addNode(p);
    graph.nodes[0].spawn=true;graph.nodes[2].destination=true;graph.nodes[1].openRadius=open?18:0;
    graph.connect(0,1,4,8);graph.connect(1,2,4,8);for(auto& road:graph.connections)road.roadType=1;
    Simulation s(map,42);s.setLanes(compileRoads(graph).lanes);s.setStuckVehicleSettings({false,30,false,60});
    SumoBackend backend(s,graph,42);need(s.spawnOnLane(0,VehicleKind::Car),"SUMO deletion fixture did not spawn");
    std::uint64_t selected=0;
    for(int i=0;i<500 && !selected;++i){s.tick(.1);for(const auto& e:s.entities())if(e.center.x>=(guard?83:95) && e.center.x<118)selected=e.id;}
    need(selected && !s.completed(),"SUMO deletion fixture did not reach the junction");
    need(s.spawnOnLane(0,VehicleKind::Car),"SUMO pending follower failed");
    auto time=s.seconds();auto stats=s.stopStatistics();
    need(s.deleteVehicle(selected),"SUMO deletion failed");
    need(s.seconds()==time && s.completed()==1 && s.manuallyCompleted()==1 && !s.removedStuckVehicles(),"SUMO deletion miscounted or advanced time");
    need(s.entities().size()+backend.pending()==1,"SUMO deletion affected an unselected pending trip");
    need(s.stopStatistics().totalSeconds==stats.totalSeconds && !s.deleteVehicle(selected),"SUMO deletion lost statistics or allowed a duplicate completion");
    for(int i=0;i<10;++i)s.tick(.1);
    need(s.completed()==1 && s.manuallyCompleted()==1 && s.entities().size()+backend.pending()==1,"SUMO resurrected or double-counted a deleted vehicle");
    need(backend.routePreview(selected).empty() && !backend.collisions() && !backend.teleports(),"SUMO deletion left a trip or caused recovery events");return 0;
}
#endif
int main(int argc,char** argv){try{
#ifdef _WIN32
    if(argc>1)return sumoTest(argv[1]);
#endif
    nativeTests();std::cout<<"Manual deletion: selection scope, accounting, statistics, traffic resumption and export passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
