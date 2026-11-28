#include "traffic/Simulation.hpp"
#include "traffic/SimulationResults.hpp"
#include "traffic/NodeGraph.hpp"
#include "traffic/SumoBackend.hpp"
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
using namespace traffic;
void need(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
TrafficLight signal(){
    TrafficLight l;l.a={30,40};l.b={30,60};l.greenSeconds=1;l.yellowSeconds=1;l.redSeconds=4;l.offsetSeconds=3;
    l.completeJunctionOnGreen=true;l.completionJunction=1;l.completionCenter={50,50};l.completionRadius=12;return l;
}
Simulation scene(){
    Simulation s(Map(200,100,std::vector<Tile>(20000,Tile::Road)),42);s.setCollisionWorkers(1);
    s.setProfile(VehicleKind::Car,{{2,0},{1,0},{.5,0},{3,0,3,0},1});
    s.setStuckVehicleSettings({false,30,false,60});s.setTrafficRecoverySettings({false,1,25});
    auto routing=s.routingSettings();routing.rtsStyle=routing.spreadAtSignals=routing.refreshMovingRoutes=false;s.setRoutingSettings(routing);return s;
}
void stationary(Simulation& s,Vec2 p){
    need(s.spawnAt(p,{p.x+20,p.y},VehicleKind::Car,0),"Could not insert clearance fixture");
    auto& e=const_cast<std::vector<Entity>&>(s.entities()).back();e.desiredSpeed=e.speed=0;
}
void nativeTests(){
    auto s=scene();auto l=signal();auto duplicate=l;duplicate.id=99;s.setTrafficLights({l,duplicate});
    stationary(s,{50,50});stationary(s,{56,55});stationary(s,{25,50});stationary(s,{150,50});
    s.tick(2.9);need(!s.completed() && s.entities().size()==4,"Red/yellow cleared a junction");
    s.tick(.2);need(s.completed()==2 && s.completedOnGreen()==2 && s.entities().size()==2,"Green must clear all inside vehicles exactly once, preserving outside queues");
    need(!s.removedStuckVehicles(),"Green completions were counted as stuck deletions");
    auto stats=s.stopStatistics();need(stats.vehicles==4 && stats.vehiclesThatStopped==4 && stats.totalSeconds>10,"Green completion lost stopped-time statistics");
    stationary(s,{50,50});s.tick(.3);need(s.entities().size()==3 && s.completedOnGreen()==2,"Continuous green kept deleting new entrants");
    s.tick(5.7);need(s.completedOnGreen()==3 && s.entities().size()==2,"The next green transition did not clear the junction");
    auto r=captureSimulationResult(s,{});bool exported=false;
    for(std::size_t i=0;i<r.columns.size();++i)if(r.columns[i]=="completed_on_green")exported=r.values[i]=="3";
    need(exported,"Green completions are missing from comparison results");
    auto disabled=scene();l.completeJunctionOnGreen=false;disabled.setTrafficLights({l});stationary(disabled,{50,50});disabled.tick(4);
    need(disabled.completed()==0,"Default-off green completion deleted traffic");
    auto external=scene();l=signal();l.externallyControlled=true;l.externalPhase=LightPhase::Green;external.setTrafficLights({l});stationary(external,{50,50});external.tick(.1);
    need(!external.completed(),"Initial green installation cleared vehicles without a phase change");
    l=external.trafficLights().front();l.externalPhase=LightPhase::Red;external.setTrafficLights({l});external.tick(.1);
    auto other=l;other.id=77;other.completeJunctionOnGreen=false;external.setTrafficLights({other,l});
    l.externalPhase=LightPhase::Green;external.setTrafficLights({other,l});external.tick(.01);
    need(external.completed()==1 && external.completedOnGreen()==1,"External/RL green transition or reordered signal identity was missed");
    stationary(external,{50,50});l.enabled=false;external.setTrafficLights({other,l});external.tick(.1);
    need(external.completed()==1,"Disabling a light triggered completion");
    l.enabled=true;external.setTrafficLights({other,l});external.tick(.1);need(external.completed()==1,"Enabling an already green light triggered completion");
    auto file=std::filesystem::temp_directory_path()/("green-completion-"+newSimulationRunId()+".signals");
    external.saveTrafficLights(file.u8string());auto loaded=scene();loaded.loadTrafficLights(file.u8string());std::filesystem::remove(file);
    const auto& saved=loaded.trafficLights().back();need(saved.completeJunctionOnGreen && saved.completionJunction==1 && saved.completionCenter.x==50 && saved.completionRadius==12,"Green clearing settings did not survive .signals save/load");
    Simulation legacy(Map(500,500,std::vector<Tile>(250000,Tile::Road)),42);
    legacy.loadTrafficLights(std::string(TRAFFIC_TEST_ROOT)+"/tests/fixtures/directional-signals/custom-map.png.signals");
    for(const auto& old:legacy.trafficLights())need(!old.completeJunctionOnGreen,"Older .signals files enabled green completion");
}
void topologyTests(){
    NodeGraph g;for(Vec2 p:std::vector<Vec2>{{10,50},{30,50},{70,50},{70,10},{70,90},{100,50}})g.addNode(p);
    for(auto pair:std::vector<std::pair<int,int>>{{0,1},{1,2},{2,3},{2,4},{2,5}}){g.connect(pair.first,pair.second);g.connections.back().roadType=2;}
    g.nodes[1].roadStation=true;auto l=signal();l.anchorNode=1;l.outDirection={1,0};need(connectedSignalJunction(g,l)==2,"Signal did not find the downstream connected junction");
    l.outDirection={-1,0};need(connectedSignalJunction(g,l)==-1,"Reversed light selected a junction behind it");
}
#ifdef _WIN32
int sumoTest(bool open,bool guard){
    Map map(220,80,std::vector<Tile>(17600,Tile::Road));NodeGraph g;g.explicitDestinations=true;
    for(Vec2 p:std::vector<Vec2>{{10,40},{100,40},{210,40}})g.addNode(p);
    g.nodes[0].spawn=true;g.nodes[2].destination=true;g.nodes[1].openRadius=open?18:0;
    g.connect(0,1,4,8);g.connect(1,2,4,8);for(auto& r:g.connections)r.roadType=1;
    auto roads=compileRoads(g);Simulation s(map,42);s.setLanes(roads.lanes);s.setStuckVehicleSettings({false,30,false,60});
    auto l=signal();l.a={80,35};l.b={80,45};l.completionCenter={100,40};l.completionRadius=18;l.externallyControlled=true;l.externalPhase=LightPhase::Green;s.setTrafficLights({l});
    SumoBackend backend(s,g,42);need(s.spawnOnLane(0,VehicleKind::Car),"SUMO green-clearance spawn failed");
    bool inside=false;for(int i=0;i<500 && !inside;++i){s.tick(.1);for(const auto& e:s.entities())if(e.center.x>=(guard?83:95) && e.center.x<118)inside=true;}
    need(inside && !s.completed(),"SUMO vehicle did not enter the test junction");
    l=s.trafficLights().front();l.externalPhase=LightPhase::Red;s.setTrafficLights({l});s.tick(.1);
    need(s.spawnOnLane(0,VehicleKind::Car),"SUMO pending queue spawn failed");
    l.externalPhase=LightPhase::Green;s.setTrafficLights({l});s.tick(.1);
    need(s.completed()==1 && s.completedOnGreen()==1 && !s.removedStuckVehicles(),"SUMO green completion was lost or miscounted");
    need(s.entities().size()+backend.pending()==1,"SUMO clearing deleted an outside or pending vehicle");
    for(int i=0;i<10;++i)s.tick(.1);
    need(s.completed()==1 && s.completedOnGreen()==1 && s.entities().size()+backend.pending()==1,"SUMO resurrected or double-counted a cleared vehicle");
    need(!backend.collisions() && !backend.teleports(),"SUMO clearing caused a collision or teleport");return 0;
}
#endif
int main(int argc,char** argv){try{
#ifdef _WIN32
    if(argc>1){std::string mode=argv[1];return sumoTest(mode!="sumo",mode=="sumo-guard");}
#endif
    nativeTests();topologyTests();std::cout<<"Green junction completion: edge triggers, scope, accounting, signals, export and topology passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
