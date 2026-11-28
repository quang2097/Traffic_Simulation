#include "traffic/Simulation.hpp"
#include "traffic/TrainingData.hpp"
#include "traffic/SignalLearning.hpp"
#include "traffic/NodeGraph.hpp"
#include "traffic/SumoBackend.hpp"
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
using namespace traffic;
void need(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
bool near(double a,double b){return std::abs(a-b)<1e-6;}
TrafficLight light(int id,int group){TrafficLight l;l.id=id;l.a={30.+id*10,40};l.b={30.+id*10,60};l.outDirection={1,0};l.greenSeconds=group?12:10;l.yellowSeconds=3;l.trSeconds=20;l.trJunction=1;l.trGroup=group;return l;}
Simulation scene(){Simulation s(Map(200,100,std::vector<Tile>(20000,Tile::Road)),42);s.setCollisionWorkers(1);s.setStuckVehicleSettings({false,30,false,60});s.setTrafficRecoverySettings({false,1,25});return s;}
void nativeTests(){
    auto s=scene();auto a=light(1,0),b=light(2,1),c=light(3,0),unrelated=light(4,0);unrelated.trSeconds=0;unrelated.greenSeconds=1000;
    s.setTrafficLights({a,b,c,unrelated});auto ls=s.trafficLights();
    need(near(ls[0].normalTr.cycle,68) && near(ls[0].normalTr.guard,3),"Shared cycle did not include two full Tr intervals");
    for(int step=0;step<4000;++step){double t=step*.05;auto p=ls[0].phase(t),q=ls[1].phase(t);
        need(!(p==LightPhase::Green && q==LightPhase::Green),"Normal groups were green together");
        need(p==ls[2].phase(t),"Linked parallel lights lost synchronization");
        if(ls[0].trRemaining(t)>=0)need(p==LightPhase::Red && q==LightPhase::Red,"Shared Tr was not all red");
        need(ls[3].phase(t)==LightPhase::Green,"Shared Tr affected an unrelated light");
    }
    need(near(ls[0].trRemaining(3),20) && near(ls[0].trRemaining(22),1),"Shared Tr countdown is wrong");
    need(ls[0].phase(22.999)==LightPhase::Red && ls[0].phase(23)==LightPhase::Green,"First green skipped startup yellow or Tr");
    need(ls[1].phase(55.999)==LightPhase::Red && ls[1].phase(56)==LightPhase::Green,"Second group skipped Tr");
    s.tick(5);auto epoch=s.trafficLights()[0].normalTr.epoch;s.setTrafficLights(s.trafficLights());
    need(s.trafficLights()[0].normalTr.epoch==epoch && near(s.trafficLights()[0].trRemaining(5),18),"Routine signal updates restarted Tr");
    auto reordered=s.trafficLights();std::swap(reordered[0],reordered[1]);s.setTrafficLights(reordered);need(s.trafficLights()[0].normalTr.epoch==epoch,"Reordering lights restarted the shared clock");
    auto regions=defaultSignalRegions(ls);regions[0].group=0;regions[1].group=1;regions[2].group=0;regions[3].group=0;
    auto controlled=ls;HoldSignalController hold;hold.begin(controlled,regions,5,60,2,1,1);s.setTrafficLights(controlled);hold.update(controlled,6);s.setTrafficLights(controlled);hold.update(controlled,8);s.setTrafficLights(controlled);
    need(!s.trafficLights()[0].normalTr.active && s.trafficLights()[0].phase(8)==LightPhase::Green,"Normal Tr was added on top of RL hold clearance");
    hold.stop(controlled);s.setTrafficLights(controlled);need(s.trafficLights()[0].normalTr.active,"Stopping hold control did not restore normal Tr");
    controlled=s.trafficLights();SignalTimingController density;density.begin(controlled,regions,5,3,1);s.setTrafficLights(controlled);density.queue({10,12});density.update(controlled,9);s.setTrafficLights(controlled);
    need(!s.trafficLights()[0].normalTr.active,"Normal Tr interfered with density policy timing");density.stop(controlled);s.setTrafficLights(controlled);need(s.trafficLights()[0].normalTr.active && !s.trafficLights()[0].suspendNormalTr,"Stopping density control did not restore normal Tr");
    auto file=std::filesystem::temp_directory_path()/("normal-tr-"+std::to_string(std::random_device{}())+".signals");s.saveTrafficLights(file.u8string());auto restored=scene();restored.loadTrafficLights(file.u8string());std::filesystem::remove(file);
    need(restored.trafficLights()[0].trSeconds==20 && restored.trafficLights()[0].trJunction==1 && restored.trafficLights()[0].normalTr.active,"Normal Tr did not survive save/load");
    Simulation legacy(Map(500,500,std::vector<Tile>(250000,Tile::Road)),42);legacy.loadTrafficLights(std::string(TRAFFIC_TEST_ROOT)+"/tests/fixtures/directional-signals/custom-map.png.signals");for(const auto& l:legacy.trafficLights())need(l.trSeconds==0 && !l.normalTr.active,"Old maps enabled shared Tr automatically");
    auto traffic=scene();a=light(1,0);a.a={30,40};a.b={30,60};a.permissive=true;a.allowedA={45,40};a.allowedB={45,60};traffic.setTrafficLights({a});traffic.setProfile(VehicleKind::Car,{{2,0},{1,0},{.5,0},{4,0,3,0},1});
    auto route=traffic.routingSettings();route.rtsStyle=route.spreadAtSignals=route.refreshMovingRoutes=false;traffic.setRoutingSettings(route);traffic.tick(5);
    need(traffic.spawnAt({24,50},{70,50},VehicleKind::Car,0),"Tr vehicle spawn failed");need(!traffic.routePermitsRed(traffic.entities().front(),traffic.trafficLights().front()),"Permissive OUT bypassed shared Tr");
    traffic.tick(10);traffic.tick(7.9);need(traffic.entities().front().center.x+1<30+1e-5,"Vehicle crossed during normal Tr");traffic.tick(2);need(traffic.entities().front().center.x>30,"Normal green did not release traffic after Tr");
}
#ifdef _WIN32
void sumoTest(){
    Map map(220,80,std::vector<Tile>(17600,Tile::Road));NodeGraph g;g.explicitDestinations=true;for(Vec2 p:std::vector<Vec2>{{10,40},{100,40},{210,40}})g.addNode(p);g.nodes[0].spawn=true;g.nodes[2].destination=true;g.connect(0,1,4,8);g.connect(1,2,4,8);for(auto& r:g.connections)r.roadType=1;
    Simulation s(map,42);s.setLanes(compileRoads(g).lanes);auto l=light(1,0);l.a={95,35};l.b={95,45};s.setTrafficLights({l});SumoBackend backend(s,g,42);need(s.spawnOnLane(0,VehicleKind::Car),"SUMO Tr spawn failed");
    for(int i=0;i<225;++i){s.tick(.1);for(const auto& e:s.entities())need(e.center.x<100,"SUMO crossed before full normal Tr");}
    for(int i=0;i<400 && !s.completed();++i)s.tick(.1);need(s.completed()==1 && !backend.collisions() && !backend.teleports(),"SUMO did not release its normal Tr queue cleanly");
}
#endif
int main(int argc,char**){try{
#ifdef _WIN32
    if(argc>1){sumoTest();return 0;}
#endif
    nativeTests();std::cout<<"Normal shared Tr: timing, synchronization, persistence, traffic and RL handover passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
