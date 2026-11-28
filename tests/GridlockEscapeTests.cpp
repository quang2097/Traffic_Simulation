#include "traffic/Simulation.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <unordered_map>
using namespace traffic;
void need(bool value,const char* why) {if(!value)throw std::runtime_error(why);}
Simulation jam(int category,bool escape) {
    Simulation s(Map(70,3,std::vector<Tile>(210,Tile::Road)),42);
    s.setCollisionWorkers(1);s.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{3,0,3,0},1});
    auto routing=s.routingSettings();routing.rtsStyle=false;routing.spreadAtSignals=false;routing.refreshMovingRoutes=false;s.setRoutingSettings(routing);
    s.setTrafficRecoverySettings({false,1,25});s.setStuckVehicleSettings({true,8,false,60});
    s.setGridlockEscapeSettings({escape,1,.5,1.5});
    need(s.spawnAt({25,1.5},{60,1.5},VehicleKind::Car,0),"front spawn");
    need(s.spawnAt({33,1.5},{5,1.5},VehicleKind::Car,0),"opposing spawn");
    need(s.spawnAt({18,1.5},{60,1.5},VehicleKind::Car,0),"follower spawn");
    for(auto& e:const_cast<std::vector<Entity>&>(s.entities())) {
        if(category==1)e.internalJunction=9;
        if(category==2){e.inOpenJunction=true;e.junctionCenter={35,1.5};e.junctionRadius=100;}
    }
    return s;
}
int main(){try{
    for(int category=0;category<3;++category) {
        auto s=jam(category,true);s.setStuckVehicleSettings({false,30,false,60});
        for(int step=0;step<1000;++step) {
            std::unordered_map<std::uint64_t,Vec2> before;
            for(const auto& e:s.entities())before[e.id]=e.center;
            s.tick(.01);
            for(const auto& e:s.entities()) {
                need(s.map().fits(e.footprint()),"escape left road mask");
                need(length(e.center-before.at(e.id))<=.031,"escape teleported instead of moving");
                need(!e.escapeOverlap,"ghost escape policy became active");
            }
        }
        need(s.completed()==0 && s.entities().size()==3 && s.removedStuckVehicles()==0,"sealed blockage was cleared by disappearing or overlapping bodies");
        need(s.gridlockEscapes()>0 && s.overlapEscapes()==0 && s.physicalCollisionEvents()==0,"reservation release must retain physical collision checks");
        need((category==0?s.laneGridlockEscapes():category==1?s.internalGridlockEscapes():s.freeGridlockEscapes())==s.gridlockEscapes(),"escape category counter mismatch");
    }
    auto disabled=jam(0,false);disabled.tick(4);need(disabled.gridlockEscapes()==0 && disabled.completed()==0,"disabled escape changed normal traffic");
    for(int category=0;category<3;++category) {
        auto s=jam(category,false);s.setStuckVehicleSettings({false,30,false,60});
        s.setGridlockEscapeSettings({false,1,.5,1.5,true});
        s.tick(1.4);need(s.overlapEscapes()==0,"overlap activated before the reservation and extra wait elapsed");
        for(int step=0;step<8000 && !s.entities().empty();++step) {
            std::unordered_map<std::uint64_t,Vec2> before;
            for(const auto& e:s.entities())before[e.id]=e.center;
            s.tick(.01);
            for(const auto& e:s.entities()) {
                need(s.map().fits(e.footprint()),"overlap escape left road mask");
                need(length(e.center-before.at(e.id))<=.031,"overlap escape teleported");
            }
        }
        need(s.completed()==3 && s.entities().empty(),"optional overlap did not drain sealed opposing traffic");
        need(s.overlapEscapes()>0 && s.escapeOverlapPairs()>0 && s.removedStuckVehicles()==0,"optional overlap must move vehicles through bodies without deleting them");
    }
    Simulation red(Map(70,3,std::vector<Tile>(210,Tile::Road)),42);
    red.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{3,0,3,0},1});
    red.setTrafficLights({{{30,.01},{30,2.99},.1,0,100,1,true}});
    red.setGridlockEscapeSettings({true,1,.5,1.5});red.setStuckVehicleSettings({false,30,false,60});
    need(red.spawnAt({24,1.5},{60,1.5},VehicleKind::Car,0),"red leader");need(red.spawnAt({17,1.5},{60,1.5},VehicleKind::Car,0),"red follower");
    red.tick(5);need(red.gridlockEscapes()==0 && red.entities().size()==2,"legal red queue must not activate overlap escape");
    red.setGridlockEscapeSettings({false,1,0,1.5,true});
    red.tick(5);need(red.gridlockEscapes()==0 && red.overlapEscapes()==0,"optional overlap bypassed a legal red-light queue");
    auto& redLeader=const_cast<std::vector<Entity>&>(red.entities()).front();
    redLeader.escapingGridlock=redLeader.escapeOverlap=true;redLeader.escapeAnchor=redLeader.center;
    red.tick(5);need(red.entities().front().center.x+2<=30+1e-5,"active overlap escape crossed a red light");
    auto toggled=jam(0,false);toggled.setStuckVehicleSettings({false,30,false,60});
    toggled.setGridlockEscapeSettings({false,1,0,1.5,true});
    for(int i=0;i<1000 && !toggled.escapeOverlapPairs();++i)toggled.tick(.01);
    need(toggled.escapeOverlapPairs()>0,"toggle fixture never overlapped");
    auto escapes=toggled.overlapEscapes();toggled.setGridlockEscapeSettings({false,1,0,1.5,false});
    for(int i=0;i<1000;++i)toggled.tick(.01);
    need(toggled.overlapEscapes()==escapes,"disabling overlap admitted a new overlap escape");
    for(const auto& e:toggled.entities())need(!e.escapeOverlap,"disabling overlap stranded a vehicle inside another body");
    auto fixture=std::string(TRAFFIC_TEST_ROOT)+"/tests/fixtures/directional-signals/custom-map.png";
    Simulation supplied(Map::fromPng(fixture,1),42);supplied.loadTrafficLights(fixture+".signals");
    supplied.setCollisionWorkers(1);supplied.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{3,0,3,0},1});
    auto routing=supplied.routingSettings();routing.rtsStyle=routing.spreadAtSignals=routing.refreshMovingRoutes=false;supplied.setRoutingSettings(routing);
    supplied.setTrafficRecoverySettings({false,1,25});supplied.setStuckVehicleSettings({false,30,false,60});
    need(supplied.spawnAt({230,250},{310,250},VehicleKind::Car,0) && supplied.spawnAt({238,250},{200,250},VehicleKind::Car,std::acos(-1.)),"supplied intersection jam spawn");
    supplied.tick(5);need(supplied.completed()==0 && supplied.entities().size()==2,"supplied jam fixture did not block");
    supplied.setGridlockEscapeSettings({false,1,.5,1.5,true});
    for(int i=0;i<800 && !supplied.entities().empty();++i) {
        supplied.tick(.1);for(const auto& e:supplied.entities())need(supplied.map().fits(e.footprint()),"supplied map overlap escape left road");
    }
    need(supplied.completed()==2 && supplied.escapeOverlapPairs()>0 && !supplied.removedStuckVehicles(),"supplied map blockage did not drain through optional overlap");
    std::cout<<"Optional overlap drains lane/internal/free and supplied-map blockages; default spacing and red queues protected\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
