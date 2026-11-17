#include "traffic/Simulation.hpp"
#include <iostream>
#include <stdexcept>
using namespace traffic;
void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
Simulation lane() {
    Simulation s(Map(70,3,std::vector<Tile>(210,Tile::Road)),42);
    auto profile=s.profile(VehicleKind::Car);
    profile.length={4,0}; profile.width={1.8,0}; profile.distance={.5,0}; profile.motion={3,0,3,0};
    s.setProfile(VehicleKind::Car,profile);
    auto routing=s.routingSettings(); routing.rtsStyle=false; routing.spreadAtSignals=false; s.setRoutingSettings(routing);
    return s;
}
void jam(Simulation& s) {
    require(s.spawnAt({25,1.5},{60,1.5},VehicleKind::Car,0),"front spawn");
    require(s.spawnAt({33,1.5},{5,1.5},VehicleKind::Car,0),"opposing spawn");
    require(s.spawnAt({18,1.5},{60,1.5},VehicleKind::Car,0),"follower spawn");
}
int main() {
    try {
        auto s=lane(); s.setStuckVehicleSettings({true,2}); jam(s);
        auto front=s.entities()[0].id,follower=s.entities()[2].id;
        for(int step=0;step<1200 && s.removedStuckVehicles()==0;++step) s.tick(.01);
        require(s.removedStuckVehicles()==1,"stationary queue blocker removed");
        require(s.lastRemovedStuckVehicle()==front,"front blocker chosen instead of rear follower");
        bool followerPresent=false;
        for(const auto& e:s.entities()) { require(e.id!=front,"removed body retained"); followerPresent|=e.id==follower; }
        require(followerPresent && s.entities().size()==2 && s.completed()==0,"removal separate from successful arrival");
        require(s.stopStatistics().vehicles==3 && s.stopStatistics(false).vehicles==2,"removed wait telemetry retained");
        for(int step=0;step<150;++step) s.tick(.01);
        require(s.removedStuckVehicles()==1,"cooldown prevents immediate queue deletion");
        for(int step=0;step<6000 && !s.entities().empty();++step) s.tick(.01);
        require(s.entities().empty() && s.completed()+s.removedStuckVehicles()==3,"queue drains with removed trips accounted for");

        auto disabled=lane(); disabled.setStuckVehicleSettings({false,1}); jam(disabled);
        for(int step=0;step<500;++step) disabled.tick(.01);
        require(disabled.removedStuckVehicles()==0 && disabled.entities().size()==3,"disabled policy preserves jammed vehicles");

        auto red=lane(); red.setStuckVehicleSettings({true,1});
        TrafficLight light{{30,.01},{30,2.99},.1,0,100,1,true}; red.setTrafficLights({light});
        require(red.spawnAt({24,1.5},{60,1.5},VehicleKind::Car,0),"red leader spawn");
        require(red.spawnAt({17,1.5},{60,1.5},VehicleKind::Car,0),"red follower spawn");
        for(int step=0;step<1500;++step) red.tick(.01);
        require(red.removedStuckVehicles()==0 && red.entities().size()==2,"red queue protected beyond timeout");
        require(red.entities()[0].stoppedSeconds>5,"red protection does not erase stopped statistics");
        light.greenSeconds=100; light.redSeconds=.1; red.setTrafficLights({light});
        for(int step=0;step<50;++step) red.tick(.01);
        require(red.removedStuckVehicles()==0,"green gets fresh observation period");
        for(int step=0;step<3000 && !red.entities().empty();++step) red.tick(.01);
        require(red.completed()==2 && red.removedStuckVehicles()==0,"normal signal queue finishes without removal");
        bool rejected=false;
        try { red.setStuckVehicleSettings({true,0}); } catch(const std::invalid_argument&) { rejected=true; }
        require(rejected,"invalid timeout rejected");
        std::cout<<"Stuck leader removal, cooldown and signal protection passed\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
