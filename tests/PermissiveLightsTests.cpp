#include "traffic/Simulation.hpp"
#include <iostream>
#include <stdexcept>
using namespace traffic;
void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
Simulation scene() {
    Simulation s(Map(100,30,std::vector<Tile>(3000,Tile::Road)),42);
    s.setProfile(VehicleKind::Car,{{2,0},{1,0},{.5,0},{8,0,8,0},1});
    auto routing=s.routingSettings(); routing.refreshMovingRoutes=false; routing.spreadAtSignals=false; s.setRoutingSettings(routing);
    return s;
}
TrafficLight special(double offset=21) {
    TrafficLight light{{25,.01},{25,29.99},.1,20,100,offset,true};
    light.permissive=true; light.allowedA={50,.01}; light.allowedB={50,14}; return light;
}
void invariants(const Simulation& s) {
    for(std::size_t i=0;i<s.entities().size();++i) {
        require(s.map().fits(s.entities()[i].footprint()),"permitted vehicle left road");
        for(std::size_t j=i+1;j<s.entities().size();++j)
            require(clearance(s.entities()[i].footprint(),s.entities()[j].footprint())+1e-5>=std::min(s.entities()[i].preferredDistance,s.entities()[j].preferredDistance),"permission bypassed physical spacing");
    }
}
int main() {
    try {
        for(double offset:{.11,21.}) {
            auto s=scene(); auto light=special(offset); s.setTrafficLights({light});
            require(s.spawnAt({10,8},{85,8},VehicleKind::Car,0) && s.spawnAt({10,22},{85,22},VehicleKind::Car,0),"branch vehicles spawn");
            require(s.routePermitsRed(s.entities()[0],light) && !s.routePermitsRed(s.entities()[1],light),"route-specific eligibility");
            for(int i=0;i<600;++i) { s.tick(.01); invariants(s); require(s.entities()[1].center.x+1<25+1e-6,"nonqualifying vehicle crossed red/yellow"); }
            require(s.entities()[0].center.x>45,"qualifying vehicle did not pass red/yellow");
        }
        auto s=scene(); Entity route; route.center={10,8}; route.goal={85,8}; route.next=1;
        route.path={{10,8},{35,20},{60,20},{85,8}};
        require(!s.routePermitsRed(route,special()),"endpoint alone must not confer permission");
        route.path={{10,8},{85,8}}; auto before=special(); before.allowedA={15,0}; before.allowedB={15,14};
        require(!s.routePermitsRed(route,before),"exit crossed before stop line must not confer permission");
        auto parallel=special(); parallel.allowedA={35,8}; parallel.allowedB={55,8};
        require(!s.routePermitsRed(route,parallel),"driving along a line is not crossing it");
        auto other=special(); other.a={70,.01}; other.b={70,29.99};
        require(!s.routePermitsRed(route,other),"exit permission belongs to its own upstream stop line");

        auto independent=scene(); auto normal=TrafficLight{{60,.01},{60,29.99},.1,0,100,1,true};
        independent.setTrafficLights({special(),normal}); require(independent.spawnAt({10,8},{85,8},VehicleKind::Car,0),"independent light spawn");
        for(int i=0;i<1000;++i) independent.tick(.01);
        require(independent.entities()[0].center.x>30 && independent.entities()[0].center.x+1<60+1e-6,"one permission must not bypass another red light");

        auto queued=scene(); auto routing=queued.routingSettings(); routing.spreadAtSignals=true; queued.setRoutingSettings(routing);
        auto light=special(); light.permissive=false; queued.setTrafficLights({light});
        require(queued.spawnAt({20,8},{85,8},VehicleKind::Car,0),"queued vehicle spawn");
        for(int i=0;i<500;++i) queued.tick(.01);
        require(queued.entities()[0].signalLight==0 && queued.entities()[0].waitingForLight,"vehicle holds a signal slot");
        light.permissive=true; double clock=queued.seconds(); auto id=queued.entities()[0].id;
        queued.setTrafficLights({light});
        require(queued.routePermitsRed(queued.entities()[0],light),"queued preferred departure route determines permission");
        for(int i=0;i<300;++i) queued.tick(.01);
        require(queued.seconds()>clock && queued.entities()[0].id==id && queued.entities()[0].center.x>30,"live linked-exit edit releases a qualifying red queue");

        Simulation narrow(Map(70,3,std::vector<Tile>(210,Tile::Road)),7);
        narrow.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{6,0,3,0},1});
        routing=narrow.routingSettings(); routing.rtsStyle=false; routing.spreadAtSignals=false; narrow.setRoutingSettings(routing);
        auto gate=special(); gate.a={25,.01}; gate.b={25,2.99}; gate.allowedA={45,.01}; gate.allowedB={45,2.99}; narrow.setTrafficLights({gate});
        require(narrow.spawnAt({20,1.5},{35,1.5},VehicleKind::Car,0) && narrow.spawnAt({13,1.5},{65,1.5},VehicleKind::Car,0),"physical queue spawn");
        for(int i=0;i<800;++i) { narrow.tick(.01); invariants(narrow); }
        require(narrow.entities().size()==2 && narrow.entities()[1].center.x<narrow.entities()[0].center.x && narrow.entities()[0].center.x+2<25+1e-6,"permitted follower cannot drive through a stopped leader");
        bool rejected=false; gate.allowedB=gate.allowedA;
        try { narrow.setTrafficLights({gate}); } catch(const std::invalid_argument&) { rejected=true; }
        require(rejected,"degenerate linked exit rejected");
        std::cout<<"Linked-exit red/yellow permissions, live queues and collision safety passed\n";
    } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
