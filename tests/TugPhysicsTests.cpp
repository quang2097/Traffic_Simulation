#include "traffic/Simulation.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace traffic;
void require(bool value,const char* text) { if(!value) throw std::runtime_error(text); }
Simulation world() {
    Simulation s(Map(100,40,std::vector<Tile>(4000,Tile::Road)),42);
    s.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{4,0,2,0},1});
    auto routing=s.routingSettings(); routing.rtsStyle=false; routing.spreadAtSignals=false; s.setRoutingSettings(routing);
    s.setStuckVehicleSettings({false,30}); s.setTrafficRecoverySettings({false,1});
    auto tug=s.tugSettings(); tug.enabled=true; tug.alwaysOn=true; s.setTugSettings(tug);
    return s;
}
void safe(const Simulation& s) {
    for(const auto& a:s.entities()) {
        require(s.map().fits(a.footprint()),"force body left mask");
        require(std::isfinite(a.heading) && std::isfinite(a.speed),"nonfinite force state");
        for(const auto& b:s.entities()) if(a.id<b.id)
            require(clearance(a.footprint(),b.footprint())>=std::min(a.preferredDistance,b.preferredDistance)-1e-5,"force contact violated preferred clearance");
    }
}
int main() {
 try {
    auto light=world(),heavy=world();
    for(auto* s:{&light,&heavy}) {
        require(s->spawnAt({10,10},{50,20},VehicleKind::Car,0),"mass comparison spawn");
        auto& e=const_cast<std::vector<Entity>&>(s->entities())[0]; e.heading=0;
    }
    light.setEntityMass(light.entities()[0].id,100); heavy.setEntityMass(heavy.entities()[0].id,5000);
    for(int i=0;i<30;++i) { light.tick(.01); heavy.tick(.01); }
    require(light.entities()[0].center.x>heavy.entities()[0].center.x+.03,"larger mass resists translational acceleration");
    require(light.entities()[0].heading>heavy.entities()[0].heading && heavy.entities()[0].heading>0,"front force produces torque with rotational inertia");
    auto waypoint=world(); require(waypoint.spawnAt({10,10},{35,10},VehicleKind::Car,0),"waypoint spawn");
    auto& w=const_cast<std::vector<Entity>&>(waypoint.entities())[0];
    w.path={{10,10},{10.1,10.1},{35,10}}; w.pathHeadings={0,.1,0}; w.next=1;
    double closest=100;
    for(int i=0;i<6000 && !waypoint.entities().empty();++i) {
        waypoint.tick(.01); safe(waypoint);
        if(!waypoint.entities().empty()) closest=std::min(closest,length(waypoint.entities()[0].center-Vec2{10.1,10.1}));
    }
    require(waypoint.completed()==1 && closest>.02,"force guidance passes waypoints without reaching exact coordinates");
    auto bump=world();
    require(bump.spawnAt({10,10},{80,10},VehicleKind::Car,0) && bump.spawnAt({15,10},{80,10},VehicleKind::Car,0),"impulse spawns");
    auto& cars=const_cast<std::vector<Entity>&>(bump.entities());
    bump.setEntityMass(cars[0].id,10000); bump.setEntityMass(cars[1].id,200);
    cars[0].tugActive=true; cars[0].tugVelocity={3,0}; cars[0].speed=3;
    for(int i=0;i<150;++i) { bump.tick(.01); safe(bump); }
    require(bump.tugContacts()>0 && cars[1].center.x>16,"mass-weighted contacts transfer momentum without penetration");
    require(std::abs(cars[0].heading)<.05 && std::abs(cars[1].heading)<.05,"symmetric face contacts do not create artificial spin");
    auto red=world(); red.setTrafficLights({{{25,.01},{25,39.99},.1,0,100,1,true}});
    for(double x:{15.,9.,3.}) require(red.spawnAt({x,20},{60,20},VehicleKind::Car,0),"red tug spawn");
    for(int i=0;i<1800;++i) { red.tick(.01); safe(red); for(const auto& e:red.entities()) for(auto p:corners(e.footprint())) require(p.x<25,"tug crossed red light"); }
    require(red.completed()==0 && red.entities().size()==3,"red wait retains all force-guided vehicles");
    red.setTrafficLights({});
    for(int i=0;i<12000 && !red.entities().empty();++i) { red.tick(.01); safe(red); }
    require(red.completed()==3,"force queue resumes and reaches goal regions");
    auto stuck=world(); auto settings=stuck.tugSettings(); settings.alwaysOn=false; stuck.setTugSettings(settings);
    require(stuck.spawnAt({10,10},{40,10},VehicleKind::Car,0),"stale path spawn");
    auto& actor=const_cast<std::vector<Entity>&>(stuck.entities())[0]; actor.path[0]={4,10}; actor.currentStopSeconds=3;
    for(int i=0;i<200;++i) { stuck.tick(.01); safe(stuck); }
    require(stuck.entities()[0].center.x>10.5,"standstill tug escapes a stale trajectory anchor");
    auto oldPose=stuck.entities()[0].footprint();
    settings=stuck.tugSettings(); settings.enabled=false; stuck.setTugSettings(settings);
    require(!stuck.entities()[0].tugActive && length(stuck.entities()[0].center-oldPose.center)==0 && stuck.entities()[0].heading==oldPose.heading,"disabling force guidance reconnects without snapping pose");
    Map split(50,25,std::vector<Tile>(1250,Tile::Road));
    for(int y=0;y<25;++y) split.tiles[split.index({25,y})]=Tile::Divider;
    Simulation divider(split,42); auto tug=divider.tugSettings(); tug.enabled=tug.alwaysOn=true; divider.setTugSettings(tug);
    divider.setStuckVehicleSettings({false,30});
    require(divider.spawnAt({20,12},{8,12},VehicleKind::Car,0),"divider spawn");
    auto& d=const_cast<std::vector<Entity>&>(divider.entities())[0];
    d.heading=0; d.goal={40,12}; d.path={{20,12},{40,12}}; d.pathHeadings={0,0}; d.next=1;
    for(int i=0;i<1000;++i) { divider.tick(.01); safe(divider); for(auto p:corners(divider.entities()[0].footprint())) require(p.x<25,"force crossed solid divider"); }
    require(divider.completed()==0,"unreachable force target cannot fabricate arrival");
    auto invalid=stuck.tugSettings(); invalid.damping=0; bool rejected=false;
    try { stuck.setTugSettings(invalid); } catch(const std::invalid_argument&) { rejected=true; }
    require(rejected,"invalid force integration settings rejected");
    std::cout<<"Mass, front torque, waypoint-free guidance, buffer impulses and red-light protection passed\n";
 } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
