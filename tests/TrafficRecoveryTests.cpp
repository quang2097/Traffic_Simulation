#include "traffic/Simulation.hpp"
#include "traffic/Geometry.hpp"
#include <iostream>
#include <stdexcept>
using namespace traffic;
void require(bool v,const char* message) { if(!v) throw std::runtime_error(message); }
int main() {
 try {
    Footprint a{{10,10},0,4,2},b{{10,13},0,4,2},to=a; to.center.x+=.1;
    require(geometry::motionClearance(a,to,b,1),"exact preferred gap permits parallel motion");
    b.center.y=12.9; to.center.y-=.1;
    require(!geometry::motionClearance(a,to,b,1),"ordinary checks preserve preferred gap");
    require(geometry::motionClearance(a,to,b,1,true),"cooperative escape can increase existing undersized gap");
    to.center.y=10.1;
    require(!geometry::motionClearance(a,to,b,1,true),"cooperative escape cannot reduce existing gap");
    b.center.y=11;
    require(!geometry::motionClearance(a,to,b,0,true),"zero preference still forbids body overlap");

    Simulation s(Map(60,30,std::vector<Tile>(1800,Tile::Road)),42);
    s.setProfile(VehicleKind::Car,{{4,0},{2,0},{1,0},{4,0,4,0},1});
    s.setStuckVehicleSettings({false,30});
    auto routing=s.routingSettings(); routing.rtsStyle=false; s.setRoutingSettings(routing);
    require(s.spawnAt({10,10},{50,10},VehicleKind::Car,0),"first spawn");
    require(s.spawnAt({10,15},{50,15},VehicleKind::Car,0),"second spawn");
    auto& es=const_cast<std::vector<Entity>&>(s.entities());
    // Only the future reservation intersects; both actual roads are free.
    es[1].reservedTurn={{8,8},{20,8},{20,12},{8,12}};
    for(auto& e:es) { e.waitingForManeuver=true; e.currentStopSeconds=2; e.stoppedSeconds=2; e.blockedSeconds=2; }
    const auto before=es; const auto ticks=s.ticks();
    Simulation automatic=s; automatic.tick(.01);
    require(automatic.trafficRecoveryRepairs()==1 && automatic.seconds()<=.0100001,"stoppage automatically invokes recovery without adding preview time");
    s.planStoppedTraffic(true);
    require(s.trafficRecoveryRepairs()==1,"free-road reservation deadlock repaired in one batch");
    require(s.seconds()==0 && s.ticks()==ticks && s.completed()==0 && s.removedStuckVehicles()==0,"preview cannot advance real clock or trip counters");
    for(std::size_t i=0;i<es.size();++i) {
        require(length(es[i].center-before[i].center)==0 && es[i].heading==before[i].heading && es[i].speed==before[i].speed,"preview must not move real vehicles");
        require(es[i].stoppedSeconds==2 && es[i].currentStopSeconds==2 && es[i].ageSeconds==before[i].ageSeconds,"preview must preserve stop telemetry");
    }
    for(int n=0;n<100;++n) {
        s.tick(.01);
        require(clearance(es[0].footprint(),es[1].footprint())>=1-1e-7,"coordinated motion respects pair gap");
    }
    require(es[0].center.x>11 && es[1].center.x>11,"both vehicles resume together");
    es[0].waitingForLight=true; es[0].speed=0; es[0].currentStopSeconds=2; es[0].waitingForManeuver=true;
    require(s.stopReason(es[0])==StopReason::Unexplained,"stale light flag cannot justify a stop on a free road");
    s.setTrafficLights({{{es[0].center.x+2.15,1},{es[0].center.x+2.15,29},.1,0,100,1,true}});
    require(s.stopReason(es[0])==StopReason::Signal,"actual red crossing justifies stationary traffic");
    es[1].speed=1;
    auto passes=s.trafficRecoveryPasses(); s.planStoppedTraffic(true);
    require(s.trafficRecoveryPasses()==passes,"intentional signal waits excluded");
    Simulation sealed(Map(14,3,std::vector<Tile>(42,Tile::Road)),1);
    sealed.setProfile(VehicleKind::Car,{{2,0},{2,0},{1,0},{4,0,4,0},1});
    sealed.setStuckVehicleSettings({false,30}); sealed.setRoutingSettings(routing);
    for(double x:{5.,8.,2.}) require(sealed.spawnAt({x,1.5},{12,1.5},VehicleKind::Car,0),"sealed queue spawn");
    auto& trapped=const_cast<std::vector<Entity>&>(sealed.entities());
    trapped[0].currentStopSeconds=2; trapped[0].blockedSeconds=2; trapped[0].waitingForManeuver=true;
    for(int i:{1,2}) { trapped[i].waitingForLight=true; trapped[i].desiredSpeed=0; }
    auto original=trapped; sealed.planStoppedTraffic(true);
    require(sealed.trafficRecoveryRepairs()==0 && sealed.entities().size()==3 && sealed.seconds()==0,"no safe solution must leave real traffic intact");
    for(std::size_t i=0;i<trapped.size();++i)
        require(length(trapped[i].center-original[i].center)==0 && trapped[i].path.size()==original[i].path.size() && trapped[i].next==original[i].next,"failed preview preserves original pose and route");
    auto attempted=sealed.trafficRecoveryPasses(); sealed.planStoppedTraffic();
    require(sealed.trafficRecoveryPasses()==attempted,"failed recovery observes retry cooldown");
    require(sealed.stopReason(trapped[0])==StopReason::Vehicle,"a real stopped leader is a legal blockage");
    Simulation fleet(Map(80,90,std::vector<Tile>(7200,Tile::Road)),17);
    fleet.setProfile(VehicleKind::Car,{{4,0},{2,0},{1,0},{4,0,4,0},1});
    fleet.setStuckVehicleSettings({false,30}); fleet.setRoutingSettings(routing);
    for(int i=0;i<18;++i) require(fleet.spawnAt({10,3.+i*5},{70,3.+i*5},VehicleKind::Car,0),"whole-fleet spawn");
    auto& all=const_cast<std::vector<Entity>&>(fleet.entities());
    all[0].currentStopSeconds=2; all[0].waitingForLight=true;
    all[0].path[0]={4,3}; // Stale path anchor: physics correctly refuses to jump to it.
    // No blocked flag or reservation: a free-road standstill must still trigger.
    auto fleetBefore=all; fleet.planStoppedTraffic(true);
    require(fleet.lastFleetRouteCount()==18 && fleet.trafficRecoveryRepairs()==1,"unexplained stoppage maps every active vehicle, beyond old group limit");
    require(length(all[0].path.front()-all[0].center)<1e-9,"fleet repair replaces stale path anchor from current physical pose");
    require(fleet.seconds()==0 && fleet.ticks()==0,"fleet mapping holds simulation clock");
    for(std::size_t i=0;i<all.size();++i)
        require(length(all[i].center-fleetBefore[i].center)==0 && length(all[i].goal-fleetBefore[i].goal)==0 && all[i].ageSeconds==0,"fleet mapping preserves poses goals and telemetry");
    std::cout<<"Inclusive clearance and frozen-clock coordinated recovery passed\n";
 } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
