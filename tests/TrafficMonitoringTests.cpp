#include "traffic/Simulation.hpp"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace traffic;
namespace {
void need(bool value,const char* why) { if(!value) throw std::runtime_error(why); }
bool near(double a,double b) { return std::abs(a-b)<1e-6; }
Simulation base() {
    Simulation s(Map(100,30,std::vector<Tile>(3000,Tile::Road)),42);
    s.setCollisionWorkers(1); s.setStuckVehicleSettings({false,30,false,1});
    s.setTrafficRecoverySettings({false,1,25});
    auto routing=s.routingSettings(); routing.rtsStyle=false; routing.spreadAtSignals=false;
    routing.refreshMovingRoutes=false; s.setRoutingSettings(routing);
    return s;
}
std::vector<Entity>& actors(Simulation& s) { return const_cast<std::vector<Entity>&>(s.entities()); }
void add(Simulation& s,Vec2 p,bool junction=false) {
    Entity e; e.id=s.entities().size()+1; e.center=p; e.length=4; e.width=2;
    e.path={p,{80,p.y}}; e.pathHeadings={0,0}; e.goal=e.path.back(); e.next=1;
    e.desiredSpeed=0; e.acceleration=3; e.preferredDistance=0; e.inOpenJunction=junction;
    e.junctionCenter=p; e.junctionRadius=20;
    actors(s).push_back(e);
}
void detectors() {
    auto rotated=base(); add(rotated,{8.5,5});
    actors(rotated)[0].heading=std::acos(-1.)*.5;
    TrafficLight orientation{{50,1},{50,25}}; orientation.queueDetectors={{{10,1},{10,9}}};
    rotated.setTrafficLights({orientation}); auto rotatedId=rotated.trafficLights()[0].id;
    need(!rotated.trafficLightQueueStatus(rotatedId).lines[0].occupied,"rotated width must not use an axis-aligned bounding length");
    actors(rotated)[0].heading=0; rotated.setTrafficLights(rotated.trafficLights());
    need(rotated.trafficLightQueueStatus(rotatedId).lines[0].occupied,"rotated detector test must detect actual body intersection");
    auto s=base(); add(s,{9,5});
    TrafficLight light{{50,1},{50,25}}; light.queueWindowSeconds=4; light.queueThresholdPercent=50;
    light.queueDetectors={{{10,1},{10,9}},{{15,1},{15,9}},{{10,20},{10,25}}};
    s.setTrafficLights({light}); auto id=s.trafficLights()[0].id;
    auto status=s.trafficLightQueueStatus(id);
    need(status.lines[0].occupied && !status.lines[1].occupied && !status.lines[2].occupied,"finite detector must touch full body, not an infinite line or centre point");
    s.tick(2); status=s.trafficLightQueueStatus(id);
    need(status.qualityLine==0 && near(status.lines[0].occupiedPercent,100),"full-window warmup required");
    // Moving the stationary fixture simulates traffic leaving the detector.
    actors(s)[0].center={30,5}; actors(s)[0].path={{30,5},{80,5}};
    s.tick(2); status=s.trafficLightQueueStatus(id);
    need(status.qualityLine==1 && !status.lines[0].occupied && near(status.lines[0].occupiedPercent,50),"quality uses time occupancy rather than current presence");
    auto edited=s.trafficLights(); edited[0].greenSeconds=25; s.setTrafficLights(edited);
    need(near(s.trafficLightQueueStatus(id).lines[0].observedSeconds,4),"signal timing edit must retain detector history");
    s.tick(1); need(near(s.trafficLightQueueStatus(id).lines[0].occupiedPercent,25),"rolling window must expire old occupied time");
    s.tick(1); need(s.trafficLightQueueStatus(id).qualityLine==0,"empty window clears queue level");
    add(s,{15,5}); s.tick(4);
    status=s.trafficLightQueueStatus(id);
    need(status.qualityLine==2 && status.lines[1].occupied,"farthest qualifying line sets quality");
    edited=s.trafficLights(); std::swap(edited[0].queueDetectors[0],edited[0].queueDetectors[1]);
    s.setTrafficLights(edited); status=s.trafficLightQueueStatus(id);
    need(status.qualityLine==1 && near(status.lines[0].occupiedPercent,100),"reorder retains identity history and renumbers quality");
    edited=s.trafficLights(); edited.push_back(TrafficLight{{60,1},{60,25}}); s.setTrafficLights(edited);
    edited=s.trafficLights(); std::swap(edited[0],edited[1]); s.setTrafficLights(edited);
    need(s.trafficLightQueueStatus(id).qualityLine==1 && s.trafficLightQueueStatus(edited[0].id).lines.empty(),"detectors must remain attached to their own light");
    s.saveTrafficLights("monitoring-roundtrip.signals"); auto loaded=base(); loaded.loadTrafficLights("monitoring-roundtrip.signals");
    need(loaded.trafficLights()[1].queueDetectors.size()==3 && loaded.trafficLights()[1].id==id && loaded.trafficLights()[1].queueWindowSeconds==4,"version 3 roundtrip must retain detector ownership and settings");
    need(loaded.trafficLightQueueStatus(id).qualityLine==0 && loaded.trafficLightQueueStatus(id).lines[0].observedSeconds==0,"load restarts measurements, does not restore live occupancy");
    std::remove("monitoring-roundtrip.signals");
    edited=s.trafficLights(); edited[1].queueWindowSeconds=2; s.setTrafficLights(edited);
    need(s.trafficLightQueueStatus(id).lines[0].observedSeconds==0,"window edit starts a fresh observation period");
    bool rejected=false; edited[1].queueThresholdPercent=101;
    try { s.setTrafficLights(edited); } catch(const std::invalid_argument&) { rejected=true; }
    need(rejected,"invalid detector threshold rejected");
    rejected=false; edited=s.trafficLights(); edited[1].queueDetectors.push_back(edited[1].queueDetectors[0]);
    try { s.setTrafficLights(edited); } catch(const std::invalid_argument&) { rejected=true; }
    need(rejected,"duplicate detector identity rejected");
    { std::ofstream old("monitoring-v2.signals"); old<<"TRAFFIC_SIGNALS 2 1\n9 50 1 50 25 10 2 12 0 1 0 0 0 0 0\n"; }
    loaded.loadTrafficLights("monitoring-v2.signals"); std::remove("monitoring-v2.signals");
    need(loaded.trafficLights().size()==1 && loaded.trafficLights()[0].queueDetectors.empty(),"version 2 files remain compatible");
    auto fractional=base(); add(fractional,{10,5});
    light.queueWindowSeconds=1.005; light.queueThresholdPercent=49; fractional.setTrafficLights({light});
    auto fractionalId=fractional.trafficLights()[0].id; fractional.tick(.5);
    actors(fractional)[0].center={30,5}; actors(fractional)[0].path={{30,5},{80,5}};
    fractional.tick(.505);
    need(near(fractional.trafficLightQueueStatus(fractionalId).lines[0].occupiedPercent,100*.5/1.005),"non-integral window must weight actual time");
    fractional.tick(.25);
    need(near(fractional.trafficLightQueueStatus(fractionalId).lines[0].occupiedPercent,100*.25/1.005),"partial oldest sample must be clipped precisely");
}
void deletion() {
    auto s=base(); s.setStuckVehicleSettings({false,30,true,1});
    add(s,{10,5},true); add(s,{10,15},false);
    need(s.openJunctionVehicles()==1,"junction occupancy excludes lane vehicles");
    s.tick(.9); need(s.removedStuckVehicles()==0,"do not delete before configured junction timeout");
    s.tick(.11);
    need(s.removedJunctionVehicles()==1 && s.removedStuckVehicles()==1 && s.removedQueueBlockers()==0,"junction and total deletion counters");
    need(s.openJunctionVehicles()==0 && s.entities().size()==1 && s.completed()==0,"junction deletion must not remove lane vehicles or count arrivals");
    need(s.stopStatistics().vehicles==2 && s.stopStatistics(false).vehicles==1,"deleted vehicle stop telemetry retained");
    auto red=base(); red.setStuckVehicleSettings({false,30,true,1});
    red.setTrafficLights({{{50,1},{50,25},.1,0,100,1,true}});
    add(red,{49,5},true); add(red,{49,15},false);
    for(auto& e:actors(red)) { e.signalLight=0; e.path={e.center}; e.pathHeadings={0}; }
    red.tick(1.1);
    need(red.removedJunctionVehicles()==1 && red.entities().size()==1 && !red.entities()[0].inOpenJunction,"junction timeout overrides signal wait only inside free junction");
    auto moving=base(); moving.setStuckVehicleSettings({false,30,true,1}); add(moving,{10,5},true);
    moving.tick(.7); actors(moving)[0].desiredSpeed=2;
    moving.tick(.4); need(!moving.entities().empty() && moving.entities()[0].junctionStationarySeconds==0,"actual movement resets junction timer");
    actors(moving)[0].desiredSpeed=0; actors(moving)[0].speed=0;
    moving.tick(.7); need(moving.removedJunctionVehicles()==0,"moving vehicle gets a fresh stationary period");
    moving.tick(.4); need(moving.removedJunctionVehicles()==1,"resumed stationary timer expires");
    auto disabled=base(); add(disabled,{10,5},true); disabled.tick(2);
    need(disabled.entities().size()==1 && disabled.removedStuckVehicles()==0,"junction policy can be disabled independently");
    disabled.setStuckVehicleSettings({false,30,true,1}); disabled.tick(.5);
    need(disabled.entities().size()==1,"enabling junction deletion starts a fresh period");
    actors(disabled)[0].inOpenJunction=false; disabled.tick(.6);
    need(disabled.entities()[0].junctionStationarySeconds==0,"leaving open junction clears timer");
    bool rejected=false;
    try { disabled.setStuckVehicleSettings({false,30,true,std::numeric_limits<double>::quiet_NaN()}); }
    catch(const std::invalid_argument&) { rejected=true; }
    need(rejected,"nonfinite junction timeout rejected");
}
}
int main() { try { detectors(); deletion(); std::cout<<"Junction deletion and rolling queue detector monitoring passed\n"; }
    catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; } }
