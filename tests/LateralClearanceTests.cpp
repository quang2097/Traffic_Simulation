#include "traffic/NodeGraph.hpp"
#include "traffic/Geometry.hpp"
#include <iostream>
#include <stdexcept>
using namespace traffic;
void need(bool b,const char* s){if(!b)throw std::runtime_error(s);}
int main(){try{
// Rounded coordinates from the seed-7 blockage. A 0.1 mm separating step
// used to be rejected by the iterative clearance test; a 15 cm step passed.
Footprint closeA{{50.8509,61.4205},-.66091,1.95267,.744706};
Footprint closeB{{48.4952,61.635},.448363,1.9433,.667584};
auto separating=closeA;separating.center=separating.center+Vec2{std::cos(closeA.heading),std::sin(closeA.heading)}*.0001;
need(geometry::motionClearance(closeA,separating,closeB,.337392,true),"tiny separating motion falsely overlaps");
auto approaching=closeA;approaching.center=approaching.center-Vec2{std::cos(closeA.heading),std::sin(closeA.heading)}*.0001;
need(!geometry::motionClearance(closeA,approaching,closeB,.337392,true),"recovery worsens an existing short gap");
NodeGraph g;g.addNode({10,30});g.addNode({80,30});g.connect(0,1,4,6);g.connections[0].roadType=1;Simulation sim(Map(100,60,std::vector<Tile>(6000,Tile::Road)),42);sim.setLanes(g.lanes());sim.setTrafficFlowSettings({true,true,.4,.15});sim.setTrafficRecoverySettings({false,.6});sim.setStuckVehicleSettings({false,30});sim.setProfile(VehicleKind::Bike,{{1.9,0},{.6,0},{.05,0},{4,0,2,0},1});need(sim.spawnOnLane(0,VehicleKind::Bike)&&sim.spawnOnLane(0,VehicleKind::Bike),"fixture insertion failed");auto& es=const_cast<std::vector<Entity>&>(sim.entities());for(int i=0;i<2;++i){auto& e=es[i];e.center={14.,30.+i*.7};e.heading=0;e.path={e.center,{60,e.center.y}};e.pathHeadings={0,0};e.next=1;e.segmentProgress=0;e.goal=e.path.back();e.speed=0;e.lateralOffset=i*.7;e.sublaneAfter=e.laneChangeAfter=1e9;}
need(clearance(es[0].footprint(),es[1].footprint())>.05,"fixture preferred gap invalid");
for(int i=0;i<200;++i){sim.tick(.01);need(clearance(es[0].footprint(),es[1].footprint())>=.05-1e-7,"legal packed pair lost physical spacing");}need(es[0].center.x>15 && es[1].center.x>15,"legal packed pair blocked by desired planning gap");std::cout<<"Legal preferred clearances remain movable below the desired lateral planning buffer\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
