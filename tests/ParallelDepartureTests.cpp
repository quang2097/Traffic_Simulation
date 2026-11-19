#include "traffic/Simulation.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace traffic;
namespace {
void need(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
std::vector<Entity>& actors(Simulation& s) { return const_cast<std::vector<Entity>&>(s.entities()); }
Simulation fixture() {
    DirectedLane entry; entry.points={{10,30},{30,30}}; entry.width=6; entry.speedLimit=6;
    entry.roadGroup=0; entry.sourceNode=17; entry.successors={1,2}; entry.inferConnections=false;
    DirectedLane crossing; crossing.points={{30,30},{60,30}}; crossing.width=6; crossing.speedLimit=6;
    crossing.entry=false; crossing.inferConnections=false; crossing.openJunction=0;
    crossing.junctionCenter={45,30}; crossing.junctionRadius=25; crossing.successors={3};
    DirectedLane secondCrossing=crossing; secondCrossing.points.back()={60,31.4}; secondCrossing.successors={4};
    DirectedLane left; left.points={{60,30},{90,30}}; left.width=1.1; left.speedLimit=6;
    left.roadGroup=1; left.entry=false; left.destination=true; left.inferConnections=false;
    DirectedLane right=left; right.points={{60,31.4},{90,31.4}}; right.roadGroup=2;
    DirectedLane alternate=entry; alternate.roadGroup=9; alternate.sourceNode=18;
    Simulation s(Map(100,60,std::vector<Tile>(6000,Tile::Road)),42);
    s.setLanes({entry,crossing,secondCrossing,left,right,alternate}); s.setCollisionWorkers(1);
    s.setTrafficFlowSettings({true,true,.4,.15}); s.setStuckVehicleSettings({false,30,false,60});
    s.setTrafficRecoverySettings({false,1,25});
    s.setProfile(VehicleKind::Motorbike,{{2,0},{.7,0},{.3,0},{6,0,2,0},1});
    need(s.spawnOnLane(0,VehicleKind::Motorbike,3),"first departure fixture spawn failed");
    need(s.spawnOnLane(0,VehicleKind::Motorbike,4),"second departure fixture spawn failed");
    need(s.entities()[0].spawnLane==0 && s.entities()[0].spawnNode==17,"spawn must record immutable lane and source node");
    for(int i=0;i<2;++i) {
        auto& e=actors(s)[i]; double y=30+i*1.4;
        e.center={i?28.6:28.8,y}; e.heading=0; e.goal={28.8,y};
        e.path={e.center,e.goal}; e.pathHeadings={0,0}; e.next=i?1:2; e.segmentProgress=0;
        e.speed=4; e.sublaneAfter=e.laneChangeAfter=1e9;
        e.junctionPlans.assign(3,{}); e.junctionPlans[1]={{{28.8,y},{63.2,y}},{0,0},{}};
        e.admissionPlan={};
    }
    return s;
}
void safe(const Simulation& s) {
    for(const auto& a:s.entities()) {
        need(s.map().fits(a.footprint()),"parallel departure left road mask");
        for(const auto& b:s.entities()) if(a.id<b.id)
            need(clearance(a.footprint(),b.footprint())>=std::min(a.preferredDistance,b.preferredDistance)-1e-5,"parallel departure violated preferred body clearance");
    }
}
}
int main() { try {
    auto parallel=fixture(); double minSpeed=4;
    for(int i=0;i<10;++i) { parallel.tick(.01); safe(parallel); minSpeed=std::min(minSpeed,parallel.entities()[1].speed); }
    need(parallel.junctionAdmissions()==2 && parallel.openJunctionVehicles()==2,"same-source side-by-side follower hesitated at junction entry");
    need(minSpeed>3.9,"compatible follower unnecessarily stopped at lane endpoint");
    for(int i=0;i<2000 && !parallel.entities().empty();++i) {
        parallel.tick(.01); safe(parallel);
        for(const auto& e:parallel.entities()) need(e.spawnLane==0 && e.spawnNode==17,"origin changed after junction or outgoing-lane handoff");
    }
    need(parallel.completed()==2 && parallel.removedStuckVehicles()==0,"parallel departures must reach their actual destinations without deletion");
    auto unrelated=fixture(); actors(unrelated)[1].spawnLane=5; actors(unrelated)[1].spawnNode=18;
    unrelated.tick(.1); need(unrelated.junctionAdmissions()==1,"unrelated origin incorrectly received parallel forecast relaxation");
    auto unknown=fixture(); for(auto& e:actors(unknown)) {e.spawnLane=-1;e.spawnNode=-1;}
    unknown.tick(.1); need(unknown.junctionAdmissions()==1,"unknown origins must not be grouped together");
    auto otherApproach=fixture(); actors(otherApproach)[1].networkRoute[0]=5;
    otherApproach.tick(.1); need(otherApproach.junctionAdmissions()==1,"shared trip source must not authorize a different current approach");
    auto crossing=fixture(); auto& second=actors(crossing)[1];
    second.junctionPlans[1]={{{28.8,31.4},{40,30},{63.2,31.4}},{0,0,0},{}};
    crossing.tick(.1); need(crossing.junctionAdmissions()==1,"same-origin intersecting paths must still yield"); safe(crossing);
    auto red=fixture(); red.setTrafficLights({{{30,25},{30,35},.1,0,100,1,true}});
    red.tick(.1); for(const auto& e:red.entities()) need(e.waitingForLight || e.speed<4,"parallel departure must still brake for red");
    std::cout<<"Parallel same-source admissions=2 within 0.1s, minimum follower speed="<<minSpeed
             <<"; both arrived; conflicting/unknown origins remain protected\n";
} catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; } }
