#include "traffic/NodeGraph.hpp"
#include "traffic/Geometry.hpp"
#include <iostream>
#include <stdexcept>
using namespace traffic;
namespace traffic {struct JunctionPriorityTestAccess {
    static void changes(Simulation& sim){sim.updateLaneChanges();}
    static bool clear(Simulation& sim,const Entity& actor,Footprint from,Footprint to){return sim.feasible(from,to,actor,true,false);}
};}
void need(bool value,const char* message){if(!value)throw std::runtime_error(message);}
std::vector<Entity>& actors(Simulation& sim){return const_cast<std::vector<Entity>&>(sim.entities());}
Simulation road(bool leader=true){
    NodeGraph graph;graph.addNode({10,30});graph.addNode({300,30});graph.nodes[1].destination=true;graph.connect(0,1,3.5,12);graph.connections[0].roadType=1;graph.connections[0].laneCount=2;
    Simulation sim(Map(320,60,std::vector<Tile>(19200,Tile::Road)),42);sim.setLanes(compileRoads(graph).lanes);sim.setCollisionWorkers(1);sim.setTrafficFlowSettings({true,false,.4,.15});sim.setStuckVehicleSettings({false,30,false,60});sim.setTrafficRecoverySettings({false,1,25});
    auto profile=defaultProfiles()[2];profile.length.stdDev=profile.width.stdDev=profile.motion.speedStdDev=profile.motion.accelerationStdDev=0;sim.setProfile(VehicleKind::Car,profile);
    need(sim.spawnOnLane(0),"Follower did not spawn");auto& follower=actors(sim).front();double y=sim.lanes()[0].points.front().y;follower.center={40,y};follower.heading=0;follower.path={follower.center,{295,y}};follower.pathHeadings={0,0};follower.goal=follower.path.back();follower.next=1;follower.segmentProgress=0;follower.speed=6;follower.desiredSpeed=10;follower.ageSeconds=10;
    if(leader){Entity front=follower;front.id=2;front.center={65,y};front.path={front.center,{295,y}};front.goal=front.path.back();front.speed=front.desiredSpeed=1;actors(sim).push_back(front);}
    return sim;
}
void safe(const Simulation& sim){for(const auto& a:sim.entities()){need(sim.map().fits(a.footprint()),"Rollback left the map");for(const auto& b:sim.entities())if(a.id<b.id)need(!geometry::violatesClearance(corners(a.footprint()),corners(b.footprint()),0),"Rollback overlapped another vehicle");}}
int main(){try{
    auto clearRoad=road(false);actors(clearRoad)[0].speed=actors(clearRoad)[0].desiredSpeed=8;JunctionPriorityTestAccess::changes(clearRoad);need(!clearRoad.entities()[0].changingLanes,"Empty-road keep-right weaving is still enabled");
    auto passing=road();actors(passing)[0].speed=1;JunctionPriorityTestAccess::changes(passing);need(passing.entities()[0].changingLanes,"A slow moving vehicle cannot begin useful overtaking");
    for(int i=0;i<600 && passing.entities()[0].changingLanes;++i){passing.tick(.01);safe(passing);}need(!passing.entities()[0].changingLanes && !passing.entities()[0].laneChangesDisabled && !passing.entities()[0].laneChangeOrigin,"Unobstructed overtaking did not complete and release its return position");
    auto sim=road();JunctionPriorityTestAccess::changes(sim);need(sim.entities()[0].changingLanes && sim.entities()[0].laneChangeOrigin,"A substantial passing opportunity no longer starts a safe lane change");
    auto& changing=actors(sim)[0];double sourceY=changing.laneChangeOrigin->center.y;std::size_t halfway=std::min<std::size_t>(20,changing.path.size()-2);changing.center=changing.path[halfway];changing.heading=changing.pathHeadings[halfway];changing.next=halfway+1;changing.segmentProgress=0;
    changing.speed=1;JunctionPriorityTestAccess::changes(sim);need(sim.entities()[0].changingLanes,"A moving lane change was aborted");
    actors(sim)[0].speed=0;double priorAge=sim.entities()[0].ageSeconds;sim.tick(.01);safe(sim);
    auto& returned=sim.entities()[0];need(!returned.changingLanes && returned.laneChangesDisabled && !returned.laneChangeOrigin,"Stopped merge did not abort permanently");need(std::abs(returned.center.y-sourceY)<1e-6 && returned.networkRoute[returned.networkStage]==0,"Stopped merge did not return fully to its original lane/route");need(returned.ageSeconds>=priorAge,"Rollback rewound trip telemetry");
    auto count=returned.laneChanges;for(int i=0;i<200;++i){sim.tick(.1);safe(sim);}need(sim.entities()[0].laneChanges==count && sim.entities()[0].laneChangesDisabled,"Aborted vehicle tried changing lanes again");
    auto occupied=road();JunctionPriorityTestAccess::changes(occupied);need(occupied.entities()[0].changingLanes,"Fallback setup did not begin a merge");
    auto origin=occupied.entities()[0].laneChangeOrigin;auto& merge=actors(occupied)[0];merge.center={52,occupied.lanes()[1].points.front().y};merge.heading=0;merge.speed=0;
    Entity blocker=occupied.entities()[1];blocker.id=3;blocker.center={52,sourceY};blocker.speed=0;blocker.path={blocker.center,{295,sourceY}};actors(occupied).push_back(blocker);
    auto from=blocker.footprint();from.center={32,sourceY};auto to=from;to.center={39,sourceY};need(!JunctionPriorityTestAccess::clear(occupied,blocker,from,to),"Traffic can occupy the protected rollback position");
    JunctionPriorityTestAccess::changes(occupied);safe(occupied);need(occupied.entities()[0].laneChangesDisabled && length(occupied.entities()[0].center-origin->center)<1e-7,"Occupied return point did not use the clear original position");
    auto blocked=road();JunctionPriorityTestAccess::changes(blocked);auto& moving=actors(blocked)[0];std::size_t middle=75;moving.center=moving.path[middle];moving.heading=moving.pathHeadings[middle];moving.next=middle+1;moving.segmentProgress=0;moving.speed=1;moving.currentStopSeconds=0;
    Entity obstacle=actors(blocked)[1];obstacle.id=3;obstacle.heading=moving.heading;obstacle.center=moving.center+Vec2{std::cos(moving.heading),std::sin(moving.heading)}*(moving.length+.00001);obstacle.speed=obstacle.desiredSpeed=0;obstacle.networkRoute={1};obstacle.laneRoute={1};obstacle.path={obstacle.center,obstacle.center};obstacle.pathHeadings={obstacle.heading,obstacle.heading};obstacle.next=1;actors(blocked).push_back(obstacle);
    blocked.tick(.01);safe(blocked);need(blocked.entities()[0].laneChangesDisabled && !blocked.entities()[0].changingLanes,"A merge stopped by traffic was not rolled back in the same physics step");
    std::cout<<"Conservative lane selection, stopped-merge rollback, protected fallback, collision clearance, telemetry and permanent trip disable passed\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
