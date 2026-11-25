#include "traffic/NodeGraph.hpp"
#include "traffic/SumoBackend.hpp"
#include <iostream>
#include <stdexcept>
#include <set>
using namespace traffic;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
NodeGraph network(){NodeGraph g;g.explicitDestinations=true;for(Vec2 p:std::vector<Vec2>{{10,40},{65,40},{100,40},{180,40},{100,100},{180,100}})g.addNode(p);
    g.nodes[0].spawn=true;g.nodes[3].destination=g.nodes[5].destination=true;
    g.connect(0,1,4,6);g.connections[0].roadType=1;
    g.connect(2,3,4,6);g.connections[1].roadType=2;g.connections[1].laneCount=2;
    g.connect(4,5,4,6);g.connections[2].roadType=1;
    g.setLaneTurnRule(0,{0,0,{{2,{1},{3},1,0}}});return g;
}
int physical(const std::vector<DirectedLane>& lanes,int road,int direction,int lane){for(std::size_t i=0;i<lanes.size();++i)if(lanes[i].roadGroup==road && lanes[i].travelDirection==direction && lanes[i].roadLane==lane)return static_cast<int>(i);throw std::runtime_error("Missing test lane");}
int main(int argc,char** argv){try{
    bool sumo=argc>1;auto g=network();auto lanes=compileRoads(g).lanes;int source=physical(lanes,0,0,0),target=physical(lanes,1,0,1),forbidden=physical(lanes,2,0,0),opposite=physical(lanes,1,1,1);
    require(lanes[source].successors.size()==1,"Semantic source gained an unchosen exit");auto& link=lanes[lanes[source].successors[0]];
    require(link.semanticTransfer && link.openJunction>=0 && link.successors==std::vector<int>{target} && link.movementPriority==3,"Nonlocal directed target did not compile as a free transfer");
    require(!g.turnAllowed(0,0,0,1,1,1),"Opposite two-way direction was silently allowed");
    g.save("semantic-roundtrip.nodes");auto restored=NodeGraph::load("semantic-roundtrip.nodes");auto& saved=restored.connections[0].laneTurns[0].exits[0];require(saved.road==1 && saved.direction==0 && saved.node==2 && saved.priorities[0]==3,"Semantic target did not persist");restored.pruneLaneTurns();require(!restored.connections[0].laneTurns[0].exits.empty(),"Nonlocal semantic target was pruned");
    Simulation sim(Map(200,140,std::vector<Tile>(28000,Tile::Road)),42);sim.setLanes(lanes);sim.setTrafficFlowSettings({true,false,.4,.15});sim.setStuckVehicleSettings({false,30,false,60});sim.setTrafficRecoverySettings({false,1,25});
    auto profile=defaultProfiles()[2];profile.length={2,0};profile.width={1,0};profile.motion={6,0,2,0};sim.setProfile(VehicleKind::Car,profile);
#ifdef _WIN32
    std::unique_ptr<SumoBackend> backend;if(sumo)backend=std::make_unique<SumoBackend>(sim,g,42);
#endif
    require(!sim.spawnOnLane(source,VehicleKind::Car,forbidden),"Spawn accepted an unreachable trip destination");
    require(!sim.spawnOnLane(source,VehicleKind::Car,opposite),"Spawn accepted the unchosen opposite target direction");
    require(sim.entities().empty(),"A rejected destination left a vehicle on the road");
    require(sim.spawnOnLane(source,VehicleKind::Car,target),"Nonlocal permitted destination could not spawn");
    bool crossed=false;for(int i=0;i<650;++i){sim.tick(.1);for(auto& e:sim.entities())if(e.inOpenJunction)crossed=true;}
    require(crossed && sim.completed()==1,"Semantic transfer did not drive across the gap and complete its trip");require(!sim.physicalCollisionEvents(),"Semantic transfer collided");
#ifdef _WIN32
    if(backend)require(!backend->collisions() && !backend->teleports(),"Semantic transfer used collision/teleport recovery");
#endif
    if(!sumo){Map blocked(200,140,std::vector<Tile>(28000,Tile::Road));for(int y=0;y<140;++y)blocked.tiles[blocked.index({82,y})]=Tile::Wall;blocked.rebuildFeatures();Simulation unreachable(blocked,42);unreachable.setLanes(lanes);unreachable.setTrafficFlowSettings({true,false,.4,.15});unreachable.setProfile(VehicleKind::Car,profile);require(!unreachable.spawnOnLane(source,VehicleKind::Car,target) && unreachable.entities().empty(),"Semantic permission bypassed a wall or spawned without a feasible crossing");}
    // Topological destination choice is exercised independently of insertion.
    for(int seed=0;seed<12 && !sumo;++seed){Simulation random(Map(200,140,std::vector<Tile>(28000,Tile::Road)),seed);random.setLanes(lanes);random.setTrafficFlowSettings({true,false,.4,.15});random.setProfile(VehicleKind::Car,profile);require(random.spawnOnLane(source,VehicleKind::Car),"Reachable random destination did not spawn");require(random.entities()[0].networkRoute.back()==target,"Random birth selected an unreachable or unchosen destination");}
    std::cout<<(sumo?"SUMO":"Native")<<" semantic nonlocal transfer, target lane/direction, unreachable destination rejection, random destination filtering and persistence passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
