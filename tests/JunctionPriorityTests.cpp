#include "traffic/Simulation.hpp"
#include "traffic/NodeGraph.hpp"
#include "traffic/Demo.hpp"
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <cmath>
using namespace traffic;
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
namespace traffic {
struct JunctionPriorityTestAccess {
static void run(){
    Simulation sim(Map(100,100,std::vector<Tile>(10000,Tile::Road)),42);
    DirectedLane high,low;high.openJunction=low.openJunction=0;high.movementPriority=1;low.movementPriority=4;
    sim.lanes_={high,low};
    Entity a,b;a.id=1;b.id=2;a.center={20,50};b.center={50,20};a.length=b.length=2;a.width=b.width=1;
    a.path={{20,50},{80,50}};b.path={{50,20},{50,80}};a.pathHeadings={0,0};b.pathHeadings={1.570796,1.570796};
    a.networkRoute={0};b.networkRoute={1};a.networkStage=b.networkStage=0;a.inOpenJunction=b.inOpenJunction=true;
    sim.entities_={a,b};sim.preparePriorityYields();require(!sim.entities_[0].priorityYielding && sim.entities_[1].priorityYielding,"Smaller movement priority did not win");
    sim.seconds_=20;sim.preparePriorityYields();require(!sim.priorityGrant_ && sim.entities_[1].priorityYielding,"Default-off starvation relief changed priority");
    sim.setGridlockEscapeSettings({false,1,0,1.5,true});
    sim.entities_[1].currentStopSeconds=20;sim.entities_[1].desiredSpeed=4;sim.entities_[1].next=1;
    sim.updateGridlockEscapes();require(!sim.entities_[1].escapingGridlock && !sim.entities_[1].escapeOverlap,"Overlap recovery bypassed a priority yield");
    sim.entities_[1].escapingGridlock=true;sim.entities_[1].escapeStarted=0;
    sim.updateGridlockEscapes();require(!sim.entities_[1].escapeOverlap,"Existing reservation escape bypassed a priority yield");
    sim.entities_[1].escapingGridlock=false;
    auto flow=sim.flow_;flow.swapStarvedPriorities=true;flow.priorityWaitSeconds=3;sim.setTrafficFlowSettings(flow);sim.seconds_=21;sim.preparePriorityYields();
    require(sim.priorityGrant_==2 && !sim.entities_[1].priorityYielding && sim.entities_[0].priorityYielding,"Starved movement did not receive the priority grant");
    sim.entities_.erase(sim.entities_.begin()+1);sim.seconds_=22;sim.preparePriorityYields();require(!sim.priorityGrant_ && !sim.entities_[0].priorityYielding,"Priority grant or yield remained after winner left");
    sim.entities_={a,b};sim.lanes_[1].openJunction=7;sim.lanes_[1].semanticTransfer=true;sim.flow_.swapStarvedPriorities=false;sim.priorityCheckAfter_=0;sim.preparePriorityYields();require(sim.entities_[1].priorityYielding,"Semantic paths in separate junction groups ignored movement priority");
    sim.entities_={a,b};sim.entities_[1].path={{50,20},{50,35}};sim.priorityCheckAfter_=0;sim.preparePriorityYields();require(!sim.entities_[0].priorityYielding && !sim.entities_[1].priorityYielding,"Nonintersecting movements were stopped");
}
};}
int main(int argc,char**){try{
    JunctionPriorityTestAccess::run();
    {
        NodeGraph g;g.explicitDestinations=true;for(Vec2 p:std::vector<Vec2>{{50,50},{5,50},{95,50},{50,5},{50,95}})g.addNode(p);g.nodes[0].openRadius=15;
        for(auto ends:std::vector<std::pair<int,int>>{{1,0},{0,2},{3,0},{0,4}}){g.connect(ends.first,ends.second,4,4);g.connections.back().roadType=1;}
        g.nodes[1].spawn=g.nodes[3].spawn=true;g.nodes[2].destination=g.nodes[4].destination=true;
        g.setLaneTurnRule(0,{0,0,{{2,{0},{1}}}});g.setLaneTurnRule(2,{0,0,{{4,{0},{4}}}});auto lanes=compileRoads(g).lanes;
        Simulation sim(Map(100,100,std::vector<Tile>(10000,Tile::Road)),1);sim.setLanes(lanes);sim.setTrafficFlowSettings({true,false,.4,.15});sim.setStuckVehicleSettings({false,30,false,60});sim.setTrafficRecoverySettings({false,1,25});
        auto profile=defaultProfiles()[2];profile.length={2,0};profile.width={1,0};profile.motion={4,0,2,0};sim.setProfile(VehicleKind::Car,profile);
        require(sim.spawnOnLane(2,VehicleKind::Car,3) && sim.spawnOnLane(0,VehicleKind::Car,1),"Priority crossing vehicles could not spawn");
        bool observed=false;for(int i=0;i<600;++i){sim.tick(.1);if(sim.completed()==1){require(sim.entities().size()==1 && sim.entities()[0].id==1,"Lower-priority trip finished first");observed=true;}}
        require(observed && sim.completed()==2 && !sim.physicalCollisionEvents(),"Priority crossing stalled or collided");
    }
    auto original=NodeGraph::load(std::string(TRAFFIC_TEST_ROOT)+"/tests/fixtures/native-priority/custom.nodes");
    require(!original.missingLaneExits().empty(),"Unassigned user network passed mandatory validation");
    auto built=makeJunctionDemoGraph();require(built.missingLaneExits().empty(),"Built-in network is missing manual rules");
    auto before=compileRoads(built).lanes;int station=built.splitRoad(0,{40.572502136230469,65});built.nodes[station].roadStation=true;auto after=compileRoads(built).lanes;
    double boundary=0;for(auto& lane:after)if(lane.roadGroup==0 && lane.travelDirection==0)boundary=lane.points.back().x;
    require(boundary<41.01,"Signal station extended the approach into the free junction");
    for(auto& lane:compileRoads(original).lanes)if(lane.roadGroup>=0){auto& r=original.connections[lane.roadGroup];require(original.junctionAt(r.from)!=original.junctionAt(r.to),"Redundant physical lane remains inside free junction");}
    built=makeJunctionDemoGraph();built.connections[0].laneTurns[0].exits[0].priorities={7};built.save("priority-roundtrip.nodes");auto loaded=NodeGraph::load("priority-roundtrip.nodes");require(loaded.connections[0].laneTurns[0].exits[0].priorities[0]==7,"Per-path priority did not persist");
    std::cout<<"Priority ordering, default-off relief, starvation grant, non-conflicts, required exits, station geometry and persistence passed\n";
    if(argc>1)return 0;
    // Replay the supplied map with its original directions, without inventing
    // missing manual rules. Core replay is diagnostic; the UI refuses Play.
    for(int seed:{7,42,123}){
        Simulation sim(Map::fromPng(std::string(TRAFFIC_TEST_ROOT)+"/tests/fixtures/native-priority/custom-map.png",1),seed);auto lanes=compileRoads(original).lanes;sim.setLanes(lanes);
        sim.setTrafficFlowSettings({true,false,.4,.15});sim.setStuckVehicleSettings({false,30,false,60});sim.setTrafficRecoverySettings({false,1,25});
        auto routing=sim.routingSettings();routing.rtsStyle=false;routing.refreshMovingRoutes=false;routing.spreadAtSignals=false;sim.setRoutingSettings(routing);
        auto profiles=vietnameseCityProfiles();for(int kind=0;kind<vehicleKindCount;++kind)sim.setProfile(static_cast<VehicleKind>(kind),profiles[kind]);
        sim.loadTrafficLights(std::string(TRAFFIC_TEST_ROOT)+"/tests/fixtures/native-priority/custom.nodes.signals");
        for(int tick=0;tick<900;++tick){if(tick%20==0)for(std::size_t lane=0;lane<lanes.size();++lane)if(lanes[lane].entry)sim.spawnRandomOnLane(static_cast<int>(lane));sim.tick(.1);}
        std::cout<<"User map seed "<<seed<<": "<<sim.completed()<<" arrived, "<<sim.entities().size()<<" active, "<<sim.physicalCollisionEvents()<<" collisions, "<<sim.removedStuckVehicles()<<" deleted\n";
        require(sim.completed()>5,"Supplied native map produced no sustained arrivals");require(!sim.physicalCollisionEvents(),"Supplied native map had physical collisions");
    }
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
