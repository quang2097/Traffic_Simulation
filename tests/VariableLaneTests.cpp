#include "traffic/NodeGraph.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace traffic;
void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
void setup(Simulation& s) {
    s.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{6,0,2,0},1}); s.setStuckVehicleSettings({false,30});
    auto urban=s.urbanTrafficSettings(); urban.enabled=true; s.setUrbanTrafficSettings(urban); s.setTrafficRecoverySettings({true,.6});
}
void safe(const Simulation& s) {
    for(const auto& e:s.entities()) {
        require(s.map().fits(e.footprint()),"variable-width vehicle left the mask");
        for(const auto& other:s.entities()) if(e.id<other.id) require(clearance(e.footprint(),other.footprint())>=.5-1e-5,"lane transition collision");
    }
}
int main() {
 try {
    NodeGraph width; width.addNode({10,30}); width.addNode({140,30}); width.connect(0,1,3,8);
    width.connections[0].roadType=1; width.connections[0].shape={{10,30},{70,30},{140,30}}; width.connections[0].shapeWidths={3,7,2};
    width.explicitDestinations=true; width.nodes[1].destination=true;
    auto compiled=width.lanes(); require(compiled[0].pointWidths==std::vector<double>({3,7,2}),"per-node width profile was replaced by a global taper");
    Simulation geometry(Map(160,60,std::vector<Tile>(9600,Tile::Road)),42); setup(geometry); geometry.setLanes(compiled);
    require(std::abs(geometry.laneWidthAt(0,{40,30})-5)<1e-9 && std::abs(geometry.laneWidthAt(0,{105,30})-4.5)<1e-9,"quadrilateral width interpolation is incorrect");
    width.save("variable-width.nodes"); auto restored=NodeGraph::load("variable-width.nodes");
    require(restored.explicitDestinations && restored.nodes[1].destination && restored.connections[0].shapeWidths==width.connections[0].shapeWidths,"point widths or explicit destinations were not saved");
    geometry.saveLanes("variable-width.lanes"); Simulation copy(geometry.map()); copy.loadLanes("variable-width.lanes");
    require(copy.lanes()[0].destination && copy.lanes()[0].pointWidths==compiled[0].pointWidths,"engine snapshot lost destination/width metadata");
    std::vector<Tile> taperMask(4000,Tile::Wall);
    for(int y=0;y<40;++y) for(int x=0;x<100;++x) { double widthAtX=8-6*std::clamp((x+.5-10)/80.,0.,1.); if(std::abs(y+.5-20)<widthAtX*.5+1.5) taperMask[y*100+x]=Tile::Road; }
    NodeGraph tapered; tapered.addNode({10,20}); tapered.addNode({90,20}); tapered.connect(0,1,8,8); tapered.connections[0].roadType=1; tapered.connections[0].shapeWidths={8,2};
    validateRoadMask(tapered.lanes(),Map(100,40,std::move(taperMask)));
    NodeGraph joined; joined.addNode({10,30}); joined.addNode({70,30}); joined.addNode({140,30});
    joined.connect(0,1,3,8); joined.connect(1,2,7,8);
    for(auto& road:joined.connections) road.roadType=1;
    joined.connections[0].shapeWidths={3,7}; joined.connections[1].shapeWidths={7,2};
    auto joins=joined.lanes();
    require(std::any_of(joins.begin(),joins.end(),[](const auto& lane){return lane.roadGroup<0 && !lane.entry && lane.width>5;}),"joining tapered sections used the distant narrowest width instead of the width at the join");

    NodeGraph detached; detached.explicitDestinations=true;
    detached.addNode({5,50}); detached.addNode({35,50}); detached.addNode({65,50}); detached.addNode({95,50}); detached.addNode({50,50});
    detached.nodes[4].openRadius=20; detached.nodes[3].destination=true;
    detached.connect(0,1,4,7); detached.connect(2,3,4,7);
    for(auto& road:detached.connections) road.roadType=1;
    auto network=compileRoads(detached);
    require(!network.lanes[0].allowTripEnd && !network.lanes[0].successors.empty(),"unconnected lane endpoint inside a free area is still a trip exit");
    Simulation trip(Map(100,100,std::vector<Tile>(10000,Tile::Road)),42); setup(trip); trip.setLanes(network.lanes);
    require(trip.spawnOnLane(0,VehicleKind::Car),"separately drawn junction lanes did not route to the destination");
    bool free=false; double lastX=0;
    for(int step=0;step<7000 && !trip.entities().empty();++step) {
        lastX=trip.entities()[0].center.x; trip.tick(.01); safe(trip);
        if(!trip.entities().empty()) free|=trip.entities()[0].inOpenJunction && trip.entities()[0].laneRoute.empty();
    }
    require(free && lastX>90 && trip.completed()==1,"vehicle despawned at the approach end instead of traversing the junction");
    NodeGraph pointGoal=detached; pointGoal.connections.pop_back(); pointGoal.nodes[3].destination=false;
    int freeGoal=pointGoal.addNode({58,58}); pointGoal.nodes[freeGoal].destination=true;
    Simulation openTrip(trip.map(),42); setup(openTrip); openTrip.setLanes(pointGoal.lanes());
    require(openTrip.spawnOnLane(0,VehicleKind::Car),"standalone destination inside the free area was not routable");
    bool reachedFree=false; double finalDistance=1e100;
    for(int i=0;i<7000 && !openTrip.entities().empty();++i) { const auto& e=openTrip.entities()[0]; finalDistance=length(e.center-pointGoal.nodes[freeGoal].position); reachedFree|=e.inOpenJunction; openTrip.tick(.01); safe(openTrip); }
    require(reachedFree && finalDistance<.3 && openTrip.completed()==1,"vehicle did not reach its free-space destination");

    NodeGraph dead; dead.explicitDestinations=true; dead.addNode({5,20}); dead.addNode({60,20}); dead.connect(0,1); dead.connections[0].roadType=1;
    Simulation noExit(Map(100,40,std::vector<Tile>(4000,Tile::Road)),42); setup(noExit); noExit.setLanes(dead.lanes());
    require(!noExit.spawnOnLane(0,VehicleKind::Car) && noExit.completed()==0,"unmarked dead end was silently treated as a trip destination");

    NodeGraph separate; separate.addNode({10,18}); separate.addNode({190,18}); separate.addNode({10,22.5}); separate.addNode({190,22.5});
    separate.connect(0,1,4,8); separate.connect(2,3,4,8); for(auto& road:separate.connections) road.roadType=1;
    Simulation passing(Map(200,40,std::vector<Tile>(8000,Tile::Road)),7); setup(passing); passing.setLanes(separate.lanes());
    require(passing.spawnOnLane(0,VehicleKind::Car),"separate lane leader spawn failed"); passing.setEntityMotion(passing.entities()[0].id,1,2);
    for(int i=0;i<2000;++i) passing.tick(.01);
    require(passing.spawnOnLane(0,VehicleKind::Car),"separate lane follower spawn failed"); auto follower=passing.entities().back().id;
    bool changed=false,arrived=false;
    for(int i=0;i<6500;++i) {
        passing.tick(.01); safe(passing);
        for(const auto& e:passing.entities()) if(e.id==follower) changed|=e.laneChanges>0;
        if(std::none_of(passing.entities().begin(),passing.entities().end(),[&](const auto& e){return e.id==follower;})) { arrived=true; break; }
    }
    require(changed && arrived,"vehicles cannot switch between separately drawn parallel lanes");
    std::cout<<"Per-node quadrilaterals, explicit destinations, detached free-junction lanes and cross-road overtaking passed\n";
 } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
