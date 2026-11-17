#include "traffic/NodeGraph.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <algorithm>
using namespace traffic;
void need(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
void setup(Simulation& s,double speed=8) { s.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{speed,0,2,0},1}); auto u=s.urbanTrafficSettings(); u.enabled=true; s.setUrbanTrafficSettings(u); s.setStuckVehicleSettings({false,30}); }
void safe(const Simulation& s) { for(const auto& e:s.entities()) { need(s.map().fits(e.footprint()),"decision model left mask"); for(const auto& o:s.entities()) if(e.id<o.id) need(clearance(e.footprint(),o.footprint())>=.5-1e-5,"decision model violated minimum gap"); } }
NodeGraph denseRoad() { NodeGraph g; g.addNode({10,25}); g.addNode({190,25}); g.connect(0,1,4,10); g.connections[0].roadType=1; g.connections[0].laneCount=2; for(int x=10;x<=190;++x) g.connections[0].shape.push_back({static_cast<double>(x),25}); return g; }
void place(Simulation& s,std::uint64_t id,double x,double speed) { for(auto& e:const_cast<std::vector<Entity>&>(s.entities())) if(e.id==id) { e.center.x=x; e.speed=speed; e.path={e.center,e.goal}; e.pathHeadings={e.heading,e.heading}; e.next=1; e.segmentProgress=0; ++e.routeRevision; e.progressAnchor=e.center; } }
int main() {
 try {
    NodeGraph bend; bend.addNode({10,20}); bend.addNode({50,65}); int turn=bend.addNode({50,20}); bend.connect(0,1,12,8); bend.connections[0].roadType=1; bend.connections[0].shape={{10,20},{50,20},{50,65}};
    auto marked=bend; marked.nodes[turn].turn=true; marked.nodes[turn].turnRadius=10; marked.nodes[turn].braking=1.25;
    marked.save("turn-node-roundtrip.nodes"); auto loaded=NodeGraph::load("turn-node-roundtrip.nodes"); need(loaded.nodes[turn].turn && loaded.nodes[turn].braking==1.25,"turn-node choice/rate did not persist");
    double cornerSpeeds[2]={100,100};
    for(int mode=0;mode<2;++mode) {
        Simulation s(Map(80,80,std::vector<Tile>(6400,Tile::Road)),42); setup(s); s.setLanes((mode?marked:bend).lanes()); need(s.spawnOnLane(0,VehicleKind::Car),"turn fixture insertion failed");
        for(int i=0;i<8000 && !s.entities().empty();++i) { s.tick(.01); safe(s); if(!s.entities().empty()) { const auto& e=s.entities()[0]; if(e.heading>.3 && e.heading<1.2) cornerSpeeds[mode]=std::min(cornerSpeeds[mode],e.speed); } }
        need(s.completed()==1,"turn fixture did not finish");
        s.saveLanes("turn-control-roundtrip.lanes"); Simulation copy(s.map(),42); copy.loadLanes("turn-control-roundtrip.lanes"); need(copy.lanes()[0].explicitTurns && copy.lanes()[0].turnControls.size()==static_cast<std::size_t>(mode),"turn controls were lost in lane snapshot");
    }
    need(cornerSpeeds[0]>7.5 && cornerSpeeds[1]<4 && cornerSpeeds[1]>.2,"ordinary shape nodes still brake, or marked turn did not slow down");
    Simulation live(Map(80,80,std::vector<Tile>(6400,Tile::Road)),42); setup(live); live.setLanes(bend.lanes()); need(live.spawnOnLane(0,VehicleKind::Car),"live turn spawn failed"); live.tick(1); auto before=live.entities()[0]; live.setTurnControls({{{50,20},10,1.25}}); need(live.entities()[0].id==before.id && length(live.entities()[0].center-before.center)<1e-9,"live turn change reset or moved the vehicle");
    NodeGraph straight; straight.addNode({10,20}); straight.addNode({50,20}); straight.addNode({90,20}); straight.connect(0,1,4,8); straight.connect(1,2,4,8); for(auto& r:straight.connections) r.roadType=1;
    Simulation through(Map(100,50,std::vector<Tile>(5000,Tile::Road)),42); setup(through); through.setLanes(straight.lanes()); need(through.spawnOnLane(0,VehicleKind::Car),"straight node spawn failed");
    double nodeSpeed=100;
    for(int i=0;i<5000 && !through.entities().empty();++i) { through.tick(.01); safe(through); for(const auto& e:through.entities()) if(e.center.x>40 && e.center.x<60) nodeSpeed=std::min(nodeSpeed,e.speed); }
    need(through.completed()==1 && nodeSpeed>7.5,"unmarked straight-through node imposed a junction slowdown");

    Simulation passing(Map(200,50,std::vector<Tile>(10000,Tile::Road)),7); setup(passing); passing.setLanes(denseRoad().lanes()); need(passing.spawnOnLane(1,VehicleKind::Car),"moderate leader insertion failed"); passing.setEntityMotion(passing.entities()[0].id,4,2); passing.tick(6); need(passing.spawnOnLane(1,VehicleKind::Car),"follower insertion failed"); auto follower=passing.entities().back().id; place(passing,follower,12.2,6);
    bool gain=false,right=false; double changeSpeed=0;
    for(int i=0;i<7500;++i) { passing.tick(.01); safe(passing); for(const auto& e:passing.entities()) if(e.id==follower) { if(e.laneChangeStatus=="changing: speed gain") { if(!gain) changeSpeed=e.speed; gain=true; } right|=e.laneChangeStatus=="changing: keep right"; } if(passing.entities().empty()) break; }
    if(!gain) { std::cerr<<"gain="<<gain<<" keepRight="<<right<<" changes="<<passing.totalLaneChanges()<<" attempts="<<passing.laneChangeAttempts()<<'\n'; for(const auto& e:passing.entities()) std::cerr<<e.id<<" x="<<e.center.x<<" status="<<e.laneChangeStatus<<'\n'; }
    need(gain && passing.totalLaneChanges()>0 && changeSpeed>4.8,"moving speed-gain passing on dense shape nodes did not occur before severe slowdown");
    need(right,"vehicle did not return right after passing");

    NodeGraph strategic; strategic.addNode({10,60}); strategic.addNode({100,60}); strategic.addNode({100,10}); strategic.nodes[1].openRadius=12; strategic.connect(0,1,4,8); strategic.connect(1,2,4,8); for(auto& r:strategic.connections) r.roadType=1; strategic.connections[0].laneCount=2;
    Simulation prepare(Map(120,100,std::vector<Tile>(12000,Tile::Road)),42); setup(prepare); prepare.setLanes(strategic.lanes()); need(prepare.spawnOnLane(1,VehicleKind::Car),"strategic spawn failed"); bool positioned=false;
    for(int i=0;i<6000 && !prepare.entities().empty();++i) { prepare.tick(.01); safe(prepare); for(const auto& e:prepare.entities()) positioned|=e.laneChangeStatus=="changing: strategic exit positioning"; }
    need(positioned && prepare.completed()==1,"vehicle did not prepare for its selected exit");

    Simulation cooperate(Map(200,50,std::vector<Tile>(10000,Tile::Road)),42); setup(cooperate); cooperate.setLanes(denseRoad().lanes()); need(cooperate.spawnOnLane(1,VehicleKind::Car),"cooperative leader spawn failed"); auto leader=cooperate.entities().back().id; cooperate.setEntityMotion(leader,1,2); place(cooperate,leader,55,1);
    need(cooperate.spawnOnLane(1,VehicleKind::Car),"cooperative requester spawn failed"); auto requester=cooperate.entities().back().id; place(cooperate,requester,40,6);
    need(cooperate.spawnOnLane(0,VehicleKind::Car),"cooperative rear spawn failed"); auto rear=cooperate.entities().back().id; place(cooperate,rear,32,8);
    for(int i=0;i<100;++i) { cooperate.tick(.01); safe(cooperate); }
    need(cooperate.cooperativeGapRequests()>0,"blocked merge did not request a cooperative gap");
    for(const auto& e:cooperate.entities()) if(e.id==rear) need(e.speed<8,"target-lane follower did not yield to the request");
    std::cout<<"Unmarked corner="<<cornerSpeeds[0]<<" m/s, marked="<<cornerSpeeds[1]<<"; dense-road speed gain, strategic positioning, keep-right and cooperative yielding passed\n";
 } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
