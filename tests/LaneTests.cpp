#include "traffic/Demo.hpp"
#include "traffic/NodeGraph.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace traffic;
void need(bool v,const char* s) { if(!v) throw std::runtime_error(s); }
int main() {
 try {
    NodeGraph nodes; int a=nodes.addNode({5,20}),b=nodes.addNode({35,20}),c=nodes.addNode({65,40}),isolated=nodes.addNode({10,50});
    nodes.connect(a,b,12,6); nodes.connect(b,c,12,6); nodes.connect(a,b);
    need(nodes.connections.size()==2 && !nodes.nodes[a].spawn && !nodes.nodes[b].spawn,"new connections created automatic spawn points or duplicate edges");
    nodes.nodes[b].spawn=true; nodes.nodes[b].rate=2.5; nodes.connections[0].smoothing=2.25;
    std::vector<int> selection;
    need(nodes.extendSelection(selection,a) && !nodes.extendSelection(selection,c) && nodes.extendSelection(selection,b) && nodes.extendSelection(selection,c),"path selection accepted disconnected nodes");
    nodes.save("node-roundtrip.nodes"); auto nodeCopy=NodeGraph::load("node-roundtrip.nodes");
    need(nodeCopy.nodes.size()==4 && !nodeCopy.nodes[isolated].spawn && nodeCopy.nodes[b].spawn && nodeCopy.nodes[b].rate==2.5 && nodeCopy.connections[0].smoothing==2.25,"node network settings did not persist");
    nodeCopy.sever(b,c); need(nodeCopy.nodes.size()==4 && nodeCopy.connection(b,c)<0,"sever deleted a node or retained the connection");
    Simulation explicitGraph(Map(100,60,std::vector<Tile>(6000,Tile::Road)),42); explicitGraph.setLanes(nodeCopy.lanes());
    need(explicitGraph.lanes()[0].successors.empty(),"severed connection was recreated");
    NodeGraph near; near.addNode({5,10}); near.addNode({30,10}); near.addNode({30.2,10}); near.addNode({90,10}); near.connect(0,1); near.connect(2,3);
    explicitGraph.setLanes(near.lanes()); need(explicitGraph.lanes()[0].successors.empty(),"nearby unconnected nodes were implicitly connected");
    auto smoothMap=Map(100,60,std::vector<Tile>(6000,Tile::Road));
    Footprint car{{10,10},0,4,1.8}; std::vector<Vec2> corner{{10,10},{40,10},{40,40}};
    auto sharp=curvePath(smoothMap,car,corner,0,{1,.5,1}),rounded=curvePath(smoothMap,car,corner,0,{1,2,1});
    need(!sharp.turns.empty() && !rounded.turns.empty() && rounded.points[rounded.turns[0].begin].x<sharp.points[sharp.turns[0].begin].x-1,"turn smoothing did not increase the arc radius");
    int upper=0,lower=0;
    for(unsigned seed=1;seed<=32;++seed) {
        Simulation choices(Map(100,60,std::vector<Tile>(6000,Tile::Road)),seed);
        choices.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{5,0,2,0},1});
        choices.setLanes({{{{4,30},{30,30}},10,6,true,{1,2}},
                          {{{30,30},{60,30}},10,6,false,{}},
                          {{{30,30},{55,45},{95,45}},10,6,false,{}}});
        need(choices.spawnOnLane(0,VehicleKind::Car),"random endpoint insertion failed");
        if(choices.entities()[0].laneRoute.back()==1) ++upper; else ++lower;
    }
    need(upper>5 && lower>5,"endpoint choice always favored the short route");
    Simulation branch(Map(100,60,std::vector<Tile>(6000,Tile::Road)),42);
    branch.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{5,0,2,0},1});
    branch.setStuckVehicleSettings({false,30});
    branch.setLanes({{{{3,20},{35,20},{65,20},{97,20}},6,6,true,{}},
                     {{{35,20},{50,30},{65,20}},6,6,false,{}}});
    need(branch.lanes().size()==4,"shared vertices did not split the main lane");
    need(branch.lanes()[0].successors.size()==2,"interior connection did not create a branch");
    need(branch.lanes()[3].successors==std::vector<int>({2}),"branch did not merge back into main lane");
    Simulation merge(Map(100,60,std::vector<Tile>(6000,Tile::Road)),42);
    merge.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{5,0,2,0},1});
    merge.setStuckVehicleSettings({false,30});
    merge.setLanes({{{{4,18},{28,18},{48,30}},7,6,true,{}},
                    {{{4,42},{28,42},{48,30}},7,6,true,{}},
                    {{{48,30},{96,30}},7,6,false,{}}});
    need(merge.spawnOnLane(0,VehicleKind::Car) && merge.spawnOnLane(1,VehicleKind::Car),"merge insertion failed");
    for(int step=0;step<12000 && !merge.entities().empty();++step) {
        merge.tick(.01);
        const auto& actors=merge.entities();
        if(actors.size()==2) need(clearance(actors[0].footprint(),actors[1].footprint())>=.5-1e-5,"merge violated minimum spacing");
    }
    for(const auto& e:merge.entities()) std::cerr<<"merge "<<e.id<<" "<<e.center.x<<","<<e.center.y<<" speed "<<e.speed<<" heading "<<e.heading<<" reason "<<stopReasonName(merge.stopReason(e))<<'\n';
    need(merge.completed()==2 && merge.removedStuckVehicles()==0,"merge traffic failed to drain without deletion");
    Simulation s(Map(100,20,std::vector<Tile>(2000,Tile::Road)),42);
    s.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{7,0,2,0},1});
    s.setLanes({{{{3,7},{97,7}},3.5,8,true,{}},{{{97,12},{3,12}},3.5,8,true,{}}});
    s.setStuckVehicleSettings({false,30}); int entered=0;
    for(int step=0;step<12000;++step) {
        if(step<3000 && step%100==0) entered+=s.spawnOnLane((step/100)%2,VehicleKind::Car);
        s.tick(.01);
        if(step==1) need(s.setEntityMotion(s.entities().front().id,20,2),"live lane speed override failed");
        for(const auto& e:s.entities()) {
            need(e.speed<=8+1e-6,"live vehicle settings bypassed the lane speed cap");
            need(std::abs(std::sin(e.heading))<1e-5,"lane insertion or motion is not parallel");
            need(std::abs(e.center.y-(e.laneRoute.front()==0?7:12))<.01,"crossed into opposing lane");
            for(const auto& o:s.entities()) if(e.id<o.id) need(clearance(e.footprint(),o.footprint())>=.5-1e-5,"lane collision");
        }
        if(step>3000 && s.entities().empty()) break;
    }
    need(entered>15 && s.completed()==static_cast<unsigned>(entered),"bidirectional queue failed to drain");
    s.setLaneTurnDeceleration(0,1.25); s.setLaneTurnDeceleration(1,5);
    s.setPointBraking({3,7},.75);
    s.saveLanes("lane-roundtrip.lanes"); Simulation restored(s.map(),42); restored.loadLanes("lane-roundtrip.lanes");
    need(restored.lanes()[0].turnDeceleration==1.25 && restored.lanes()[1].turnDeceleration==5,"per-lane turn braking was not preserved");
    need(restored.lanes()[0].pointBraking.front()==.75,"point braking was not preserved");
    auto configured=restored.lanes(); configured[0].spawnRate=2.75; configured[1].spawnRate=0;
    restored.setLanes(configured); restored.saveLanes("lane-roundtrip.lanes");
    Simulation rateCopy(s.map(),42); rateCopy.loadLanes("lane-roundtrip.lanes");
    need(rateCopy.lanes()[0].spawnRate==2.75 && rateCopy.lanes()[1].spawnRate==0,"per-entry spawn rates were not preserved");
    need(restored.lanes().size()==2 && restored.lanes()[1].points.front().x==97,"lane network save/load lost direction");
    Simulation connected(Map(100,30,std::vector<Tile>(3000,Tile::Road)),42);
    connected.setLanes({{{{3,10},{20,10}},4,8,true,{1}},{{{20,10},{40,10}},4,8,false,{2,3}},
        {{{40,10},{20,10}},4,8,false,{1}},{{{40,10},{97,10}},4,8,false,{}}});
    need(connected.spawnOnLane(0),"routing failed to find exit beyond a cyclic branch");
    need(connected.entities()[0].laneRoute==std::vector<int>({0,1,3}),"lane routing chose a cycle or wrong-way continuation");
    for(int i=0;i<6000 && !connected.entities().empty();++i) connected.tick(.01);
    need(connected.completed()==1,"connected lane route did not reach its exit");
    Simulation narrow(Map(100,20,std::vector<Tile>(2000,Tile::Road)),42);
    narrow.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{1.5,0},{7,0,2,0},1});
    narrow.setLanes({{{{3,7},{97,7}},3,8,true,{}},{{{97,10},{3,10}},3,8,true,{}}});
    need(narrow.spawnOnLane(0) && !narrow.spawnOnLane(1),"incompatible opposing bodies must wait at insertion instead of meeting head-on");
    for(int i=0;i<5000 && !narrow.entities().empty();++i) narrow.tick(.01);
    need(narrow.completed()==1 && narrow.spawnOnLane(1),"deferred direction must become available after the incompatible vehicle clears");
    for(auto seed:{7u,42u,123u}) {
    Simulation demo(makeDemoMap(),seed); auto p=vietnameseCityProfiles();
    for(int k=0;k<vehicleKindCount;++k) demo.setProfile(static_cast<VehicleKind>(k),p[k]);
    demo.setLanes(makeDemoLanes()); demo.setStuckVehicleSettings({false,30}); entered=0;
    auto urban=demo.urbanTrafficSettings(); urban.enabled=true; demo.setUrbanTrafficSettings(urban);
    auto turns=demo.turnNegotiationSettings(); turns.enabled=true; demo.setTurnNegotiationSettings(turns);
    auto tug=demo.tugSettings(); tug.enabled=true; demo.setTugSettings(tug);
    double clock=0;
    for(int step=0;step<24000;++step) {
        if(step<3600) { clock+=1./120; if(clock+1e-10>=1) { clock=std::max(0.0,clock-1); entered+=demo.spawnRandom(0); } }
        demo.tick(1./120);
        for(const auto& e:demo.entities()) for(const auto& o:demo.entities()) if(e.id<o.id)
            need(clearance(e.footprint(),o.footprint())>=std::min(e.preferredDistance,o.preferredDistance)-1e-5,"curved two-way collision");
        if(step>3600 && demo.entities().empty()) break;
    }
    if(!demo.entities().empty()) for(const auto& e:demo.entities()) std::cerr<<"stuck "<<e.id<<" lane="<<e.laneRoute[0]<<" x="<<e.center.x<<" stop="<<e.currentStopSeconds<<'\n';
    need(entered>=10 && demo.completed()==static_cast<unsigned>(entered),"curved two-way signal traffic failed to drain");
    std::cout<<"Two-way curved flow seed "<<seed<<": "<<entered<<" arrivals without deletion\n";
    }
 } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
