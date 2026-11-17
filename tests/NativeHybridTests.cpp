#include "traffic/NodeGraph.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace traffic;
bool predictive=false;
void need(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
void setup(Simulation& s) {
    s.setTrafficFlowSettings({predictive,false,.4,.15});
    s.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{6,0,2,0},1});
    s.setStuckVehicleSettings({false,30}); auto u=s.urbanTrafficSettings(); u.enabled=true; s.setUrbanTrafficSettings(u);
    s.setTrafficRecoverySettings({true,.6});
}
void safe(const Simulation& s) {
    for(const auto& e:s.entities()) {
        need(s.map().fits(e.footprint()),"hybrid vehicle left the road mask");
        for(const auto& other:s.entities()) if(e.id<other.id) need(clearance(e.footprint(),other.footprint())>=std::min(e.preferredDistance,other.preferredDistance)-1e-5,"hybrid vehicle violated physical spacing");
    }
}
int main(int argc,char**) {
 predictive=argc>1;
 try {
    std::cout<<std::unitbuf;
    NodeGraph graph; graph.addNode({10,50}); graph.addNode({50,50}); graph.addNode({90,50});
    graph.nodes[1].openRadius=12;
    graph.connect(0,1,4,7); graph.connect(1,2,4,7);
    for(auto& r:graph.connections) { r.roadType=1; r.laneCount=2; }
    auto network=compileRoads(graph);
    Simulation s(Map(100,100,std::vector<Tile>(10000,Tile::Road)),42); setup(s); s.setLanes(network.lanes);
    need(s.spawnOnLane(0,VehicleKind::Car),"native junction spawn failed");
    bool entered=false,rejoined=false;
    for(int step=0;step<8000 && !s.entities().empty();++step) {
        s.tick(.01); safe(s);
        if(!s.entities().empty()) { const auto& e=s.entities()[0]; entered|=e.inOpenJunction && e.laneRoute.empty(); rejoined|=entered && !e.inOpenJunction && !e.laneRoute.empty(); }
    }
    if(!s.entities().empty()) { const auto& e=s.entities()[0]; std::cerr<<"handoff x="<<e.center.x<<" y="<<e.center.y<<" stage="<<e.networkStage<<" next="<<e.next<<"/"<<e.path.size()<<" free="<<e.inOpenJunction<<'\n'; }
    need(entered && rejoined && s.completed()==1,"lane/free-space/lane handoff failed");
    Simulation blocked(Map(100,100,std::vector<Tile>(10000,Tile::Road)),42); setup(blocked); blocked.setLanes(network.lanes);
    need(blocked.spawnOnLane(0,VehicleKind::Car),"blocked-exit fixture spawn failed");
    int chosenExit=blocked.entities()[0].networkRoute[2];
    need(blocked.spawnOnLane(chosenExit,VehicleKind::Car),"exit blocker insertion failed");
    auto blockerId=blocked.entities().back().id;
    auto& stationary=const_cast<std::vector<Entity>&>(blocked.entities()).back(); stationary.desiredSpeed=0; stationary.speed=0;
    bool held=false;
    for(int i=0;i<1200;++i) { blocked.tick(.01); safe(blocked); held|=blocked.entities()[0].waitingForJunction; need(!blocked.entities()[0].inOpenJunction,"vehicle entered a blocked exit junction"); }
    need(held,"blocked exit did not hold the upstream vehicle");
    auto& actors=const_cast<std::vector<Entity>&>(blocked.entities()); actors.erase(std::remove_if(actors.begin(),actors.end(),[&](const auto& e){return e.id==blockerId;}),actors.end());
    for(int i=0;i<7000 && !blocked.entities().empty();++i) { blocked.tick(.01); safe(blocked); }
    need(blocked.completed()==1,"junction did not restart after its exit cleared");
    Simulation signals(s.map(),42); setup(signals); signals.setLanes(network.lanes);
    signals.setTrafficLights({{{25,35},{25,65},2,1,4,3,true}});
    need(signals.spawnOnLane(0,VehicleKind::Car),"hybrid signal fixture spawn failed");
    for(int i=0;i<350;++i) { signals.tick(.01); safe(signals); if(!signals.entities().empty()) need(!signals.entities()[0].inOpenJunction,"red signal slot was mistaken for a junction handoff"); }
    for(int i=0;i<9000 && !signals.entities().empty();++i) { signals.tick(.01); safe(signals); }
    need(signals.completed()==1,"hybrid approach did not restart after green");
    s.saveLanes("native-roundtrip.lanes"); Simulation snapshot(s.map(),42); snapshot.loadLanes("native-roundtrip.lanes");
    need(std::any_of(snapshot.lanes().begin(),snapshot.lanes().end(),[](const auto& lane){return lane.openJunction>=0 && lane.junctionRadius==12;}),"hybrid lane metadata was lost on save/load");
    NodeGraph asym=graph; asym.connections[0].roadType=2; asym.connections[0].reverseLaneCount=1; asym.connections[0].forwardWidths={3,5}; asym.connections[0].reverseWidths={4}; asym.connections[0].endWidth=3;
    asym.save("native-asymmetric.nodes"); auto restored=NodeGraph::load("native-asymmetric.nodes");
    need(restored.nodes[1].openRadius==12 && restored.connections[0].forwardWidths==std::vector<double>({3,5}) && restored.connections[0].reverseLaneCount==1,"asymmetric geometry did not persist");
    auto a=compileRoads(restored); need(a.lanes[0].pointWidths.front()>a.lanes[0].pointWidths.back(),"lane taper was not built");
    NodeGraph straight; straight.addNode({10,20}); straight.addNode({190,20}); straight.connect(0,1,4,8); straight.connections[0].roadType=1; straight.connections[0].laneCount=2;
    Simulation change(Map(200,40,std::vector<Tile>(8000,Tile::Road)),7); setup(change); change.setLanes(straight.lanes());
    need(change.spawnOnLane(0,VehicleKind::Car),"leader spawn failed");
    auto leader=change.entities()[0].id; change.setEntityMotion(leader,1,2);
    for(int i=0;i<2000;++i) change.tick(.01);
    need(change.spawnOnLane(0,VehicleKind::Car),"follower spawn failed"); auto follower=change.entities().back().id;
    bool changed=false; unsigned followerFinished=0;
    for(int i=0;i<6500;++i) {
        change.tick(.01); safe(change);
        for(const auto& e:change.entities()) if(e.id==follower) changed|=e.laneChanges>0;
        if(std::none_of(change.entities().begin(),change.entities().end(),[&](const auto& e){return e.id==follower;})) { followerFinished=1; break; }
    }
    if(!changed) for(const auto& e:change.entities()) std::cerr<<"lane change "<<e.id<<" x="<<e.center.x<<" y="<<e.center.y<<" speed="<<e.speed<<" count="<<e.laneChanges<<'\n';
    need(changed && followerFinished,"necessary safe overtaking did not finish");
    std::cout<<"Lane handoff, blocked-exit admission, persistence, asymmetric taper and safe overtaking passed\n";
    for(unsigned seed:{7u,42u,123u}) {
        NodeGraph crossing; crossing.addNode({8,50}); crossing.addNode({50,50}); crossing.addNode({92,50}); crossing.addNode({50,8}); crossing.addNode({50,92}); crossing.nodes[1].openRadius=14;
        crossing.connect(0,1,4,6); crossing.connect(1,2,4,6); crossing.connect(3,1,4,6); crossing.connect(1,4,4,6);
        for(auto& r:crossing.connections) { r.roadType=2; r.laneCount=1; }
        auto roads=compileRoads(crossing); Simulation flow(Map(100,100,std::vector<Tile>(10000,Tile::Road)),seed); setup(flow); flow.setLanes(roads.lanes);
        int enteredCount=0; bool freeMotion=false;
        for(int step=0;step<24000;++step) {
            if(step<2400 && step%300==0) for(std::size_t lane=0;lane<roads.originNodes.size();++lane) if(roads.originNodes[lane]>=0 && roads.originNodes[lane]!=1) enteredCount+=flow.spawnOnLane(static_cast<int>(lane),VehicleKind::Car);
            flow.tick(.01); safe(flow); for(const auto& e:flow.entities()) freeMotion|=e.inOpenJunction;
            if(step>=2400 && flow.entities().empty()) break;
        }
        if(!flow.entities().empty()) for(const auto& e:flow.entities()) std::cerr<<"cross seed="<<seed<<" id="<<e.id<<" stage="<<e.networkStage<<" free="<<e.inOpenJunction<<" pos="<<e.center.x<<","<<e.center.y<<" reason="<<stopReasonName(flow.stopReason(e))<<'\n';
        if(!flow.entities().empty()) for(const auto& e:flow.entities()) if(e.inOpenJunction) std::cerr<<"free goal="<<e.goal.x<<","<<e.goal.y<<" heading="<<e.heading<<" next="<<e.next<<"/"<<e.path.size()<<" wait="<<e.waitingForManeuver<<" detour="<<e.usingLocalDetour<<" recovery="<<e.cooperativeRecovery<<'\n';
        need(freeMotion && enteredCount>=8 && flow.completed()==static_cast<unsigned>(enteredCount) && flow.removedStuckVehicles()==0,"open intersection did not drain without deletion");
        std::cout<<"Native open junction seed "<<seed<<": "<<enteredCount<<" arrivals\n";
    }
 } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
