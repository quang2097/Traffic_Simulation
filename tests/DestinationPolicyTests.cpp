#include "traffic/Demo.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>
using namespace traffic;
void need(bool b,const char* message) { if(!b) throw std::runtime_error(message); }
Simulation scene(const NodeGraph& graph,unsigned seed=42) {
    Simulation s(Map(130,130,std::vector<Tile>(16900,Tile::Road)),seed);
    s.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{7,0,2,0},1}); s.setLanes(graph.lanes()); return s;
}
int laneFrom(const Simulation& s,int node) {
    for(std::size_t i=0;i<s.lanes().size();++i) if(s.lanes()[i].sourceNode==node && s.lanes()[i].roadGroup>=0) return static_cast<int>(i);
    throw std::runtime_error("missing source lane");
}
int laneTo(const Simulation& s,int node) {
    for(std::size_t i=0;i<s.lanes().size();++i) if(s.lanes()[i].destinationNode==node && s.lanes()[i].destination && s.lanes()[i].roadGroup>=0) return static_cast<int>(i);
    throw std::runtime_error("missing destination lane");
}
int main() {
 try {
    auto graph=makeJunctionDemoGraph(); graph.nodes[1].excludeOwnDestination=true; graph.nodes[1].rate=.015;
    auto s=scene(graph);
    need(!s.spawnOnLane(laneFrom(s,1),VehicleKind::Car,laneTo(s,1)),"same-origin destination must be excluded");
    need(s.spawnOnLane(laneFrom(s,2),VehicleKind::Car,laneTo(s,1)),"another origin must still be able to finish at the excluded node");
    std::set<int> targets;
    for(unsigned seed=1;seed<=32;++seed) {
        auto trial=scene(graph,seed); need(trial.spawnOnLane(laneFrom(trial,1),VehicleKind::Car),"foreign destination selection failed");
        int destination=trial.lanes()[trial.entities()[0].networkRoute.back()].destinationNode;
        need(destination!=1,"random trip returned to its excluded origin"); targets.insert(destination);
    }
    need(targets.size()==3,"origin exclusion must retain all other destination choices");
    auto liveGraph=graph; liveGraph.nodes[1].excludeOwnDestination=false;
    auto live=scene(liveGraph); need(live.spawnOnLane(laneFrom(live,1),VehicleKind::Car,laneTo(live,1)),"return trips must remain available when enabled");
    auto oldRoute=live.entities()[0].networkRoute; live.setSourceDestinationPolicy(1,true);
    need(live.entities()[0].networkRoute==oldRoute && live.seconds()==0,"live policy edits must preserve active trips and clock");
    live.tick(3);
    need(!live.spawnOnLane(laneFrom(live,1),VehicleKind::Car,laneTo(live,1)),"live exclusion must apply to new trips");
    need(live.spawnOnLane(laneFrom(live,1),VehicleKind::Car,laneTo(live,2)),"live policy must retain foreign destinations");

    graph.save("origin-policy.nodes"); auto restored=NodeGraph::load("origin-policy.nodes");
    need(restored.nodes[1].excludeOwnDestination && std::abs(restored.nodes[1].rate-.015)<1e-12,"node policy and precise spawn rate must round trip");
    auto stored=scene(restored); stored.saveLanes("origin-policy.lanes");
    Simulation loaded(stored.map(),42); loaded.loadLanes("origin-policy.lanes");
    need(!loaded.spawnOnLane(laneFrom(loaded,1),VehicleKind::Car,laneTo(loaded,1)),"lane snapshots must retain source and destination identities");
    auto only=graph; for(std::size_t i=2;i<only.nodes.size();++i) only.nodes[i].destination=false;
    auto blocked=scene(only); need(!blocked.spawnOnLane(laneFrom(blocked,1),VehicleKind::Car),"no alternative must not silently fall back to the excluded origin");
    blocked.setSourceDestinationPolicy(1,false); need(blocked.spawnOnLane(laneFrom(blocked,1),VehicleKind::Car),"disabling exclusion must restore a valid return trip");
    {
        std::ofstream old("origin-v5.nodes");
        old<<"TRAFFIC_NODES 5 2 1 1\n10 20 1 1 3 0 0 0 0 12\n90 20 0 1 3 0 0 1 0 12\n0 1 3.5 7 1 1 1 0 0 0 0 0 0 0\n\n\n\n";
    }
    need(!NodeGraph::load("origin-v5.nodes").nodes[0].excludeOwnDestination,"older node files must preserve return-trip behavior");
    std::cout<<"Per-origin destination exclusion, random alternatives, live edits and backward-compatible persistence passed\n";
 } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
