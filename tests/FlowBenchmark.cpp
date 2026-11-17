#include "traffic/NodeGraph.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <stdexcept>
using namespace traffic;
int main(int argc,char** argv) {
 try {
    std::string mode=argc>1?argv[1]:"predictive";
    unsigned seed=argc>2?static_cast<unsigned>(std::stoul(argv[2])):42;
    double duration=argc>3?std::stod(argv[3]):120;
    bool compact=mode=="compact";
    bool guided=mode=="guided",modern=mode!="legacy",packing=mode=="packed" || compact || guided;
    bool real=argc>4 && std::string(argv[4])=="wassup";
    NodeGraph g;
    for(Vec2 p:std::vector<Vec2>{{65,65},{10,65},{120,65},{65,10},{65,120}}) g.addNode(p);
    if(!guided) g.nodes[0].openRadius=24;
    for(auto pair:std::vector<std::pair<int,int>>{{1,0},{0,2},{3,0},{0,4}}) {
        g.connect(pair.first,pair.second,5,8); auto& r=g.connections.back(); r.roadType=2; r.laneCount=1;
    }
    Map map(130,130,std::vector<Tile>(16900,Tile::Road));
    if(real) { g=NodeGraph::load(std::string(TRAFFIC_TEST_ROOT)+"/examples/Wassup5.nodes"); g.nodes[0].openRadius=160; map=Map::fromPng(std::string(TRAFFIC_TEST_ROOT)+"/examples/Wassup5.png",.73); }
    auto roads=compileRoads(g);
    Simulation s(std::move(map),seed);
    s.setLanes(roads.lanes); s.setTrafficFlowSettings({modern,packing,.4,.15,compact}); s.setStuckVehicleSettings({false,30});
    auto urban=s.urbanTrafficSettings(); urban.enabled=true; s.setUrbanTrafficSettings(urban);
    auto tug=s.tugSettings(); tug.enabled=true; s.setTugSettings(tug);
    auto turns=s.turnNegotiationSettings(); turns.enabled=true; s.setTurnNegotiationSettings(turns);
    s.setTrafficRecoverySettings({true,.6});
    s.setProfile(VehicleKind::Motorbike,{{2,0},{.7,0},{.35,0},{7,0,2,0},1});
    s.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{7,0,2,0},1});
    std::vector<int> sourceNodes=real?std::vector<int>{2,8,9}:std::vector<int>{1,2,3,4};
    std::vector<int> entries(sourceNodes.size()),exits(sourceNodes.size());
    std::vector<std::vector<int>> reachable(sourceNodes.size());
    for(std::size_t n=0;n<sourceNodes.size();++n) {
        auto it=std::find(roads.originNodes.begin(),roads.originNodes.end(),sourceNodes[n]);
        if(it==roads.originNodes.end()) throw std::runtime_error("missing source");
        entries[n]=static_cast<int>(it-roads.originNodes.begin()); exits[n]=-1;
        for(std::size_t l=0;l<roads.lanes.size();++l) if(roads.originNodes[l]==0 && length(roads.lanes[l].points.back()-g.nodes[n+1].position)<6) exits[n]=static_cast<int>(l);
        if(!real && exits[n]<0) throw std::runtime_error("missing destination");
        std::vector<bool> seen(roads.lanes.size(),false); std::vector<int> todo{entries[n]}; seen[entries[n]]=true;
        for(std::size_t k=0;k<todo.size();++k) {
            int lane=todo[k];
            if(roads.lanes[lane].destination || (roads.lanes[lane].successors.empty() && roads.lanes[lane].allowTripEnd)) reachable[n].push_back(lane);
            for(int next:roads.lanes[lane].successors) if(!seen[next]) { seen[next]=true; todo.push_back(next); }
        }
        if(reachable[n].empty()) throw std::runtime_error("no reachable destination");
    }
    struct Request { int destination; VehicleKind kind; };
    std::vector<std::deque<Request>> pending(sourceNodes.size());
    int offered=0,inserted=0,completed60=0; std::uint64_t entries60=0; double minimum=1e9;
    const auto start=std::chrono::steady_clock::now();
    for(int step=0;step<static_cast<int>(duration*10);++step) {
        int period=real?50:20,spawnUntil=real?(argc>5?static_cast<int>(std::stod(argv[5])*10):600):400;
        if(step<spawnUntil && step%period==0) for(std::size_t source=0;source<sourceNodes.size();++source) {
            int sequence=step/period;
            int destination=real?reachable[source][(sequence+seed+source)%reachable[source].size()]:exits[(source+1+(sequence+seed)%3)%4];
            pending[source].push_back({destination,sequence%5==0?VehicleKind::Car:VehicleKind::Motorbike}); ++offered;
        }
        if(step%5==0) for(std::size_t source=0;source<sourceNodes.size();++source) if(!pending[source].empty()) {
            auto r=pending[source].front();
            if(s.spawnOnLane(entries[source],r.kind,r.destination)) { pending[source].pop_front(); ++inserted; }
        }
        s.tick(.1);
        if(step==599) { completed60=static_cast<int>(s.completed()); entries60=s.junctionAdmissions(); }
        if(step%10==0) for(const auto& e:s.entities()) {
            if(!s.map().fits(e.footprint())) throw std::runtime_error("benchmark vehicle outside mask");
            for(const auto& o:s.entities()) if(e.id<o.id) {
                double gap=clearance(e.footprint(),o.footprint()); minimum=std::min(minimum,gap);
                if(gap+1e-5<std::min(e.preferredDistance,o.preferredDistance)) throw std::runtime_error("benchmark clearance violation");
            }
        }
    }
    auto stopped=s.stopStatistics(true);
    if(argc>4 && !real) for(const auto& e:s.entities()) if(e.inOpenJunction || e.waitingForJunction) {
        std::cerr<<"id="<<e.id<<" xy="<<e.center.x<<','<<e.center.y<<" stage="<<e.networkStage<<" free="<<e.inOpenJunction<<" wait="<<e.waitingForJunction<<" speed="<<e.speed<<" stop="<<e.currentStopSeconds<<" blocked="<<e.blockedSeconds<<" lateral="<<e.lateralOffset<<" goal="<<e.goal.x<<','<<e.goal.y<<'\n';
        if(e.inOpenJunction) {
            for(std::size_t i=e.next-1;i<std::min(e.next+3,e.path.size());++i) std::cerr<<" path="<<e.path[i].x<<','<<e.path[i].y<<" heading="<<e.pathHeadings[i]<<'\n';
            auto pose=e.footprint(); double distance=0;
            for(std::size_t i=e.next;i<e.path.size();++i) {
                double segment=length(e.path[i]-pose.center);
                if(distance+segment>=.25) {
                    double f=(.25-distance)/segment; pose.center=pose.center+(e.path[i]-pose.center)*f; pose.heading+=angleDelta(pose.heading,e.pathHeadings[i])*f;
                    for(const auto& o:s.entities()) if(o.id!=e.id && clearance(pose,o.footprint())<std::min(e.preferredDistance,o.preferredDistance)+1e-7) std::cerr<<" obstacle="<<o.id<<" xy="<<o.center.x<<','<<o.center.y<<" reservation="<<o.reservesSublaneGap()<<'\n';
                    break;
                }
                distance+=segment; pose.center=e.path[i]; pose.heading=e.pathHeadings[i];
            }
        }
    }
    double wall=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    if(argc>4 && std::string(argv[4])=="verify" && s.completed()!=static_cast<unsigned>(offered)) throw std::runtime_error("predictive mixed-traffic regression failed to drain by its fixed deadline");
    std::cout<<mode<<','<<seed<<','<<duration<<','<<offered<<','<<inserted<<','<<s.completed()<<','<<s.entities().size()<<','<<(offered-inserted)<<','<<stopped.meanSeconds<<','<<wall<<','<<minimum<<','<<s.removedStuckVehicles()<<','<<completed60<<','<<s.junctionAdmissions()<<','<<entries60<<'\n';
 } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
