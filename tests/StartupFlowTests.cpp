#include "traffic/Demo.hpp"
#include "traffic/Geometry.hpp"
#include "traffic/SpawnClock.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <unordered_map>
using namespace traffic;
int main(int argc,char** argv) {
 try {
    std::cout<<std::unitbuf;
    for(double rate:{.5,1.,3.}) {
        double credit=0; int count=0;
        for(int frame=0;frame<14400;++frame) count+=consumeSpawnClock(credit,1./120,rate);
        if(count!=static_cast<int>(120*rate)) throw std::runtime_error("spawn clock changed the requested rate");
    }
    unsigned seed=argc>1?static_cast<unsigned>(std::stoul(argv[1])):42;
    int spawnSeconds=argc>2?std::stoi(argv[2]):180;
    auto graph=makeJunctionDemoGraph(); auto roads=compileRoads(graph);
    Simulation s(makeJunctionDemoMap(),seed); s.setLanes(roads.lanes);
    s.setTrafficFlowSettings({true,true,.4,.15}); s.setStuckVehicleSettings({false,30});
    auto urban=s.urbanTrafficSettings(); urban.enabled=true; s.setUrbanTrafficSettings(urban);
    auto routing=s.routingSettings(); routing.localInterval=.35; routing.movingRefreshInterval=1.2; s.setRoutingSettings(routing);
    auto tug=s.tugSettings(); tug.enabled=true; s.setTugSettings(tug);
    auto turns=s.turnNegotiationSettings(); turns.enabled=true; s.setTurnNegotiationSettings(turns);
    s.setTrafficRecoverySettings({true,.6});
    auto profiles=vietnameseCityProfiles();
    for(int i=0;i<vehicleKindCount;++i) s.setProfile(static_cast<VehicleKind>(i),profiles[i]);
    std::mt19937 spawnRandom(seed); std::vector<double> clocks(graph.nodes.size(),0);
    int admitted=0,offered=0; constexpr double dt=1./120;
    for(int step=0;step<(spawnSeconds+180)*120;++step) {
        if(step<spawnSeconds*120) for(std::size_t n=0;n<graph.nodes.size();++n) {
            const auto& node=graph.nodes[n]; if(!node.spawn) continue;
            if(!consumeSpawnClock(clocks[n],dt,node.rate)) continue;
            ++offered;
            std::vector<int> outgoing; for(std::size_t l=0;l<roads.originNodes.size();++l) if(roads.originNodes[l]==static_cast<int>(n)) outgoing.push_back(static_cast<int>(l));
            if(!outgoing.empty() && s.entities().size()<100) admitted+=s.spawnRandomOnLane(outgoing[std::uniform_int_distribution<std::size_t>(0,outgoing.size()-1)(spawnRandom)]);
        }
        std::unordered_map<std::uint64_t,std::pair<Vec2,double>> before;
        for(const auto& e:s.entities()) before.emplace(e.id,std::make_pair(e.center,e.speed));
        s.tick(dt);
        for(const auto& e:s.entities()) {
            const auto& old=before.at(e.id);
            if(length(e.center-old.first)>std::max(old.second,e.speed)*dt+e.acceleration*dt*dt+1e-6) throw std::runtime_error("startup lane/junction handoff skipped physical movement");
        }
        if(step%30==0) for(const auto& e:s.entities()) {
            if(!s.map().fits(e.footprint())) throw std::runtime_error("startup vehicle left mask");
            for(const auto& o:s.entities()) if(e.id<o.id && clearance(e.footprint(),o.footprint())+1e-5<std::min(e.preferredDistance,o.preferredDistance)) throw std::runtime_error("startup clearance violation");
        }
        if(step%3600==3599) std::cout<<"seed="<<seed<<" t="<<s.seconds()<<" admitted="<<admitted<<" done="<<s.completed()<<" active="<<s.entities().size()<<'\n';
        bool stalled=std::any_of(s.entities().begin(),s.entities().end(),[&](const Entity& e){ return e.inOpenJunction && e.currentStopSeconds>15 && s.seconds()-e.junctionEnteredAt>15; });
        if(stalled || (step>=spawnSeconds*120 && s.entities().empty())) break;
    }
    int diagnostics=0;
    if(!s.entities().empty()) for(const auto& e:s.entities()) if(diagnostics++<20) {
        std::cerr<<"id="<<e.id<<" kind="<<vehicleName(e.kind)<<" xy="<<e.center.x<<','<<e.center.y<<" stage="<<e.networkStage<<" lane="<<e.networkRoute[e.networkStage]<<" free="<<e.inOpenJunction<<" wait="<<e.waitingForJunction<<" next="<<e.next<<'/'<<e.path.size()<<" stop="<<e.currentStopSeconds<<" blocked="<<e.blockedSeconds<<" heading="<<e.heading<<" goal="<<e.goal.x<<','<<e.goal.y<<" body="<<e.length<<','<<e.width<<" reason="<<stopReasonName(s.stopReason(e))<<'\n';
        if(e.inOpenJunction && e.next<e.path.size()) {
            std::cerr<<" speed="<<e.speed<<" desired="<<e.desiredSpeed<<" accel="<<e.acceleration<<'\n';
            for(std::size_t j=e.next-1;j<std::min(e.next+4,e.path.size());++j) std::cerr<<" path["<<j<<"]="<<e.path[j].x<<','<<e.path[j].y<<" heading="<<e.pathHeadings[j]<<'\n';
            auto to=e.footprint(); auto d=e.path[e.next]-e.center; double f=std::min(1.,.02/std::max(1e-9,length(d))); to.center=e.center+d*f; to.heading=e.heading+angleDelta(e.heading,e.pathHeadings[e.next])*f;
            for(const auto& o:s.entities()) if(o.id!=e.id && !geometry::motionClearance(e.footprint(),to,o.footprint(),std::min(e.preferredDistance,o.preferredDistance),true)) std::cerr<<" blocked by "<<o.id<<'\n';
        }
    }
    if(s.seconds()+1e-6>=spawnSeconds && offered!=spawnSeconds*2) throw std::runtime_error("startup spawn clock produced duplicate or missing requests");
    if(admitted<80 || s.completed()!=static_cast<unsigned>(admitted) || s.removedStuckVehicles()) throw std::runtime_error("startup preset did not drain sustained Gaussian mixed traffic without deletion");
    std::cout<<"Startup preset drained "<<admitted<<" Gaussian mixed trips without deletion\n";
 } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
