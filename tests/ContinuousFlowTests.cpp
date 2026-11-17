#include "traffic/Demo.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace traffic;
int main() {
 try {
    std::cout<<std::unitbuf;
    Simulation s(makeDemoMap(),42); auto profiles=vietnameseCityProfiles();
    for(int k=0;k<vehicleKindCount;++k) s.setProfile(static_cast<VehicleKind>(k),profiles[k]);
    auto u=s.urbanTrafficSettings(); u.enabled=true; s.setUrbanTrafficSettings(u);
    auto tug=s.tugSettings(); tug.enabled=true; s.setTugSettings(tug);
    auto turns=s.turnNegotiationSettings(); turns.enabled=true; s.setTurnNegotiationSettings(turns);
    auto r=s.routingSettings(); r.localInterval=.35; r.movingRefreshInterval=1.2; s.setRoutingSettings(r);
    s.setTrafficRecoverySettings({true,.6}); s.setStuckVehicleSettings({false,30});
    int admitted=0; double spawnClock=0;
    for(int step=0;step<36000;++step) {
        constexpr double dt=1./120;
        if(step<7200) { spawnClock+=dt; if(spawnClock+1e-10>=1) { spawnClock=std::max(0.,spawnClock-1); admitted+=s.spawnRandom(3); } }
        s.tick(dt);
        for(const auto& e:s.entities()) {
            if(!s.map().fits(e.footprint())) throw std::runtime_error("continuous flow left road");
            for(const auto& other:s.entities()) if(e.id<other.id && clearance(e.footprint(),other.footprint())+1e-5<std::min(e.preferredDistance,other.preferredDistance))
                throw std::runtime_error("continuous flow violated clearance");
        }
        if(step%1200==0) std::cout<<"t="<<s.seconds()<<" active="<<s.entities().size()<<" done="<<s.completed()<<" admitted="<<admitted<<" repairs="<<s.trafficRecoveryRepairs()<<" contacts="<<s.tugContacts()<<'\n';
        if(step==6240) {
            std::size_t moving=0;
            for(const auto& e:s.entities()) {
                auto reason=s.stopReason(e);
                // Priority turns intentionally pause crossing traffic, including
                // occupants temporarily unable to clear. Only live, validated
                // claims count; a stale ID cannot excuse a stopped cluster.
                bool legalTurnWait=s.maneuverYieldTo(e) || s.ownsNegotiatedTurn(e) || reason==StopReason::BlockedExit;
                moving+=e.speed>Simulation::stoppedSpeedThreshold || legalTurnWait;
                if(e.center.x<22 && e.currentStopSeconds>=3 && !legalTurnWait) {
                    for(const auto& stuck:s.entities()) std::cerr<<"id="<<stuck.id<<" at="<<stuck.center.x<<','<<stuck.center.y<<" stop="<<stuck.currentStopSeconds<<" reason="<<stopReasonName(s.stopReason(stuck))<<" owner="<<s.ownsNegotiatedTurn(stuck)<<" yield="<<s.maneuverYieldTo(stuck)<<'\n';
                    throw std::runtime_error("screenshot regression: upstream vehicles remain stuck during green");
                }
            }
            if(moving*4<s.entities().size()*3)
                throw std::runtime_error("screenshot regression: majority of fleet is stationary without a verified turn wait at 52 seconds");
        }
        if(step==7200 && (s.completed()<20 || s.completed()*2<static_cast<unsigned>(admitted)))
            throw std::runtime_error("continuous flow throughput collapsed before spawning ended");
        if(step>=7200 && s.entities().empty()) break;
    }
    if(s.completed()!=static_cast<unsigned>(admitted)) throw std::runtime_error("continuous startup traffic did not drain");
    std::cout<<"Continuous Vietnamese startup flow passed: "<<admitted<<" arrivals without deletion\n";
 } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
