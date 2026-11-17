#include "traffic/Simulation.hpp"
#include "traffic/Demo.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace traffic;
void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
Simulation open(bool urban) {
    Simulation s(Map(80,60,std::vector<Tile>(4800,Tile::Road)),42);
    s.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{6,0,2,0},1});
    s.setStuckVehicleSettings({false,30});
    auto r=s.routingSettings(); r.rtsStyle=false; s.setRoutingSettings(r);
    auto u=s.urbanTrafficSettings(); u.enabled=urban; s.setUrbanTrafficSettings(u);
    return s;
}
void safe(const Simulation& s) {
    for(const auto& e:s.entities()) {
        require(s.map().fits(e.footprint()),"urban vehicle left mask");
        for(const auto& other:s.entities()) if(e.id<other.id)
            require(clearance(e.footprint(),other.footprint())>=std::min(e.preferredDistance,other.preferredDistance)-1e-5,"urban collision or minimum gap violation");
    }
}
int main() {
 try {
    std::cout<<std::unitbuf;
    auto profiles=vietnameseCityProfiles(); double total=0;
    for(const auto& p:profiles) total+=p.weight;
    require(total==100 && profiles[1].weight==80,"Vietnamese preset shares");
    auto bend=open(true),reference=open(false);
    for(auto* s:{&bend,&reference}) {
        require(s->spawnAt({10,10},{30,40},VehicleKind::Car,0),"bend spawn");
        auto& e=const_cast<std::vector<Entity>&>(s->entities())[0];
        auto route=curvePath(s->map(),e.footprint(),{{10,10},{30,10},{30,40}});
        require(!route.turns.empty(),"bend fixture");
        e.path=route.points; e.pathHeadings=route.headings; e.maneuvers=route.turns;
        e.next=1; e.segmentProgress=0; e.speed=6;
    }
    bool slowed=false; double cornerSpeed=100;
    for(int step=0;step<2000 && !bend.entities().empty();++step) {
        bend.tick(.01); reference.tick(.01); safe(bend);
        if(!bend.entities().empty()) {
            const auto& e=bend.entities()[0];
            if(e.heading>.3 && e.heading<1.2) { slowed=true; cornerSpeed=std::min(cornerSpeed,e.speed); }
        }
    }
    require(slowed && cornerSpeed>.2 && cornerSpeed<4,"bend braking preserves rolling motion");
    require(bend.completed()==1,"urban bend reaches original endpoint");
    auto crossing=open(true);
    require(crossing.spawnAt({10,25},{60,25},VehicleKind::Car,0),"crossing priority spawn");
    require(crossing.spawnAt({25,10},{25,50},VehicleKind::Car,0),"crossing yield spawn");
    auto& cars=const_cast<std::vector<Entity>&>(crossing.entities());
    for(auto& e:cars) { e.speed=4; e.desiredSpeed=4; }
    bool earlyYield=false;
    for(int step=0;step<2500 && !crossing.entities().empty();++step) {
        crossing.tick(.01); safe(crossing);
        if(crossing.entities().size()==2 && cars[1].center.y<21 && cars[1].speed<3.8) earlyYield=true;
    }
    require(earlyYield && crossing.completed()==2,"predictive crossing yield lets both vehicles arrive");
    auto flow=open(true); auto r=flow.routingSettings(); r.rtsStyle=true; flow.setRoutingSettings(r);
    for(int k=0;k<vehicleKindCount;++k) flow.setProfile(static_cast<VehicleKind>(k),profiles[k]);
    flow.setTrafficLights({{{45,1},{45,59},4,1,4,4,true}});
    for(int i=0;i<12;++i)
        require(flow.spawnAt({6.+(i%3)*8,8.+(i/3)*11},{70,8.+(i/3)*11},i%4==0?VehicleKind::Car:VehicleKind::Motorbike,0),"mixed urban spawn");
    for(int step=0;step<12000 && !flow.entities().empty();++step) { flow.tick(.01); safe(flow); }
    require(flow.completed()==12 && flow.removedStuckVehicles()==0,"motorbike-heavy signal traffic drains without deletion");
    Simulation demo(makeDemoMap(),42); auto uDemo=demo.urbanTrafficSettings(); uDemo.enabled=true; demo.setUrbanTrafficSettings(uDemo);
    for(int k=0;k<vehicleKindCount;++k) demo.setProfile(static_cast<VehicleKind>(k),profiles[k]);
    auto rDemo=demo.routingSettings(); rDemo.localInterval=.35; rDemo.movingRefreshInterval=1.2; demo.setRoutingSettings(rDemo);
    demo.setTrafficRecoverySettings({true,.6}); demo.setStuckVehicleSettings({false,30});
    int admitted=0; double spawnClock=0;
    for(int step=0;step<24000;++step) {
        constexpr double dt=1./120;
        if(step<3600) {
            spawnClock+=dt;
            if(spawnClock+1e-10>=1) { spawnClock=std::max(0.,spawnClock-1); admitted+=demo.spawnRandom(3); }
        }
        demo.tick(dt); safe(demo);
        if(step%3600==0) std::cout<<"Urban demo t="<<demo.seconds()<<" admitted="<<admitted<<" completed="<<demo.completed()<<" active="<<demo.entities().size()<<'\n';
        if(step>=3600 && demo.entities().empty()) break;
    }
    if(demo.completed()!=static_cast<unsigned>(admitted)) for(const auto& e:demo.entities())
        std::cerr<<e.id<<" at "<<e.center.x<<','<<e.center.y<<" stop="<<e.currentStopSeconds<<" reason="<<stopReasonName(demo.stopReason(e))<<'\n';
    require(admitted>=10 && demo.completed()==static_cast<unsigned>(admitted),"Vietnamese startup preset drains every admitted vehicle without deletion");
    bool rejected=false; auto u=flow.urbanTrafficSettings(); u.predictionSeconds=0;
    try { flow.setUrbanTrafficSettings(u); } catch(const std::invalid_argument&) { rejected=true; }
    require(rejected,"invalid urban settings rejected");
    std::cout<<"Urban bend braking, crossing yield and mixed signal flow passed\n";
 } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
