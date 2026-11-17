#include "traffic/Simulation.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace traffic;
void require(bool v,const char* text) { if(!v) throw std::runtime_error(text); }
int main() {
 try {
    Simulation s(Map(60,40,std::vector<Tile>(2400,Tile::Road)),42);
    s.setProfile(VehicleKind::Motorbike,{{2,0},{.7,0},{.25,0},{4,0,2,0},1});
    auto urban=s.urbanTrafficSettings(); urban.enabled=true; s.setUrbanTrafficSettings(urban);
    s.setStuckVehicleSettings({false,30}); s.setTrafficRecoverySettings({true,.6});
    require(s.spawnAt({20,20},{50,20},VehicleKind::Motorbike,0) && s.spawnAt({20,21.55},{50,21.55},VehicleKind::Motorbike,0),"angled neighbor spawns");
    auto& es=const_cast<std::vector<Entity>&>(s.entities());
    for(std::size_t i=0;i<es.size();++i) {
        auto& e=es[i]; e.heading=i?-.3:.3;
        auto route=curvePath(s.map(),e.footprint(),{e.center,e.goal});
        require(!route.points.empty(),"angled departure route");
        e.path=route.points; e.pathHeadings=route.headings; e.maneuvers=route.turns; e.next=1;
        e.currentStopSeconds=2; e.stoppedSeconds=2;
    }
    require(clearance(es[0].footprint(),es[1].footprint())>=.25,"initial pair is physically legal");
    auto red=s; red.setTrafficLights({{{21.07,1},{21.07,39},.1,0,100,1,true}});
    for(auto& e:const_cast<std::vector<Entity>&>(red.entities())) { e.signalLight=0; e.signalTarget=e.center; }
    red.planStoppedTraffic(true);
    require(red.trafficRecoveryPasses()==0 && red.seconds()==0,"an actual red signal protects an otherwise circular waiting group");
    s.planStoppedTraffic(true);
    require(s.trafficRecoveryPasses()==1,"circular physical blockage invokes coordination");
    require(s.trafficRecoveryRepairs()==1,"angled deadlock has a validated yield solution");
    require(s.seconds()==0 && es[0].center.x==20 && es[1].center.x==20,"deadlock planner cannot move actual vehicles");
    for(int step=0;step<6000 && !s.entities().empty();++step) {
        s.tick(.01);
        for(const auto& e:s.entities()) for(const auto& other:s.entities()) if(e.id<other.id)
            require(clearance(e.footprint(),other.footprint())>=.25-1e-5,"coordinated yield violates spacing");
    }
    require(s.completed()==2,"both angled neighbors arrive without deletion");
    std::cout<<"Angled physical deadlock recovered without deletion or teleportation\n";
 } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
