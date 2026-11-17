#include "traffic/Simulation.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace traffic;
void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
void safe(const Simulation& s) {
    for(std::size_t i=0;i<s.entities().size();++i) {
        const auto& e=s.entities()[i]; require(s.map().fits(e.footprint()),"batch route left road");
        for(std::size_t j=i+1;j<s.entities().size();++j) require(clearance(e.footprint(),s.entities()[j].footprint())+1e-5>=std::min(e.preferredDistance,s.entities()[j].preferredDistance),"concurrent approaches violated physical gap");
    }
}
int main() {
    try {
        Simulation s(Map(60,24,std::vector<Tile>(1440,Tile::Road)),91);
        s.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{6,0,3,0},1});
        s.setProfile(VehicleKind::Bike,{{1.8,0},{.6,0},{.2,0},{4,0,3,0},1});
        s.setTrafficLights({{{30,1},{30,23},.5,0,30,.5,true}});
        for(double x:{24.,16.,8.}) require(s.spawnAt({x,12},{50,12},VehicleKind::Car,0),"batch queue car spawn");
        require(s.spawnAt({3,12},{50,12},VehicleKind::Bike,0),"batch queue bike spawn");
        auto before=s.entities(); double clock=s.seconds(); auto ticks=s.ticks();
        s.planSignalQueues(true);
        require(s.seconds()==clock && s.ticks()==ticks,"planning must not advance simulation clock or physics ticks");
        require(s.signalPlanningBatches()==1 && s.lastSignalPlanCount()==4,"all four approaches prepared in one frozen-clock planning pass");
        for(std::size_t i=0;i<s.entities().size();++i) {
            const auto& e=s.entities()[i];
            require(e.signalLight==0 && e.center.x==before[i].center.x && e.center.y==before[i].center.y && e.speed==before[i].speed && e.heading==before[i].heading && e.ageSeconds==0,"planning changes routes but not physical state or timers");
            for(std::size_t j=i+1;j<s.entities().size();++j) {
                const auto& other=s.entities()[j];
                require(clearance({e.signalTarget,e.signalStopHeading,e.length,e.width},{other.signalTarget,other.signalStopHeading,other.length,other.width})+1e-5>=std::min(e.preferredDistance,other.preferredDistance),"batch targets overlap");
            }
        }
        for(int step=0;step<50;++step) { s.tick(.01); safe(s); }
        int moving=0;
        for(std::size_t i=0;i<s.entities().size();++i) {
            const auto& e=s.entities()[i]; moving+=length(e.center-before[i].center)>.05 && e.speed>.1;
            require(e.signalRemaining>0,"concurrency checked before first vehicle reaches its stop");
        }
        require(moving==4,"all vehicles advance together before the leader finishes its approach");
        for(int step=0;step<1950;++step) { s.tick(.01); safe(s); for(const auto& e:s.entities()) for(Vec2 p:corners(e.footprint())) require(p.x<30+1e-6,"planned group crosses red stop line"); }
        require(s.entities().size()==4 && s.completed()==0,"holding positions are not destinations");
        s.setTrafficLights({}); s.planSignalQueues(true);
        for(int step=0;step<6000 && !s.entities().empty();++step) { s.tick(.01); safe(s); }
        require(s.completed()==4,"planned group resumes to original endpoints");

        // A queue larger than the collision-worker threshold exercises virtual
        // target broad-phase checks and row allocation beyond the old 3 rows.
        Simulation large(Map(120,3,std::vector<Tile>(360,Tile::Road)),42);
        large.setProfile(VehicleKind::Car,{{1,0},{1.8,0},{.1,0},{4,0,2,0},1});
        large.setTrafficLights({{{110,.01},{110,2.99},.1,0,100,1,true}});
        for(int i=0;i<36;++i) require(large.spawnAt({105.-2*i,1.5},{118,1.5},VehicleKind::Car,0),"large queue spawn");
        before=large.entities(); large.planSignalQueues(true);
        require(large.lastSignalPlanCount()==36 && large.seconds()==0,"entire long queue planned without time advance");
        for(std::size_t i=0;i<large.entities().size();++i) for(std::size_t j=i+1;j<large.entities().size();++j) {
            const auto& a=large.entities()[i]; const auto& b=large.entities()[j];
            require(clearance({a.signalTarget,a.signalStopHeading,a.length,a.width},{b.signalTarget,b.signalStopHeading,b.length,b.width})>=.1-1e-5,"parallel planning must see reserved future queue positions");
        }
        for(int step=0;step<50;++step) { large.tick(.01); safe(large); }
        moving=0; for(std::size_t i=0;i<large.entities().size();++i) moving+=length(large.entities()[i].center-before[i].center)>.05;
        require(moving==36,"whole long queue advances in the same simulation interval");
        Simulation opposed(Map(80,3,std::vector<Tile>(240,Tile::Road)),4);
        opposed.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{1,0},{4,0,2,0},1});
        opposed.setTrafficLights({{{40,.01},{40,2.99},.1,0,100,1,true}});
        require(opposed.spawnAt({30,1.5},{70,1.5},VehicleKind::Car,0) && opposed.spawnAt({50,1.5},{10,1.5},VehicleKind::Car,0),"opposing approaches spawn");
        opposed.planSignalQueues(true);
        require(opposed.lastSignalPlanCount()==2,"both opposing approaches assigned together");
        const auto& a=opposed.entities()[0]; const auto& b=opposed.entities()[1];
        require(clearance({a.signalTarget,a.signalStopHeading,a.length,a.width},{b.signalTarget,b.signalStopHeading,b.length,b.width})>=1-1e-5,"opposing final targets respect pairwise spacing across the line");
        std::cout<<"Frozen-clock batch planning and concurrent 4/36-vehicle queue movement passed\n";
    } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
