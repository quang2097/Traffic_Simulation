#include "traffic/Demo.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace traffic;
void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
int main() {
    try {
    std::cout<<std::unitbuf;
    Simulation s(makeDemoMap(),42); double spawnClock=0;
    unsigned spawned=0; double mostProgress=0;
    for(int step=0;step<120*10;++step) {
        constexpr double dt=1./120; spawnClock+=dt;
        if(spawnClock+1e-10>=1) {
            spawnClock=std::max(0.,spawnClock-1);
            spawned+=s.spawnRandom(3);
        }
        s.tick(dt);
        require(s.seconds()>3 || s.localDetours()==0,"startup must not overtake a leader that is still accelerating");
        for(std::size_t j=0;j<s.entities().size();++j) {
            const auto& e=s.entities()[j]; mostProgress=std::max(mostProgress,e.center.x);
            require(s.map().fits(e.footprint()),"startup vehicle leaves mask");
            for(std::size_t k=j+1;k<s.entities().size();++k) {
                const auto& other=s.entities()[k];
                require(clearance(e.footprint(),other.footprint())+1e-5>=std::min(e.preferredDistance,other.preferredDistance),"startup clearance violated");
            }
            if(e.center.x<22 && !e.reversing) {
                Vec2 direction=(e.goal-e.center)*(1/length(e.goal-e.center)),previous=e.center; double checked=0;
                for(std::size_t i=e.next;i<e.path.size() && checked<6;++i) {
                    Vec2 segment=e.path[i]-previous; double distance=length(segment);
                    require(segment.x*direction.x+segment.y*direction.y>=-1e-6,"startup avoidance path doubles back toward the spawn");
                    double desired=std::atan2(direction.y,direction.x);
                    require(std::abs(angleDelta(desired,e.pathHeadings[i]))<1.3,"startup avoidance turns sideways instead of following a moving leader");
                    checked+=distance; previous=e.path[i];
                }
            }
        }
    }
    require(spawned>=5 && mostProgress>25,"startup traffic continues advancing");
    Simulation queue(Map(80,12,std::vector<Tile>(960,Tile::Road)),12);
    queue.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{1,0},{8,0,2,0},1});
    require(queue.spawnAt({20,6},{70,6},VehicleKind::Car,0) && queue.spawnAt({12,6},{70,6},VehicleKind::Car,0),"accelerating queue spawn");
    for(int i=0;i<600;++i) {
        queue.tick(.01);
        for(const auto& e:queue.entities()) require(!e.usingLocalDetour,"equal-speed accelerating cars should follow, not overtake");
    }
    require(queue.localDetours()==0,"no needless accelerating-leader bypasses");
    Simulation ghost(Map(70,3,std::vector<Tile>(210,Tile::Road)),13);
    ghost.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{4,0,3,0},1});
    require(ghost.spawnAt({25,1.5},{60,1.5},VehicleKind::Car,0) && ghost.spawnAt({29.5001,1.5},{5,1.5},VehicleKind::Car,0),"stale signal cycle spawn");
    for(auto& e:const_cast<std::vector<Entity>&>(ghost.entities())) { e.waitingForLight=true; e.signalQueue=true; e.speed=1; }
    ghost.tick(.01);
    for(const auto& e:ghost.entities()) require(!e.waitingForLight && !e.signalQueue,"unanchored signal waits must not circulate through blocked traffic");
    std::cout<<"Actual startup preset avoids backward/sideways loops and follows accelerating leaders\n";
    } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
