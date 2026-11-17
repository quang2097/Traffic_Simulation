#include "traffic/Simulation.hpp"
#include "traffic/Geometry.hpp"
#include <iostream>
#include <stdexcept>
using namespace traffic;
void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
void setPath(const Map& map,Entity& entity,std::vector<Vec2> guide) {
    auto path=curvePath(map,entity.footprint(),guide);
    require(!path.points.empty() && !path.turns.empty(),"fixture has a valid curved maneuver");
    entity.path=std::move(path.points); entity.pathHeadings=std::move(path.headings); entity.maneuvers=std::move(path.turns);
    entity.next=1; entity.segmentProgress=0; entity.localSteerAfter=1000; entity.replanAfter=1000;
}
int main() {
    try {
        Simulation s(Map(50,40,std::vector<Tile>(2000,Tile::Road)),7);
        VehicleProfile car{{4.5,0},{1.8,0},{1,0},{4,0,4,0},1};
        s.setProfile(VehicleKind::Car,car); s.setStuckVehicleSettings({false,30});
        // Isolate ordinary reservation admission; coordinated reservation release
        // is covered separately by TrafficRecoveryTests.
        s.setTrafficRecoverySettings({false,1});
        auto routing=s.routingSettings(); routing.rtsStyle=false; routing.spreadAtSignals=false; s.setRoutingSettings(routing);
        require(s.spawnAt({10,10},{40,10},VehicleKind::Car,0),"approaching car spawn");
        require(s.spawnAt({30,20},{10,20},VehicleKind::Car,0),"reservation owner spawn");
        // Controlled motion/reservation fixture: Entity is publicly described,
        // but production entities() stays read-only. The underlying s is mutable.
        auto& entities=const_cast<std::vector<Entity>&>(s.entities());
        setPath(s.map(),entities[0],{{10,10},{20,10},{20,30}});
        setPath(s.map(),entities[1],{{30,20},{20,20},{20,7}});
        auto& owner=entities[1]; const auto& turn=owner.maneuvers.front();
        owner.reservedTurn=turn.sweep; owner.turnPadding=turn.padding; owner.reservationEnd=turn.end;
        owner.turnCenter=owner.path[turn.end]; owner.turnHeading=owner.pathHeadings[turn.end];
        auto entry=corners({owner.path[turn.begin],owner.pathHeadings[turn.begin],owner.length,owner.width});
        owner.reservedApproach=corners(owner.footprint());
        owner.reservedApproach.insert(owner.reservedApproach.end(),entry.begin(),entry.end());
        owner.reservedApproach=geometry::convexHull(std::move(owner.reservedApproach));
        owner.desiredSpeed=.05;
        const auto begin=entities[0].maneuvers.front().begin;
        const Vec2 arcEntry=entities[0].path[begin];
        double gap=clearance(entities[0].footprint(),owner.footprint());
        require(gap>5,"physical road ahead is clear");
        for(int i=0;i<100;++i) s.tick(.01);
        require(s.entities()[0].center.x>11,"denied future maneuver must not stop car far before entry");
        require(!s.entities()[0].waitingForManeuver,"advancing car is not marked stationary for maneuver");
        for(int i=0;i<500;++i) {
            s.tick(.01);
            require(s.entities()[0].next<=begin+1,"unreserved car entered committed arc");
            require(clearance(s.entities()[0].footprint(),s.entities()[1].footprint())>=1-1e-6,"pairwise spacing violated");
        }
        require(length(s.entities()[0].center-arcEntry)<.05,"car advances to the safe arc entry");
        require(s.entities()[0].waitingForManeuver,"car waits only when it reaches the contested entry");
        auto id=s.entities()[1].id; s.setEntityMotion(id,4,4);
        for(int i=0;i<100;++i) s.tick(.01);
        require(geometry::distance({{32.25,20}},s.entities()[1].reservedApproach)>.5,"approach reservation releases vacated road");
        std::cout<<"Maneuver approach progress and reservation trimming passed\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
