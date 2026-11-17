#include "traffic/Simulation.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace traffic;
void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
Simulation open() {
    Simulation s(Map(100,60,std::vector<Tile>(6000,Tile::Road)),7);
    s.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{1,0},{4,0,4,0},1});
    auto settings=s.routingSettings(); settings.spreadAtSignals=false; settings.movingRefreshInterval=.5;
    s.setRoutingSettings(settings); s.setStuckVehicleSettings({false,30}); return s;
}
void reservedBypass(Simulation& s,const std::vector<Vec2>& guide) {
    auto& e=const_cast<std::vector<Entity>&>(s.entities())[0];
    auto path=curvePath(s.map(),e.footprint(),guide);
    require(!path.turns.empty(),"bypass fixture has a turn");
    e.path=std::move(path.points); e.pathHeadings=std::move(path.headings); e.maneuvers=std::move(path.turns);
    e.next=1; e.segmentProgress=0; e.speed=4; e.usingLocalDetour=true;
    e.detourMerge={45,10}; e.detourExitDirection={1,0};
    const auto& arc=e.maneuvers.front();
    e.reservedTurn=arc.sweep; e.reservedApproach=corners(e.footprint());
    e.turnPadding=arc.padding; e.reservationEnd=arc.end;
    e.turnCenter=e.path[arc.end]; e.turnHeading=e.pathHeadings[arc.end];
}
int main() {
    try {
        auto s=open(),disabled=open();
        auto off=disabled.routingSettings(); off.refreshMovingRoutes=false; disabled.setRoutingSettings(off);
        require(s.spawnAt({10,10},{90,10},VehicleKind::Car,0) && disabled.spawnAt({10,10},{90,10},VehicleKind::Car,0),"straight refresh spawn");
        for(int step=0;step<300;++step) {
            Entity before=s.entities()[0]; s.tick(.01); disabled.tick(.01); const auto& e=s.entities()[0];
            require(length(e.center-before.center)<=.040001,"refresh cannot teleport");
            require(e.speed+1e-9>=before.speed,"unobstructed refresh preserves momentum");
            require(std::abs(angleDelta(before.heading,e.heading))<1e-9 && e.goal.x==90 && e.goal.y==10,"heading and final endpoint preserved");
            require(length(e.center-disabled.entities()[0].center)<1e-7,"straight trajectory unchanged by reissue");
        }
        require(s.destinationRefreshes()>=4 && disabled.destinationRefreshAttempts()==0,"configurable moving interval and opt-out");

        auto detour=open(); require(detour.spawnAt({10,10},{90,10},VehicleKind::Car,0),"detour spawn");
        auto& e=const_cast<std::vector<Entity>&>(detour.entities())[0];
        auto path=curvePath(detour.map(),e.footprint(),{{10,10},{30,10},{30,30},{90,30},{90,10}});
        require(!path.turns.empty(),"controlled detour fixture");
        e.path=std::move(path.points); e.pathHeadings=std::move(path.headings); e.maneuvers=std::move(path.turns);
        e.next=1; e.usingLocalDetour=true; e.detourMerge={90,10}; e.detourExitDirection={1,0};
        e.speed=4; e.movingSinceRefresh=.5;
        detour.tick(.01);
        require(detour.destinationRefreshes()==1 && !detour.entities()[0].usingLocalDetour && detour.entities()[0].path.size()==2,
            "reissuing final endpoint discards obsolete detour");
        require(std::abs(detour.entities()[0].center.x-10.04)<1e-8 && detour.entities()[0].speed==4,"reissue keeps current momentum");

        auto small=open(); require(small.spawnAt({10,10},{90,10},VehicleKind::Car,0),"small bypass spawn");
        reservedBypass(small,{{10,10},{25,10},{30,12},{45,10},{90,10}});
        auto quick=small.routingSettings(); quick.movingRefreshInterval=6; quick.localInterval=.25; small.setRoutingSettings(quick);
        const_cast<std::vector<Entity>&>(small.entities())[0].movingSinceRefresh=.25;
        small.tick(.01);
        require(!small.entities()[0].usingLocalDetour && small.destinationRefreshes()==1 && small.entities()[0].reservedTurn.empty(),
            "small obsolete bypass cancels despite its reservation and ordinary five-percent threshold");

        auto obstacle=open();
        require(obstacle.spawnAt({10,10},{90,10},VehicleKind::Car,0) && obstacle.spawnAt({27,10},{90,10},VehicleKind::Car,0),"blocker scenario spawns");
        reservedBypass(obstacle,{{10,10},{20,10},{20,16},{35,16},{35,10},{90,10}});
        auto& units=const_cast<std::vector<Entity>&>(obstacle.entities()); units[0].movingSinceRefresh=.5;
        obstacle.tick(.01);
        require(obstacle.entities()[0].usingLocalDetour && obstacle.destinationRefreshes()==0,"bypass retained while obstacle still blocks return corridor");
        // The leading car has now driven forward out of the return corridor.
        units[1].center={40,10}; units[1].path={{40,10},{90,10}}; units[1].pathHeadings={0,0}; units[1].segmentProgress=0; units[1].next=1;
        units[0].movingSinceRefresh=.5; Entity old=units[0];
        obstacle.tick(.01);
        require(!obstacle.entities()[0].usingLocalDetour && obstacle.destinationRefreshes()==1,"bypass cancelled when moving leader clears the lane");
        require(length(obstacle.entities()[0].center-old.center)<=.040001 && obstacle.entities()[0].speed==old.speed,"cleared-obstacle cancellation preserves momentum");

        auto turning=open(); require(turning.spawnAt({10,10},{90,10},VehicleKind::Car,0),"turning bypass spawn");
        auto noRefresh=turning.routingSettings(); noRefresh.refreshMovingRoutes=false; turning.setRoutingSettings(noRefresh);
        reservedBypass(turning,{{10,10},{20,10},{20,30},{90,30},{90,10}});
        bool inArc=false;
        for(int i=0;i<500;++i) {
            turning.tick(.01); const auto& vehicle=turning.entities()[0];
            if(vehicle.heading>.25 && vehicle.heading<.8) { inArc=true; break; }
        }
        require(inArc,"reached middle of avoidance arc");
        noRefresh.refreshMovingRoutes=true; turning.setRoutingSettings(noRefresh);
        auto& turningUnit=const_cast<std::vector<Entity>&>(turning.entities())[0]; turningUnit.movingSinceRefresh=.5;
        old=turningUnit; turning.tick(.01);
        const auto& corrected=turning.entities()[0];
        require(!corrected.usingLocalDetour && turning.destinationRefreshes()==1,"obsolete bypass can be replaced mid-arc");
        require(length(corrected.center-old.center)<=.040001 && std::abs(angleDelta(old.heading,corrected.heading))<.1 && corrected.speed==old.speed,
            "mid-arc return keeps continuous position heading and speed");
        require(!corrected.reservedTurn.empty() && length(corrected.turnCenter-corrected.center)<9,"return reserves a short blending curve rather than entire distant endpoint path");

        auto committed=open(); require(committed.spawnAt({10,10},{90,10},VehicleKind::Car,0),"committed spawn");
        auto& turner=const_cast<std::vector<Entity>&>(committed.entities())[0];
        auto bend=curvePath(committed.map(),turner.footprint(),{{10,10},{30,10},{30,40}});
        turner.path=std::move(bend.points); turner.pathHeadings=std::move(bend.headings); turner.maneuvers=std::move(bend.turns);
        turner.reservedTurn=turner.maneuvers[0].sweep; turner.reservationEnd=turner.maneuvers[0].end;
        turner.turnPadding=turner.maneuvers[0].padding; turner.speed=4; turner.movingSinceRefresh=1;
        committed.tick(.01); require(committed.destinationRefreshAttempts()==0,"committed turn defers reissue");
        turner.reservedTurn.clear(); turner.reservedApproach.clear();
        committed.tick(.01); require(committed.destinationRefreshes()==1,"overdue reissue resumes after reservation clears");

        auto red=open(); red.setTrafficLights({{{25,.01},{25,59.99},.1,0,100,1,true}});
        require(red.spawnAt({10,10},{90,10},VehicleKind::Car,0),"red wait spawn");
        for(int i=0;i<800;++i) red.tick(.01);
        require(red.destinationRefreshAttempts()==0,"red approach is protected before a queue slot is assigned");
        auto attempts=red.destinationRefreshAttempts(); double timer=red.entities()[0].movingSinceRefresh;
        require(red.entities()[0].speed==0,"red vehicle stationary");
        for(int i=0;i<200;++i) red.tick(.01);
        require(red.destinationRefreshAttempts()==attempts && red.entities()[0].movingSinceRefresh==timer,"stationary time neither triggers nor advances reissues");

        auto fair=open();
        for(int i=0;i<3;++i) require(fair.spawnAt({10,10.+10*i},{90,10.+10*i},VehicleKind::Car,0),"fair scheduler spawn");
        auto& fleet=const_cast<std::vector<Entity>&>(fair.entities());
        for(int i=0;i<3;++i) { fleet[i].speed=4; fleet[i].movingSinceRefresh=3-i; }
        for(int i=0;i<3;++i) fair.tick(.01);
        for(const auto& vehicle:fair.entities()) require(vehicle.destinationRefreshes==1,"overdue units share refresh budget fairly");
        // Reissuing an unchanged, wall-constrained route must not change future
        // randomly selected vehicle classes. The disconnected spawn pocket
        // keeps probe spawns independent of the moving test car's occupancy.
        Map constrained(100,60,std::vector<Tile>(6000,Tile::Wall));
        for(int y=5;y<36;++y) for(int x=5;x<31;++x)
            if(y<19 || x>=20) constrained.tiles[constrained.index({x,y})]=Tile::Road;
        for(int y=35;y<55;++y) for(int x=65;x<96;++x) constrained.tiles[constrained.index({x,y})]=Tile::Road;
        constrained.tiles[constrained.index({70,45})]=Tile::Spawn;
        constrained.tiles[constrained.index({90,45})]=Tile::Destination;
        Simulation stable(constrained,17),reference(constrained,17);
        for(int i=0;i<vehicleKindCount;++i) {
            VehicleProfile p{{1,0},{.4,0},{.2,0},{4,0,4,0},1};
            stable.setProfile(static_cast<VehicleKind>(i),p); reference.setProfile(static_cast<VehicleKind>(i),p);
        }
        auto refresh=stable.routingSettings(); refresh.movingRefreshInterval=.25;
        stable.setRoutingSettings(refresh); refresh.refreshMovingRoutes=false; reference.setRoutingSettings(refresh);
        require(stable.spawnAt({10,12},{25,31},VehicleKind::Car,0) && reference.spawnAt({10,12},{25,31},VehicleKind::Car,0),"constrained route spawn");
        for(Simulation* simulation:{&stable,&reference}) {
            auto& unit=const_cast<std::vector<Entity>&>(simulation->entities())[0];
            unit.speed=4; unit.movingSinceRefresh=.25;
            simulation->tick(.01);
        }
        require(stable.destinationRefreshAttempts()==1,"constrained search reissued");
        for(int i=0;i<16;++i) {
            require(stable.spawnRandom(0) && reference.spawnRandom(0),"random stream probe spawn");
            require(stable.entities().back().kind==reference.entities().back().kind,"refresh must preserve future spawn class sequence");
            const_cast<std::vector<Entity>&>(stable.entities()).pop_back();
            const_cast<std::vector<Entity>&>(reference.entities()).pop_back();
        }
        bool rejected=false; auto invalid=fair.routingSettings(); invalid.movingRefreshInterval=std::numeric_limits<double>::quiet_NaN();
        try { fair.setRoutingSettings(invalid); } catch(const std::invalid_argument&) { rejected=true; }
        require(rejected,"invalid refresh interval rejected");
        std::cout<<"Moving destination reissue, momentum, deferred turns and fair scheduling passed\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
