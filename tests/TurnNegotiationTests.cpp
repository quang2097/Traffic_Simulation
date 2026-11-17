#include "traffic/Simulation.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace traffic;
void require(bool value,const char* text) { if(!value) throw std::runtime_error(text); }
Simulation world(int width=60,int height=50) {
    Simulation s(Map(width,height,std::vector<Tile>(width*height,Tile::Road)),42);
    s.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{4,0,3,0},1});
    auto r=s.routingSettings(); r.rtsStyle=false; r.spreadAtSignals=false; s.setRoutingSettings(r);
    s.setStuckVehicleSettings({false,30}); s.setTrafficRecoverySettings({false,1});
    auto n=s.turnNegotiationSettings(); n.enabled=true; s.setTurnNegotiationSettings(n);
    return s;
}
void path(Simulation& s,std::size_t i,std::vector<Vec2> guide,double heading) {
    auto& e=const_cast<std::vector<Entity>&>(s.entities())[i]; e.heading=heading;
    auto curve=curvePath(s.map(),e.footprint(),guide); require(!curve.turns.empty(),"fixture has a curved turn");
    e.path=curve.points; e.pathHeadings=curve.headings; e.maneuvers=curve.turns; e.next=1; e.segmentProgress=0;
    e.localSteerAfter=e.replanAfter=10000; ++e.routeRevision;
}
void safe(const Simulation& s) {
    for(const auto& e:s.entities()) {
        require(s.map().fits(e.footprint()),"negotiated maneuver left road");
        for(const auto& other:s.entities()) if(e.id<other.id)
            require(clearance(e.footprint(),other.footprint())>=.5-1e-5,"negotiated maneuver violated spacing");
    }
}
int main() {
 try {
    auto s=world();
    require(s.spawnAt({15,10},{30,35},VehicleKind::Car,0) && s.spawnAt({25,28},{25,4},VehicleKind::Car,0),"crossing fixture spawn");
    path(s,0,{{15,10},{30,10},{30,35}},0);
    auto ownerId=s.entities()[0].id; bool yielded=false,crept=false,resumed=false; double grantedAt=-1;
    for(int i=0;i<6000 && !s.entities().empty();++i) {
        s.tick(.01); safe(s);
        for(const auto& e:s.entities()) {
            if(e.id==ownerId && s.ownsNegotiatedTurn(e) && e.speed>.1) {
                if(grantedAt<0) grantedAt=s.seconds();
                if(s.seconds()-grantedAt>.5) { crept=true; require(e.speed<=2.001,"owner failed to creep after braking"); }
            }
            if(e.id!=ownerId && s.maneuverYieldTo(e)==ownerId && !e.clearingTurn && e.speed<1) yielded=true;
            if(e.id!=ownerId && yielded && !s.maneuverYieldTo(e) && e.speed>1) resumed=true;
        }
    }
    require(yielded && crept && resumed && s.completed()==2,"crossing traffic yields until turning vehicle clears then resumes");
    auto rear=world();
    require(rear.spawnAt({24.5,10},{30,35},VehicleKind::Car,0) && rear.spawnAt({17.5,10},{50,10},VehicleKind::Car,0),"rear swing fixture spawns");
    path(rear,0,{{24.5,10},{30,10},{30,35}},0);
    const_cast<std::vector<Entity>&>(rear.entities())[1].speed=4;
    rear.tick(.01); safe(rear);
    require(rear.ownsNegotiatedTurn(rear.entities()[0]) && rear.maneuverYieldTo(rear.entities()[1])==rear.entities()[0].id,"straight follower must yield to turning rear swing even on the same incoming road");
    for(int i=0;i<6000 && !rear.entities().empty();++i) { rear.tick(.01); safe(rear); }
    require(rear.completed()==2,"rear swing and straight follower both clear");

    auto inside=world();
    require(inside.spawnAt({24.5,10},{30,35},VehicleKind::Car,0) && inside.spawnAt({28,15},{28,4},VehicleKind::Car,0),"occupant fixture spawn");
    path(inside,0,{{24.5,10},{30,10},{30,35}},0);
    const_cast<std::vector<Entity>&>(inside.entities())[1].speed=4;
    auto red=inside; red.setTrafficLights({{{29,.01},{29,49.99},.1,0,100,1,true}});
    for(int i=0;i<400;++i) {
        red.tick(.01); safe(red);
        for(const auto& e:red.entities()) for(auto p:corners(e.footprint())) require(p.x<29+1e-6,"turn priority crossed red signal");
    }
    require(red.negotiatedTurns()==0,"red signal takes precedence over a turn request");
    inside.tick(.01); safe(inside);
    require(inside.ownsNegotiatedTurn(inside.entities()[0]) && inside.entities()[1].clearingTurn,"existing occupant is identified as clearing");
    double before=inside.entities()[1].center.y;
    for(int i=0;i<20;++i) { inside.tick(.01); safe(inside); }
    require(inside.entities()[1].center.y<before-.5,"inside occupant must keep moving out, not freeze in the turn");
    for(int i=0;i<6000 && !inside.entities().empty();++i) { inside.tick(.01); safe(inside); }
    require(inside.completed()==2,"occupant and turn owner both drain");

    auto physics=world();
    require(physics.spawnAt({24.5,10},{30,35},VehicleKind::Car,0) && physics.spawnAt({30,28},{30,4},VehicleKind::Car,0),"force yield fixture spawns");
    path(physics,0,{{24.5,10},{30,10},{30,35}},0);
    const_cast<std::vector<Entity>&>(physics.entities())[1].speed=4;
    physics.tick(.01); require(physics.ownsNegotiatedTurn(physics.entities()[0]),"force fixture has a turn owner");
    auto tug=physics.tugSettings(); tug.enabled=tug.alwaysOn=true; physics.setTugSettings(tug);
    bool forceHeld=false;
    for(int i=0;i<10000 && !physics.entities().empty();++i) {
        physics.tick(.01); safe(physics);
        for(const auto& e:physics.entities()) if(physics.maneuverYieldTo(e) && !e.clearingTurn && e.speed<.1) forceHeld=true;
    }
    require(forceHeld && physics.completed()==2,"gravitational force cannot override a negotiated yield");

    auto competing=world();
    require(competing.spawnAt({15,10},{30,35},VehicleKind::Car,0) && competing.spawnAt({45,10},{30,35},VehicleKind::Car,0),"competing turn spawns");
    path(competing,0,{{15,10},{30,10},{30,35}},0);
    path(competing,1,{{45,10},{30,10},{30,35}},3.141592653589793);
    for(int i=0;i<8000 && !competing.entities().empty();++i) {
        competing.tick(.01); safe(competing); int owners=0;
        for(const auto& e:competing.entities()) owners+=competing.ownsNegotiatedTurn(e);
        require(owners<=1,"overlapping turn corridors received simultaneous priority");
    }
    require(competing.completed()==2 && competing.negotiatedTurns()>=2,"competing turns take turns without starvation");
    auto parallel=world(120,80);
    require(parallel.spawnAt({24.5,10},{30,35},VehicleKind::Car,0) && parallel.spawnAt({30,28},{30,4},VehicleKind::Car,0) &&
        parallel.spawnAt({84.5,50},{90,75},VehicleKind::Car,0) && parallel.spawnAt({90,68},{90,44},VehicleKind::Car,0),"independent intersection spawns");
    path(parallel,0,{{24.5,10},{30,10},{30,35}},0); path(parallel,2,{{84.5,50},{90,50},{90,75}},0);
    auto& streams=const_cast<std::vector<Entity>&>(parallel.entities()); streams[1].speed=streams[3].speed=4;
    parallel.tick(.01); safe(parallel);
    require(parallel.ownsNegotiatedTurn(streams[0]) && parallel.ownsNegotiatedTurn(streams[2]),"disjoint turn spaces proceed concurrently");
    for(int i=0;i<6000 && !parallel.entities().empty();++i) { parallel.tick(.01); safe(parallel); }
    require(parallel.completed()==4,"independent flows drain together");

    auto blocked=world();
    require(blocked.spawnAt({24.5,10},{30,35},VehicleKind::Car,0) && blocked.spawnAt({30,18},{30,40},VehicleKind::Car,0),"blocked exit spawns");
    path(blocked,0,{{24.5,10},{30,10},{30,35}},0);
    auto& b=const_cast<std::vector<Entity>&>(blocked.entities()); b[1].desiredSpeed=0;
    auto entry=b[0].maneuvers.front().begin;
    for(int i=0;i<400;++i) { blocked.tick(.01); safe(blocked); require(b[0].next<=entry+1,"turn entered without downstream room"); }
    require(blocked.negotiatedTurns()==0 && b[0].turnExitBlocked,"blocked exit is a legal wait, not a claimed intersection");
    b[1].desiredSpeed=4;
    for(int i=0;i<6000 && !blocked.entities().empty();++i) { blocked.tick(.01); safe(blocked); }
    require(blocked.completed()==2,"blocked exit resumes when downstream traffic moves");

    auto stale=world();
    require(stale.spawnAt({24.5,10},{30,35},VehicleKind::Car,0) && stale.spawnAt({30,28},{30,4},VehicleKind::Car,0),"lease fixture spawns");
    path(stale,0,{{24.5,10},{30,10},{30,35}},0);
    auto& stopped=const_cast<std::vector<Entity>&>(stale.entities()); stopped[1].speed=4;
    stale.tick(.01); require(stale.ownsNegotiatedTurn(stopped[0]),"lease granted");
    require(!stale.spawnAt({30,18},{30,40},VehicleKind::Car,0),"new spawn cannot appear inside claimed turn space");
    auto changed=stale; auto& rerouted=const_cast<std::vector<Entity>&>(changed.entities()); ++rerouted[0].routeRevision;
    require(!changed.ownsNegotiatedTurn(rerouted[0]) && !changed.maneuverYieldTo(rerouted[1]),"changed route immediately invalidates its old yield ownership");
    stopped[0].speed=stopped[0].desiredSpeed=0;
    for(int i=0;i<215;++i) { stale.tick(.01); safe(stale); }
    require(!stale.ownsNegotiatedTurn(stopped[0]) && !stale.maneuverYieldTo(stopped[1]),"abandoned owner cannot leave a ghost yield");
    auto disable=stale.turnNegotiationSettings(); disable.enabled=false; stale.setTurnNegotiationSettings(disable);
    require(!stale.maneuverYieldTo(stopped[1]),"live disable releases claims");
    auto dependency=world(); dependency.setTrafficRecoverySettings({true,.6});
    require(dependency.spawnAt({10,10},{40,10},VehicleKind::Car,0) && dependency.spawnAt({10,20},{40,20},VehicleKind::Car,0),"exit dependency fixture spawns");
    auto& waits=const_cast<std::vector<Entity>&>(dependency.entities());
    for(std::size_t i=0;i<waits.size();++i) {
        waits[i].turnExitBlocked=true; waits[i].exitWaitFor=waits[1-i].id; waits[i].currentStopSeconds=3;
    }
    dependency.planStoppedTraffic(true);
    require(dependency.trafficRecoveryRepairs()==1 && dependency.seconds()==0 && !waits[0].turnExitBlocked && !waits[1].turnExitBlocked,"mutual downstream waits cannot become permanent legal-stop deadlocks");
    std::cout<<"Turn right-of-way, creeping, occupant clearing, exit room and lease release passed\n";
 } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
