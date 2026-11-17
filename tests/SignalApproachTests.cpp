#include "traffic/Simulation.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace traffic;
void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
TrafficLight red(double x,double height=30) { return {{x,.01},{x,height-.01},.1,0,100,1,true}; }
Simulation open() {
    Simulation s(Map(100,30,std::vector<Tile>(3000,Tile::Road)),42);
    s.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{8,0,4,0},1});
    auto routing=s.routingSettings(); routing.refreshMovingRoutes=false; s.setRoutingSettings(routing); return s;
}
double frontX(const Entity& e) { double value=-1e9; for(Vec2 p:corners(e.footprint())) value=std::max(value,p.x); return value; }
int main() {
    try {
        auto selector=open(); auto nearEuclidean=red(12); nearEuclidean.a.y=11; nearEuclidean.b.y=13;
        auto first=red(40); first.b.y=11; selector.setTrafficLights({nearEuclidean,first});
        Entity winding; winding.center={10,10}; winding.length=2; winding.width=1;
        winding.path={{10,10},{60,10},{60,12},{5,12}}; winding.pathHeadings={0,0,1.57079632679,3.14159265359}; winding.next=1;
        double distance=0; Vec2 crossing;
        require(selector.closestStoppingLight(winding,&distance,&crossing)==1 && std::abs(distance-30)<1e-8,"closest signal must use route order, not Euclidean distance or list order");

        for(bool spread:{false,true}) {
            auto s=open(); auto routing=s.routingSettings(); routing.spreadAtSignals=spread; s.setRoutingSettings(routing);
            s.setTrafficLights({red(60),red(30)}); require(s.spawnAt({10,10},{90,10},VehicleKind::Car,0),"nearest stop spawn");
            for(int i=0;i<800;++i) { s.tick(.01); require(frontX(s.entities()[0])<30+1e-6,"vehicle crossed first red stop"); }
            const auto& e=s.entities()[0];
            require(e.signalLight==1 && 30-frontX(e)>.1 && 30-frontX(e)<.2,"vehicle approaches nearest red within 15 cm, with or without spreading");
            require(e.goal.x==90 && e.heading==0,"signal target preserves original endpoint and natural heading");
        }

        auto changing=open(); auto routing=changing.routingSettings(); routing.lookAhead=60; changing.setRoutingSettings(routing);
        auto near=red(30); near.greenSeconds=1; near.offsetSeconds=0;
        changing.setTrafficLights({red(60),near}); require(changing.spawnAt({10,10},{90,10},VehicleKind::Car,0),"retarget spawn");
        changing.tick(.01); require(changing.entities()[0].signalLight==0,"initial farther red target acquired");
        for(int i=0;i<200;++i) {
            Vec2 before=changing.entities()[0].center; changing.tick(.01);
            require(length(changing.entities()[0].center-before)<=.080001,"retarget cannot teleport");
        }
        require(changing.entities()[0].signalLight==1,"new nearer red replaces an existing farther target");
        for(int i=0;i<600;++i) changing.tick(.01);
        require(30-frontX(changing.entities()[0])<.2,"retargeted vehicle closes up to nearer red");

        auto clearing=open(); auto clearingLight=red(30); clearing.setTrafficLights({clearingLight});
        require(clearing.spawnAt({10,10},{90,10},VehicleKind::Car,0),"green transition spawn");
        for(int i=0;i<250;++i) clearing.tick(.01);
        require(clearing.entities()[0].signalLight==0 && clearing.entities()[0].speed>0,"vehicle is still approaching its red target");
        clearingLight.greenSeconds=100; clearingLight.redSeconds=.1; clearingLight.offsetSeconds=0; clearing.setTrafficLights({clearingLight});
        for(int i=0;i<200;++i) {
            double speed=clearing.entities()[0].speed; clearing.tick(.01);
            require(clearing.entities()[0].speed+1e-8>=speed,"green must not force braking at the obsolete holding point");
        }
        require(clearing.entities()[0].center.x>30,"green continuation crosses without an artificial stop");

        auto exempt=open(); auto permitted=red(30); permitted.permissive=true; permitted.allowedA={45,.01}; permitted.allowedB={45,29.99};
        exempt.setTrafficLights({permitted,red(60)}); require(exempt.spawnAt({10,10},{90,10},VehicleKind::Car,0),"permitted nearest spawn");
        require(exempt.closestStoppingLight(exempt.entities()[0])==1,"permitted red is excluded from stop targets");

        Map bent(40,40,std::vector<Tile>(1600,Tile::Wall));
        for(int y=5;y<36;++y) for(int x=5;x<31;++x) if(y<19 || x>=20) bent.tiles[bent.index({x,y})]=Tile::Road;
        Simulation curve(bent,1); curve.setProfile(VehicleKind::Car,{{2,0},{1,0},{.5,0},{4,0,4,0},1});
        TrafficLight across{{20,25},{30.99,25},.1,0,100,1,true}; curve.setTrafficLights({across});
        require(curve.spawnAt({10,12},{25,31},VehicleKind::Car,0),"curved approach spawn");
        for(int i=0;i<1200;++i) { curve.tick(.01); require(curve.map().fits(curve.entities()[0].footprint()),"signal shortcut cuts outside curved road"); }
        double frontY=-1e9; for(Vec2 p:corners(curve.entities()[0].footprint())) frontY=std::max(frontY,p.y);
        require(25-frontY>.1 && 25-frontY<.2,"curved route approaches stop closely without forcing a straight shortcut");

        Simulation queue(Map(80,3,std::vector<Tile>(240,Tile::Road)),7);
        queue.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{6,0,3,0},1});
        auto gate=red(30,3); queue.setTrafficLights({gate});
        require(queue.spawnAt({24,1.5},{70,1.5},VehicleKind::Car,0) && queue.spawnAt({16,1.5},{35,1.5},VehicleKind::Car,0),"compacting queue spawn");
        for(int i=0;i<800;++i) queue.tick(.01);
        auto follower=queue.entities()[1].id; double oldX=queue.entities()[1].center.x;
        gate.permissive=true; gate.allowedA={50,.01}; gate.allowedB={50,2.99}; queue.setTrafficLights({gate});
        for(int i=0;i<600;++i) queue.tick(.01);
        const Entity* waiting=nullptr; for(const auto& e:queue.entities()) if(e.id==follower) waiting=&e;
        require(waiting && waiting->center.x>oldX+1 && 30-frontX(*waiting)<.2 && frontX(*waiting)<30,"waiting vehicle fills vacated front-row space while its own light remains red");
        require(waiting->goal.x==35,"queue compaction preserves original destination");
        std::cout<<"Nearest-route red targeting, phase retargeting, curved approach and queue compaction passed\n";
    } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
