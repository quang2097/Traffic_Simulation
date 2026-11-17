#include "traffic/NodeGraph.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace traffic;
void need(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
int main() {
    try {
        NodeGraph g; g.addNode({10,50}); g.addNode({65,50});
        g.nodes[1].openRadius=17;
        g.connect(0,1,6,7); g.connections[0].roadType=2; g.connections[0].laneCount=1;
        auto lanes=g.lanes();
        Simulation s(Map(100,100,std::vector<Tile>(10000,Tile::Road)),42);
        s.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{6,0,3,0},1});
        s.setStuckVehicleSettings({false,30}); s.setLanes(lanes);
        auto u=s.urbanTrafficSettings(); u.enabled=true; s.setUrbanTrafficSettings(u);
        need(s.spawnOnLane(0,VehicleKind::Car),"U-turn trip must spawn with a feasible crossing");
        auto original=s.entities()[0];
        need(original.networkRoute.size()==3,"two-way road should offer its return lane as a trip");
        need(original.junctionPlans.size()==3 && !original.junctionPlans[1].points.empty(),"crossing must be planned at spawn");
        const auto& outgoing=lanes[original.networkRoute[2]];
        auto y=outgoing.points.back().y;
        TrafficLight light{{40,40},{40,60},.1,1,600,2,true};
        light.permissive=true; light.allowedA={30,y-2}; light.allowedB={30,y+2};
        s.setTrafficLights({light});
        need(light.phase(s.seconds())==LightPhase::Red,"fixture must be red");
        need(s.routePermitsRed(s.entities()[0],light),"future return lane must grant U-turn red permission");
        auto wrong=light; wrong.allowedA={30,75}; wrong.allowedB={30,80};
        need(!s.routePermitsRed(s.entities()[0],wrong),"unselected exit must not grant permission");
        bool crossed=false,entered=false; double stoppedInside=0;
        auto plansAtSpawn=s.globalPlans();
        for(int step=0;step<6000 && !s.entities().empty();++step) {
            s.tick(.01);
            if(s.entities().empty()) break;
            const auto& e=s.entities()[0];
            if(e.inOpenJunction && !entered) need(s.globalPlans()==plansAtSpawn,"handoff must reuse the spawn-time crossing without a new search");
            need(s.map().fits(e.footprint()),"U-turn left road mask");
            crossed|=e.center.x>42; entered|=e.inOpenJunction;
            if(e.inOpenJunction && e.speed<Simulation::stoppedSpeedThreshold) stoppedInside+=.01;
        }
        need(crossed && entered && s.completed()==1,"permitted U-turn must finish while the light stays red");
        need(stoppedInside<.1,"unblocked preplanned crossing must not pause inside junction");

        Simulation held(Map(100,100,std::vector<Tile>(10000,Tile::Road)),42);
        held.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{6,0,3,0},1});
        held.setLanes(lanes); held.setTrafficLights({wrong});
        need(held.spawnOnLane(0,VehicleKind::Car),"nonpermitted trip spawn");
        for(int i=0;i<1200;++i) held.tick(.01);
        need(!held.entities().empty() && held.entities()[0].center.x+2<40,"unselected exit must still stop at red");
        held.setTrafficLights({light});
        need(held.routePermitsRed(held.entities()[0],light),"queued native trip must retain its future allowed exit");
        for(int i=0;i<6000 && !held.entities().empty();++i) held.tick(.01);
        need(held.completed()==1,"live allowed-exit edit must release a queued native U-turn during red");

        Simulation local(Map(80,90,std::vector<Tile>(7200,Tile::Road)),17);
        local.setProfile(VehicleKind::Car,{{4,0},{2,0},{1,0},{4,0,4,0},1});
        local.setStuckVehicleSettings({false,30}); local.setTrafficRecoverySettings({true,.6,15});
        for(double y0:{10.,16.,70.}) need(local.spawnAt({10,y0},{70,y0},VehicleKind::Car,0),"local recovery fixture spawn");
        auto& actors=const_cast<std::vector<Entity>&>(local.entities());
        for(auto& e:actors) { e.inOpenJunction=true; e.planningExitGate=true; e.junctionCenter={40,40}; e.junctionRadius=100; }
        actors[0].path[0]={4,10}; actors[0].currentStopSeconds=2;
        actors[1].speed=1;
        auto before=actors; local.planStoppedTraffic(true);
        need(local.trafficRecoveryRepairs()==1,"local coordinated preview must repair stale crossing");
        need(actors[0].routeRevision>before[0].routeRevision && actors[1].routeRevision>before[1].routeRevision,"nearby moving crossing must join coordinated replan");
        need(actors[2].routeRevision==before[2].routeRevision,"distant route must stay committed");
        need(local.seconds()==0 && local.ticks()==0,"coordinated preview must freeze simulation time");
        for(std::size_t i=0;i<actors.size();++i) need(length(actors[i].center-before[i].center)==0 && actors[i].ageSeconds==before[i].ageSeconds,"preview must preserve real poses and telemetry");
        NodeGraph chain;
        for(double x:{10.,50.,100.,140.}) chain.addNode({x,50});
        chain.nodes[1].openRadius=12; chain.nodes[2].openRadius=12;
        for(int i=0;i<3;++i) { chain.connect(i,i+1,5,7); chain.connections.back().roadType=1; }
        Simulation multi(Map(160,100,std::vector<Tile>(16000,Tile::Road)),42);
        multi.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{6,0,3,0},1}); multi.setLanes(chain.lanes());
        need(multi.spawnOnLane(0,VehicleKind::Car),"multi-junction trip spawn");
        const auto& trip=multi.entities()[0];
        need(trip.junctionPlans.size()==5 && !trip.junctionPlans[1].points.empty() && !trip.junctionPlans[3].points.empty(),"every crossing on a multi-junction trip must be prepared at spawn");
        for(int i=0;i<6000 && !multi.entities().empty();++i) multi.tick(.01);
        need(multi.completed()==1,"complete preplanned multi-junction trip must arrive");
        std::cout<<"Spawn-time crossing plan, red U-turn permission, exit specificity and continuous handoff passed\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
