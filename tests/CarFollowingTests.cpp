#include "traffic/Simulation.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <unordered_map>
using namespace traffic;
void need(bool value,const char* why){if(!value)throw std::runtime_error(why);}
Simulation empty(){Simulation s(Map(220,24,std::vector<Tile>(5280,Tile::Road)),42);auto r=s.routingSettings();r.rtsStyle=false;r.spreadAtSignals=false;r.refreshMovingRoutes=false;s.setRoutingSettings(r);s.setTrafficRecoverySettings({false,1,25});s.setStuckVehicleSettings({false,30,false,60});s.setCollisionWorkers(1);return s;}
std::vector<Entity>& actors(Simulation& s){return const_cast<std::vector<Entity>&>(s.entities());}
void add(Simulation& s,Vec2 center,double heading=0,double speed=3){Entity e;e.id=s.entities().size()+1;e.center=center;e.heading=heading;e.length=4;e.width=1.8;e.preferredDistance=20;e.minGap=.7;e.tau=1;e.desiredSpeed=6;e.speed=speed;e.acceleration=2;e.path={center,{heading==0?210.:5.,center.y}};e.pathHeadings={heading,heading};e.goal=e.path.back();actors(s).push_back(e);}
void safe(const Simulation& s){for(const auto& e:s.entities()){need(s.map().fits(e.footprint()),"body left mask");for(const auto& o:s.entities())if(e.id<o.id)need(!geometry::violatesClearance(corners(e.footprint()),corners(o.footprint()),0),"physical vehicle overlap");}}
int main(){try{
    int equations=0;
    for(double dt:{.01,.1,1.})for(double b:{2.,4.5})for(double tau:{1.,1.5})for(double leader:{0.,3.,15.})for(double gap:{0.,.1,1.,10.,40.}) {
        double budget=gap+Simulation::brakingDistance(leader,std::max(b,9.),dt),n=std::floor(.5-tau/dt+std::sqrt(std::pow(tau/dt-.5,2)+2*std::max(0.,budget-1e-9)/(b*dt*dt)));
        double full=b*dt*(dt*n*(n-1)*.5+n*tau),reference=n*b*dt+(budget-full)/(n*dt+tau);
        double speed=Simulation::safeFollowingSpeed(gap,leader,9,b,tau,dt);
        need(std::abs(reference-speed)<1e-6,"discrete safe speed differs from SUMO stopping-distance equation");++equations;
    }
    auto parallel=empty();add(parallel,{20,10});add(parallel,{20,11.9});
    for(int i=0;i<500;++i){parallel.tick(.01);safe(parallel);need(std::abs(parallel.entities()[0].speed-parallel.entities()[1].speed)<1e-8,"parallel neighbor became a longitudinal leader");}
    need(parallel.entities()[0].center.x>40 && parallel.physicalCollisionEvents()==0,"close side-by-side traffic did not flow");
    auto compressed=empty();add(compressed,{50,10});add(compressed,{45.8,10});
    auto zero=compressed;for(auto& e:actors(zero))e.preferredDistance=0;
    auto reversed=compressed;std::reverse(actors(reversed).begin(),actors(reversed).end());
    for(int i=0;i<1000;++i){
        if(i==500){for(auto* sim:{&compressed,&zero,&reversed})for(auto& e:actors(*sim))if(e.id==1)e.desiredSpeed=e.speed=0;}
        compressed.tick(.01);zero.tick(.01);reversed.tick(.01);safe(compressed);safe(zero);safe(reversed);
        for(const auto& e:compressed.entities())for(const auto& o:zero.entities())if(e.id==o.id)need(length(e.center-o.center)<1e-8 && std::abs(e.speed-o.speed)<1e-8,"retired preferred distance still controls movement");
        for(const auto& e:compressed.entities())for(const auto& o:reversed.entities())if(e.id==o.id)need(length(e.center-o.center)<1e-8 && std::abs(e.speed-o.speed)<1e-8,"longitudinal decisions depend on entity storage order");
    }
    need(compressed.compressedGapEvents()>0 && compressed.emergencyBrakeEvents()>0 && compressed.physicalCollisionEvents()==0,"compressed gap/emergency counters missing or actual collision occurred");
    auto impossible=empty();add(impossible,{50,10},0,8);add(impossible,{55,10},std::acos(-1.),8);impossible.setGridlockEscapeSettings({true,1,0,1.5});
    for(auto& e:actors(impossible))e.escapeOverlap=true;
    for(int i=0;i<500;++i){impossible.tick(.01);safe(impossible);for(const auto& e:impossible.entities())need(!e.escapeOverlap,"removed ghost mode became active");}
    need(impossible.completed()==0 && impossible.contactLimitEvents()>0 && impossible.physicalCollisionEvents()==0,"impossible head-on blockage was solved by interpenetration");
    auto detected=empty();add(detected,{50,10},0,0);add(detected,{51,10},0,0);for(auto& e:actors(detected))e.desiredSpeed=0;
    detected.tick(.1);need(detected.physicalCollisionEvents()==1,"real imported physical overlap must be counted");detected.tick(.1);need(detected.physicalCollisionEvents()==1,"persistent overlap counted every frame");
    std::cout<<equations<<" discrete stopping-equation checks; compressed gaps, parallel flow, synchronous decisions, emergency braking and no ghost escape passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
