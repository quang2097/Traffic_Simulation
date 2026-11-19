#include "traffic/Simulation.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace traffic;
int main() { try {
    // Reduced seed-1024 case: a curved lane meets a circular free junction.
    // Both ends of a 15 cm departure fit, but its first millimetre does not.
    DirectedLane road;
    road.points={{133.08884953214135,7.35343773734569},{106.60134697589388,28.056265245715892},
        {103.94325547863386,29.587759451979316},{100.33567029095128,31.099630030167344},
        {98.51475202257676,32.14015358779101},{94.53819383082393,34.81404958067858},
        {92.805064187069,36.54717728037333},{91.91250278380093,38.07727596389571}};
    road.pointWidths={3.140000104904175,3.319999933242798,3.049999952316284,3.5,3.5,3.5,3.5,3.5};
    road.width=3.5;road.roadGroup=0;road.inferConnections=false;road.successors={1};
    DirectedLane junction;junction.points={road.points.back(),{97.66673923343693,62.93263249234227}};
    junction.width=3.5;junction.entry=false;junction.inferConnections=false;junction.openJunction=18;
    junction.junctionCenter={83.07182006835937,54.24737927246093};junction.junctionRadius=20.5;junction.successors={2};
    DirectedLane exit;exit.points={junction.points.back(),{123.96076869579734,85.78623475339163}};
    exit.width=3.32;exit.roadGroup=1;exit.entry=false;exit.destination=true;exit.inferConnections=false;
    Simulation s(Map(160,100,std::vector<Tile>(16000,Tile::Road)),1024);
    s.setLanes({road,junction,exit});s.setTrafficFlowSettings({true,true,.4,.15});s.setStuckVehicleSettings({false,30});
    s.setTrafficRecoverySettings({true,.6});
    Entity e;e.id=1;e.kind=VehicleKind::Motorbike;e.center={98.33938655808913,33.62601989655584};
    e.heading=2.2885745251049898;e.length=2.064368224092153;e.width=.7212515621350595;
    e.preferredDistance=.35447454502063086;e.desiredSpeed=7.658291389069706;e.acceleration=1.888208707768725;
    e.networkRoute={0,1,2};e.laneRoute={0,1};e.goal={92.9954641829551,38.709006163080645};e.destinationPoint=exit.points.back();
    e.lateralOffset=-1.285422123182334;e.currentStopSeconds=3;e.cooperativeRecovery=true;e.reservationEnd=1;
    e.sublaneAfter=e.laneChangeAfter=1e9;
    Vec2 forward{std::cos(e.heading),std::sin(e.heading)};
    e.path={e.center,e.center+forward*.15,e.goal};e.pathHeadings={e.heading,e.heading,2.1};
    auto start=e.center;const_cast<std::vector<Entity>&>(s.entities()).push_back(e);
    for(int i=0;i<3600 && !s.entities().empty();++i) {
        s.tick(1./120);
        for(const auto& actor:s.entities()) if(!s.map().fits(actor.footprint())) throw std::runtime_error("lane-edge recovery left the mask");
    }
    if(!s.entities().empty() && length(s.entities()[0].center-start)<1) throw std::runtime_error("recovery kept retrying an unexecutable lane-edge departure");
    if(s.removedStuckVehicles()) throw std::runtime_error("recovery deleted its vehicle");
    std::cout<<"Lane-edge departure escaped without deletion\n";
} catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;} }
