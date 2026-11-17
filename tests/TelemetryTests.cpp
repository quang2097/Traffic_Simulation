#include "traffic/Simulation.hpp"
#include "traffic/Parallel.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace traffic;
void require(bool condition,const char* text) { if(!condition) throw std::runtime_error(text); }
void near(double a,double b,const char* text) { require(std::abs(a-b)<1e-7,text); }
Map open(int w,int h) { return Map(w,h,std::vector<Tile>(w*h,Tile::Road)); }
int main() {
    try {
        Simulation s(open(70,14),42);
        auto settings=s.routingSettings(); settings.spreadAtSignals=false; s.setRoutingSettings(settings);
        auto car=s.profile(VehicleKind::Car);
        car.length={4,0}; car.width={1.8,0}; car.distance={.5,0}; car.motion={6,0,3,0};
        s.setProfile(VehicleKind::Car,car);
        auto empty=s.stopStatistics(); require(empty.vehicles==0 && empty.minSeconds==0 && empty.meanSeconds==0,"empty statistics");
        TrafficLight light{{20,.01},{20,13.99},.1,0,100,1,true}; s.setTrafficLights({light});
        require(s.spawnAt({12,4},{60,4},VehicleKind::Car,0) && s.spawnAt({8,10},{60,10},VehicleKind::Car,0),"telemetry spawns");
        for(int i=0;i<800;++i) s.tick(.01);
        auto first=s.entities()[0],second=s.entities()[1];
        require(first.currentStopSeconds>1 && second.currentStopSeconds>1,"red waits recorded");
        for(int i=0;i<200;++i) s.tick(.01);
        near(s.entities()[0].stoppedSeconds-first.stoppedSeconds,2,"stationary duration uses simulation seconds");
        near(s.entities()[0].currentStopSeconds-first.currentStopSeconds,2,"current stop continues");
        auto stats=s.stopStatistics(); first=s.entities()[0]; second=s.entities()[1];
        near(stats.meanSeconds,(first.stoppedSeconds+second.stoppedSeconds)/2,"fleet mean");
        near(stats.minSeconds,std::min(first.stoppedSeconds,second.stoppedSeconds),"fleet minimum");
        near(stats.maxSeconds,std::max(first.stoppedSeconds,second.stoppedSeconds),"fleet maximum");
        require(stats.vehicles==2 && stats.vehiclesThatStopped==2,"stopped sample population");
        light.greenSeconds=100; light.redSeconds=.1; s.setTrafficLights({light});
        for(int i=0;i<100;++i) s.tick(.01);
        require(s.entities()[0].currentStopSeconds==0 && s.entities()[0].longestStopSeconds>2,"motion ends current stop but preserves longest");
        double stopped=s.stopStatistics().totalSeconds;
        for(int i=0;i<2000 && !s.entities().empty();++i) s.tick(.01);
        require(s.completed()==2,"telemetry trips finish");
        // A partial final arrival step can also average below the threshold.
        require(s.stopStatistics().totalSeconds>=stopped-1e-7 && s.stopStatistics().totalSeconds<=stopped+.0200001,"completed stop totals retained");
        require(s.stopStatistics().vehicles==2 && s.stopStatistics(false).vehicles==0,"completed and active populations");
        Simulation serial(open(100,125),7),parallel(open(100,125),7);
        // Long parallel bodies exercise a dense broad phase without overlap.
        auto longBody=serial.profile(VehicleKind::Bike); longBody.length={20,0}; longBody.width={.6,0}; longBody.distance={.4,0};
        serial.setProfile(VehicleKind::Bike,longBody); parallel.setProfile(VehicleKind::Bike,longBody);
        serial.setCollisionWorkers(1); parallel.setCollisionWorkers(Simulation::maximumCollisionWorkers());
        for(int i=0;i<40;++i) {
            require(serial.spawnAt({15,2.+3*i},{80,2.+3*i},VehicleKind::Bike,0),"serial fleet spawn");
            require(parallel.spawnAt({15,2.+3*i},{80,2.+3*i},VehicleKind::Bike,0),"parallel fleet spawn");
        }
        for(int step=0;step<100;++step) { serial.tick(.01); parallel.tick(.01); }
        for(std::size_t i=0;i<serial.entities().size();++i) {
            near(length(serial.entities()[i].center-parallel.entities()[i].center),0,"parallel collision determinism");
            near(serial.entities()[i].stoppedSeconds,parallel.entities()[i].stoppedSeconds,"parallel stop determinism");
        }
        if(Simulation::maximumCollisionWorkers()>1) {
            require(parallel.parallelCollisionBatches()>0,"parallel collision batches executed");
            auto caller=std::this_thread::get_id(); std::atomic<unsigned> offThread{0};
            require(detail::parallelAll(100,4,[&](std::size_t) { if(std::this_thread::get_id()!=caller) ++offThread; return true; }),"worker predicate result");
            require(offThread>0,"work ran on a worker thread");
            bool caught=false;
            try { detail::parallelAll(100,4,[](std::size_t i) { if(i==50) throw std::runtime_error("test"); return true; }); }
            catch(const std::runtime_error&) { caught=true; }
            require(caught,"worker exception propagated after joining");
            require(!detail::parallelAll(100,4,[](std::size_t i) { return i!=37; }),"worker rejection propagated");
        }
        std::cout<<"Stopped-time telemetry and parallel collision checks passed\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
