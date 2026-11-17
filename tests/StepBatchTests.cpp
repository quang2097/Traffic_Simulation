#include "traffic/Simulation.hpp"
#include "traffic/StepBatch.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace traffic;
void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
void near(double a,double b,const char* message) { require(std::abs(a-b)<1e-8,message); }
Simulation scene() {
    Simulation s(Map(60,20,std::vector<Tile>(1200,Tile::Road)),42);
    s.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{1,0},{4,0,4,0},1});
    auto routing=s.routingSettings(); routing.spreadAtSignals=false; s.setRoutingSettings(routing);
    s.setTrafficLights({{{20,.01},{20,19.99},.5,.5,1,1,true}});
    require(s.spawnAt({10,10},{50,10},VehicleKind::Car,0),"step test spawn"); return s;
}
int main() {
    try {
        auto manual=scene(),fixed=scene(); StepBatch batch;
        for(int iteration=0;iteration<10;++iteration) {
            batch.request(.5);
            int calls=0;
            // Chunk sizes stand in for varying render/CPU budgets; callback
            // count and physics outcome must not depend on those budgets.
            while(batch.active()) {
                for(int work=0;work<1+iteration && batch.active();++work)
                    batch.advanceOne([&](double dt) { require(dt<=1./120+1e-12,"physics quantum remains small"); manual.tick(dt); ++calls; });
            }
            require(calls==60,"half-second step completes its full duration");
            near(batch.completedSeconds(),.5,"step completion duration"); near(batch.progress(),1,"completed progress");
            for(int i=0;i<60;++i) fixed.tick(1./120);
            near(manual.seconds(),fixed.seconds(),"manual and continuous simulation clocks agree");
            require(manual.entities().size()==fixed.entities().size(),"manual population parity");
            for(std::size_t i=0;i<manual.entities().size();++i) {
                near(length(manual.entities()[i].center-fixed.entities()[i].center),0,"manual physics trajectory parity");
                near(manual.entities()[i].stoppedSeconds,fixed.entities()[i].stoppedSeconds,"manual wait telemetry parity");
            }
            require(manual.trafficLights()[0].phase(manual.seconds())==fixed.trafficLights()[0].phase(fixed.seconds()),"manual signal timing parity");
        }
        double before=manual.seconds(); int idleCalls=0;
        require(!batch.advanceOne([&](double) { ++idleCalls; }),"finished batch stays idle");
        near(manual.seconds(),before,"idle frames add no simulation time"); require(idleCalls==0,"idle batch performs no work");
        batch.request(.017); double elapsed=0;
        while(batch.active()) batch.advanceOne([&](double dt) { elapsed+=dt; });
        near(elapsed,.017,"non-integral tick duration advances exactly requested time");
        batch.request(1); batch.advanceOne([&](double dt) { manual.tick(dt); });
        double advanced=manual.seconds(); require(batch.progress()>0 && batch.progress()<1,"partial progress visible");
        bool busyRejected=false;
        try { batch.request(1); } catch(const std::logic_error&) { busyRejected=true; }
        require(busyRejected,"cannot overwrite an unfinished step");
        batch.cancel(); require(!batch.active(),"cancel clears pending time");
        require(!batch.advanceOne([&](double dt) { manual.tick(dt); }),"cancelled work does not resume");
        near(manual.seconds(),advanced,"cancel retains already advanced time");
        for(double invalid:{0.,-1.,3601.,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
            bool rejected=false; try { batch.request(invalid); } catch(const std::invalid_argument&) { rejected=true; }
            require(rejected,"invalid step duration rejected");
        }
        std::cout<<"On-demand step duration, physics/signals/telemetry parity and cancellation passed\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
