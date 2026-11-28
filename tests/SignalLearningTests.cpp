#include "traffic/SignalLearning.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace traffic;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
bool near(double a,double b){return std::abs(a-b)<1e-7;}
int main(){try{
    TrafficLight west;west.id=11;west.a={20,0};west.b={20,10};west.outDirection={1,0};
    TrafficLight north;north.id=22;north.a={0,20};north.b={10,20};north.outDirection={0,1};
    auto lights=std::vector<TrafficLight>{west,north};auto regions=defaultSignalRegions(lights);for(auto& region:regions)region.depth=10;
    Entity whole;whole.id=1;whole.center={15,5};whole.heading=0;whole.length=4;whole.width=2;
    Entity half=whole;half.id=2;half.center={10,8}; // Half of the footprint lies outside the upstream rectangle.
    Entity incoming=whole;incoming.id=3;incoming.center={15,2};incoming.heading=3.141592653589793;
    Entity downstream=whole;downstream.id=4;downstream.center={25,5};
    auto observation=observeSignals(lights,{whole,half,incoming,downstream},regions);
    require(near(observation.density[0],.12),"Density did not clip bodies or exclude IN/downstream traffic");
    require(observation.regions[0].vehicles==2 && observation.mask==std::array<double,2>{1,1},"Observation group count/mask is wrong");
    auto rotated=whole;rotated.center={5,15};rotated.heading=3.141592653589793*.5;
    require(near(observeSignals(lights,{rotated},regions).density[1],.08),"Rotated signal measurement is wrong");
    auto twin=west;twin.id=33;twin.a={40,0};twin.b={40,20};lights.push_back(twin);auto weighted=defaultSignalRegions(lights);for(auto& region:weighted)region.depth=10;
    require(near(observeSignals(lights,{whole,half},weighted).density[0],.04),"Group densities must weight upstream areas");
    auto baseline=lights;SignalTimingController controller;controller.begin(lights,weighted,0,3,1);controller.queue({10,20});
    for(const auto& light:lights)require(light.phase(0)!=LightPhase::Green,"Enable skipped the yellow guard");
    require(!controller.update(lights,3.9) && controller.update(lights,4),"Initial cycle did not wait for clearance");
    controller.queue({8,12});require(!controller.update(lights,10) && near(lights[0].greenSeconds,10),"Policy changed a cycle in progress");
    for(int i=40;i<420;++i){double t=i*.1;require(!(lights[0].phase(t)==LightPhase::Green && lights[1].phase(t)==LightPhase::Green),"Conflicting groups were green together");require(lights[0].phase(t)==lights[2].phase(t),"Matching lights were not synchronized");}
    require(controller.update(lights,42) && near(lights[0].greenSeconds,8) && near(lights[1].greenSeconds,12),"Pending timing did not apply at the cycle boundary");
    bool rejected=false;try{controller.queue({0,NAN});}catch(...){rejected=true;}require(rejected,"Invalid model action was accepted");
    controller.stop(lights);require(!controller.active() && near(lights[0].greenSeconds,baseline[0].greenSeconds) && near(lights[1].offsetSeconds,baseline[1].offsetSeconds),"Stopping policy did not restore manual timing");
    lights[1].enabled=false;rejected=false;try{controller.begin(lights,weighted,0);}catch(...){rejected=true;}require(rejected,"Disabled signals were accepted for synchronized control");
    std::cout<<"Signal density, OUT filtering, area weighting, paired synchronization, clearance, cycle-boundary actions and manual restoration passed\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
