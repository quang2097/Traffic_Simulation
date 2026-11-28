#include "traffic/PolicyRuntime.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace traffic;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
PolicyReply wait(PolicyRuntime& runtime){auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(15);PolicyReply reply;while(!runtime.poll(reply)){if(std::chrono::steady_clock::now()>deadline)throw std::runtime_error("Native policy worker timed out");std::this_thread::sleep_for(std::chrono::milliseconds(5));}if(reply.kind==PolicyReply::Error)throw std::runtime_error(reply.message);return reply;}
int main(int argc,char** argv){try{
    require(argc==4 || argc==5 || argc==8,"Expected Python, worker, weights, optional v2 and algorithm/action expectations");bool training=argc>=5;PolicyRuntime runtime;
    runtime.load(argv[1],argv[2],argv[3],"Auto");auto reply=wait(runtime);
    require(reply.kind==PolicyReply::Ready && reply.message==(argc==8?argv[5]:"MAPPO") && runtime.loaded(),"Native loader did not detect the model");
    require(runtime.trainingSchema()==training,"Native worker lost the policy observation schema");
    SignalObservation observation;observation.density={.2,.8};observation.mask={1,1};if(training)runtime.requestTraining(TrainingSnapshot{}, {1,1});else runtime.request(observation);reply=wait(runtime);
    if(training)require(reply.kind==PolicyReply::HoldAction && reply.green==(argc==8?std::array<double,2>{std::stod(argv[6]),std::stod(argv[7])}:std::array<double,2>{0,1}),"Native worker lost the hold/request actions");
    else require(reply.kind==PolicyReply::Action && std::abs(reply.green[0]-15)<1e-6 && std::abs(reply.green[1]-45)<1e-6,"Native worker transport changed the policy actions");
    runtime.request(observation);runtime.shutdown();require(!runtime.loaded() && !runtime.busy(),"Policy cancellation left work active");
    std::cout<<"Native asynchronous policy load, auto-detection, inference and cancellation passed\n";return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
