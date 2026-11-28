#include "traffic/TrainingData.hpp"
#include "traffic/TrainingDataset.hpp"
#include "traffic/NodeGraph.hpp"
#include <filesystem>
#include <iostream>
#include <memory>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

using namespace traffic;
namespace {
struct Config {
    std::string map,nodes,signals,output="build/training-data",groups;
    int episodes=10,maxVehicles=500;std::uint32_t seed=42;
    double duration=600,interval=5,scale=1,maxRed=60,tr=8,yellow=3,minGreen=5;
    bool interactive=false,append=true;QueueConfig queue;DemandConfig demand;
};
void jsonString(std::ostream& out,const std::string& text){out<<'"';for(char c:text){if(c=='"' || c=='\\')out<<'\\'<<c;else if(c=='\n')out<<"\\n";else if(static_cast<unsigned char>(c)>=32)out<<c;}out<<'"';}
class Environment {
public:
    Config config;std::unique_ptr<Simulation> sim;NodeGraph graph;std::vector<TrafficLight> lights;std::vector<SignalRegion> regions;
    SignalWaitTracker tracker;RandomDemand demand;HoldSignalController hold;TrainingRecorder recorder;TrainingDataset dataset;std::mt19937 random;
    int episode=0;double nextSample=0;bool done=false;TrainingSnapshot current;std::uint64_t requestedSpawns=0,insertedSpawns=0;
    explicit Environment(Config value):config(std::move(value)){}
    ~Environment(){try{finish("closed");}catch(...){}}
    TrainingSnapshot snapshot(){auto result=tracker.snapshot();for(int group=0;group<2;++group)result.groups[group].countdown=hold.countdown(group,result.time);for(auto& [id,q]:result.lights){auto region=std::find_if(regions.begin(),regions.end(),[&](auto& r){return r.lightId==id;});if(region!=regions.end())q.countdown=hold.countdown(region->group,result.time);}return result;}
    void finish(const std::string& outcome){if(sim && recorder.active()){if(!done)recorder.transition(current,-1,current,true,false);tracker.finish(sim->seconds(),outcome);recorder.waits(tracker.takeCompleted());recorder.close();}if(dataset.active())dataset.finish();done=true;}
    void reset(std::uint32_t seed){
        if(sim)finish("reset");graph=NodeGraph::load(config.nodes);auto issues=graph.missingLaneExits();if(!issues.empty())throw std::runtime_error("Assign lane exits before training: "+issues.front());
        auto map=Map::fromPng(config.map,config.scale);auto lanes=compileRoads(graph).lanes;validateRoadMask(lanes,map);sim=std::make_unique<Simulation>(map,seed);sim->setCollisionWorkers(1);sim->setLanes(std::move(lanes));sim->setTrafficFlowSettings({true,false,.4,.15});sim->setStuckVehicleSettings({false,30,false,60});sim->setTrafficRecoverySettings({false,1,25});auto routing=sim->routingSettings();routing.rtsStyle=routing.spreadAtSignals=routing.refreshMovingRoutes=false;sim->setRoutingSettings(routing);
        auto profiles=vietnameseCityProfiles();for(int kind=0;kind<vehicleKindCount;++kind)sim->setProfile(static_cast<VehicleKind>(kind),profiles[kind]);sim->loadTrafficLights(config.signals);lights=sim->trafficLights();regions=defaultSignalRegions(lights);for(auto& region:regions)region.depth=config.queue.horizon;
        if(!config.groups.empty()){std::istringstream input(config.groups);std::string value;std::size_t index=0;while(std::getline(input,value,',')){if(index>=regions.size() || (value!="0" && value!="1"))throw std::runtime_error("--groups needs one 0/1 value per signal, sorted by signal ID");regions[index++].group=std::stoi(value);}if(index!=regions.size())throw std::runtime_error("--groups count differs from the number of lights");}
        auto directory=std::filesystem::u8path(dataset.begin(config.output,config.append || episode>0,seed));++episode;
        try {
        auto randomized=config.demand;randomized.seed=seed;recorder.open(directory.u8string(),seed,config.queue,randomized,config.maxRed,config.tr,config.yellow,0);
        map.savePng((directory/"map.png").u8string());graph.save((directory/"network.nodes").u8string());sim->saveTrafficLights((directory/"map.png.signals").u8string());sim->saveTrafficLights((directory/"network.nodes.signals").u8string());
        std::ofstream cameras(directory/"camera_groups.csv");cameras<<"signal_id,group,camera_range_m,camera_width_m\n";for(auto& region:regions)cameras<<region.lightId<<','<<region.group<<','<<config.queue.horizon<<','<<region.width<<'\n';
        std::ofstream setup(directory/"environment.json");setup<<std::setprecision(17)<<"{\"meters_per_pixel\":"<<config.scale<<",\"duration\":"<<config.duration<<",\"decision_interval\":"<<config.interval<<",\"physics_step\":0.1,\"max_vehicles\":"<<config.maxVehicles<<",\"min_green\":"<<config.minGreen<<",\"engine\":\"Native\",\"group_rule\":\"see_camera_groups.csv\"}\n";
        tracker.reset();requestedSpawns=insertedSpawns=0;hold.stop(lights);hold.begin(lights,regions,0,config.maxRed,config.tr,config.yellow,config.minGreen);sim->setTrafficLights(lights);
        std::vector<DemandSource> sources;for(std::size_t node=0;node<graph.nodes.size();++node)if(graph.nodes[node].spawn)sources.push_back({static_cast<int>(node),graph.nodes[node].rate});if(sources.empty())throw std::runtime_error("Mark at least one spawn node before training");demand.begin(sources,randomized,0);recorder.demand(demand.takeEvents());random.seed(seed^0x51e1f00d);done=false;nextSample=0;
        tracker.update(lights,sim->entities(),regions,0,config.queue);current=snapshot();recorder.sample(current);nextSample=1;
        }catch(...){recorder.close();dataset.cancel();done=true;throw;}
    }
    TrainingSnapshot step(int action,bool& forced,std::array<int,2> requests={-1,-1}){
        if(!sim || done)throw std::runtime_error("Reset the environment before stepping");if(action<0 || action>2)throw std::runtime_error("Action must be 0, 1 or 2 (hold)");auto state=current;hold.request(action==2?-1:action);forced=false;
        double target=std::min(config.duration,sim->seconds()+config.interval);
        while(sim->seconds()+1e-9<target){double now=sim->seconds();if(hold.update(lights,now))sim->setTrafficLights(lights);forced=forced || hold.forced();demand.update(now);recorder.demand(demand.takeEvents());
            for(auto& source:demand.sources())while(demand.due(source.node,now)){
                std::vector<int> entries;for(std::size_t lane=0;lane<sim->lanes().size();++lane)if(sim->lanes()[lane].entry && sim->lanes()[lane].sourceNode==source.node)entries.push_back(static_cast<int>(lane));
                bool inserted=false;if(!entries.empty() && sim->entities().size()<static_cast<std::size_t>(config.maxVehicles)){int lane=entries[std::uniform_int_distribution<std::size_t>(0,entries.size()-1)(random)];inserted=sim->spawnRandomOnLane(lane);}++requestedSpawns;if(inserted)++insertedSpawns;recorder.spawn(now,source.node,source.rate,inserted);
            }
            sim->tick(std::min(.1,target-now));tracker.update(lights,sim->entities(),regions,sim->seconds(),config.queue);recorder.waits(tracker.takeCompleted());current=snapshot();
            if(sim->seconds()+1e-9>=nextSample){recorder.sample(current);nextSample+=1;}
        }
        done=sim->seconds()+1e-9>=config.duration;recorder.transition(state,action==2?-1:action,current,done,forced,requests);if(done){tracker.finish(sim->seconds());recorder.waits(tracker.takeCompleted());recorder.close();dataset.finish();}return state;
    }
    void response(const TrainingSnapshot* previous=nullptr,bool forced=false){auto values=trainingFeatures(current);std::cout<<std::setprecision(17)<<"{\"observation\":[";for(int group=0;group<2;++group){if(group)std::cout<<',';std::cout<<'[';for(int feature=0;feature<trainingFeatureCount;++feature){if(feature)std::cout<<',';std::cout<<values[group*trainingFeatureCount+feature];}std::cout<<']';}std::cout<<"],\"rewards\":[";for(int group=0;group<2;++group){if(group)std::cout<<',';std::cout<<(previous?-(current.totalWaitByGroup[group]-previous->totalWaitByGroup[group]):0);}std::cout<<"],\"done\":"<<(done?"true":"false")<<",\"forced_switch\":"<<(forced?"true":"false")<<",\"time\":"<<current.time<<",\"arrivals\":"<<sim->completed()<<",\"active_vehicles\":"<<sim->entities().size()<<",\"requested_spawns\":"<<requestedSpawns<<",\"inserted_spawns\":"<<insertedSpawns<<",\"directory\":";jsonString(std::cout,recorder.directory());std::cout<<"}\n"<<std::flush;}
};
Config parse(int argc,char** argv){Config config;for(int i=1;i<argc;++i){std::string key=argv[i];if(key=="--interactive"){config.interactive=true;continue;}if(i+1==argc)throw std::runtime_error("Missing value for "+key);std::string value=argv[++i];if(key=="--map")config.map=value;else if(key=="--nodes")config.nodes=value;else if(key=="--signals")config.signals=value;else if(key=="--output")config.output=value;else if(key=="--append"){if(value=="true" || value=="True" || value=="1")config.append=true;else if(value=="false" || value=="False" || value=="0")config.append=false;else throw std::runtime_error("--append must be true or false");}else if(key=="--groups")config.groups=value;else if(key=="--episodes")config.episodes=std::stoi(value);else if(key=="--duration")config.duration=std::stod(value);else if(key=="--decision-interval")config.interval=std::stod(value);else if(key=="--seed")config.seed=static_cast<std::uint32_t>(std::stoul(value));else if(key=="--max-vehicles")config.maxVehicles=std::stoi(value);else if(key=="--meters-per-pixel")config.scale=std::stod(value);else if(key=="--max-red")config.maxRed=std::stod(value);else if(key=="--tr")config.tr=std::stod(value);else if(key=="--yellow")config.yellow=std::stod(value);else if(key=="--min-green")config.minGreen=std::stod(value);else if(key=="--camera-range")config.queue.horizon=std::stod(value);else if(key=="--short-limit")config.queue.shortLimit=std::stod(value);else if(key=="--moderate-limit")config.queue.moderateLimit=std::stod(value);else if(key=="--min-rate")config.demand.minimum=std::stod(value);else if(key=="--max-rate")config.demand.maximum=std::stod(value);else if(key=="--rate-change-min")config.demand.changeMin=std::stod(value);else if(key=="--rate-change-max")config.demand.changeMax=std::stod(value);else if(key=="--zero-probability")config.demand.zeroProbability=std::stod(value);else throw std::runtime_error("Unknown option "+key);}
    if(config.map.empty() || config.nodes.empty() || config.signals.empty())throw std::runtime_error("Required: --map map.png --nodes map.nodes --signals map.png.signals");if(!std::isfinite(config.duration) || !std::isfinite(config.interval) || !std::isfinite(config.scale) || config.duration<=0 || config.interval<=0 || config.interval>60 || config.scale<=0 || config.episodes<1 || config.maxVehicles<1)throw std::runtime_error("Invalid episode configuration");return config;}
}
int main(int argc,char** argv){try{auto config=parse(argc,argv);Environment env(config);if(config.interactive){std::string line;while(std::getline(std::cin,line)){try{std::istringstream command(line);std::string op;command>>op;if(op=="RESET"){std::uint32_t seed=config.seed;command>>seed;env.reset(seed);env.response();}else if(op=="STEP"){int action=-1;if(!(command>>action))throw std::runtime_error("STEP needs an action");std::array<int,2> requests{-1,-1};if(command>>requests[0]){if(!(command>>requests[1]) || (requests[0]!=0 && requests[0]!=1) || (requests[1]!=0 && requests[1]!=1))throw std::runtime_error("Invalid agent hold/request actions");}bool forced=false;auto previous=env.step(action,forced,requests);env.response(&previous,forced);}else if(op=="QUIT"){env.finish("closed");break;}else throw std::runtime_error("Use RESET seed, STEP action or QUIT");}catch(const std::exception& error){std::cout<<"{\"error\":";jsonString(std::cout,error.what());std::cout<<"}\n"<<std::flush;}}return 0;}
    std::mt19937 actions(config.seed^0xa6c124u);for(int episode=0;episode<config.episodes;++episode){actions.seed((config.seed+episode)^0xa6c124u);env.reset(config.seed+episode);while(!env.done){int roll=std::uniform_int_distribution<int>(0,9)(actions);int action=roll<6?2:roll<8?0:1;bool forced=false;env.step(action,forced);}std::cout<<"Episode "<<episode<<": "<<env.sim->completed()<<" arrivals; "<<env.recorder.directory()<<'\n';}return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
