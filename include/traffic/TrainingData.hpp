#pragma once
#include "traffic/SignalLearning.hpp"
#include <fstream>
#include <map>
#include <random>

namespace traffic {
struct QueueConfig {double shortLimit=20,moderateLimit=60,horizon=80,stoppedSpeed=.05;bool byCount=false;};
struct WaitSummary {int count=0;double minimum=0,maximum=0,median=0,mean=0;};
struct QueueFeatures {
    int category=0,vehicles=0,phase=-1;bool cameraLimit=false;
    double metres=0,phaseAge=0,redAge=0,countdown=-1,density=0;
    WaitSummary wait;
};
struct WaitEpisode {std::uint64_t episode=0,vehicle=0,light=0;int group=0,bursts=0;double firstStop=0,end=0,wait=0;std::string outcome;};
struct TrainingSnapshot {double time=0,totalWait=0;std::array<double,2> totalWaitByGroup{};std::array<QueueFeatures,2> groups;std::map<std::uint64_t,QueueFeatures> lights;};
const char* queueCategoryName(int category);
constexpr int trainingFeatureCount=14;
std::array<double,trainingFeatureCount*2> trainingFeatures(const TrainingSnapshot& snapshot);
class SignalWaitTracker {
public:
    void reset();
    void update(const std::vector<TrafficLight>& lights,const std::vector<Entity>& vehicles,const std::vector<SignalRegion>& regions,double now,const QueueConfig& config={});
    const TrainingSnapshot& snapshot() const {return snapshot_;}
    std::vector<WaitEpisode> takeCompleted();
    void finish(double now,const std::string& outcome="episode_end");
private:
    struct Pending {WaitEpisode episode;bool stopped=false;};
    struct Clock {int phase=-1;double changed=0,redSince=0;};
    std::map<std::uint64_t,Pending> pending_;
    std::map<std::uint64_t,Clock> clocks_;
    std::vector<WaitEpisode> completed_;
    TrainingSnapshot snapshot_;
    double previousTime_=0,totalWait_=0;
    bool initialized_=false;
    std::array<double,2> waitByGroup_{};
    std::uint64_t nextEpisode_=1;
};
// Hold a green indefinitely until requested otherwise or a red deadline wins.
// Once maxRed is reached a forced switch finishes yellow, then holds all red
// for the full Tr clearance period before the waiting group receives green.
class HoldSignalController {
public:
    void begin(std::vector<TrafficLight>& lights,const std::vector<SignalRegion>& regions,double now,double maxRed=60,double tr=8,double yellow=3,double minGreen=5);
    bool update(std::vector<TrafficLight>& lights,double now);
    void request(int group); // -1 = hold, 0/1 = request that group's green.
    void stop(std::vector<TrafficLight>& lights);
    bool active() const {return active_;}
    int greenGroup() const {return stage_==0?current_:-1;}
    bool forced() const {return forced_;}
    double countdown(int group,double now) const;
private:
    std::vector<TrafficLight> original_;
    std::vector<SignalRegion> regions_;
    double maxRed_=60,tr_=8,yellow_=3,minGreen_=5,stageStart_=0,greenStart_=0,lastNow_=0;
    std::array<double,2> redSince_{},deadline_{-1,-1};
    int stage_=2,current_=0,target_=0,requested_=-1;
    bool active_=false,forced_=false;
    void paint(std::vector<TrafficLight>& lights,double now);
};
struct DemandConfig {std::uint32_t seed=42;double minimum=.02,maximum=1.5,changeMin=10,changeMax=40,zeroProbability=.15;};
struct DemandSource {int node=0;double originalRate=0,rate=0,nextChange=0,nextArrival=0;std::uint64_t pendingArrivals=0;};
struct DemandEvent {double time=0,rate=0;int node=0;};
class RandomDemand {
public:
    void begin(std::vector<DemandSource> sources,const DemandConfig& config,double now=0);
    void update(double now);
    bool due(int node,double now);
    const std::vector<DemandSource>& sources() const {return sources_;}
    std::vector<DemandEvent> takeEvents();
    bool active() const {return active_;}
    void stop(){active_=false;sources_.clear();events_.clear();}
private:
    std::vector<DemandSource> sources_;std::vector<DemandEvent> events_;DemandConfig config_;std::mt19937 random_;bool active_=false;
    double arrival(double now,double rate);
    void draw(DemandSource& source,double now);
};
class TrainingRecorder {
public:
    void open(const std::string& directory,std::uint32_t seed,const QueueConfig& queue,const DemandConfig& demand,double maxRed,double tr,double yellow,double clearance);
    void sample(const TrainingSnapshot& snapshot);
    void waits(const std::vector<WaitEpisode>& episodes);
    void demand(const std::vector<DemandEvent>& events);
    void spawn(double now,int node,double rate,bool inserted);
    void transition(const TrainingSnapshot& state,int requested,const TrainingSnapshot& next,bool done,bool forced,std::array<int,2> agentActions={-1,-1});
    void close();
    bool active() const {return groups_.is_open();}
    const std::string& directory() const {return directory_;}
private:
    std::string directory_;std::ofstream groups_,lights_,waits_,rates_,spawns_,transitions_;std::uint32_t seed_=0;
};
}
