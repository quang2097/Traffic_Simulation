#include "traffic/TrainingData.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace traffic {
namespace {
double dot(Vec2 a,Vec2 b){return a.x*b.x+a.y*b.y;}
WaitSummary summarize(std::vector<double> values){WaitSummary result;result.count=static_cast<int>(values.size());if(values.empty())return result;std::sort(values.begin(),values.end());result.minimum=values.front();result.maximum=values.back();result.median=(values[(values.size()-1)/2]+values[values.size()/2])*.5;for(double value:values)result.mean+=value;result.mean/=values.size();return result;}
int code(LightPhase phase){return phase==LightPhase::Red?0:phase==LightPhase::Yellow?1:2;}
double offset(double phase,double now,double cycle){double value=std::fmod(phase-now,cycle);return value<0?value+cycle:value;}
void row(std::ostream& out,double time,std::uint64_t id,const QueueFeatures& q){out<<time<<','<<id<<','<<q.category<<','<<queueCategoryName(q.category)<<','<<q.metres<<','<<q.vehicles<<','<<q.cameraLimit<<','<<q.phase<<','<<q.phaseAge<<','<<q.redAge<<','<<q.countdown<<','<<q.wait.count<<','<<q.wait.minimum<<','<<q.wait.maximum<<','<<q.wait.median<<','<<q.wait.mean<<','<<q.density<<'\n';}
void array(std::ostream& out,const TrainingSnapshot& snapshot){auto values=trainingFeatures(snapshot);out<<'[';for(std::size_t i=0;i<values.size();++i){if(i)out<<',';out<<values[i];}out<<']';}
}
const char* queueCategoryName(int category){static const char* names[]{"none","short","moderate","long"};return names[std::clamp(category,0,3)];}
std::array<double,trainingFeatureCount*2> trainingFeatures(const TrainingSnapshot& snapshot){std::array<double,trainingFeatureCount*2> result{};for(int group=0;group<2;++group){const auto& q=snapshot.groups[group];std::array<double,trainingFeatureCount> values{static_cast<double>(q.category),q.metres,static_cast<double>(q.vehicles),q.cameraLimit?1.:0.,static_cast<double>(q.phase),q.phaseAge,q.redAge,q.countdown,static_cast<double>(q.wait.count),q.wait.minimum,q.wait.maximum,q.wait.median,q.wait.mean,q.density};std::copy(values.begin(),values.end(),result.begin()+group*trainingFeatureCount);}return result;}
void SignalWaitTracker::reset(){pending_.clear();clocks_.clear();completed_.clear();snapshot_={};previousTime_=totalWait_=0;initialized_=false;waitByGroup_={};nextEpisode_=1;}
std::vector<WaitEpisode> SignalWaitTracker::takeCompleted(){auto result=std::move(completed_);completed_.clear();return result;}
void SignalWaitTracker::finish(double now,const std::string& outcome){for(auto& [id,pending]:pending_){pending.episode.end=now;pending.episode.outcome=outcome;completed_.push_back(pending.episode);}pending_.clear();}
void SignalWaitTracker::update(const std::vector<TrafficLight>& lights,const std::vector<Entity>& vehicles,const std::vector<SignalRegion>& regions,double now,const QueueConfig& config){
    if(!std::isfinite(now) || now<0 || !std::isfinite(config.horizon) || config.horizon<=0 || !std::isfinite(config.shortLimit) || !std::isfinite(config.moderateLimit) || config.shortLimit<0 || config.moderateLimit<=config.shortLimit || !std::isfinite(config.stoppedSpeed) || config.stoppedSpeed<0)throw std::invalid_argument("Invalid queue/camera configuration");
    if(now<previousTime_)reset();double dt=initialized_?now-previousTime_:0;initialized_=true;previousTime_=now;snapshot_={};snapshot_.time=now;
    auto findLight=[&](std::uint64_t id){return std::find_if(lights.begin(),lights.end(),[&](const auto& light){return light.id==id;});};
    for(const auto& light:lights)if(light.enabled){int phase=code(light.phase(now));auto [it,inserted]=clocks_.emplace(light.id,Clock{phase,now,phase==0?now:-1});auto& clock=it->second;if(clock.phase!=phase){clock.changed=now;if(phase==0)clock.redSince=now;else clock.redSince=-1;clock.phase=phase;}auto& q=snapshot_.lights[light.id];q.phase=phase;q.phaseAge=now-clock.changed;q.redAge=clock.redSince<0?0:now-clock.redSince;}
    std::map<std::uint64_t,const Entity*> present;for(const auto& vehicle:vehicles)present[vehicle.id]=&vehicle;
    for(auto it=pending_.begin();it!=pending_.end();){auto light=findLight(it->second.episode.light);auto vehicle=present.find(it->first);std::string outcome;
        if(vehicle==present.end())outcome="removed_before_crossing";else if(light==lights.end() || !light->enabled)outcome="signal_removed";else{auto d=light->controlledDirection();Vec2 side{-d.y,d.x};auto body=corners(vehicle->second->footprint());double rear=1e100,front=-1e100,lowSide=1e100,highSide=-1e100;for(auto p:body){auto delta=p-(light->a+light->b)*.5;rear=std::min(rear,dot(delta,d));front=std::max(front,dot(delta,d));lowSide=std::min(lowSide,dot(delta,side));highSide=std::max(highSide,dot(delta,side));}auto region=std::find_if(regions.begin(),regions.end(),[&](auto& r){return r.lightId==light->id;});double width=region!=regions.end() && region->width>0?region->width:length(light->b-light->a);if(rear>0)outcome="crossed";else if(front< -config.horizon || lowSide>width*.5 || highSide< -width*.5)outcome="left_camera";}
        if(!outcome.empty()){auto event=it->second.episode;event.end=now;event.outcome=outcome;completed_.push_back(event);it=pending_.erase(it);}else ++it;
    }
    for(const auto& vehicle:vehicles){bool stopped=vehicle.speed<=config.stoppedSpeed;
        auto pending=pending_.find(vehicle.id);
        if(pending==pending_.end() && stopped){double nearest=1e100;const TrafficLight* selected=nullptr;int group=0;
            Vec2 travel{std::cos(vehicle.heading),std::sin(vehicle.heading)};if(vehicle.reversing)travel=travel*-1;
            for(const auto& region:regions){auto light=findLight(region.lightId);if(light==lights.end() || !light->enabled || region.group<0 || region.group>1 || !light->controlsDirection(travel))continue;
                auto d=light->controlledDirection();Vec2 side{-d.y,d.x},delta=(light->a+light->b)*.5-vehicle.center;double ahead=dot(delta,d),width=region.width>0?region.width:length(light->b-light->a);
                if(ahead< -vehicle.length*.5 || ahead>config.horizon+vehicle.length*.5 || std::abs(dot(delta,side))>width*.5+vehicle.width*.5)continue;
                bool residual=std::any_of(pending_.begin(),pending_.end(),[&](const auto& item){return item.second.episode.light==light->id;});
                if(light->phase(now)==LightPhase::Green && !residual)continue;
                if(ahead<nearest){nearest=ahead;selected=&*light;group=region.group;}
            }
            if(selected){WaitEpisode episode;episode.episode=nextEpisode_++;episode.vehicle=vehicle.id;episode.light=selected->id;episode.group=group;episode.firstStop=now-dt;pending=pending_.emplace(vehicle.id,Pending{episode,false}).first;}
        }
        if(pending!=pending_.end()){if(stopped){pending->second.episode.wait+=dt;totalWait_+=dt;waitByGroup_[pending->second.episode.group]+=dt;if(!pending->second.stopped)++pending->second.episode.bursts;}pending->second.stopped=stopped;}
    }
    std::map<std::uint64_t,std::vector<double>> lightWaits;std::array<std::vector<double>,2> groupWaits;
    for(const auto& [id,pending]:pending_){auto found=present.find(id);auto light=findLight(pending.episode.light);if(found==present.end() || light==lights.end())continue;const auto& vehicle=*found->second;
        auto& q=snapshot_.lights[light->id];auto d=light->controlledDirection();double tail=0;for(auto corner:corners(vehicle.footprint()))tail=std::max(tail,-dot(corner-(light->a+light->b)*.5,d));q.metres=std::max(q.metres,std::min(config.horizon,tail));q.cameraLimit=q.cameraLimit || tail>=config.horizon-.01;++q.vehicles;
        lightWaits[light->id].push_back(pending.episode.wait);groupWaits[pending.episode.group].push_back(pending.episode.wait);
    }
    auto cameraRegions=regions;for(auto& region:cameraRegions)region.depth=config.horizon;auto densities=observeSignals(lights,vehicles,cameraRegions);std::array<bool,2> initialized{};
    for(const auto& region:regions){auto found=snapshot_.lights.find(region.lightId);if(found==snapshot_.lights.end())continue;auto& q=found->second;q.wait=summarize(lightWaits[region.lightId]);double extent=config.byCount?q.vehicles:q.metres;q.category=q.vehicles==0?0:extent<=config.shortLimit?1:extent<=config.moderateLimit?2:3;
        auto density=std::find_if(densities.regions.begin(),densities.regions.end(),[&](auto& value){return value.lightId==region.lightId;});if(density!=densities.regions.end())q.density=density->density;
        if(region.group<0 || region.group>1)continue;auto& group=snapshot_.groups[region.group];if(!initialized[region.group]){group.phase=q.phase;group.phaseAge=q.phaseAge;initialized[region.group]=true;}else{if(group.phase!=q.phase)group.phase=-1;group.phaseAge=std::min(group.phaseAge,q.phaseAge);}group.redAge=std::max(group.redAge,q.redAge);group.metres=std::max(group.metres,q.metres);group.vehicles+=q.vehicles;group.cameraLimit=group.cameraLimit || q.cameraLimit;
    }
    for(int group=0;group<2;++group){auto& q=snapshot_.groups[group];q.wait=summarize(groupWaits[group]);double extent=config.byCount?q.vehicles:q.metres;q.category=q.vehicles==0?0:extent<=config.shortLimit?1:extent<=config.moderateLimit?2:3;q.density=densities.density[group];}snapshot_.totalWait=totalWait_;snapshot_.totalWaitByGroup=waitByGroup_;
}
void HoldSignalController::begin(std::vector<TrafficLight>& lights,const std::vector<SignalRegion>& regions,double now,double maxRed,double tr,double yellow,double minGreen){
    if(active_ || !std::isfinite(now) || now<0 || !std::isfinite(maxRed) || !std::isfinite(tr) || !std::isfinite(yellow) || !std::isfinite(minGreen) || minGreen<0 || maxRed<tr+minGreen || tr<.1 || yellow<.1 || maxRed>3600 || tr>120 || yellow>30)throw std::invalid_argument("Maximum red must allow Tr plus minimum green; use positive Tr/yellow");
    if(lights.size()<2 || lights.size()>4)throw std::invalid_argument("Assign 2-4 lights to two groups");std::array<bool,2> groups{};
    for(const auto& light:lights){auto region=std::find_if(regions.begin(),regions.end(),[&](auto& value){return value.lightId==light.id;});if(!light.enabled || region==regions.end() || region->group<0 || region->group>1)throw std::invalid_argument("Enable and assign every signal to a group");groups[region->group]=true;}if(!groups[0] || !groups[1])throw std::invalid_argument("Both signal groups need a light");
    original_=lights;regions_=regions;maxRed_=maxRed;tr_=tr;yellow_=yellow;minGreen_=minGreen;stage_=3;current_=target_=0;requested_=-1;stageStart_=greenStart_=lastNow_=now;redSince_={now+yellow,now+yellow};deadline_={-1,-1};active_=true;forced_=false;paint(lights,now);
}
void HoldSignalController::request(int group){if(group< -1 || group>1)throw std::invalid_argument("Hold action must be -1, 0 or 1");requested_=group;}
double HoldSignalController::countdown(int group,double now) const {return active_ && stage_==2 && group==target_?std::max(0.,stageStart_+tr_-now):-1;}
void HoldSignalController::paint(std::vector<TrafficLight>& lights,double now){
    for(auto& light:lights){auto region=std::find_if(regions_.begin(),regions_.end(),[&](const auto& r){return r.lightId==light.id;});if(region==regions_.end())throw std::invalid_argument("Signal layout changed during hold control");int group=region->group;bool green=stage_==0 && group==current_,yellow=(stage_==1 && group==current_) || stage_==3;
        light.externallyControlled=true;light.externalPhase=green?LightPhase::Green:yellow?LightPhase::Yellow:LightPhase::Red;
    }
}
bool HoldSignalController::update(std::vector<TrafficLight>& lights,double now){
    if(!active_)return false;if(now<lastNow_ || !std::isfinite(now))throw std::invalid_argument("Hold controller clock moved backwards");lastNow_=now;
    if(lights.size()!=original_.size())throw std::invalid_argument("Signal layout changed during hold control");for(const auto& light:lights)if(std::none_of(original_.begin(),original_.end(),[&](const auto& old){return light.id==old.id && light.enabled==old.enabled && length(light.a-old.a)+length(light.b-old.b)<1e-8 && length(light.controlledDirection()-old.controlledDirection())<1e-8;}))throw std::invalid_argument("Signal geometry changed during hold control");
    bool changed=false;
    // Start Tr when the engine actually changes to all-red. A step crossing a
    // boundary may lengthen yellow/clearance, but must never shorten Tr.
    if((stage_==1 || stage_==3) && now+1e-9>=stageStart_+yellow_){stageStart_=now;if(stage_==1)redSince_[current_]=now;else redSince_={now,now};stage_=2;changed=true;}
    if(stage_==2 && now+1e-9>=stageStart_+tr_){stageStart_=now;stage_=0;current_=target_;greenStart_=now;redSince_[current_]=-1;deadline_[current_]=-1;requested_=-1;forced_=false;changed=true;}
    if(stage_==0){int other=1-current_;if(redSince_[other]<0)redSince_[other]=greenStart_;bool expired=now-redSince_[other]+1e-9>=maxRed_;if(expired || (requested_==other && now-greenStart_+1e-9>=minGreen_)){target_=other;stage_=1;stageStart_=now;forced_=expired;requested_=-1;changed=true;}}
    // Re-anchor constant held phases; no automatic fixed-cycle release is allowed.
    paint(lights,now);return changed;
}
void HoldSignalController::stop(std::vector<TrafficLight>& lights){if(active_)for(auto& light:lights){auto found=std::find_if(original_.begin(),original_.end(),[&](auto& old){return old.id==light.id;});if(found!=original_.end()){light.greenSeconds=found->greenSeconds;light.yellowSeconds=found->yellowSeconds;light.redSeconds=found->redSeconds;light.offsetSeconds=found->offsetSeconds;light.externallyControlled=found->externallyControlled;light.externalPhase=found->externalPhase;}}active_=false;original_.clear();}
double RandomDemand::arrival(double now,double rate){return rate<=0?std::numeric_limits<double>::infinity():now+std::exponential_distribution<double>(rate)(random_);}
void RandomDemand::draw(DemandSource& source,double now){bool quiet=std::bernoulli_distribution(config_.zeroProbability)(random_);source.rate=quiet?0:std::exp(std::uniform_real_distribution<double>(std::log(std::max(.000001,config_.minimum)),std::log(config_.maximum))(random_));source.nextChange=now+std::uniform_real_distribution<double>(config_.changeMin,config_.changeMax)(random_);source.nextArrival=arrival(now,source.rate);events_.push_back({now,source.rate,source.node});}
void RandomDemand::begin(std::vector<DemandSource> sources,const DemandConfig& config,double now){if(!std::isfinite(config.minimum) || !std::isfinite(config.maximum) || !std::isfinite(config.changeMin) || !std::isfinite(config.changeMax) || !std::isfinite(config.zeroProbability) || config.minimum<0 || config.maximum<=0 || config.maximum<config.minimum || config.maximum>100 || config.changeMin<=0 || config.changeMax<config.changeMin || config.zeroProbability<0 || config.zeroProbability>1)throw std::invalid_argument("Invalid randomized demand configuration");config_=config;sources_=std::move(sources);events_.clear();random_.seed(config.seed);active_=true;for(auto& source:sources_)draw(source,now);}
void RandomDemand::update(double now){if(!std::isfinite(now) || now<0)throw std::invalid_argument("Invalid demand clock");if(active_)for(auto& source:sources_)while(std::min(source.nextArrival,source.nextChange)<=now+1e-9){if(source.nextArrival<source.nextChange){++source.pendingArrivals;source.nextArrival=arrival(source.nextArrival,source.rate);}else draw(source,source.nextChange);}}
bool RandomDemand::due(int node,double now){if(!active_)return false;update(now);auto source=std::find_if(sources_.begin(),sources_.end(),[&](auto& s){return s.node==node;});if(source==sources_.end() || !source->pendingArrivals)return false;--source->pendingArrivals;return true;}
std::vector<DemandEvent> RandomDemand::takeEvents(){auto result=std::move(events_);events_.clear();return result;}
void TrainingRecorder::open(const std::string& directory,std::uint32_t seed,const QueueConfig& queue,const DemandConfig& demand,double maxRed,double tr,double yellow,double clearance){
    close();auto path=std::filesystem::u8path(directory);if(std::filesystem::exists(path) && !std::filesystem::is_empty(path))throw std::invalid_argument("Training directory must be new or empty; existing data will not be overwritten");std::filesystem::create_directories(path);directory_=directory;seed_=seed;
    auto file=[&](std::ofstream& stream,const char* name){stream.exceptions(std::ios::badbit|std::ios::failbit);stream.open(path/name);stream<<std::setprecision(17);};
    file(groups_,"groups.csv");file(lights_,"lights.csv");file(waits_,"wait_episodes.csv");file(rates_,"spawn_rates.csv");file(spawns_,"spawn_attempts.csv");file(transitions_,"transitions.jsonl");
    const char* header="time_s,id,queue_code,queue_label,visible_queue_m,queued_vehicles,camera_limit,phase,phase_age_s,red_age_s,tr_remaining_s,wait_count,wait_min_s,wait_max_s,wait_median_s,wait_mean_s,density\n";groups_<<header;lights_<<header;waits_<<"episode,vehicle_id,signal_id,group,first_stop_s,end_s,cumulative_wait_s,stop_bursts,outcome\n";rates_<<"time_s,node_id,vehicles_per_s\n";spawns_<<"time_s,node_id,rate,inserted\n";
    std::ofstream meta(path/"metadata.json");meta<<std::setprecision(17)<<"{\"schema\":2,\"seed\":"<<seed<<",\"camera_range_m\":"<<queue.horizon<<",\"short_limit_m\":"<<queue.shortLimit<<",\"moderate_limit_m\":"<<queue.moderateLimit<<",\"stopped_speed_m_s\":"<<queue.stoppedSpeed<<",\"max_red_s\":"<<maxRed<<",\"Tr_s\":"<<tr<<",\"yellow_s\":"<<yellow<<",\"extra_clearance_s\":"<<clearance<<",\"rate_min\":"<<demand.minimum<<",\"rate_max\":"<<demand.maximum<<",\"rate_change_min_s\":"<<demand.changeMin<<",\"rate_change_max_s\":"<<demand.changeMax<<",\"zero_rate_probability\":"<<demand.zeroProbability<<",\"features_per_group\":14,\"feature_names\":[\"queue_code\",\"visible_queue_m\",\"queued_vehicles\",\"camera_limit\",\"phase\",\"phase_age_s\",\"red_age_s\",\"tr_remaining_s\",\"wait_count\",\"wait_min_s\",\"wait_max_s\",\"wait_median_s\",\"wait_mean_s\",\"density\"],\"action_codes\":{\"0\":\"request_group_1\",\"1\":\"request_group_2\",\"2\":\"hold\"},\"reward\":\"negative incremental signal wait seconds\"}\n";
}
void TrainingRecorder::sample(const TrainingSnapshot& snapshot){if(!active())return;for(int group=0;group<2;++group)row(groups_,snapshot.time,group,snapshot.groups[group]);for(const auto& [id,q]:snapshot.lights)row(lights_,snapshot.time,id,q);}
void TrainingRecorder::waits(const std::vector<WaitEpisode>& episodes){if(active())for(const auto& e:episodes)waits_<<e.episode<<','<<e.vehicle<<','<<e.light<<','<<e.group<<','<<e.firstStop<<','<<e.end<<','<<e.wait<<','<<e.bursts<<','<<e.outcome<<'\n';}
void TrainingRecorder::demand(const std::vector<DemandEvent>& events){if(active())for(auto e:events)rates_<<e.time<<','<<e.node<<','<<e.rate<<'\n';}
void TrainingRecorder::spawn(double now,int node,double rate,bool inserted){if(active())spawns_<<now<<','<<node<<','<<rate<<','<<inserted<<'\n';}
void TrainingRecorder::transition(const TrainingSnapshot& state,int requested,const TrainingSnapshot& next,bool done,bool forced,std::array<int,2> agentActions){if(!active())return;if(agentActions[0]<0)agentActions={requested==0?1:0,requested==1?1:0};transitions_<<"{\"schema\":2,\"seed\":"<<seed_<<",\"time\":"<<state.time<<",\"next_time\":"<<next.time<<",\"observation\":";array(transitions_,state);transitions_<<",\"action\":"<<(requested<0?2:requested)<<",\"agent_actions\":["<<agentActions[0]<<','<<agentActions[1]<<"],\"reward\":"<<-(next.totalWait-state.totalWait)<<",\"rewards\":["<<-(next.totalWaitByGroup[0]-state.totalWaitByGroup[0])<<','<<-(next.totalWaitByGroup[1]-state.totalWaitByGroup[1])<<"],\"next_observation\":";array(transitions_,next);transitions_<<",\"done\":"<<(done?"true":"false")<<",\"forced_switch\":"<<(forced?"true":"false")<<"}\n";}
void TrainingRecorder::close(){for(auto* stream:{&groups_,&lights_,&waits_,&rates_,&spawns_,&transitions_}){if(stream->is_open())stream->close();stream->clear();}}
}
