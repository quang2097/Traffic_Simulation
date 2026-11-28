#include "traffic/SimulationView.hpp"
#include "imgui.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <exception>

namespace traffic {
SimulationView::~SimulationView(){try{stopTraining("app_closed");}catch(...) {}}
TrainingSnapshot SimulationView::trainingSnapshot() const {
    auto result=waitTracker_.snapshot();for(int group=0;group<2;++group)result.groups[group].countdown=holdTiming_.countdown(group,result.time);
    for(auto& [id,q]:result.lights){auto region=std::find_if(policyRegions_.begin(),policyRegions_.end(),[&](auto& r){return r.lightId==id;});if(region!=policyRegions_.end())q.countdown=holdTiming_.countdown(region->group,result.time);}return result;
}
void SimulationView::updateTraining(){
    waitTracker_.update(lights_,simulation_.entities(),policyRegions_,simulation_.seconds(),queueConfig_);
    trainingRecorder_.waits(waitTracker_.takeCompleted());
    if(trainingRecorder_.active() && simulation_.seconds()+1e-9>=trainingNextSample_){trainingRecorder_.sample(trainingSnapshot());trainingNextSample_=simulation_.seconds()+1;}
}
void SimulationView::stopTraining(const std::string& outcome){
    policyHoldEnabled_=false;
    std::exception_ptr failure;
    try {
    if(outcome=="start_failed"){trainingRecorder_.close();trainingDataset_.cancel();}
    else if(trainingRecorder_.active()){
        auto final=trainingSnapshot();trainingRecorder_.transition(trainingPrevious_,trainingAction_,final,true,trainingForced_);
        waitTracker_.finish(simulation_.seconds(),outcome);trainingRecorder_.waits(waitTracker_.takeCompleted());trainingRecorder_.close();trainingStatus_="Saved training run: "+trainingRecorder_.directory();
    }
    if(trainingDataset_.active()){auto root=trainingDataset_.root();trainingDataset_.finish();trainingStatus_="Appended training run to "+root+"/transitions.jsonl";}
    }catch(...){failure=std::current_exception();trainingStatus_="Dataset append failed; the per-run recording is preserved. Close programs holding the CSV/JSON files and retry saving.";}
    if(holdTiming_.active()){holdTiming_.stop(lights_);simulation_.setTrafficLights(lights_);}
    if(randomDemand_.active()){for(auto& source:randomDemand_.sources())if(source.node>=0 && source.node<static_cast<int>(graph_.nodes.size()))graph_.nodes[source.node].rate=source.originalRate;randomDemand_.stop();}
    if(failure)std::rethrow_exception(failure);
}
void SimulationView::startTraining(){
    if(trainingDataset_.active())throw std::runtime_error("Finish saving the previous dataset append before starting another run");
    if(!validateRun())throw std::runtime_error("Assign all lane exits before collecting training data");
    auto savedRegions=policyRegions_;stopPolicy();policyRuntime_.shutdown();policyAlgorithm_.clear();seed_=static_cast<int>(demandConfig_.seed);reset();
    policyRegions_=defaultSignalRegions(lights_);for(auto& region:policyRegions_){auto saved=std::find_if(savedRegions.begin(),savedRegions.end(),[&](auto& r){return r.lightId==region.lightId;});if(saved!=savedRegions.end())region=*saved;}
    std::vector<DemandSource> sources;for(std::size_t node=0;node<graph_.nodes.size();++node)if(graph_.nodes[node].spawn)sources.push_back({static_cast<int>(node),graph_.nodes[node].rate});if(sources.empty())throw std::runtime_error("Mark at least one spawn node");
    auto root=std::filesystem::u8path(trainingDirectory_);if(root.is_relative())root=std::filesystem::path(TRAFFIC_PROJECT_ROOT)/root;
    // Validate controller settings before creating files; store original signals
    // in the snapshot so the same experiment can be restarted from its seed.
    auto check=lights_;HoldSignalController validate;validate.begin(check,policyRegions_,0,trainingMaxRed_,trainingTr_,trainingYellow_,trainingMinGreen_);
    SignalWaitTracker validateQueue;validateQueue.update(lights_,{},policyRegions_,0,queueConfig_);if(trainingRandomSpawns_){RandomDemand validateDemand;validateDemand.begin(sources,demandConfig_);}
    auto directory=std::filesystem::u8path(trainingDataset_.begin(root.u8string(),trainingAppend_,demandConfig_.seed));
    try{
        trainingRecorder_.open(directory.u8string(),demandConfig_.seed,queueConfig_,demandConfig_,trainingMaxRed_,trainingTr_,trainingYellow_,0);
        source_.savePng((directory/"map.png").u8string());graph_.save((directory/"network.nodes").u8string());simulation_.saveTrafficLights((directory/"map.png.signals").u8string());simulation_.saveTrafficLights((directory/"network.nodes.signals").u8string());
        std::ofstream layout(directory/"camera_groups.csv");layout<<"signal_id,group,camera_range_m,camera_width_m\n";for(auto& region:policyRegions_)layout<<region.lightId<<','<<region.group<<','<<queueConfig_.horizon<<','<<(region.width>0?region.width:length(std::find_if(lights_.begin(),lights_.end(),[&](auto& light){return light.id==region.lightId;})->b-std::find_if(lights_.begin(),lights_.end(),[&](auto& light){return light.id==region.lightId;})->a))<<'\n';
        std::ofstream environment(directory/"environment.json");environment<<std::setprecision(17)<<"{\"meters_per_pixel\":"<<source_.metersPerPixel<<",\"decision_interval\":"<<trainingDecisionInterval_<<",\"min_green\":"<<trainingMinGreen_<<",\"vehicle_limit\":"<<maximumVehicles_<<",\"random_spawns\":"<<(trainingRandomSpawns_?"true":"false")<<",\"random_actions\":"<<(trainingRandomActions_?"true":"false")<<",\"engine\":\""<<(engine_==SimulationEngine::Native?"Native":"SUMO")<<"\"}\n";
        std::ofstream profiles(directory/"vehicle_profiles.csv");profiles<<std::setprecision(17)<<"kind,length_mean,length_sd,width_mean,width_sd,speed_mean,speed_sd,accel_mean,accel_sd,min_gap,headway,deceleration,emergency_deceleration,lateral_gap,spawn_percent\n";auto mix=simulation_.spawnPercentages();for(int kind=0;kind<vehicleKindCount;++kind){const auto& p=profiles_[kind];profiles<<kind<<','<<p.length.mean<<','<<p.length.stdDev<<','<<p.width.mean<<','<<p.width.stdDev<<','<<p.motion.speedMean<<','<<p.motion.speedStdDev<<','<<p.motion.accelerationMean<<','<<p.motion.accelerationStdDev<<','<<p.minGap<<','<<p.tau<<','<<p.deceleration<<','<<p.emergencyDeceleration<<','<<p.lateralMinGap<<','<<mix[kind]<<'\n';}
        holdTiming_.begin(lights_,policyRegions_,0,trainingMaxRed_,trainingTr_,trainingYellow_,trainingMinGreen_);simulation_.setTrafficLights(lights_);
        if(trainingRandomSpawns_){randomDemand_.begin(sources,demandConfig_,0);trainingRecorder_.demand(randomDemand_.takeEvents());}
        trainingRandom_.seed(demandConfig_.seed^0xa6c124u);trainingAction_=-1;trainingNextDecision_=0;trainingNextSample_=1;trainingForced_=false;waitTracker_.reset();updateTraining();trainingPrevious_=trainingSnapshot();trainingRecorder_.sample(trainingPrevious_);running_=true;trainingStatus_="Recording to "+directory.u8string();
    }catch(...){stopTraining("start_failed");throw;}
}
void SimulationView::trainingPanel(){
    ImGui::SeparatorText("Training data");
    ImGui::TextWrapped("Design a map, assign its lane exits and lights, then collect two-group observations and transitions. Start resets traffic to the chosen seed.");
    bool recording=trainingRecorder_.active();
    ImGui::BeginDisabled(recording);
    ImGui::TextUnformatted("Output folder");ImGui::SetNextItemWidth(-1);ImGui::InputText("##training_directory",trainingDirectory_,sizeof(trainingDirectory_));ImGui::EndDisabled();
    ImGui::BeginDisabled(recording || trainingDataset_.active());ImGui::Checkbox("Append to existing dataset",&trainingAppend_);ImGui::EndDisabled();
    ImGui::TextWrapped("Finished runs are added to this folder's combined transitions.jsonl. Each run keeps its own CSVs and scenario snapshot. New dataset mode requires an empty folder.");
    std::array<bool,2> assigned{};for(const auto& region:policyRegions_)if(region.group>=0 && region.group<2)assigned[region.group]=true;
    bool ready=engineReady_ && lights_.size()>=2 && lights_.size()<=4 && assigned[0] && assigned[1] && std::all_of(lights_.begin(),lights_.end(),[](auto& light){return light.enabled;});
    if(!recording && trainingDataset_.active()){if(ImGui::Button("Retry saving dataset",ImVec2(-1,0))){try{stopTraining();}catch(const std::exception& error){trainingStatus_=error.what();}}}
    else if(!recording){ImGui::BeginDisabled(!ready);if(ImGui::Button("Start recording and run",ImVec2(-1,0))){try{startTraining();}catch(const std::exception& error){trainingStatus_=error.what();}}ImGui::EndDisabled();if(!ready)ImGui::TextWrapped("Enable 2-4 traffic lights and assign both groups before recording.");}
    else if(ImGui::Button("Stop and finish dataset",ImVec2(-1,0))){try{stopTraining();}catch(const std::exception& error){trainingStatus_=error.what();}}
    ImGui::TextWrapped("%s",trainingStatus_.c_str());
    auto input=[&](const char* label,double& value,double low,double high){float edit=static_cast<float>(value);ImGui::TextUnformatted(label);ImGui::SetNextItemWidth(-1);if(ImGui::InputFloat((std::string("##train_")+label).c_str(),&edit) && std::isfinite(edit))value=std::clamp(static_cast<double>(edit),low,high);};
    ImGui::BeginDisabled(recording || holdTiming_.active() || policyTiming_.active());
    if(ImGui::CollapsingHeader("Camera and queue categories",ImGuiTreeNodeFlags_DefaultOpen)){
        input("Camera range upstream (m)",queueConfig_.horizon,5,2000);input("Short queue: up to (m)",queueConfig_.shortLimit,1,2000);input("Moderate queue: up to (m)",queueConfig_.moderateLimit,2,2000);
        ImGui::TextWrapped("No tracked queue = none; longer than the moderate limit = long. Vehicles outside camera range are not observed.");
        if(ImGui::TreeNode("Signal group assignment")){for(auto& region:policyRegions_){ImGui::PushID(static_cast<int>(region.lightId));ImGui::Text("Signal ID %llu",static_cast<unsigned long long>(region.lightId));const char* groups[]{"Group 1","Group 2"};ImGui::SetNextItemWidth(-1);ImGui::Combo("##training_group",&region.group,groups,2);ImGui::PopID();}ImGui::TreePop();}
    }
    if(ImGui::CollapsingHeader("Hold / switch and final Tr")){
        input("Maximum red before forced switch (s)",trainingMaxRed_,5,3600);input("Tr: final all-red clearance (s)",trainingTr_,.1,120);input("Yellow before Tr (s)",trainingYellow_,.1,30);input("Minimum green (s)",trainingMinGreen_,0,120);input("Decision interval (s)",trainingDecisionInterval_,.1,60);
        ImGui::Checkbox("Random hold / switch exploration",&trainingRandomActions_);
        ImGui::TextWrapped("The current green can be held. A request or the maximum-red limit starts yellow, then both groups stay red for Tr before the waiting group gets green.");
    }
    if(ImGui::CollapsingHeader("Random spawn rates")){
        ImGui::Checkbox("Randomize every spawn node independently",&trainingRandomSpawns_);int seed=static_cast<int>(demandConfig_.seed);if(ImGui::InputInt("Seed",&seed))demandConfig_.seed=static_cast<std::uint32_t>(std::max(0,seed));
        input("Minimum rate (vehicles/s)",demandConfig_.minimum,.000001,100);input("Maximum rate (vehicles/s)",demandConfig_.maximum,.000001,100);input("Rate changes: minimum interval (s)",demandConfig_.changeMin,.1,3600);input("Rate changes: maximum interval (s)",demandConfig_.changeMax,.1,3600);input("Probability of a quiet period",demandConfig_.zeroProbability,0,1);
        ImGui::TextWrapped("Rates are log-uniform with optional zero-demand periods. Arrivals are Poisson. Equal seeds reproduce the same scenario.");
    }
    ImGui::EndDisabled();
    if(holdTiming_.active()){
        ImGui::BeginDisabled(trainingRandomActions_ || policyHoldEnabled_);
        auto action=[&](const char* label,int group){if(ImGui::Button(label,ImVec2(-1,0))){auto next=trainingSnapshot();if(next.time>trainingPrevious_.time)trainingRecorder_.transition(trainingPrevious_,trainingAction_,next,false,trainingForced_);trainingPrevious_=next;trainingAction_=group;trainingNextDecision_=simulation_.seconds()+trainingDecisionInterval_;trainingForced_=false;holdTiming_.request(group);}};
        action("Hold current allocation",-1);action("Request group 1 green",0);action("Request group 2 green",1);ImGui::EndDisabled();
    }
    auto snapshot=trainingSnapshot();ImGui::SeparatorText("Live camera observations");
    for(int group=0;group<2;++group){auto& q=snapshot.groups[group];ImGui::Text("Group %d: %s (%.1f m)",group+1,queueCategoryName(q.category),q.metres);ImGui::Text("%s for %.1f s | red %.1f s",q.phase==0?"RED":q.phase==1?"YELLOW":q.phase==2?"GREEN":"MIXED",q.phaseAge,q.redAge);if(q.countdown>=0)ImGui::Text("Final Tr remaining: %.1f s",q.countdown);ImGui::Text("Wait min / max: %.1f / %.1f s",q.wait.minimum,q.wait.maximum);ImGui::Text("Median / mean: %.1f / %.1f s",q.wait.median,q.wait.mean);ImGui::TextDisabled("%d vehicle wait samples%s",q.wait.count,q.cameraLimit?" | camera limit reached":"");}
    if(randomDemand_.active())for(auto& source:randomDemand_.sources())ImGui::Text("Spawn N%d: %.3f vehicles/s",source.node,source.rate);
}
}
