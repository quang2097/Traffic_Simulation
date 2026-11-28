#include "traffic/SimulationView.hpp"
#include "imgui.h"
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <filesystem>
#include <windows.h>
#include <commdlg.h>

namespace traffic {
namespace {
const char* families[]{"Auto","MAPPO","IPPO","QMIX","IDQN","MPLight","CoLight","PressLight","MA2C","DQN","DuelingDQN"};
constexpr int familyCount=sizeof(families)/sizeof(families[0]);
std::string browsePolicy(bool python){
    wchar_t path[32768]{};OPENFILENAMEW dialog{};dialog.lStructSize=sizeof(dialog);dialog.hwndOwner=GetActiveWindow();dialog.lpstrFile=path;dialog.nMaxFile=32768;
    dialog.lpstrFilter=python?L"Python interpreter\0python.exe\0Executables\0*.exe\0\0":L"Exported policies\0*.onnx;*.pt;*.pth;*.torchscript\0All files\0*.*\0\0";
    dialog.Flags=OFN_FILEMUSTEXIST|OFN_PATHMUSTEXIST|OFN_NOCHANGEDIR;
    if(!GetOpenFileNameW(&dialog))return {};
    int size=WideCharToMultiByte(CP_UTF8,0,path,-1,nullptr,0,nullptr,nullptr);std::string result(size,0);WideCharToMultiByte(CP_UTF8,0,path,-1,result.data(),size,nullptr,nullptr);result.pop_back();return result;
}
}
void SimulationView::stopPolicy(){
    if(holdTiming_.active() || trainingRecorder_.active())stopTraining("policy_change");
    if(policyTiming_.active()){policyTiming_.stop(lights_);simulation_.setTrafficLights(lights_);}
    ++policyEpoch_;
    policyHoldEnabled_=false;
}
void SimulationView::pollPolicy(){
    // Preserve region settings by stable signal ID, including after reordering.
    auto defaults=defaultSignalRegions(lights_);for(auto& region:defaults){auto old=std::find_if(policyRegions_.begin(),policyRegions_.end(),[&](const auto& r){return r.lightId==region.lightId;});if(old!=policyRegions_.end())region=*old;}policyRegions_=std::move(defaults);
    policyObservation_=observeSignals(lights_,simulation_.entities(),policyRegions_);
    PolicyReply reply;
    if(policyRuntime_.poll(reply)){
        if(reply.kind==PolicyReply::Ready){policyAlgorithm_=reply.message;policyStatus_="Loaded "+reply.message+" policy. Assign groups and enable control.";}
        else if(reply.kind==PolicyReply::SelectModel){policySelectModel_=true;policyStatus_=reply.message;}
        else if(reply.kind==PolicyReply::Error){stopPolicy();policyStatus_="Policy stopped: "+reply.message;}
        else if(reply.kind==PolicyReply::HoldAction && holdTiming_.active() && policyHoldEnabled_ && policyRequestEpoch_==policyEpoch_){
            int action=-1;if(reply.green[0]==1 && reply.green[1]==1)action=holdTiming_.greenGroup()<0?0:1-holdTiming_.greenGroup();else if(reply.green[0]==1)action=0;else if(reply.green[1]==1)action=1;trainingAction_=action;holdTiming_.request(action);policyStatus_="Hold/request action received; yellow and final Tr govern every switch.";
        }
        else if(reply.kind==PolicyReply::Action && policyTiming_.active() && policyRequestEpoch_==policyEpoch_){
            try{policyTiming_.queue(reply.green);policyStatus_="Policy timings received; apply at the next common cycle boundary.";}
            catch(const std::exception& error){stopPolicy();policyStatus_=error.what();}
        }
    }
    if((policyTiming_.active() || (holdTiming_.active() && policyHoldEnabled_)) && policyRuntime_.loaded() && !policyRuntime_.busy() && simulation_.seconds()+1e-9>=policyNextDecision_){
        policyRequestEpoch_=policyEpoch_;if(policyRuntime_.trainingSchema())policyRuntime_.requestTraining(trainingSnapshot(),policyObservation_.mask);else policyRuntime_.request(policyObservation_);policyNextDecision_=simulation_.seconds()+policyInterval_;
    }
}
void SimulationView::policyPanel(){
    ImGui::SeparatorText("RL signal controller");
    ImGui::TextWrapped("Two agents control synchronized signal groups. Traffic-v2 policies observe queues and cumulative waits and can hold phases; density-v1 policies retain green-duration control.");
    auto load=[&](){
        stopPolicy();policyAlgorithm_.clear();policyStatus_="Loading and validating policy...";
        policyRuntime_.load(policyPython_,std::string(TRAFFIC_PROJECT_ROOT)+"/scripts/rl_policy_worker.py",policyPath_,families[policyFamily_]);
    };
    auto changed=[&](){policyRuntime_.shutdown();policyAlgorithm_.clear();policyStatus_="Weights or model settings changed. Click Load weights.";};
    if(ImGui::CollapsingHeader("1. Load policy",ImGuiTreeNodeFlags_DefaultOpen)){
        ImGui::BeginDisabled(policyTiming_.active() || holdTiming_.active());
        ImGui::SetNextItemWidth(-1);if(ImGui::InputTextWithHint("##policy_path","ONNX or TorchScript weights",policyPath_,sizeof(policyPath_)))changed();
        if(ImGui::Button("Browse weights")){auto path=browsePolicy(false);if(!path.empty()){std::snprintf(policyPath_,sizeof(policyPath_),"%s",path.c_str());changed();}}
        ImGui::TextWrapped("Notebook export: select traffic_policy.pt and leave Model family on Auto. Its queue/wait inputs and three DQN actions are recognized automatically. ONNX exports also need their .data file when saved separately.");
        ImGui::TextWrapped("train.ipynb: Auto recognizes its dueling TorchScript export. For its ONNX export, choose DuelingDQN; Auto also runs it using the same joint DQN action contract. Normalization is already inside these exports.");
        ImGui::TextWrapped("train (1).ipynb is supported by DuelingDQN too, including its embedded safety mask. Match its Tr, yellow and minimum green in Training data; its supplied settings are 20 / 3 / 5 seconds.");
        ImGui::TextUnformatted("Model family");
        ImGui::SetNextItemWidth(-1);if(ImGui::Combo("##policy_family",&policyFamily_,families,familyCount))changed();
        ImGui::BeginDisabled(!policyPath_[0] || policyRuntime_.busy());
        if(ImGui::Button("Load weights"))load();
        ImGui::EndDisabled();
        if(ImGui::TreeNode("Python runtime")){
            ImGui::SetNextItemWidth(-1);if(ImGui::InputText("##policy_python",policyPython_,sizeof(policyPython_)))changed();
            if(ImGui::Button("Choose python.exe")){auto path=browsePolicy(true);if(!path.empty()){std::snprintf(policyPython_,sizeof(policyPython_),"%s",path.c_str());changed();}}
            ImGui::TextWrapped("ONNX: numpy + onnxruntime. TorchScript: numpy + torch. Export requirements are in docs/RL_CONTROLLER.md.");ImGui::TreePop();
        }
        ImGui::EndDisabled();
        if(policyRuntime_.busy())ImGui::TextUnformatted("Policy worker is busy...");
        if(!policyAlgorithm_.empty())ImGui::Text("Loaded model: %s",policyAlgorithm_.c_str());
        ImGui::TextWrapped("%s",policyStatus_.c_str());
    }
    if(policySelectModel_){ImGui::OpenPopup("Choose policy model");policySelectModel_=false;}
    if(ImGui::BeginPopupModal("Choose policy model",nullptr,ImGuiWindowFlags_AlwaysAutoResize)){
        ImGui::TextUnformatted("The weights do not identify their model family.");
        ImGui::Combo("Model",&policyFamily_,families,familyCount);
        ImGui::BeginDisabled(policyFamily_==0);if(ImGui::Button("Load selected model")){load();ImGui::CloseCurrentPopup();}ImGui::EndDisabled();
        ImGui::SameLine();if(ImGui::Button("Cancel"))ImGui::CloseCurrentPopup();ImGui::EndPopup();
    }
    if(ImGui::CollapsingHeader("2. Groups and density areas",ImGuiTreeNodeFlags_DefaultOpen)){
        if(policyRegions_.empty())ImGui::TextWrapped("Add traffic lights in the Traffic lights section to define observation areas.");
        ImGui::TextWrapped("Default: horizontal OUT directions in group 1; vertical in group 2. Adjust membership to match your junction.");
        ImGui::BeginDisabled(policyTiming_.active() || holdTiming_.active());
        for(auto& region:policyRegions_){ImGui::PushID(static_cast<int>(region.lightId));
            auto light=std::find_if(lights_.begin(),lights_.end(),[&](const auto& l){return l.id==region.lightId;});
            ImGui::Text("L%d (signal ID %llu)",static_cast<int>(light-lights_.begin()),static_cast<unsigned long long>(region.lightId));
            const char* groups[]{"Group 1","Group 2"};ImGui::SetNextItemWidth(-1);ImGui::Combo("##signal_group",&region.group,groups,2);
            float depth=static_cast<float>(region.depth),width=static_cast<float>(region.width);
            ImGui::SetNextItemWidth(-1);if(ImGui::SliderFloat("##region_depth",&depth,5,150,"Upstream: %.1f m") && std::isfinite(depth))region.depth=std::clamp(depth,5.f,150.f);
            ImGui::SetNextItemWidth(-1);if(ImGui::SliderFloat("##region_width",&width,0,100,"Width: %.1f m (0 = stop line)") && std::isfinite(width))region.width=std::clamp(width,0.f,100.f);
            auto found=std::find_if(policyObservation_.regions.begin(),policyObservation_.regions.end(),[&](const auto& r){return r.lightId==region.lightId;});
            if(found!=policyObservation_.regions.end())ImGui::Text("%.1f%% packed | %d vehicles",100*found->density,found->vehicles);else ImGui::TextDisabled("Disabled signal");
            ImGui::PopID();
        }
        ImGui::EndDisabled();ImGui::Checkbox("Show observation areas on map",&policyShowRegions_);
        if(ImGui::TreeNode("How density is measured")){ImGui::TextWrapped("Density is occupied vehicle footprint area / observation rectangle area. Only OUT traffic is counted; each group's density is weighted by its observation areas.");ImGui::TreePop();}
    }
    ImGui::SeparatorText("Agent observations");
    for(int group=0;group<2;++group){ImGui::Text("Group %d: %.2f%%",group+1,policyObservation_.density[group]*100);ImGui::ProgressBar(static_cast<float>(policyObservation_.density[group]),ImVec2(-1,0));}
    ImGui::BeginDisabled(policyTiming_.active());
    float interval=static_cast<float>(policyInterval_),yellow=static_cast<float>(policyYellow_),clearance=static_cast<float>(policyClearance_);
    ImGui::SetNextItemWidth(-1);if(ImGui::SliderFloat("##policy_interval",&interval,1,30,"Decision interval: %.1f s") && std::isfinite(interval))policyInterval_=std::clamp(interval,1.f,30.f);
    ImGui::SetNextItemWidth(-1);if(ImGui::SliderFloat("##policy_yellow",&yellow,1,10,"Yellow: %.1f s") && std::isfinite(yellow))policyYellow_=std::clamp(yellow,1.f,10.f);
    ImGui::SetNextItemWidth(-1);if(ImGui::SliderFloat("##policy_clearance",&clearance,.5f,10,"All-red clearance: %.1f s") && std::isfinite(clearance))policyClearance_=std::clamp(clearance,.5f,10.f);
    ImGui::EndDisabled();
    bool enabled=std::all_of(lights_.begin(),lights_.end(),[](const auto& light){return light.enabled;});
    bool ready=policyRuntime_.loaded() && !policyRuntime_.busy() && !holdTiming_.active() && lights_.size()<=4 && enabled && policyObservation_.mask[0]>0 && policyObservation_.mask[1]>0 && engineReady_;
    if(!policyTiming_.active() && !policyHoldEnabled_){
        ImGui::BeginDisabled(!ready);if(ImGui::Button("Enable policy control",ImVec2(-1,0))){try{if(policyRuntime_.trainingSchema()){holdTiming_.begin(lights_,policyRegions_,simulation_.seconds(),trainingMaxRed_,trainingTr_,trainingYellow_,trainingMinGreen_);policyHoldEnabled_=true;updateTraining();}else policyTiming_.begin(lights_,policyRegions_,simulation_.seconds(),policyYellow_,policyClearance_);simulation_.setTrafficLights(lights_);++policyEpoch_;policyNextDecision_=simulation_.seconds();policyStatus_="Policy enabled. Press Play to advance the simulation.";}catch(const std::exception& error){stopPolicy();policyStatus_=error.what();}}ImGui::EndDisabled();
        if(lights_.size()>4)ImGui::TextWrapped("Policy control supports at most four physical lights across two groups.");
        else if(!enabled || !policyObservation_.mask[0] || !policyObservation_.mask[1])ImGui::TextWrapped("Enable every light and assign at least one to each group.");
    }else{
        if(ImGui::Button("Stop policy / restore manual timings",ImVec2(-1,0))){stopPolicy();policyStatus_="Manual timing restored.";}
        if(policyTiming_.active()){ImGui::Text("Next green: %.1f / %.1f s",policyTiming_.pending()[0],policyTiming_.pending()[1]);ImGui::Text("Next cycle in %.1f s",std::max(0.,policyTiming_.nextCycle()-simulation_.seconds()));}
        else ImGui::TextWrapped("Hold/request control active. Camera, maximum red, yellow and Tr settings are in Training data.");
    }
}
}
