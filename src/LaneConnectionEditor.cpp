#include "traffic/SimulationView.hpp"
#include "imgui.h"
#include <algorithm>
#include <cstdio>
#include <map>

namespace traffic {
bool SimulationView::validateRun(){
    try{validationIssues_=graph_.missingLaneExits();}catch(const std::exception& e){validationIssues_={e.what()};}
    laneExitsReady_=validationIssues_.empty();if(laneExitsReady_)return true;
    running_=false;stepBatch_.cancel();validationNotice_=true;status_="Simulation refused: missing manual lane exits.";return false;
}
void SimulationView::loadLaneDemo(){
    try {
        std::string prefix=std::string(TRAFFIC_PROJECT_ROOT)+"/examples/seven-lane-routing";
        auto map=Map::fromPng(prefix+".png",1);auto graph=NodeGraph::load(prefix+".nodes");
        cancelEditorAction();source_=std::move(map);graph_=std::move(graph);lights_.clear();pathNodes_.clear();selectedNode_=1;turnRoad_=0;turnDirection_=0;turnLane_=1;highlightedExitNodes_.clear();
        metersPerPixel_=1;mapZoom_=1;mapPan_={};workspace_=1;running_=false;
        std::snprintf(mapPath_,sizeof(mapPath_),"%s.png",prefix.c_str());std::snprintf(laneFile_,sizeof(laneFile_),"%s.nodes",prefix.c_str());
        refreshMap(true);status_="Seven-lane example loaded. Node 1 contains its lane destination rules.";
    }catch(const std::exception& e){status_=e.what();}
}
namespace {
std::vector<std::pair<int,int>> arriving(const NodeGraph& graph,int node){
    std::vector<std::pair<int,int>> result;
    for(std::size_t r=0;r<graph.connections.size();++r){auto& road=graph.connections[r];if(graph.junctionAt(road.from)==graph.junctionAt(road.to))continue;if(road.to==node)result.push_back({static_cast<int>(r),0});if(road.roadType==2 && road.from==node)result.push_back({static_cast<int>(r),1});}
    return result;
}
std::string numbers(const std::vector<int>& values,int offset=0){std::string out;for(int v:values){if(!out.empty())out+=", ";out+=std::to_string(v+offset);}return out;}
std::vector<std::pair<int,int>> departures(const NodeGraph& graph,int node){
    std::vector<std::pair<int,int>> result;for(std::size_t r=0;r<graph.connections.size();++r){auto& road=graph.connections[r];if(graph.junctionAt(road.from)==graph.junctionAt(road.to))continue;if(road.from==node)result.push_back({static_cast<int>(r),0});if(road.roadType==2 && road.to==node)result.push_back({static_cast<int>(r),1});}return result;

}
}
void SimulationView::toggleLaneExit(int road,int direction,int lane){
    if(road<0 || road>=static_cast<int>(graph_.connections.size()) || lane<1 || lane>graph_.directionalLaneCount(road,direction))return;
    auto it=std::find_if(turnExits_.begin(),turnExits_.end(),[&](const auto& draft){return draft.road==road && draft.direction==direction;});
    if(it==turnExits_.end()){
        ExitDraft draft;draft.road=road;draft.direction=direction;draft.node=direction?graph_.connections[road].to:graph_.connections[road].from;
        turnExits_.push_back(draft);it=std::prev(turnExits_.end());
    }
    auto lanes=parseNumberList(it->lanes.data(),1,graph_.directionalLaneCount(road,direction));
    auto found=std::find(lanes.begin(),lanes.end(),lane);
    if(found==lanes.end()){lanes.push_back(lane);it->priorities.emplace(lane,1);}else lanes.erase(found);
    std::sort(lanes.begin(),lanes.end());auto text=numbers(lanes);
    std::snprintf(it->lanes.data(),it->lanes.size(),"%s",text.c_str());
    highlightedExitNodes_.clear();for(const auto& draft:turnExits_)if(draft.lanes[0])highlightedExitNodes_.push_back(draft.node);
    turnDirty_=true;turnError_.clear();exitRoadChoice_=road;exitDirectionChoice_=direction;
}
bool SimulationView::applyLaneExitDraft(){
    try{
        auto proposed=graph_;LaneTurnRule rule;rule.direction=turnDirection_;rule.lane=turnLane_-1;
        for(auto& draft:turnExits_){auto list=parseNumberList(draft.lanes.data(),1,graph_.directionalLaneCount(draft.road,draft.direction));if(list.empty())continue;
            std::vector<int> priorities;for(int& lane:list){priorities.push_back(draft.priorities.count(lane)?draft.priorities[lane]:1);--lane;}
            rule.exits.push_back({draft.node,list,priorities,draft.road,draft.direction});}
        proposed.setLaneTurnRule(turnRoad_,std::move(rule));
        if(!applyGraph(proposed))return false;
        turnDirty_=false;turnDraftNode_=-1;status_="Lane connections applied; traffic reset.";return true;
    }catch(const std::exception& e){status_=turnError_=e.what();return false;}
}
bool SimulationView::laneConnectionsPopup(){
    ImGui::TextWrapped("Choose one incoming lane. Click outgoing lanes on the map, or use the numbered buttons below. Only this lane's connections are shown.");
    auto approaches=arriving(graph_,selectedNode_);if(approaches.empty()){ImGui::TextUnformatted("No incoming road at this node.");return false;}
    auto current=std::find(approaches.begin(),approaches.end(),std::make_pair(turnRoad_,turnDirection_));int selected=current==approaches.end()?0:static_cast<int>(current-approaches.begin());
    auto label=[&](std::pair<int,int> approach){auto& road=graph_.connections[approach.first];int source=approach.second?road.to:road.from;return "N"+std::to_string(source)+" -> N"+std::to_string(selectedNode_)+" (road "+std::to_string(approach.first)+")";};
    ImGui::SeparatorText("1. Incoming lane (gold)");
    ImGui::BeginDisabled(turnDirty_);
    ImGui::SetNextItemWidth(-1);
    if(ImGui::BeginCombo("##Incoming road",label(approaches[selected]).c_str())){for(std::size_t i=0;i<approaches.size();++i)if(ImGui::Selectable(label(approaches[i]).c_str(),i==static_cast<std::size_t>(selected)))selected=static_cast<int>(i);ImGui::EndCombo();}
    turnRoad_=approaches[selected].first;turnDirection_=approaches[selected].second;
    int count=graph_.directionalLaneCount(turnRoad_,turnDirection_);turnLane_=std::clamp(turnLane_,1,count);
    for(int lane=1;lane<=count;++lane){ImGui::PushID(lane);if((lane-1)%7)ImGui::SameLine();if(ImGui::Selectable(std::to_string(lane).c_str(),turnLane_==lane,0,ImVec2(32,26)))turnLane_=lane;ImGui::PopID();}
    ImGui::EndDisabled();
    ImGui::TextWrapped("Lane %d of %d. Lane 1 is leftmost facing travel.",turnLane_,count);
    if(turnDirty_)ImGui::TextWrapped("Unsaved changes: apply or discard before switching the incoming lane.");
    auto load=[&](){
        turnExits_.clear();highlightedExitNodes_.clear();turnError_.clear();turnDirty_=false;
        auto rule=graph_.laneTurnRule(turnRoad_,turnDirection_,turnLane_-1);
        if(rule)for(const auto& exit:rule->exits)for(auto target:graph_.targetRoads(turnRoad_,turnDirection_,exit)){
            const auto& road=graph_.connections[target.first];ExitDraft draft;draft.road=target.first;draft.direction=target.second;draft.node=target.second?road.to:road.from;
            for(std::size_t i=0;i<exit.lanes.size();++i)draft.priorities[exit.lanes[i]+1]=i<exit.priorities.size()?exit.priorities[i]:1;
            auto text=numbers(exit.lanes,1);std::snprintf(draft.lanes.data(),draft.lanes.size(),"%s",text.c_str());turnExits_.push_back(draft);highlightedExitNodes_.push_back(draft.node);
        }
        turnDraftNode_=selectedNode_;turnDraftRoad_=turnRoad_;turnDraftDirection_=turnDirection_;turnDraftLane_=turnLane_;
        if(!turnExits_.empty()){exitRoadChoice_=turnExits_.front().road;exitDirectionChoice_=turnExits_.front().direction;}
        else {auto local=departures(graph_,selectedNode_);if(!local.empty()){exitRoadChoice_=local.front().first;exitDirectionChoice_=local.front().second;}}
    };
    if(turnDraftNode_!=selectedNode_ || turnDraftRoad_!=turnRoad_ || turnDraftDirection_!=turnDirection_ || turnDraftLane_!=turnLane_)load();
    ImGui::SeparatorText("2. Outgoing lanes (teal)");
    std::vector<std::pair<int,int>> roads;for(std::size_t node=0;node<graph_.nodes.size();++node){auto next=departures(graph_,static_cast<int>(node));roads.insert(roads.end(),next.begin(),next.end());}
    auto roadLabel=[&](std::pair<int,int> target){const auto& road=graph_.connections[target.first];return "N"+std::to_string(target.second?road.to:road.from)+" -> N"+std::to_string(target.second?road.from:road.to)+" (road "+std::to_string(target.first)+")";};
    if(!roads.empty()){
        auto chosen=std::find(roads.begin(),roads.end(),std::make_pair(exitRoadChoice_,exitDirectionChoice_));if(chosen==roads.end()){exitRoadChoice_=roads.front().first;exitDirectionChoice_=roads.front().second;}
        ImGui::SetNextItemWidth(-1);
        if(ImGui::BeginCombo("##Outgoing road",roadLabel({exitRoadChoice_,exitDirectionChoice_}).c_str())){for(auto target:roads)if(ImGui::Selectable(roadLabel(target).c_str(),target==std::make_pair(exitRoadChoice_,exitDirectionChoice_))){exitRoadChoice_=target.first;exitDirectionChoice_=target.second;}ImGui::EndCombo();}
        int targetCount=graph_.directionalLaneCount(exitRoadChoice_,exitDirectionChoice_);
        auto draft=std::find_if(turnExits_.begin(),turnExits_.end(),[&](const auto& e){return e.road==exitRoadChoice_ && e.direction==exitDirectionChoice_;});
        auto picked=draft==turnExits_.end()?std::vector<int>{}:parseNumberList(draft->lanes.data(),1,targetCount);
        ImGui::PushID("target lanes");
        for(int lane=1;lane<=targetCount;++lane){ImGui::PushID(lane);if((lane-1)%7)ImGui::SameLine();if(ImGui::Selectable(std::to_string(lane).c_str(),std::find(picked.begin(),picked.end(),lane)!=picked.end(),0,ImVec2(32,26)))toggleLaneExit(exitRoadChoice_,exitDirectionChoice_,lane);ImGui::PopID();}
        ImGui::PopID();
    }
    ImGui::TextWrapped("Click a number to allow or remove that outgoing lane. You can choose lanes on several roads.");
    bool any=false;
    for(auto& draft:turnExits_)if(draft.lanes[0]){any=true;ImGui::BulletText("To N%d -> N%d: lane(s) %s",draft.node,draft.direction?graph_.connections[draft.road].from:graph_.connections[draft.road].to,draft.lanes.data());}
    if(!any)ImGui::TextWrapped("No exits chosen for lane %d. A non-destination lane needs at least one exit before Play.",turnLane_);
    if(ImGui::TreeNode("Movement priorities (optional)")){
        for(auto& draft:turnExits_)for(int lane:parseNumberList(draft.lanes.data(),1,graph_.directionalLaneCount(draft.road,draft.direction))){
            ImGui::PushID(draft.road*2+draft.direction);ImGui::PushID(lane);auto [it,inserted]=draft.priorities.emplace(lane,1);
            ImGui::Text("N%d, lane %d (1 = highest)",draft.node,lane);ImGui::SetNextItemWidth(-1);
            if(ImGui::InputInt("##priority",&it->second)){it->second=std::clamp(it->second,1,1000000);turnDirty_=true;}
            ImGui::PopID();ImGui::PopID();}
        ImGui::TreePop();
    }
    ImGui::Separator();
    bool applied=ImGui::Button("Apply lane exits") && applyLaneExitDraft();
    ImGui::SameLine();if(ImGui::Button("Discard changes"))load();
    if(!turnError_.empty()){ImGui::PushStyleColor(ImGuiCol_Text,ImVec4(1,.45f,.35f,1));ImGui::TextWrapped("%s",turnError_.c_str());ImGui::PopStyleColor();}
    return applied;
}
void SimulationView::roadSignalPopup(){
    ImGui::SeparatorText("Road-mounted light");
    auto approaches=arriving(graph_,selectedNode_);if(approaches.empty()){ImGui::TextUnformatted("Connect an incoming road first.");return;}
    int selected=0;for(std::size_t i=0;i<approaches.size();++i)if(approaches[i].first==signalRoad_)selected=static_cast<int>(i);
    auto label=[&](int i){auto pair=approaches[i];auto& road=graph_.connections[pair.first];return "Node "+std::to_string(pair.second?road.to:road.from)+" -> "+std::to_string(selectedNode_);};
    ImGui::TextUnformatted("OUT approach (obeys this light)");ImGui::SetNextItemWidth(-1);
    if(ImGui::BeginCombo("##Controlled approach",label(selected).c_str())){for(std::size_t i=0;i<approaches.size();++i)if(ImGui::Selectable(label(static_cast<int>(i)).c_str(),i==static_cast<std::size_t>(selected)))selected=static_cast<int>(i);ImGui::EndCombo();}
    signalRoad_=approaches[selected].first;
    if(ImGui::Button("Create perpendicular light",ImVec2(-1,0))){try{auto line=roadSignalAtNode(graph_,signalRoad_,selectedNode_);addLight(line.a,line.b,false,{},{},line.outDirection,selectedNode_);selectedLight_=static_cast<int>(lights_.size())-1;status_="Directional road signal added.";}catch(const std::exception& e){status_=e.what();}}
    std::vector<int> nodeLights;for(std::size_t i=0;i<lights_.size();++i)if(lights_[i].anchorNode==selectedNode_)nodeLights.push_back(static_cast<int>(i));
    if(!nodeLights.empty()){
        if(std::find(nodeLights.begin(),nodeLights.end(),selectedLight_)==nodeLights.end())selectedLight_=nodeLights.front();
        auto selectedLabel="L"+std::to_string(selectedLight_);
        ImGui::Spacing();ImGui::SeparatorText("Selected light");ImGui::SetNextItemWidth(-1);
        if(ImGui::BeginCombo("##Node light",selectedLabel.c_str())){for(int i:nodeLights){auto text="L"+std::to_string(i);if(ImGui::Selectable(text.c_str(),i==selectedLight_))selectedLight_=i;}ImGui::EndCombo();}
        auto direction=lights_[selectedLight_].controlledDirection();
        const char* toward=std::abs(direction.x)>std::abs(direction.y)?(direction.x>0?"right":"left"):(direction.y>0?"down":"up");
        ImGui::Text("OUT toward %s: obeys signal",toward);ImGui::TextDisabled("IN travels the reverse way: passes red");
        if(ImGui::Button("Reverse controlled direction",ImVec2(-1,0))){lights_[selectedLight_].outDirection=lights_[selectedLight_].controlledDirection()*-1;simulation_.setTrafficLights(lights_);}
    }
    ImGui::BeginDisabled(nodeLights.empty());
    if(ImGui::Button("Draw allowed exit",ImVec2(-1,0)) && !lights_.empty()){
        int matching=selectedLight_>=0 && selectedLight_<static_cast<int>(lights_.size()) && lights_[selectedLight_].anchorNode==selectedNode_?selectedLight_:-1;
        if(matching>=0){selectedLight_=matching;editorMode_=7;linkedStage_=1;linkedLight_=matching;pendingStopA_=lights_[matching].a;pendingStopB_=lights_[matching].b;ImGui::CloseCurrentPopup();}
        else status_="Add this node's signal first.";
    }
    if(ImGui::Button("Edit timings and detectors",ImVec2(-1,0))){workspace_=2;ImGui::CloseCurrentPopup();}
    ImGui::EndDisabled();
}
}
