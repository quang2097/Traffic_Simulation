#include "traffic/SimulationView.hpp"
#include "traffic/Demo.hpp"
#include "traffic/SpawnClock.hpp"
#include "imgui.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <chrono>
#include <filesystem>
#include <windows.h>
#include <commdlg.h>

namespace traffic {
namespace {
std::string chooseFile(bool save,const char* filter,const char* extension,const char* initial) {
    char path[4096]{}; auto native=std::filesystem::absolute(initial).make_preferred().string();
    std::snprintf(path,sizeof(path),"%s",native.c_str());
    OPENFILENAMEA dialog{}; dialog.lStructSize=sizeof(dialog);
    dialog.hwndOwner=GetActiveWindow(); dialog.lpstrFile=path; dialog.nMaxFile=sizeof(path);
    dialog.lpstrFilter=filter; dialog.lpstrDefExt=extension;
    dialog.Flags=OFN_NOCHANGEDIR|OFN_PATHMUSTEXIST|(save?OFN_OVERWRITEPROMPT:OFN_FILEMUSTEXIST);
    if(save?GetSaveFileNameA(&dialog):GetOpenFileNameA(&dialog)) return path;
    if(CommDlgExtendedError()) throw std::runtime_error("The file dialog could not open. Check the file path in Files.");
    return "";
}
bool typedFloat(const char* label,float* value,float low,float high,const char* format="%.3f") {
    if(label[0]!='#' || label[1]!='#')ImGui::TextUnformatted(label);
    ImGui::SetNextItemWidth(-1);
    if(!ImGui::InputFloat((std::string("##")+label).c_str(),value,0,0,format,ImGuiInputTextFlags_EnterReturnsTrue)) return false;
    *value=std::isfinite(*value)?std::clamp(*value,low,high):low;
    return true;
}
bool typedDouble(const char* label,double* value,double low,double high,const char* format="%.6f") {
    if(label[0]!='#' || label[1]!='#')ImGui::TextUnformatted(label);
    ImGui::SetNextItemWidth(-1);
    if(!ImGui::InputDouble((std::string("##")+label).c_str(),value,0,0,format,ImGuiInputTextFlags_EnterReturnsTrue)) return false;
    *value=std::isfinite(*value)?std::clamp(*value,low,high):low;
    return true;
}
std::vector<double> shapeWidths(const RoadConnection& road) {
    std::size_t count=road.shape.empty()?2:road.shape.size();
    if(road.shapeWidths.size()==count) return road.shapeWidths;
    std::vector<double> result(count,road.width);
    for(std::size_t i=0;i<count;++i) result[i]=road.width+(road.endWidth>0?road.endWidth-road.width:0)*i/std::max<std::size_t>(1,count-1);
    return result;
}
void distributionPlot(const char* label,float mean,float sigma,float maximum) {
    float values[100];
    for (int i=0;i<100;++i) {
        float x=maximum*i/99;
        values[i]=sigma>0 ? std::exp(-0.5f*(x-mean)*(x-mean)/(sigma*sigma))
                         : (std::abs(x-mean)<maximum/99 ? 1.f:0.f);
    }
    ImGui::PlotLines(label,values,100,0,nullptr,0,1,ImVec2(-1,48));
    ImGui::TextDisabled("0                          %.1f",maximum);
}
}
SimulationView::SimulationView(const char* initialMap,SimulationEngine engine) : engine_(engine),source_(makeDemoMap()),simulation_(source_,42) {
    workspace_=engine_==SimulationEngine::Native?0:5;
    profiles_=vietnameseCityProfiles(); urbanTraffic_.enabled=false;
    routing_.rtsStyle=false;routing_.spreadAtSignals=false;routing_.refreshMovingRoutes=false;
    trafficRecoverySettings_.enabled=false;stuckVehicleSettings_.enabled=false;stuckVehicleSettings_.junctionEnabled=false;flow_.sublanes=false;
    tugSettings_.enabled=false;
    turnNegotiation_.enabled=false;
    routing_.localInterval=.35; routing_.movingRefreshInterval=1.2;
    trafficRecoverySettings_.stopSeconds=.6;
    if(!initialMap) { source_=makeJunctionDemoMap(); graph_=makeJunctionDemoGraph(); lanes_=graph_.lanes(); metersPerPixel_=1; }
    if (initialMap) {
        std::snprintf(mapPath_,sizeof(mapPath_),"%s",initialMap);
        try { source_=Map::fromPng(mapPath_,metersPerPixel_); lights_.clear(); }
        catch(const std::exception& e) { status_=e.what(); }
    }
    if(initialMap) graph_=NodeGraph::fromLanes(lanes_);
    refreshMap(false,initialMap!=nullptr);
    auto shares=simulation_.spawnPercentages();
    for(int i=0;i<vehicleKindCount;++i) spawnMix_[i]=static_cast<float>(shares[i]);
}
void SimulationView::cancelPendingLine() { linePending_=false; linkedStage_=0; linkedLight_=-1; if(editorMode_==8) editorMode_=0; }
bool SimulationView::editorActionActive() const {
    return connectionFrom_>=0 || drawingRoad_ || splittingRoad_ || placingRoadSignal_ || editorMode_!=0 || placeLight_ || linePending_ || linkedStage_>0;
}
const char* SimulationView::editorActionLabel() const {
    if(connectionFrom_>=0)return severing_?"Sever connection":"Connect nodes";
    if(drawingRoad_)return "Sketch road";
    if(placingRoadSignal_)return "Place road signal node";
    if(splittingRoad_)return "Insert road node";
    if(editorMode_==3)return "Place nodes";
    if(editorMode_==8)return "Queue detector";
    if(editorMode_==7)return linkedStage_==0?"Traffic-light line":"Allowed exit";
    if(placeLight_ || editorMode_==5)return "Traffic-light placement";
    if(editorMode_==1)return "Paint road";
    if(editorMode_==2)return "Erase road";
    if(editorMode_==6)return "Lane divider";
    return "Draw line";
}
void SimulationView::cancelEditorAction() {
    connectionFrom_=-1;severing_=false;drawingRoad_=splittingRoad_=false;
    roadDraft_.clear();roadDraftWidths_.clear();placingRoadSignal_=false;strokeActive_=false;placeLight_=false;
    cancelPendingLine();editorMode_=0;
}
void SimulationView::selectEngine(SimulationEngine engine) {
    if(engine_==engine && engineReady_)return;
    running_=false;engine_=engine;workspace_=engine_==SimulationEngine::Native?0:5;
    cancelEditorAction();reset();
}
void SimulationView::refreshMap(bool pauseTraffic,bool importSignals) {
    for(auto& tile:source_.tiles) if(tile==Tile::Destination) tile=Tile::Road;
    source_.rebuildFeatures();
    if(importSignals) {
    std::vector<TrafficLight> next;
    for(const auto& feature:source_.features()) if(feature.type==Tile::Signal && length(feature.b-feature.a)>=.1) {
        TrafficLight light{feature.a,feature.b,10,2,12,0,true};
        for(const auto& previous:lights_) {
            bool same=length(previous.a-feature.a)+length(previous.b-feature.b)<source_.metersPerPixel*1.5;
            bool reversed=length(previous.b-feature.a)+length(previous.a-feature.b)<source_.metersPerPixel*1.5;
            if(same || reversed) { light=previous; light.a=feature.a; light.b=feature.b; break; }
        }
        next.push_back(light);
    }
    lights_=std::move(next);
    }
    mapDirty_=false;
    if(pauseTraffic) running_=false;
    reset();
    if(importSignals && std::filesystem::exists(std::string(mapPath_)+".signals")) {
        try { simulation_.loadTrafficLights(std::string(mapPath_)+".signals"); lights_=simulation_.trafficLights(); syncSignalMask(); }
        catch(const std::exception& e) { running_=false; status_=e.what(); }
    }
}
void SimulationView::syncSignalMask() {
    for(auto& tile:source_.tiles) if(tile==Tile::Signal) tile=Tile::Road;
    for(const auto& light:lights_)
        source_.paintStroke(light.a,light.b,Tile::Signal,source_.metersPerPixel*.6);
    source_.rebuildFeatures();
}
void SimulationView::addLight(Vec2 a,Vec2 b,bool permissive,Vec2 allowedA,Vec2 allowedB,Vec2 outDirection,int anchorNode) {
    auto next=lights_;
    next.push_back({a,b,10,2,12,0,true});
    next.back().permissive=permissive; next.back().allowedA=allowedA; next.back().allowedB=allowedB;next.back().outDirection=outDirection;next.back().anchorNode=anchorNode;
    simulation_.setTrafficLights(next);
    lights_=simulation_.trafficLights(); selectedLight_=static_cast<int>(lights_.size())-1;
    syncSignalMask();
    status_="Light added. Adjust its timings live in Traffic lights; click again to add another.";
}
void SimulationView::reset() {
    sumo_.reset();
    engineReady_=false;turnDraftNode_=-1;laneExitsReady_=false;
    for(auto& tile:source_.tiles) if(tile==Tile::Destination) tile=Tile::Road;
    source_.rebuildFeatures();
    laneOriginNodes_.clear();
    if(std::any_of(graph_.connections.begin(),graph_.connections.end(),[](const auto& e){return e.roadType!=0 || e.laneCount!=1 || e.reverseLaneCount>0 || !e.changeSections.empty() || !e.laneTurns.empty();}) || std::any_of(graph_.nodes.begin(),graph_.nodes.end(),[](const auto& n){return n.openRadius>0 || n.allowUTurn || n.internalRadius>0 || n.roadStation;})) {
        auto compiled=compileRoads(graph_); lanes_=std::move(compiled.lanes); laneOriginNodes_=std::move(compiled.originNodes);
    } else { lanes_=graph_.lanes(); for(const auto& edge:graph_.connections) laneOriginNodes_.push_back(edge.from); }
    nodeSpawnClocks_.assign(graph_.nodes.size(),0); spawnNodeRandom_.seed(static_cast<unsigned>(seed_));
    simulation_=Simulation(source_,static_cast<std::uint32_t>(seed_));
    simulation_.setRoutingSettings(routing_);
    simulation_.setTrafficFlowSettings(flow_);
    simulation_.setStuckVehicleSettings(stuckVehicleSettings_);
    simulation_.setGridlockEscapeSettings(gridlockEscape_);
    simulation_.setTrafficRecoverySettings(trafficRecoverySettings_);
    simulation_.setUrbanTrafficSettings(urbanTraffic_);
    simulation_.setTugSettings(tugSettings_);
    simulation_.setTurnNegotiationSettings(turnNegotiation_);
    simulation_.setCollisionWorkers(static_cast<unsigned>(collisionWorkers_));
    for(int i=0;i<vehicleKindCount;++i) simulation_.setProfile(static_cast<VehicleKind>(i),profiles_[i]);
    simulation_.setTrafficLights(lights_);
    lights_=simulation_.trafficLights();
    try {
        simulation_.setLanes(lanes_);
        if(engine_==SimulationEngine::Sumo) {
            sumo_=std::make_unique<SumoBackend>(simulation_,graph_,static_cast<unsigned>(seed_));
            lanes_=simulation_.lanes();
        }
        engineReady_=true;
        status_=engine_==SimulationEngine::Native?"Native simulation ready.":"SUMO simulation ready.";
    } catch(const std::exception& e) { running_=false;status_=e.what(); }
    selected_=spawned_=0; elapsed_=accumulator_=0;
    stepBatch_.cancel();
    planningTimeToExclude_=0;
    cancelEditorAction();
    laneExitsReady_=graph_.missingLaneExits().empty();if(running_ && !laneExitsReady_)validateRun();
}
void SimulationView::advanceTick(double seconds) {
    if(!engineReady_) { running_=false;stepBatch_.cancel();return; }
    try {
    double planningBefore=simulation_.totalSignalPlanningSeconds()+simulation_.totalTrafficRecoverySeconds();
    if(!laneExitsReady_){validateRun();return;}
    if(automatic_) {
        nodeSpawnClocks_.resize(graph_.nodes.size(),0);
        for(std::size_t n=0;n<graph_.nodes.size();++n) {
            const auto& node=graph_.nodes[n];
            if(!node.spawn || node.rate<=0) { nodeSpawnClocks_[n]=0; continue; }
            if(!consumeSpawnClock(nodeSpawnClocks_[n],seconds,node.rate)) continue;
            std::vector<int> outgoing;
            for(std::size_t e=0;e<laneOriginNodes_.size();++e) if(laneOriginNodes_[e]==static_cast<int>(n)) outgoing.push_back(static_cast<int>(e));
            if(!outgoing.empty() && simulation_.entities().size()+(sumo_?sumo_->pending():0)<static_cast<std::size_t>(maximumVehicles_)) {
                int edge=outgoing[std::uniform_int_distribution<std::size_t>(0,outgoing.size()-1)(spawnNodeRandom_)];
                if(simulation_.spawnRandomOnLane(edge)) ++spawned_;
            }
        }
    }
    simulation_.tick(seconds); elapsed_=simulation_.seconds();
    planningTimeToExclude_+=simulation_.totalSignalPlanningSeconds()+simulation_.totalTrafficRecoverySeconds()-planningBefore;
    } catch(const std::exception& e) {running_=false;accumulator_=0;stepBatch_.cancel();engineReady_=false;status_=std::string(engine_==SimulationEngine::Sumo?"SUMO stopped: ":"Native stopped: ")+e.what();}
}
void SimulationView::frame(double realSeconds) {
    realSeconds=std::max(0.0,realSeconds-planningTimeToExclude_); planningTimeToExclude_=0;
    auto* viewport=ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::Begin("Traffic Simulation",nullptr,ImGuiWindowFlags_NoDecoration|ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoSavedSettings|ImGuiWindowFlags_MenuBar);
    menuBar();
    if(validationNotice_){ImGui::OpenPopup("Lane exits required");validationNotice_=false;}
    ImGui::SetNextWindowSize(ImVec2(520,370),ImGuiCond_Appearing);
    if(ImGui::BeginPopupModal("Lane exits required",nullptr,ImGuiWindowFlags_NoResize)) {
        ImGui::TextWrapped("Simulation refused: assign at least one outgoing lane for every non-destination lane. Select each listed node, then open Lane exits.");
        ImGui::BeginChild("Missing lane exits",ImVec2(0,-45));for(auto& issue:validationIssues_)ImGui::BulletText("%s",issue.c_str());ImGui::EndChild();
        if(ImGui::Button("Go to map design")){workspace_=1;ImGui::CloseCurrentPopup();}ImGui::SameLine();if(ImGui::Button("Close"))ImGui::CloseCurrentPopup();ImGui::EndPopup();
    }
    if(ImGui::IsKeyPressed(ImGuiKey_F1))manualOpen_=!manualOpen_;
    if(ImGui::IsKeyPressed(ImGuiKey_Escape) && !ImGui::GetIO().WantTextInput) {
        if(laneExitEditorOpen_){laneExitEditorOpen_=false;turnDirty_=false;turnDraftNode_=-1;}
        else if(editorActionActive())cancelEditorAction();else manualOpen_=false;
    }
    const char* workspaces[]{"Native simulation","SUMO simulation","Map design","Traffic lights","Vehicles","Files"};
    const int pages[]{0,5,1,2,3,4};
    for(int i=0;i<6;++i) {
        if(i)ImGui::SameLine();
        if(ImGui::Selectable(workspaces[i],workspace_==pages[i],0,ImVec2(ImGui::CalcTextSize(workspaces[i]).x+24,24)))workspace_=pages[i];
    }
    ImGui::Separator();
    ImGui::BeginDisabled(clockMode_==1 || stepBatch_.active() || !engineReady_);
    if(ImGui::Button(running_?"Pause":"Play")) {
        bool start=!running_; if(start) { if(mapDirty_) refreshMap(true); cancelEditorAction(); }
        running_=start && engineReady_ && validateRun(); accumulator_=0;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(running_ || stepBatch_.active() || !engineReady_);
    if(ImGui::Button("Step")) { if(mapDirty_) refreshMap(true);cancelEditorAction(); accumulator_=0;if(validateRun())stepBatch_.request(manualStepSeconds_); }
    ImGui::EndDisabled(); ImGui::SameLine();
    if(ImGui::Button("Reset traffic")) reset();
    ImGui::SameLine(); if(ImGui::Button("Fit view")) { mapZoom_=1; mapPan_={}; }
    ImGui::SameLine();ImGui::TextDisabled("%s | %.1f s",engine_==SimulationEngine::Native?"Native":"SUMO",elapsed_);
    ImGui::SameLine();if(ImGui::Button("User manual (F1)"))manualOpen_=true;
    ImGui::Separator();
    const float panelHeight=std::max(100.f,ImGui::GetContentRegionAvail().y-28);
    ImGui::BeginChild("Controls",ImVec2(350,panelHeight),ImGuiChildFlags_Borders);
    controls();
    ImGui::EndChild(); ImGui::SameLine();
    // SUMO lane decisions advance at a fixed 10 Hz; free motion uses 100 Hz substeps.
    const double dt=engine_==SimulationEngine::Sumo?.1:1.0/120;
    if (running_ && clockMode_==0) accumulator_+=std::clamp(realSeconds,0.0,0.1)*timeScale_;
    while (running_ && accumulator_>=dt) {
        advanceTick(dt); accumulator_-=dt;
    }
    if(stepBatch_.active()) {
        // A long requested step is processed across frames, not in one giant
        // blocking tick. One expensive path search can still exceed this budget.
        auto started=std::chrono::steady_clock::now();
        do {
            stepBatch_.advanceOne([&](double seconds) { advanceTick(seconds); });
        } while(stepBatch_.active() && std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count()<8);
    }
    ImGui::BeginChild("Workspace",ImVec2(0,panelHeight));
    ImGui::Text("%zu active   |   %llu completed   |   %zu in open junctions",
        simulation_.entities().size(),static_cast<unsigned long long>(simulation_.completed()),simulation_.openJunctionVehicles());
    if(ImGui::CollapsingHeader("Live statistics and diagnostics")) {
    if(sumo_) ImGui::Text("SUMO collisions: %llu | teleports: %llu | free handoffs: %llu | pending: %zu",
        static_cast<unsigned long long>(sumo_->collisions()),static_cast<unsigned long long>(sumo_->teleports()),
        static_cast<unsigned long long>(sumo_->handoffs()),sumo_->pending());
    ImGui::Text("%zu active | %llu completed | %.1fs | %llu gap replans",
        simulation_.entities().size(),static_cast<unsigned long long>(simulation_.completed()),elapsed_,static_cast<unsigned long long>(simulation_.replans()));
    ImGui::Text("Lane transitions: %llu",static_cast<unsigned long long>(simulation_.totalLaneChanges()));
    auto stops=simulation_.stopStatistics(includeCompletedStops_);
    ImGui::Text("Stopped seconds / vehicle: mean (average) %.2f | min %.2f | max %.2f",
        stops.meanSeconds,stops.minSeconds,stops.maxSeconds);
    if(editorMode_==7) ImGui::TextColored(ImVec4(.3f,.9f,1,1),"Permissive light: click %s %s",linkedStage_==0?"stop-line":"allowed-exit",linePending_?"end":"start");
    if(editorMode_==8) ImGui::TextColored(ImVec4(1,.7f,.3f,1),"Queue detector for L%d: click %s (Esc cancels)",linkedLight_,linePending_?"end":"start");
    }
    canvas();
    if(showInspector_) {
        ImGui::BeginChild("Inspector",ImVec2(0,200),ImGuiChildFlags_Borders);inspector();ImGui::EndChild();
    }
    ImGui::EndChild();
    ImGui::Separator();
    auto summary=status_.empty()?std::string("Ready"):status_.substr(0,status_.find('\n'));
    ImGui::TextUnformatted(summary.c_str());
    if(ImGui::IsItemHovered() && !status_.empty()) {
        ImGui::BeginTooltip();ImGui::PushTextWrapPos(ImGui::GetFontSize()*75);ImGui::TextUnformatted(status_.c_str());ImGui::PopTextWrapPos();ImGui::EndTooltip();
    }
    ImGui::End();
    manual();
}
void SimulationView::menuBar() {
    bool newMap=false;
    if(ImGui::BeginMenuBar()) {
        if(ImGui::BeginMenu("File")) {
            if(ImGui::MenuItem("New map...")) newMap=true;
            if(ImGui::MenuItem("Open map PNG...")) fileAction(0);
            if(ImGui::MenuItem("Open node network...")) fileAction(1);
            if(ImGui::BeginMenu("Examples")){
                if(ImGui::MenuItem("Seven-lane routing"))loadLaneDemo();
                ImGui::EndMenu();
            }
            ImGui::Separator();
            if(ImGui::MenuItem("Save node network")) fileAction(2);
            if(ImGui::MenuItem("Save node network as...")) fileAction(3);
            if(ImGui::MenuItem("Export map PNG...")) fileAction(4);
            ImGui::Separator();
            if(ImGui::MenuItem("Map scale and file paths")) workspace_=4;
            ImGui::EndMenu();
        }
        if(ImGui::BeginMenu("View")) {
            if(ImGui::MenuItem("Fit map")) { mapZoom_=1; mapPan_={}; }
            ImGui::MenuItem("Selected route",nullptr,&routes_);
            ImGui::MenuItem("Vehicle clearance",nullptr,&showClearance_);
            ImGui::MenuItem("Vehicle inspector",nullptr,&showInspector_);
            ImGui::EndMenu();
        }
        if(ImGui::BeginMenu("Simulation")) {
            if(ImGui::MenuItem("Native simulation",nullptr,engine_==SimulationEngine::Native))workspace_=0;
            if(ImGui::MenuItem("SUMO simulation",nullptr,engine_==SimulationEngine::Sumo))workspace_=5;
            ImGui::EndMenu();
        }
        if(ImGui::BeginMenu("Help")) {
            if(ImGui::MenuItem("User manual","F1"))manualOpen_=true;
            ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
    }
    if(newMap) ImGui::OpenPopup("New map");
    if(ImGui::BeginPopupModal("New map",nullptr,ImGuiWindowFlags_AlwaysAutoResize)) {
        ImGui::TextUnformatted("Creating a map replaces the current layout. Save your work first.");
        ImGui::InputInt("Width (pixels)",&newWidth_); ImGui::InputInt("Height (pixels)",&newHeight_);
        ImGui::InputDouble("Meters per pixel",&metersPerPixel_,0,0,"%.6f");
        bool valid=std::isfinite(metersPerPixel_) && metersPerPixel_>=.01 && metersPerPixel_<=100 && newWidth_>=8 && newWidth_<=2048 && newHeight_>=8 && newHeight_<=2048;
        ImGui::BeginDisabled(!valid);
        if(ImGui::Button("Create map")) {
            source_=Map(newWidth_,newHeight_,std::vector<Tile>(static_cast<std::size_t>(newWidth_)*newHeight_,Tile::Wall),metersPerPixel_);
            graph_={}; lanes_.clear(); lights_.clear(); pathNodes_.clear(); selectedNode_=-1; editorMode_=1; workspace_=1;
            refreshMap(true);editorMode_=1; ImGui::CloseCurrentPopup();
        }
        ImGui::EndDisabled(); ImGui::SameLine();
        if(ImGui::Button("Cancel")) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}
void SimulationView::fileAction(int action) {
    try {
        const bool png=action==0 || action==4;
        std::string path=action==2?laneFile_:chooseFile(action>=2,png?"PNG maps\0*.png\0\0":"Node networks\0*.nodes\0\0",png?"png":"nodes",png?(action==0?mapPath_:savePath_):laneFile_);
        if(path.empty()) return;
        if(action==0) {
            auto map=Map::fromPng(path,metersPerPixel_); source_=std::move(map); graph_={}; lanes_.clear(); lights_.clear(); pathNodes_.clear(); selectedNode_=-1; editorMode_=0;
            std::snprintf(mapPath_,sizeof(mapPath_),"%s",path.c_str()); refreshMap(true,true); workspace_=1;
        } else if(action==1) {
            auto graph=NodeGraph::load(path);
            if(!applyGraph(graph)) return;
            pathNodes_.clear(); selectedNode_=-1;
            if(std::filesystem::exists(path+".signals")) { simulation_.loadTrafficLights(path+".signals"); lights_=simulation_.trafficLights(); syncSignalMask(); }
            std::snprintf(laneFile_,sizeof(laneFile_),"%s",path.c_str()); workspace_=1;
        } else if(action==4) {
            syncSignalMask(); source_.savePng(path); simulation_.saveTrafficLights(path+".signals");
            std::snprintf(savePath_,sizeof(savePath_),"%s",path.c_str());
        } else {
            graph_.save(path); simulation_.saveTrafficLights(path+".signals");
            std::snprintf(laneFile_,sizeof(laneFile_),"%s",path.c_str());
        }
        status_=(action<2?"Opened: ":"Saved: ")+path;
    } catch(const std::exception& error) { status_=error.what(); }
    accumulator_=0;
}
void SimulationView::simulationControls() {
    const auto target=workspace_==0?SimulationEngine::Native:SimulationEngine::Sumo;
    if(engine_!=target || !engineReady_) {
        if(ImGui::Button(target==SimulationEngine::Native?"Use Native":"Use SUMO"))selectEngine(target);
        if(ImGui::IsItemHovered())ImGui::SetTooltip("Resets traffic and pauses. Map, signals and vehicle profiles are kept.");
    } else ImGui::TextColored(ImVec4(.4f,.9f,.65f,1),"Active engine");
    ImGui::BeginDisabled(engine_!=target || !engineReady_);
    if(target==SimulationEngine::Native) {
        if(ImGui::CollapsingHeader("Junction priority")) {
            bool changed=ImGui::Checkbox("Swap priority after a long yield",&flow_.swapStarvedPriorities);
            ImGui::BeginDisabled(!flow_.swapStarvedPriorities);
            changed|=typedDouble("Yield timeout (s)",&flow_.priorityWaitSeconds,1,3600,"%.1f");ImGui::EndDisabled();
            if(changed)simulation_.setTrafficFlowSettings(flow_);
        }
        if(ImGui::CollapsingHeader("Native driving")) {
            bool changed=ImGui::Checkbox("Predictive junctions",&flow_.predictiveJunctions);
            changed|=ImGui::Checkbox("Sublane movement",&flow_.sublanes);
            changed|=typedDouble("Lateral resolution (m)",&flow_.lateralResolution,.1,2,"%.2f");
            changed|=typedDouble("Lateral gap (m)",&flow_.lateralGap,0,2,"%.2f");
            if(changed)simulation_.setTrafficFlowSettings(flow_);
            bool urban=ImGui::Checkbox("Smooth mixed traffic",&urbanTraffic_.enabled);
            urban|=typedDouble("Two-wheeler headway (s)",&urbanTraffic_.motorbikeHeadway,.2,3,"%.2f");
            urban|=typedDouble("Conflict prediction (s)",&urbanTraffic_.predictionSeconds,.2,3,"%.2f");
            if(urban)simulation_.setUrbanTrafficSettings(urbanTraffic_);
        }
        if(ImGui::CollapsingHeader("Native recovery")) {
            bool changed=ImGui::Checkbox("Replan stopped traffic",&trafficRecoverySettings_.enabled);
            changed|=typedDouble("Replan delay (s)",&trafficRecoverySettings_.stopSeconds,.1,30,"%.2f");
            changed|=typedDouble("Replan radius (m)",&trafficRecoverySettings_.radius,5,250,"%.1f");
            if(changed)simulation_.setTrafficRecoverySettings(trafficRecoverySettings_);
            bool removal=ImGui::Checkbox("Remove stuck queue leaders",&stuckVehicleSettings_.enabled);
            removal|=typedDouble("Queue blocker timeout (s)",&stuckVehicleSettings_.timeoutSeconds,1,3600,"%.1f");
            if(removal)simulation_.setStuckVehicleSettings(stuckVehicleSettings_);
            bool escapes=ImGui::Checkbox("Release stale reservations",&gridlockEscape_.enabled);
            escapes|=typedDouble("Reservation timeout (s)",&gridlockEscape_.afterSeconds,1,3600,"%.1f");
            if(escapes)simulation_.setGridlockEscapeSettings(gridlockEscape_);
        }
        if(ImGui::CollapsingHeader("Native advanced controls")) {
            bool routing=ImGui::Checkbox("RTS routing",&routing_.rtsStyle);
            routing|=ImGui::Checkbox("Refresh moving routes",&routing_.refreshMovingRoutes);
            routing|=typedDouble("Route refresh interval (s)",&routing_.movingRefreshInterval,.25,120,"%.2f");
            if(routing)simulation_.setRoutingSettings(routing_);
            if(ImGui::Checkbox("Tug assistance",&tugSettings_.enabled))simulation_.setTugSettings(tugSettings_);
            if(ImGui::Checkbox("Turn negotiation",&turnNegotiation_.enabled))simulation_.setTurnNegotiationSettings(turnNegotiation_);
            if(ImGui::SliderInt("Collision workers",&collisionWorkers_,1,static_cast<int>(Simulation::maximumCollisionWorkers())))simulation_.setCollisionWorkers(collisionWorkers_);
        }
    } else if(ImGui::CollapsingHeader("SUMO diagnostics") && sumo_)ImGui::TextWrapped("%s",sumo_->status().c_str());
    if(ImGui::CollapsingHeader("Clock and spawning",ImGuiTreeNodeFlags_DefaultOpen)) {
        const char* modes[]{"Real time","Manual steps"};
        if(ImGui::Combo("Clock",&clockMode_,modes,2)){running_=false;accumulator_=0;stepBatch_.cancel();}
        if(typedFloat("Seconds per step",&manualStepSeconds_,.1f,3600,"%.1f"))manualStepSeconds_=std::max(.1f,std::round(manualStepSeconds_*10)/10);
        if(stepBatch_.active()) {
            ImGui::ProgressBar(static_cast<float>(stepBatch_.progress()),ImVec2(-1,0));
            if(ImGui::Button("Cancel remaining step"))stepBatch_.cancel();
        }
        ImGui::BeginDisabled(clockMode_==1);ImGui::SliderFloat("Time scale",&timeScale_,.1f,5,"%.1fx");ImGui::EndDisabled();
        ImGui::InputInt("Seed",&seed_);
        ImGui::Checkbox("Automatic spawning",&automatic_);
        ImGui::SetNextItemWidth(100);
        if(ImGui::InputInt("Vehicle limit",&maximumVehicles_,0,0,ImGuiInputTextFlags_EnterReturnsTrue))maximumVehicles_=std::clamp(maximumVehicles_,1,10000);
        if(ImGui::Button("Spawn vehicle") && validateRun()) {
            std::vector<int> entries;
            for(std::size_t l=0;l<laneOriginNodes_.size();++l)if(laneOriginNodes_[l]>=0 && graph_.nodes[laneOriginNodes_[l]].spawn)entries.push_back(static_cast<int>(l));
            try {
                bool spawned=!entries.empty() && simulation_.spawnRandomOnLane(entries[std::uniform_int_distribution<std::size_t>(0,entries.size()-1)(spawnNodeRandom_)]);
                if(spawned){++spawned_;status_="Vehicle requested.";}else status_="No feasible insertion or destination.";
            } catch(const std::exception& e){status_=e.what();running_=false;}
        }
    }
    ImGui::EndDisabled();
}
void SimulationView::controls() {
    if(workspace_!=1 || running_){laneExitEditorOpen_=false;turnDirty_=false;}
    if(workspace_==1 && laneExitEditorOpen_) {
        if(selectedNode_<0 || selectedNode_>=static_cast<int>(graph_.nodes.size())) { laneExitEditorOpen_=false;turnDirty_=false; }
        else {
            ImGui::SeparatorText("Lane connections");
            ImGui::Text("At node N%d",selectedNode_);
            ImGui::BeginDisabled(turnDirty_);
            bool done=ImGui::Button("Done / return to map tools");ImGui::EndDisabled();
            if(done) { laneExitEditorOpen_=false;turnDirty_=false;turnDraftNode_=-1; }
            else laneConnectionsPopup();
            return;
        }
    }
    ImGui::PushItemWidth(170);
    const char* titles[]{"Native simulation","Map design","Traffic lights","Vehicle settings","Files and map scale","SUMO simulation"};
    ImGui::SeparatorText(titles[workspace_]);
    if(workspace_==1 || workspace_==4) {
    if(workspace_==1) {
        ImGui::TextWrapped("Paint the drivable area, sketch roads inside it, then set where vehicles start and finish.");
        if(ImGui::Button("Fit map view")) { mapZoom_=1; mapPan_={}; }
        ImGui::SameLine();
        if(ImGui::Button("Design guide")) { manualOpen_=true; manualChapter_=5; manualSearch_[0]=0; }
        ImGui::TextDisabled("Wheel: zoom | Middle drag: pan");
        if(editorActionActive()) {
            ImGui::TextColored(ImVec4(1,.85f,.35f,1),"Active: %s",editorActionLabel());
            if(drawingRoad_) ImGui::TextWrapped("Click points on the road area. Add at least two, then Finish road. Existing nodes snap to your sketch.");
            else if(editorMode_==1 || editorMode_==2) ImGui::TextWrapped("Drag on the map to paint or erase the drivable area. Painting alone does not create traffic lanes.");
            else if(splittingRoad_ || placingRoadSignal_) ImGui::TextWrapped("Click an existing road centerline to insert a node.");
            else if(connectionFrom_>=0) ImGui::TextWrapped("Click the other node to complete the connection action.");
            else if(editorMode_==3) ImGui::TextWrapped("Click inside the road area to add a node. Connect it from its Roads tab.");
            else ImGui::TextWrapped("Click the start and end of the line on the map.");
            if(drawingRoad_) {
                ImGui::Text("%zu points placed (minimum 2)",roadDraft_.size());
                ImGui::BeginDisabled(roadDraft_.empty());
                if(ImGui::Button("Undo last point")){roadDraft_.pop_back();roadDraftWidths_.pop_back();}
                ImGui::EndDisabled();
                if(ImGui::TreeNode("Taper and point widths")) {
                ImGui::TextUnformatted("Width at next node (m)"); ImGui::SetNextItemWidth(-1);
                ImGui::SliderFloat("##next_node_width",&nextPointWidth_,.5f,15,"%.2f");
                for(std::size_t i=0;i<roadDraftWidths_.size();++i) {
                    ImGui::PushID(static_cast<int>(i)); float width=static_cast<float>(roadDraftWidths_[i]);
                    ImGui::Text("Sketch node %zu width (m)",i); ImGui::SetNextItemWidth(-1);
                    if(ImGui::SliderFloat("##draft_width",&width,.5f,15,"%.2f")) roadDraftWidths_[i]=width;
                    ImGui::PopID();
                }

                ImGui::TreePop();
                }
                ImGui::BeginDisabled(roadDraft_.size()<2);
                if(ImGui::Button("Finish road")) {
                    try {
                        auto proposed=graph_; proposed.useExplicitDestinations(); int a=proposed.addNode(roadDraft_.front()),b=proposed.addNode(roadDraft_.back());
                        if(proposed.pathConnection(a,b)>=0) throw std::invalid_argument("Those junctions are already connected; select that path to edit it.");
                        proposed.connect(a,b,laneWidth_,laneSpeed_); auto& road=proposed.connections.back(); road.roadType=newRoadType_; road.laneCount=newRoadLanes_; road.shape=roadDraft_; road.shapeWidths=roadDraftWidths_; road.width=roadDraftWidths_.front();
                        if(applyGraph(proposed)) { drawingRoad_=false; roadDraft_.clear(); roadDraftWidths_.clear(); }
                    } catch(const std::exception& e) { status_=e.what(); }
                }
                ImGui::EndDisabled();
            }
            if(ImGui::Button("Stop tool (Esc)")) cancelEditorAction();
        } else ImGui::TextWrapped("Select a tool below, or click a node on the map to edit it.");
        if(ImGui::CollapsingHeader("1. Paint drivable area",ImGuiTreeNodeFlags_DefaultOpen)) {
            if(ImGui::Button("Paint road area")) { cancelEditorAction(); editorMode_=1; running_=false; stepBatch_.cancel(); }
            ImGui::SameLine();
            if(ImGui::Button("Erase area")) { cancelEditorAction(); editorMode_=2; running_=false; stepBatch_.cancel(); }
            ImGui::SetNextItemWidth(-1); ImGui::SliderFloat("##paint_width",&brushWidth_,.2f,20,"Brush: %.1f m");
        }
    }
    if(workspace_==1 && ImGui::CollapsingHeader("2. Create roads",ImGuiTreeNodeFlags_DefaultOpen)) {
        if(ImGui::TreeNode("Example networks")) {
        if(ImGui::Button("Load lane-change demonstration")) {
            NodeGraph demo; demo.explicitDestinations=true; demo.addNode({10,25}); demo.addNode({190,25}); demo.nodes[0].spawn=true; demo.nodes[1].destination=true;
            demo.connect(0,1,4,10); demo.connections[0].roadType=1; demo.connections[0].laneCount=2;
            for(int x=10;x<=190;++x) demo.connections[0].shape.push_back({static_cast<double>(x),25});
            source_=Map(200,50,std::vector<Tile>(10000,Tile::Road),1); graph_=demo; lights_.clear(); metersPerPixel_=1; pathNodes_.clear(); selectedNode_=-1; mapZoom_=1; mapPan_={};
            reset(); automatic_=false; clockMode_=0; timeScale_=1;
            auto profile=profiles_[static_cast<int>(VehicleKind::Car)]; simulation_.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{8,0,2,0},1});
            if(simulation_.spawnOnLane(1,VehicleKind::Car)) simulation_.setEntityMotion(simulation_.entities().back().id,2.5,2);
            simulation_.tick(8);
            if(simulation_.spawnOnLane(1,VehicleKind::Car)) simulation_.setEntityMotion(simulation_.entities().back().id,8,2);
            simulation_.setProfile(VehicleKind::Car,profile); spawned_=simulation_.entities().size(); elapsed_=simulation_.seconds(); running_=true;
            status_="Native passing demo: 2.5 m/s leader and 8 m/s follower in the right lane. Observe speed-gain passing, then keep-right return. Automatic spawning is off.";
        }
        if(ImGui::Button("Load Wassup5 road preset")) {
            try {
                auto map=Map::fromPng(std::string(TRAFFIC_PROJECT_ROOT)+"/examples/Wassup5.png",.73);
                auto graph=NodeGraph::load(std::string(TRAFFIC_PROJECT_ROOT)+"/examples/Wassup5.nodes");
                graph.nodes[0].openRadius=160; for(auto& node:graph.nodes) { node.signal=false; if(node.spawn) node.rate=.015; }
                source_=std::move(map); metersPerPixel_=.73; lights_.clear(); graph_=graph; pathNodes_.clear(); selectedNode_=-1;
                std::snprintf(mapPath_,sizeof(mapPath_),"%s/examples/Wassup5.png",TRAFFIC_PROJECT_ROOT);
                running_=false; reset(); status_="Wassup5: right-hand traffic, paired one-way carriageways, others two-way; 0.73 m/pixel. Native lane changes and open-junction navigation enabled.";
            } catch(const std::exception& e) { status_=e.what(); }
        }
        ImGui::TreePop();}
        ImGui::TextUnformatted("Direction");ImGui::SetNextItemWidth(-1);ImGui::Combo("##Road direction",&newRoadType_,"Legacy centerline\0One-way road\0Two-way road\0");
        if(newRoadType_==1) ImGui::TextWrapped("Traffic travels from your first point to your last point.");
        ImGui::TextUnformatted("Lanes per direction");ImGui::SetNextItemWidth(-1);ImGui::InputInt("##Road lanes",&newRoadLanes_);newRoadLanes_=std::clamp(newRoadLanes_,1,maximumRoadLanes);
        ImGui::TextUnformatted("Lane width (m)"); ImGui::SetNextItemWidth(-1);
        ImGui::SliderFloat("##connection_width",&laneWidth_,.5f,10,"%.1f");
        ImGui::TextUnformatted("Speed limit (m/s)"); ImGui::SetNextItemWidth(-1);
        ImGui::SliderFloat("##connection_speed",&laneSpeed_,.5f,30,"%.1f");
        ImGui::BeginDisabled(drawingRoad_);
        if(ImGui::Button("Sketch road")) { cancelEditorAction(); drawingRoad_=true; nextPointWidth_=laneWidth_; running_=false; stepBatch_.cancel(); }
        ImGui::EndDisabled();

        ImGui::Text("%zu junction nodes | %zu roads",graph_.nodes.size(),graph_.connections.size());

    }
    if(workspace_==1 && ImGui::CollapsingHeader("3. Configure nodes and trips",ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::TextWrapped("Click a node: General sets Spawn point and Trip destination. Lane exits sets where each incoming lane can go (required before Play).");
        if(ImGui::Button("Select / inspect")) cancelEditorAction();
        if(ImGui::Button("Place signal node on road")){cancelEditorAction();placingRoadSignal_=true;running_=false;stepBatch_.cancel();}
        if(ImGui::Button("Insert junction node on road")) { cancelEditorAction(); splittingRoad_=true; running_=false; stepBatch_.cancel(); status_="Click an existing road centerline, then connect a side road to the inserted node."; }
        if(ImGui::Button("Place nodes")) { cancelEditorAction(); editorMode_=3; running_=false; stepBatch_.cancel(); }
        if(connectionFrom_>=0) {
            ImGui::Text("Source node: %d",connectionFrom_);
        }
    }
    if(workspace_==1 && ImGui::CollapsingHeader("Advanced road properties")) {
        ImGui::TextWrapped("Ctrl + click connected nodes in travel order to select roads for editing.");
        ImGui::Text("Selected path: %zu nodes",pathNodes_.size());
        if(ImGui::Button("Clear selected path")) pathNodes_.clear();
        if(pathNodes_.size()>1) {
            int edge=graph_.pathConnection(pathNodes_[0],pathNodes_[1]);
            if(edge>=0) {
                float smooth=static_cast<float>(graph_.connections[edge].smoothing);
                ImGui::TextUnformatted("Path turn smoothness"); ImGui::SetNextItemWidth(-1);
                if(ImGui::SliderFloat("##path_smoothness",&smooth,.2f,4,"%.2fx")) {
                    auto proposed=graph_;
                    for(std::size_t i=1;i<pathNodes_.size();++i) { int e=proposed.pathConnection(pathNodes_[i-1],pathNodes_[i]); if(e>=0) proposed.connections[e].smoothing=smooth; }
                    applyGraph(proposed);
                }

                if(ImGui::TreeNode("Widths at every sketch node")) {
                    auto widths=shapeWidths(graph_.connections[edge]);
                    for(std::size_t point=0;point<widths.size();++point) {
                        ImGui::PushID(static_cast<int>(point)); ImGui::Text("Shape node %zu width (m)",point); float width=static_cast<float>(widths[point]);
                        if(ImGui::SliderFloat("##shape_width",&width,.5f,15,"%.2f")) { widths[point]=width; auto proposed=graph_; proposed.connections[edge].shapeWidths=widths; applyGraph(proposed); }
                        const auto& road=graph_.connections[edge];
                        Vec2 position=road.shape.empty()?graph_.nodes[point==0?road.from:road.to].position:road.shape[point];
                        bool turn=false; for(const auto& node:graph_.nodes) if(length(node.position-position)<.05) turn=node.turn;
                        if(ImGui::Checkbox("This sketch node is a turn",&turn)) {
                            int node=graph_.addNode(position); graph_.nodes[node].turn=turn;
                            std::vector<TurnControl> controls; for(const auto& n:graph_.nodes) if(n.turn) controls.push_back({n.position,n.turnRadius,n.braking}); simulation_.setTurnControls(std::move(controls));
                        }
                        ImGui::PopID();
                    }
                    ImGui::TreePop();
                }
                int roadType=graph_.connections[edge].roadType,lanes=graph_.connections[edge].laneCount;
                bool changed=ImGui::Combo("Selected road type",&roadType,"Legacy centerline\0One-way road\0Two-way road\0");
                changed|=ImGui::SliderInt("Selected lanes / dir",&lanes,1,maximumRoadLanes);
                auto edited=graph_.connections[edge];
                int reverse=edited.reverseLaneCount>0?edited.reverseLaneCount:edited.laneCount;
                if(roadType==2) changed|=ImGui::SliderInt("Opposing lanes",&reverse,1,maximumRoadLanes);
                float shift=static_cast<float>(edited.lateralOffset),endScale=static_cast<float>(!edited.shapeWidths.empty()?edited.shapeWidths.back()/edited.shapeWidths.front():edited.endWidth>0?edited.endWidth/edited.width:1);
                changed|=ImGui::SliderFloat("Road lateral offset (m)",&shift,-30,30,"%.1f");
                bool endEdited=ImGui::SliderFloat("End width ratio",&endScale,.4f,2,"%.2fx"); changed|=endEdited;
                edited.forwardWidths.resize(lanes,edited.width);
                edited.reverseWidths.resize(reverse,edited.width);
                for(int direction=0;direction<(roadType==2?2:1);++direction) {
                    auto& widths=direction?edited.reverseWidths:edited.forwardWidths;
                    for(std::size_t lane=0;lane<widths.size();++lane) {
                        auto label=std::string(direction?"Reverse lane ":"Forward lane ")+std::to_string(lane)+" width (m)";
                        float width=static_cast<float>(widths[lane]);
                        ImGui::TextUnformatted(label.c_str()); ImGui::SetNextItemWidth(-1);
                        if(ImGui::SliderFloat(("##"+label).c_str(),&width,.5f,10,"%.1f")) { widths[lane]=width; changed=true; }
                    }
                }
                if(changed) {
                    auto proposed=graph_;
                    for(std::size_t i=1;i<pathNodes_.size();++i) { int e=proposed.pathConnection(pathNodes_[i-1],pathNodes_[i]); if(e>=0) { proposed.connections[e].roadType=roadType; proposed.connections[e].laneCount=lanes; proposed.connections[e].reverseLaneCount=reverse; proposed.connections[e].lateralOffset=shift; proposed.connections[e].endWidth=proposed.connections[e].width*endScale; if(endEdited && !proposed.connections[e].shapeWidths.empty()) proposed.connections[e].shapeWidths.back()=proposed.connections[e].shapeWidths.front()*endScale; proposed.connections[e].forwardWidths=edited.forwardWidths; proposed.connections[e].reverseWidths=edited.reverseWidths; } }
                    applyGraph(proposed);
                }
                roadSections(edge);
            }
        }
    }
    if(workspace_==4) {
        ImGui::TextUnformatted("Network file"); ImGui::SetNextItemWidth(-1);
        ImGui::InputText("##network_file",laneFile_,sizeof(laneFile_));
        if(ImGui::Button("Save node network")) { try { graph_.save(laneFile_); simulation_.saveTrafficLights(std::string(laneFile_)+".signals"); status_="Node network and linked signals saved."; } catch(const std::exception& e) { status_=e.what(); } }
        if(ImGui::Button("Load node network")) { try { auto graph=NodeGraph::load(laneFile_); if(applyGraph(graph)) { pathNodes_.clear(); selectedNode_=-1; if(std::filesystem::exists(std::string(laneFile_)+".signals")) { simulation_.loadTrafficLights(std::string(laneFile_)+".signals"); lights_=simulation_.trafficLights(); syncSignalMask(); } } } catch(const std::exception& e) { status_=e.what(); } }
        if(ImGui::Button("Import old .lanes file")) { try { Simulation check(source_,42); check.loadLanes(laneFile_); if(applyGraph(NodeGraph::fromLanes(check.lanes()))) { pathNodes_.clear(); selectedNode_=-1; } } catch(const std::exception& e) { status_=e.what(); } }
        if(ImGui::Button("Clear node network (reset traffic)")) { applyGraph({}); pathNodes_.clear(); selectedNode_=-1; }
    }
    }
    if(workspace_==0 || workspace_==5)simulationControls();
    if(workspace_==3) {
    if(ImGui::CollapsingHeader("Vehicle spawn mix (%)",ImGuiTreeNodeFlags_DefaultOpen)) {
        double total=0; bool valid=true;
        for(int i=0;i<vehicleKindCount;++i) {
            spawnMixDirty_|=ImGui::InputFloat(vehicleName(static_cast<VehicleKind>(i)),&spawnMix_[i],0,0,"%.2f");
            total+=spawnMix_[i];
            valid=valid && std::isfinite(spawnMix_[i]) && spawnMix_[i]>=0 && spawnMix_[i]<=100;
        }
        ImGui::Text("Total: %.2f%% | %s",total,spawnMixDirty_?"Unapplied changes":"Applied");
        ImGui::BeginDisabled(!valid || !std::isfinite(total) || std::abs(total-100)>.01);
        if(ImGui::Button("Apply mix")) {
            std::array<double,vehicleKindCount> shares{};
            for(int i=0;i<vehicleKindCount;++i) shares[i]=spawnMix_[i];
            simulation_.setSpawnPercentages(shares);
            auto applied=simulation_.spawnPercentages();
            for(int i=0;i<vehicleKindCount;++i) { profiles_[i].weight=simulation_.profile(static_cast<VehicleKind>(i)).weight; spawnMix_[i]=static_cast<float>(applied[i]); }
            spawnMixDirty_=false; status_="Spawn percentages applied to new spawn attempts.";
        }
        ImGui::EndDisabled(); ImGui::SameLine();
        ImGui::BeginDisabled(!std::isfinite(total) || total<=0);
        if(ImGui::Button("Normalize to 100%")) { for(auto& share:spawnMix_) share=static_cast<float>(share*100/total); spawnMixDirty_=true; }
        ImGui::EndDisabled();
        if(ImGui::Button("Revert mix")) {
            auto shares=simulation_.spawnPercentages();
            for(int i=0;i<vehicleKindCount;++i) spawnMix_[i]=static_cast<float>(shares[i]);
            spawnMixDirty_=false;
        }

    }
    }
    if((workspace_==0 && engine_==SimulationEngine::Native) || (workspace_==5 && engine_==SimulationEngine::Sumo)) {
    if(ImGui::CollapsingHeader("Monitoring")) {
        if(sumo_) {
            ImGui::Text("SUMO collisions: %llu",static_cast<unsigned long long>(sumo_->collisions()));
            ImGui::Text("SUMO teleports: %llu",static_cast<unsigned long long>(sumo_->teleports()));
            ImGui::Text("Open-junction handoffs: %llu",static_cast<unsigned long long>(sumo_->handoffs()));
            ImGui::Text("Waiting off-network: %zu",sumo_->pending());
        }

        bool changed=ImGui::Checkbox("Delete stopped in open junctions",&stuckVehicleSettings_.junctionEnabled);
        changed|=typedDouble("Junction stationary timeout (s)",&stuckVehicleSettings_.junctionTimeoutSeconds,1,3600,"%.2f");
        if(changed) simulation_.setStuckVehicleSettings(stuckVehicleSettings_);
        ImGui::Text("Open-junction deletions: %llu",static_cast<unsigned long long>(simulation_.removedJunctionVehicles()));
        ImGui::Checkbox("Include finished / removed trips",&includeCompletedStops_);
        auto stats=simulation_.stopStatistics(includeCompletedStops_);
        if(ImGui::BeginTable("Stopped time",3,ImGuiTableFlags_BordersInnerV|ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableSetupColumn("Mean (s)");ImGui::TableSetupColumn("Min (s)");ImGui::TableSetupColumn("Max (s)");ImGui::TableHeadersRow();
            ImGui::TableNextRow();ImGui::TableNextColumn();ImGui::Text("%.2f",stats.meanSeconds);
            ImGui::TableNextColumn();ImGui::Text("%.2f",stats.minSeconds);ImGui::TableNextColumn();ImGui::Text("%.2f",stats.maxSeconds);ImGui::EndTable();
        }
    }
    ImGui::Checkbox("Show selected route",&routes_);
    }
    if(workspace_==3) {
    if(ImGui::CollapsingHeader("Vehicle population",ImGuiTreeNodeFlags_DefaultOpen)) {
    const char* names[]{"Bike","Motorbike","Car","Truck","Bus"};
    ImGui::Combo("Vehicle class",&selectedKind_,names,vehicleKindCount);
    auto& profile=profiles_[selectedKind_];
    auto& distribution_=profile.motion;
    ImGui::TextDisabled("Speed: m/s | Acceleration: m/s^2");
    float mean=static_cast<float>(distribution_.speedMean),sigma=static_cast<float>(distribution_.speedStdDev);
    bool changed=ImGui::SliderFloat("Speed mean",&mean,0.05f,15,"%.2f");
    changed|=ImGui::SliderFloat("Speed std dev",&sigma,0,8,"%.2f");
    distribution_.speedMean=mean; distribution_.speedStdDev=sigma;
    distributionPlot("##speedCurve",mean,sigma,std::max(5.f,mean+3*sigma));
    mean=static_cast<float>(distribution_.accelerationMean); sigma=static_cast<float>(distribution_.accelerationStdDev);
    changed|=ImGui::SliderFloat("Accel mean",&mean,0.05f,25,"%.2f");
    changed|=ImGui::SliderFloat("Accel std dev",&sigma,0,12,"%.2f");
    distribution_.accelerationMean=mean; distribution_.accelerationStdDev=sigma;
    distributionPlot("##accelCurve",mean,sigma,std::max(5.f,mean+3*sigma));
    auto gaussianControl=[&](const char* name,Gaussian& g,float low,float high) {
        ImGui::PushID(name);
        ImGui::TextUnformatted(name);
        float m=static_cast<float>(g.mean),s=static_cast<float>(g.stdDev);
        bool edit=ImGui::SliderFloat("Mean (m)",&m,low,high,"%.2f");
        edit|=ImGui::SliderFloat("Std dev (m)",&s,0,high*.3f,"%.2f");
        g={m,s}; ImGui::PopID(); return edit;
    };
    changed|=gaussianControl("Body length",profile.length,.2f,20);
    changed|=gaussianControl("Body width",profile.width,.15f,5);
    ImGui::SeparatorText("Car following");
    changed|=typedDouble("Standstill gap (m)",&profile.minGap,0,20,"%.2f");
    changed|=typedDouble("Time headway (s)",&profile.tau,.1,10,"%.2f");
    changed|=typedDouble("Comfortable deceleration (m/s^2)",&profile.deceleration,.1,50,"%.2f");
    profile.emergencyDeceleration=std::max(profile.emergencyDeceleration,profile.deceleration);
    changed|=typedDouble("Emergency deceleration (m/s^2)",&profile.emergencyDeceleration,profile.deceleration,50,"%.2f");
    changed|=typedDouble("Lateral planning gap (m)",&profile.lateralMinGap,0,3,"%.2f");
    ImGui::Text("Applied spawn share: %.2f%%",simulation_.spawnPercentages()[selectedKind_]);
    if (changed) simulation_.setProfile(static_cast<VehicleKind>(selectedKind_),profile);

    if (ImGui::Button("Resample active vehicles")) simulation_.resampleMotion();

    }
    }
    if(workspace_==2) {

    if(ImGui::CollapsingHeader("Add a light")) {
    if(ImGui::Button("Place on road guide",ImVec2(-1,0))){cancelEditorAction();placingRoadSignal_=true;workspace_=1;running_=false;stepBatch_.cancel();}
    if(ImGui::Checkbox("Place stop line on map",&placeLight_) && placeLight_) { editorMode_=0; cancelPendingLine(); }
    if(ImGui::Button("Draw permissive light + exit")) { editorMode_=7; placeLight_=false; cancelPendingLine(); }

    ImGui::Checkbox("Vertical stop line",&verticalLine_);
    ImGui::TextUnformatted("Line length (m)");ImGui::SetNextItemWidth(-1);ImGui::SliderFloat("##line_length",&stopLineLength_,.5f,40,"%.1f");

    }

    ImGui::Text("%zu independent lights",lights_.size());
    if(!lights_.empty()) {
        selectedLight_=std::clamp(selectedLight_,0,static_cast<int>(lights_.size())-1);
        std::string selectedLabel="L"+std::to_string(selectedLight_);
        ImGui::TextUnformatted("Selected light");ImGui::SetNextItemWidth(-1);
        if(ImGui::BeginCombo("##Selected light",selectedLabel.c_str())) {
            for(int i=0;i<static_cast<int>(lights_.size());++i) {
                std::string label="L"+std::to_string(i);
                if(ImGui::Selectable(label.c_str(),selectedLight_==i)) selectedLight_=i;
            }
            ImGui::EndCombo();
        }
        auto& light=lights_[selectedLight_]; bool edit=ImGui::Checkbox("Enabled",&light.enabled);
        if(light.anchorNode>=0)ImGui::Text("Road station: node %d",light.anchorNode);
        auto phase=light.phase(simulation_.seconds());ImGui::SameLine();
        ImGui::TextColored(phase==LightPhase::Red?ImVec4(1,.4f,.4f,1):phase==LightPhase::Yellow?ImVec4(1,.8f,.3f,1):ImVec4(.4f,1,.6f,1),"%s",phase==LightPhase::Red?"RED":phase==LightPhase::Yellow?"YELLOW":"GREEN");
        if(ImGui::CollapsingHeader("Travel direction",ImGuiTreeNodeFlags_DefaultOpen)) {
            auto d=light.controlledDirection();const char* toward=std::abs(d.x)>std::abs(d.y)?(d.x>0?"right":"left"):(d.y>0?"down":"up");
            ImGui::Text("OUT toward %s: obeys signal",toward);ImGui::TextDisabled("IN: reverse direction passes red");
            if(ImGui::Button("Reverse IN / OUT",ImVec2(-1,0))){light.outDirection=d*-1;edit=true;}
        }
        auto duration=[&](const char* label,double& v,double low,double high) { return typedDouble(label,&v,low,high,"%.3f"); };
        if(ImGui::CollapsingHeader("Timing",ImGuiTreeNodeFlags_DefaultOpen)) {
        edit|=duration("Green (s)",light.greenSeconds,.1f,3600);
        edit|=duration("Yellow (s)",light.yellowSeconds,0,3600);
        edit|=duration("Red (s)",light.redSeconds,.1f,3600);
        edit|=duration("Offset (s)",light.offsetSeconds,0,10800);
        }
        if(ImGui::CollapsingHeader("Queue detectors")) {
        edit|=duration("Occupancy window (s)",light.queueWindowSeconds,.1,3600);
        edit|=duration("Occupied threshold (%)",light.queueThresholdPercent,0,100);
        auto queue=simulation_.trafficLightQueueStatus(light.id);
        ImGui::Text("Queue quality: line %d",queue.qualityLine);

        if(ImGui::Button("Draw queue detector line")) {
            cancelPendingLine(); linkedLight_=selectedLight_; editorMode_=8; placeLight_=false;
            drawingRoad_=false; splittingRoad_=false; connectionFrom_=-1; roadDraft_.clear(); roadDraftWidths_.clear();
            status_="Click both ends of the detector line. Traffic can keep running.";
        }
        if(editorMode_==8 && ImGui::Button("Cancel detector drawing")) { editorMode_=0; cancelPendingLine(); }
        for(std::size_t i=0;i<light.queueDetectors.size();++i) {
            ImGui::PushID(static_cast<int>(i));
            const auto& detector=light.queueDetectors[i];
            auto found=std::find_if(queue.lines.begin(),queue.lines.end(),[&](const QueueDetectorStatus& s){return s.id==detector.id;});
            if(found!=queue.lines.end()) {
                ImGui::Text("Line %zu: vehicle %s | %.1f%%",i+1,found->occupied?"YES":"NO",found->occupiedPercent);
                ImGui::TextDisabled("Observed %.1f / %.1f s",found->observedSeconds,light.queueWindowSeconds);
            }
            bool changed=false;
            if(i>0 && ImGui::SmallButton("Move nearer")) { std::swap(light.queueDetectors[i],light.queueDetectors[i-1]); changed=true; }
            if(i+1<light.queueDetectors.size()) {
                ImGui::SameLine();
                if(ImGui::SmallButton("Move farther")) { std::swap(light.queueDetectors[i],light.queueDetectors[i+1]); changed=true; }
            }
            ImGui::SameLine();
            if(ImGui::SmallButton("Delete line")) { light.queueDetectors.erase(light.queueDetectors.begin()+i); changed=true; }
            ImGui::PopID();
            if(changed) { simulation_.setTrafficLights(lights_); break; }
        }
        }
        if(ImGui::CollapsingHeader("Allowed exit")) {
        ImGui::TextDisabled("%s",light.permissive?"Permission line linked":"No permission line");
        if(ImGui::Button(light.permissive?"Redraw allowed exit":"Set allowed exit for this light")) {
            editorMode_=7; placeLight_=false; cancelPendingLine();
            linkedStage_=1; linkedLight_=selectedLight_; pendingStopA_=light.a; pendingStopB_=light.b;
        }
        if(light.permissive && ImGui::Button("Remove red/yellow permission")) { light.permissive=false; simulation_.setTrafficLights(lights_); }
        }
        if(edit) simulation_.setTrafficLights(lights_);
        ImGui::Spacing();ImGui::Separator();
        if(ImGui::Button("Remove selected light",ImVec2(-1,0))) {
            cancelPendingLine(); if(editorMode_==7 || editorMode_==8) editorMode_=0;
            lights_.erase(lights_.begin()+selectedLight_);
            simulation_.setTrafficLights(lights_); syncSignalMask();
            selectedLight_=std::max(0,selectedLight_-1);
            status_="Light removed; traffic and simulation clock preserved.";
        }
    }
    if(engine_==SimulationEngine::Native && ImGui::CollapsingHeader("Native queue planning")) {
    ImGui::SeparatorText("Signal control");
    if(ImGui::Checkbox("Spread across red lights",&routing_.spreadAtSignals)) simulation_.setRoutingSettings(routing_);
    ImGui::Text("Queue placements: %llu",static_cast<unsigned long long>(simulation_.signalPlacements()));
    ImGui::Text("Planning passes: %llu",static_cast<unsigned long long>(simulation_.signalPlanningBatches()));
    ImGui::Text("Last: %llu routes / %.1f ms",static_cast<unsigned long long>(simulation_.lastSignalPlanCount()),simulation_.lastSignalPlanningMilliseconds());
    if(ImGui::Button("Plan all signal queues now")) {
        double before=simulation_.totalSignalPlanningSeconds(); simulation_.planSignalQueues(true);
        planningTimeToExclude_+=simulation_.totalSignalPlanningSeconds()-before;
        status_="Signal routes prepared with the simulation clock held fixed.";
    }

    }
    }
    if(workspace_==4) {
    ImGui::SeparatorText("Map scale and files");
    ImGui::InputDouble("Meters / pixel",&metersPerPixel_,0,0,"%.6f");
    if(ImGui::Button("Apply scale and reset")) {
        try {
            if(!std::isfinite(metersPerPixel_) || metersPerPixel_<.01 || metersPerPixel_>100) throw std::invalid_argument("Meters per pixel must be between 0.01 and 100");
            double factor=metersPerPixel_/source_.metersPerPixel;
            auto map=source_; map.metersPerPixel=metersPerPixel_; map.rebuildFeatures();
            auto graph=graph_; for(auto& node:graph.nodes) node.position=node.position*factor;
            for(auto& road:graph.connections) for(auto& point:road.shape) point=point*factor;
            auto lights=lights_; for(auto& light:lights) { light.a=light.a*factor; light.b=light.b*factor; light.allowedA=light.allowedA*factor; light.allowedB=light.allowedB*factor; for(auto& line:light.queueDetectors) { line.a=line.a*factor; line.b=line.b*factor; } }
            auto lanes=graph.lanes(); validateRoadMask(lanes,map);
            Simulation check(map,seed_); check.setLanes(lanes); check.setTrafficLights(lights);
            source_=std::move(map); graph_=std::move(graph); lights_=std::move(lights); refreshMap(false);
            status_="Scale applied. Authored points stay aligned with the PNG; lane widths and vehicle sizes retain their meter values.";
        } catch(const std::exception& e) { status_=e.what(); }
    }
    ImGui::Text("Physical size: %.1f x %.1f m",source_.width*source_.metersPerPixel,source_.height*source_.metersPerPixel);
    int spawns=0,ends=0,dividers=0;
    for(const auto& feature:source_.features()) { spawns+=feature.type==Tile::Spawn; ends+=feature.type==Tile::Destination; dividers+=feature.type==Tile::Divider; }
    ImGui::Text("Spawns: %d | Ends: %d",spawns,ends);
    ImGui::Text("Signals: %zu | Dividers: %d",lights_.size(),dividers);
    ImGui::InputText("PNG path",mapPath_,sizeof(mapPath_));
    if (ImGui::Button("Load PNG")) {
        try { auto map=Map::fromPng(mapPath_,metersPerPixel_); source_=std::move(map); lanes_.clear(); graph_={}; pathNodes_.clear(); selectedNode_=-1; lights_.clear(); editorMode_=0; linePending_=false; refreshMap(true,true); status_="Map loaded. Place nodes, connect them, and mark spawn nodes."; }
        catch (const std::exception& e) { status_=e.what(); }
    }
    ImGui::SameLine();
    if (ImGui::Button("Demo map")) {
        source_=makeJunctionDemoMap(); graph_=makeJunctionDemoGraph(); pathNodes_.clear(); selectedNode_=-1; metersPerPixel_=1; lights_.clear(); editorMode_=0; linePending_=false; mapZoom_=1; mapPan_={};
        refreshMap(false); status_="Mixed-traffic demo: finite-width two-way lanes, side-by-side insertion, and a free junction. Mark destination nodes and add linked signal permissions independently.";
    }

    }
    if(workspace_==1) {
    if(ImGui::CollapsingHeader("Advanced markings")) {
    ImGui::TextWrapped("Line tools use two clicks. Traffic lights and their timings are also available in Traffic lights.");
    const char* tools[]{"Select / inspect","Paint road area","Erase area","Place road nodes","Allowed exit (selected light)","Traffic-light line","Lane divider","Permissive light + exit"};
    int chosenTool=editorMode_;
    if(ImGui::Combo("Marking tool",&chosenTool,tools,8)) {
        cancelEditorAction(); editorMode_=chosenTool;
        if(editorMode_==4) {
            if(lights_.empty()) { editorMode_=0; status_="Add a traffic light before drawing its allowed exit."; }
            else { selectedLight_=std::clamp(selectedLight_,0,static_cast<int>(lights_.size())-1); editorMode_=7; linkedStage_=1; linkedLight_=selectedLight_; pendingStopA_=lights_[selectedLight_].a; pendingStopB_=lights_[selectedLight_].b; }
        }
        if(editorMode_!=0 && editorMode_!=5 && editorMode_!=7) { running_=false; accumulator_=0; stepBatch_.cancel(); }
    }
    ImGui::SliderFloat("Brush width (m)",&brushWidth_,.2f,20,"%.1f");
    }
    ImGui::TextWrapped("Save your design in File: Export map PNG and Save node network. Keep both files and their .signals companions together.");
    }
    if(workspace_==4) {
    ImGui::InputText("Save PNG path",savePath_,sizeof(savePath_));
    if(ImGui::Button("Save PNG")) {
        try { syncSignalMask(); source_.savePng(savePath_); simulation_.saveTrafficLights(std::string(savePath_)+".signals"); status_="PNG saved with its .signals file containing timings, light identities and linked exits. Keep both files together."; }
        catch(const std::exception& error) { status_=error.what(); }
    }
    ImGui::InputInt("New width (px)",&newWidth_); ImGui::InputInt("New height (px)",&newHeight_);
    if(ImGui::Button("New blank map")) {
        int w=std::clamp(newWidth_,8,2048),h=std::clamp(newHeight_,8,2048);
        source_=Map(w,h,std::vector<Tile>(static_cast<std::size_t>(w)*h,Tile::Wall),metersPerPixel_);
        lights_.clear(); lanes_.clear(); graph_={}; pathNodes_.clear(); selectedNode_=-1; editorMode_=1; linePending_=false; refreshMap(true);editorMode_=1; status_="Blank map ready. Paint roads, then place and connect nodes.";
    }
    }
    ImGui::PopItemWidth();
}
bool SimulationView::applyGraph(const NodeGraph& input) {
    auto proposed=input;proposed.pruneLaneTurns();
    try {
        for(const auto& node:proposed.nodes) if(!source_.contains(source_.pixel(node.position)) || !source_.walkable(source_.pixel(node.position))) throw std::invalid_argument("Nodes must lie on the road mask");
        Simulation check(source_,42); auto lanes=proposed.lanes();
        if(std::any_of(proposed.connections.begin(),proposed.connections.end(),[](const auto& e){return e.roadType!=0 || e.laneCount!=1 || e.reverseLaneCount>0 || !e.changeSections.empty() || !e.laneTurns.empty();})) validateRoadMask(lanes,source_);
        check.setLanes(std::move(lanes));
        graph_=proposed; lanes_=check.lanes(); running_=false; reset();
        status_="Node network applied; traffic reset."; return true;
    } catch(const std::exception& e) { status_=e.what(); return false; }
}
void SimulationView::roadSections(int road) {
    if(road<0 || road>=static_cast<int>(graph_.connections.size())) return;
    auto current=graph_.connections[road];
    auto points=current.shape.empty()?std::vector<Vec2>{graph_.nodes[current.from].position,graph_.nodes[current.to].position}:current.shape;
    double span=0; for(std::size_t i=1;i<points.size();++i) span+=length(points[i]-points[i-1]);

    if(engine_==SimulationEngine::Sumo && !current.changeSections.empty()) {
        if(ImGui::SmallButton("Section compatibility...")){manualChapter_=5;manualOpen_=true;}
    }
    if(engine_==SimulationEngine::Native && ImGui::CollapsingHeader("Lane-change sections")) {
        typedDouble("Section start (m)",&sectionBegin_,0,span,"%.2f");
        typedDouble("Section end (m)",&sectionEnd_,0,span,"%.2f");
        ImGui::Combo("Direction",&sectionDirection_,"Both\0Forward\0Reverse\0");
        int reverse=current.reverseLaneCount>0?current.reverseLaneCount:current.laneCount;
        int count=sectionDirection_==1?current.laneCount:sectionDirection_==2?reverse:std::max(current.laneCount,reverse);
        ImGui::InputInt("Lane (0 = all)",&sectionLane_);sectionLane_=std::clamp(sectionLane_,0,count);
        ImGui::Checkbox("Allow entering",&sectionEnter_);ImGui::Checkbox("Allow leaving",&sectionExit_);
        if(ImGui::Button("Add section")){
            auto proposed=graph_;proposed.connections[road].changeSections.push_back({sectionBegin_,sectionEnd_,sectionDirection_-1,sectionLane_-1,sectionEnter_,sectionExit_});
            applyGraph(proposed);
        }
        for(std::size_t i=0;i<current.changeSections.size();++i){auto& rule=current.changeSections[i];ImGui::PushID(static_cast<int>(i));
            ImGui::Text("%.1f - %.1f m | lane %d",rule.begin,rule.end,rule.lane+1);ImGui::SameLine();
            bool remove=ImGui::SmallButton("Remove");ImGui::PopID();
            if(remove){auto proposed=graph_;proposed.connections[road].changeSections.erase(proposed.connections[road].changeSections.begin()+i);applyGraph(proposed);break;}
        }
    }
    if(current.roadType==2 && ImGui::CollapsingHeader("Mid-road U-turn opening",ImGuiTreeNodeFlags_DefaultOpen)) {
        typedDouble("Opening start (m)",&openingBegin_,1,std::max(1.,span-1),"%.2f");
        typedDouble("Opening end (m)",&openingEnd_,1,std::max(1.,span-1),"%.2f");

        if(ImGui::Button("Insert optional U-turn opening")) {
            try {
                auto proposed=graph_;int node=proposed.addUTurnOpening(road,openingBegin_,openingEnd_);
                if(applyGraph(proposed)) {pathNodes_.clear();selectedNode_=node;status_="U-turn opening added with straight-through connections preserved.";}
            } catch(const std::exception& e) {status_=e.what();}
        }
    }
}
void SimulationView::nodePopup() {
    auto viewport=ImGui::GetMainViewport();
    ImGui::SetNextWindowSize(ImVec2(500,std::min(580.f,viewport->WorkSize.y-70)),ImGuiCond_Always);
    if(!ImGui::BeginPopup("Node settings")) return;
    if(selectedNode_<0 || selectedNode_>=static_cast<int>(graph_.nodes.size())) {ImGui::EndPopup();return;}
    auto& node=graph_.nodes[selectedNode_];
    ImGui::Text("Node %d",selectedNode_);ImGui::SameLine();ImGui::TextDisabled("%.2f, %.2f m",node.position.x,node.position.y);
    ImGui::SameLine(ImGui::GetWindowWidth()-70);if(ImGui::SmallButton("Close"))ImGui::CloseCurrentPopup();
    const char* tabs[]{"General","Roads","Lane exits","Signals","Junction"};
    if(ImGui::BeginTabBar("Node panels")) {
        for(int panel=0;panel<5;++panel) {
            auto flags=nodePopupTab_==panel?ImGuiTabItemFlags_SetSelected:ImGuiTabItemFlags_None;
            if(!ImGui::BeginTabItem(tabs[panel],nullptr,flags))continue;
            if(nodePopupTab_==panel)nodePopupTab_=-1;
            ImGui::BeginChild("Node content",ImVec2(0,0));
            auto content=[&]() {
                if(panel==2){
                    ImGui::TextWrapped("Edit one incoming lane at a time beside the map. Pick lane numbers and click the outgoing lanes to see exactly where traffic can go.");
                    if(ImGui::Button("Edit lane connections on map",ImVec2(-1,0))){
                        cancelEditorAction();laneExitEditorOpen_=true;turnDraftNode_=-1;turnDirty_=false;workspace_=1;running_=false;stepBatch_.cancel();ImGui::CloseCurrentPopup();
                    }
                    return;
                }
                if(panel==3){roadSignalPopup();return;}
                if(panel==0) {
                    ImGui::SeparatorText("Trip endpoints");
                    ImGui::Checkbox("Spawn vehicles here",&node.spawn);
                    if(node.spawn) {
                        typedDouble("Spawn rate (vehicles / s)",&node.rate,0,100);
                        if(ImGui::Checkbox("Exclude this node from its own trips",&node.excludeOwnDestination))simulation_.setSourceDestinationPolicy(selectedNode_,node.excludeOwnDestination);
                    }
                    bool destination=node.destination;
                    if(ImGui::Checkbox("Trip destination",&destination)) {auto proposed=graph_;proposed.useExplicitDestinations();proposed.nodes[selectedNode_].destination=destination;applyGraph(proposed);return;}
                    ImGui::Spacing();ImGui::SeparatorText("Turn behavior");
                    bool changed=ImGui::Checkbox("This node is a turn",&node.turn);
                    if(changed && node.turn && node.openRadius>12 && node.turnRadius==12)node.turnRadius=node.openRadius;
                    if(node.turn) {
                        float braking=static_cast<float>(node.braking),radius=static_cast<float>(node.turnRadius);
                        ImGui::TextUnformatted("Turn deceleration (m/s^2)");ImGui::SetNextItemWidth(-1);
                        if(ImGui::SliderFloat("##turn_braking",&braking,.1f,20,"%.2f")){node.braking=braking;changed=true;}
                        ImGui::TextUnformatted("Turn influence radius (m)");ImGui::SetNextItemWidth(-1);
                        if(ImGui::SliderFloat("##turn_radius",&radius,.5f,500,"%.1f")){node.turnRadius=radius;changed=true;}
                    }
                    if(changed){std::vector<TurnControl> controls;for(const auto& n:graph_.nodes)if(n.turn)controls.push_back({n.position,n.turnRadius,n.braking});simulation_.setTurnControls(std::move(controls));}
                }
                if(panel==1) {
                    ImGui::SeparatorText("Connect nodes");
                    ImGui::TextUnformatted("New connection type");ImGui::SetNextItemWidth(-1);
                    ImGui::Combo("##arm_direction",&newRoadType_,"Legacy centerline\0One-way road\0Two-way road\0");
                    ImGui::TextUnformatted("Lanes per direction");ImGui::SetNextItemWidth(-1);ImGui::InputInt("##arm_lanes",&newRoadLanes_);newRoadLanes_=std::clamp(newRoadLanes_,1,maximumRoadLanes);
                    if(ImGui::Button("Connect to another node",ImVec2(-1,0))){connectionFrom_=selectedNode_;severing_=false;editorMode_=0;ImGui::CloseCurrentPopup();}
                    if(ImGui::Button("Sever a connection",ImVec2(-1,0))){connectionFrom_=selectedNode_;severing_=true;editorMode_=0;ImGui::CloseCurrentPopup();}
                    if(ImGui::Button("Select path starting here",ImVec2(-1,0))){pathNodes_={selectedNode_};ImGui::CloseCurrentPopup();}
                    ImGui::Spacing();ImGui::SeparatorText("Connected roads");
                    for(std::size_t r=0;r<graph_.connections.size();++r) {
                        const auto& road=graph_.connections[r];if(road.from!=selectedNode_ && road.to!=selectedNode_)continue;
                        auto widths=shapeWidths(road);std::size_t point=road.from==selectedNode_?0:widths.size()-1;
                        ImGui::PushID(static_cast<int>(r));ImGui::Text("Road %zu: node %d -> %d%s",r,road.from,road.to,road.roadType==2?" (two-way)":"");
                        ImGui::TextUnformatted("Lane width at this node (m)");ImGui::SetNextItemWidth(-1);float width=static_cast<float>(widths[point]);
                        if(ImGui::SliderFloat("##endpoint_width",&width,.5f,15,"%.2f")){widths[point]=width;auto proposed=graph_;proposed.connections[r].shapeWidths=widths;applyGraph(proposed);ImGui::PopID();return;}
                        ImGui::PopID();
                    }
                }
                if(panel==4) {
                    ImGui::SeparatorText("Junction area");
                    float radius=static_cast<float>(node.openRadius);
                    if(typedFloat("Free junction radius (m), 0 = guided",&radius,0,500,"%.2f")){auto proposed=graph_;proposed.nodes[selectedNode_].openRadius=radius;applyGraph(proposed);return;}
                    if(node.openRadius==0) {
                        double extent=node.internalRadius;
                        if(typedDouble("Guided junction extent (m), 0 = auto",&extent,0,500,"%.2f")){auto proposed=graph_;proposed.nodes[selectedNode_].internalRadius=extent;applyGraph(proposed);return;}
                        bool uturn=node.allowUTurn;
                        if(ImGui::Checkbox("Allow U-turn connections",&uturn)){auto proposed=graph_;proposed.nodes[selectedNode_].allowUTurn=uturn;applyGraph(proposed);return;}
                    }
                }
            };
            content();ImGui::EndChild();ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }
    ImGui::EndPopup();
}
void SimulationView::canvas() {
    const auto& map=source_;
    ImVec2 size=ImGui::GetContentRegionAvail(); size.y=std::max(100.f,size.y-(showInspector_?230:24));
    ImGui::InvisibleButton("map",size);
    ImVec2 origin=ImGui::GetItemRectMin();
    auto* draw=ImGui::GetWindowDrawList();
    draw->AddRectFilled(origin,ImVec2(origin.x+size.x,origin.y+size.y),IM_COL32(16,21,29,255),8);
    const ImVec2 canvasMin=ImGui::GetItemRectMin(),canvasMax=ImGui::GetItemRectMax();
    const ImVec2 toolMin{canvasMin.x+12,canvasMax.y-52};
    const ImVec2 toolSize{std::max(120.f,std::min(390.f,size.x-24)),40};
    const ImVec2 toolMax{toolMin.x+toolSize.x,toolMin.y+toolSize.y};
    bool overToolBar=editorActionActive() && ImGui::IsMouseHoveringRect(toolMin,toolMax);
    bool canvasHovered=ImGui::IsItemHovered() && !overToolBar; const auto& io=ImGui::GetIO();
    if(canvasHovered && io.MouseWheel!=0) {
        float previous=mapZoom_; mapZoom_=std::clamp(mapZoom_*std::pow(1.2f,io.MouseWheel),.5f,40.f);
        double ratio=mapZoom_/previous,dx=io.MousePos.x-origin.x-size.x*.5,dy=io.MousePos.y-origin.y-size.y*.5;
        mapPan_={dx-(dx-mapPan_.x)*ratio,dy-(dy-mapPan_.y)*ratio};
    }
    if(canvasHovered && ImGui::IsMouseDragging(2)) mapPan_=mapPan_+Vec2{io.MouseDelta.x,io.MouseDelta.y};
    float scale=std::min((size.x-24)/map.width,(size.y-24)/map.height)*mapZoom_;
    origin.x+=(size.x-map.width*scale)/2+static_cast<float>(mapPan_.x); origin.y+=(size.y-map.height*scale)/2+static_cast<float>(mapPan_.y);
    auto screen=[&](Vec2 p) { return ImVec2(origin.x+static_cast<float>(p.x/map.metersPerPixel)*scale,origin.y+static_cast<float>(p.y/map.metersPerPixel)*scale); };
    draw->PushClipRect(ImGui::GetItemRectMin(),ImGui::GetItemRectMax(),true);
    // Combine same-color horizontal runs: large PNGs don't require one quad per pixel.
    for (int y=0;y<map.height;++y) for (int x=0;x<map.width;) {
        Tile tile=map.tiles[map.index({x,y})]; int end=x+1;
        while (end<map.width && map.tiles[map.index({end,y})]==tile) ++end;
        ImU32 color=tile==Tile::Wall?IM_COL32(29,38,49,255):tile==Tile::Spawn?IM_COL32(114,43,50,255):
                    tile==Tile::Destination?IM_COL32(45,199,115,255):tile==Tile::Signal?IM_COL32(65,121,250,255):
                    tile==Tile::Divider?IM_COL32(244,80,191,255):IM_COL32(114,43,50,255);
        draw->AddRectFilled(ImVec2(std::floor(origin.x+x*scale),std::floor(origin.y+y*scale)),ImVec2(std::ceil(origin.x+end*scale),std::ceil(origin.y+(y+1)*scale)),color);
        x=end;
    }
    auto drawLane=[&](const std::vector<Vec2>& points,ImU32 color) {
        for(std::size_t i=1;i<points.size();++i) {
            draw->AddLine(screen(points[i-1]),screen(points[i]),color,2);
            if(i%5==0 || points.size()<6) {
                auto a=points[i-1],b=points[i],direction=b-a; double n=length(direction);
                if(n>.01) { direction=direction*(1/n); auto tip=a+(b-a)*.65; auto base=tip-direction*.6; auto side=Vec2{-direction.y,direction.x}*.25;
                    draw->AddTriangleFilled(screen(tip),screen(base+side),screen(base-side),color); }
            }
        }
    };
    auto ribbon=[&](const std::vector<Vec2>& points,const std::vector<double>& widths,double width,ImU32 fill,ImU32 border) {
        for(std::size_t i=1;i<points.size();++i) {
            auto d=points[i]-points[i-1]; double n=length(d); if(n<1e-8) continue;
            Vec2 side{-d.y/n,d.x/n}; double a=widths.size()==points.size()?widths[i-1]:width,b=widths.size()==points.size()?widths[i]:width;
            auto p=points[i-1],q=points[i]; ImVec2 polygon[]{screen(p+side*(a*.5)),screen(q+side*(b*.5)),screen(q-side*(b*.5)),screen(p-side*(a*.5))};
            draw->AddConvexPolyFilled(polygon,4,fill); draw->AddLine(polygon[0],polygon[1],border,1); draw->AddLine(polygon[3],polygon[2],border,1);
        }
    };
    for(const auto& node:graph_.nodes) if(node.openRadius>0) {
        draw->AddCircle(screen(node.position),static_cast<float>(node.openRadius/map.metersPerPixel*scale),IM_COL32(255,190,90,150),100,2);
        if(workspace_==1){auto label=screen(node.position);draw->AddText(ImVec2(label.x+8,label.y+8),IM_COL32(255,225,170,255),"Free junction");}
    }
    for(const auto& lane:lanes_) if(lane.openJunction<0 && (!laneExitEditorOpen_ || lane.roadGroup>=0)) {
        ribbon(lane.points,lane.pointWidths,lane.width,lane.travelDirection?IM_COL32(140,180,255,60):IM_COL32(80,225,180,60),IM_COL32(180,230,235,190));
        drawLane(lane.points,IM_COL32(180,225,245,130));
        if(laneExitEditorOpen_ && workspace_==1 && turnRoad_>=0 && selectedNode_>=0 && turnDraftNode_==selectedNode_ && lane.roadGroup==turnRoad_ && lane.travelDirection==turnDirection_ && lane.roadLane==turnLane_-1){
            ribbon(lane.points,lane.pointWidths,lane.width,IM_COL32(255,210,70,100),IM_COL32(255,225,100,255));drawLane(lane.points,IM_COL32(255,230,100,255));
        }
        for(const auto& rule:lane.changeSections) {
            double at=0;
            for(std::size_t i=1;i<lane.points.size();++i) {
                auto d=lane.points[i]-lane.points[i-1]; double span=length(d),a=std::max(at,rule.begin),b=std::min(at+span,rule.end);
                if(b>a && span>1e-8 && (!rule.allowEnter || !rule.allowExit)) draw->AddLine(screen(lane.points[i-1]+d*((a-at)/span)),screen(lane.points[i-1]+d*((b-at)/span)),IM_COL32(255,120,80,210),4);
                at+=span;
            }
        }
    }
    for(const auto& edge:graph_.connections) {
        bool chosen=false;
        for(std::size_t i=1;i<pathNodes_.size();++i) if((pathNodes_[i-1]==edge.from && pathNodes_[i]==edge.to) || (edge.roadType==2 && pathNodes_[i-1]==edge.to && pathNodes_[i]==edge.from)) chosen=true;
        auto shape=edge.shape.empty()?std::vector<Vec2>{graph_.nodes[edge.from].position,graph_.nodes[edge.to].position}:edge.shape;
        if(chosen || edge.roadType==0 || splittingRoad_ || placingRoadSignal_ || workspace_==1) drawLane(shape,chosen?IM_COL32(255,220,80,255):IM_COL32(220,220,220,170));
    }
    for(std::size_t n=0;n<graph_.nodes.size();++n) {
        const auto& node=graph_.nodes[n]; bool chosen=std::find(pathNodes_.begin(),pathNodes_.end(),static_cast<int>(n))!=pathNodes_.end();
        auto pos=screen(node.position);
        auto color=chosen?IM_COL32(255,220,80,255):node.spawn?IM_COL32(90,255,140,255):node.destination?IM_COL32(255,160,110,255):IM_COL32(130,220,255,255);
        draw->AddCircleFilled(pos,selectedNode_==static_cast<int>(n)?6.f:3.f,color);
        if(laneExitEditorOpen_ && workspace_==1 && selectedNode_>=0 && turnDraftNode_==selectedNode_ && std::find(highlightedExitNodes_.begin(),highlightedExitNodes_.end(),static_cast<int>(n))!=highlightedExitNodes_.end())draw->AddCircle(pos,11,IM_COL32(80,235,205,255),0,3);
        if(node.roadStation)draw->AddCircle(pos,7,IM_COL32(120,230,255,230),0,2);
        {
            // Node ID badges are drawn after signals and vehicles so they stay legible.
        }
    }
    bool hovered=canvasHovered;
    auto mouse=ImGui::GetIO().MousePos;
    Vec2 cursor{(mouse.x-origin.x)/scale*map.metersPerPixel,(mouse.y-origin.y)/scale*map.metersPerPixel};
    double snap=10*map.metersPerPixel/scale; int hitNode=-1;
    for(std::size_t n=0;n<graph_.nodes.size();++n) if(length(cursor-graph_.nodes[n].position)<snap) { snap=length(cursor-graph_.nodes[n].position); hitNode=static_cast<int>(n); }
    if(laneExitEditorOpen_ && workspace_==1) {
        const DirectedLane* source=nullptr;const DirectedLane* target=nullptr;
        double nearest=9*map.metersPerPixel/scale;
        for(const auto& lane:lanes_)if(lane.roadGroup>=0 && lane.openJunction<0 && lane.internalJunction<0 && !lane.points.empty()) {
            if(lane.roadGroup==turnRoad_ && lane.travelDirection==turnDirection_ && lane.roadLane==turnLane_-1)source=&lane;
            for(std::size_t point=1;point<lane.points.size();++point){
                auto a=lane.points[point-1],d=lane.points[point]-a;double span=d.x*d.x+d.y*d.y;
                double t=span>0?std::clamp(((cursor.x-a.x)*d.x+(cursor.y-a.y)*d.y)/span,0.,1.):0;
                double gap=length(cursor-(a+d*t));if(hovered && gap<nearest){nearest=gap;target=&lane;}
            }
        }
        if(hovered && target && ImGui::IsMouseClicked(0))toggleLaneExit(target->roadGroup,target->travelDirection,target->roadLane+1);
        for(const auto& lane:lanes_)if(lane.roadGroup>=0 && lane.openJunction<0 && lane.internalJunction<0 && !lane.points.empty()){
            bool chosen=false;
            for(const auto& draft:turnExits_)if(draft.road==lane.roadGroup && draft.direction==lane.travelDirection){
                auto picked=parseNumberList(draft.lanes.data(),1,graph_.directionalLaneCount(draft.road,draft.direction));chosen=std::find(picked.begin(),picked.end(),lane.roadLane+1)!=picked.end();break;
            }
            if(chosen || &lane==target){
                auto color=chosen?IM_COL32(60,240,200,255):IM_COL32(190,245,255,255);
                ribbon(lane.points,lane.pointWidths,lane.width,chosen?IM_COL32(60,240,200,90):IM_COL32(190,245,255,70),color);drawLane(lane.points,color);
                auto at=screen(lane.points[(lane.points.size()-1)/4]);auto text="To lane "+std::to_string(lane.roadLane+1)+" (N"+std::to_string(lane.sourceNode)+" -> N"+std::to_string(lane.destinationNode)+")";
                auto extent=ImGui::CalcTextSize(text.c_str());draw->AddRectFilled(ImVec2(at.x-3,at.y-19),ImVec2(at.x+extent.x+5,at.y),IM_COL32(8,16,27,245),3);draw->AddText(ImVec2(at.x,at.y-18),color,text.c_str());
                if(chosen && source){
                    auto a=screen(source->points.back()),b=screen(lane.points.front());
                    Vec2 incoming=source->points.size()>1?source->points.back()-source->points[source->points.size()-2]:Vec2{1,0};
                    Vec2 outgoing=lane.points.size()>1?lane.points[1]-lane.points.front():Vec2{1,0};
                    double reach=length(lane.points.front()-source->points.back())*.35;
                    auto c=screen(source->points.back()+incoming*(reach/std::max(.001,length(incoming))));
                    auto d=screen(lane.points.front()-outgoing*(reach/std::max(.001,length(outgoing))));
                    draw->AddBezierCubic(a,c,d,b,color,3);
                    Vec2 unit=outgoing*(1/std::max(.001,length(outgoing)));auto tip=lane.points.front();
                    double arrow=8*map.metersPerPixel/scale;auto base=tip-unit*arrow;Vec2 side{-unit.y*arrow*.5,unit.x*arrow*.5};
                    draw->AddTriangleFilled(screen(tip),screen(base+side),screen(base-side),color);
                }
            }
        }
        if(source){auto at=screen(source->points[(source->points.size()-1)*2/3]);auto text="From lane "+std::to_string(turnLane_);auto extent=ImGui::CalcTextSize(text.c_str());draw->AddRectFilled(ImVec2(at.x,at.y+5),ImVec2(at.x+extent.x+8,at.y+25),IM_COL32(8,16,27,245),3);draw->AddText(ImVec2(at.x+4,at.y+7),IM_COL32(255,220,80,255),text.c_str());}
        if(target && hovered)ImGui::SetTooltip("%s N%d -> N%d, lane %d", "Click to toggle:",target->sourceNode,target->destinationNode,target->roadLane+1);
        hovered=false; // Lane picking owns map clicks; keep normal node and signal editors closed.
    } else if(splittingRoad_ || placingRoadSignal_) {
        int nearest=-1; double gap=10*map.metersPerPixel/scale; Vec2 point;
        for(std::size_t r=0;r<graph_.connections.size();++r) {
            const auto& road=graph_.connections[r]; auto shape=road.shape.empty()?std::vector<Vec2>{graph_.nodes[road.from].position,graph_.nodes[road.to].position}:road.shape;
            for(std::size_t i=1;i<shape.size();++i) { auto d=shape[i]-shape[i-1];double t=std::clamp(((cursor.x-shape[i-1].x)*d.x+(cursor.y-shape[i-1].y)*d.y)/std::max(1e-9,d.x*d.x+d.y*d.y),0.,1.);auto q=shape[i-1]+d*t;if(length(q-cursor)<gap){gap=length(q-cursor);point=q;nearest=static_cast<int>(r);} }
        }
        if(hovered && nearest>=0) {
            draw->AddCircleFilled(screen(point),6,IM_COL32(255,220,80,255));
            if(ImGui::IsMouseClicked(0)) {try {auto proposed=graph_;int node=proposed.splitRoad(nearest,point);
                if(placingRoadSignal_){int degree=0;for(auto& road:proposed.connections)degree+=(road.from==node || road.to==node);if(degree==2)proposed.nodes[node].roadStation=true;}
                bool signalStation=placingRoadSignal_;
                if(applyGraph(proposed)){selectedNode_=node;nodePopupTab_=signalStation?3:4;turnRoad_=-1;highlightedExitNodes_.clear();ImGui::OpenPopup("Node settings");}
            }catch(const std::exception& error){status_=error.what();}}
        }
    } else if(drawingRoad_) {
        std::vector<double> previewWidths=roadDraftWidths_; double count=newRoadLanes_*(newRoadType_==2?2:1);
        for(auto& width:previewWidths) width*=count;
        ribbon(roadDraft_,previewWidths,laneWidth_*count,IM_COL32(255,220,80,60),IM_COL32(255,220,80,180));
        drawLane(roadDraft_,IM_COL32(255,220,80,255));
        if(hovered) {
            auto target=hitNode>=0?graph_.nodes[hitNode].position:cursor;
            if(!roadDraft_.empty()) {
                ribbon({roadDraft_.back(),target},{roadDraftWidths_.back()*count,nextPointWidth_*count},laneWidth_*count,IM_COL32(255,220,80,60),IM_COL32(255,220,80,180));
                draw->AddLine(screen(roadDraft_.back()),screen(target),IM_COL32(255,220,80,255),2);
            }
            if(ImGui::IsMouseClicked(0) && map.contains(map.pixel(target)) && map.walkable(map.pixel(target)) && (roadDraft_.empty() || length(target-roadDraft_.back())>.05)) { roadDraft_.push_back(target); roadDraftWidths_.push_back(nextPointWidth_); }
        }
    } else if((editorMode_==0 || editorMode_==3) && hovered) {
        if(connectionFrom_>=0 && connectionFrom_<static_cast<int>(graph_.nodes.size())) draw->AddLine(screen(graph_.nodes[connectionFrom_].position),hitNode>=0?screen(graph_.nodes[hitNode].position):screen(cursor),IM_COL32(255,220,80,255),2);
        if(ImGui::IsMouseClicked(0)) {
            if(hitNode>=0) {
                if(connectionFrom_>=0) {
                    auto proposed=graph_;
                    try {
                        if(severing_) { if(proposed.pathConnection(connectionFrom_,hitNode)<0) throw std::invalid_argument("No directed connection to sever"); proposed.sever(connectionFrom_,hitNode); }
                        else {
                            if(proposed.pathConnection(connectionFrom_,hitNode)>=0) throw std::invalid_argument("These nodes are already connected");
                            proposed.useExplicitDestinations();
                            proposed.connect(connectionFrom_,hitNode,laneWidth_,laneSpeed_);
                            int edge=proposed.connection(connectionFrom_,hitNode);
                            proposed.connections[edge].roadType=newRoadType_; proposed.connections[edge].laneCount=newRoadLanes_;
                        }
                        if(applyGraph(proposed)) { connectionFrom_=-1; pathNodes_.clear(); }
                    } catch(const std::exception& e) { status_=e.what(); }
                } else if(ImGui::GetIO().KeyCtrl) {
                    if(!graph_.extendSelection(pathNodes_,hitNode)) status_="Ctrl-click a directly connected next node in travel order, or clear the selected path.";
                    selectedNode_=hitNode;
                } else { if(selectedNode_!=hitNode){turnRoad_=-1;highlightedExitNodes_.clear();}selectedNode_=hitNode; selected_=0; ImGui::OpenPopup("Node settings"); }
            } else if(editorMode_==3 && map.contains(map.pixel(cursor)) && map.walkable(map.pixel(cursor))) {
                auto proposed=graph_; int node=proposed.addNode(cursor);
                if(applyGraph(proposed)) { editorMode_=3;selectedNode_=node; ImGui::OpenPopup("Node settings"); }
            }
        }
    } else if(editorMode_!=0) {
        if(editorMode_!=5 && editorMode_!=7 && editorMode_!=8) running_=false;
        if(hovered && map.contains(map.pixel(cursor))) {
            if((editorMode_==1 || editorMode_==2) && ImGui::IsMouseDown(0)) {
                source_.paintStroke(strokeActive_?lastStroke_:cursor,cursor,editorMode_==1?Tile::Road:Tile::Wall,brushWidth_);
                lastStroke_=cursor; strokeActive_=true; mapDirty_=true;
            } else if(editorMode_>=4 && ImGui::IsMouseClicked(0)) {
                Vec2 point=source_.center(source_.pixel(cursor));
                if(!linePending_) { lineStart_=point; linePending_=true; }
                else {
                    if(editorMode_==8) {
                        linePending_=false;
                        try {
                            if(linkedLight_<0 || linkedLight_>=static_cast<int>(lights_.size())) throw std::runtime_error("Select a traffic light first");
                            auto next=lights_; next[linkedLight_].queueDetectors.push_back({lineStart_,point});
                            simulation_.setTrafficLights(next); lights_=simulation_.trafficLights();
                            selectedLight_=linkedLight_; editorMode_=0; cancelPendingLine();
                            status_="Queue detector added. Draw another line to track farther upstream.";
                        } catch(const std::exception& error) { status_=error.what(); }
                    } else if(editorMode_==7) {
                        linePending_=false;
                        if(linkedStage_==0) {
                            if(length(point-lineStart_)<.1) status_="Stop-line endpoints must be at least 0.1 m apart.";
                            else { pendingStopA_=lineStart_; pendingStopB_=point; linkedStage_=1; status_="Now draw both ends of this light's allowed exit."; }
                        } else {
                            try {
                                if(linkedLight_>=0 && linkedLight_<static_cast<int>(lights_.size())) {
                                    auto next=lights_; auto& light=next[linkedLight_];
                                    light.permissive=true; light.allowedA=lineStart_; light.allowedB=point;
                                    simulation_.setTrafficLights(next); lights_=simulation_.trafficLights(); selectedLight_=linkedLight_;
                                } else addLight(pendingStopA_,pendingStopB_,true,lineStart_,point);
                                bool editedExisting=linkedLight_>=0;
                                cancelPendingLine(); if(editedExisting) editorMode_=0;
                                status_="Allowed exit linked to its traffic light. Qualifying routes may pass on any color and continue their trip.";
                            } catch(const std::exception& error) { status_=error.what(); }
                        }
                    } else {
                    Tile type=editorMode_==5?Tile::Signal:Tile::Divider;
                    linePending_=false;
                    if(type==Tile::Signal) {
                        try { addLight(lineStart_,point); }
                        catch(const std::exception& error) { status_=error.what(); }
                    } else {
                        source_.paintStroke(lineStart_,point,type,source_.metersPerPixel*.6);
                        refreshMap(true); status_="Line added. Save PNG to keep the map.";
                    }
                    }
                }
            }
        } else if(ImGui::IsMouseDown(0)) strokeActive_=false;
        if(mapDirty_ && ImGui::IsMouseReleased(0)) { strokeActive_=false; refreshMap(true); status_="Map stroke applied."; }
        if(linePending_) draw->AddLine(screen(lineStart_),screen(cursor),IM_COL32(255,255,255,200),2);
        if(editorMode_==7 && linkedStage_==1) draw->AddLine(screen(pendingStopA_),screen(pendingStopB_),IM_COL32(255,190,70,255),4);
    }
    if(placeLight_ && hovered && ImGui::IsMouseClicked(0)) {
        auto mouse=ImGui::GetIO().MousePos;
        Vec2 c{(mouse.x-origin.x)/scale*map.metersPerPixel,(mouse.y-origin.y)/scale*map.metersPerPixel};
        Vec2 half=verticalLine_?Vec2{0,stopLineLength_/2}:Vec2{stopLineLength_/2,0};
        auto clampPoint=[&](Vec2 p) { return Vec2{std::clamp(p.x,.01,map.width*map.metersPerPixel-.01),std::clamp(p.y,.01,map.height*map.metersPerPixel-.01)}; };
        if(map.contains(map.pixel(c))) {
            try { addLight(clampPoint(c-half),clampPoint(c+half)); }
            catch(const std::exception& error) { status_=error.what(); }
        }
    }
    for(std::size_t i=0;i<simulation_.trafficLights().size();++i) {
        const auto& light=simulation_.trafficLights()[i];
        auto phase=light.phase(simulation_.seconds());
        ImU32 color=!light.enabled?IM_COL32(150,150,150,255):phase==LightPhase::Green?IM_COL32(72,230,150,255):phase==LightPhase::Yellow?IM_COL32(255,208,65,255):IM_COL32(250,85,90,255);
        draw->AddLine(screen(light.a),screen(light.b),color,3);
        draw->AddCircleFilled(screen(light.a),6,color);
        if(hovered && editorMode_==0 && !placeLight_) {
            auto a=screen(light.a),b=screen(light.b);
            double dx=b.x-a.x,dy=b.y-a.y,norm=dx*dx+dy*dy;
            double t=norm>0?std::clamp(((mouse.x-a.x)*dx+(mouse.y-a.y)*dy)/norm,0.0,1.0):0;
            if(std::hypot(mouse.x-a.x-t*dx,mouse.y-a.y-t*dy)<8) {
                ImGui::SetTooltip("Light L%zu: click to select and edit timings",i);
                if(ImGui::IsMouseClicked(0)) selectedLight_=static_cast<int>(i);
            }
        }
        if(static_cast<int>(i)==selectedLight_) {
            draw->AddCircle(screen(light.a),9,IM_COL32(255,255,255,255),0,2);
            draw->AddCircle(screen(light.b),5,IM_COL32(255,255,255,255),0,2);
        }
        auto direction=light.controlledDirection();Vec2 side{-direction.y,direction.x};auto middle=(light.a+light.b)*.5;
        double displayUnit=map.metersPerPixel/std::max(.001f,scale);
        auto arrow=[&](Vec2 start,Vec2 end,ImU32 arrowColor){draw->AddLine(screen(start),screen(end),arrowColor,2);auto d=end-start;d=d*(1/std::max(1e-9,length(d)));auto base=end-d*(6*displayUnit);Vec2 cross{-d.y*3*displayUnit,d.x*3*displayUnit};draw->AddTriangleFilled(screen(end),screen(base+cross),screen(base-cross),arrowColor);};
        {
            auto offset=side*(16*displayUnit);
            arrow(middle-direction*(36*displayUnit)+offset,middle+direction*(36*displayUnit)+offset,color);
            arrow(middle+direction*(36*displayUnit)-offset,middle-direction*(36*displayUnit)-offset,IM_COL32(100,235,150,240));
            auto tag=[&](Vec2 point,const char* text,ImU32 tint){auto at=screen(point);auto size=ImGui::CalcTextSize(text);at.x-=size.x*.5;at.y-=size.y*.5;draw->AddRectFilled(ImVec2(at.x-3,at.y-2),ImVec2(at.x+size.x+3,at.y+size.y+2),IM_COL32(16,21,29,235),3);draw->AddText(at,tint,text);};
            tag(middle+offset,"OUT",color);tag(middle-offset,"IN",IM_COL32(100,235,150,255));
        }
        auto queue=simulation_.trafficLightQueueStatus(light.id);
        char label[48]; std::snprintf(label,sizeof(label),"L%zu | queue %d",i,queue.qualityLine);
        auto p=screen(light.a); draw->AddText(ImVec2(p.x+8,p.y-9),color,label);
        for(std::size_t j=0;j<light.queueDetectors.size();++j) {
            const auto& line=light.queueDetectors[j];
            bool occupied=j<queue.lines.size() && queue.lines[j].occupied;
            ImU32 detectorColor=occupied?IM_COL32(255,165,50,255):IM_COL32(175,190,205,220);
            draw->AddLine(screen(line.a),screen(line.b),detectorColor,static_cast<int>(i)==selectedLight_?3:2);
            auto point=screen(line.a); char text[64];
            std::snprintf(text,sizeof(text),"L%zu line %zu: %s",i,j+1,occupied?"YES":"NO");
            draw->AddText(ImVec2(point.x+5,point.y+5),detectorColor,text);
        }
        if(light.permissive) {
            ImU32 exitColor=IM_COL32(55,225,245,255);
            draw->AddLine(screen(light.allowedA),screen(light.allowedB),exitColor,3);
            draw->AddCircle(screen(light.allowedA),5,exitColor,0,2);
            auto exit=screen(light.allowedA); char exitLabel[48]; std::snprintf(exitLabel,sizeof(exitLabel),"L%zu allowed exit",i);
            draw->AddText(ImVec2(exit.x+7,exit.y-12),exitColor,exitLabel);
            auto a=screen((light.a+light.b)*.5),b=screen((light.allowedA+light.allowedB)*.5);
            draw->AddLine(a,b,IM_COL32(55,225,245,90),1);
        }
    }
    for (const auto& e:simulation_.entities()) {
        if(e.id==selected_ && e.signalLight>=0) draw->AddCircle(screen(e.signalTarget),5,IM_COL32(255,190,70,255),0,2);
        if (routes_ && e.id==selected_) {
            auto preview=sumo_ && !e.inOpenJunction?sumo_->routePreview(e.id):e.path;
            for(std::size_t i=1;i<preview.size();++i)
                draw->AddLine(screen(preview[i-1]),screen(preview[i]),IM_COL32(255,211,98,170),2);
        }
        ImVec2 p=screen(e.center);
        ImU32 colors[]{IM_COL32(150,225,145,255),IM_COL32(101,204,246,255),IM_COL32(177,171,255,255),IM_COL32(245,176,103,255),IM_COL32(244,140,177,255)};
        auto box=corners(e.footprint()); ImVec2 polygon[4];
        for(int i=0;i<4;++i) polygon[i]=screen(box[i]);
        draw->AddConvexPolyFilled(polygon,4,e.id==selected_?IM_COL32(255,212,105,255):colors[static_cast<int>(e.kind)]);
        draw->AddPolyline(polygon,4,e.id==selected_?IM_COL32(255,255,255,255):IM_COL32(20,25,35,255),e.id==selected_?2.f:1.f,ImDrawFlags_Closed);
        Vec2 nose=e.center+Vec2{std::cos(e.heading),std::sin(e.heading)}*(e.length*.4);
        draw->AddLine(p,screen(nose),IM_COL32(30,35,45,255),2);
        if(showClearance_ || e.id==selected_) {
            Vec2 forward{std::cos(e.heading),std::sin(e.heading)};
            auto front=e.center+forward*(e.length*.5);
            draw->AddLine(screen(front),screen(front+forward*(e.minGap+e.speed*e.tau)),IM_COL32(160,210,245,170),2);
        }
        auto mouse=ImGui::GetIO().MousePos;
        double mx=(mouse.x-p.x)*map.metersPerPixel/scale,my=(mouse.y-p.y)*map.metersPerPixel/scale;
        double along=mx*std::cos(e.heading)+my*std::sin(e.heading),across=-mx*std::sin(e.heading)+my*std::cos(e.heading);
        if (hovered && hitNode<0 && std::abs(along)<e.length/2+.15 && std::abs(across)<e.width/2+.15) {
            const char* state=e.inOpenJunction?"\nNative free-junction movement":sumo_?"\nSUMO lane / guided-junction movement":"\nNative lane / junction movement";
            ImGui::SetTooltip("%s %llu\nSpeed %.2f / %.2f m/s\nBody %.2f x %.2f m | %.0f kg\nStandstill gap %.2f m | headway %.2f s%s",vehicleName(e.kind),static_cast<unsigned long long>(e.id),e.speed,e.desiredSpeed,e.length,e.width,e.massKg,e.minGap,e.tau,state);
            if (editorMode_==0 && !placeLight_ && ImGui::IsMouseClicked(0)) {selected_=e.id;showInspector_=true;}
        }
    }
    // Fixed-size badges sit above all map content, with leaders back to nodes.
    std::vector<ImVec4> nodeBadgeBounds;
    for(std::size_t n=0;n<graph_.nodes.size();++n){const auto& node=graph_.nodes[n];auto point=screen(node.position);if(point.x<canvasMin.x || point.x>canvasMax.x || point.y<canvasMin.y || point.y>canvasMax.y)continue;
        bool signal=std::any_of(lights_.begin(),lights_.end(),[&](const auto& light){if(light.anchorNode==static_cast<int>(n))return true;auto mid=(light.a+light.b)*.5;return length(mid-node.position)<2*map.metersPerPixel;});
        auto text="N"+std::to_string(n);float font=ImGui::GetFontSize()+2;auto extent=ImGui::CalcTextSize(text.c_str());float width=extent.x*font/ImGui::GetFontSize()+16,height=font+10;
        ImVec2 at;for(int candidate=0;candidate<12;++candidate){float dx=signal?48.f:18.f,dy=signal?-58.f:-36.f;at={std::clamp(point.x+(candidate%2?-width-dx:dx),canvasMin.x+4,canvasMax.x-width-4),std::clamp(point.y+dy+(candidate/2)*34,canvasMin.y+4,canvasMax.y-height-4)};bool occupied=false;for(auto box:nodeBadgeBounds)if(at.x<box.z+5 && at.x+width>box.x-5 && at.y<box.w+5 && at.y+height>box.y-5)occupied=true;if(!occupied)break;}
        auto border=selectedNode_==static_cast<int>(n)?IM_COL32(255,215,70,255):IM_COL32(140,220,255,255);draw->AddLine(point,ImVec2(at.x+width*.5f,at.y+height*.5f),border,1.5f);
        draw->AddRectFilled(at,ImVec2(at.x+width,at.y+height),IM_COL32(8,16,27,250),5);draw->AddRect(at,ImVec2(at.x+width,at.y+height),border,5.f,2.f,0);
        draw->AddText(ImGui::GetFont(),font,ImVec2(at.x+8,at.y+5),IM_COL32(255,255,255,255),text.c_str());nodeBadgeBounds.push_back({at.x,at.y,at.x+width,at.y+height});
        if(canvasHovered && ImGui::IsMouseHoveringRect(at,ImVec2(at.x+width,at.y+height)))ImGui::SetTooltip("Node %zu%s%s%s",n,node.spawn?" | spawn":"",node.destination?" | destination":"",signal?" | traffic light":"");
    }
    draw->PopClipRect();
    nodePopup();
    if(editorActionActive()) {
        auto saved=ImGui::GetCursorScreenPos();ImGui::SetCursorScreenPos(toolMin);
        ImGui::PushStyleColor(ImGuiCol_ChildBg,ImVec4(.07f,.09f,.12f,.97f));
        ImGui::BeginChild("Active tool",toolSize,ImGuiChildFlags_Borders,ImGuiWindowFlags_NoScrollbar|ImGuiWindowFlags_NoScrollWithMouse);
        if(ImGui::Button("Cancel (Esc)"))cancelEditorAction();
        ImGui::SameLine();if(editorActionActive())ImGui::TextUnformatted(editorActionLabel());
        ImGui::EndChild();ImGui::PopStyleColor();ImGui::SetCursorScreenPos(saved);
    }
    ImGui::TextDisabled("%.1f x %.1f m | %.3f m/pixel",map.width*map.metersPerPixel,map.height*map.metersPerPixel,map.metersPerPixel);
}
void SimulationView::inspector() {
    ImGui::SeparatorText("Vehicle inspector");
    const Entity* selected=nullptr;
    for (const auto& e:simulation_.entities()) if (e.id==selected_) selected=&e;
    if (!selected) { ImGui::TextUnformatted("No active vehicle selected."); return; }
    ImGui::Text("%s %llu | %.2f m/s | %.2f x %.2f m | standstill gap %.2f m | headway %.2f s",vehicleName(selected->kind),static_cast<unsigned long long>(selected->id),selected->speed,selected->length,selected->width,selected->minGap,selected->tau);
    ImGui::TextDisabled("Controller: %s",selected->inOpenJunction?"native free junction":sumo_?"SUMO lane / guided junction":"native lane / junction");
    ImGui::Text("Stopped: total %.2f s | current %.2f s | longest %.2f s | trip %.2f s",
        selected->stoppedSeconds,selected->currentStopSeconds,selected->longestStopSeconds,selected->ageSeconds);
    if(selected->inOpenJunction) ImGui::Text("Stationary inside junction: %.2f s / %.2f s%s",selected->junctionStationarySeconds,stuckVehicleSettings_.junctionTimeoutSeconds,stuckVehicleSettings_.junctionEnabled?"":" (deletion disabled)");
    if(!selected->junctionStatus.empty()) ImGui::TextWrapped("Junction: %s",selected->junctionStatus.c_str());
    if(selected->internalJunction>=0) ImGui::Text("Guided junction connector: node %d",selected->internalJunction);
    if(selected->escapingGridlock) ImGui::TextColored(ImVec4(1,.65f,.2f,1),"Stale reservations released; physical contact remains enforced");
    ImGui::Text("Trip origin: lane %d | spawn node %d",selected->spawnLane,selected->spawnNode);
    ImGui::TextWrapped("Lane change: %s | completed by this vehicle: %llu",selected->laneChangeStatus.c_str(),static_cast<unsigned long long>(selected->laneChanges));
    if(selected->inOpenJunction)ImGui::TextWrapped("Native free controller: %s",stopReasonName(simulation_.stopReason(*selected)));
    float speed=static_cast<float>(selected->desiredSpeed),accel=static_cast<float>(selected->acceleration);
    ImGui::SetNextItemWidth(230);
    bool changed=ImGui::SliderFloat("Desired speed",&speed,0.05f,50,"%.2f");
    ImGui::SameLine(); ImGui::SetNextItemWidth(230);
    changed|=ImGui::SliderFloat("Acceleration",&accel,0.05f,100,"%.2f");
    if (changed) simulation_.setEntityMotion(selected->id,speed,accel);
}
}
