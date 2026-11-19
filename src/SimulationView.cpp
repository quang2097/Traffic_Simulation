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
    if(!ImGui::InputFloat(label,value,0,0,format,ImGuiInputTextFlags_EnterReturnsTrue)) return false;
    *value=std::isfinite(*value)?std::clamp(*value,low,high):low;
    return true;
}
bool typedDouble(const char* label,double* value,double low,double high,const char* format="%.6f") {
    if(!ImGui::InputDouble(label,value,0,0,format,ImGuiInputTextFlags_EnterReturnsTrue)) return false;
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
SimulationView::SimulationView(const char* initialMap) : source_(makeDemoMap()),simulation_(source_,42) {
    profiles_=vietnameseCityProfiles(); urbanTraffic_.enabled=true;
    tugSettings_.enabled=true;
    turnNegotiation_.enabled=true;
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
void SimulationView::addLight(Vec2 a,Vec2 b,bool permissive,Vec2 allowedA,Vec2 allowedB) {
    auto next=lights_;
    next.push_back({a,b,10,2,12,0,true});
    next.back().permissive=permissive; next.back().allowedA=allowedA; next.back().allowedB=allowedB;
    simulation_.setTrafficLights(next);
    lights_=simulation_.trafficLights(); selectedLight_=static_cast<int>(lights_.size())-1;
    syncSignalMask();
    status_="Light added. Adjust its timings live in Traffic lights; click again to add another.";
}
void SimulationView::reset() {
    for(auto& tile:source_.tiles) if(tile==Tile::Destination) tile=Tile::Road;
    source_.rebuildFeatures();
    laneOriginNodes_.clear();
    if(std::any_of(graph_.connections.begin(),graph_.connections.end(),[](const auto& e){return e.roadType!=0;}) || std::any_of(graph_.nodes.begin(),graph_.nodes.end(),[](const auto& n){return n.openRadius>0;})) {
        auto compiled=compileRoads(graph_); lanes_=std::move(compiled.lanes); laneOriginNodes_=std::move(compiled.originNodes);
    } else { lanes_=graph_.lanes(); for(const auto& edge:graph_.connections) laneOriginNodes_.push_back(edge.from); }
    nodeSpawnClocks_.assign(graph_.nodes.size(),0); spawnNodeRandom_.seed(static_cast<unsigned>(seed_));
    simulation_=Simulation(source_,static_cast<std::uint32_t>(seed_));
    simulation_.setRoutingSettings(routing_);
    simulation_.setTrafficFlowSettings(flow_);
    simulation_.setStuckVehicleSettings(stuckVehicleSettings_);
    simulation_.setTrafficRecoverySettings(trafficRecoverySettings_);
    simulation_.setUrbanTrafficSettings(urbanTraffic_);
    simulation_.setTugSettings(tugSettings_);
    simulation_.setTurnNegotiationSettings(turnNegotiation_);
    simulation_.setCollisionWorkers(static_cast<unsigned>(collisionWorkers_));
    for(int i=0;i<vehicleKindCount;++i) simulation_.setProfile(static_cast<VehicleKind>(i),profiles_[i]);
    simulation_.setTrafficLights(lights_);
    lights_=simulation_.trafficLights();
    try { simulation_.setLanes(lanes_); }
    catch(const std::exception& e) { running_=false; status_=std::string("Lane network needs repair: ")+e.what(); }
    selected_=spawned_=0; elapsed_=accumulator_=0;
    stepBatch_.cancel();
    planningTimeToExclude_=0;
    cancelPendingLine(); splittingRoad_=false;
}
void SimulationView::advanceTick(double seconds) {
    double planningBefore=simulation_.totalSignalPlanningSeconds()+simulation_.totalTrafficRecoverySeconds();
    if(automatic_) {
        nodeSpawnClocks_.resize(graph_.nodes.size(),0);
        for(std::size_t n=0;n<graph_.nodes.size();++n) {
            const auto& node=graph_.nodes[n];
            if(!node.spawn || node.rate<=0) { nodeSpawnClocks_[n]=0; continue; }
            if(!consumeSpawnClock(nodeSpawnClocks_[n],seconds,node.rate)) continue;
            std::vector<int> outgoing;
            for(std::size_t e=0;e<laneOriginNodes_.size();++e) if(laneOriginNodes_[e]==static_cast<int>(n)) outgoing.push_back(static_cast<int>(e));
            if(!outgoing.empty() && simulation_.entities().size()<static_cast<std::size_t>(maximumVehicles_)) {
                int edge=outgoing[std::uniform_int_distribution<std::size_t>(0,outgoing.size()-1)(spawnNodeRandom_)];
                if(simulation_.spawnRandomOnLane(edge)) ++spawned_;
            }
        }
    }
    simulation_.tick(seconds); elapsed_=simulation_.seconds();
    planningTimeToExclude_+=simulation_.totalSignalPlanningSeconds()+simulation_.totalTrafficRecoverySeconds()-planningBefore;
}
void SimulationView::frame(double realSeconds) {
    realSeconds=std::max(0.0,realSeconds-planningTimeToExclude_); planningTimeToExclude_=0;
    auto* viewport=ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::Begin("Traffic Simulation",nullptr,ImGuiWindowFlags_NoDecoration|ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoSavedSettings|ImGuiWindowFlags_MenuBar);
    menuBar();
    if(editorMode_==8 && ImGui::IsKeyPressed(ImGuiKey_Escape)) { editorMode_=0; cancelPendingLine(); }
    const char* workspaces[]{"Simulation","Map design","Traffic lights","Vehicles","Files"};
    for(int i=0;i<5;++i) {
        if(i) ImGui::SameLine();
        if(ImGui::Selectable(workspaces[i],workspace_==i,0,ImVec2(i==2?150:110,24))) workspace_=i;
    }
    ImGui::SameLine(); ImGui::TextDisabled("Traffic Lab");
    ImGui::Separator();
    ImGui::BeginDisabled(clockMode_==1 || stepBatch_.active());
    if(ImGui::Button(running_?"Pause":"Play")) {
        bool start=!running_; if(start) { splittingRoad_=false; if(mapDirty_) refreshMap(true); editorMode_=0; cancelPendingLine(); strokeActive_=false; }
        running_=start; accumulator_=0;
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    ImGui::BeginDisabled(running_ || stepBatch_.active());
    if(ImGui::Button("Step")) { if(mapDirty_) refreshMap(true); editorMode_=0; cancelPendingLine(); strokeActive_=false; placeLight_=false; accumulator_=0; stepBatch_.request(manualStepSeconds_); }
    ImGui::EndDisabled(); ImGui::SameLine();
    if(ImGui::Button("Reset traffic")) reset();
    ImGui::SameLine(); if(ImGui::Button("Fit view")) { mapZoom_=1; mapPan_={}; }
    ImGui::SameLine(); ImGui::TextDisabled("%.1f s  |  %zu vehicles",elapsed_,simulation_.entities().size());
    ImGui::Separator();
    if(!status_.empty()) ImGui::TextWrapped("%s",status_.c_str());
    ImGui::BeginChild("Controls",ImVec2(370,0),ImGuiChildFlags_Borders);
    controls();
    ImGui::EndChild(); ImGui::SameLine();
    // Run a fixed 120 Hz clock, independent of rendering frame rate.
    constexpr double dt=1.0/120;
    if (running_ && clockMode_==0) accumulator_+=std::clamp(realSeconds,0.0,0.1)*timeScale_;
    while (accumulator_>=dt) {
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
    ImGui::BeginChild("Workspace",ImVec2(0,0));
    ImGui::Text("Open junction: %zu vehicles",simulation_.openJunctionVehicles());
    ImGui::Text("Deleted: %llu total | %llu stuck | %llu junction",
        static_cast<unsigned long long>(simulation_.removedStuckVehicles()),
        static_cast<unsigned long long>(simulation_.removedQueueBlockers()),
        static_cast<unsigned long long>(simulation_.removedJunctionVehicles()));
    if(ImGui::CollapsingHeader("Live statistics and diagnostics")) {
    ImGui::Text("%zu active | %llu completed | %.1fs | %llu gap replans",
        simulation_.entities().size(),static_cast<unsigned long long>(simulation_.completed()),elapsed_,static_cast<unsigned long long>(simulation_.replans()));
    ImGui::Text("Lane changes: %llu | candidate checks: %llu | gap requests: %llu",static_cast<unsigned long long>(simulation_.totalLaneChanges()),static_cast<unsigned long long>(simulation_.laneChangeAttempts()),static_cast<unsigned long long>(simulation_.cooperativeGapRequests()));
    auto inside=std::count_if(simulation_.entities().begin(),simulation_.entities().end(),[](const Entity& e){return e.inOpenJunction;});
    ImGui::Text("Junction entries: %llu | crossing now: %zu",static_cast<unsigned long long>(simulation_.junctionAdmissions()),static_cast<std::size_t>(inside));
    ImGui::TextDisabled("%llu yielding recoveries | Turn approaches reserved",static_cast<unsigned long long>(simulation_.recoveries()));
    ImGui::TextDisabled("Last deleted vehicle: %llu",static_cast<unsigned long long>(simulation_.lastRemovedStuckVehicle()));
    ImGui::TextDisabled("%llu local detours | %llu global searches | %llu shared-goal cache hits",
        static_cast<unsigned long long>(simulation_.localDetours()),static_cast<unsigned long long>(simulation_.globalPlans()),
        static_cast<unsigned long long>(simulation_.navigation().cacheHits()));
    auto stops=simulation_.stopStatistics(includeCompletedStops_);
    ImGui::Text("Stopped seconds / vehicle: mean (average) %.2f | min %.2f | max %.2f",
        stops.meanSeconds,stops.minSeconds,stops.maxSeconds);
    if(editorMode_==7) ImGui::TextColored(ImVec4(.3f,.9f,1,1),"Permissive light: click %s %s",linkedStage_==0?"stop-line":"allowed-exit",linePending_?"end":"start");
    if(editorMode_==8) ImGui::TextColored(ImVec4(1,.7f,.3f,1),"Queue detector for L%d: click %s (Esc cancels)",linkedLight_,linePending_?"end":"start");
    }
    canvas();
    inspector();
    ImGui::EndChild(); ImGui::End();
}
void SimulationView::menuBar() {
    bool newMap=false;
    if(ImGui::BeginMenuBar()) {
        if(ImGui::BeginMenu("File")) {
            if(ImGui::MenuItem("New map...")) newMap=true;
            if(ImGui::MenuItem("Open map PNG...")) fileAction(0);
            if(ImGui::MenuItem("Open node network...")) fileAction(1);
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
            ImGui::EndMenu();
        }
        if(ImGui::BeginMenu("Help")) {
            ImGui::TextUnformatted("Map: wheel to zoom, middle drag to pan.");
            ImGui::TextUnformatted("Click a node for properties. Ctrl-click connected nodes to select a path.");
            ImGui::TextUnformatted("Save PNG and node network separately; keep .signals companions together.");
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
            refreshMap(true); ImGui::CloseCurrentPopup();
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
void SimulationView::controls() {
    ImGui::PushItemWidth(170);
    const char* titles[]{"Simulation settings","Map design","Traffic lights","Vehicle settings","Files and map scale"};
    ImGui::SeparatorText(titles[workspace_]);
    if(workspace_==1 || workspace_==4) {
    if(workspace_==1 && ImGui::CollapsingHeader("Roads and nodes",ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::TextWrapped("Canvas: mouse wheel zoom, middle-button drag to pan.");
        if(ImGui::Button("Fit map view")) { mapZoom_=1; mapPan_={}; }
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
        ImGui::Combo("Road direction",&newRoadType_,"Legacy centerline\0One-way road\0Two-way road\0");
        ImGui::SliderInt("Lanes / direction",&newRoadLanes_,1,4);
        if(ImGui::Button("Insert junction node on road")) { splittingRoad_=true; drawingRoad_=false; editorMode_=0; running_=false; stepBatch_.cancel(); status_="Click an existing road centerline, then connect a side road to the inserted node."; }
        if(splittingRoad_ && ImGui::Button("Cancel junction insertion")) splittingRoad_=false;
        if(ImGui::Button("Sketch road")) { splittingRoad_=false; drawingRoad_=true; roadDraft_.clear(); roadDraftWidths_.clear(); nextPointWidth_=laneWidth_; editorMode_=0; connectionFrom_=-1; running_=false; stepBatch_.cancel(); }
        if(ImGui::Button("Draw individual lane")) { newRoadType_=1; newRoadLanes_=1; splittingRoad_=false; drawingRoad_=true; roadDraft_.clear(); roadDraftWidths_.clear(); nextPointWidth_=laneWidth_; editorMode_=0; connectionFrom_=-1; running_=false; stepBatch_.cancel(); }
        if(drawingRoad_) {
            ImGui::TextUnformatted("Width at next node (m)"); ImGui::SetNextItemWidth(-1);
            ImGui::SliderFloat("##next_node_width",&nextPointWidth_,.5f,15,"%.2f");
            for(std::size_t i=0;i<roadDraftWidths_.size();++i) {
                ImGui::PushID(static_cast<int>(i)); float width=static_cast<float>(roadDraftWidths_[i]);
                ImGui::Text("Sketch node %zu width (m)",i); ImGui::SetNextItemWidth(-1);
                if(ImGui::SliderFloat("##draft_width",&width,.5f,15,"%.2f")) roadDraftWidths_[i]=width;
                ImGui::PopID();
            }
            ImGui::TextWrapped("Click centerline points; snap the first/last to existing junction nodes. Choose direction and lanes above. No spawning is enabled automatically.");
            if(ImGui::Button("Finish road") && roadDraft_.size()>=2) {
                try {
                    auto proposed=graph_; proposed.useExplicitDestinations(); int a=proposed.addNode(roadDraft_.front()),b=proposed.addNode(roadDraft_.back());
                    if(proposed.pathConnection(a,b)>=0) throw std::invalid_argument("Those junctions are already connected; select that path to edit it.");
                    proposed.connect(a,b,laneWidth_,laneSpeed_); auto& road=proposed.connections.back(); road.roadType=newRoadType_; road.laneCount=newRoadLanes_; road.shape=roadDraft_; road.shapeWidths=roadDraftWidths_; road.width=roadDraftWidths_.front();
                    if(applyGraph(proposed)) { drawingRoad_=false; roadDraft_.clear(); roadDraftWidths_.clear(); }
                } catch(const std::exception& e) { status_=e.what(); }
            }
            if(ImGui::Button("Cancel road sketch")) { drawingRoad_=false; roadDraft_.clear(); roadDraftWidths_.clear(); }
        }
        ImGui::TextWrapped("Native C++ traffic: finite-width lanes, gap-checked lane changes and free-space junctions. No external simulator is required.");
        ImGui::Text("%zu junction nodes | %zu roads",graph_.nodes.size(),graph_.connections.size());
        ImGui::TextWrapped("Left-click a node for settings. Use Place nodes to add isolated nodes. Ctrl + left-click connected nodes in travel order to select a path.");
        if(ImGui::Button("Place nodes")) { editorMode_=3; running_=false; stepBatch_.cancel(); }
        ImGui::SameLine(); if(ImGui::Button("Inspect nodes")) { editorMode_=0; connectionFrom_=-1; }
        ImGui::TextUnformatted("New connection width (m)"); ImGui::SetNextItemWidth(-1);
        ImGui::SliderFloat("##connection_width",&laneWidth_,.5f,10,"%.1f");
        ImGui::TextUnformatted("New connection speed (m/s)"); ImGui::SetNextItemWidth(-1);
        ImGui::SliderFloat("##connection_speed",&laneSpeed_,.5f,30,"%.1f");
        if(connectionFrom_>=0) {
            ImGui::Text("%s from node %d: click target node",severing_?"Sever":"Connect",connectionFrom_);
            if(ImGui::Button("Cancel connection action")) connectionFrom_=-1;
        }
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
                ImGui::TextWrapped("Larger values round turns earlier with wider arcs, within the road and vehicle corridor. Changes reset traffic and affect turns leaving the selected connections.");
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
                changed|=ImGui::SliderInt("Selected lanes / dir",&lanes,1,4);
                auto edited=graph_.connections[edge];
                int reverse=edited.reverseLaneCount>0?edited.reverseLaneCount:edited.laneCount;
                if(roadType==2) changed|=ImGui::SliderInt("Opposing lanes",&reverse,1,4);
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
    if(workspace_==0) {
    if(!flow_.predictiveJunctions && ImGui::CollapsingHeader("Negotiated turns",ImGuiTreeNodeFlags_DefaultOpen)) {
        bool changed=ImGui::Checkbox("Yield to turning vehicles",&turnNegotiation_.enabled);
        float creep=static_cast<float>(turnNegotiation_.creepSpeed);
        changed|=ImGui::SliderFloat("Turn creep cap (m/s)",&creep,.2f,4,"%.1f");
        turnNegotiation_.creepSpeed=creep;
        if(changed) simulation_.setTurnNegotiationSettings(turnNegotiation_);
        ImGui::Text("Granted turns: %llu",static_cast<unsigned long long>(simulation_.negotiatedTurns()));
        ImGui::TextWrapped("Conflicting turns take turns using their swept road space. Approaching traffic yields; vehicles already inside keep clearing. The owner creeps forward with collision checks until its tail has passed. Blocked exits and abandoned claims cannot hold the road indefinitely.");
    }
    if(!flow_.predictiveJunctions && ImGui::CollapsingHeader("Gravitational tug physics",ImGuiTreeNodeFlags_DefaultOpen)) {
        if(!simulation_.lanes().empty()) ImGui::TextWrapped("Lane vehicles use constrained lane following. Free-road tugs, reversing and mask reroutes do not override lane directions.");
        bool changed=ImGui::Checkbox("Enable front-point tug",&tugSettings_.enabled);
        changed|=ImGui::Checkbox("Use force motion for every vehicle",&tugSettings_.alwaysOn);
        float strength=static_cast<float>(tugSettings_.strengthNewtons),damping=static_cast<float>(tugSettings_.damping),speed=static_cast<float>(tugSettings_.maximumSpeed),delay=static_cast<float>(tugSettings_.afterStopSeconds);
        changed|=ImGui::SliderFloat("Tug strength (N)",&strength,10,10000,"%.0f");
        ImGui::TextDisabled("Force reference: 200 kg");
        changed|=ImGui::SliderFloat("Tug damping (/s)",&damping,.1f,10,"%.1f");
        changed|=ImGui::SliderFloat("Tug speed cap (m/s)",&speed,.1f,10,"%.1f");
        changed|=ImGui::SliderFloat("Tug after standstill (s)",&delay,.1f,10,"%.2f");
        tugSettings_.strengthNewtons=strength; tugSettings_.damping=damping; tugSettings_.maximumSpeed=speed; tugSettings_.afterStopSeconds=delay;
        if(changed) {
            try { simulation_.setTugSettings(tugSettings_); }
            catch(const std::exception& e) { status_=e.what(); tugSettings_=simulation_.tugSettings(); }
        }
        ImGui::Text("Mass-weighted buffer contacts: %llu",static_cast<unsigned long long>(simulation_.tugContacts()));
        ImGui::TextWrapped("A force at the nose pulls along a rolling route target, producing momentum and torque. Other bodies and road edges repel it. Default: brief assistance after persistent unexplained stops; ordinary queues remain guided. All-vehicle mode keeps force guidance active. Physical gaps and red lights remain enforced.");
    }
    if(ImGui::CollapsingHeader("Urban traffic behavior",ImGuiTreeNodeFlags_DefaultOpen)) {
        bool changed=ImGui::Checkbox("Smooth mixed traffic",&urbanTraffic_.enabled);
        float headway=static_cast<float>(urbanTraffic_.motorbikeHeadway),prediction=static_cast<float>(urbanTraffic_.predictionSeconds),filter=static_cast<float>(urbanTraffic_.filterSpeed);
        changed|=ImGui::SliderFloat("Two-wheeler headway (s)",&headway,.2f,3,"%.2f");
        changed|=ImGui::SliderFloat("Conflict prediction (s)",&prediction,.2f,3,"%.1f");
        changed|=ImGui::SliderFloat("Filtering cap (m/s)",&filter,.5f,8,"%.1f");
        urbanTraffic_.motorbikeHeadway=headway; urbanTraffic_.predictionSeconds=prediction; urbanTraffic_.filterSpeed=filter;
        if(changed) simulation_.setUrbanTrafficSettings(urbanTraffic_);
        if(ImGui::Button("Vietnamese city preset (reset traffic)")) {
            profiles_=vietnameseCityProfiles(); urbanTraffic_=UrbanTrafficSettings{}; urbanTraffic_.enabled=true;
            routing_.localInterval=.35; routing_.movingRefreshInterval=1.2;
            trafficRecoverySettings_.stopSeconds=.6;
            reset(); auto shares=simulation_.spawnPercentages();
            for(int i=0;i<vehicleKindCount;++i) spawnMix_[i]=static_cast<float>(shares[i]);
            spawnMixDirty_=false;
        }
        ImGui::TextWrapped("Preset: 80%% motorbikes, 14%% cars, 3%% bikes, 2%% buses, 1%% trucks. Demonstration assumptions; tune to your street. Smooth following, bend braking and predictive crossing yields retain hard physical gaps and red-light rules.");
    }
    const char* clockModes[]{"Real time","Manual steps (on demand)"};
    if(ImGui::Combo("Clock mode",&clockMode_,clockModes,2)) {
        running_=false; accumulator_=0; stepBatch_.cancel();
    }
    ImGui::DragFloat("Seconds / step",&manualStepSeconds_,.05f,.01f,60,"%.3f",ImGuiSliderFlags_AlwaysClamp);
    if(stepBatch_.active()) {
        char progress[96]; std::snprintf(progress,sizeof(progress),"%.3f / %.3f simulation seconds",stepBatch_.completedSeconds(),stepBatch_.requestedSeconds());
        ImGui::ProgressBar(static_cast<float>(stepBatch_.progress()),ImVec2(-1,0),progress);
        if(ImGui::Button("Cancel remaining step")) stepBatch_.cancel();
    } else if(stepBatch_.requestedSeconds()>0) ImGui::TextDisabled("Last advance: %.3f simulation seconds",stepBatch_.requestedSeconds());
    if(clockMode_==1) ImGui::TextWrapped("Advance only when you click. Each step includes automatic spawning, lights and statistics. Larger durations require more computation; collision substeps stay small.");
    ImGui::BeginDisabled(clockMode_==1);
    ImGui::SliderFloat("Time scale",&timeScale_,0.1f,5.f,"%.1fx");
    ImGui::EndDisabled();
    ImGui::InputInt("Seed",&seed_);
    ImGui::TextDisabled("Seed is applied on reset.");
    ImGui::Checkbox("Automatic spawning",&automatic_);
    ImGui::SetNextItemWidth(80);
    if(ImGui::InputInt("Maximum active vehicles",&maximumVehicles_,0,0,ImGuiInputTextFlags_EnterReturnsTrue)) maximumVehicles_=std::clamp(maximumVehicles_,1,10000);
    ImGui::TextWrapped("Type settings and press Enter to commit. Spawning pauses at this limit; active vehicles are never removed by it.");
    ImGui::TextWrapped("Click a node to mark it as a spawn point and set its rate.");
    if (ImGui::Button("Spawn vehicle")) {
        std::vector<int> entries;
        for(std::size_t i=0;i<laneOriginNodes_.size();++i) if(laneOriginNodes_[i]>=0 && graph_.nodes[laneOriginNodes_[i]].spawn) entries.push_back(static_cast<int>(i));
        bool spawned=false;
        if(!entries.empty()) spawned=simulation_.spawnRandomOnLane(entries[std::uniform_int_distribution<std::size_t>(0,entries.size()-1)(spawnNodeRandom_)]);
        if(spawned) { ++spawned_; status_="Vehicle spawned at a marked node."; }
        else status_="Mark connected spawn and trip-destination nodes, or wait for a safe insertion gap.";
    }
    ImGui::TextWrapped("Routing: random reachable endpoint; shortest-distance route to the chosen endpoint.");
    }
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
        ImGui::TextWrapped("Enter all five percentages totaling 100, then Apply mix. Zero excludes a class. Shares apply to attempts; a vehicle that cannot fit may fail to spawn.");
    }
    }
    if(workspace_==0) {
    if(ImGui::CollapsingHeader("Performance and stopped time",ImGuiTreeNodeFlags_DefaultOpen)) {
        if(ImGui::SliderInt("Collision workers",&collisionWorkers_,1,static_cast<int>(Simulation::maximumCollisionWorkers())))
            simulation_.setCollisionWorkers(static_cast<unsigned>(collisionWorkers_));
        ImGui::Text("Parallel batches: %llu",static_cast<unsigned long long>(simulation_.parallelCollisionBatches()));
        bool flowChange=ImGui::Checkbox("Predictive junction controller",&flow_.predictiveJunctions);
        flowChange|=ImGui::Checkbox("Allow sublane packing",&flow_.sublanes);
        flowChange|=ImGui::Checkbox("Higher junction throughput",&flow_.compactAdmission);
        if(flowChange) simulation_.setTrafficFlowSettings(flow_);
        ImGui::TextWrapped("Predictive junctions use committed curves, exit-space checks and braking-aware following. Sublanes allow continuous movement into reachable side gaps. These replace tugging and RTS avoidance for native junction trips.");
        if(!flow_.predictiveJunctions) {
        bool recoveryChange=ImGui::Checkbox("Untangle stopped traffic",&trafficRecoverySettings_.enabled);
        float recoveryDelay=static_cast<float>(trafficRecoverySettings_.stopSeconds);
        recoveryChange|=ImGui::DragFloat("Untangle after (s)",&recoveryDelay,.1f,.1f,30,"%.1f",ImGuiSliderFlags_AlwaysClamp);
        trafficRecoverySettings_.stopSeconds=recoveryDelay;
        float recoveryRadius=static_cast<float>(trafficRecoverySettings_.radius);
        recoveryChange|=ImGui::DragFloat("Junction replan radius (m)",&recoveryRadius,1.f,5.f,250.f,"%.0f",ImGuiSliderFlags_AlwaysClamp);
        trafficRecoverySettings_.radius=recoveryRadius;
        if(recoveryChange) simulation_.setTrafficRecoverySettings(trafficRecoverySettings_);
        if(ImGui::Button("Untangle stopped traffic now")) {
            double before=simulation_.totalTrafficRecoverySeconds();
            simulation_.planStoppedTraffic(true);
            planningTimeToExclude_+=simulation_.totalTrafficRecoverySeconds()-before;
        }
        ImGui::Text("Untangling: %llu repairs / %llu passes (last %.1f ms)",
            static_cast<unsigned long long>(simulation_.trafficRecoveryRepairs()),
            static_cast<unsigned long long>(simulation_.trafficRecoveryPasses()),simulation_.lastTrafficRecoveryMilliseconds());
        ImGui::Text("Fleet routes mapped: %llu",static_cast<unsigned long long>(simulation_.lastFleetRouteCount()));
        ImGui::TextWrapped("Verifies stopped vehicles against signals and obstacles. Junction conflicts freeze simulation time while nearby vehicles replan and preview coordinated movement, including reverse yields. Working routes outside the radius remain in place.");
        }
        ImGui::TextWrapped("Read-only collision checks use workers at 32+ active vehicles. Movement commits stay ordered. One worker disables parallel checks.");
        bool change=ImGui::Checkbox("Remove stuck queue leaders",&stuckVehicleSettings_.enabled);
        float timeout=static_cast<float>(stuckVehicleSettings_.timeoutSeconds);
        change|=ImGui::DragFloat("Stuck timeout (s)",&timeout,1,1,3600,"%.0f",ImGuiSliderFlags_AlwaysClamp);
        stuckVehicleSettings_.timeoutSeconds=timeout;
        change|=ImGui::Checkbox("Delete stationary junction vehicles",&stuckVehicleSettings_.junctionEnabled);
        change|=typedDouble("Junction stationary timeout (s)",&stuckVehicleSettings_.junctionTimeoutSeconds,1,3600,"%.2f");
        if(change) simulation_.setStuckVehicleSettings(stuckVehicleSettings_);
        ImGui::TextWrapped("After the timeout, remove one stationary front blocker holding up a queue. Red/yellow waits are protected. Allow 2 seconds for traffic to resume before another removal.");
        ImGui::TextWrapped("The separate junction timeout deletes vehicles stationary at <= 0.05 m/s after entering the free junction, including blocked or signal waits there. Lane queues are excluded. Movement resets the timer. Pause adds no time; reset clears counters.");
        ImGui::Checkbox("Include finished / removed trips",&includeCompletedStops_);
        auto stats=simulation_.stopStatistics(includeCompletedStops_);
        ImGui::Text("Vehicles: %llu | Ever stopped: %llu",static_cast<unsigned long long>(stats.vehicles),static_cast<unsigned long long>(stats.vehiclesThatStopped));
        ImGui::Text("Mean / average: %.2f s",stats.meanSeconds);
        ImGui::Text("Min: %.2f s | Max: %.2f s",stats.minSeconds,stats.maxSeconds);
        ImGui::Text("Total across vehicles: %.2f s",stats.totalSeconds);
        ImGui::TextWrapped("Cumulative time per vehicle at <= 0.05 m/s, measured from actual movement. Includes red lights and queues. Uses simulation seconds; pause adds no time. Reset clears statistics.");
    }
    if(!flow_.predictiveJunctions && ImGui::CollapsingHeader("RTS-style routing",ImGuiTreeNodeFlags_DefaultOpen)) {
        bool changed=ImGui::Checkbox("Enable RTS routing",&routing_.rtsStyle);
        float look=static_cast<float>(routing_.lookAhead),interval=static_cast<float>(routing_.localInterval),weight=static_cast<float>(routing_.congestionWeight);
        changed|=ImGui::SliderFloat("Look ahead (m)",&look,2,30,"%.1f");
        changed|=ImGui::SliderFloat("Local update (s)",&interval,.1f,2,"%.2f");
        changed|=ImGui::SliderFloat("Avoid congestion",&weight,0,5,"%.1f");
        routing_.lookAhead=look; routing_.localInterval=interval; routing_.congestionWeight=weight;
        changed|=ImGui::Checkbox("Reissue destination while moving",&routing_.refreshMovingRoutes);
        float refresh=static_cast<float>(routing_.movingRefreshInterval);
        changed|=ImGui::DragFloat("Moving refresh (s)",&refresh,.25f,.25f,120,"%.2f",ImGuiSliderFlags_AlwaysClamp);
        routing_.movingRefreshInterval=refresh;
        if(changed) simulation_.setRoutingSettings(routing_);
        ImGui::Text("Destination refreshes: %llu / %llu attempts",static_cast<unsigned long long>(simulation_.destinationRefreshes()),static_cast<unsigned long long>(simulation_.destinationRefreshAttempts()));
        ImGui::TextWrapped("Periodically reissue the original endpoint after this much moving time. Stopped time does not count. Road turns and signal approaches finish first. Active bypasses recheck sooner and can return mid-turn when clear; position, heading and speed are preserved.");
        ImGui::TextWrapped("Detour around slow traffic when a gap fits, then rejoin the route. Follow moving queues and wait at signals.");
    }
    ImGui::Checkbox("Show selected route",&routes_);
    ImGui::Checkbox("Show preferred clearance",&showClearance_);
    ImGui::TextWrapped("Vehicles reserve turn approaches. Persistent standstills pause admission and trigger a checked reverse maneuver.");
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
    changed|=gaussianControl("Preferred gap",profile.distance,0,10);
    ImGui::Text("Applied spawn share: %.2f%%",simulation_.spawnPercentages()[selectedKind_]);
    if (changed) simulation_.setProfile(static_cast<VehicleKind>(selectedKind_),profile);
    ImGui::TextWrapped("New vehicles sample these distributions once at spawn. Zero standard deviation gives identical values.");
    if (ImGui::Button("Resample active vehicles")) simulation_.resampleMotion();
    ImGui::TextWrapped("Body sizes and preferred gap are sampled at spawn. Motion resampling affects all active classes. Pairwise gap = smaller preferred distance.");
    }
    }
    if(workspace_==2) {
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
    ImGui::TextWrapped("Signal queues are planned together before movement. Planning time is excluded from the running clock. Vehicles then advance together while keeping physical gaps.");
    if(ImGui::Checkbox("Place stop line on map",&placeLight_) && placeLight_) { editorMode_=0; cancelPendingLine(); }
    if(ImGui::Button("Draw permissive light + exit")) { editorMode_=7; placeLight_=false; cancelPendingLine(); }
    ImGui::TextWrapped("Draw the stop line, then its allowed exit across outgoing lanes. Vehicles whose route crosses that exit after this stop line may pass regardless of color and continue to their trip destination. Green PNG marks are no longer used.");
    ImGui::Checkbox("Vertical stop line",&verticalLine_);
    ImGui::SliderFloat("Line length (m)",&stopLineLength_,.5f,40,"%.1f");
    ImGui::TextWrapped("Each click adds an independent light without resetting traffic. Turn placement off when finished. You can also use the two-click Traffic-light line tool.");
    ImGui::Text("%zu independent lights",lights_.size());
    if(!lights_.empty()) {
        selectedLight_=std::clamp(selectedLight_,0,static_cast<int>(lights_.size())-1);
        std::string selectedLabel="L"+std::to_string(selectedLight_);
        if(ImGui::BeginCombo("Selected light",selectedLabel.c_str())) {
            for(int i=0;i<static_cast<int>(lights_.size());++i) {
                std::string label="L"+std::to_string(i);
                if(ImGui::Selectable(label.c_str(),selectedLight_==i)) selectedLight_=i;
            }
            ImGui::EndCombo();
        }
        auto& light=lights_[selectedLight_]; bool edit=ImGui::Checkbox("Enabled",&light.enabled);
        auto duration=[&](const char* label,double& v,double low,double high) { return typedDouble(label,&v,low,high,"%.3f"); };
        edit|=duration("Green (s)",light.greenSeconds,.1f,3600);
        edit|=duration("Yellow (s)",light.yellowSeconds,0,3600);
        edit|=duration("Red (s)",light.redSeconds,.1f,3600);
        edit|=duration("Offset (s)",light.offsetSeconds,0,10800);
        ImGui::SeparatorText("Queue detector lines");
        edit|=duration("Occupancy window (s)",light.queueWindowSeconds,.1,3600);
        edit|=duration("Occupied threshold (%)",light.queueThresholdPercent,0,100);
        if(edit) simulation_.setTrafficLights(lights_);
        auto queue=simulation_.trafficLightQueueStatus(light.id);
        ImGui::Text("Queue quality: line %d",queue.qualityLine);
        ImGui::TextWrapped("Draw lines nearest to farthest upstream. Quality is the highest line occupied for the threshold percentage of a full rolling window. Occupied means any vehicle body touches the line, moving or stopped. Detector lines do not stop or remove vehicles.");
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
                ImGui::Text("Line %zu: vehicle %s | %.1f%% | %.1f / %.1f s",i+1,found->occupied?"YES":"NO",found->occupiedPercent,found->observedSeconds,light.queueWindowSeconds);
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
        ImGui::TextDisabled("Detector edits restart that light's changed observation windows.");
        ImGui::TextWrapped("Type seconds and press Enter to apply while running. The cycle uses the current clock, so an edit may change the phase immediately.");
        auto phase=light.phase(simulation_.seconds());
        ImGui::Text("Phase: %s",phase==LightPhase::Green?"GREEN":phase==LightPhase::Yellow?"YELLOW":"RED");
        ImGui::Text("Type: %s",light.permissive?"Linked-exit permissive":"Normal");
        if(ImGui::Button(light.permissive?"Redraw allowed exit":"Set allowed exit for this light")) {
            editorMode_=7; placeLight_=false; cancelPendingLine();
            linkedStage_=1; linkedLight_=selectedLight_; pendingStopA_=light.a; pendingStopB_=light.b;
        }
        if(light.permissive && ImGui::Button("Remove red/yellow permission")) { light.permissive=false; simulation_.setTrafficLights(lights_); }
        if(ImGui::Button("Remove selected light")) {
            cancelPendingLine(); if(editorMode_==7 || editorMode_==8) editorMode_=0;
            lights_.erase(lights_.begin()+selectedLight_);
            simulation_.setTrafficLights(lights_); syncSignalMask();
            selectedLight_=std::max(0,selectedLight_-1);
            status_="Light removed; traffic and simulation clock preserved.";
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
    ImGui::TextWrapped("Red: road | Arrows: directed lanes | Blue: signals | Cyan: linked signal permissions | D nodes: trip destinations | Pink: solid divider. Legacy green marks are treated as road.");
    }
    if(workspace_==1) {
    ImGui::SeparatorText("Painting and markings");
    const char* tools[]{"Inspect / simulate","Paint roads (red)","Erase / block (black)","Place road nodes","Allowed exit (selected light)","Traffic-light line (blue)","Lane divider (pink)","Permissive light + exit"};
    if(ImGui::Combo("Edit tool",&editorMode_,tools,8)) {
        cancelPendingLine(); strokeActive_=false; placeLight_=false;
        if(editorMode_==4) {
            if(lights_.empty()) { editorMode_=0; status_="Add a traffic light before drawing its allowed exit."; }
            else { selectedLight_=std::clamp(selectedLight_,0,static_cast<int>(lights_.size())-1); editorMode_=7; linkedStage_=1; linkedLight_=selectedLight_; pendingStopA_=lights_[selectedLight_].a; pendingStopB_=lights_[selectedLight_].b; }
        }
        if(editorMode_!=0 && editorMode_!=5 && editorMode_!=7) { running_=false; accumulator_=0; stepBatch_.cancel(); }
    }
    ImGui::SliderFloat("Brush width (m)",&brushWidth_,.2f,20,"%.1f");
    ImGui::TextWrapped("Paint/erase: drag. Spawns: click a road. Lines: click both ends. Roads, spawns, exits and divider edits pause/reset traffic. Lights can be added and timed while running.");
    if((linePending_ || linkedStage_>0) && ImGui::Button("Cancel pending line")) cancelPendingLine();
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
        lights_.clear(); lanes_.clear(); graph_={}; pathNodes_.clear(); selectedNode_=-1; editorMode_=1; linePending_=false; refreshMap(true); status_="Blank map ready. Paint roads, then place and connect nodes.";
    }
    }
    if (!status_.empty()) { ImGui::Separator(); ImGui::TextWrapped("%s",status_.c_str()); }
    ImGui::PopItemWidth();
}
bool SimulationView::applyGraph(const NodeGraph& proposed) {
    try {
        for(const auto& node:proposed.nodes) if(!source_.contains(source_.pixel(node.position)) || !source_.walkable(source_.pixel(node.position))) throw std::invalid_argument("Nodes must lie on the road mask");
        Simulation check(source_,42); auto lanes=proposed.lanes();
        if(std::any_of(proposed.connections.begin(),proposed.connections.end(),[](const auto& e){return e.roadType!=0;})) validateRoadMask(lanes,source_);
        check.setLanes(std::move(lanes));
        graph_=proposed; lanes_=check.lanes(); running_=false; reset();
        status_="Node network applied; traffic reset."; return true;
    } catch(const std::exception& e) { status_=e.what(); return false; }
}
void SimulationView::nodePopup() {
    if(!ImGui::BeginPopup("Node settings")) return;
    if(selectedNode_>=0 && selectedNode_<static_cast<int>(graph_.nodes.size())) {
        auto& node=graph_.nodes[selectedNode_];
        ImGui::Text("Node %d (%.2f, %.2f m)",selectedNode_,node.position.x,node.position.y);
        ImGui::Checkbox("Spawn point",&node.spawn);
        if(node.spawn) {
            if(ImGui::Checkbox("Exclude this node from its own spawn trips",&node.excludeOwnDestination)) simulation_.setSourceDestinationPolicy(selectedNode_,node.excludeOwnDestination);
            ImGui::TextWrapped("Applies to new trips from this node only. Other origins may still finish here. Existing trips keep their destinations.");
        }
        ImGui::SeparatorText("Junction connections");
        int approaches=0; for(const auto& road:graph_.connections) if(road.from==selectedNode_ || road.to==selectedNode_) ++approaches;
        ImGui::Text("%d connected roads: %s",approaches,approaches==3?"T-junction":approaches==4?"4-way":"custom / merge / branch");
        bool uturn=node.allowUTurn;
        ImGui::BeginDisabled(node.openRadius>0);
        if(ImGui::Checkbox("Allow U-turn connections",&uturn)) { auto proposed=graph_; proposed.nodes[selectedNode_].allowUTurn=uturn; applyGraph(proposed); ImGui::EndDisabled(); ImGui::EndPopup(); return; }
        ImGui::EndDisabled();
        if(node.openRadius>0) ImGui::TextDisabled("Free junctions already allow U-turn routes.");
        ImGui::TextWrapped("One-way arms retain their direction; two-way arms connect both ways. Straight-through routes stay available. Connect a third road for a T-junction or a fourth for a crossroad. Paint room for the turns first.");
        ImGui::TextUnformatted("New arm direction");
        ImGui::Combo("##arm_direction",&newRoadType_,"Legacy centerline\0One-way road\0Two-way road\0");
        ImGui::SliderInt("New arm lanes / direction",&newRoadLanes_,1,4);
        ImGui::Separator();
        bool turnChanged=ImGui::Checkbox("This node is a turn",&node.turn);
        if(turnChanged && node.turn && node.openRadius>12 && node.turnRadius==12) node.turnRadius=node.openRadius;
        if(node.turn) {
            float braking=static_cast<float>(node.braking),radius=static_cast<float>(node.turnRadius);
            if(ImGui::SliderFloat("Turn deceleration (m/s^2)",&braking,.1f,20,"%.2f")) { node.braking=braking; turnChanged=true; }
            if(ImGui::SliderFloat("Turn influence radius (m)",&radius,.5f,500,"%.1f")) { node.turnRadius=radius; turnChanged=true; }
            if(!urbanTraffic_.enabled) ImGui::TextWrapped("Enable Smooth mixed traffic to apply turn braking.");
        } else ImGui::TextWrapped("Shape/width node: no turn slowdown. Mark this node as a turn to set its deceleration.");
        if(turnChanged) { std::vector<TurnControl> controls; for(const auto& item:graph_.nodes) if(item.turn) controls.push_back({item.position,item.turnRadius,item.braking}); simulation_.setTurnControls(std::move(controls)); }
        bool destination=node.destination;
        if(ImGui::Checkbox("Trip destination (vehicles finish here)",&destination)) {
            auto proposed=graph_; proposed.useExplicitDestinations(); proposed.nodes[selectedNode_].destination=destination;
            applyGraph(proposed); ImGui::EndPopup(); return;
        }
        ImGui::TextWrapped("Unmarked lane endpoints inside an open junction connect to its exits automatically; they do not finish a trip.");
        for(std::size_t r=0;r<graph_.connections.size();++r) {
            const auto& road=graph_.connections[r]; if(road.from!=selectedNode_ && road.to!=selectedNode_) continue;
            auto widths=shapeWidths(road); std::size_t point=road.from==selectedNode_?0:widths.size()-1;
            ImGui::PushID(static_cast<int>(r)); ImGui::Text("Road %zu width at this node (m)",r);
            float width=static_cast<float>(widths[point]);
            if(ImGui::SliderFloat("##endpoint_width",&width,.5f,15,"%.2f")) {
                widths[point]=width; auto proposed=graph_; proposed.connections[r].shapeWidths=widths;
                applyGraph(proposed); ImGui::PopID(); ImGui::EndPopup(); return;
            }
            ImGui::PopID();
        }
        float radius=static_cast<float>(node.openRadius);
        ImGui::TextUnformatted("Open junction radius (m), 0 = lane guided");
        if(typedFloat("##open_radius",&radius,0,500,"%.2f")) {
            auto proposed=graph_; proposed.nodes[selectedNode_].openRadius=radius;
            applyGraph(proposed);
            ImGui::EndPopup(); return;
        }
        ImGui::TextWrapped("Inside this boundary vehicles navigate the red mask freely toward their reserved exit. Road lanes end at the boundary. Changes reset traffic.");
        typedDouble("Vehicles / s",&node.rate,0,100);
        ImGui::TextWrapped("Spawn and braking settings apply live. A spawn node needs an outgoing path long enough to fit the vehicle.");
        if(ImGui::Button("Connect one-way arm...")) { newRoadType_=1; connectionFrom_=selectedNode_; severing_=false; editorMode_=0; ImGui::CloseCurrentPopup(); }
        if(ImGui::Button("Connect two-way arm...")) { newRoadType_=2; connectionFrom_=selectedNode_; severing_=false; editorMode_=0; ImGui::CloseCurrentPopup(); }
        if(ImGui::Button("Connect to node...")) { connectionFrom_=selectedNode_; severing_=false; editorMode_=0; ImGui::CloseCurrentPopup(); }
        if(ImGui::Button("Sever outgoing connection...")) { connectionFrom_=selectedNode_; severing_=true; editorMode_=0; ImGui::CloseCurrentPopup(); }
        ImGui::TextUnformatted("Outgoing connections:");
        for(const auto& edge:graph_.connections) { if(edge.from==selectedNode_) ImGui::BulletText("Node %d%s",edge.to,edge.roadType==2?" (two-way)":""); else if(edge.roadType==2 && edge.to==selectedNode_) ImGui::BulletText("Node %d (two-way)",edge.from); }
        if(ImGui::Button("Start path selection here")) { pathNodes_={selectedNode_}; ImGui::CloseCurrentPopup(); }
    }
    ImGui::EndPopup();
}
void SimulationView::canvas() {
    const auto& map=source_;
    ImVec2 size=ImGui::GetContentRegionAvail(); size.y=std::max(100.f,size.y-180);
    ImGui::InvisibleButton("map",size);
    ImVec2 origin=ImGui::GetItemRectMin();
    auto* draw=ImGui::GetWindowDrawList();
    draw->AddRectFilled(origin,ImVec2(origin.x+size.x,origin.y+size.y),IM_COL32(16,21,29,255),8);
    bool canvasHovered=ImGui::IsItemHovered(); const auto& io=ImGui::GetIO();
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
        auto label=screen(node.position); draw->AddText(ImVec2(label.x+8,label.y+8),IM_COL32(255,225,170,255),"OPEN JUNCTION - free navigation");
    }
    for(const auto& lane:lanes_) if(lane.openJunction<0) {
        ribbon(lane.points,lane.pointWidths,lane.width,lane.travelDirection?IM_COL32(140,180,255,60):IM_COL32(80,225,180,60),IM_COL32(180,230,235,190));
        drawLane(lane.points,IM_COL32(180,225,245,130));
    }
    for(const auto& edge:graph_.connections) {
        bool chosen=false;
        for(std::size_t i=1;i<pathNodes_.size();++i) if((pathNodes_[i-1]==edge.from && pathNodes_[i]==edge.to) || (edge.roadType==2 && pathNodes_[i-1]==edge.to && pathNodes_[i]==edge.from)) chosen=true;
        auto shape=edge.shape.empty()?std::vector<Vec2>{graph_.nodes[edge.from].position,graph_.nodes[edge.to].position}:edge.shape;
        if(chosen || edge.roadType==0 || splittingRoad_) drawLane(shape,chosen?IM_COL32(255,220,80,255):IM_COL32(220,220,220,170));
    }
    for(std::size_t n=0;n<graph_.nodes.size();++n) {
        const auto& node=graph_.nodes[n]; bool chosen=std::find(pathNodes_.begin(),pathNodes_.end(),static_cast<int>(n))!=pathNodes_.end();
        auto pos=screen(node.position);
        auto color=chosen?IM_COL32(255,220,80,255):node.spawn?IM_COL32(90,255,140,255):node.destination?IM_COL32(255,160,110,255):IM_COL32(130,220,255,255);
        draw->AddCircleFilled(pos,selectedNode_==static_cast<int>(n)?6.f:3.f,color);
        if(chosen || node.spawn || node.destination || node.turn || selectedNode_==static_cast<int>(n)) {
            auto label=std::to_string(n)+(node.spawn?" S":"")+(node.destination?" D":"")+(node.turn?" T":""); draw->AddText(ImVec2(pos.x+6,pos.y-13),color,label.c_str());
        }
    }
    bool hovered=ImGui::IsItemHovered();
    auto mouse=ImGui::GetIO().MousePos;
    Vec2 cursor{(mouse.x-origin.x)/scale*map.metersPerPixel,(mouse.y-origin.y)/scale*map.metersPerPixel};
    double snap=10*map.metersPerPixel/scale; int hitNode=-1;
    for(std::size_t n=0;n<graph_.nodes.size();++n) if(length(cursor-graph_.nodes[n].position)<snap) { snap=length(cursor-graph_.nodes[n].position); hitNode=static_cast<int>(n); }
    if(splittingRoad_) {
        int nearest=-1; double gap=10*map.metersPerPixel/scale; Vec2 point;
        for(std::size_t r=0;r<graph_.connections.size();++r) {
            const auto& road=graph_.connections[r]; auto shape=road.shape.empty()?std::vector<Vec2>{graph_.nodes[road.from].position,graph_.nodes[road.to].position}:road.shape;
            for(std::size_t i=1;i<shape.size();++i) { auto d=shape[i]-shape[i-1];double t=std::clamp(((cursor.x-shape[i-1].x)*d.x+(cursor.y-shape[i-1].y)*d.y)/std::max(1e-9,d.x*d.x+d.y*d.y),0.,1.);auto q=shape[i-1]+d*t;if(length(q-cursor)<gap){gap=length(q-cursor);point=q;nearest=static_cast<int>(r);} }
        }
        if(hovered && nearest>=0) {
            draw->AddCircleFilled(screen(point),6,IM_COL32(255,220,80,255));
            if(ImGui::IsMouseClicked(0)) { try { auto proposed=graph_; int node=proposed.splitRoad(nearest,point); if(applyGraph(proposed)){selectedNode_=node;splittingRoad_=false;ImGui::OpenPopup("Node settings");} }catch(const std::exception& error){status_=error.what();} }
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
                } else { selectedNode_=hitNode; selected_=0; ImGui::OpenPopup("Node settings"); }
            } else if(editorMode_==3 && map.contains(map.pixel(cursor)) && map.walkable(map.pixel(cursor))) {
                auto proposed=graph_; int node=proposed.addNode(cursor);
                if(applyGraph(proposed)) { selectedNode_=node; ImGui::OpenPopup("Node settings"); }
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
            for (std::size_t i=1;i<e.path.size();++i)
                draw->AddLine(screen(e.path[i-1]),screen(e.path[i]),IM_COL32(255,211,98,170),2);
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
            ImVec2 buffer[36]; int n=0;
            for(int corner=0;corner<4;++corner) for(int arc=0;arc<=8;++arc) {
                double a=e.heading+(corner+arc/8.0)*1.5707963267948966;
                buffer[n++]=screen(box[corner]+Vec2{std::cos(a),std::sin(a)}*e.preferredDistance);
            }
            draw->AddPolyline(buffer,n,IM_COL32(160,210,245,95),1.f,ImDrawFlags_Closed);
        }
        auto mouse=ImGui::GetIO().MousePos;
        double mx=(mouse.x-p.x)*map.metersPerPixel/scale,my=(mouse.y-p.y)*map.metersPerPixel/scale;
        double along=mx*std::cos(e.heading)+my*std::sin(e.heading),across=-mx*std::sin(e.heading)+my*std::cos(e.heading);
        if (hovered && hitNode<0 && std::abs(along)<e.length/2+.15 && std::abs(across)<e.width/2+.15) {
            const char* state=e.reversing?"\nReversing to yield":e.signalLight!=-1?(e.next>=e.path.size()?"\nWaiting in stop-line slot":"\nFilling stop-line space"):e.waitingForLight?"\nWaiting for signal":e.tugActive?"\nForce-guided: momentum and torque":e.waitingForManeuver?"\nWaiting for reserved maneuver":e.usingLocalDetour?"\nLocal detour":e.following?"\nFollowing traffic":"";
            ImGui::SetTooltip("%s %llu\nSpeed %.2f / %.2f m/s\nBody %.2f x %.2f m | %.0f kg\nPreferred gap %.2f m%s",vehicleName(e.kind),static_cast<unsigned long long>(e.id),e.speed,e.desiredSpeed,e.length,e.width,e.massKg,e.preferredDistance,state);
            if (editorMode_==0 && !placeLight_ && ImGui::IsMouseClicked(0)) selected_=e.id;
        }
    }
    draw->PopClipRect();
    nodePopup();
    ImGui::TextDisabled("%.1f x %.1f m | %.3f m/pixel | Bike / Motorbike / Car / Truck / Bus",map.width*map.metersPerPixel,map.height*map.metersPerPixel,map.metersPerPixel);
}
void SimulationView::inspector() {
    ImGui::SeparatorText("Vehicle inspector");
    ImGui::TextDisabled("Click a vehicle on the map to select it.");
    const Entity* selected=nullptr;
    for (const auto& e:simulation_.entities()) if (e.id==selected_) selected=&e;
    if (!selected) { ImGui::TextUnformatted("No active vehicle selected."); return; }
    ImGui::Text("%s %llu | %.2f m/s | %.2f x %.2f m | preferred gap %.2f m",vehicleName(selected->kind),static_cast<unsigned long long>(selected->id),selected->speed,selected->length,selected->width,selected->preferredDistance);
    ImGui::TextDisabled("%s | %.1fs without 0.25m progress",selected->reversing?"Yielding in reverse":selected->signalLight!=-1?(selected->next>=selected->path.size()?"Waiting at stop line":"Filling stop-line space"):selected->waitingForLight?"Waiting for signal":selected->waitingForManeuver?"Waiting for maneuver":selected->usingLocalDetour?"Local detour":selected->following?"Following traffic":"Driving / queueing",selected->noProgressSeconds);
    ImGui::Text("Stopped: total %.2f s | current %.2f s | longest %.2f s | trip %.2f s",
        selected->stoppedSeconds,selected->currentStopSeconds,selected->longestStopSeconds,selected->ageSeconds);
    if(selected->inOpenJunction) ImGui::Text("Stationary inside junction: %.2f s / %.2f s%s",selected->junctionStationarySeconds,stuckVehicleSettings_.junctionTimeoutSeconds,stuckVehicleSettings_.junctionEnabled?"":" (deletion disabled)");
    if(!selected->junctionStatus.empty()) ImGui::TextWrapped("Junction: %s",selected->junctionStatus.c_str());
    ImGui::Text("Trip origin: lane %d | spawn node %d",selected->spawnLane,selected->spawnNode);
    ImGui::TextWrapped("Lane change: %s | completed by this vehicle: %llu",selected->laneChangeStatus.c_str(),static_cast<unsigned long long>(selected->laneChanges));
    if(flow_.sublanes) ImGui::TextWrapped("Within-lane gap: %s",selected->sublaneStatus.c_str());
    if(selected->cooperativeUntil>simulation_.seconds()) ImGui::Text("Creating gap for vehicle %llu",static_cast<unsigned long long>(selected->cooperativeFor));
    ImGui::TextWrapped("Verified state: %s",stopReasonName(simulation_.stopReason(*selected)));
    if(simulation_.ownsNegotiatedTurn(*selected)) ImGui::TextColored(ImVec4(1,.75f,.2f,1),"Turn priority: creeping through claimed space");
    else if(auto owner=simulation_.maneuverYieldTo(*selected)) ImGui::Text("%s vehicle %llu",selected->clearingTurn?"Clearing space for":"Yielding to",static_cast<unsigned long long>(owner));
    ImGui::Text("Force guidance: %s | mass %.0f kg | yaw %.2f rad/s",selected->tugActive?"Active":"Standby",selected->massKg,selected->angularVelocity);
    float mass=static_cast<float>(selected->massKg);
    if(ImGui::DragFloat("Vehicle mass (kg)",&mass,10,20,100000,"%.0f",ImGuiSliderFlags_AlwaysClamp)) simulation_.setEntityMass(selected->id,mass);
    ImGui::Text("Destination refreshes: %llu | Moving timer: %.2f s",static_cast<unsigned long long>(selected->destinationRefreshes),selected->movingSinceRefresh);
    double redDistance=0; int nearestRed=simulation_.closestStoppingLight(*selected,&redDistance);
    if(nearestRed>=0) ImGui::Text("Nearest required stop: L%d | %.2f m along route",nearestRed,redDistance);
    if(!lights_.empty()) {
        const auto& light=lights_[std::clamp(selectedLight_,0,static_cast<int>(lights_.size())-1)];
        if(light.permissive) ImGui::Text("Selected light red/yellow: %s",simulation_.routePermitsRed(*selected,light)?"Allowed by planned route":"Must stop");
    }
    float speed=static_cast<float>(selected->desiredSpeed),accel=static_cast<float>(selected->acceleration);
    ImGui::SetNextItemWidth(230);
    bool changed=ImGui::SliderFloat("Desired speed",&speed,0.05f,50,"%.2f");
    ImGui::SameLine(); ImGui::SetNextItemWidth(230);
    changed|=ImGui::SliderFloat("Acceleration",&accel,0.05f,100,"%.2f");
    if (changed) simulation_.setEntityMotion(selected->id,speed,accel);
}
}
