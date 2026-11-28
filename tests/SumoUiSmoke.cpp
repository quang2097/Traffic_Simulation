#include "traffic/SimulationView.hpp"
#include "imgui.h"
#include "imgui_internal.h"
#include <iostream>
#include <stdexcept>
#include <string>
#include <chrono>
#include <thread>
#include <filesystem>
#include <fstream>
#include <iterator>

namespace {
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
void render(traffic::SimulationView& view,double seconds=0){
    ImGui::NewFrame();view.frame(seconds);ImGui::Render();
    require(ImGui::GetDrawData() && ImGui::GetDrawData()->TotalVtxCount>0,"UI generated no map or controls");
}
}
namespace traffic {
struct SimulationViewTestAccess {
    static void normalTr(SimulationView& view){
        loadEditorFixture(view);view.workspace_=2;view.automatic_=false;view.laneExitsReady_=true;
        TrafficLight west;west.id=1;west.a={180,235};west.b={180,265};west.outDirection={1,0};west.anchorNode=1;
        auto east=west;east.id=2;east.a.x=east.b.x=320;east.outDirection={-1,0};
        auto north=west;north.id=3;north.a={235,180};north.b={265,180};north.outDirection={0,1};
        auto south=north;south.id=4;south.a.y=south.b.y=320;south.outDirection={0,-1};
        view.lights_={west,east,north,south};view.selectedLight_=0;view.simulation_.setTrafficLights(view.lights_);view.applyNormalTr(1,20);render(view);
        require(!view.running_,"Applying normal Tr did not pause traffic");
        for(const auto& light:view.simulation_.trafficLights())require(light.normalTr.active && light.trSeconds==20,"Shared Tr did not include every connected light");
        require(view.lights_[0].trGroup==view.lights_[1].trGroup && view.lights_[0].trGroup!=view.lights_[2].trGroup,"Normal Tr did not group opposing approaches");
        for(int i=0;i<50;++i)view.advanceTick(.1);auto before=view.simulation_.trafficLights()[0].trRemaining(view.simulation_.seconds());render(view);
        require(before>0 && std::abs(view.simulation_.trafficLights()[0].trRemaining(view.simulation_.seconds())-before)<1e-6,"Rendering normal Tr restarted the clearance clock");
        view.applyNormalTr(1,0);render(view);for(const auto& light:view.simulation_.trafficLights())require(!light.normalTr.active && light.redSeconds==west.redSeconds,"Disabling Tr did not restore independent manual timings");
    }
    static void deleteVehicle(SimulationView& view){
        view.source_=Map(100,60,std::vector<Tile>(6000,Tile::Road));view.graph_={};view.lights_.clear();view.reset();view.running_=false;view.automatic_=false;
        require(view.simulation_.spawnAt({10,20},{80,20},VehicleKind::Car,0) && view.simulation_.spawnAt({10,40},{80,40},VehicleKind::Car,0),"Delete UI fixture failed to spawn");
        view.selected_=view.simulation_.entities().front().id;auto survivor=view.simulation_.entities().back().id;view.showInspector_=true;render(view);
        auto& io=ImGui::GetIO();view.running_=true;io.AddKeyEvent(ImGuiKey_Delete,true);render(view);io.AddKeyEvent(ImGuiKey_Delete,false);render(view);
        require(view.simulation_.completed()==1 && view.simulation_.manuallyCompleted()==1 && !view.selected_,"Delete key did not complete and deselect the chosen trip");
        require(view.running_ && view.simulation_.seconds()==0 && view.simulation_.entities().size()==1 && view.simulation_.entities().front().id==survivor,"Delete key changed playback, time or another vehicle");
        view.selected_=survivor;char text[64]="Editing notes";
        auto textFrame=[&](bool focus){ImGui::NewFrame();view.frame(0);ImGui::SetNextWindowPos(ImVec2(400,120),ImGuiCond_Always);ImGui::SetNextWindowSize(ImVec2(350,120),ImGuiCond_Always);if(focus)ImGui::SetNextWindowFocus();ImGui::Begin("Deletion text guard");if(focus)ImGui::SetKeyboardFocusHere();ImGui::InputText("Notes",text,sizeof(text));ImGui::End();ImGui::Render();};
        textFrame(true);textFrame(true);textFrame(false);require(io.WantTextInput,"Text guard fixture did not focus its input");
        io.AddKeyEvent(ImGuiKey_Delete,true);textFrame(false);io.AddKeyEvent(ImGuiKey_Delete,false);textFrame(false);
        require(view.simulation_.completed()==1 && view.selected_==survivor,"Delete in a text field deleted a vehicle");
        ImGui::ClearActiveID();render(view);view.deleteSelectedVehicle();render(view);
        require(view.simulation_.completed()==2 && view.simulation_.manuallyCompleted()==2 && view.simulation_.entities().empty() && !view.selected_,"Inspector deletion did not complete the remaining trip");
    }
    static void greenCompletion(SimulationView& view){
        loadEditorFixture(view);view.running_=false;view.workspace_=2;
        auto geometry=roadSignalAtNode(view.graph_,0,1);view.addLight(geometry.a,geometry.b,false,{},{},geometry.outDirection,1);
        auto& light=view.lights_.back();int junction=connectedSignalJunction(view.graph_,light);require(junction==1,"Signal UI did not find its connected junction");
        view.setGreenCompletionTarget(light,junction);light.completeJunctionOnGreen=true;view.simulation_.setTrafficLights(view.lights_);render(view);
        require(view.simulation_.trafficLights().back().completeJunctionOnGreen && light.completionRadius>0,"Signal UI did not apply completion region");
        view.workspace_=8;render(view);
    }
    static void results(SimulationView& view){
        view.running_=false;view.workspace_=8;render(view);
        auto id=view.resultRunId_;auto actualSeed=view.runSeed_;view.seed_=actualSeed+10;
        auto snapshot=view.resultSnapshot();
        auto field=[&](const SimulationResult& r,const std::string& key){for(std::size_t i=0;i<r.columns.size();++i)if(r.columns[i]==key)return r.values[i];throw std::runtime_error("Results field missing");};
        require(field(snapshot,"seed")==std::to_string(actualSeed),"Export used the edited seed before a reset");
        auto file=std::filesystem::temp_directory_path()/("traffic-ui-results-"+newSimulationRunId()+".csv");
        std::snprintf(view.resultsPath_,sizeof(view.resultsPath_),"%s",file.u8string().c_str());
        std::snprintf(view.resultLabel_,sizeof(view.resultLabel_),"%s","Baseline");view.saveResult();
        require(view.resultsStatus_.find("Saved one snapshot")==0,"Results panel did not save");
        require(view.resultRunId_==id && !view.running_,"Saving changed run identity or playback");
        view.reset();view.running_=false;view.workspace_=8;
        require(view.resultRunId_!=id && view.runSeed_==view.seed_,"Reset did not assign a new result run and seed");
        std::snprintf(view.resultLabel_,sizeof(view.resultLabel_),"%s","Overlap enabled");view.gridlockEscape_.allowOverlap=true;view.simulation_.setGridlockEscapeSettings(view.gridlockEscape_);view.saveResult();render(view);
        std::ifstream input(file,std::ios::binary);std::string saved{std::istreambuf_iterator<char>(input),{}};input.close();
        require(saved.find("Baseline")!=std::string::npos && saved.find("Overlap enabled")!=std::string::npos,"Saving a second run lost the first result");
        auto header=saved.find("schema_version");require(header==0 && saved.find("schema_version",1)==std::string::npos,"Results repeated CSV header");
        std::filesystem::remove(file);
    }
    static void training(SimulationView& view){
        auto root=std::filesystem::path(TRAFFIC_PROJECT_ROOT)/"tests/fixtures/directional-signals";
        view.source_=Map::fromPng((root/"custom-map.png").string(),1);view.graph_=NodeGraph::load((root/"custom.nodes").string());view.lights_.clear();view.running_=false;view.reset();
        view.simulation_.loadTrafficLights((root/"custom-map.png.signals").string());view.lights_=view.simulation_.trafficLights();view.policyRegions_=defaultSignalRegions(view.lights_);for(auto& region:view.policyRegions_)region.group=1-region.group;
        auto expected=view.policyRegions_;std::vector<double> rates;for(auto& node:view.graph_.nodes)rates.push_back(node.rate);
        auto output=std::filesystem::temp_directory_path()/("traffic-ui-training-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));std::snprintf(view.trainingDirectory_,sizeof(view.trainingDirectory_),"%s",output.string().c_str());
        view.workspace_=7;view.demandConfig_.maximum=.1;view.demandConfig_.minimum=.02;view.demandConfig_.seed=71;view.trainingMaxRed_=10;view.trainingTr_=2;view.trainingYellow_=1;view.trainingMinGreen_=1;view.startTraining();
        require(view.trainingRecorder_.active() && view.holdTiming_.active() && view.randomDemand_.active(),"Training UI did not start recording/control/random demand");
        require(view.policyRegions_[0].group==expected[0].group,"Starting recording discarded edited signal groups");
        render(view);for(int step=0;step<50;++step)view.advanceTick(.1);render(view);
        auto directory=view.trainingRecorder_.directory();view.stopTraining();
        require(!view.holdTiming_.active() && !view.randomDemand_.active() && !view.trainingRecorder_.active(),"Stopping recording left automatic control active");
        for(auto& light:view.lights_)require(!light.externallyControlled,"Stopping recording left externally held phases active");
        for(std::size_t node=0;node<rates.size();++node)require(view.graph_.nodes[node].rate==rates[node],"Random demand did not restore original spawn rates");
        require(std::filesystem::file_size(std::filesystem::path(directory)/"transitions.jsonl")>100,"Training UI did not export transitions");
        auto read=[](const std::filesystem::path& path){std::ifstream in(path,std::ios::binary);return std::string(std::istreambuf_iterator<char>(in),{});};
        auto combined=output/"transitions.jsonl";auto before=read(combined);auto original=read(std::filesystem::u8path(directory)/"transitions.jsonl");
        require(before.find("\"session_id\"")!=std::string::npos,"Training UI did not publish combined transitions");
        view.demandConfig_.seed=72;view.startTraining();auto secondDirectory=view.trainingRecorder_.directory();
        for(int step=0;step<50;++step)view.advanceTick(.1);view.stopTraining();render(view);
        auto appended=read(combined);require(secondDirectory!=directory && appended.size()>before.size() && appended.substr(0,before.size())==before,"Training UI append overwrote the existing dataset");
        require(read(std::filesystem::u8path(directory)/"transitions.jsonl")==original && appended.find("\"seed\":72")!=std::string::npos,"Training UI append lost per-run identity or modified old records");
    }
    static void policySelection(SimulationView& view,const char* python,const char* weights){
        view.workspace_=6;view.running_=false;view.policyRuntime_.load(python,std::string(TRAFFIC_PROJECT_ROOT)+"/scripts/rl_policy_worker.py",weights,"Auto");
        auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(15);bool opened=false;
        while(!opened){render(view);auto* popup=ImGui::FindWindowByName("Choose policy model");opened=popup && popup->Active;if(std::chrono::steady_clock::now()>deadline)throw std::runtime_error("Unknown-model UI did not request model selection");std::this_thread::sleep_for(std::chrono::milliseconds(5));}
        require(!view.policyRuntime_.loaded() && view.policyFamily_==0,"Unknown weights silently guessed a policy family");view.policyRuntime_.shutdown();
    }
    static void policy(SimulationView& view,const char* python,const char* weights,bool masked=false){
        loadEditorFixture(view);view.automatic_=false;view.workspace_=6;
        TrafficLight west;west.id=1;west.a={180,235};west.b={180,265};west.outDirection={1,0};
        auto east=west;east.id=2;east.a.x=east.b.x=320;east.outDirection={-1,0};
        auto north=west;north.id=3;north.a={235,180};north.b={265,180};north.outDirection={0,1};
        auto south=north;south.id=4;south.a.y=south.b.y=320;south.outDirection={0,-1};
        view.lights_={west,east,north,south};view.simulation_.setTrafficLights(view.lights_);view.laneExitsReady_=true;render(view);
        view.policyRuntime_.load(python,std::string(TRAFFIC_PROJECT_ROOT)+"/scripts/rl_policy_worker.py",weights,"Auto");
        auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(15);
        while(!view.policyRuntime_.loaded()){render(view);if(!view.policyRuntime_.loaded() && !view.policyRuntime_.busy())throw std::runtime_error(view.policyStatus_);if(std::chrono::steady_clock::now()>deadline)throw std::runtime_error("UI policy load timed out");std::this_thread::sleep_for(std::chrono::milliseconds(5));}
        require(view.policyRegions_.size()==4 && view.policyObservation_.mask==std::array<double,2>{1,1},"RL UI did not group the four lights into two agents");
        if(view.policyRuntime_.trainingSchema()){
            require(view.policyAlgorithm_=="DuelingDQN","UI did not identify the train.ipynb implementation");
            if(masked)view.trainingTr_=20;
            view.holdTiming_.begin(view.lights_,view.policyRegions_,0,view.trainingMaxRed_,view.trainingTr_,view.trainingYellow_,view.trainingMinGreen_);
            view.policyHoldEnabled_=true;view.simulation_.setTrafficLights(view.lights_);++view.policyEpoch_;view.policyNextDecision_=0;view.updateTraining();render(view);
            while(view.policyRuntime_.busy()){render(view);if(std::chrono::steady_clock::now()>deadline)throw std::runtime_error("Dueling policy inference timed out");std::this_thread::sleep_for(std::chrono::milliseconds(5));}
            require(view.trainingAction_==(masked?-1:1) && view.holdTiming_.active(),"Dueling policy request did not reach the hold controller");
            if(masked){
                for(int tick=0;tick<290;++tick){view.advanceTick(.1);auto t=view.simulation_.seconds();const auto& lights=view.simulation_.trafficLights();
                    if(t<23-1e-6)require(lights[0].phase(t)!=LightPhase::Green && lights[2].phase(t)!=LightPhase::Green,"Masked policy shortened startup yellow/20-second Tr");}
                view.policyNextDecision_=view.simulation_.seconds();render(view);
                while(view.policyRuntime_.busy()){render(view);if(std::chrono::steady_clock::now()>deadline)throw std::runtime_error("Masked policy inference timed out");std::this_thread::sleep_for(std::chrono::milliseconds(5));}
                require(view.trainingAction_==1,"Masked policy did not permit a switch after minimum green");
                auto switched=view.simulation_.seconds();bool released=false;
                for(int tick=0;tick<300;++tick){view.advanceTick(.1);auto t=view.simulation_.seconds();const auto& lights=view.simulation_.trafficLights();
                    if(lights[2].phase(t)==LightPhase::Green){require(t+1e-6>=switched+23,"Masked policy shortened switch yellow/20-second Tr");released=true;}}
                require(released,"Masked policy never released the requested green");
            }
            for(int tick=0;tick<200;++tick){view.advanceTick(.1);double t=view.simulation_.seconds();const auto& lights=view.simulation_.trafficLights();
                require(lights[0].phase(t)==lights[1].phase(t) && lights[2].phase(t)==lights[3].phase(t),"Dueling policy desynchronized paired signals");
                require(!(lights[0].phase(t)==LightPhase::Green && lights[2].phase(t)==LightPhase::Green),"Dueling policy allowed conflicting greens");}
            view.stopPolicy();require(!view.holdTiming_.active() && view.lights_[0].greenSeconds==west.greenSeconds,"Dueling policy stop did not restore manual timing");
            view.policyRuntime_.shutdown();return;
        }
        view.policyTiming_.begin(view.lights_,view.policyRegions_,0);view.simulation_.setTrafficLights(view.lights_);++view.policyEpoch_;view.policyNextDecision_=0;
        render(view);
        while(view.policyRuntime_.busy()){render(view);if(std::chrono::steady_clock::now()>deadline)throw std::runtime_error("UI policy inference timed out");std::this_thread::sleep_for(std::chrono::milliseconds(5));}
        require(view.policyTiming_.active() && view.policyTiming_.pending()==std::array<double,2>{5,5},"Policy actions did not reach the UI timing controller");
        for(int tick=0;tick<200;++tick){view.advanceTick(.1);double t=view.simulation_.seconds();auto& lights=view.simulation_.trafficLights();
            require(lights[0].phase(t)==lights[1].phase(t) && lights[2].phase(t)==lights[3].phase(t),"Paired UI lights drifted out of sync");
            require(!(lights[0].phase(t)==LightPhase::Green && lights[2].phase(t)==LightPhase::Green),"UI policy allowed conflicting green groups");}
        view.stopPolicy();require(!view.policyTiming_.active() && view.lights_[0].greenSeconds==west.greenSeconds,"UI stop did not restore manual timing");
        view.policyTiming_.begin(view.lights_,view.policyRegions_,view.simulation_.seconds());view.simulation_.setTrafficLights(view.lights_);
        view.reset();require(!view.policyTiming_.active() && view.lights_[0].greenSeconds==west.greenSeconds,"Simulation reset kept policy timing active");
        view.policyRuntime_.shutdown();
    }
    static void loadEditorFixture(SimulationView& view){
        // Example files are editable by users; keep UI assertions independent
        // of saved changes to the application's seven-lane demonstration.
        NodeGraph graph;graph.explicitDestinations=true;
        for(Vec2 point:std::vector<Vec2>{{20,250},{250,250},{250,20},{480,250},{250,480}})graph.addNode(point);
        for(int node:{0,2,3,4})graph.nodes[node].destination=true;
        graph.nodes[0].spawn=true;
        for(auto ends:std::vector<std::pair<int,int>>{{0,1},{2,1},{1,3},{1,4}})graph.connect(ends.first,ends.second,3.5,8);
        for(int road=0;road<4;++road){graph.connections[road].roadType=road==1 || road==3?2:1;graph.connections[road].laneCount=road==1?3:road==3?2:7;}
        graph.connections[3].reverseLaneCount=4;
        graph.setLaneTurnRule(0,{0,0,{{2,{0}}}});
        for(int lane=1;lane<6;++lane)graph.setLaneTurnRule(0,{0,lane,{{3,{0,1,2,3,4,5,6}}}});
        graph.setLaneTurnRule(0,{0,6,{{4,{1}}}});
        view.cancelEditorAction();view.graph_=std::move(graph);view.source_=Map(500,500,std::vector<Tile>(250000,Tile::Road));view.lights_.clear();
        view.running_=false;view.workspace_=1;view.selectedNode_=1;view.turnRoad_=0;view.turnDirection_=0;view.turnLane_=1;view.reset();
    }
    static void startup(SimulationView& view){
        require(view.engineReady_,"Engine failed to initialize");
        for(int frame=0;frame<50;++frame){render(view,.1);require(view.isAdvancing(),"Simulation paused during startup");}
        require(view.elapsed_>.5,"Simulation clock did not advance");
        require(view.spawned_>0,"Selected engine did not spawn vehicles");
        require((view.sumo_!=nullptr)==(view.engine_==SimulationEngine::Sumo),"Engine selection did not attach the right controller");
    }
    static void switching(SimulationView& view){
        auto tiles=view.source_.tiles;auto nodes=view.graph_.nodes.size(),roads=view.graph_.connections.size();auto speed=view.profiles_[2].motion.speedMean;
        startup(view);
        view.selectEngine(SimulationEngine::Native);
        require(view.engineReady_ && !view.sumo_ && !view.running_,"Native activation failed or did not pause");
        require(view.simulation_.entities().empty() && view.elapsed_==0,"Engine switch did not reset traffic");
        view.workspace_=5;render(view);require(view.engine_==SimulationEngine::Native,"Browsing a section changed engines");
        view.running_=true;startup(view);
        view.selectEngine(SimulationEngine::Sumo);
        require(view.engineReady_ && view.sumo_ && !view.running_,"SUMO reactivation failed or did not pause");
        require(view.simulation_.entities().empty() && view.elapsed_==0,"SUMO reactivation did not reset traffic");
        require(view.source_.tiles==tiles && view.graph_.nodes.size()==nodes && view.graph_.connections.size()==roads && view.profiles_[2].motion.speedMean==speed,"Engine switch changed the design or profiles");
    }
    static ImVec2 cancelButton(){
        for(auto* window:ImGui::GetCurrentContext()->Windows)if(window->Active && std::string(window->Name).find("Active tool")!=std::string::npos)
            return {window->Pos.x+35,window->Pos.y+19};
        throw std::runtime_error("Active editor action has no visible Cancel button");
    }
    static void tools(SimulationView& view){
        view.running_=false;view.workspace_=1;view.mapZoom_=2;
        for(int tool=0;tool<10;++tool){
            view.cancelEditorAction();
            switch(tool){
                case 0:view.connectionFrom_=1;break;
                case 1:view.connectionFrom_=1;view.severing_=true;break;
                case 2:view.drawingRoad_=true;view.roadDraft_={{20,65},{30,65}};view.roadDraftWidths_={3.5,4};break;
                case 3:view.splittingRoad_=true;break;
                case 4:view.editorMode_=3;break;
                case 5:view.editorMode_=1;break;
                case 6:view.editorMode_=7;view.linkedStage_=1;view.linePending_=true;break;
                case 7:view.editorMode_=8;view.linePending_=true;break;
                case 8:view.placeLight_=true;break;
                case 9:view.placingRoadSignal_=true;break;
            }
            auto tiles=view.source_.tiles;auto nodes=view.graph_.nodes.size(),roads=view.graph_.connections.size();
            render(view);require(view.editorActionActive(),"Tool did not stay active until cancelled");
            auto point=cancelButton();auto& io=ImGui::GetIO();io.AddMousePosEvent(point.x,point.y);render(view);
            io.AddMouseButtonEvent(0,true);render(view);io.AddMouseButtonEvent(0,false);render(view);
            require(!view.editorActionActive(),"Cancel button did not cancel the editor action");
            require(view.connectionFrom_==-1 && !view.severing_ && view.roadDraft_.empty() && view.roadDraftWidths_.empty(),"Cancel left a stale connection or sketch");
            require(view.source_.tiles==tiles && view.graph_.nodes.size()==nodes && view.graph_.connections.size()==roads,"Cancel click changed the map underneath it");
        }
        view.connectionFrom_=1;render(view);auto& io=ImGui::GetIO();
        io.AddKeyEvent(ImGuiKey_Escape,true);render(view);io.AddKeyEvent(ImGuiKey_Escape,false);render(view);
        require(!view.editorActionActive(),"Escape did not cancel a connection");
        view.manualOpen_=true;render(view);require(ImGui::FindWindowByName("User manual")->Active,"In-app manual did not open");
        std::snprintf(view.manualSearch_,sizeof(view.manualSearch_),"cancel");render(view);
        view.manualSearch_[0]=0;view.manualChapter_=1;render(view);
        io.AddKeyEvent(ImGuiKey_Escape,true);render(view);io.AddKeyEvent(ImGuiKey_Escape,false);render(view);
        require(!view.manualOpen_,"Escape did not close the manual");
        loadEditorFixture(view);
        // Render every panel in a constrained window; detect unbalanced popup,
        // child and tab scopes, including after a signal is added at this node.
        for(int panel=0;panel<5;++panel){
            view.nodePopupTab_=panel;
            for(int frame=0;frame<3 && view.nodePopupTab_>=0;++frame){ImGui::NewFrame();ImGui::Begin("Node editor test");ImGui::OpenPopup("Node settings");view.nodePopup();ImGui::End();ImGui::Render();}
            require(view.nodePopupTab_==-1,"Requested node panel did not open");
        }
        auto signal=roadSignalAtNode(view.graph_,0,1);view.addLight(signal.a,signal.b,false,{},{},signal.outDirection,1);
        view.nodePopupTab_=3;ImGui::NewFrame();ImGui::Begin("Node editor test");ImGui::OpenPopup("Node settings");view.nodePopup();ImGui::End();ImGui::Render();
        view.workspace_=2;render(view);
        // Edit a single source lane in the sidebar while the map remains available.
        loadEditorFixture(view);view.laneExitEditorOpen_=true;render(view);
        require(view.turnDraftLane_==1 && !view.turnExits_.empty(),"Lane editor did not load the selected source lane");
        auto original=view.graph_.laneTurnRule(0,0,0)->exits.size();
        view.toggleLaneExit(2,0,3);
        require(view.turnDirty_ && view.graph_.laneTurnRule(0,0,0)->exits.size()==original,"Map picking committed an unfinished draft");
        view.toggleLaneExit(2,0,3); // Clicking again removes this exact target.
        view.toggleLaneExit(2,0,4);
        view.toggleLaneExit(3,1,4); // Reverse carriageway has its own lane count.
        for(auto& draft:view.turnExits_)if(draft.road==2 && draft.direction==0)draft.priorities[4]=7;
        require(view.applyLaneExitDraft(),"Visual lane choices could not be applied");
        require(view.graph_.turnAllowed(0,0,0,2,0,3) && !view.graph_.turnAllowed(0,0,0,2,0,2),"Numbered picking selected the wrong target lane");
        require(view.graph_.turnAllowed(0,0,0,3,1,3),"Reverse-direction target was lost");
        require(view.graph_.turnAllowed(0,0,1,2,0,2),"Editing lane 1 changed lane 2's connections");
        auto rule=view.graph_.laneTurnRule(0,0,0);bool prioritySaved=false;
        for(const auto& exit:rule->exits)if(exit.road==2)prioritySaved=exit.priorities==std::vector<int>{7};
        require(prioritySaved,"Visual picking lost the movement priority");
        render(view);view.turnLane_=2;render(view);
        require(view.turnDraftLane_==2 && view.turnExits_.size()==1 && view.turnExits_[0].road==2,"Switching source lane leaked the previous lane's preview");
        view.toggleLaneExit(3,0,1);view.turnDraftNode_=-1;render(view);
        require(!view.turnDirty_ && view.turnExits_.size()==1,"Reload did not discard the unsaved target");
        auto& editorIo=ImGui::GetIO();editorIo.AddKeyEvent(ImGuiKey_Escape,true);render(view);editorIo.AddKeyEvent(ImGuiKey_Escape,false);render(view);
        require(!view.laneExitEditorOpen_,"Escape left the map in lane-picking mode");
        view.graph_.connections[0].laneTurns.clear();view.running_=true;
        require(!view.validateRun() && !view.running_ && !view.validationIssues_.empty(),"Unassigned exits did not refuse simulation");
        render(view);require(ImGui::FindWindowByName("Lane exits required")->Active,"Missing-exit notification did not open");
    }
};
}
int main(int argc,char** argv){
    ImGui::CreateContext();
    try {
        auto& io=ImGui::GetIO();io.DisplaySize={1280,900};io.DeltaTime=.1f;io.IniFilename=nullptr;
        unsigned char* pixels;int width,height;io.Fonts->GetTexDataAsRGBA32(&pixels,&width,&height);
        std::string mode=argc>1?argv[1]:"sumo";
        {
            traffic::SimulationView view(nullptr,mode=="native" || mode=="normal-tr" || mode=="tools" || mode=="policy" || mode=="policy-masked" || mode=="policy-select" || mode=="training" || mode=="results" || mode=="green-clear" || mode=="delete-vehicle"?traffic::SimulationEngine::Native:traffic::SimulationEngine::Sumo);
            if(mode=="policy"){require(argc==4,"Policy UI test requires Python and ONNX paths");traffic::SimulationViewTestAccess::policy(view,argv[2],argv[3]);}
            else if(mode=="policy-masked"){require(argc==4,"Masked policy UI test requires Python and weights paths");traffic::SimulationViewTestAccess::policy(view,argv[2],argv[3],true);}
            else if(mode=="normal-tr")traffic::SimulationViewTestAccess::normalTr(view);
            else if(mode=="policy-select"){require(argc==4,"Policy selection test requires Python and ONNX paths");traffic::SimulationViewTestAccess::policySelection(view,argv[2],argv[3]);}
            else if(mode=="training")traffic::SimulationViewTestAccess::training(view);
            else if(mode=="results")traffic::SimulationViewTestAccess::results(view);
            else if(mode=="green-clear")traffic::SimulationViewTestAccess::greenCompletion(view);
            else if(mode=="delete-vehicle")traffic::SimulationViewTestAccess::deleteVehicle(view);
            else if(mode=="tools")traffic::SimulationViewTestAccess::tools(view);
            else if(mode=="switch")traffic::SimulationViewTestAccess::switching(view);
            else traffic::SimulationViewTestAccess::startup(view);
        }
        ImGui::DestroyContext();std::cout<<mode<<" UI checks passed\n";return 0;
    }catch(const std::exception& e){ImGui::DestroyContext();std::cerr<<e.what()<<"\n";return 1;}
}
