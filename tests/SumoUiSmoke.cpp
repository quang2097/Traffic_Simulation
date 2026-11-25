#include "traffic/SimulationView.hpp"
#include "imgui.h"
#include "imgui_internal.h"
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
void render(traffic::SimulationView& view,double seconds=0){
    ImGui::NewFrame();view.frame(seconds);ImGui::Render();
    require(ImGui::GetDrawData() && ImGui::GetDrawData()->TotalVtxCount>0,"UI generated no map or controls");
}
}
namespace traffic {
struct SimulationViewTestAccess {
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
            traffic::SimulationView view(nullptr,mode=="native" || mode=="tools"?traffic::SimulationEngine::Native:traffic::SimulationEngine::Sumo);
            if(mode=="tools")traffic::SimulationViewTestAccess::tools(view);
            else if(mode=="switch")traffic::SimulationViewTestAccess::switching(view);
            else traffic::SimulationViewTestAccess::startup(view);
        }
        ImGui::DestroyContext();std::cout<<mode<<" UI checks passed\n";return 0;
    }catch(const std::exception& e){ImGui::DestroyContext();std::cerr<<e.what()<<"\n";return 1;}
}
