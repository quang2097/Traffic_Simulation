#pragma once
#include "traffic/Simulation.hpp"
#include "traffic/StepBatch.hpp"
#include "traffic/NodeGraph.hpp"
#include "traffic/SumoBackend.hpp"
#include <string>
#include <map>

namespace traffic {
enum class SimulationEngine { Native, Sumo };
class SimulationView {
public:
    explicit SimulationView(const char* initialMap = nullptr,SimulationEngine engine=SimulationEngine::Sumo);
    void frame(double realSeconds);
    bool isAdvancing() const { return running_ || stepBatch_.active(); }
    void selectEngine(SimulationEngine engine);
    SimulationEngine engine() const { return engine_; }
    bool engineReady() const { return engineReady_; }
    bool editorActionActive() const;
    void cancelEditorAction();
private:
    friend struct SimulationViewTestAccess;
    void reset();
    void controls();
    void simulationControls();
    void menuBar();
    void fileAction(int action);
    int workspace_=5;
    SimulationEngine engine_=SimulationEngine::Sumo;
    bool engineReady_=false,manualOpen_=false,showInspector_=false;
    int manualChapter_=0;
    char manualSearch_[128]{};
    void manual();
    const char* editorActionLabel() const;
    void canvas();
    void inspector();
    void refreshMap(bool pauseTraffic,bool importSignals=false);
    void addLight(Vec2 a,Vec2 b,bool permissive=false,Vec2 allowedA={},Vec2 allowedB={},Vec2 outDirection={},int anchorNode=-1);
    void cancelPendingLine();
    void syncSignalMask();
    void advanceTick(double seconds);
    bool applyGraph(const NodeGraph& graph);
    void nodePopup();
    void roadSections(int road);
    bool laneConnectionsPopup();
    void toggleLaneExit(int road,int direction,int lane);
    bool applyLaneExitDraft();
    bool laneExitEditorOpen_=false,turnDirty_=false;
    int exitRoadChoice_=-1,exitDirectionChoice_=0;
    void roadSignalPopup();
    void loadLaneDemo();
    int turnRoad_=-1,turnDirection_=0,turnLane_=1,turnDraftNode_=-1,turnDraftRoad_=-1,turnDraftDirection_=-1,turnDraftLane_=-1;
    struct ExitDraft { int node=-1,road=-1,direction=0;std::array<char,256> lanes{};std::map<int,int> priorities; };
    std::vector<ExitDraft> turnExits_;
    std::string turnError_;
    std::vector<int> highlightedExitNodes_;
    int signalRoad_=-1;
    int nodePopupTab_=-1;
    bool placingRoadSignal_=false;
    bool validateRun();
    bool validationNotice_=false,laneExitsReady_=false;
    std::vector<std::string> validationIssues_;
    NodeGraph graph_;
    int selectedNode_=-1,connectionFrom_=-1;
    bool severing_=false;
    std::vector<int> pathNodes_;
    std::vector<double> nodeSpawnClocks_;
    std::vector<int> laneOriginNodes_;
    bool splittingRoad_=false;
    bool drawingRoad_=false;
    int newRoadType_=2,newRoadLanes_=2;
    std::vector<Vec2> roadDraft_;
    std::vector<double> roadDraftWidths_;
    float nextPointWidth_=3.5f;
    float mapZoom_=1;
    Vec2 mapPan_;
    std::mt19937 spawnNodeRandom_{42};
    Map source_;
    Simulation simulation_;
    std::unique_ptr<SumoBackend> sumo_;
    RoutingSettings routing_;
    TrafficFlowSettings flow_{true,true,.4,.15};
    StuckVehicleSettings stuckVehicleSettings_{true,30,true,60};
    GridlockEscapeSettings gridlockEscape_{false,15,3,1.5};
    double sectionBegin_=10,sectionEnd_=30,openingBegin_=40,openingEnd_=55;
    int sectionDirection_=0,sectionLane_=0;
    bool sectionEnter_=false,sectionExit_=false;
    TrafficRecoverySettings trafficRecoverySettings_;
    UrbanTrafficSettings urbanTraffic_;
    TugSettings tugSettings_;
    TurnNegotiationSettings turnNegotiation_;
    std::array<VehicleProfile,vehicleKindCount> profiles_=defaultProfiles();
    std::array<float,vehicleKindCount> spawnMix_{};
    bool spawnMixDirty_=false;
    std::vector<TrafficLight> lights_;
    std::vector<DirectedLane> lanes_;
    float laneWidth_=3.5f;
    float laneSpeed_=8;
    char laneFile_[1024]="examples/custom.nodes";
    int selectedKind_=2,selectedLight_=0;
    double metersPerPixel_=0.5;
    float stopLineLength_=10;
    int maximumVehicles_=100;
    bool showClearance_=false,placeLight_=false,verticalLine_=true;
    bool running_=true, automatic_=true, routes_=false;
    float timeScale_=1;
    double accumulator_=0, elapsed_=0;
    double planningTimeToExclude_=0;
    StepBatch stepBatch_;
    int clockMode_=0;
    float manualStepSeconds_=1;
    int seed_=42;
    int collisionWorkers_=static_cast<int>(Simulation::maximumCollisionWorkers());
    bool includeCompletedStops_=true;
    std::uint64_t selected_=0, spawned_=0;
    char mapPath_[1024]="examples/annotated-map.png";
    std::string status_;
    int editorMode_=0,newWidth_=256,newHeight_=192;
    float brushWidth_=8;
    bool linePending_=false,strokeActive_=false,mapDirty_=false;
    Vec2 lineStart_,lastStroke_;
    int linkedStage_=0,linkedLight_=-1;
    Vec2 pendingStopA_,pendingStopB_;
    char savePath_[1024]="examples/custom-map.png";
};
}
