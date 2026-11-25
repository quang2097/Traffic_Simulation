#pragma once
#include <array>
#include <algorithm>
#include <cstdint>
#include <deque>
#include <random>
#include <string>
#include <vector>

namespace traffic {
struct Cell {
    int x,y;
    bool operator==(Cell b) const { return x==b.x && y==b.y; }
    bool operator!=(Cell b) const { return !(*this==b); }
};
struct Vec2 { double x=0,y=0; };
Vec2 operator+(Vec2 a,Vec2 b);
Vec2 operator-(Vec2 a,Vec2 b);
Vec2 operator*(Vec2 a,double s);
double length(Vec2 a);
double angleDelta(double from,double to);
struct Footprint {
    Vec2 center;
    double heading=0,length=4.5,width=1.8;
};
std::vector<Vec2> corners(const Footprint& box);
double clearance(const Footprint& a,const Footprint& b);
enum class Tile { Wall,Road,Spawn,Destination,Signal,Divider };
enum class MapPalette { Traffic,Legacy };
struct MapFeature {
    Tile type=Tile::Wall;
    Vec2 center,a,b;
    bool line=false;
    std::vector<Cell> pixels;
};
struct Map {
    int width,height;
    std::vector<Tile> tiles;
    double metersPerPixel;
    Map(int width,int height,std::vector<Tile> tiles,double metersPerPixel=1);
    bool contains(Cell p) const;
    bool walkable(Cell p) const;
    int index(Cell p) const;
    Cell pixel(Vec2 p) const;
    Vec2 center(Cell p) const;
    std::vector<Cell> neighbors(Cell p) const;
    std::vector<Cell> cells(Tile type) const;
    bool fits(const Footprint& box) const;
    void prepareCollision();
    bool hasWalls(int x0,int y0,int x1,int y1) const;
    static Map fromPng(const std::string& path,double metersPerPixel=1,MapPalette palette=MapPalette::Traffic);
    void savePng(const std::string& path) const;
    void rebuildFeatures();
    const std::vector<MapFeature>& features() const { return features_; }
    int destinationAt(Vec2 point) const;
    bool touchesDestination(const Footprint& box,int feature) const;
    void paintStroke(Vec2 from,Vec2 to,Tile type,double widthMeters);
    void eraseFeature(std::size_t index);
private:
    std::vector<int> wallIntegral_;
    std::vector<MapFeature> features_;
    std::vector<int> destinationLabels_;
};
struct NavigationRegion {
    Vec2 center;
    double area=0;
    int component=-1;
    std::vector<int> neighbors;
};
// Static connected areas inside small sectors form the global navigation
// graph. Goal fields are shared by units targeting the same destination area.
class Navigation {
public:
    explicit Navigation(const Map& map);
    int regionAt(Vec2 point) const;
    bool connected(Vec2 start,Vec2 goal) const;
    const std::vector<double>& goalField(int region);
    const std::vector<NavigationRegion>& regions() const { return regions_; }
    std::uint64_t cacheHits() const { return cacheHits_; }
    std::uint64_t fieldBuilds() const { return fieldBuilds_; }
private:
    int width_,height_;
    double scale_;
    std::vector<int> labels_;
    std::vector<NavigationRegion> regions_;
    std::vector<std::pair<int,std::vector<double>>> fields_;
    std::uint64_t cacheHits_=0,fieldBuilds_=0;
};
struct RoutingSettings {
    bool rtsStyle=true;
    double lookAhead=12,localInterval=.75,congestionWeight=1.5;
    bool spreadAtSignals=true;
    bool refreshMovingRoutes=true;
    double movingRefreshInterval=3;
};
struct TrafficFlowSettings {
    bool predictiveJunctions=false;
    bool sublanes=false;
    double lateralResolution=.4,lateralGap=.15;
    bool compactAdmission=true;
    bool swapStarvedPriorities=false;
    double priorityWaitSeconds=15;
};
struct CarFollowingSettings {
    bool enabled=true; // False is retained only for controlled legacy comparisons.
};
struct StuckVehicleSettings {
    bool enabled=true;
    double timeoutSeconds=30;
    bool junctionEnabled=false;
    double junctionTimeoutSeconds=60;
};
struct GridlockEscapeSettings {
    bool enabled=false;
    // overlapAfterSeconds is retained for source compatibility, never enables overlap.
    double afterSeconds=15,overlapAfterSeconds=3,speed=1.5;
};
struct TrafficRecoverySettings {
    bool enabled=true;
    double stopSeconds=1;
    double radius=25;
};
struct UrbanTrafficSettings {
    bool enabled=false;
    double motorbikeHeadway=.65,otherHeadway=1.1;
    double predictionSeconds=1.2,filterSpeed=3,lateralAcceleration=1.8;
};
struct TugSettings {
    bool enabled=false,alwaysOn=false;
    double afterStopSeconds=2,strengthNewtons=800,damping=1.5;
    double maximumSpeed=3,lookAhead=3,arrivalRadius=.5;
};
struct TurnNegotiationSettings {
    bool enabled=false;
    double creepSpeed=2,lookAhead=12,noProgressTimeout=2;
};
enum class StopReason { Moving, Starting, Arrived, Signal, Vehicle, Road, Yielding, Unexplained, Deadlock, ManeuverYield, BlockedExit, PriorityYield };
const char* stopReasonName(StopReason reason);
std::vector<Cell> route(const Map&,Cell start,Cell goal,std::mt19937&,double variation=2);
enum class VehicleKind { Bike,Motorbike,Car,Truck,Bus,Count };
constexpr int vehicleKindCount=static_cast<int>(VehicleKind::Count);
const char* vehicleName(VehicleKind kind);
double defaultVehicleMass(VehicleKind kind);
struct Gaussian { double mean=1,stdDev=0; };
struct MotionDistribution {
    double speedMean=8,speedStdDev=1.5;
    double accelerationMean=2,accelerationStdDev=0.4;
};
struct VehicleProfile {
    // distance is a deprecated legacy-comparison field; native following uses minGap/tau.
    Gaussian length,width,distance;
    MotionDistribution motion;
    double weight=1;
    double minGap=.7,tau=1,deceleration=4.5,emergencyDeceleration=9,lateralMinGap=.1;
};
struct TurnControl { Vec2 point; double radius=12,braking=3; };
struct LaneChangeSection {
    double begin=0,end=0; // Meters along this directed lane.
    bool allowEnter=false,allowExit=false;
};
struct DirectedLane {
    std::vector<Vec2> points;
    double width=3.5,speedLimit=8;
    bool entry=true;
    std::vector<int> successors;
    double turnDeceleration=3;
    double spawnRate=1;
    std::vector<double> pointBraking;
    double turnSmoothing=1;
    bool inferConnections=true;
    int roadGroup=-1,travelDirection=0,openJunction=-1;
    int roadLane=0; // Left-to-right index within this travel direction.
    Vec2 junctionCenter;
    double junctionRadius=0;
    std::vector<double> pointWidths;
    bool allowTripEnd=true,destination=false;
    bool explicitTurns=false;
    std::vector<TurnControl> turnControls;
    int sourceNode=-1,destinationNode=-1;
    bool excludeOwnDestination=false;
    int internalJunction=-1; // Fixed SUMO-style internal connector, not free space.
    std::vector<LaneChangeSection> changeSections;
    int movementPriority=1;
    bool semanticTransfer=false;
};
std::array<VehicleProfile,vehicleKindCount> defaultProfiles();
std::array<VehicleProfile,vehicleKindCount> vietnameseCityProfiles();
struct PathManeuver {
    std::size_t begin=0,end=0;
    std::vector<Vec2> sweep;
    double padding=0;
};
struct MotionPath {
    std::vector<Vec2> points;
    std::vector<double> headings;
    std::vector<PathManeuver> turns;
};
// Rounded circular fillets, sampled at <= 5cm / one degree; initial
// replans use a tangent-continuous Bezier connector from the current pose.
MotionPath curvePath(const Map& map,const Footprint& pose,const std::vector<Vec2>& waypoints,double initialConnectorLimit=0,const std::vector<double>& smoothing={});
struct Entity {
    double minGap=.7,tau=1,deceleration=4.5,emergencyDeceleration=9,lateralMinGap=.1;
    bool gapCompressed=false,wasGapCompressed=false,emergencyBraking=false,wasEmergencyBraking=false,contactLimited=false,wasContactLimited=false;
    bool approachBlocked=false;
    double approachCheckAfter=0;
    int internalJunction=-1;
    bool escapingGridlock=false,escapeOverlap=false;
    double escapeStarted=0;
    Vec2 escapeAnchor;
    bool onJunction() const { return inOpenJunction || internalJunction>=0; }
    int spawnLane=-1,spawnNode=-1; // Immutable trip origin; current lane may change.
    double sublaneAfter=0,lateralOffset=0;
    std::string sublaneStatus="centered";
    std::vector<PathManeuver> sublaneReservation;
    std::uint64_t sublaneReservationRevision=0;
    std::size_t sublaneReservationEnd=0;
    double sublaneReservationProgress=0;
    std::vector<int> networkRoute;
    std::size_t networkStage=0;
    bool inOpenJunction=false,waitingForJunction=false,planningExitGate=false;
    bool junctionEntryCleared=false; // A continuous handoff prefix has cleared the entry row.
    int activeJunction=-1;
    Vec2 junctionCenter,exitDirection;
    double junctionRadius=0,laneChangeAfter=0,junctionEnteredAt=0,junctionRetryAfter=0;
    std::uint64_t laneChanges=0,junctionCrossings=0;
    bool changingLanes=false;
    int changingFrom=-1,changingTo=-1;
    Vec2 laneChangeEnd,laneChangeDirection;
    double cooperativeUntil=0,cooperativeSpeedCap=50;
    std::uint64_t cooperativeFor=0;
    int cooperativeLane=-1;
    std::string laneChangeStatus="not evaluated";
    std::vector<int> laneRoute;
    std::vector<Footprint> laneEnvelope;
    std::uint64_t id=0;
    VehicleKind kind=VehicleKind::Car;
    Vec2 center,goal;
    int destinationFeature=-1;
    Vec2 destinationPoint;
    double heading=0,length=4.5,width=1.8,preferredDistance=1;
    double desiredSpeed=8,acceleration=2,speed=0;
    double massKg=1400,angularVelocity=0;
    Vec2 tugVelocity,tugStart;
    double tugSeconds=0,tugSuppressedUntil=0;
    bool tugActive=false;
    std::uint64_t routeRevision=0,yieldToVehicle=0,exitWaitFor=0;
    double turnRequestSince=-1,turnRetryAfter=0;
    bool clearingTurn=false;
    std::string junctionStatus;
    bool turnExitBlocked=false;
    bool priorityYielding=false;
    double priorityYieldSince=-1;
    std::size_t blockedTurnEntry=0;
    double stoppedSeconds=0,currentStopSeconds=0,longestStopSeconds=0,ageSeconds=0;
    double junctionStationarySeconds=0;
    double stuckSeconds=0;
    double movingSinceRefresh=0;
    bool cooperativeRecovery=false;
    bool deadlocked=false;
    double trafficRecoveryAfter=0;
    std::uint64_t destinationRefreshes=0;
    std::vector<Vec2> path;
    std::vector<double> pathHeadings;
    std::vector<PathManeuver> maneuvers;
    double segmentProgress=0;
    std::size_t reservationEnd=0;
    std::size_t next=1;
    double blockedSeconds=0,replanAfter=0;
    bool waitingForLight=false;
    bool waitingForManeuver=false;
    unsigned replanFailures=0;
    Vec2 progressAnchor,yieldTarget;
    double noProgressSeconds=0,recoveryAfter=0;
    bool reversing=false;
    MotionPath recoveryExit;
    MotionPath admissionPlan;
    // Indexed by networkRoute stage; only free-junction crossings need a plan.
    std::vector<MotionPath> junctionPlans;
    double localSteerAfter=0;
    int detourSide=1;
    bool usingLocalDetour=false,following=false;
    bool signalQueue=false,maneuverQueue=false;
    Vec2 detourMerge,detourExitDirection;
    int signalLight=-1; // -1: normal route, -2: removed light, finish approach then exit
    int stopTargetLight=-1;
    Vec2 signalTarget,signalForward,signalA,signalB;
    double signalStopHeading=0,signalRemaining=0,signalRetryAfter=0;
    MotionPath signalExit;
    std::vector<Vec2> reservedTurn;
    std::vector<Vec2> reservedApproach;
    Vec2 turnCenter;
    double turnHeading=0,turnPadding=0;
    Footprint footprint() const { return {center,heading,length,width}; }
    bool reservesSublaneGap() const {
        return !sublaneReservation.empty() && sublaneReservationRevision==routeRevision &&
            (next<sublaneReservationEnd || (next==sublaneReservationEnd && segmentProgress<sublaneReservationProgress-1e-9));
    }
};
enum class LightPhase { Green,Yellow,Red };
struct StopStatistics {
    std::uint64_t vehicles=0,vehiclesThatStopped=0;
    double totalSeconds=0,meanSeconds=0,minSeconds=0,maxSeconds=0;
};
struct QueueDetector {
    Vec2 a,b;
    std::uint64_t id=0;
};
struct QueueDetectorStatus {
    std::uint64_t id=0;
    bool occupied=false;
    double occupiedPercent=0,observedSeconds=0;
};
struct TrafficLightQueueStatus {
    std::uint64_t lightId=0;
    int qualityLine=0; // 1-based highest qualifying line; 0 means none.
    std::vector<QueueDetectorStatus> lines;
};
struct TrafficLight {
    Vec2 a,b; // Finite oriented stop line in meters. Only outDirection is controlled.
    double greenSeconds=10,yellowSeconds=2,redSeconds=12,offsetSeconds=0;
    bool enabled=true;
    bool permissive=false;
    Vec2 allowedA,allowedB; // Linked route-crossing gate; not a physical barrier.
    std::uint64_t id=0;
    double queueWindowSeconds=30,queueThresholdPercent=50;
    std::vector<QueueDetector> queueDetectors; // Order: nearest to farthest upstream.
    Vec2 outDirection; // Controlled travel direction; the reverse direction is always permitted by this light.
    int anchorNode=-1; // Optional road-station node. Permission gates remain separate from destinations.
    Vec2 controlledDirection() const;
    bool controlsDirection(Vec2 travel) const;
    bool controlsMovement(Vec2 from,Vec2 to) const;
    LightPhase phase(double simulationSeconds) const;
};
class Simulation {
    friend struct JunctionPriorityTestAccess;
    friend class SumoBackend;
    class SumoBackend* sumoBackend_=nullptr; // Non-owning; attached by the application.
public:
    explicit Simulation(Map map,std::uint32_t seed=1);
    bool spawn(Cell start,Cell goal,double variation=2,VehicleKind kind=VehicleKind::Car);
    bool spawnAt(Vec2 start,Vec2 goal,VehicleKind kind=VehicleKind::Car,double variation=2);
    bool spawnRandom(double variation=2);
    void setLanes(std::vector<DirectedLane> lanes);
    void saveLanes(const std::string& file) const;
    void setLaneTurnDeceleration(int lane,double value);
    double laneTurnDeceleration(const Entity& entity) const;
    void setPointBraking(Vec2 point,double value);
    void setTurnControls(std::vector<TurnControl> controls);
    bool spawnRandomOnLane(int lane);
    double laneMergeSpeed(const Entity& entity,double requested) const;
    std::uint64_t totalLaneChanges() const { return totalLaneChanges_; }
    std::uint64_t laneChangeAttempts() const { return laneChangeAttempts_; }
    std::uint64_t cooperativeGapRequests() const { return cooperativeGapRequests_; }
    double laneWidthAt(int lane,Vec2 point) const;
    void loadLanes(const std::string& file);
    const std::vector<DirectedLane>& lanes() const { return lanes_; }
    bool spawnOnLane(int lane,VehicleKind kind=VehicleKind::Car,int destinationLane=-1);
    void setSourceDestinationPolicy(int sourceNode,bool excludeOwnDestination);
    void tick(double seconds=1);
    void setMotionDistribution(MotionDistribution distribution);
    void setProfile(VehicleKind kind,const VehicleProfile& profile);
    std::array<double,vehicleKindCount> spawnPercentages() const;
    void setSpawnPercentages(const std::array<double,vehicleKindCount>& percentages);
    const VehicleProfile& profile(VehicleKind kind) const;
    void resampleMotion();
    bool setEntityMotion(std::uint64_t id,double desiredSpeed,double acceleration);
    bool setEntityMass(std::uint64_t id,double massKg);
    void setTugSettings(TugSettings settings);
    const TugSettings& tugSettings() const { return tugSettings_; }
    std::uint64_t tugContacts() const { return tugContacts_; }
    void setTurnNegotiationSettings(TurnNegotiationSettings settings);
    const TurnNegotiationSettings& turnNegotiationSettings() const { return turnNegotiation_; }
    bool ownsNegotiatedTurn(const Entity& entity) const;
    std::uint64_t maneuverYieldTo(const Entity& entity) const;
    std::uint64_t negotiatedTurns() const { return negotiatedTurns_; }
    void setTrafficLights(std::vector<TrafficLight> lights);
    void saveTrafficLights(const std::string& file) const;
    void loadTrafficLights(const std::string& file);
    void setTrafficFlowSettings(TrafficFlowSettings settings);
    const TrafficFlowSettings& trafficFlowSettings() const { return flow_; }
    std::uint64_t junctionAdmissions() const { return junctionAdmissions_; }
    const std::vector<TrafficLight>& trafficLights() const { return lights_; }
    bool routePermitsRed(const Entity& entity,const TrafficLight& light) const;
    int closestStoppingLight(const Entity& entity,double* routeDistance=nullptr,Vec2* crossingPoint=nullptr) const;
    void planSignalQueues(bool force=false);
    std::uint64_t signalPlanningBatches() const { return signalPlanningBatches_; }
    std::uint64_t lastSignalPlanCount() const { return lastSignalPlanCount_; }
    double lastSignalPlanningMilliseconds() const { return lastSignalPlanningMilliseconds_; }
    double totalSignalPlanningSeconds() const { return totalSignalPlanningSeconds_; }
    void planStoppedTraffic(bool force=false);
    void setUrbanTrafficSettings(UrbanTrafficSettings settings);
    const UrbanTrafficSettings& urbanTrafficSettings() const { return urbanTraffic_; }
    StopReason stopReason(const Entity& entity) const;
    std::uint64_t lastFleetRouteCount() const { return lastFleetRouteCount_; }
    void setTrafficRecoverySettings(TrafficRecoverySettings settings);
    const TrafficRecoverySettings& trafficRecoverySettings() const { return trafficRecoverySettings_; }
    std::uint64_t trafficRecoveryPasses() const { return trafficRecoveryPasses_; }
    std::uint64_t trafficRecoveryRepairs() const { return trafficRecoveryRepairs_; }
    double lastTrafficRecoveryMilliseconds() const { return lastTrafficRecoveryMilliseconds_; }
    double totalTrafficRecoverySeconds() const { return totalTrafficRecoverySeconds_; }
    const Map& map() const { return map_; }
    const std::vector<Entity>& entities() const { return entities_; }
    std::uint64_t completed() const { return completed_; }
    std::uint64_t ticks() const { return ticks_; }
    std::uint64_t replans() const { return replans_; }
    std::uint64_t recoveries() const { return recoveries_; }
    double seconds() const { return seconds_; }
    void setRoutingSettings(RoutingSettings settings);
    const RoutingSettings& routingSettings() const { return routing_; }
    const Navigation& navigation() const { return navigation_; }
    std::uint64_t localDetours() const { return localDetours_; }
    std::uint64_t globalPlans() const { return globalPlans_; }
    std::uint64_t signalPlacements() const { return signalPlacements_; }
    static constexpr double stoppedSpeedThreshold=.05;
    StopStatistics stopStatistics(bool includeCompleted=true) const;
    void setCollisionWorkers(unsigned workers);
    unsigned collisionWorkers() const { return collisionWorkers_; }
    static unsigned maximumCollisionWorkers();
    std::uint64_t parallelCollisionBatches() const { return parallelCollisionBatches_; }
    void setStuckVehicleSettings(StuckVehicleSettings settings);
    const StuckVehicleSettings& stuckVehicleSettings() const { return stuckVehicleSettings_; }
    std::uint64_t removedStuckVehicles() const { return removedStuckVehicles_; }
    std::uint64_t removedJunctionVehicles() const { return removedJunctionVehicles_; }
    std::uint64_t removedQueueBlockers() const { return removedStuckVehicles_-removedJunctionVehicles_; }
    std::size_t openJunctionVehicles() const;
    TrafficLightQueueStatus trafficLightQueueStatus(std::uint64_t lightId) const;
    std::uint64_t lastRemovedStuckVehicle() const { return lastRemovedStuckVehicle_; }
    void setGridlockEscapeSettings(GridlockEscapeSettings settings);
    const GridlockEscapeSettings& gridlockEscapeSettings() const { return gridlockEscape_; }
    std::uint64_t gridlockEscapes() const { return gridlockEscapes_; }
    std::uint64_t laneGridlockEscapes() const { return laneGridlockEscapes_; }
    std::uint64_t internalGridlockEscapes() const { return internalGridlockEscapes_; }
    std::uint64_t freeGridlockEscapes() const { return freeGridlockEscapes_; }
    std::uint64_t overlapEscapes() const { return overlapEscapes_; }
    std::uint64_t escapeOverlapPairs() const { return escapeOverlapPairs_; }
    void setCarFollowingSettings(CarFollowingSettings settings) { carFollowing_=settings; }
    const CarFollowingSettings& carFollowingSettings() const { return carFollowing_; }
    double movementGap(const Entity& a,const Entity& b) const { return carFollowing_.enabled?0:std::min(a.preferredDistance,b.preferredDistance); }
    double routeBuffer(const Entity& a) const { return carFollowing_.enabled?0:a.preferredDistance; }
    std::uint64_t compressedGapEvents() const { return compressedGapEvents_; }
    std::uint64_t emergencyBrakeEvents() const { return emergencyBrakeEvents_; }
    std::uint64_t contactLimitEvents() const { return contactLimitEvents_; }
    std::uint64_t physicalCollisionEvents() const { return physicalCollisionEvents_; }
    static double brakingDistance(double speed,double deceleration,double step);
    static double safeFollowingSpeed(double netGap,double leaderSpeed,double leaderDeceleration,double deceleration,double tau,double step);
    bool laneChangeAllowed(int from,int to,Vec2 start,Vec2 end,double bodyLength=0) const;
    std::uint64_t destinationRefreshes() const { return destinationRefreshes_; }
    std::uint64_t destinationRefreshAttempts() const { return destinationRefreshAttempts_; }
private:
    double sample(Gaussian distribution,double minimum,double maximum);
    void sampleMotion(Entity& entity);
    bool fitsLaneRoute(const Footprint&,const Entity&) const;
    void advance(double seconds);
    void clearStuckVehicles();
    void updateTrafficMonitoring(double seconds);
    void updateGridlockEscapes();
    void recordEscapeOverlaps();
    void prepareDrivingSnapshot(double dt);
    void prepareApproachSpeeds();
    double carFollowingSpeed(Entity& entity) const;
    double clearMovementFraction(const Footprint& from,const Footprint& to,const Entity& entity,bool reservations) const;
    void recordDrivingStatistics();
    struct DrivingObservation { std::uint64_t id; Footprint body; double speed,deceleration; bool signal,maneuver; int lane=-1,exit=-1,junction=-1; };
    std::vector<DrivingObservation> drivingSnapshot_;
    double drivingStep_=.01;
    CarFollowingSettings carFollowing_;
    std::uint64_t compressedGapEvents_=0,emergencyBrakeEvents_=0,contactLimitEvents_=0,physicalCollisionEvents_=0;
    std::vector<std::pair<std::uint64_t,std::uint64_t>> previousPhysicalCollisions_;
    void reconcileQueueDetectors();
    struct DetectorSample { double seconds; bool occupied; };
    struct DetectorHistory {
        std::uint64_t lightId=0,id=0;
        Vec2 a,b;
        double window=0,threshold=0,observed=0,occupiedTime=0;
        bool occupied=false;
        std::deque<DetectorSample> samples;
    };
    std::vector<DetectorHistory> detectorHistories_;
    bool refreshMovingRoute();
    void recordFinishedStopTime(const Entity& entity);
    bool feasible(const Footprint& from,const Footprint& to,const Entity& entity,bool traffic,bool reservations=true) const;
    bool blockedByManeuver(const Footprint& from,const Footprint& to,const Entity& entity) const;
    bool reserveTurn(Entity& entity,const Footprint& from,const Footprint& to);
    bool installPath(Entity& entity,std::vector<Vec2> path,double initialConnectorLimit=0);
    bool installNetworkStage(Entity& entity,std::size_t stage);
    bool prepareJunctionPlans(Entity& entity);
    bool advancePredictiveVehicle(Entity& entity,double dt);
    bool predictiveAdmission(const Entity& entity,int link,const std::vector<Vec2>& movement) const;
    bool parallelDeparture(const Entity& a,const Entity& b) const;
    void updateSublanes();
    bool advanceNetworkStage(Entity& entity);
    void updateLaneChanges();
    bool junctionAdmission(const Entity& entity,int link,const std::vector<Vec2>& movement) const;
    bool reserveManeuver(Entity& entity,const PathManeuver& maneuver);
    bool beginRecovery(Entity& entity,bool shortYield=false);
    void preparePriorityYields();
    std::uint64_t priorityGrant_=0;
    double priorityCheckAfter_=0;
    bool blockedByLight(const Footprint& from,const Footprint& to,const Entity& entity) const;
    Footprint stopProbe(const Entity& entity) const;
    bool lightStops(const TrafficLight& light,const Entity& entity) const;
    bool localDetour(Entity& entity);
    double followingSpeed(Entity& entity) const;
    double urbanSpeedLimit(const Entity& entity,double requestedSpeed,bool& brakingForBend,double* selectedBraking=nullptr) const;
    bool advanceTug(Entity& entity,double seconds);
    void activateTug(Entity& entity);
    struct TurnPiece { std::vector<Vec2> polygon; double padding=0,distance=0; };
    struct TurnClaim {
        std::uint64_t owner=0,revision=0;
        std::size_t clearIndex=0;
        double clearProgress=0,entryHeading=0,lastProgress=0,created=0;
        Vec2 origin,anchor;
        std::vector<TurnPiece> pieces;
    };
    std::vector<TurnPiece> turnPieces(const Entity&,std::size_t end,double distance) const;
    const Entity* turnOwner(const TurnClaim&) const;
    bool sameTurnStream(const Entity&,const TurnClaim&) const;
    bool insideTurn(const Footprint&,const Entity&,const TurnClaim&) const;
    bool turnYieldBlocks(const Footprint&,const Footprint&,const Entity&) const;
    double negotiatedSpeed(const Entity&,double requested) const;
    void updateTurnNegotiation();
    bool spreadAtSignal(Entity& entity);
    bool followRouteToSignal(Entity& entity,int light,bool queueBehind=false);
    bool releaseSignalSlot(Entity& entity);
    std::vector<Vec2> plan(const Entity& entity,double variation,bool traffic);
    Map map_;
    Navigation navigation_;
    RoutingSettings routing_;
    std::mt19937 random_,motionRandom_;
    std::array<VehicleProfile,vehicleKindCount> profiles_=defaultProfiles();
    std::vector<Entity> entities_;
    std::vector<DirectedLane> lanes_;
    std::uint64_t totalLaneChanges_=0,laneChangeAttempts_=0,cooperativeGapRequests_=0;
    std::vector<TrafficLight> lights_;
    std::vector<LightPhase> signalPhases_;
    bool planningSignalBatch_=false;
    int planningSignalLight_=-1;
    Vec2 planningSignalForward_;
    std::uint64_t signalPlanningBatches_=0,lastSignalPlanCount_=0;
    double lastSignalPlanningMilliseconds_=0,totalSignalPlanningSeconds_=0;
    TrafficRecoverySettings trafficRecoverySettings_;
    UrbanTrafficSettings urbanTraffic_;
    TugSettings tugSettings_;
    std::uint64_t tugContacts_=0;
    TurnNegotiationSettings turnNegotiation_;
    std::vector<TurnClaim> turnClaims_;
    double nextTurnCheck_=0;
    std::uint64_t negotiatedTurns_=0;
    bool recoveryPreview_=false;
    TrafficFlowSettings flow_;
    std::uint64_t junctionAdmissions_=0;
    std::uint64_t nextLightId_=1;
    double nextTrafficRecoveryCheck_=0,totalTrafficRecoverySeconds_=0,lastTrafficRecoveryMilliseconds_=0;
    std::uint64_t trafficRecoveryPasses_=0,trafficRecoveryRepairs_=0;
    std::uint64_t lastFleetRouteCount_=0;
    std::vector<Cell> spawns_,destinations_;
    std::uint64_t nextId_=1,completed_=0,ticks_=0,replans_=0,recoveries_=0;
    std::uint64_t localDetours_=0,globalPlans_=0;
    std::uint64_t signalPlacements_=0;
    unsigned collisionWorkers_=maximumCollisionWorkers();
    mutable std::uint64_t parallelCollisionBatches_=0;
    double completedStopTotal_=0,completedStopMin_=0,completedStopMax_=0;
    std::uint64_t completedThatStopped_=0;
    StuckVehicleSettings stuckVehicleSettings_;
    GridlockEscapeSettings gridlockEscape_;
    std::uint64_t gridlockEscapes_=0,laneGridlockEscapes_=0,internalGridlockEscapes_=0,freeGridlockEscapes_=0,overlapEscapes_=0,escapeOverlapPairs_=0;
    std::vector<std::pair<std::uint64_t,std::uint64_t>> previousEscapeOverlaps_;
    std::uint64_t removedStuckVehicles_=0,lastRemovedStuckVehicle_=0;
    std::uint64_t removedJunctionVehicles_=0;
    double nextStuckCheck_=0,removeStuckAfter_=0;
    std::uint64_t destinationRefreshes_=0,destinationRefreshAttempts_=0;
    double seconds_=0;
};
}
