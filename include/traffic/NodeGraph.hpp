#pragma once
#include "traffic/Simulation.hpp"
#include <string>
namespace traffic {
constexpr int maximumRoadLanes=32;
struct RoadNode { Vec2 position; bool spawn=false; double rate=1,braking=3; bool signal=false; double openRadius=0; bool destination=false; bool turn=false; double turnRadius=12; bool excludeOwnDestination=false; bool allowUTurn=false; double internalRadius=0; bool roadStation=false; };
struct LaneExit { int node=-1; std::vector<int> lanes; std::vector<int> priorities; int road=-1,direction=0; }; // Explicit road targets use node as the lane entry; -1 retains legacy end-node addressing.
struct LaneTurnRule { int direction=0,lane=0; std::vector<LaneExit> exits; }; // An empty explicit list closes this lane's exits.
struct RoadLaneSection {
    double begin=0,end=10; // Meters from the authored road's from-node.
    int direction=-1,lane=-1; // -1: both directions / all lanes; otherwise 0-based.
    bool allowEnter=false,allowExit=false;
};
struct RoadConnection {
    int from=0,to=0; double width=3.5,speed=8,smoothing=1;
    int roadType=0,laneCount=1; // 0: legacy centerline, 1: one-way road, 2: two-way road
    std::vector<Vec2> shape;
    int reverseLaneCount=0;
    double lateralOffset=0,endWidth=0;
    std::vector<double> forwardWidths,reverseWidths;
    std::vector<double> shapeWidths;
    std::vector<RoadLaneSection> changeSections;
    std::vector<LaneTurnRule> laneTurns;
};
struct NodeGraph {
    std::vector<RoadNode> nodes;
    std::vector<RoadConnection> connections;
    bool explicitDestinations=false;
    void useExplicitDestinations();
    int addNode(Vec2 position);
    int connection(int from,int to) const;
    int pathConnection(int from,int to) const;
    void connect(int from,int to,double width=3.5,double speed=8);
    void sever(int from,int to);
    int splitRoad(int road,Vec2 position);
    int addUTurnOpening(int road,double begin,double end);
    bool extendSelection(std::vector<int>& path,int node) const;
    std::vector<DirectedLane> lanes() const;
    static NodeGraph fromLanes(const std::vector<DirectedLane>& lanes);
    void save(const std::string& file) const;
    static NodeGraph load(const std::string& file);
    int junctionAt(int node) const;
    int directionalLaneCount(int road,int direction) const;
    const LaneTurnRule* laneTurnRule(int road,int direction,int lane) const;
    bool turnAllowed(int incomingRoad,int incomingDirection,int incomingLane,int outgoingRoad,int outgoingDirection,int outgoingLane) const;
    void setLaneTurnRule(int road,LaneTurnRule rule);
    void validateLaneTurns() const;
    void pruneLaneTurns();
    std::vector<std::string> missingLaneExits() const;
    int turnPriority(int road,int direction,int lane,int targetNode,int targetLane) const;
    std::vector<std::pair<int,int>> targetRoads(int sourceRoad,int sourceDirection,const LaneExit& exit) const;
};
struct CompiledRoads { std::vector<DirectedLane> lanes; std::vector<int> originNodes; };
struct RoadSignalGeometry { Vec2 a,b,outDirection; };
RoadSignalGeometry roadSignalAtNode(const NodeGraph& graph,int road,int node);
std::vector<int> parseNumberList(const std::string& text,int minimum,int maximum);
CompiledRoads compileRoads(const NodeGraph& graph);
void validateRoadMask(const std::vector<DirectedLane>& lanes,const Map& map);
}
