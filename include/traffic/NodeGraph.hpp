#pragma once
#include "traffic/Simulation.hpp"
#include <string>
namespace traffic {
struct RoadNode { Vec2 position; bool spawn=false; double rate=1,braking=3; bool signal=false; double openRadius=0; bool destination=false; bool turn=false; double turnRadius=12; bool excludeOwnDestination=false; bool allowUTurn=false; };
struct RoadConnection {
    int from=0,to=0; double width=3.5,speed=8,smoothing=1;
    int roadType=0,laneCount=1; // 0: legacy centerline, 1: one-way road, 2: two-way road
    std::vector<Vec2> shape;
    int reverseLaneCount=0;
    double lateralOffset=0,endWidth=0;
    std::vector<double> forwardWidths,reverseWidths;
    std::vector<double> shapeWidths;
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
    bool extendSelection(std::vector<int>& path,int node) const;
    std::vector<DirectedLane> lanes() const;
    static NodeGraph fromLanes(const std::vector<DirectedLane>& lanes);
    void save(const std::string& file) const;
    static NodeGraph load(const std::string& file);
};
struct CompiledRoads { std::vector<DirectedLane> lanes; std::vector<int> originNodes; };
CompiledRoads compileRoads(const NodeGraph& graph);
void validateRoadMask(const std::vector<DirectedLane>& lanes,const Map& map);
}
