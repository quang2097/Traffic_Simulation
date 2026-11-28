#pragma once
#include "traffic/Simulation.hpp"
#include <string>
#include <vector>

namespace traffic {
struct ResultContext {
    std::string runId,label,notes,engine="Native",controller="Fixed timings",mapPath,networkPath;
    std::string nodeDemand,signalTimings;
    int seed=42,maximumVehicles=100;
    bool automaticSpawning=true;
    std::uint64_t acceptedSpawnRequests=0,sumoCollisions=0,sumoTeleports=0,sumoHandoffs=0;
    std::size_t pending=0;
};
struct SimulationResult {
    std::vector<std::string> columns,values;
};
std::string newSimulationRunId();
SimulationResult captureSimulationResult(const Simulation& simulation,const ResultContext& context);
// Creates a CSV or appends one snapshot after validating the existing schema.
void appendSimulationResult(const std::string& path,const SimulationResult& result);
}
