#pragma once
#include "traffic/NodeGraph.hpp"
#include <memory>
#include <string>

namespace traffic {
// Actual Eclipse SUMO owns all road traffic. Native movement is used only
// between the entry and exit gates of an explicitly open junction.
class SumoBackend {
public:
    SumoBackend(Simulation& view,const NodeGraph& graph,unsigned seed,
                const std::string& runtime="",const std::string& project="");
    ~SumoBackend();
    SumoBackend(const SumoBackend&)=delete;
    SumoBackend& operator=(const SumoBackend&)=delete;
    bool spawn(int compiledLane,VehicleKind kind,int destinationLane=-1);
    bool deleteVehicle(std::uint64_t id);
    void step(double seconds);
    std::string status() const;
    std::uint64_t collisions() const;
    std::uint64_t teleports() const;
    std::uint64_t handoffs() const;
    std::size_t pending() const;
    std::vector<Vec2> routePreview(std::uint64_t id) const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
