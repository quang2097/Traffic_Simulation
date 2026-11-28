#pragma once
#include "traffic/Simulation.hpp"
#include <array>

namespace traffic {
constexpr int signalAgentCount=2;
struct SignalRegion {
    std::uint64_t lightId=0;
    int group=0;
    double depth=30,width=0; // Meters upstream; zero width follows the stop line.
};
struct SignalDensity {
    std::uint64_t lightId=0;
    int group=0,vehicles=0;
    double density=0,area=0,occupiedArea=0;
    std::vector<Vec2> polygon;
};
struct SignalObservation {
    std::array<double,2> density{},mask{};
    std::vector<SignalDensity> regions;
};
std::vector<SignalRegion> defaultSignalRegions(const std::vector<TrafficLight>& lights);
SignalObservation observeSignals(const std::vector<TrafficLight>& lights,const std::vector<Entity>& vehicles,const std::vector<SignalRegion>& regions);

// Two agents choose green durations. A common cycle supplies yellow and
// all-red clearance and keeps every signal in a group in phase.
class SignalTimingController {
public:
    void begin(std::vector<TrafficLight>& lights,const std::vector<SignalRegion>& regions,double now,double yellow=3,double clearance=1);
    void queue(std::array<double,2> green);
    bool update(std::vector<TrafficLight>& lights,double now);
    void stop(std::vector<TrafficLight>& lights);
    bool active() const {return active_;}
    const std::array<double,2>& green() const {return green_;}
    const std::array<double,2>& pending() const {return pending_;}
    double nextCycle() const {return boundary_;}
private:
    std::vector<TrafficLight> original_;
    std::vector<SignalRegion> regions_;
    std::array<double,2> green_{15,15},pending_{15,15};
    double yellow_=3,clearance_=1,boundary_=0;
    bool active_=false;
};
}
