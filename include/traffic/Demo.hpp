#pragma once
#include "traffic/Simulation.hpp"
#include "traffic/NodeGraph.hpp"
namespace traffic { Map makeDemoMap(); std::vector<DirectedLane> makeDemoLanes(); }
namespace traffic { Map makeJunctionDemoMap(); NodeGraph makeJunctionDemoGraph(); }
