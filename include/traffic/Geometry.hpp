#pragma once
#include "traffic/Simulation.hpp"
namespace traffic::geometry {
using Polygon=std::vector<Vec2>;
Polygon convexHull(Polygon points);
double distance(const Polygon& a,const Polygon& b);
bool closerThan(const Polygon& a,const Polygon& b,double clearance);
bool violatesClearance(const Polygon& a,const Polygon& b,double clearance);
bool motionClearance(const Footprint& from,const Footprint& to,const Footprint& obstacle,double clearance,bool allowSeparating=false);
Polygon swept(const Footprint& from,const Footprint& to,double& padding);
bool fits(const Map& map,const Polygon& polygon,double padding=0);
bool intersectsLine(const Polygon& polygon,Vec2 a,Vec2 b);
}
