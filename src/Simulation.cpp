#include "traffic/Simulation.hpp"
#include "traffic/Parallel.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>

namespace traffic {
Map::Map(int w, int h, std::vector<Tile> t,double scale) : width(w), height(h), tiles(std::move(t)), metersPerPixel(scale) {
    if (w <= 0 || h <= 0 || static_cast<std::size_t>(w) * h != tiles.size() || !std::isfinite(scale) || scale<0.01 || scale>100)
        throw std::invalid_argument("Invalid map dimensions");
}
bool Map::contains(Cell p) const { return p.x >= 0 && p.y >= 0 && p.x < width && p.y < height; }
int Map::index(Cell p) const { return p.y * width + p.x; }
Cell Map::pixel(Vec2 p) const {
    if(!std::isfinite(p.x) || !std::isfinite(p.y) || p.x<0 || p.y<0 ||
       p.x>=width*metersPerPixel || p.y>=height*metersPerPixel) return {-1,-1};
    return {static_cast<int>(std::floor(p.x/metersPerPixel)),static_cast<int>(std::floor(p.y/metersPerPixel))};
}
Vec2 Map::center(Cell p) const { return {(p.x+0.5)*metersPerPixel,(p.y+0.5)*metersPerPixel}; }
void Map::prepareCollision() {
    wallIntegral_.assign(static_cast<std::size_t>(width+1)*(height+1),0);
    for(int y=0;y<height;++y) {
        int row=0;
        for(int x=0;x<width;++x) {
            row+=!walkable({x,y})?1:0;
            wallIntegral_[static_cast<std::size_t>(y+1)*(width+1)+x+1]=row+wallIntegral_[static_cast<std::size_t>(y)*(width+1)+x+1];
        }
    }
}
bool Map::hasWalls(int x0,int y0,int x1,int y1) const {
    if(wallIntegral_.empty()) {
        for(int y=y0;y<=y1;++y) for(int x=x0;x<=x1;++x) if(!walkable({x,y})) return true;
        return false;
    }
    auto at=[&](int x,int y) { return wallIntegral_[static_cast<std::size_t>(y)*(width+1)+x]; };
    return at(x1+1,y1+1)-at(x0,y1+1)-at(x1+1,y0)+at(x0,y0)>0;
}
bool Map::walkable(Cell p) const { return contains(p) && tiles[index(p)]!=Tile::Wall && tiles[index(p)]!=Tile::Divider; }
std::vector<Cell> Map::neighbors(Cell p) const {
    std::vector<Cell> result;
    for (Cell q : {Cell{p.x+1,p.y}, Cell{p.x-1,p.y}, Cell{p.x,p.y+1}, Cell{p.x,p.y-1}})
        if (walkable(q)) result.push_back(q);
    return result;
}
std::vector<Cell> Map::cells(Tile type) const {
    std::vector<Cell> result;
    for (int y=0; y<height; ++y) for (int x=0; x<width; ++x)
        if (tiles[index({x,y})] == type) result.push_back({x,y});
    return result;
}
std::vector<Cell> route(const Map& map, Cell start, Cell goal, std::mt19937& rng, double variation) {
    if (!std::isfinite(variation) || variation < 0) throw std::invalid_argument("Invalid route variation");
    if (!map.walkable(start) || !map.walkable(goal)) return {};
    std::vector<double> costs(map.tiles.size(), std::numeric_limits<double>::infinity());
    std::vector<int> parent(map.tiles.size(), -1);
    using Entry = std::pair<double,int>;
    std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> queue;
    int source=map.index(start), target=map.index(goal);
    costs[source]=0; queue.push({0,source});
    std::uniform_real_distribution<double> noise(0,variation);
    while (!queue.empty()) {
        auto [cost,i]=queue.top(); queue.pop();
        if (cost != costs[i]) continue;
        if (i == target) break;
        auto adjacent=map.neighbors({i%map.width,i/map.width});
        std::shuffle(adjacent.begin(),adjacent.end(),rng);
        for (Cell p : adjacent) {
            int j=map.index(p);
            double candidate=cost+1+noise(rng);
            if (candidate < costs[j]) { costs[j]=candidate; parent[j]=i; queue.push({candidate,j}); }
        }
    }
    if (!std::isfinite(costs[target])) return {};
    std::vector<Cell> result;
    for (int i=target; i!=-1; i=parent[i]) result.push_back({i%map.width,i/map.width});
    std::reverse(result.begin(),result.end()); return result;
}

namespace {
constexpr double pi=3.141592653589793;
void validate(double value,double low,double high,const char* name) {
    if(!std::isfinite(value) || value<low || value>high) throw std::invalid_argument(name);
}
void validateGaussian(Gaussian g,double low,double high) {
    validate(g.mean,low,high,"Invalid Gaussian mean");
    validate(g.stdDev,0,high,"Invalid Gaussian standard deviation");
}
void validateMotion(MotionDistribution d) {
    validateGaussian({d.speedMean,d.speedStdDev},0.05,50);
    validateGaussian({d.accelerationMean,d.accelerationStdDev},0.05,100);
}
double bearing(Vec2 a,Vec2 b) { return std::atan2(b.y-a.y,b.x-a.x); }
bool finite(Vec2 p) { return std::isfinite(p.x) && std::isfinite(p.y); }
}
const char* vehicleName(VehicleKind kind) {
    static const char* names[]{"Bike","Motorbike","Car","Truck","Bus"};
    int i=static_cast<int>(kind); return i>=0 && i<vehicleKindCount?names[i]:"Unknown";
}
std::array<VehicleProfile,vehicleKindCount> defaultProfiles() {
    return {{ {{1.8,.12},{.6,.05},{.4,.1},{5,1,1,.2},2},
              {{2.1,.15},{.75,.07},{.5,.15},{9,2,2.5,.5},3},
              {{4.5,.4},{1.8,.12},{1.5,.3},{11,2,2,.4},4},
              {{8,1},{2.5,.12},{2,.4},{8,1.5,1,.2},1},
              {{12,.8},{2.5,.1},{2,.4},{7,1,1,.2},1} }};
}
LightPhase TrafficLight::phase(double seconds) const {
    if(!enabled) return LightPhase::Green;
    double cycle=greenSeconds+yellowSeconds+redSeconds;
    double t=std::fmod(seconds+offsetSeconds,cycle); if(t<0) t+=cycle;
    return t<greenSeconds?LightPhase::Green:t<greenSeconds+yellowSeconds?LightPhase::Yellow:LightPhase::Red;
}
Simulation::Simulation(Map map,std::uint32_t seed)
    : map_(std::move(map)),navigation_(map_),random_(seed),motionRandom_(seed^0x9e3779b9u),
      spawns_(map_.cells(Tile::Spawn)),destinations_(map_.cells(Tile::Destination)) {
    map_.prepareCollision(); map_.rebuildFeatures();
    for(const auto& feature:map_.features()) if(feature.type==Tile::Signal && length(feature.b-feature.a)>=.1)
        lights_.push_back({feature.a,feature.b,10,2,12,0,true});
}
const VehicleProfile& Simulation::profile(VehicleKind kind) const {
    int i=static_cast<int>(kind);
    if(i<0 || i>=vehicleKindCount) throw std::invalid_argument("Invalid vehicle class");
    return profiles_[i];
}
void Simulation::setProfile(VehicleKind kind,const VehicleProfile& p) {
    profile(kind);
    validateGaussian(p.length,.2,30); validateGaussian(p.width,.15,5);
    validateGaussian(p.distance,0,20); validateMotion(p.motion); validate(p.weight,0,100,"Invalid vehicle weight");
    profiles_[static_cast<int>(kind)]=p;
}
void Simulation::setMotionDistribution(MotionDistribution d) {
    validateMotion(d); for(auto& p:profiles_) p.motion=d;
}
double Simulation::sample(Gaussian g,double low,double high) {
    double value=g.stdDev==0?g.mean:std::normal_distribution<double>(g.mean,g.stdDev)(motionRandom_);
    return std::clamp(value,low,high);
}
void Simulation::sampleMotion(Entity& e) {
    const auto& d=profile(e.kind).motion;
    e.desiredSpeed=sample({d.speedMean,d.speedStdDev},.05,50);
    e.acceleration=sample({d.accelerationMean,d.accelerationStdDev},.05,100);
    e.speed=std::min(e.speed,e.desiredSpeed);
}
void Simulation::resampleMotion() { for(auto& e:entities_) sampleMotion(e); }
bool Simulation::setEntityMotion(std::uint64_t id,double speed,double acceleration) {
    validateMotion({speed,0,acceleration,0});
    for(auto& e:entities_) if(e.id==id) {
        e.desiredSpeed=speed; e.acceleration=acceleration; e.speed=std::min(e.speed,speed); return true;
    }
    return false;
}
void Simulation::setTrafficLights(std::vector<TrafficLight> lights) {
    std::vector<std::uint64_t> identities;
    for(auto& light:lights) {
        bool automaticIdentity=!light.id;
        if(!light.id) {
            for(const auto& old:lights_) if(length(old.a-light.a)+length(old.b-light.b)<1e-8 && std::find(identities.begin(),identities.end(),old.id)==identities.end()) { light.id=old.id; break; }
            if(!light.id) light.id=nextLightId_++;
        }
        if(std::find(identities.begin(),identities.end(),light.id)!=identities.end()) {
            if(automaticIdentity) light.id=nextLightId_++;
            else throw std::invalid_argument("Traffic lights must have distinct identities");
        }
        identities.push_back(light.id);
        nextLightId_=std::max(nextLightId_,light.id+1);
        if(!finite(light.a) || !finite(light.b) || length(light.b-light.a)<.1 ||
           !map_.contains(map_.pixel(light.a)) || !map_.contains(map_.pixel(light.b)))
            throw std::invalid_argument("Stop line must have valid endpoints inside the map");
        validate(light.greenSeconds,.1,3600,"Invalid green duration");
        validate(light.yellowSeconds,0,3600,"Invalid yellow duration");
        validate(light.redSeconds,.1,3600,"Invalid red duration");
        validate(light.offsetSeconds,0,10800,"Invalid phase offset");
        validate(light.queueWindowSeconds,.1,3600,"Queue window must be 0.1-3600 seconds");
        validate(light.queueThresholdPercent,0,100,"Queue threshold must be 0-100 percent");
        std::uint64_t nextDetector=1;
        for(const auto& detector:light.queueDetectors) nextDetector=std::max(nextDetector,detector.id+1);
        std::vector<std::uint64_t> detectorIds;
        for(auto& detector:light.queueDetectors) {
            if(!detector.id) detector.id=nextDetector++;
            if(std::find(detectorIds.begin(),detectorIds.end(),detector.id)!=detectorIds.end())
                throw std::invalid_argument("Queue detectors must have distinct identities within a light");
            detectorIds.push_back(detector.id);
            if(!finite(detector.a) || !finite(detector.b) || length(detector.b-detector.a)<.1 ||
                !map_.contains(map_.pixel(detector.a)) || !map_.contains(map_.pixel(detector.b)))
                throw std::invalid_argument("Queue detector endpoints must be inside the map and at least 0.1 m apart");
        }
        if(light.permissive && (!finite(light.allowedA) || !finite(light.allowedB) || length(light.allowedB-light.allowedA)<.1 ||
           !map_.contains(map_.pixel(light.allowedA)) || !map_.contains(map_.pixel(light.allowedB))))
            throw std::invalid_argument("Linked exit must have valid endpoints inside the map");
    }
    for(auto& e:entities_) if(e.signalLight!=-1) {
        e.signalLight=-2;
        for(std::size_t i=0;i<lights.size();++i)
            if(length(lights[i].a-e.signalA)<1e-8 && length(lights[i].b-e.signalB)<1e-8) { e.signalLight=static_cast<int>(i); break; }
    }
    for(auto& e:entities_) e.signalRetryAfter=seconds_;
    lights_=std::move(lights);
    reconcileQueueDetectors();
}
bool Simulation::feasible(const Footprint& from,const Footprint& to,const Entity& e,bool traffic,bool reservations) const {
    double pad;
    auto sweep=geometry::swept(from,to,pad);
    if(!geometry::fits(map_,sweep,pad)) return false;
    if(!fitsLaneRoute(from,e) || !fitsLaneRoute(to,e)) return false;
    if(traffic && turnYieldBlocks(from,to,e)) return false;
    auto clear=[&](const Entity& other) {
        if(other.id==e.id) return true;
        double gap=std::min(e.preferredDistance,other.preferredDistance);
        // Desired lateral gaps guide placement; motion enforces the preferred minimum.
        bool sameQueue=planningSignalBatch_ && planningSignalLight_>=0 && other.signalLight==planningSignalLight_ &&
            other.signalForward.x*planningSignalForward_.x+other.signalForward.y*planningSignalForward_.y>.5;
        if(planningSignalBatch_ && planningSignalLight_>=0 && other.signalLight>=0 &&
           geometry::closerThan(sweep,corners({other.signalTarget,other.signalStopHeading,other.length,other.width}),gap+pad+.04)) return false;
        if(sameQueue && other.next<other.path.size() && length(other.signalTarget-other.center)>.25) return true;
        Vec2 claimForward{std::cos(other.heading),std::sin(other.heading)};
        auto beforeClaim=from.center-other.center,afterClaim=to.center-other.center;
        double ahead=beforeClaim.x*claimForward.x+beforeClaim.y*claimForward.y;
        bool clearingAhead=(ahead>.02 || (std::abs(ahead)<=.02 && e.id<other.id)) &&
            std::cos(from.heading-other.heading)>.95 &&
            afterClaim.x*claimForward.x+afterClaim.y*claimForward.y>ahead+1e-10;
        // A following merge must not reserve the leader's escape. Physical
        // body clearance still applies below; only the future claim is ignored.
        if(flow_.sublanes && !clearingAhead && !(e.cooperativeRecovery && !e.laneRoute.empty()) && other.reservesSublaneGap()) for(const auto& piece:other.sublaneReservation)
            if(piece.end>=other.next && geometry::violatesClearance(sweep,piece.sweep,gap+pad+piece.padding)) return false;
        double bound=.5*std::hypot(e.length,e.width)+.5*std::hypot(other.length,other.width)+gap+length(to.center-from.center)+pad;
        if(length(from.center-other.center)<=bound && geometry::violatesClearance(sweep,corners(other.footprint()),gap+pad) &&
           !geometry::motionClearance(from,to,other.footprint(),gap,e.cooperativeRecovery)) return false;
        if(reservations && !other.reservedTurn.empty() &&
           geometry::violatesClearance(sweep,other.reservedTurn,gap+pad+other.turnPadding)) return false;
        if(reservations && !other.reservedApproach.empty() &&
           geometry::violatesClearance(sweep,other.reservedApproach,gap+pad+other.turnPadding)) return false;
        return true;
    };
    if(traffic) {
        // Small fleets use the serial early-out to avoid scheduling overhead.
        if(collisionWorkers_>1 && entities_.size()>=32) {
            std::vector<const Entity*> candidates;
            for(const auto& other:entities_) if(other.id!=e.id) {
                double gap=std::min(e.preferredDistance,other.preferredDistance);
                double bound=.5*std::hypot(e.length,e.width)+.5*std::hypot(other.length,other.width)+gap+length(to.center-from.center)+pad;
                if((planningSignalBatch_ && other.signalLight>=0) || length(from.center-other.center)<=bound || (flow_.sublanes && other.reservesSublaneGap()) ||
                   (reservations && (!other.reservedTurn.empty() || !other.reservedApproach.empty()))) candidates.push_back(&other);
            }
            if(candidates.size()>=8) {
                ++parallelCollisionBatches_;
                return detail::parallelAll(candidates.size(),collisionWorkers_,[&](std::size_t i) { return clear(*candidates[i]); });
            }
            for(const auto* other:candidates) if(!clear(*other)) return false;
            return true;
        }
        for(const auto& other:entities_) if(!clear(other)) return false;
    }
    return true;
}
bool Simulation::blockedByManeuver(const Footprint& from,const Footprint& to,const Entity& e) const {
    if(turnYieldBlocks(from,to,e)) return true;
    double pad; auto sweep=geometry::swept(from,to,pad);
    for(const auto& other:entities_) if(other.id!=e.id && !other.reservedTurn.empty()) {
        double gap=std::min(e.preferredDistance,other.preferredDistance)+pad+other.turnPadding;
        if(geometry::violatesClearance(sweep,other.reservedTurn,gap) ||
           geometry::violatesClearance(sweep,other.reservedApproach,gap)) return true;
    }
    return false;
}
bool Simulation::reserveTurn(Entity& e,const Footprint& from,const Footprint& to) {
    if(!feasible(from,to,e,true)) return false;
    // Reserve the approach as well as the swing: otherwise another car can
    // enter between the owner and its corner and strand both indefinitely.
    if(!feasible(e.footprint(),from,e,true)) return false;
    double padding;
    e.reservedApproach=geometry::swept(e.footprint(),from,padding);
    e.reservedTurn=geometry::swept(from,to,e.turnPadding);
    e.turnPadding=std::max(e.turnPadding,padding);
    e.turnCenter=from.center; e.turnHeading=to.heading;
    return true;
}
bool Simulation::installPath(Entity& e,std::vector<Vec2> points,double initialConnectorLimit) {
    std::vector<double> smoothing(points.size(),1);
    for(std::size_t i=0;i<points.size();++i) {
        for(int id:e.laneRoute) if(length(points[i]-lanes_[id].points.back())<1e-5) { smoothing[i]=lanes_[id].turnSmoothing; break; }
    }
    auto motion=curvePath(map_,e.footprint(),points,initialConnectorLimit,smoothing);
    // Reject a lane trajectory before committing to it, rather than discovering
    // an impossible body rotation only after a queue has formed behind it.
    auto insideLane=[&](const MotionPath& trial) {
        if(trial.points.empty()) return false;
        if(e.networkRoute.empty() || e.inOpenJunction) return true;
        for(std::size_t i=1;i<trial.points.size();++i) {
            double distance=length(trial.points[i]-trial.points[i-1]);
            double angle=angleDelta(trial.headings[i-1],trial.headings[i]);
            int samples=std::max(1,static_cast<int>(std::ceil(std::max(distance/.25,std::abs(angle)/.02))));
            for(int k=0;k<=samples;++k) {
                double f=static_cast<double>(k)/samples;
                Footprint pose{trial.points[i-1]+(trial.points[i]-trial.points[i-1])*f,
                    trial.headings[i-1]+angle*f,e.length,e.width};
                if(!fitsLaneRoute(pose,e)) return false;
            }
        }
        return true;
    };
    if(!insideLane(motion) && !e.networkRoute.empty() && !e.inOpenJunction) {
        for(double factor:{.5,2.,.25,4.}) {
            auto adjusted=smoothing;for(auto& value:adjusted)value*=factor;
            auto trial=curvePath(map_,e.footprint(),points,initialConnectorLimit*factor,adjusted);
            if(insideLane(trial)){motion=std::move(trial);break;}
        }
    }
    if(!insideLane(motion)) return false;
    e.path=std::move(motion.points); e.pathHeadings=std::move(motion.headings); e.maneuvers=std::move(motion.turns);
    e.next=1; e.segmentProgress=0; e.reservedTurn.clear(); e.reservedApproach.clear(); e.reservationEnd=0;
    e.usingLocalDetour=false; e.following=false;
    e.cooperativeRecovery=false;
    ++e.routeRevision;
    e.turnExitBlocked=false;
    e.exitWaitFor=0;
    return true;
}
bool Simulation::reserveManeuver(Entity& e,const PathManeuver& maneuver) {
    std::vector<Vec2> approach=corners(e.footprint());
    for(std::size_t i=e.next;i<=maneuver.begin && i<e.path.size();++i) {
        Footprint pose{e.path[i],e.pathHeadings[i],e.length,e.width}; auto box=corners(pose);
        approach.insert(approach.end(),box.begin(),box.end());
    }
    approach=geometry::convexHull(std::move(approach));
    auto free=[&](const std::vector<Vec2>& polygon,double padding) {
        for(const auto& other:entities_) if(other.id!=e.id) {
            double gap=std::min(e.preferredDistance,other.preferredDistance)+padding;
            if(geometry::violatesClearance(polygon,corners(other.footprint()),gap)) return false;
            gap+=other.turnPadding;
            if(geometry::violatesClearance(polygon,other.reservedTurn,gap) || geometry::violatesClearance(polygon,other.reservedApproach,gap)) return false;
        }
        return true;
    };
    auto turn=maneuver.sweep;
    if(e.next>maneuver.begin) {
        // A red light can pause a committed arc. Reacquire only its remaining
        // sweep after releasing the unused reservation to adjacent queues.
        std::vector<Vec2> remaining=corners(e.footprint());
        for(std::size_t i=e.next;i<=maneuver.end;++i) {
            auto box=corners({e.path[i],e.pathHeadings[i],e.length,e.width});
            remaining.insert(remaining.end(),box.begin(),box.end());
        }
        turn=geometry::convexHull(std::move(remaining));
    }
    if(!free(turn,maneuver.padding) || !free(approach,0)) return false;
    e.reservedTurn=std::move(turn); e.reservedApproach=std::move(approach); e.turnPadding=maneuver.padding;
    e.reservationEnd=maneuver.end; e.turnCenter=e.path[maneuver.end]; e.turnHeading=e.pathHeadings[maneuver.end];
    return true;
}
bool Simulation::blockedByLight(const Footprint& from,const Footprint& to,const Entity& entity) const {
    double padding; auto sweep=geometry::swept(from,to,padding);
    for(const auto& light:lights_) if(lightStops(light,entity) &&
       !geometry::intersectsLine(corners(from),light.a,light.b) &&
       geometry::distance(sweep,{light.a,light.b})<=padding+1e-8) return true;
    return false;
}
bool Simulation::beginRecovery(Entity& e,bool shortYield) {
    if(!e.laneRoute.empty()) return false;
    e.recoveryAfter=seconds_+2;
    auto from=e.footprint();
    Vec2 backward{-std::cos(e.heading),-std::sin(e.heading)};
    double maximum=std::clamp(e.length*.75,1.0,3.0);
    std::array<double,3> distances=shortYield?std::array<double,3>{.35,.75,maximum}:std::array<double,3>{maximum,.75,.35};
    for(double distance:distances) {
        auto to=from; to.center=from.center+backward*distance;
        if(!feasible(from,to,e,true) || blockedByLight(from,to,e)) continue;
        Entity candidate=e; candidate.center=to.center;
        auto exit=plan(candidate,0,shortYield);
        if(exit.empty() || !installPath(candidate,std::move(exit))) continue;
        if(shortYield) {
            // A backup is useful only when the curved departure can clear the
            // neighboring bodies, rather than immediately recreating the knot.
            bool clear=true; auto previous=candidate.footprint(); double checked=0;
            for(std::size_t i=1;i<candidate.path.size() && checked<std::max(3.0,e.length);++i) {
                Footprint pose{candidate.path[i],candidate.pathHeadings[i],e.length,e.width};
                if(!feasible(previous,pose,candidate,true,false)) { clear=false; break; }
                checked+=length(pose.center-previous.center); previous=pose;
            }
            if(!clear) continue;
        }
        e.recoveryExit={std::move(candidate.path),std::move(candidate.pathHeadings),std::move(candidate.maneuvers)};
        e.reservedApproach.clear();
        e.reservedTurn=geometry::swept(from,to,e.turnPadding);
        e.turnCenter=to.center; e.turnHeading=e.heading;
        e.yieldTarget=to.center; e.reversing=true; e.speed=0; e.tugVelocity={}; e.angularVelocity=0;
        ++recoveries_; return true;
    }
    return false;
}
std::vector<Vec2> Simulation::plan(const Entity& e,double variation,bool traffic) {
    if(!e.laneRoute.empty()) return {}; // Lane actors use their directed route and cached signal continuation.
    if(e.inOpenJunction && !e.planningExitGate) {
        Vec2 gate=e.networkStage+1<e.networkRoute.size()?lanes_[e.networkRoute[e.networkStage+1]].points.front():e.goal-e.exitDirection*(e.length*1.5+2);
        if((e.center.x-gate.x)*e.exitDirection.x+(e.center.y-gate.y)*e.exitDirection.y<0) {
            Entity approach=e; approach.goal=gate; approach.planningExitGate=true;
            auto route=plan(approach,variation,traffic);
            if(!route.empty()) route.push_back(e.goal);
            return route;
        }
    }
    validate(variation,0,100,"Invalid route variation");
    if(!navigation_.connected(e.center,e.goal)) return {};
    ++globalPlans_;
    const auto* field=routing_.rtsStyle?&navigation_.goalField(navigation_.regionAt(e.goal)):nullptr;
    std::vector<double> congestion(navigation_.regions().size(),0);
    if(routing_.rtsStyle && routing_.congestionWeight>0) for(const auto& other:entities_) if(other.id!=e.id) {
        int region=navigation_.regionAt(other.center);
        if(region>=0) congestion[region]+=(other.length+other.preferredDistance)*(other.width+other.preferredDistance)/navigation_.regions()[region].area;
    }
    auto heuristic=[&](Vec2 p) {
        double direct=length(e.goal-p); int region=navigation_.regionAt(p);
        // This area guide is deliberately soft: detailed footprint checks can
        // take another corridor when the coarse point graph suggests a gap
        // too small for a truck. No coarse region excludes a valid detailed node.
        if(field && region>=0 && std::isfinite((*field)[region])) return std::max(direct,.5*direct+.5*(*field)[region]);
        return direct;
    };
    auto connect=[&](Vec2 a,double heading,Vec2 b) {
        if(length(a-b)<1e-8) return true;
        double direction=bearing(a,b);
        Footprint from{a,heading,e.length,e.width},turned{a,direction,e.length,e.width},to{b,direction,e.length,e.width};
        return feasible(from,turned,e,traffic,false) && feasible(turned,to,e,traffic,false);
    };
    if(length(e.center-e.goal)<1e-7) return {e.center};
    if(variation==0 && connect(e.center,e.heading,e.goal)) return {e.center,e.goal};
    // The mask is sampled for navigation only. Continuous boxes, never pixel
    // occupancy, decide physical clearance. Limit sampling to at most 0.5m.
    double step=std::min(.5,map_.metersPerPixel);
    double columns=std::ceil(map_.width*map_.metersPerPixel/step),rows=std::ceil(map_.height*map_.metersPerPixel/step);
    if(columns>std::numeric_limits<int>::max() || rows>std::numeric_limits<int>::max()) return {};
    int w=static_cast<int>(columns),h=static_cast<int>(rows);
    auto node=[&](Vec2 p) { return static_cast<std::int64_t>(std::clamp(static_cast<int>(p.y/step),0,h-1))*w+std::clamp(static_cast<int>(p.x/step),0,w-1); };
    auto source=node(e.center),target=node(e.goal);
    auto point=[&](std::int64_t i) { return i==source?e.center:i==target?e.goal:Vec2{(i%w+.5)*step,(i/w+.5)*step}; };
    if(source==target) return connect(e.center,e.heading,e.goal)?std::vector<Vec2>{e.center,e.goal}:std::vector<Vec2>{};
    struct Record { double cost,heading; std::int64_t parent; };
    struct Entry {
        double score,cost; std::int64_t key;
        bool operator>(const Entry& b) const { return score>b.score || (score==b.score && key>b.key); }
    };
    std::unordered_map<std::int64_t,Record> records;
    std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> open;
    std::int64_t first=source*8;
    records.emplace(first,Record{0,e.heading,-1}); open.push({heuristic(e.center),0,first});
    const std::uint32_t salt=random_();
    const int dx[]{1,1,0,-1,-1,-1,0,1},dy[]{0,1,1,1,0,-1,-1,-1};
    std::int64_t found=-1;
    int expanded=0;
    while(!open.empty() && expanded<12000) {
        auto current=open.top(); open.pop();
        auto rec=records.at(current.key);
        if(current.cost!=rec.cost) continue;
        auto i=current.key/8;
        if(i==target) { found=current.key; break; }
        ++expanded;
        Vec2 from=point(i);
        for(int d=0;d<8;++d) {
            int x=static_cast<int>(i%w)+dx[d],y=static_cast<int>(i/w)+dy[d];
            if(x<0 || y<0 || x>=w || y>=h) continue;
            auto j=static_cast<std::int64_t>(y)*w+x;
            if(j==source) continue;
            Vec2 to=point(j);
            if(!map_.walkable(map_.pixel(to))) continue;
            double direction=bearing(from,to);
            double turn=std::abs(angleDelta(rec.heading,direction));
            std::uint32_t noise=static_cast<std::uint32_t>(j)^salt;
            noise^=noise>>16; noise*=0x7feb352du; noise^=noise>>15;
            int region=navigation_.regionAt(to);
            double crowd=region>=0?std::min(4.0,congestion[region]*routing_.congestionWeight):0;
            double cost=rec.cost+length(to-from)*(1+variation*(noise%10000)/10000.0+crowd)+.15*turn;
            auto key=j*8+d;
            auto existing=records.find(key);
            if(existing!=records.end() && existing->second.cost<=cost) continue;
            if(!connect(from,rec.heading,to)) continue;
            records[key]={cost,direction,current.key};
            open.push({cost+(1+variation*.5)*heuristic(to),cost,key});
        }
    }
    if(found<0) return {};
    std::vector<Vec2> path;
    for(auto i=found;i>=0;i=records.at(i).parent) path.push_back(point(i/8));
    std::reverse(path.begin(),path.end());
    // Pull strings through the mask, checking the whole body and turn sweep.
    std::vector<Vec2> smooth{path.front()};
    double heading=e.heading;
    for(std::size_t i=0;i+1<path.size();) {
        std::size_t selected=i+1;
        for(std::size_t j=path.size()-1;j>i+1;--j) {
            if(!connect(path[i],heading,path[j])) continue;
            double arrival=bearing(path[i],path[j]);
            if(j+1<path.size()) {
                Footprint before{path[j],arrival,e.length,e.width},after=before;
                after.heading=bearing(path[j],path[j+1]);
                if(!feasible(before,after,e,traffic,false)) continue;
            }
            selected=j; break;
        }
        heading=bearing(path[i],path[selected]); smooth.push_back(path[selected]); i=selected;
    }
    return smooth;
}
bool Simulation::spawn(Cell start,Cell goal,double variation,VehicleKind kind) {
    if(!map_.walkable(start) || !map_.walkable(goal)) return false;
    return spawnAt(map_.center(start),map_.center(goal),kind,variation);
}
bool Simulation::spawnAt(Vec2 start,Vec2 goal,VehicleKind kind,double variation) {
    validate(variation,0,100,"Invalid route variation");
    const auto& p=profile(kind);
    if(!finite(start) || !finite(goal) || !map_.walkable(map_.pixel(start)) || !map_.walkable(map_.pixel(goal)) || !navigation_.connected(start,goal)) return false;
    // Do not keep filling an entry while a real standstill is being resolved.
    for(const auto& active:entities_) if(active.noProgressSeconds>6 && !active.waitingForLight && !active.waitingForManeuver) return false;
    Entity e; e.id=nextId_; e.kind=kind; e.center=start; e.goal=goal; e.progressAnchor=start;
    e.length=sample(p.length,.2,30); e.width=sample(p.width,.15,5); e.preferredDistance=sample(p.distance,0,20);
    e.massKg=defaultVehicleMass(kind);
    int destination=map_.destinationAt(goal);
    if(destination>=0 && map_.features()[destination].line) {
        const auto& gate=map_.features()[destination];
        double span=length(gate.b-gate.a),margin=e.width*.5+.02;
        if(span<=margin*2) return false;
        Vec2 tangent=(gate.b-gate.a)*(1/span),normal{-tangent.y,tangent.x};
        double side=(start.x-gate.a.x)*normal.x+(start.y-gate.a.y)*normal.y;
        Vec2 forward=normal*(side>0?-1.0:1.0);
        double along=std::clamp((goal.x-gate.a.x)*tangent.x+(goal.y-gate.a.y)*tangent.y,margin,span-margin);
        e.destinationPoint=gate.a+tangent*along; e.destinationFeature=destination;
        e.goal=e.destinationPoint-forward*(e.length*.5+.01);
        if(!map_.walkable(map_.pixel(e.goal)) || !navigation_.connected(start,e.goal)) return false;
    }
    sampleMotion(e);
    // Try multiple initial headings for a spawn near a curved road boundary.
    double direction=bearing(start,e.goal);
    for(int turn:{0,1,-1,2,-2,3,-3,4}) {
        e.heading=direction+turn*pi/4;
        if(!feasible(e.footprint(),e.footprint(),e,true)) continue;
        e.path=plan(e,variation,false);
        if(!e.path.empty()) {
            // Spawn already aligned with the first leg. Otherwise a car can
            // spawn beside traffic and immediately need an impossible turn.
            if(e.path.size()>1) e.heading=bearing(e.path[0],e.path[1]);
            if(!feasible(e.footprint(),e.footprint(),e,true) || !installPath(e,std::move(e.path))) continue;
            ++nextId_; entities_.push_back(std::move(e)); return true;
        }
    }
    return false;
}
bool Simulation::spawnRandom(double variation) {
    if(!lanes_.empty()) {
        std::vector<int> entries; for(std::size_t i=0;i<lanes_.size();++i) if(lanes_[i].entry) entries.push_back(static_cast<int>(i));
        if(entries.empty()) return false;
        double weights[vehicleKindCount]; double total=0; for(int i=0;i<vehicleKindCount;++i) total+=(weights[i]=profiles_[i].weight);
        if(total<=0) return false;
        auto kind=static_cast<VehicleKind>(std::discrete_distribution<int>(weights,weights+vehicleKindCount)(random_));
        std::shuffle(entries.begin(),entries.end(),random_); for(int lane:entries) if(spawnOnLane(lane,kind)) return true;
        return false;
    }
    if(spawns_.empty() || destinations_.empty()) return false;
    double weights[vehicleKindCount]; double total=0;
    for(int i=0;i<vehicleKindCount;++i) total+=(weights[i]=profiles_[i].weight);
    if(total==0) return false;
    auto kind=static_cast<VehicleKind>(std::discrete_distribution<int>(weights,weights+vehicleKindCount)(random_));
    // One requested vehicle class per attempt; don't silently replace a bus
    // that doesn't fit with a bike. Cap retries to keep failed spawns bounded.
    for(int attempt=0;attempt<8;++attempt) {
        auto s=spawns_[std::uniform_int_distribution<std::size_t>(0,spawns_.size()-1)(random_)];
        auto g=destinations_[std::uniform_int_distribution<std::size_t>(0,destinations_.size()-1)(random_)];
        if(spawn(s,g,variation,kind)) return true;
    }
    return false;
}
void Simulation::tick(double seconds) {
    validate(seconds,1e-8,10,"Time step must be positive and at most 10 seconds");
    int steps=static_cast<int>(std::ceil(seconds/.01));
    for(int i=0;i<steps;++i) advance(seconds/steps);
    ++ticks_;
}
void Simulation::advance(double dt) {
    std::vector<Vec2> before; before.reserve(entities_.size());
    for(const auto& entity:entities_) before.push_back(entity.center);
    for(auto& e:entities_) advanceNetworkStage(e);
    updateLaneChanges();
    updateSublanes();
    bool hasStoppingSignal=false,phaseChanged=signalPhases_.size()!=lights_.size();
    signalPhases_.resize(lights_.size(),LightPhase::Green);
    for(std::size_t i=0;i<lights_.size();++i) {
        auto phase=lights_[i].phase(seconds_); phaseChanged=phaseChanged || phase!=signalPhases_[i];
        signalPhases_[i]=phase; hasStoppingSignal=hasStoppingSignal || phase!=LightPhase::Green;
    }
    if(phaseChanged) for(auto& e:entities_) e.signalRetryAfter=seconds_;
    for(auto& e:entities_) {
        bool wasWaiting=e.waitingForLight || e.signalQueue;
        e.signalQueue=false;
        e.waitingForLight=e.signalLight>=0 && e.signalLight<static_cast<int>(lights_.size()) &&
            lightStops(lights_[e.signalLight],e);
        if(!e.waitingForLight && wasWaiting && hasStoppingSignal) {
            auto probe=e.footprint(); probe.center=probe.center+Vec2{std::cos(e.heading),std::sin(e.heading)}*.5;
            e.waitingForLight=blockedByLight(e.footprint(),probe,e);
        }
    }
    // Finish or expire a bounded clearance prefix. A stalled recovery must
    // not reserve the neighborhood indefinitely after traffic changes.
    for(auto& e:entities_) if(e.cooperativeRecovery) {
        if(e.next>e.reservationEnd) e.cooperativeRecovery=false;
        else if(!e.networkRoute.empty() && !e.laneRoute.empty() && e.currentStopSeconds>2 && seconds_>=e.trafficRecoveryAfter) {
            e.cooperativeRecovery=false;e.sublaneReservation.clear();e.reservationEnd=0;
        }
    }

    planSignalQueues();
    updateTurnNegotiation();
    if(!recoveryPreview_) planStoppedTraffic();
    // A recovery rollout validates the proposed plans. Incidental RTS searches
    // would both waste work and credit routes that are not part of the commit.
    // Resolve a lane-side contact by first translating a short distance at
    // the current heading. Turning immediately can pinch a legally packed
    // neighbor even though there is clear road in front. No pose is moved by
    // planning; the ordinary swept movement checks execute this prefix.
    if(flow_.predictiveJunctions && trafficRecoverySettings_.enabled && !recoveryPreview_) {
        for(auto& e:entities_) {
            if(e.inOpenJunction || e.networkRoute.empty() || e.laneRoute.empty() || e.waitingForJunction ||
               e.turnExitBlocked || e.signalLight!=-1 || e.reversing ||
               e.cooperativeRecovery || e.currentStopSeconds<2 || e.desiredSpeed<.1 ||
               seconds_<e.trafficRecoveryAfter || e.next>=e.path.size()) continue;
            if(stopReason(e)==StopReason::Signal) continue;
            e.trafficRecoveryAfter=seconds_+.5;
            bool nearbyRecovery=false;
            for(const auto& other:entities_) if(other.id!=e.id && other.cooperativeRecovery && !other.laneRoute.empty() && length(other.center-e.center)<12) nearbyRecovery=true;
            if(nearbyRecovery) continue;
            auto started=std::chrono::steady_clock::now();
            auto repair=[&]()->bool {
                if(!fitsLaneRoute(e.footprint(),e)) {e.junctionStatus="Recovery: current body outside lane envelope";return false;}
                Vec2 forward{std::cos(e.heading),std::sin(e.heading)};
                double total=0;Vec2 last=e.center;
                for(std::size_t i=e.next;i<e.path.size();++i){total+=length(e.path[i]-last);last=e.path[i];}
                double wanted=std::min(std::max(3.,e.length*3),std::max(.5,total*.6));
                double walked=0;last=e.center;std::size_t join=e.next;
                for(;join<e.path.size();++join) {walked+=length(e.path[join]-last);last=e.path[join];if(walked>=wanted && (last.x-e.center.x)*forward.x+(last.y-e.center.y)*forward.y>.5) break;}
                if(join>=e.path.size()) {
                    join=e.path.size()-1;
                    if((e.path[join].x-e.center.x)*forward.x+(e.path[join].y-e.center.y)*forward.y<.05) {
                        e.junctionStatus="Recovery: no forward join before route end";return false;
                    }
                }
                // A small reverse prefix creates steering room at a lane edge.
                // It is committed only if its entire swept body/gap is clear.
                for(bool recenter:{false,true}) for(double distance:{.35,.15,.6,-.05,-.15,-.35}) {
                    Entity candidate=e; candidate.cooperativeRecovery=true;
                    auto from=e.footprint(),nudge=from;nudge.center=from.center+forward*distance;
                    if((e.path[join].x-nudge.center.x)*forward.x+(e.path[join].y-nudge.center.y)*forward.y<.1)continue;
                    if(blockedByLight(from,nudge,candidate)) {e.junctionStatus="Recovery: signal blocks short step";continue;}
                    if(!feasible(from,nudge,candidate,true,false)) {e.junctionStatus=feasible(from,nudge,candidate,false,false)?"Recovery: short step blocked by traffic":"Recovery: short step leaves lane or mask";continue;}
                    candidate.path={e.center,nudge.center};candidate.pathHeadings={e.heading,e.heading};candidate.maneuvers.clear();
                    if(recenter) {
                        // A cached lateral detour may aim outside a narrowing
                        // road. Rebuild toward its centre instead of repeatedly
                        // trying to rejoin the same unusable lateral tail.
                        Entity continuation=e;continuation.center=nudge.center;
                        continuation.lateralOffset=0;continuation.admissionPlan={};
                        if(!installNetworkStage(continuation,e.networkStage))continue;
                        candidate.goal=continuation.goal;candidate.lateralOffset=0;
                        candidate.path.insert(candidate.path.end(),continuation.path.begin()+1,continuation.path.end());
                        candidate.pathHeadings.insert(candidate.pathHeadings.end(),continuation.pathHeadings.begin()+1,continuation.pathHeadings.end());
                    } else {
                        auto end=e.path[join];double h=e.pathHeadings[join];Vec2 out{std::cos(h),std::sin(h)};
                        double handle=length(end-nudge.center)*.35;auto b=nudge.center+forward*handle,c=end-out*handle;
                        auto previous=nudge;bool valid=true;
                        int count=std::max(30,static_cast<int>(std::ceil((length(b-nudge.center)+length(c-b)+length(end-c))/.1)));
                        for(int i=1;i<=count && valid;++i) {
                            double t=static_cast<double>(i)/count,u=1-t;
                            auto point=nudge.center*(u*u*u)+b*(3*u*u*t)+c*(3*u*t*t)+end*(t*t*t);
                            auto tangent=(b-nudge.center)*(3*u*u)+(c-b)*(6*u*t)+(end-c)*(3*t*t);
                            if(length(tangent)<1e-8){valid=false;break;}
                            auto pose=from;pose.center=point;pose.heading=std::atan2(tangent.y,tangent.x);
                            if(!feasible(previous,pose,candidate,false,false)){valid=false;break;}
                            candidate.path.push_back(point);candidate.pathHeadings.push_back(pose.heading);previous=pose;
                        }
                        if(!valid) {e.junctionStatus="Recovery: continuation bridge leaves lane or mask";continue;}
                        candidate.path.insert(candidate.path.end(),e.path.begin()+join+1,e.path.end());
                        candidate.pathHeadings.insert(candidate.pathHeadings.end(),e.pathHeadings.begin()+join+1,e.pathHeadings.end());
                    }
                    // Endpoint-only checks can jump across a thin gap in the
                    // union of a curved lane and its junction envelope. Check
                    // the departure at finer than the 0.5 m/s physics step.
                    bool executable=true;double checked=0,checkLimit=std::max(3.,e.length*2);auto probe=e.footprint();
                    for(std::size_t i=1;i<candidate.path.size() && checked<checkLimit;++i) {
                        auto start=probe;double d=length(candidate.path[i]-start.center);
                        double take=std::min(d,checkLimit-checked),fraction=d>1e-9?take/d:1.;
                        double angle=angleDelta(start.heading,candidate.pathHeadings[i])*fraction;
                        int samples=std::max(1,static_cast<int>(std::ceil(std::max(take/.002,std::abs(angle)/.001))));
                        for(int j=1;j<=samples;++j) {
                            double t=static_cast<double>(j)/samples;auto pose=start;
                            pose.center=start.center+(candidate.path[i]-start.center)*(t*fraction);pose.heading=start.heading+angle*t;
                            if(!feasible(probe,pose,candidate,i==1,false)){executable=false;break;}
                            probe=pose;
                        }
                        if(!executable)break;checked+=take;
                    }
                    if(!executable){e.junctionStatus="Recovery: departure crosses lane boundary";continue;}
                    candidate.next=1;candidate.segmentProgress=0;++candidate.routeRevision;
                    candidate.reservedTurn.clear();candidate.reservedApproach.clear();candidate.reservationEnd=1;
                    candidate.sublaneReservation.clear();PathManeuver claim;claim.begin=0;claim.end=1;
                    claim.sweep=geometry::swept(from,nudge,claim.padding);candidate.sublaneReservation={claim};
                    candidate.sublaneReservationRevision=candidate.routeRevision;candidate.sublaneReservationEnd=1;candidate.sublaneReservationProgress=std::abs(distance);
                    candidate.sublaneAfter=seconds_+2;candidate.laneChangeAfter=seconds_+2;candidate.trafficRecoveryAfter=seconds_+2;
                    candidate.waitingForManeuver=false;candidate.following=false;candidate.sublaneStatus=distance<0?"Short reverse clearance recovery":"Short forward clearance recovery";
                    if(!prepareJunctionPlans(candidate))continue;
                    e=std::move(candidate);++recoveries_;++trafficRecoveryRepairs_;lastFleetRouteCount_=1;return true;
                }
                return false;
            };
            bool repaired=repair();++trafficRecoveryPasses_;
            totalTrafficRecoverySeconds_+=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
            if(repaired) break;
        }
    }
    bool replanned=recoveryPreview_ || refreshMovingRoute();
    for(auto& e:entities_) {
        if(e.waitingForJunction) { e.speed=0; continue; }
        if(e.signalLight!=-1 && e.next>=e.path.size()) {
            bool red=e.signalLight>=0 && e.signalLight<static_cast<int>(lights_.size()) && lightStops(lights_[e.signalLight],e);
            if(red) {
                if(e.next<e.path.size()) { e.waitingForLight=false; }
                else {
                    e.speed=0; e.waitingForLight=true; e.waitingForManeuver=false; e.noProgressSeconds=0;
                    e.reservedTurn.clear(); e.reservedApproach.clear();
                    continue;
                }
            }
            if(!red) releaseSignalSlot(e);
        }
        if(flow_.predictiveJunctions && e.inOpenJunction && !e.networkRoute.empty() && advancePredictiveVehicle(e,dt)) continue;
        if(e.laneRoute.empty() && tugSettings_.enabled && advanceTug(e,dt)) continue;
        if(e.usingLocalDetour && (e.center.x-e.detourMerge.x)*e.detourExitDirection.x+
           (e.center.y-e.detourMerge.y)*e.detourExitDirection.y>=0) e.usingLocalDetour=false;
        if(length(e.center-e.progressAnchor)>=.25) { e.progressAnchor=e.center; e.noProgressSeconds=0; }
        else e.noProgressSeconds+=dt;
        if(e.waitingForLight) e.noProgressSeconds=0;
        if(e.laneRoute.empty() && !recoveryPreview_ && !e.turnExitBlocked && !maneuverYieldTo(e) && e.signalLight==-1 && !e.reversing && e.noProgressSeconds>6 && !e.waitingForLight &&
           (!e.waitingForManeuver || e.noProgressSeconds>30) && seconds_>=e.recoveryAfter) beginRecovery(e);
        e.waitingForLight=false; e.waitingForManeuver=false;
        if(e.reversing) {
            Vec2 delta=e.yieldTarget-e.center; double remaining=length(delta);
            auto from=e.footprint(),to=from;
            if(remaining>1e-7) to.center=e.center+delta*(std::min(remaining,dt*std::min(1.0,e.desiredSpeed))/remaining);
            e.waitingForLight=blockedByLight(from,to,e);
            if(!e.waitingForLight && feasible(from,to,e,true)) { e.center=to.center; e.speed=std::min(1.0,e.desiredSpeed); }
            else e.speed=0;
            if(length(e.center-e.yieldTarget)<1e-7) {
                e.reversing=false; e.reservedTurn.clear(); e.reservedApproach.clear(); e.speed=0;
                bool cooperative=e.cooperativeRecovery;
                bool replaced=false;
                if(!cooperative || e.recoveryExit.points.empty()) {
                    auto alternate=plan(e,0,true);
                    if(alternate.empty()) alternate=plan(e,0,false);
                    replaced=!alternate.empty() && installPath(e,std::move(alternate));
                }
                if(!replaced && !e.recoveryExit.points.empty()) {
                    // The exit validated before reversing remains a safe static
                    // fallback. Never resume the pre-reverse trajectory from
                    // its old anchor if a fresh traffic-aware patch fails.
                    e.path=std::move(e.recoveryExit.points); e.pathHeadings=std::move(e.recoveryExit.headings);
                    e.maneuvers=std::move(e.recoveryExit.turns); e.next=1; e.segmentProgress=0;
                    e.usingLocalDetour=false; replaced=true;
                }
                e.recoveryExit={};
                if(cooperative && replaced) {
                    e.cooperativeRecovery=true; e.reservationEnd=e.path.size();
                    for(const auto& turn:e.maneuvers) if(turn.end>=e.next) { e.reservationEnd=turn.end; break; }
                }
                if(replaced) ++replans_;
                e.progressAnchor=e.center; e.noProgressSeconds=0; e.blockedSeconds=0;
            }
            continue;
        }
        if(!e.reservedTurn.empty() && e.next>e.reservationEnd) { e.reservedTurn.clear(); e.reservedApproach.clear(); }
        if(!e.reservedTurn.empty() && !e.reservedApproach.empty()) {
            // Keep future approach space reserved, but release the road already
            // vacated by the owner so a following vehicle can close the gap.
            for(const auto& maneuver:e.maneuvers) if(maneuver.end==e.reservationEnd) {
                if(e.next>maneuver.begin) e.reservedApproach.clear();
                else {
                    auto remaining=corners(e.footprint());
                    for(std::size_t i=e.next;i<=maneuver.begin;++i) {
                        auto box=corners({e.path[i],e.pathHeadings[i],e.length,e.width});
                        remaining.insert(remaining.end(),box.begin(),box.end());
                    }
                    e.reservedApproach=geometry::convexHull(std::move(remaining));
                }
                break;
            }
        }
        if(e.next>=e.path.size()) continue;
        // Replanning is deferred until a committed arc has finished. Otherwise
        // a fresh route could invalidate a maneuver reservation halfway through.
        if(e.laneRoute.empty() && !e.turnExitBlocked && !ownsNegotiatedTurn(e) && !maneuverYieldTo(e) && e.signalLight==-1 && routing_.rtsStyle && e.reservedTurn.empty() && !e.waitingForLight &&
           seconds_>=e.localSteerAfter && !replanned) {
            e.localSteerAfter=seconds_+routing_.localInterval;
            if(localDetour(e)) replanned=true;
        }
        // A long probe can overlap a moving leader even when the actual next
        // step is safe. Do not turn ordinary following into a global detour.
        if(e.laneRoute.empty() && !e.turnExitBlocked && !ownsNegotiatedTurn(e) && !maneuverYieldTo(e) && e.signalLight==-1 && e.reservedTurn.empty() && seconds_>=e.replanAfter && !replanned &&
           (!routing_.rtsStyle || e.blockedSeconds>.25)) {
            followingSpeed(e);
            if((!e.following || e.currentStopSeconds>3) && !e.signalQueue && !e.maneuverQueue) {
                auto from=e.footprint(),ahead=from;
                Vec2 delta=e.path[e.next]-e.center; double remaining=length(delta);
                if(remaining>1e-8) {
                    ahead.center=e.center+delta*(std::min(remaining,std::max(e.length,e.speed*.6))/remaining);
                    ahead.heading=e.pathHeadings[e.next];
                    if(!feasible(from,ahead,e,true) && !blockedByManeuver(from,ahead,e)) {
                        bool repaired=routing_.rtsStyle && localDetour(e);
                        auto alternate=repaired?std::vector<Vec2>{}:plan(e,0,true);
                        if(repaired || (!alternate.empty() && installPath(e,std::move(alternate)))) { e.replanFailures=0; if(!repaired) ++replans_; }
                        else ++e.replanFailures;
                        e.replanAfter=seconds_+std::min(15.0,2.0*(1+e.replanFailures))+(e.id%7)*.1; replanned=true;
                    }
                }
            }
        }
        auto acquire=[&](const PathManeuver& maneuver) {
            bool ready=reserveManeuver(e,maneuver);
            if(!ready) {
                Footprint entry{e.path[maneuver.begin],e.pathHeadings[maneuver.begin],e.length,e.width};
                e.waitingForManeuver=blockedByManeuver(e.footprint(),entry,e);
                for(const auto& other:entities_) if(other.id!=e.id && !other.reservedTurn.empty()) {
                    double gap=std::min(e.preferredDistance,other.preferredDistance)+maneuver.padding+other.turnPadding;
                    if(geometry::violatesClearance(maneuver.sweep,other.reservedTurn,gap) ||
                       geometry::violatesClearance(maneuver.sweep,other.reservedApproach,gap)) e.waitingForManeuver=true;
                }
            }
            return ready;
        };
        bool ready=true;
        double deniedEntryDistance=std::numeric_limits<double>::infinity();
        if(e.turnExitBlocked) {
            deniedEntryDistance=0; Vec2 previous=e.center;
            for(std::size_t i=e.next;i<=e.blockedTurnEntry && i<e.path.size();++i) { deniedEntryDistance+=length(e.path[i]-previous); previous=e.path[i]; }
            ready=deniedEntryDistance>1e-8; e.waitingForManeuver=!ready;
        }
        if(e.laneRoute.empty() && !e.turnExitBlocked && e.signalLight<0 && !e.cooperativeRecovery && !ownsNegotiatedTurn(e) && !maneuverYieldTo(e) && e.reservedTurn.empty()) for(const auto& maneuver:e.maneuvers) if(maneuver.end>=e.next) {
            double approach=0; Vec2 previous=e.center;
            for(std::size_t i=e.next;i<=maneuver.begin;++i) { approach+=length(e.path[i]-previous); previous=e.path[i]; }
            if(approach<std::max({3.0,e.length*1.5,e.speed*.6})) {
                if(!acquire(maneuver)) {
                    deniedEntryDistance=approach;
                    ready=approach>1e-8;
                    // A reservation needed later is not a reason to stop now.
                    // Actual step admission still checks bodies and reservations.
                    if(ready) e.waitingForManeuver=false;
                }
            }
            break;
        }
        bool brakingForBend=false; double bendBraking=laneTurnDeceleration(e);
        double followingTarget=followingSpeed(e);
        // Native signals constrain speed on the committed route, not a new slot path.
        if(flow_.predictiveJunctions && !e.networkRoute.empty()) {
            double signalDistance=0;
            if(closestStoppingLight(e,&signalDistance)>=0)
                followingTarget=std::min(followingTarget,std::sqrt(4*e.acceleration*std::max(0.,signalDistance-e.length*.5-.12)));
        }
        double bendTarget=urbanSpeedLimit(e,followingTarget,brakingForBend,&bendBraking);
        double targetSpeed=negotiatedSpeed(e,bendTarget),travel=0,newSpeed=0;
        targetSpeed=laneMergeSpeed(e,targetSpeed);
        if(e.cooperativeRecovery && !e.laneRoute.empty() && e.next<=e.reservationEnd) targetSpeed=std::min(.5,e.desiredSpeed);
        if(!e.networkRoute.empty()) targetSpeed=std::min(targetSpeed,lanes_[e.networkRoute[e.networkStage]].speedLimit);
        else for(int lane:e.laneRoute) targetSpeed=std::min(targetSpeed,lanes_[lane].speedLimit);
        if(seconds_<e.cooperativeUntil) targetSpeed=std::min(targetSpeed,e.cooperativeSpeedCap);
        if(std::isfinite(deniedEntryDistance))
            targetSpeed=std::min(targetSpeed,std::sqrt(std::max(0.0,4*e.acceleration*deniedEntryDistance)));
        if(e.signalLight>=0 && e.signalLight<static_cast<int>(lights_.size()) && lightStops(lights_[e.signalLight],e))
            targetSpeed=std::min(targetSpeed,std::sqrt(std::max(0.0,4*e.acceleration*e.signalRemaining)));
        if(e.speed<=targetSpeed) {
            double accelerating=std::min(dt,std::max(0.0,(targetSpeed-e.speed)/e.acceleration));
            travel=e.speed*accelerating+.5*e.acceleration*accelerating*accelerating+targetSpeed*(dt-accelerating);
            newSpeed=std::min(targetSpeed,e.speed+e.acceleration*dt);
        } else {
            double braking=brakingForBend && targetSpeed>=bendTarget-1e-8?bendBraking:e.acceleration*2;
            double slowing=std::min(dt,(e.speed-targetSpeed)/braking);
            travel=e.speed*slowing-.5*braking*slowing*slowing+targetSpeed*(dt-slowing);
            newSpeed=std::max(targetSpeed,e.speed-braking*dt);
        }
        if(std::isfinite(deniedEntryDistance) && travel>=deniedEntryDistance) {
            travel=deniedEntryDistance; newSpeed=0;
        }
        bool accepted=ready;
        while(accepted && travel>1e-12 && e.next<e.path.size()) {
            if(e.laneRoute.empty() && !e.turnExitBlocked && e.signalLight<0 && !e.cooperativeRecovery && !ownsNegotiatedTurn(e) && !maneuverYieldTo(e) && e.reservedTurn.empty()) for(const auto& maneuver:e.maneuvers)
                if(e.next>maneuver.begin && e.next<=maneuver.end) { accepted=acquire(maneuver); break; }
            if(!accepted) break;
            double segment=length(e.path[e.next]-e.path[e.next-1]);
            if(segment<1e-10) { ++e.next; e.segmentProgress=0; continue; }
            double move=std::min(travel,segment-e.segmentProgress);
            double progress=std::clamp((e.segmentProgress+move)/segment,0.0,1.0);
            auto from=e.footprint(),to=from;
            to.center=e.path[e.next-1]+(e.path[e.next]-e.path[e.next-1])*progress;
            to.heading=e.pathHeadings[e.next-1]+angleDelta(e.pathHeadings[e.next-1],e.pathHeadings[e.next])*progress;
            e.waitingForLight=blockedByLight(from,to,e);
            accepted=length(to.center-from.center)<=move+1e-7 && !e.waitingForLight && feasible(from,to,e,true);
            if(!accepted) {
                e.waitingForManeuver=blockedByManeuver(from,to,e);
                if(!e.waitingForLight) {
                    double pad; auto sweep=geometry::swept(from,to,pad);
                    for(const auto& other:entities_) if(other.id!=e.id && other.waitingForLight &&
                        geometry::distance(sweep,corners(other.footprint()))<std::min(e.preferredDistance,other.preferredDistance)+pad+1e-6) {
                        e.waitingForLight=true; break;
                    }
                }
                break;
            }
            e.center=to.center; e.heading=to.heading; e.segmentProgress+=move; travel-=move;
            if(e.signalLight!=-1) e.signalRemaining=std::max(0.0,e.signalRemaining-move);
            if(e.segmentProgress>=segment-1e-10) {
                ++e.next; e.segmentProgress=0;
                if(!e.reservedTurn.empty() && e.next>e.reservationEnd) { e.reservedTurn.clear(); e.reservedApproach.clear(); }
            }
        }
        if(accepted) { e.speed=newSpeed; e.blockedSeconds=0; }
        else {
            e.speed=0; e.blockedSeconds+=dt;
            if(e.laneRoute.empty() && !e.turnExitBlocked && !ownsNegotiatedTurn(e) && !maneuverYieldTo(e) && e.signalLight==-1 && !e.waitingForLight && !e.waitingForManeuver && e.reservedTurn.empty() && e.blockedSeconds>.6 && seconds_>=e.replanAfter && !replanned &&
               (!e.following || e.currentStopSeconds>3) && !e.signalQueue && !e.maneuverQueue) {
                bool repaired=routing_.rtsStyle && localDetour(e);
                auto alternate=repaired?std::vector<Vec2>{}:plan(e,0,true);
                if(repaired || (!alternate.empty() && installPath(e,std::move(alternate)))) { e.replanFailures=0; if(!repaired) ++replans_; }
                else ++e.replanFailures;
                e.replanAfter=seconds_+std::min(15.0,2.0*(1+e.replanFailures))+(e.id%7)*.1; replanned=true;
            }
        }
        e.waitingForLight=e.waitingForLight || (e.following && e.signalQueue);
        e.waitingForManeuver=e.waitingForManeuver || (e.following && e.maneuverQueue);
        if(e.waitingForLight) { e.noProgressSeconds=0; e.reservedTurn.clear(); e.reservedApproach.clear(); }
    }
    for(auto& e:entities_) advanceNetworkStage(e);
    for(std::size_t i=0;i<entities_.size();++i) {
        auto& e=entities_[i]; e.ageSeconds+=dt;
        if(!e.reversing && length(e.center-before[i])>stoppedSpeedThreshold*dt)
            e.movingSinceRefresh+=dt;
        // Actual displacement also handles blocked/reversing vehicles and every
        // early-continue above, rather than trusting the requested speed.
        if(length(e.center-before[i])<=stoppedSpeedThreshold*dt) {
            e.stoppedSeconds+=dt; e.currentStopSeconds+=dt;
            e.longestStopSeconds=std::max(e.longestStopSeconds,e.currentStopSeconds);
        } else e.currentStopSeconds=0;
        // The deletion policy uses a tighter threshold than telemetry, so a
        // deliberately slow vehicle is not classified as a stuck blocker.
        if(length(e.center-before[i])<=.01*dt) e.stuckSeconds+=dt;
        else e.stuckSeconds=0;
        if(e.inOpenJunction && length(e.center-before[i])<=stoppedSpeedThreshold*dt)
            e.junctionStationarySeconds+=dt;
        else e.junctionStationarySeconds=0;
    }
    entities_.erase(std::remove_if(entities_.begin(),entities_.end(),[this](const Entity& e) {
        if(!e.networkRoute.empty() && e.networkStage+1<e.networkRoute.size()) return false;
        if(e.signalLight!=-1) return false;
        bool atExit=e.destinationFeature>=0 && map_.touchesDestination(e.footprint(),e.destinationFeature);
        if(!atExit && e.next<e.path.size()) return false;
        recordFinishedStopTime(e);
        ++completed_; return true;
    }),entities_.end());
    seconds_+=dt;
    if(!recoveryPreview_) { clearStuckVehicles(); updateTrafficMonitoring(dt); }
}
void Simulation::recordFinishedStopTime(const Entity& e) {
    completedStopMin_=(completed_+removedStuckVehicles_)?std::min(completedStopMin_,e.stoppedSeconds):e.stoppedSeconds;
    completedStopMax_=std::max(completedStopMax_,e.stoppedSeconds);
    completedStopTotal_+=e.stoppedSeconds;
    completedThatStopped_+=e.stoppedSeconds>0;
}
unsigned Simulation::maximumCollisionWorkers() { return detail::availableWorkers(); }
void Simulation::setCollisionWorkers(unsigned workers) {
    if(workers<1 || workers>maximumCollisionWorkers()) throw std::invalid_argument("Invalid collision worker count");
    collisionWorkers_=workers;
}
StopStatistics Simulation::stopStatistics(bool includeCompleted) const {
    StopStatistics result;
    if(includeCompleted && (completed_+removedStuckVehicles_)) {
        result.vehicles=completed_+removedStuckVehicles_; result.vehiclesThatStopped=completedThatStopped_;
        result.totalSeconds=completedStopTotal_; result.minSeconds=completedStopMin_; result.maxSeconds=completedStopMax_;
    }
    for(const auto& e:entities_) {
        result.minSeconds=result.vehicles?std::min(result.minSeconds,e.stoppedSeconds):e.stoppedSeconds;
        result.maxSeconds=std::max(result.maxSeconds,e.stoppedSeconds);
        result.totalSeconds+=e.stoppedSeconds; ++result.vehicles;
        result.vehiclesThatStopped+=e.stoppedSeconds>0;
    }
    if(result.vehicles) result.meanSeconds=result.totalSeconds/result.vehicles;
    return result;
}
}
