#include "traffic/Simulation.hpp"
#include <algorithm>
#include <cmath>

namespace traffic {
namespace {
// Clip the finite detector segment against the actual oriented vehicle body.
// A long line crossing a body counts even if both endpoints lie outside it.
bool touches(const QueueDetector& line,const Footprint& body) {
    double c=std::cos(body.heading),s=std::sin(body.heading);
    auto local=[&](Vec2 p) { p=p-body.center; return Vec2{p.x*c+p.y*s,-p.x*s+p.y*c}; };
    Vec2 a=local(line.a),d=local(line.b)-a;
    double low=0,high=1;
    auto clip=[&](double origin,double direction,double half) {
        if(std::abs(direction)<1e-12) return std::abs(origin)<=half+1e-9;
        double t0=(-half-origin)/direction,t1=(half-origin)/direction;
        if(t0>t1) std::swap(t0,t1);
        low=std::max(low,t0); high=std::min(high,t1);
        return low<=high+1e-9;
    };
    return clip(a.x,d.x,body.length*.5) && clip(a.y,d.y,body.width*.5);
}
}
std::size_t Simulation::openJunctionVehicles() const {
    return std::count_if(entities_.begin(),entities_.end(),[](const Entity& e) { return e.inOpenJunction; });
}
void Simulation::reconcileQueueDetectors() {
    std::vector<DetectorHistory> next;
    for(const auto& light:lights_) for(const auto& line:light.queueDetectors) {
        auto old=std::find_if(detectorHistories_.begin(),detectorHistories_.end(),[&](const DetectorHistory& h) {
            return h.lightId==light.id && h.id==line.id && length(h.a-line.a)+length(h.b-line.b)<1e-9 &&
                h.window==light.queueWindowSeconds && h.threshold==light.queueThresholdPercent;
        });
        if(old!=detectorHistories_.end()) next.push_back(std::move(*old));
        else {
            DetectorHistory h; h.lightId=light.id; h.id=line.id; h.a=line.a; h.b=line.b;
            h.window=light.queueWindowSeconds; h.threshold=light.queueThresholdPercent;
            next.push_back(std::move(h));
        }
        auto& h=next.back();
        h.occupied=std::any_of(entities_.begin(),entities_.end(),[&](const Entity& e) { return touches(line,e.footprint()); });
    }
    detectorHistories_=std::move(next);
}
void Simulation::updateTrafficMonitoring(double dt) {
    for(auto& h:detectorHistories_) {
        QueueDetector line{h.a,h.b,h.id};
        h.occupied=std::any_of(entities_.begin(),entities_.end(),[&](const Entity& e) { return touches(line,e.footprint()); });
        if(!h.samples.empty() && h.samples.back().occupied==h.occupied) h.samples.back().seconds+=dt;
        else h.samples.push_back({dt,h.occupied});
        h.observed+=dt;
        if(h.occupied) h.occupiedTime+=dt;
        // Trim partial samples too: occupancy is weighted by simulated time,
        // independently of physics step size or rendering frame rate.
        double excess=std::max(0.,h.observed-h.window);
        while(excess>0 && !h.samples.empty()) {
            auto& sample=h.samples.front(); double trim=std::min(excess,sample.seconds);
            if(sample.occupied) h.occupiedTime-=trim;
            h.observed-=trim; excess-=trim; sample.seconds-=trim;
            if(sample.seconds<=1e-12) h.samples.pop_front();
        }
        h.occupiedTime=std::clamp(h.occupiedTime,0.,h.observed);
    }
}
TrafficLightQueueStatus Simulation::trafficLightQueueStatus(std::uint64_t lightId) const {
    TrafficLightQueueStatus result; result.lightId=lightId;
    for(const auto& h:detectorHistories_) if(h.lightId==lightId) {
        double percent=h.observed>0?100*h.occupiedTime/h.observed:0;
        result.lines.push_back({h.id,h.occupied,percent,h.observed});
        if(h.observed+1e-8>=h.window && percent+1e-8>=h.threshold)
            result.qualityLine=static_cast<int>(result.lines.size());
    }
    return result;
}
}
