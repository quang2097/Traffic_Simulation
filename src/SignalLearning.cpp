#include "traffic/SignalLearning.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace traffic {
namespace {
double dot(Vec2 a,Vec2 b){return a.x*b.x+a.y*b.y;}
double area(const std::vector<Vec2>& points){double sum=0;for(std::size_t i=0;i<points.size();++i){auto a=points[i],b=points[(i+1)%points.size()];sum+=a.x*b.y-a.y*b.x;}return std::abs(sum)*.5;}
std::vector<Vec2> clip(std::vector<Vec2> points,int axis,double bound,bool upper){
    std::vector<Vec2> out;if(points.empty())return out;
    auto value=[&](Vec2 p){return axis?p.y:p.x;};
    auto inside=[&](Vec2 p){return upper?value(p)<=bound:value(p)>=bound;};
    auto a=points.back();for(auto b:points){bool ia=inside(a),ib=inside(b);if(ia!=ib){double t=(bound-value(a))/(value(b)-value(a));out.push_back(a+(b-a)*t);}if(ib)out.push_back(b);a=b;}return out;
}
double offsetFor(double cycle,double phase,double now){double offset=std::fmod(phase-now,cycle);return offset<0?offset+cycle:offset;}
}
std::vector<SignalRegion> defaultSignalRegions(const std::vector<TrafficLight>& lights){
    std::vector<SignalRegion> result;for(const auto& light:lights){auto d=light.controlledDirection();result.push_back({light.id,std::abs(d.x)>=std::abs(d.y)?0:1,30,0});}
    std::sort(result.begin(),result.end(),[](auto a,auto b){return a.lightId<b.lightId;});return result;
}
SignalObservation observeSignals(const std::vector<TrafficLight>& lights,const std::vector<Entity>& vehicles,const std::vector<SignalRegion>& regions){
    SignalObservation result;std::array<double,2> total{},occupied{};
    for(const auto& region:regions){
        auto found=std::find_if(lights.begin(),lights.end(),[&](const auto& l){return l.id==region.lightId;});
        if(found==lights.end() || !found->enabled)continue;
        if(region.group<0 || region.group>=2 || !std::isfinite(region.depth) || region.depth<=0 || !std::isfinite(region.width) || region.width<0)throw std::invalid_argument("Invalid signal observation region");
        const auto& light=*found;auto forward=light.controlledDirection();Vec2 side{-forward.y,forward.x},middle=(light.a+light.b)*.5;
        double width=region.width>0?region.width:length(light.b-light.a);if(width<=0)throw std::invalid_argument("A signal region needs positive width");
        SignalDensity density;density.lightId=light.id;density.group=region.group;density.area=width*region.depth;
        density.polygon={middle+side*(width*.5),middle-side*(width*.5),middle-side*(width*.5)-forward*region.depth,middle+side*(width*.5)-forward*region.depth};
        for(const auto& vehicle:vehicles){
            Vec2 travel{std::cos(vehicle.heading),std::sin(vehicle.heading)};if(vehicle.reversing)travel=travel*-1;
            if(!light.controlsDirection(travel))continue;
            if(length(vehicle.center-middle)>region.depth+width+vehicle.length)continue;
            auto polygon=corners(vehicle.footprint());for(auto& point:polygon){auto delta=point-middle;point={dot(delta,side),dot(delta,forward)};}
            polygon=clip(std::move(polygon),0,-width*.5,false);polygon=clip(std::move(polygon),0,width*.5,true);
            polygon=clip(std::move(polygon),1,-region.depth,false);polygon=clip(std::move(polygon),1,0,true);
            double covered=area(polygon);if(covered>1e-9){density.occupiedArea+=covered;++density.vehicles;}
        }
        density.occupiedArea=std::min(density.area,density.occupiedArea);density.density=density.occupiedArea/density.area;
        total[region.group]+=density.area;occupied[region.group]+=density.occupiedArea;result.regions.push_back(std::move(density));
    }
    for(int group=0;group<2;++group)if(total[group]>0){result.mask[group]=1;result.density[group]=std::clamp(occupied[group]/total[group],0.,1.);}
    return result;
}
void SignalTimingController::begin(std::vector<TrafficLight>& lights,const std::vector<SignalRegion>& regions,double now,double yellow,double clearance){
    if(active_)throw std::invalid_argument("Policy timing is already active");
    if(!std::isfinite(now) || now<0)throw std::invalid_argument("Invalid simulation clock");
    if(lights.empty() || lights.size()>4)throw std::invalid_argument("Policy control requires 1-4 lights in at most two groups");
    if(!std::isfinite(yellow) || yellow<1 || yellow>10 || !std::isfinite(clearance) || clearance<.5 || clearance>10)throw std::invalid_argument("Invalid yellow or clearance duration");
    std::array<bool,2> groups{};for(const auto& light:lights){auto found=std::find_if(regions.begin(),regions.end(),[&](const auto& r){return r.lightId==light.id;});if(found==regions.end() || found->group<0 || found->group>1 || !light.enabled)throw std::invalid_argument("Assign and enable every light before policy control");groups[found->group]=true;}
    if(!groups[0] || !groups[1])throw std::invalid_argument("Assign at least one light to each of the two groups");
    original_=lights;for(auto& light:lights){light.suspendNormalTr=true;light.normalTr={};}regions_=regions;yellow_=yellow;clearance_=clearance;boundary_=now+yellow+clearance;green_=pending_={15,15};active_=true;
    // Finish with yellow before the first synchronized cycle starts.
    for(auto& light:lights){auto previous=std::find_if(original_.begin(),original_.end(),[&](const auto& old){return old.id==light.id;});bool wasGreen=previous->phase(now)!=LightPhase::Red;light.greenSeconds=.1;light.yellowSeconds=yellow;light.redSeconds=yellow+clearance+1;double cycle=light.greenSeconds+light.yellowSeconds+light.redSeconds;light.offsetSeconds=offsetFor(cycle,wasGreen?.1:.1+yellow,now);}
}
void SignalTimingController::queue(std::array<double,2> green){for(double duration:green)if(!std::isfinite(duration) || duration<5 || duration>120)throw std::invalid_argument("Policy green durations must be finite and between 5 and 120 seconds");pending_=green;}
bool SignalTimingController::update(std::vector<TrafficLight>& lights,double now){
    if(!active_)return false;
    if(lights.size()!=original_.size())throw std::invalid_argument("Signal layout changed; stop policy control and reload");
    for(const auto& light:lights)if(std::none_of(original_.begin(),original_.end(),[&](const auto& l){return l.id==light.id && l.enabled==light.enabled && length(l.a-light.a)+length(l.b-light.b)<1e-8 && length(l.controlledDirection()-light.controlledDirection())<1e-8;}))throw std::invalid_argument("Signal geometry changed during policy control");
    if(now+1e-9<boundary_)return false;
    double start=boundary_;green_=pending_;double cycle=green_[0]+green_[1]+2*(yellow_+clearance_);
    // A paused/rendered frame cannot skip any transition; physics ticks drive this.
    if(now>=start+cycle)start+=std::floor((now-start)/cycle)*cycle;
    for(auto& light:lights){int group=std::find_if(regions_.begin(),regions_.end(),[&](const auto& r){return r.lightId==light.id;})->group;
        light.greenSeconds=green_[group];light.yellowSeconds=yellow_;light.redSeconds=cycle-green_[group]-yellow_;
        double phase=group?cycle-(green_[0]+yellow_+clearance_):0;light.offsetSeconds=offsetFor(cycle,phase,start);
    }
    boundary_=start+cycle;return true;
}
void SignalTimingController::stop(std::vector<TrafficLight>& lights){if(active_)for(auto& light:lights){auto old=std::find_if(original_.begin(),original_.end(),[&](const auto& l){return l.id==light.id;});if(old!=original_.end()){light.greenSeconds=old->greenSeconds;light.yellowSeconds=old->yellowSeconds;light.redSeconds=old->redSeconds;light.offsetSeconds=old->offsetSeconds;light.suspendNormalTr=old->suspendNormalTr;light.normalTr=old->normalTr;}}active_=false;original_.clear();}
}
