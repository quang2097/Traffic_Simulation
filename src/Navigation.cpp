#include "traffic/Simulation.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace traffic {
Navigation::Navigation(const Map& map) : width_(map.width),height_(map.height),scale_(map.metersPerPixel),labels_(map.tiles.size(),-1) {
    const int sector=std::clamp(static_cast<int>(std::ceil(4/scale_)),4,64);
    std::vector<int> pending;
    for(int y=0;y<height_;++y) for(int x=0;x<width_;++x) {
        int start=y*width_+x;
        if(labels_[start]>=0 || !map.walkable({x,y})) continue;
        int label=static_cast<int>(regions_.size()); regions_.push_back({});
        auto& region=regions_.back();
        pending.clear(); pending.push_back(start); labels_[start]=label;
        for(std::size_t i=0;i<pending.size();++i) {
            int p=pending[i],px=p%width_,py=p/width_;
            region.center=region.center+map.center({px,py});
            for(Cell q:map.neighbors({px,py})) {
                if(q.x/sector!=x/sector || q.y/sector!=y/sector) continue;
                int j=map.index(q);
                if(labels_[j]<0) { labels_[j]=label; pending.push_back(j); }
            }
        }
        region.center=region.center*(1.0/pending.size()); region.area=pending.size()*scale_*scale_;
    }
    for(int y=0;y<height_;++y) for(int x=0;x<width_;++x) {
        int a=labels_[y*width_+x]; if(a<0) continue;
        for(Cell q:{Cell{x+1,y},Cell{x,y+1}}) {
            if(!map.contains(q)) continue;
            int b=labels_[map.index(q)];
            if(b>=0 && a!=b) { regions_[a].neighbors.push_back(b); regions_[b].neighbors.push_back(a); }
        }
    }
    for(auto& r:regions_) {
        std::sort(r.neighbors.begin(),r.neighbors.end());
        r.neighbors.erase(std::unique(r.neighbors.begin(),r.neighbors.end()),r.neighbors.end());
    }
    int component=0;
    for(std::size_t i=0;i<regions_.size();++i) if(regions_[i].component<0) {
        pending={static_cast<int>(i)}; regions_[i].component=component;
        for(std::size_t j=0;j<pending.size();++j) for(int k:regions_[pending[j]].neighbors)
            if(regions_[k].component<0) { regions_[k].component=component; pending.push_back(k); }
        ++component;
    }
}
int Navigation::regionAt(Vec2 p) const {
    if(!std::isfinite(p.x) || !std::isfinite(p.y) || p.x<0 || p.y<0 || p.x>=width_*scale_ || p.y>=height_*scale_) return -1;
    return labels_[static_cast<int>(p.y/scale_)*width_+static_cast<int>(p.x/scale_)];
}
bool Navigation::connected(Vec2 a,Vec2 b) const {
    int x=regionAt(a),y=regionAt(b);
    return x>=0 && y>=0 && regions_[x].component==regions_[y].component;
}
const std::vector<double>& Navigation::goalField(int goal) {
    if(goal<0 || goal>=static_cast<int>(regions_.size())) throw std::invalid_argument("Invalid navigation region");
    for(std::size_t i=0;i<fields_.size();++i) if(fields_[i].first==goal) {
        ++cacheHits_;
        if(i+1<fields_.size()) { auto entry=std::move(fields_[i]); fields_.erase(fields_.begin()+i); fields_.push_back(std::move(entry)); }
        return fields_.back().second;
    }
    ++fieldBuilds_;
    std::vector<double> costs(regions_.size(),std::numeric_limits<double>::infinity());
    using Entry=std::pair<double,int>;
    std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> queue;
    costs[goal]=0; queue.push({0,goal});
    while(!queue.empty()) {
        auto [cost,i]=queue.top(); queue.pop();
        if(cost!=costs[i]) continue;
        for(int j:regions_[i].neighbors) {
            double candidate=cost+std::max(scale_,length(regions_[i].center-regions_[j].center));
            if(candidate<costs[j]) { costs[j]=candidate; queue.push({candidate,j}); }
        }
    }
    if(fields_.size()>=16) fields_.erase(fields_.begin());
    fields_.push_back({goal,std::move(costs)}); return fields_.back().second;
}
void Simulation::setRoutingSettings(RoutingSettings settings) {
    if(!std::isfinite(settings.lookAhead) || settings.lookAhead<2 || settings.lookAhead>60 ||
       !std::isfinite(settings.localInterval) || settings.localInterval<.1 || settings.localInterval>5 ||
       !std::isfinite(settings.congestionWeight) || settings.congestionWeight<0 || settings.congestionWeight>10 ||
       !std::isfinite(settings.movingRefreshInterval) || settings.movingRefreshInterval<.25 || settings.movingRefreshInterval>120)
        throw std::invalid_argument("Invalid routing settings");
    if(settings.rtsStyle!=routing_.rtsStyle || settings.refreshMovingRoutes!=routing_.refreshMovingRoutes ||
       settings.movingRefreshInterval!=routing_.movingRefreshInterval)
        for(auto& e:entities_) e.movingSinceRefresh=0;
    routing_=settings;
}
}
