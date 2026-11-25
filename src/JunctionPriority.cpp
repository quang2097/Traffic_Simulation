#include "traffic/Simulation.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <cmath>
namespace traffic {
void Simulation::preparePriorityYields(){
    if(seconds_<priorityCheckAfter_)return;priorityCheckAfter_=seconds_+.1;
    struct Movement {std::size_t entity;int junction,rank;std::vector<Vec2> path;bool semantic=false;};
    std::vector<Movement> movements;
    for(std::size_t k=0;k<entities_.size();++k){auto& e=entities_[k];
        if(e.networkRoute.empty() || e.reversing || e.waitingForLight)continue;
        std::size_t stage=e.networkStage;
        if(!e.inOpenJunction){if(stage+1>=e.networkRoute.size())continue;++stage;}
        const auto& lane=lanes_[e.networkRoute[stage]];if(lane.openJunction<0)continue;
        Movement movement{k,lane.openJunction,lane.movementPriority,{e.center},lane.semanticTransfer};
        if(e.inOpenJunction){for(std::size_t i=e.next;i<e.path.size();++i)if(length(e.path[i]-movement.path.back())>1)movement.path.push_back(e.path[i]);if(!e.path.empty())movement.path.push_back(e.path.back());}
        else {
            double distance=0;Vec2 previous=e.center;for(std::size_t i=e.next;i<e.path.size();++i){distance+=length(e.path[i]-previous);previous=e.path[i];}
            double stop=0;if(closestStoppingLight(e,&stop)>=0 && stop<=distance+e.length)continue;
            if(distance>std::max(12.,e.speed*2+e.length) || stage>=e.junctionPlans.size() || e.junctionPlans[stage].points.empty())continue;
            for(auto point:e.junctionPlans[stage].points)if(length(point-movement.path.back())>1)movement.path.push_back(point);
        }
        if(movement.path.size()>1)movements.push_back(std::move(movement));
    }
    auto find=[&](std::uint64_t id)->Movement*{for(auto& m:movements)if(entities_[m.entity].id==id && entities_[m.entity].inOpenJunction)return &m;return nullptr;};
    if(!flow_.swapStarvedPriorities || !find(priorityGrant_))priorityGrant_=0;
    if(flow_.swapStarvedPriorities && !priorityGrant_){Entity* oldest=nullptr;for(auto& m:movements){auto& e=entities_[m.entity];if(e.inOpenJunction && e.priorityYielding && e.priorityYieldSince>=0 && seconds_-e.priorityYieldSince>=flow_.priorityWaitSeconds && (!oldest || e.priorityYieldSince<oldest->priorityYieldSince || (e.priorityYieldSince==oldest->priorityYieldSince && e.id<oldest->id)))oldest=&e;}if(oldest)priorityGrant_=oldest->id;}
    // Future body corridors, rather than instantaneous headings, determine conflicts.
    auto overlap=[&](const Movement& a,const Movement& b){double gap=(entities_[a.entity].width+entities_[b.entity].width)*.5+.15;
        for(std::size_t i=1;i<a.path.size();++i)for(std::size_t j=1;j<b.path.size();++j){auto p=a.path[i-1],q=a.path[i],r=b.path[j-1],s=b.path[j];if(std::max(p.x,q.x)+gap<std::min(r.x,s.x) || std::max(r.x,s.x)+gap<std::min(p.x,q.x) || std::max(p.y,q.y)+gap<std::min(r.y,s.y) || std::max(r.y,s.y)+gap<std::min(p.y,q.y))continue;if(geometry::distance({p,q},{r,s})<gap)return true;}return false;};
    for(auto& e:entities_)e.priorityYielding=false;
    for(auto& a:movements){auto& e=entities_[a.entity];int rank=e.id==priorityGrant_?0:a.rank;
        for(auto& b:movements){auto& other=entities_[b.entity];int otherRank=other.id==priorityGrant_?0:b.rank;
            if(a.entity==b.entity || (a.junction!=b.junction && !a.semantic && !b.semantic) || rank<=otherRank)continue;
            // Vehicles already physically obstructing the granted movement must
            // clear first; forcing them to stop would make the grant unusable.
            if(other.id==priorityGrant_){auto d=other.path.size()>other.next?other.path[other.next]-other.center:Vec2{};double n=length(d);if(n>1e-8){d=d*(1/n);auto probe=other.footprint();probe.center=probe.center+d*(other.length+1);if(geometry::closerThan(corners(e.footprint()),corners(probe),.2))continue;}}
            if(overlap(a,b)){e.priorityYielding=true;break;}
        }
        if(e.priorityYielding){if(e.priorityYieldSince<0)e.priorityYieldSince=seconds_;e.junctionStatus="Yielding to higher-priority movement";}else e.priorityYieldSince=-1;
    }
    for(auto& e:entities_)if(!e.priorityYielding)e.priorityYieldSince=-1;
}
}
