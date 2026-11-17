#include "traffic/Simulation.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace traffic {
void Simulation::setStuckVehicleSettings(StuckVehicleSettings settings) {
    if(!std::isfinite(settings.timeoutSeconds) || settings.timeoutSeconds<1 || settings.timeoutSeconds>3600)
        throw std::invalid_argument("Stuck timeout must be 1–3600 seconds");
    stuckVehicleSettings_=settings;
    // A newly enabled policy gets a full observation period.
    for(auto& e:entities_) e.stuckSeconds=0;
    nextStuckCheck_=seconds_;
}
void Simulation::clearStuckVehicles() {
    if(seconds_<nextStuckCheck_) return;
    nextStuckCheck_=seconds_+.5;
    if(!stuckVehicleSettings_.enabled || entities_.empty()) return;
    const auto count=entities_.size();
    std::vector<int> ahead(count,-1),incoming(count,0),votes(count,0);
    std::vector<bool> signalWait(count,false);
    auto dot=[](Vec2 a,Vec2 b) { return a.x*b.x+a.y*b.y; };
    auto extent=[&](const Entity& e,Vec2 axis) {
        double result=0;
        for(Vec2 p:corners(e.footprint())) result=std::max(result,std::abs(dot(p-e.center,axis)));
        return result;
    };
    for(std::size_t i=0;i<count;++i) {
        const auto& e=entities_[i];
        signalWait[i]=e.turnExitBlocked || (maneuverYieldTo(e) && !e.clearingTurn);
        Vec2 forward{std::cos(e.heading),std::sin(e.heading)},side{-forward.y,forward.x};
        if(e.signalLight>=0 && e.signalLight<static_cast<int>(lights_.size()))
            signalWait[i]=signalWait[i] || lightStops(lights_[e.signalLight],e);
        if(!signalWait[i] && e.waitingForLight) {
            auto probe=e.footprint(); probe.center=probe.center+forward*std::max(2.0,e.length);
            signalWait[i]=blockedByLight(e.footprint(),probe,e);
        }
        double nearest=std::numeric_limits<double>::infinity();
        for(std::size_t j=0;j<count;++j) if(i!=j) {
            const auto& other=entities_[j]; Vec2 delta=other.center-e.center;
            double along=dot(delta,forward),gap=std::min(e.preferredDistance,other.preferredDistance);
            if(along<=0 || along>std::max({routing_.lookAhead,e.length*3,5.0}) || along>=nearest) continue;
            if(std::abs(dot(delta,side))>e.width*.5+extent(other,side)+gap+.15) continue;
            if(along-e.length*.5-extent(other,forward)>std::max(2.0,gap+.75)) continue;
            nearest=along; ahead[i]=static_cast<int>(j);
        }
        if(ahead[i]>=0) ++incoming[ahead[i]];
    }
    // Follow each stopped queue to its front. A red-light root protects its
    // entire queue, including vehicles too far back to see the stop line.
    for(std::size_t start=0;start<count;++start) {
        if(entities_[start].speed>stoppedSpeedThreshold) continue;
        std::vector<int> chain; int current=static_cast<int>(start),root=-1;
        bool protectedBySignal=false;
        while(current>=0) {
            if(signalWait[current]) { protectedBySignal=true; break; }
            if(entities_[current].speed>stoppedSpeedThreshold) break;
            auto repeated=std::find(chain.begin(),chain.end(),current);
            if(repeated!=chain.end()) {
                // Opposing/circular deadlocks have no unique front: remove
                // the blocker with most direct followers, then oldest ID.
                root=*std::max_element(repeated,chain.end(),[&](int a,int b) {
                    return incoming[a]==incoming[b]?entities_[a].id>entities_[b].id:incoming[a]<incoming[b];
                });
                break;
            }
            chain.push_back(current);
            if(ahead[current]<0) { root=current; break; }
            current=ahead[current];
        }
        if(protectedBySignal) {
            entities_[current].stuckSeconds=0;
            for(int index:chain) entities_[index].stuckSeconds=0;
        } else if(root>=0 && entities_[start].stuckSeconds>=stuckVehicleSettings_.timeoutSeconds)
            ++votes[root];
    }
    if(seconds_<removeStuckAfter_) return;
    int selected=-1;
    for(std::size_t i=0;i<count;++i) if(!entities_[i].inOpenJunction && votes[i]>=2 && entities_[i].stuckSeconds>=stuckVehicleSettings_.timeoutSeconds &&
        (selected<0 || votes[i]>votes[selected] || (votes[i]==votes[selected] && entities_[i].id<entities_[selected].id))) selected=static_cast<int>(i);
    if(selected<0) return;
    const auto& victim=entities_[selected];
    recordFinishedStopTime(victim); lastRemovedStuckVehicle_=victim.id;
    ++removedStuckVehicles_;
    // Erasing the entity also releases every reservation it owns.
    entities_.erase(entities_.begin()+selected);
    removeStuckAfter_=seconds_+2;
}
}
