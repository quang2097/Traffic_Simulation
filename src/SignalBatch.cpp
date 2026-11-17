#include "traffic/Simulation.hpp"
#include <algorithm>
#include <chrono>

namespace traffic {
void Simulation::planSignalQueues(bool force) {
    auto started=std::chrono::steady_clock::now();
    for(auto& e:entities_) {
        if(force) e.signalRetryAfter=seconds_;
        if(e.signalLight!=-1) {
            bool release=e.signalLight<0 || e.signalLight>=static_cast<int>(lights_.size()) || !lightStops(lights_[e.signalLight],e);
            if(!release && seconds_>=e.signalRetryAfter) {
                int nearest=closestStoppingLight(e); e.stopTargetLight=nearest;
                release=nearest>=0 && nearest!=e.signalLight;
                if(e.next<e.path.size()) e.signalRetryAfter=seconds_+.25;
            }
            if(release && releaseSignalSlot(e)) e.signalRetryAfter=seconds_;
        }
    }
    std::vector<std::pair<double,std::size_t>> queue;
    for(std::size_t i=0;i<entities_.size();++i) {
        auto& e=entities_[i];
        if(e.reversing || seconds_<e.signalRetryAfter || (e.signalLight>=0 && e.next<e.path.size())) continue;
        double distance=0; int light=closestStoppingLight(e,&distance); e.stopTargetLight=light;
        if(light>=0) queue.push_back({distance,i});
        else e.signalRetryAfter=seconds_+.75;
    }
    if(queue.empty()) return;
    std::stable_sort(queue.begin(),queue.end(),[&](auto a,auto b) {
        return a.first==b.first?entities_[a.second].id<entities_[b.second].id:a.first<b.first;
    });
    struct Scope {
        bool& active; int& light;
        ~Scope() { active=false; light=-1; }
    } scope{planningSignalBatch_,planningSignalLight_};
    planningSignalBatch_=true; lastSignalPlanCount_=0;
    for(auto entry:queue) {
        auto& e=entities_[entry.second]; e.signalRetryAfter=seconds_+.75;
        auto randomState=random_;
        if(spreadAtSignal(e)) ++lastSignalPlanCount_;
        random_=randomState;
    }
    ++signalPlanningBatches_;
    lastSignalPlanningMilliseconds_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
    totalSignalPlanningSeconds_+=lastSignalPlanningMilliseconds_/1000;
}
}
