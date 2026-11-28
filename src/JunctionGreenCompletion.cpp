#include "traffic/Simulation.hpp"
#include <algorithm>

namespace traffic {
std::vector<std::uint64_t> Simulation::greenJunctionCompletionVehicles() {
    std::vector<const TrafficLight*> clear;
    for(auto& state:greenCompletionStates_) {
        auto light=std::find_if(lights_.begin(),lights_.end(),[&](const auto& l){return l.id==state.id;});
        if(light==lights_.end())continue;
        const auto phase=light->phase(seconds_);
        if(light->enabled && light->completeJunctionOnGreen && light->completionJunction>=0 &&
           state.phase!=LightPhase::Green && phase==LightPhase::Green)clear.push_back(&*light);
        state.phase=phase;
    }
    if(clear.empty())return {};
    std::vector<std::uint64_t> result;
    for(const auto& e:entities_)if(std::any_of(clear.begin(),clear.end(),[&](const auto* light){
        return length(e.center-light->completionCenter)<=light->completionRadius+1e-9;
    }))result.push_back(e.id);
    return result;
}
void Simulation::completeGreenJunctions() {
    if(recoveryPreview_)return;
    const auto ids=greenJunctionCompletionVehicles();if(ids.empty())return;
    entities_.erase(std::remove_if(entities_.begin(),entities_.end(),[&](const Entity& e){
        if(std::find(ids.begin(),ids.end(),e.id)==ids.end())return false;
        recordFinishedStopTime(e);++completed_;++completedOnGreen_;return true;
    }),entities_.end());
}
}
