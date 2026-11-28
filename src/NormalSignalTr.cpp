#include "traffic/Simulation.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <map>
#include <sstream>
namespace traffic {
double TrafficLight::trRemaining(double seconds) const {
    if(!enabled || externallyControlled || !normalTr.active || seconds<normalTr.epoch+normalTr.guard)return -1;
    double t=std::fmod(seconds-normalTr.epoch-normalTr.guard,normalTr.cycle);if(t<0)t+=normalTr.cycle;
    for(int group=0;group<2;++group)if(normalTr.trDuration[group]>0 && t>=normalTr.trStart[group] && t<normalTr.trStart[group]+normalTr.trDuration[group])return normalTr.trStart[group]+normalTr.trDuration[group]-t;
    return -1;
}
void Simulation::configureNormalTr(std::vector<TrafficLight>& lights){
    std::map<int,std::vector<std::size_t>> junctions;
    for(std::size_t i=0;i<lights.size();++i){auto& l=lights[i];l.normalTr={};if(l.enabled && l.trSeconds>0 && !l.externallyControlled && !l.suspendNormalTr)junctions[l.trJunction].push_back(i);}
    std::vector<NormalTrCycle> next;
    for(auto& [junction,members]:junctions){
        std::sort(members.begin(),members.end(),[&](auto a,auto b){return lights[a].id<lights[b].id;});
        std::array<double,2> green{},yellow{},tr{},start{},trStart{};
        std::ostringstream signature;signature.imbue(std::locale::classic());signature<<std::setprecision(17);
        for(auto i:members){const auto& l=lights[i];int g=l.trGroup;green[g]=std::max(green[g],l.greenSeconds);yellow[g]=std::max(yellow[g],l.yellowSeconds);tr[g]=std::max(tr[g],l.trSeconds);signature<<l.id<<':'<<g<<':'<<l.greenSeconds<<':'<<l.yellowSeconds<<':'<<l.trSeconds<<';';}
        NormalTrCycle state;state.junction=junction;state.signature=signature.str();state.epoch=seconds_;
        for(auto i:members)state.guard=std::max(state.guard,lights[i].yellowSeconds);
        auto old=std::find_if(normalTrCycles_.begin(),normalTrCycles_.end(),[&](const auto& c){return c.junction==junction && c.signature==state.signature;});
        if(old!=normalTrCycles_.end())state=*old;
        else {bool anyYellow=false;for(auto i:members){auto previous=std::find_if(lights_.begin(),lights_.end(),[&](const auto& l){return l.id==lights[i].id;});bool yellowGuard=(previous==lights_.end()?lights[i].phase(seconds_):previous->phase(seconds_))!=LightPhase::Red;state.guardYellow.push_back({lights[i].id,yellowGuard});anyYellow=anyYellow || yellowGuard;}if(!anyYellow)state.guard=0;}
        double cycle=0;
        for(int g=0;g<2;++g)if(green[g]>0){trStart[g]=cycle;start[g]=cycle+tr[g];cycle+=tr[g]+green[g]+yellow[g];}
        for(auto i:members){auto& l=lights[i];int g=l.trGroup;auto guard=std::find_if(state.guardYellow.begin(),state.guardYellow.end(),[&](auto p){return p.first==l.id;});
            l.normalTr={true,guard!=state.guardYellow.end() && guard->second,state.epoch,state.guard,cycle,start[g],green[g],yellow[g],trStart,tr};}
        next.push_back(std::move(state));
    }
    normalTrCycles_=std::move(next);
}
}
