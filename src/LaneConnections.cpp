#include "traffic/NodeGraph.hpp"
#include <algorithm>
#include <set>
#include <tuple>
#include <regex>
#include <cctype>
#include <stdexcept>

namespace traffic {
std::vector<int> parseNumberList(const std::string& text,int minimum,int maximum){
    std::string compact;for(unsigned char c:text)if(!std::isspace(c))compact.push_back(static_cast<char>(std::tolower(c)));
    if(compact=="all" || compact=="*"){std::vector<int> out;for(int i=minimum;i<=maximum;++i)out.push_back(i);return out;}
    static const std::regex token(R"((\d+)(?:\s*-\s*(\d+))?)");std::set<int> values;std::size_t consumed=0;
    auto separator=[&](std::size_t end){for(;consumed<end;++consumed)if(!std::isspace(static_cast<unsigned char>(text[consumed])) && text[consumed]!=',')throw std::invalid_argument("Use numbers separated by commas, or ranges such as 2-6");};
    for(std::sregex_iterator i(text.begin(),text.end(),token),end;i!=end;++i){separator(static_cast<std::size_t>(i->position()));int first=std::stoi((*i)[1]),last=(*i)[2].matched?std::stoi((*i)[2]):first;
        if(first<minimum || last>maximum || first>last)throw std::invalid_argument("Numbers must be "+std::to_string(minimum)+"-"+std::to_string(maximum));
        for(int n=first;n<=last;++n)values.insert(n);consumed=static_cast<std::size_t>(i->position()+i->length());}
    separator(text.size());return {values.begin(),values.end()};
}
int NodeGraph::junctionAt(int node) const {
    if(node<0 || node>=static_cast<int>(nodes.size()))throw std::invalid_argument("Invalid junction node");
    if(nodes[node].openRadius>0)return node;
    int best=node;double nearest=1e100;
    double margin=0;if(nodes[node].roadStation)for(const auto& r:connections)if(r.from==node || r.to==node)margin=std::max(margin,r.width*.5);
    for(std::size_t n=0;n<nodes.size();++n)if(nodes[n].openRadius>0){double d=length(nodes[n].position-nodes[node].position);if(d<=nodes[n].openRadius+margin && d<nearest){best=static_cast<int>(n);nearest=d;}}
    return best;
}
int NodeGraph::directionalLaneCount(int road,int direction) const {
    if(road<0 || road>=static_cast<int>(connections.size()) || direction<0 || direction>1)throw std::invalid_argument("Invalid road direction");
    const auto& r=connections[road];if(direction && r.roadType!=2)throw std::invalid_argument("A one-way road has no reverse lanes");
    return direction && r.reverseLaneCount>0?r.reverseLaneCount:r.laneCount;
}
const LaneTurnRule* NodeGraph::laneTurnRule(int road,int direction,int lane) const {
    for(const auto& rule:connections.at(road).laneTurns)if(rule.direction==direction && rule.lane==lane)return &rule;
    return nullptr;
}
std::vector<std::pair<int,int>> NodeGraph::targetRoads(int sourceRoad,int sourceDirection,const LaneExit& exit) const {
    std::vector<std::pair<int,int>> found,local;
    if(exit.road>=0){if(exit.road>=static_cast<int>(connections.size()) || exit.direction<0 || exit.direction>1)return found;const auto& r=connections[exit.road];if(exit.direction && r.roadType!=2)return found;if((exit.direction?r.to:r.from)==exit.node)found.push_back({exit.road,exit.direction});return found;}
    int via=junctionAt(sourceDirection?connections[sourceRoad].from:connections[sourceRoad].to);
    for(std::size_t i=0;i<connections.size();++i)for(int d=0;d<(connections[i].roadType==2?2:1);++d){auto& r=connections[i];int entry=d?r.to:r.from,end=d?r.from:r.to;
        if(junctionAt(entry)==junctionAt(end))continue;
        if(end==exit.node || (entry==exit.node && junctionAt(entry)==via)){
            found.push_back({static_cast<int>(i),d});if(junctionAt(entry)==via)local.push_back({static_cast<int>(i),d});}}
    return local.empty()?found:local;
}
bool NodeGraph::turnAllowed(int in,int inDirection,int inLane,int out,int outDirection,int outLane) const {
    auto rule=laneTurnRule(in,inDirection,inLane);if(!rule)return true;
    for(const auto& exit:rule->exits)if(std::find(exit.lanes.begin(),exit.lanes.end(),outLane)!=exit.lanes.end())for(auto target:targetRoads(in,inDirection,exit))if(target==std::make_pair(out,outDirection))return true;
    return false;
}
void NodeGraph::validateLaneTurns() const {
    for(std::size_t r=0;r<connections.size();++r){const auto& road=connections[r];
        if(road.laneCount<1 || road.laneCount>maximumRoadLanes || road.reverseLaneCount<0 || road.reverseLaneCount>maximumRoadLanes)throw std::invalid_argument("Roads support 1-32 lanes per direction");
        std::set<std::pair<int,int>> seen;
        for(const auto& rule:road.laneTurns){
            if(rule.lane<0 || rule.lane>=directionalLaneCount(static_cast<int>(r),rule.direction) || !seen.insert({rule.direction,rule.lane}).second)throw std::invalid_argument("Invalid or duplicate incoming lane rule");
            std::set<std::tuple<int,int,int>> targets;
            for(const auto& exit:rule.exits){
                if(exit.node<0 || exit.node>=static_cast<int>(nodes.size()) || exit.lanes.empty() || !targets.insert({exit.node,exit.road,exit.direction}).second)throw std::invalid_argument("Invalid or duplicate exit node");
                int count=0;for(auto target:targetRoads(static_cast<int>(r),rule.direction,exit))count=std::max(count,directionalLaneCount(target.first,target.second));
                if(!count)throw std::invalid_argument("Target node "+std::to_string(exit.node)+" has no selected lane carriageway");
                std::set<int> lanes;for(int lane:exit.lanes)if(lane<0 || lane>=count || !lanes.insert(lane).second)throw std::invalid_argument("Exit lanes must be 1-"+std::to_string(count)+" in the correct outgoing direction");
                if(!exit.priorities.empty() && exit.priorities.size()!=exit.lanes.size())throw std::invalid_argument("Each target lane needs a priority");
                for(int priority:exit.priorities)if(priority<1 || priority>1000000)throw std::invalid_argument("Priority must be 1-1000000 (1 is highest)");
            }
        }
    }
}
void NodeGraph::setLaneTurnRule(int road,LaneTurnRule rule){
    auto proposed=*this;auto& rules=proposed.connections.at(road).laneTurns;
    rules.erase(std::remove_if(rules.begin(),rules.end(),[&](const auto& old){return old.direction==rule.direction && old.lane==rule.lane;}),rules.end());
    rules.push_back(std::move(rule));proposed.validateLaneTurns();connections[road].laneTurns=std::move(rules);
}
void NodeGraph::pruneLaneTurns(){
    for(std::size_t r=0;r<connections.size();++r){auto& road=connections[r];auto& rules=road.laneTurns;
        rules.erase(std::remove_if(rules.begin(),rules.end(),[&](const auto& rule){return rule.direction<0 || rule.direction>1 || (rule.direction && road.roadType!=2) || rule.lane<0 || rule.lane>=directionalLaneCount(static_cast<int>(r),rule.direction);}),rules.end());
        for(auto& rule:rules){
            for(auto& exit:rule.exits){int count=0;for(auto target:targetRoads(static_cast<int>(r),rule.direction,exit))count=std::max(count,directionalLaneCount(target.first,target.second));
                for(std::size_t i=exit.lanes.size();i-->0;)if(exit.lanes[i]<0 || exit.lanes[i]>=count){exit.lanes.erase(exit.lanes.begin()+i);if(i<exit.priorities.size())exit.priorities.erase(exit.priorities.begin()+i);}
            }
            rule.exits.erase(std::remove_if(rule.exits.begin(),rule.exits.end(),[](const auto& e){return e.lanes.empty();}),rule.exits.end());
        }
    }
}
int NodeGraph::turnPriority(int road,int direction,int lane,int targetNode,int targetLane) const {
    if(auto rule=laneTurnRule(road,direction,lane))for(const auto& exit:rule->exits)for(auto target:targetRoads(road,direction,exit)){
        auto& r=connections[target.first];if((target.second?r.from:r.to)!=targetNode)continue;
        for(std::size_t i=0;i<exit.lanes.size();++i)if(exit.lanes[i]==targetLane)return i<exit.priorities.size()?exit.priorities[i]:1;
    }return 1;
}
std::vector<std::string> NodeGraph::missingLaneExits() const {
    validateLaneTurns();std::vector<std::string> missing;
    for(std::size_t r=0;r<connections.size();++r){const auto& road=connections[r];
        if(junctionAt(road.from)==junctionAt(road.to))continue;
        for(int d=0;d<(road.roadType==2?2:1);++d){int node=d?road.from:road.to;
            // A declared destination may terminate trips without an exit.
            if(nodes[node].destination)continue;
            for(int lane=0;lane<directionalLaneCount(static_cast<int>(r),d);++lane){auto rule=laneTurnRule(static_cast<int>(r),d,lane);
                bool usable=false;if(rule)for(std::size_t out=0;out<connections.size();++out)for(int direction=0;direction<(connections[out].roadType==2?2:1);++direction){auto& candidate=connections[out];int entry=direction?candidate.to:candidate.from,exit=direction?candidate.from:candidate.to;if(junctionAt(entry)==junctionAt(exit))continue;for(int target=0;target<directionalLaneCount(static_cast<int>(out),direction);++target)usable=usable || turnAllowed(static_cast<int>(r),d,lane,static_cast<int>(out),direction,target);}
                if(!usable)missing.push_back("Node "+std::to_string(node)+", road "+std::to_string(r)+", "+(d?"reverse":"forward")+" lane "+std::to_string(lane+1));}
        }
    }return missing;
}
}
