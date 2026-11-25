#include "traffic/NodeGraph.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>
namespace traffic {
void NodeGraph::useExplicitDestinations() {
    if(explicitDestinations) return;
    std::vector<std::vector<int>> neighbors(nodes.size()); std::vector<bool> incoming(nodes.size(),false),outgoing(nodes.size(),false);
    for(const auto& road:connections) {
        for(auto pair:{std::pair<int,int>{road.from,road.to},{road.to,road.from}}) if(std::find(neighbors[pair.first].begin(),neighbors[pair.first].end(),pair.second)==neighbors[pair.first].end()) neighbors[pair.first].push_back(pair.second);
        outgoing[road.from]=true; incoming[road.to]=true;
        if(road.roadType==2) { incoming[road.from]=true; outgoing[road.to]=true; }
    }
    for(std::size_t i=0;i<nodes.size();++i) {
        bool inJunction=false; for(const auto& node:nodes) if(node.openRadius>0 && length(nodes[i].position-node.position)<=node.openRadius) inJunction=true;
        if(!inJunction && incoming[i] && (neighbors[i].size()==1 || !outgoing[i])) nodes[i].destination=true;
    }
    explicitDestinations=true;
}
int NodeGraph::addNode(Vec2 p) {
    if(!std::isfinite(p.x) || !std::isfinite(p.y)) throw std::invalid_argument("Invalid node position");
    for(std::size_t i=0;i<nodes.size();++i) if(length(nodes[i].position-p)<.05) return static_cast<int>(i);
    nodes.push_back({p}); return static_cast<int>(nodes.size()-1);
}
int NodeGraph::connection(int from,int to) const {
    for(std::size_t i=0;i<connections.size();++i) if(connections[i].from==from && connections[i].to==to) return static_cast<int>(i);
    return -1;
}
void NodeGraph::connect(int from,int to,double width,double speed) {
    if(from<0 || to<0 || from>=static_cast<int>(nodes.size()) || to>=static_cast<int>(nodes.size()) || from==to) throw std::invalid_argument("Choose two different nodes");
    if(!std::isfinite(width) || width<.5 || width>15 || !std::isfinite(speed) || speed<.1 || speed>50) throw std::invalid_argument("Invalid connection settings");
    if(pathConnection(from,to)<0) connections.push_back({from,to,width,speed,1});
    for(int node:{from,to}){int degree=0;for(auto& road:connections)degree+=road.from==node || road.to==node;if(degree>2)nodes[node].roadStation=false;}
}
int NodeGraph::splitRoad(int road,Vec2 position) {
    if(road<0 || road>=static_cast<int>(connections.size())) throw std::invalid_argument("Choose a road");
    auto original=connections[road]; auto points=original.shape;
    if(points.empty()) points={nodes[original.from].position,nodes[original.to].position};
    std::vector<double> widths=original.shapeWidths;
    if(widths.empty()) { double total=0,at=0; for(std::size_t i=1;i<points.size();++i) total+=length(points[i]-points[i-1]);
        for(std::size_t i=0;i<points.size();++i) { if(i) at+=length(points[i]-points[i-1]); widths.push_back(original.width+(original.endWidth>0?original.endWidth-original.width:0)*at/std::max(.001,total)); } }
    double best=1e100,tBest=0; std::size_t segment=1; Vec2 split;
    for(std::size_t i=1;i<points.size();++i) { auto d=points[i]-points[i-1]; double t=std::clamp(((position.x-points[i-1].x)*d.x+(position.y-points[i-1].y)*d.y)/std::max(1e-9,d.x*d.x+d.y*d.y),0.,1.); auto q=points[i-1]+d*t; double gap=length(position-q); if(gap<best) {best=gap;split=q;segment=i;tBest=t;} }
    if(length(split-points.front())<.1) return original.from;
    if(length(split-points.back())<.1) return original.to;
    int node=addNode(split); if(node==original.from || node==original.to) return node;
    double width=widths[segment-1]+(widths[segment]-widths[segment-1])*tBest;
    auto first=original,second=original; first.to=node; second.from=node;
    double cut=0; for(std::size_t i=1;i<segment;++i) cut+=length(points[i]-points[i-1]);
    cut+=length(points[segment]-points[segment-1])*tBest;
    first.changeSections.clear(); second.changeSections.clear();
    first.laneTurns.erase(std::remove_if(first.laneTurns.begin(),first.laneTurns.end(),[](const auto& r){return r.direction==0;}),first.laneTurns.end());
    second.laneTurns.erase(std::remove_if(second.laneTurns.begin(),second.laneTurns.end(),[](const auto& r){return r.direction==1;}),second.laneTurns.end());
    for(auto rule:original.changeSections) {
        if(rule.begin<cut) { auto piece=rule;piece.end=std::min(piece.end,cut);if(piece.end>piece.begin)first.changeSections.push_back(piece); }
        if(rule.end>cut) { rule.begin=std::max(0.,rule.begin-cut);rule.end-=cut;second.changeSections.push_back(rule); }
    }
    first.shape.assign(points.begin(),points.begin()+segment); first.shapeWidths.assign(widths.begin(),widths.begin()+segment);
    second.shape.assign(points.begin()+segment,points.end()); second.shapeWidths.assign(widths.begin()+segment,widths.end());
    if(length(first.shape.back()-split)<.05) { first.shape.back()=split; first.shapeWidths.back()=width; } else {first.shape.push_back(split);first.shapeWidths.push_back(width);}
    if(length(second.shape.front()-split)<.05) {second.shape.front()=split;second.shapeWidths.front()=width;} else {second.shape.insert(second.shape.begin(),split);second.shapeWidths.insert(second.shapeWidths.begin(),width);}
    connections[road]=first; connections.push_back(second);
    for(auto& edge:connections)for(auto& rule:edge.laneTurns){int via=rule.direction?edge.from:edge.to;
        for(auto& exit:rule.exits)if(exit.road<0 && ((via==original.from && exit.node==original.to) || (via==original.to && exit.node==original.from)))exit.node=node;}
    for(auto& edge:connections)for(auto& rule:edge.laneTurns)for(auto& exit:rule.exits)if(exit.road==road && exit.direction==1)exit.road=static_cast<int>(connections.size())-1;
    return node;
}
int NodeGraph::addUTurnOpening(int road,double begin,double end) {
    if(road<0 || road>=static_cast<int>(connections.size()) || connections[road].roadType!=2) throw std::invalid_argument("Choose a two-way road for a U-turn opening");
    auto points=connections[road].shape;
    if(points.empty()) points={nodes[connections[road].from].position,nodes[connections[road].to].position};
    double distance=0; for(std::size_t i=1;i<points.size();++i) distance+=length(points[i]-points[i-1]);
    if(!std::isfinite(begin) || !std::isfinite(end) || begin<1 || end>distance-1 || end-begin<2) throw std::invalid_argument("U-turn opening must be at least 2 m long and inside the road");
    double middle=(begin+end)*.5;
    if((end-begin)*.5>std::min(middle,distance-middle)*.4) throw std::invalid_argument("U-turn opening is too long for its approach roads; shorten it or move it farther from the ends");
    double station=(begin+end)*.5; Vec2 point=points.back();
    for(std::size_t i=1;i<points.size();++i) {double span=length(points[i]-points[i-1]);if(station<=span){point=points[i-1]+(points[i]-points[i-1])*(station/span);break;}station-=span;}
    int node=splitRoad(road,point); nodes[node].allowUTurn=true; nodes[node].internalRadius=(end-begin)*.5;
    return node;
}
void NodeGraph::sever(int from,int to) {
    int id=pathConnection(from,to);
    if(id>=0) { connections.erase(connections.begin()+id);for(auto& road:connections)for(auto& rule:road.laneTurns)for(auto& exit:rule.exits)if(exit.road>=id){if(exit.road==id){exit.lanes.clear();exit.priorities.clear();}else --exit.road;} pruneLaneTurns();return; }
    connections.erase(std::remove_if(connections.begin(),connections.end(),[&](const auto& e){ return e.from==from && e.to==to; }),connections.end());
}
int NodeGraph::pathConnection(int from,int to) const {
    int direct=connection(from,to); if(direct>=0) return direct;
    int reverse=connection(to,from);
    return reverse>=0 && connections[reverse].roadType==2?reverse:-1;
}
bool NodeGraph::extendSelection(std::vector<int>& path,int node) const {
    if(node<0 || node>=static_cast<int>(nodes.size())) return false;
    if(!path.empty() && pathConnection(path.back(),node)<0) return false;
    if(std::find(path.begin(),path.end(),node)!=path.end()) return false;
    path.push_back(node); return true;
}
std::vector<DirectedLane> NodeGraph::lanes() const {
    if(std::any_of(connections.begin(),connections.end(),[](const auto& e){return e.roadType!=0 || e.laneCount!=1 || e.reverseLaneCount>0 || !e.changeSections.empty() || !e.laneTurns.empty();}) || std::any_of(nodes.begin(),nodes.end(),[](const auto& n){return n.openRadius>0 || n.allowUTurn || n.internalRadius>0 || n.roadStation;})) return compileRoads(*this).lanes;
    std::vector<DirectedLane> result;
    for(const auto& edge:connections) {
        if(edge.from<0 || edge.to<0 || edge.from>=static_cast<int>(nodes.size()) || edge.to>=static_cast<int>(nodes.size()) || edge.from==edge.to) throw std::invalid_argument("Invalid node connection");
        const auto& start=nodes[edge.from]; const auto& end=nodes[edge.to];
        DirectedLane lane; lane.points={start.position,end.position}; lane.entry=start.spawn;
        lane.destination=end.destination; lane.allowTripEnd=!explicitDestinations || end.destination;
        lane.sourceNode=edge.from; lane.destinationNode=edge.to; lane.excludeOwnDestination=start.excludeOwnDestination;
        lane.width=edge.width; lane.speedLimit=edge.speed; lane.spawnRate=start.rate;
        lane.turnDeceleration=start.braking; lane.pointBraking={start.braking,end.braking};
        lane.turnSmoothing=edge.smoothing; lane.inferConnections=false;
        for(std::size_t i=0;i<connections.size();++i) if(connections[i].from==edge.to && connections[i].to!=edge.from) lane.successors.push_back(static_cast<int>(i));
        result.push_back(std::move(lane));
    }
    std::vector<TurnControl> controls; for(const auto& node:nodes) if(node.turn) controls.push_back({node.position,node.turnRadius,node.braking});
    for(auto& lane:result) { lane.explicitTurns=true; lane.turnControls=controls; }
    return result;
}
NodeGraph NodeGraph::fromLanes(const std::vector<DirectedLane>& lanes) {
    NodeGraph graph;
    for(const auto& lane:lanes) {
        int previous=-1;
        for(std::size_t i=0;i<lane.points.size();++i) {
            int id=graph.addNode(lane.points[i]);
            graph.nodes[id].braking=i<lane.pointBraking.size() && lane.pointBraking[i]>0?lane.pointBraking[i]:lane.turnDeceleration;
            if(i==0 && lane.entry) { graph.nodes[id].spawn=true; graph.nodes[id].rate=lane.spawnRate; graph.nodes[id].excludeOwnDestination=lane.excludeOwnDestination; }
            if(previous>=0) { graph.connect(previous,id,lane.width,lane.speedLimit); graph.connections[graph.connection(previous,id)].smoothing=lane.turnSmoothing; }
            previous=id;
        }
    }
    for(const auto& lane:lanes) for(const auto& c:lane.turnControls) { int node=graph.addNode(c.point); graph.nodes[node].turn=true; graph.nodes[node].turnRadius=c.radius; graph.nodes[node].braking=c.braking; }
    return graph;
}
void NodeGraph::save(const std::string& file) const {
    validateLaneTurns();
    std::ofstream out(file); out<<std::setprecision(17)<<"TRAFFIC_NODES 11 "<<nodes.size()<<' '<<connections.size()<<' '<<explicitDestinations<<'\n';
    for(const auto& n:nodes) out<<n.position.x<<' '<<n.position.y<<' '<<n.spawn<<' '<<n.rate<<' '<<n.braking<<' '<<n.signal<<' '<<n.openRadius<<' '<<n.destination<<' '<<n.turn<<' '<<n.turnRadius<<' '<<n.excludeOwnDestination<<' '<<n.allowUTurn<<' '<<n.internalRadius<<' '<<n.roadStation<<'\n';
    for(const auto& e:connections) {
        out<<e.from<<' '<<e.to<<' '<<e.width<<' '<<e.speed<<' '<<e.smoothing<<' '<<e.roadType<<' '<<e.laneCount<<' '<<e.shape.size()<<' '<<e.reverseLaneCount<<' '<<e.lateralOffset<<' '<<e.endWidth<<' '<<e.forwardWidths.size()<<' '<<e.reverseWidths.size()<<' '<<e.shapeWidths.size()<<' '<<e.changeSections.size()<<' '<<e.laneTurns.size()<<'\n';
        for(auto p:e.shape) out<<p.x<<' '<<p.y<<'\n';
        for(auto width:e.forwardWidths) out<<width<<' '; out<<'\n';
        for(auto width:e.reverseWidths) out<<width<<' '; out<<'\n';
        for(auto width:e.shapeWidths) out<<width<<' '; out<<'\n';
        for(const auto& rule:e.changeSections) out<<rule.begin<<' '<<rule.end<<' '<<rule.direction<<' '<<rule.lane<<' '<<rule.allowEnter<<' '<<rule.allowExit<<'\n';
        for(const auto& rule:e.laneTurns){out<<rule.direction<<' '<<rule.lane<<' '<<rule.exits.size()<<'\n';
            for(const auto& exit:rule.exits){out<<exit.node<<' '<<exit.lanes.size();for(int lane:exit.lanes)out<<' '<<lane;for(std::size_t i=0;i<exit.lanes.size();++i)out<<' '<<(i<exit.priorities.size()?exit.priorities[i]:1);out<<' '<<exit.road<<' '<<exit.direction<<'\n';}}

    }
    if(!out) throw std::runtime_error("Cannot save node network");
}
NodeGraph NodeGraph::load(const std::string& file) {
    std::ifstream in(file); std::string magic; int version; std::size_t n,e;
    if(!(in>>magic>>version>>n>>e) || magic!="TRAFFIC_NODES" || version<1 || version>11 || n>200000 || e>200000) throw std::runtime_error("Invalid node network");
    NodeGraph graph; graph.nodes.resize(n);
    if(version>=4 && !(in>>graph.explicitDestinations)) throw std::runtime_error("Invalid destination policy");
    for(auto& node:graph.nodes) {
        if(!(in>>node.position.x>>node.position.y>>node.spawn>>node.rate>>node.braking) || !std::isfinite(node.position.x) || !std::isfinite(node.position.y) || !std::isfinite(node.rate) || node.rate<0 || node.rate>100 || !std::isfinite(node.braking) || node.braking<.1 || node.braking>20) throw std::runtime_error("Invalid node settings");
        if(version>=2 && !(in>>node.signal)) throw std::runtime_error("Invalid junction signal flag");
        if(version>=3 && (!(in>>node.openRadius) || !std::isfinite(node.openRadius) || node.openRadius<0 || node.openRadius>500)) throw std::runtime_error("Invalid open-junction radius");
        if(version>=4 && !(in>>node.destination)) throw std::runtime_error("Invalid destination node");
        if(version>=5 && (!(in>>node.turn>>node.turnRadius) || !std::isfinite(node.turnRadius) || node.turnRadius<.5 || node.turnRadius>500)) throw std::runtime_error("Invalid turn-node settings");
        if(version>=6 && !(in>>node.excludeOwnDestination)) throw std::runtime_error("Invalid spawn destination policy");
        if(version>=7 && !(in>>node.allowUTurn)) throw std::runtime_error("Invalid U-turn policy");
        if(version>=8 && (!(in>>node.internalRadius) || !std::isfinite(node.internalRadius) || node.internalRadius<0 || node.internalRadius>500)) throw std::runtime_error("Invalid internal junction extent");
        if(version>=9 && !(in>>node.roadStation))throw std::runtime_error("Invalid road-station flag");
    }
    for(std::size_t i=0;i<e;++i) {
        RoadConnection edge;
        if(!(in>>edge.from>>edge.to>>edge.width>>edge.speed>>edge.smoothing) || !std::isfinite(edge.smoothing) || edge.smoothing<.2 || edge.smoothing>4) throw std::runtime_error("Invalid connection");
        graph.connect(edge.from,edge.to,edge.width,edge.speed);
        if(graph.connections.size()!=i+1) throw std::runtime_error("Duplicate connection");
        graph.connections.back().smoothing=edge.smoothing;
        if(version>=2) {
            auto& road=graph.connections.back(); std::size_t count;
            if(!(in>>road.roadType>>road.laneCount>>count) || road.roadType<0 || road.roadType>2 || road.laneCount<1 || road.laneCount>maximumRoadLanes || count>10000) throw std::runtime_error("Invalid road definition");
            std::size_t forwards=0,backwards=0;
            if(version>=3 && (!(in>>road.reverseLaneCount>>road.lateralOffset>>road.endWidth>>forwards>>backwards) || road.reverseLaneCount<0 || road.reverseLaneCount>maximumRoadLanes || !std::isfinite(road.lateralOffset) || std::abs(road.lateralOffset)>30 || !std::isfinite(road.endWidth) || road.endWidth<0 || road.endWidth>15 || forwards>maximumRoadLanes || backwards>maximumRoadLanes)) throw std::runtime_error("Invalid asymmetric road settings");
            std::size_t shapeWidths=0;
            if(version>=4 && (!(in>>shapeWidths) || (shapeWidths && shapeWidths!=(count?count:2)))) throw std::runtime_error("Invalid point-width profile");
            std::size_t sections=0;
            if(version>=8 && (!(in>>sections) || sections>10000)) throw std::runtime_error("Invalid lane section count");
            std::size_t turnRules=0;if(version>=9 && (!(in>>turnRules) || turnRules>maximumRoadLanes*2))throw std::runtime_error("Invalid lane connection count");
            road.shapeWidths.resize(shapeWidths);
            road.shape.resize(count);
            for(auto& p:road.shape) if(!(in>>p.x>>p.y) || !std::isfinite(p.x) || !std::isfinite(p.y)) throw std::runtime_error("Invalid road shape");
            road.forwardWidths.resize(forwards); road.reverseWidths.resize(backwards);
            for(auto* widths:{&road.forwardWidths,&road.reverseWidths,&road.shapeWidths}) for(auto& width:*widths) if(!(in>>width) || !std::isfinite(width) || width<.5 || width>15) throw std::runtime_error("Invalid individual lane width");
            road.changeSections.resize(sections);
            for(auto& rule:road.changeSections) if(!(in>>rule.begin>>rule.end>>rule.direction>>rule.lane>>rule.allowEnter>>rule.allowExit) || !std::isfinite(rule.begin) || !std::isfinite(rule.end) || rule.begin<0 || rule.end<=rule.begin || rule.direction< -1 || rule.direction>1 || rule.lane< -1 || rule.lane>=maximumRoadLanes) throw std::runtime_error("Invalid lane section");
            road.laneTurns.resize(turnRules);
            for(auto& rule:road.laneTurns){std::size_t exits;if(!(in>>rule.direction>>rule.lane>>exits) || exits>graph.connections.size()*2+graph.nodes.size())throw std::runtime_error("Invalid lane exits");rule.exits.resize(exits);
                for(auto& exit:rule.exits){std::size_t lanes;if(!(in>>exit.node>>lanes) || lanes>maximumRoadLanes)throw std::runtime_error("Invalid outgoing lane list");exit.lanes.resize(lanes);for(int& lane:exit.lanes)if(!(in>>lane))throw std::runtime_error("Incomplete outgoing lanes");if(version>=10){exit.priorities.resize(lanes);for(int& priority:exit.priorities)if(!(in>>priority))throw std::runtime_error("Incomplete movement priorities");}if(version>=11 && !(in>>exit.road>>exit.direction))throw std::runtime_error("Incomplete semantic lane target");}}
        }
    }
    graph.validateLaneTurns();
    return graph;
}
}
