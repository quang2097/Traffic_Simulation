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
}
void NodeGraph::sever(int from,int to) {
    int id=pathConnection(from,to);
    if(id>=0) { connections.erase(connections.begin()+id); return; }
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
    if(std::any_of(connections.begin(),connections.end(),[](const auto& e){return e.roadType!=0;}) || std::any_of(nodes.begin(),nodes.end(),[](const auto& n){return n.openRadius>0;})) return compileRoads(*this).lanes;
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
    std::ofstream out(file); out<<std::setprecision(17)<<"TRAFFIC_NODES 6 "<<nodes.size()<<' '<<connections.size()<<' '<<explicitDestinations<<'\n';
    for(const auto& n:nodes) out<<n.position.x<<' '<<n.position.y<<' '<<n.spawn<<' '<<n.rate<<' '<<n.braking<<' '<<n.signal<<' '<<n.openRadius<<' '<<n.destination<<' '<<n.turn<<' '<<n.turnRadius<<' '<<n.excludeOwnDestination<<'\n';
    for(const auto& e:connections) {
        out<<e.from<<' '<<e.to<<' '<<e.width<<' '<<e.speed<<' '<<e.smoothing<<' '<<e.roadType<<' '<<e.laneCount<<' '<<e.shape.size()<<' '<<e.reverseLaneCount<<' '<<e.lateralOffset<<' '<<e.endWidth<<' '<<e.forwardWidths.size()<<' '<<e.reverseWidths.size()<<' '<<e.shapeWidths.size()<<'\n';
        for(auto p:e.shape) out<<p.x<<' '<<p.y<<'\n';
        for(auto width:e.forwardWidths) out<<width<<' '; out<<'\n';
        for(auto width:e.reverseWidths) out<<width<<' '; out<<'\n';
        for(auto width:e.shapeWidths) out<<width<<' '; out<<'\n';
    }
    if(!out) throw std::runtime_error("Cannot save node network");
}
NodeGraph NodeGraph::load(const std::string& file) {
    std::ifstream in(file); std::string magic; int version; std::size_t n,e;
    if(!(in>>magic>>version>>n>>e) || magic!="TRAFFIC_NODES" || version<1 || version>6 || n>200000 || e>200000) throw std::runtime_error("Invalid node network");
    NodeGraph graph; graph.nodes.resize(n);
    if(version>=4 && !(in>>graph.explicitDestinations)) throw std::runtime_error("Invalid destination policy");
    for(auto& node:graph.nodes) {
        if(!(in>>node.position.x>>node.position.y>>node.spawn>>node.rate>>node.braking) || !std::isfinite(node.position.x) || !std::isfinite(node.position.y) || !std::isfinite(node.rate) || node.rate<0 || node.rate>100 || !std::isfinite(node.braking) || node.braking<.1 || node.braking>20) throw std::runtime_error("Invalid node settings");
        if(version>=2 && !(in>>node.signal)) throw std::runtime_error("Invalid junction signal flag");
        if(version>=3 && (!(in>>node.openRadius) || !std::isfinite(node.openRadius) || node.openRadius<0 || node.openRadius>500)) throw std::runtime_error("Invalid open-junction radius");
        if(version>=4 && !(in>>node.destination)) throw std::runtime_error("Invalid destination node");
        if(version>=5 && (!(in>>node.turn>>node.turnRadius) || !std::isfinite(node.turnRadius) || node.turnRadius<.5 || node.turnRadius>500)) throw std::runtime_error("Invalid turn-node settings");
        if(version>=6 && !(in>>node.excludeOwnDestination)) throw std::runtime_error("Invalid spawn destination policy");
    }
    for(std::size_t i=0;i<e;++i) {
        RoadConnection edge;
        if(!(in>>edge.from>>edge.to>>edge.width>>edge.speed>>edge.smoothing) || !std::isfinite(edge.smoothing) || edge.smoothing<.2 || edge.smoothing>4) throw std::runtime_error("Invalid connection");
        graph.connect(edge.from,edge.to,edge.width,edge.speed);
        if(graph.connections.size()!=i+1) throw std::runtime_error("Duplicate connection");
        graph.connections.back().smoothing=edge.smoothing;
        if(version>=2) {
            auto& road=graph.connections.back(); std::size_t count;
            if(!(in>>road.roadType>>road.laneCount>>count) || road.roadType<0 || road.roadType>2 || road.laneCount<1 || road.laneCount>4 || count>10000) throw std::runtime_error("Invalid road definition");
            std::size_t forwards=0,backwards=0;
            if(version>=3 && (!(in>>road.reverseLaneCount>>road.lateralOffset>>road.endWidth>>forwards>>backwards) || road.reverseLaneCount<0 || road.reverseLaneCount>4 || !std::isfinite(road.lateralOffset) || std::abs(road.lateralOffset)>30 || !std::isfinite(road.endWidth) || road.endWidth<0 || road.endWidth>15 || forwards>4 || backwards>4)) throw std::runtime_error("Invalid asymmetric road settings");
            std::size_t shapeWidths=0;
            if(version>=4 && (!(in>>shapeWidths) || (shapeWidths && shapeWidths!=(count?count:2)))) throw std::runtime_error("Invalid point-width profile");
            road.shapeWidths.resize(shapeWidths);
            road.shape.resize(count);
            for(auto& p:road.shape) if(!(in>>p.x>>p.y) || !std::isfinite(p.x) || !std::isfinite(p.y)) throw std::runtime_error("Invalid road shape");
            road.forwardWidths.resize(forwards); road.reverseWidths.resize(backwards);
            for(auto* widths:{&road.forwardWidths,&road.reverseWidths,&road.shapeWidths}) for(auto& width:*widths) if(!(in>>width) || !std::isfinite(width) || width<.5 || width>15) throw std::runtime_error("Invalid individual lane width");
        }
    }
    return graph;
}
}
