#include "traffic/NodeGraph.hpp"
#include <algorithm>
#include <iostream>
#include <stdexcept>
using namespace traffic;
void need(bool b,const char* s){if(!b)throw std::runtime_error(s);}
int road(const std::vector<DirectedLane>& lanes,int from,int to){for(std::size_t i=0;i<lanes.size();++i)if(lanes[i].roadGroup>=0 && lanes[i].sourceNode==from && lanes[i].destinationNode==to)return static_cast<int>(i);throw std::runtime_error("missing directed road");}
bool reaches(const std::vector<DirectedLane>& lanes,int a,int b){std::vector<int> todo{a};std::vector<bool> seen(lanes.size());seen[a]=true;for(std::size_t i=0;i<todo.size();++i){if(todo[i]==b)return true;for(int n:lanes[todo[i]].successors)if(!seen[n]){seen[n]=true;todo.push_back(n);}}return false;}
void trip(const NodeGraph& g,int from,int to){Simulation s(Map(160,160,std::vector<Tile>(25600,Tile::Road)),42);s.setLanes(g.lanes());s.setTrafficFlowSettings({true,true,.4,.15});s.setStuckVehicleSettings({false,30});s.setProfile(VehicleKind::Car,{{2,0},{1,0},{.3,0},{5,0,2,0},1});need(s.spawnOnLane(from,VehicleKind::Car,to),"trip insertion failed");for(int i=0;i<16000 && !s.entities().empty();++i){s.tick(.01);for(const auto& e:s.entities())need(s.map().fits(e.footprint()),"left road");}need(s.completed()==1,"junction trip failed to finish");}
int main(){try{
 NodeGraph g;g.addNode({20,80});g.addNode({140,80});g.addNode({80,140});g.connect(0,1,4,6);g.connections[0].roadType=1;g.connections[0].shape={{20,80},{60,80},{140,80}};g.connections[0].shapeWidths={4,5,6};
 int j=g.splitRoad(0,{80,82});need(length(g.nodes[j].position-Vec2{80,80})<1e-8,"split must project to centerline");need(g.connections.size()==2 && g.connections[0].to==j && g.connections[1].from==j,"split lost straight route");need(std::abs(g.connections[0].shapeWidths.back()-5.25)<1e-8 && g.connections[1].shapeWidths.front()==g.connections[0].shapeWidths.back(),"split lost taper");
 g.connect(j,2,4,6);g.connections.back().roadType=2;g.explicitDestinations=true;g.nodes[1].destination=g.nodes[2].destination=true;auto lanes=g.lanes();int in=road(lanes,0,j),straight=road(lanes,j,1),side=road(lanes,j,2),back=road(lanes,2,j);
 need(reaches(lanes,in,straight)&&reaches(lanes,in,side)&&reaches(lanes,back,straight),"mixed T junction lost through/turn/merge");need(std::none_of(lanes.begin(),lanes.end(),[&](const auto& l){return l.roadGroup>=0 && l.sourceNode==j && l.destinationNode==0;}),"one-way became reversible");trip(g,in,straight);trip(g,in,side);trip(g,back,straight);
 {
 Simulation s(Map(160,160,std::vector<Tile>(25600,Tile::Road)),7);s.setLanes(g.lanes());s.setTrafficFlowSettings({true,true,.4,.15});s.setStuckVehicleSettings({false,30});s.setProfile(VehicleKind::Car,{{2,0},{1,0},{.3,0},{5,0,2,0},1});
 need(s.spawnOnLane(in,VehicleKind::Car,straight),"through insertion");s.tick(3);need(s.spawnOnLane(in,VehicleKind::Car,side),"turn insertion");need(s.spawnOnLane(back,VehicleKind::Car,straight),"merge insertion");
 for(int i=0;i<20000 && !s.entities().empty();++i){s.tick(.01);for(const auto& e:s.entities())for(const auto& o:s.entities())if(e.id<o.id)need(clearance(e.footprint(),o.footprint())>=.3-1e-5,"mixed T spacing violation");}
 need(s.completed()==3 && s.removedStuckVehicles()==0,"mixed through/turn/merge did not drain");
 }
 int north=g.addNode({80,20});g.connect(north,j,4,6);g.connections.back().roadType=2;g.nodes[north].destination=true;lanes=g.lanes();need(reaches(lanes,road(lanes,0,j),road(lanes,j,north)),"four-way branch missing");trip(g,road(lanes,north,j),road(lanes,j,2));
 NodeGraph u;u.addNode({30,50});u.addNode({110,50});u.connect(0,1,4,6);u.connections[0].roadType=2;u.explicitDestinations=true;u.nodes[0].destination=true;auto before=u.lanes();need(!reaches(before,road(before,0,1),road(before,1,0)),"U turn should require permission");u.nodes[1].allowUTurn=true;auto after=u.lanes();need(reaches(after,road(after,0,1),road(after,1,0)),"U turn permission missing");trip(u,road(after,0,1),road(after,1,0));u.save("junction-u.nodes");need(NodeGraph::load("junction-u.nodes").nodes[1].allowUTurn,"U turn permission did not persist");
 std::cout<<"Mixed T through/turn/merge, four-way, directed U-turn and tapered split passed\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
