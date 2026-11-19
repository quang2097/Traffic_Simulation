#include "traffic/NodeGraph.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace traffic;
void need(bool b,const char* msg){if(!b)throw std::runtime_error(msg);}
int lane(const Simulation& s,int a,int b){for(std::size_t i=0;i<s.lanes().size();++i)if(s.lanes()[i].roadGroup>=0 && s.lanes()[i].sourceNode==a && s.lanes()[i].destinationNode==b)return static_cast<int>(i);throw std::runtime_error("lane missing");}
int main(){try{
 NodeGraph g;for(auto p:std::vector<Vec2>{{10,50},{40,50},{40,90},{100,50},{115,50}})g.addNode(p);g.explicitDestinations=true;g.nodes[4].destination=true;
 for(auto pair:{std::pair<int,int>{0,1},{1,3},{1,2},{2,3},{3,4}}){g.connect(pair.first,pair.second,4,pair.second==3 && pair.first==1?1:30);g.connections.back().roadType=1;}
 for(unsigned seed=1;seed<=20;++seed){Simulation s(Map(120,120,std::vector<Tile>(14400,Tile::Road)),seed);s.setLanes(g.lanes());s.setProfile(VehicleKind::Car,{{2,0},{1,0},{.3,0},{5,0,2,0},1});need(s.spawnOnLane(lane(s,0,1),VehicleKind::Car,lane(s,3,4)),"shortest fixture spawn");for(int id:s.entities()[0].networkRoute)need(s.lanes()[id].sourceNode!=2 && s.lanes()[id].destinationNode!=2,"route took faster but longer detour");}
 NodeGraph cross;for(auto p:std::vector<Vec2>{{150,150},{20,150},{280,150},{150,20},{150,280}})cross.addNode(p);cross.nodes[0].openRadius=60;cross.explicitDestinations=true;
 for(int i=1;i<=4;++i){cross.nodes[i].destination=true;cross.connect(i,0,6,10);cross.connections.back().roadType=2;}
 for(int destination:{2,3,4}){Simulation s(Map(300,300,std::vector<Tile>(90000,Tile::Road)),42);s.setTrafficFlowSettings({true,true,.4,.15});s.setStuckVehicleSettings({false,30});s.setLanes(cross.lanes());s.setProfile(VehicleKind::Truck,{{10,0},{2.5,0},{.5,0},{7,0,2,0},1});need(s.spawnOnLane(lane(s,1,0),VehicleKind::Truck,lane(s,0,destination)),"wide truck turn rejected");
 const auto& e=s.entities()[0];for(const auto& plan:e.junctionPlans)if(!plan.points.empty()){double rotation=0;for(std::size_t i=1;i<plan.points.size();++i){double d=length(plan.points[i]-plan.points[i-1]),angle=std::abs(angleDelta(plan.headings[i-1],plan.headings[i]));rotation+=angle;need(angle<=d/6.5+1e-7,"truck turn radius too tight");
 for(const auto& road:s.lanes())if(road.roadGroup>=0)for(std::size_t j=1;j<road.points.size();++j){auto delta=road.points[j]-road.points[j-1];double n=length(delta);auto dir=delta*(1/n),offset=plan.points[i]-road.points[j-1];double at=offset.x*dir.x+offset.y*dir.y,alignment=std::cos(plan.headings[i])*dir.x+std::sin(plan.headings[i])*dir.y;double mouth=j+1==road.points.size() && length(road.points.back()-Vec2{150,150})<=61?e.length+e.preferredDistance+.5:0;if(at>=0 && at<=n+mouth && alignment<-.25)need(std::abs(offset.x*(-dir.y)+offset.y*dir.x)>=(road.width+e.width)*.5+.1-1e-7,"crossing runs into opposing incoming stream");}}
 need(rotation<(destination==2?.5:2.1),"truck made a looping turn");}
 for(int i=0;i<18000 && !s.entities().empty();++i)s.tick(.01);need(s.completed()==1,"truck did not complete junction trip");}
 std::cout<<"Shortest-distance routing, wide forward truck turns and no upstream lane entry passed\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
