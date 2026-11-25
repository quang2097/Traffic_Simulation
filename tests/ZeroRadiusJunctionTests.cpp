#include "traffic/NodeGraph.hpp"
#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <unordered_map>
using namespace traffic;
void need(bool value,const char* why){if(!value)throw std::runtime_error(why);}
int road(const std::vector<DirectedLane>& lanes,int a,int b){for(std::size_t i=0;i<lanes.size();++i)if(lanes[i].roadGroup>=0 && lanes[i].sourceNode==a && lanes[i].destinationNode==b)return static_cast<int>(i);throw std::runtime_error("missing road");}
int main(){try{std::cout<<std::unitbuf;
    for(int arms:{3,4}) {
        NodeGraph g;g.addNode({80,80});
        for(auto p:std::vector<Vec2>{{15,80},{145,80},{80,145},{80,15}})g.addNode(p);
        for(int i=1;i<=arms;++i){g.connect(i,0,4,6);g.connections.back().roadType=2;g.nodes[i].destination=true;}g.explicitDestinations=true;
        auto lanes=g.lanes();need(std::any_of(lanes.begin(),lanes.end(),[](const DirectedLane& l){return l.internalJunction==0;}),"radius zero requires internal connectors");
        need(std::none_of(lanes.begin(),lanes.end(),[](const DirectedLane& l){return l.openJunction>=0;}),"radius-zero node became free-space junction");
        Simulation s(Map(160,160,std::vector<Tile>(25600,Tile::Road)),42);s.setLanes(lanes);s.setCollisionWorkers(1);
        s.setTrafficFlowSettings({true,true,.4,.15});s.setStuckVehicleSettings({false,30,false,60});
        s.setProfile(VehicleKind::Motorbike,{{2,0},{.7,0},{.3,0},{5,0,2,0},1});
        s.setProfile(VehicleKind::Car,{{4.4,0},{1.8,0},{.4,0},{5,0,2,0},1});
        s.setProfile(VehicleKind::Bus,{{9,0},{2.4,0},{.4,0},{4,0,1.5,0},1});
        int admitted=0;bool visited=false;
        for(int step=0;step<24000;++step) {
            if(step<2000 && step%250==0) for(int arm=1;arm<=arms;++arm) {
                int destination=1+(arm+step/250) % arms;
                if(destination==arm)destination=1+destination%arms;
                auto kind=(step/250)%3==0?VehicleKind::Bus:(step/250)%3==1?VehicleKind::Car:VehicleKind::Motorbike;
                admitted+=s.spawnOnLane(road(lanes,arm,0),kind,road(lanes,0,destination));
            }
            std::unordered_map<std::uint64_t,Vec2> before;for(const auto& e:s.entities())before[e.id]=e.center;
            s.tick(.01);
            for(const auto& e:s.entities()) {
                visited|=e.internalJunction==0;need(!e.inOpenJunction,"guided vehicle left its connector for free navigation");
                need(length(e.center-before.at(e.id))<=.061,"internal connector teleported vehicle");
                if(step%12==0) need(s.map().fits(e.footprint()),"connector left road mask");
            }
            need(s.physicalCollisionEvents()==0,"normal guided crossing caused a physical collision");
            if(step%3000==2999)std::cout<<arms<<" arms t="<<s.seconds()<<" admitted="<<admitted<<" arrived="<<s.completed()<<" active="<<s.entities().size()<<'\n';
            if(step>=2000 && s.entities().empty())break;
        }
        if(s.completed()!=static_cast<unsigned>(admitted))for(const auto& e:s.entities())std::cerr<<e.id<<" lane="<<e.networkRoute[e.networkStage]<<" internal="<<e.internalJunction<<" stop="<<e.currentStopSeconds<<" reason="<<e.junctionStatus<<'\n';
        need(visited && admitted>=12 && s.completed()==static_cast<unsigned>(admitted),"zero-radius mixed junction failed to drain");
        need(s.gridlockEscapes()==0 && s.removedStuckVehicles()==0,"normal radius-zero flow depended on escape or deletion");
        std::cout<<arms<<"-way radius-zero junction drained "<<admitted<<" mixed vehicles without overlap or deletion\n";
    }
    for(bool free:{false,true}) {
        NodeGraph g;g.addNode({10,30});g.addNode({90,30});g.addNode({50,30});g.nodes[2].openRadius=free?15:0;
        g.connect(0,2,6,5);g.connect(2,1,6,5);for(auto& connection:g.connections)connection.roadType=1;
        g.explicitDestinations=true;g.nodes[1].destination=true;auto lanes=g.lanes();
        Simulation s(Map(100,60,std::vector<Tile>(6000,Tile::Road)),42);s.setLanes(lanes);s.setTrafficFlowSettings({true,false,.4,.15});
        s.setStuckVehicleSettings({false,30,false,60});s.setGridlockEscapeSettings({true,1,.5,1.5});
        s.setProfile(VehicleKind::Motorbike,{{2,0},{.7,0},{.3,0},{5,0,2,0},1});
        need(s.spawnOnLane(road(lanes,0,2),VehicleKind::Motorbike,road(lanes,2,1)),"blocked connector trip insertion");
        for(int step=0;step<3000 && !s.entities()[0].onJunction();++step)s.tick(.01);
        need(s.entities()[0].onJunction(),"fixture never entered junction");
        auto& actors=const_cast<std::vector<Entity>&>(s.entities());actors[0].speed=0;actors[0].currentStopSeconds=0;
        Entity blocker=actors[0];blocker.id=9999;blocker.center=blocker.center+Vec2{3,0};blocker.networkRoute.clear();blocker.laneRoute.clear();
        blocker.internalJunction=-1;blocker.inOpenJunction=false;blocker.activeJunction=-1;blocker.desiredSpeed=blocker.speed=0;
        blocker.path={blocker.center,blocker.center+Vec2{1,0}};blocker.pathHeadings={0,0};blocker.heading=0;blocker.next=1;blocker.segmentProgress=0;
        actors.push_back(blocker);
        for(int step=0;step<1000;++step){s.tick(.01);for(const auto& e:s.entities())need(s.map().fits(e.footprint()),"blocked-junction recovery left mask");}
        need(s.completed()==0 && s.entities().size()==2,"a solid blocker was bypassed by overlapping bodies");
        need(s.removedStuckVehicles()==0 && s.physicalCollisionEvents()==0 && s.overlapEscapes()==0,"junction blocker recovery used collision or deletion");
        need(free?s.freeGridlockEscapes()>0:s.internalGridlockEscapes()>0,"actual junction escape used wrong category");
        auto& remaining=const_cast<std::vector<Entity>&>(s.entities());remaining.erase(std::remove_if(remaining.begin(),remaining.end(),[](const Entity& e){return e.id==9999;}),remaining.end());
        for(int step=0;step<6000 && s.completed()==0;++step)s.tick(.01);
        need(s.completed()==1 && s.physicalCollisionEvents()==0,"junction did not resume after a genuine obstacle cleared");
        std::cout<<(free?"Free":"Guided")<<" junction: solid obstacle respected and trip resumed when clear; no overlaps\n";
    }
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
