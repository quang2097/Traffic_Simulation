#include "traffic/NodeGraph.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <iostream>
#include <random>
#include <stdexcept>
using namespace traffic;
int road(const std::vector<DirectedLane>& lanes,int a,int b){for(std::size_t i=0;i<lanes.size();++i)if(lanes[i].roadGroup>=0 && lanes[i].sourceNode==a && lanes[i].destinationNode==b)return static_cast<int>(i);throw std::runtime_error("missing road");}
int main(int argc,char** argv){try{
    bool modern=argc<2 || std::string(argv[1])!="legacy",free=argc<3 || std::string(argv[2])!="guided";unsigned seed=argc>3?std::stoul(argv[3]):7;
    NodeGraph g;g.addNode({90,90});g.nodes[0].openRadius=free?22:0;
    for(auto p:std::vector<Vec2>{{15,90},{165,90},{90,165},{90,15}})g.addNode(p);
    for(int i=1;i<=4;++i){g.connect(i,0,3.5,7);g.connections.back().roadType=2;g.nodes[i].destination=true;}g.explicitDestinations=true;
    auto lanes=g.lanes();Simulation s(Map(180,180,std::vector<Tile>(32400,Tile::Road)),seed);s.setLanes(lanes);s.setCollisionWorkers(1);s.setCarFollowingSettings({modern});s.setTrafficFlowSettings({true,true,.4,.15});s.setStuckVehicleSettings({false,30,false,60});s.setTrafficRecoverySettings({false,1,25});
    auto urban=s.urbanTrafficSettings();urban.enabled=true;s.setUrbanTrafficSettings(urban);
    auto profiles=vietnameseCityProfiles();std::mt19937 demand(seed);std::discrete_distribution<int> kinds{3,80,14,1,2};
    int offered=0,admitted=0,atDemandEnd=0;std::cout<<std::unitbuf;
    for(int step=0;step<12000;++step) {
        if(step<3000 && step%200==0)for(int arm=1;arm<=4;++arm){
            int destination=1+std::uniform_int_distribution<int>(0,2)(demand);if(destination>=arm)++destination;
            auto kind=static_cast<VehicleKind>(kinds(demand));auto p=profiles[static_cast<int>(kind)];
            auto fixed=[&](Gaussian value,double low,double high){return std::clamp(std::normal_distribution<double>(value.mean,value.stdDev)(demand),low,high);};
            p.length={fixed(p.length,.2,30),0};p.width={fixed(p.width,.15,5),0};p.distance={fixed(p.distance,0,20),0};
            p.motion={fixed({p.motion.speedMean,p.motion.speedStdDev},.05,50),0,fixed({p.motion.accelerationMean,p.motion.accelerationStdDev},.05,100),0};
            s.setProfile(kind,p);++offered;admitted+=s.spawnOnLane(road(lanes,arm,0),kind,road(lanes,0,destination));
        }
        s.tick(.01);
        if(step==2999)atDemandEnd=s.completed();
        for(const auto& e:s.entities())if(!s.map().fits(e.footprint()))throw std::runtime_error("benchmark left mask");
        if(s.physicalCollisionEvents())throw std::runtime_error("benchmark physical collision");
        if(step%2000==1999)std::cout<<"PROGRESS mode="<<(modern?"new":"legacy")<<" map="<<(free?"free":"guided")<<" t="<<s.seconds()<<" arrived="<<s.completed()<<" active="<<s.entities().size()<<'\n';
    }
    auto stats=s.stopStatistics();
    if(argc>4) for(const auto& e:s.entities()) {
        std::cout<<"STUCK id="<<e.id<<" kind="<<vehicleName(e.kind)<<" xy="<<e.center.x<<','<<e.center.y<<" heading="<<e.heading<<" phase="<<e.networkStage<<" internal="<<e.internalJunction<<" lane="<<e.networkRoute[e.networkStage]<<" stop="<<e.currentStopSeconds<<" speed="<<e.speed<<" next="<<e.next<<'/'<<e.path.size()<<" status="<<e.junctionStatus<<'\n';
        if(e.next<e.path.size()) {
            auto to=e.footprint();auto delta=e.path[e.next]-e.center;double t=std::min(1.,.02/std::max(1e-9,length(delta)));to.center=to.center+delta*t;to.heading=e.heading+(e.pathHeadings[e.next]-e.heading)*t;
            for(const auto& other:s.entities())if(other.id!=e.id && !geometry::motionClearance(e.footprint(),to,other.footprint(),0))std::cout<<"BLOCK id="<<e.id<<" by="<<other.id<<'\n';
        }
    }
    std::cout<<"RESULT mode="<<(modern?"new":"legacy")<<" map="<<(free?"free":"guided")<<" seed="<<seed<<" offered="<<offered<<" inserted="<<admitted<<" completed30="<<atDemandEnd<<" completed120="<<s.completed()<<" active="<<s.entities().size()<<" mean_stopped="<<stats.meanSeconds<<" max_stopped="<<stats.maxSeconds<<" compressed="<<s.compressedGapEvents()<<" emergency="<<s.emergencyBrakeEvents()<<" contact_limits="<<s.contactLimitEvents()<<" collisions="<<s.physicalCollisionEvents()<<" deleted="<<s.removedStuckVehicles()<<'\n';
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
