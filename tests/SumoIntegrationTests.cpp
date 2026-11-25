#include "traffic/SumoBackend.hpp"
#include "traffic/Demo.hpp"
#include "traffic/StepBatch.hpp"
#include <iostream>
#include <stdexcept>

using namespace traffic;
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
int signalTest(bool permissive,bool open){
    Map map(220,80,std::vector<Tile>(220*80,Tile::Road));NodeGraph graph;graph.explicitDestinations=true;
    for(Vec2 p:std::vector<Vec2>{{10,40},{100,40},{210,40}})graph.addNode(p);
    graph.nodes[0].spawn=true;graph.nodes[2].destination=true;graph.nodes[1].openRadius=open?12:0;
    graph.connect(0,1,4,8);graph.connect(1,2,4,8);for(auto& road:graph.connections)road.roadType=1;
    auto compiled=compileRoads(graph);Simulation sim(map,7);sim.setLanes(compiled.lanes);sim.setTrafficFlowSettings({true,true,.25,.1});
    TrafficLight light;light.a={95,35};light.b={95,45};light.greenSeconds=1;light.yellowSeconds=1;light.redSeconds=1000;light.offsetSeconds=2;
    light.permissive=permissive;light.allowedA={140,35};light.allowedB={140,45};sim.setTrafficLights({light});
    SumoBackend engine(sim,graph,7);require(sim.spawnOnLane(0),"Could not spawn signal-test vehicle");bool beyondGate=false;
    for(int i=0;i<600;++i){sim.tick(.1);for(auto& e:sim.entities())if(e.center.x>145 && e.center.x<190)beyondGate=true;}
    if(permissive){require(beyondGate,"Allowed-exit gate incorrectly completed the trip");require(sim.completed()==1,"Permitted red-light movement did not finish");}
    else {require(sim.completed()==0,"Vehicle passed a red signal");light.greenSeconds=1000;light.offsetSeconds=0;sim.setTrafficLights({light});for(int i=0;i<600;++i)sim.tick(.1);require(sim.completed()==1,"Live green timing failed to release the queue");}
    require(!engine.collisions() && !engine.teleports(),"Signal test reported collisions/teleports");
    std::cout<<"Live signal / permission test passed; open="<<open<<" permissive="<<permissive<<"\n";return 0;
}
int main(int argc,char** argv){
    try {
        StepBatch batch;batch.request(.1);batch.advanceOne([&](double){batch.cancel();});require(!batch.active(),"Engine-error cancellation underflowed the manual-step counter");
        std::string mode=argc>1?argv[1]:"lanes";
        if(mode=="signals")return signalTest(false,false);
        if(mode=="gates")return signalTest(true,false);
        if(mode=="gates-open")return signalTest(true,true);
        bool chain=mode=="chain",mixed=mode=="mixed",open=mode=="open" || chain || mixed;
        auto map=makeJunctionDemoMap();auto graph=makeJunctionDemoGraph();
        graph.nodes[0].openRadius=open?15:0;
        if(chain){map=Map(500,130,std::vector<Tile>(500*130,Tile::Road));graph={};graph.explicitDestinations=true;
            for(Vec2 p:std::vector<Vec2>{{10,65},{170,65},{330,65},{490,65}})graph.addNode(p);
            graph.nodes[0].spawn=true;graph.nodes[3].destination=true;graph.nodes[1].openRadius=graph.nodes[2].openRadius=24;
            for(int i=0;i<3;++i){graph.connect(i,i+1,5,8);graph.connections.back().roadType=2;}}
        bool debug=argc>2 && std::string(argv[2])=="debug";
        if(argc>2 && !debug)graph=NodeGraph::load(argv[2]);
        if(argc>3)map=Map::fromPng(argv[3],1);
        Simulation sim(map,42);auto compiled=compileRoads(graph);sim.setLanes(compiled.lanes);
        if(argc>4)sim.loadTrafficLights(argv[4]);
        auto profiles=defaultProfiles();if(!mixed)for(auto& p:profiles){p.length.stdDev=p.width.stdDev=0;p.motion.speedStdDev=p.motion.accelerationStdDev=0;}
        for(int k=0;k<vehicleKindCount;++k)sim.setProfile(static_cast<VehicleKind>(k),profiles[k]);
        sim.setTrafficFlowSettings({true,true,.25,.1});
        SumoBackend backend(sim,graph,42);
        int offered=0,requested=0;
        for(int tick=0;tick<(debug?1800:12000);++tick){
            if(tick<1200 && tick%(open?100:20)==0){for(std::size_t l=0;l<compiled.lanes.size();++l)if(compiled.lanes[l].entry && compiled.originNodes[l]>=0){++requested;if(sim.spawnOnLane(static_cast<int>(l),mixed?static_cast<VehicleKind>(offered%vehicleKindCount):VehicleKind::Car))++offered;}}
            sim.tick(.1);
            require(sim.completed()+sim.entities().size()+backend.pending()==static_cast<unsigned>(offered),"A handoff lost or duplicated a vehicle");
            if(tick%600==599)std::cout<<sim.seconds()<<"s completed="<<sim.completed()<<" active="<<sim.entities().size()<<" pending="<<backend.pending()<<" handoffs="<<backend.handoffs()<<std::endl;
            if(tick>=1200 && sim.completed()==static_cast<unsigned>(offered))break;
        }
        std::cout<<"offered="<<offered<<" completed="<<sim.completed()<<" active="<<sim.entities().size()<<" pending="<<backend.pending()<<" collisions="<<backend.collisions()<<" physical overlaps="<<sim.physicalCollisionEvents()<<" teleports="<<backend.teleports()<<" handoffs="<<backend.handoffs()<<"\n";
        for(auto& e:sim.entities())std::cout<<"vehicle "<<e.id<<" at "<<e.center.x<<","<<e.center.y<<" heading="<<e.heading<<" speed="<<e.speed<<" open="<<e.inOpenJunction<<" stage="<<e.networkStage<<" path="<<e.next<<"/"<<e.path.size()<<" "<<e.junctionStatus<<"\n";
        if(!offered || sim.completed()!=static_cast<unsigned>(offered) || !sim.entities().empty() || backend.pending() || backend.collisions() || backend.teleports())throw std::runtime_error("SUMO demand did not drain cleanly");
        require(offered==requested,"Some physically valid test trips were rejected before departure");
        if(open)require(!sim.physicalCollisionEvents(),"Hybrid bodies overlapped");
        if(open && !backend.handoffs())throw std::runtime_error("No free-junction handoffs exercised");
        if(chain && backend.handoffs()!=static_cast<unsigned>(offered*2))throw std::runtime_error("Repeated handoffs lost a trip stage");
        return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
