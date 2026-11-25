#include "traffic/NodeGraph.hpp"
#include "traffic/SumoBackend.hpp"
#include <filesystem>
#include <fstream>
#include <cmath>
#include <iostream>
#include <regex>
#include <set>
#include <stdexcept>

using namespace traffic;
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
NodeGraph sevenLanes(){
    NodeGraph g;g.explicitDestinations=true;
    for(Vec2 p:std::vector<Vec2>{{20,250},{250,250},{250,20},{480,250},{250,480}})g.addNode(p);
    for(int n:{0,2,3,4})g.nodes[n].destination=true;
    g.nodes[0].spawn=true;
    for(auto pair:std::vector<std::pair<int,int>>{{0,1},{2,1},{1,3},{1,4}})g.connect(pair.first,pair.second,3.5,8);
    g.connections[0].roadType=1;g.connections[0].laneCount=7;
    g.connections[1].roadType=2;g.connections[1].laneCount=3;
    g.connections[2].roadType=1;g.connections[2].laneCount=7;
    g.connections[3].roadType=2;g.connections[3].laneCount=2;g.connections[3].reverseLaneCount=4;
    g.setLaneTurnRule(0,{0,0,{{2,{0}}}});
    for(int lane=1;lane<=5;++lane)g.setLaneTurnRule(0,{0,lane,{{3,{0,1,2,3,4,5,6}}}});
    g.setLaneTurnRule(0,{0,6,{{4,{1}}}});return g;
}
int physical(const std::vector<DirectedLane>& lanes,int road,int direction,int index){for(std::size_t i=0;i<lanes.size();++i)if(lanes[i].roadGroup==road && lanes[i].travelDirection==direction && lanes[i].roadLane==index)return static_cast<int>(i);throw std::runtime_error("Missing physical lane");}
std::set<int> targets(const std::vector<DirectedLane>& lanes,int source){std::set<int> result;for(int link:lanes[source].successors){if(lanes[link].roadGroup>=0)result.insert(link);else for(int out:lanes[link].successors)result.insert(out);}return result;}
void modelTests(){
    require(parseNumberList("1, 2-6, 7, 1",1,7).size()==7,"Range parsing failed");require(parseNumberList("all",1,7).size()==7,"All lanes failed");
    bool rejected=false;try{parseNumberList("0, 8",1,7);}catch(...){rejected=true;}require(rejected,"Invalid lane numbers were accepted");
    auto graph=sevenLanes();auto lanes=compileRoads(graph).lanes;
    for(int i=0;i<7;++i){auto out=targets(lanes,physical(lanes,0,0,i));std::set<int> expected;
        if(i==0)expected.insert(physical(lanes,1,1,0));else if(i==6)expected.insert(physical(lanes,3,0,1));else for(int n=0;n<7;++n)expected.insert(physical(lanes,2,0,n));
        require(out==expected,"A lane leads to a forbidden lane or wrong two-way direction");}
    auto from=lanes[physical(lanes,0,0,0)].points.front(),right=lanes[physical(lanes,0,0,6)].points.front();require(from.y<right.y,"One-way numbering is not left to right");
    auto reverseLeft=lanes[physical(lanes,1,1,0)].points.front(),reverseRight=lanes[physical(lanes,1,1,2)].points.front();require(reverseLeft.x<reverseRight.x,"Reverse carriageway numbering is wrong");
    rejected=false;try{graph.setLaneTurnRule(0,{0,0,{{2,{3}}}});}catch(...){rejected=true;}require(rejected,"An opposite-direction lane count leaked into exit validation");
    rejected=false;try{graph.setLaneTurnRule(0,{0,6,{{4,{2}}}});}catch(...){rejected=true;}require(rejected,"The four reverse lanes were offered for a two-lane outgoing direction");
    graph.save("lane-connection-roundtrip.nodes");auto loaded=NodeGraph::load("lane-connection-roundtrip.nodes");require(loaded.connections[0].laneTurns.size()==7,"Lane rules did not persist");
    require(targets(compileRoads(loaded).lanes,physical(lanes,0,0,0))==targets(lanes,physical(lanes,0,0,0)),"Lane rules changed on reload");
    int station=loaded.splitRoad(1,{250,120});loaded.nodes[station].roadStation=true;require(loaded.connections[0].laneTurns[0].exits[0].node==station,"Splitting an outgoing road lost its lane connection");loaded.validateLaneTurns();
    graph.setLaneTurnRule(0,{0,6,{}});require(targets(compileRoads(graph).lanes,physical(lanes,0,0,6)).empty(),"An explicit closed lane reverted to automatic exits");
    auto legacy=NodeGraph::load(std::string(TRAFFIC_TEST_ROOT)+"/tests/fixtures/map-demo/Map_demo.nodes");require(legacy.nodes.size()==5 && legacy.connections[0].laneTurns.empty(),"Version 8 node files no longer load");
    auto broad=sevenLanes();broad.connections[0].laneCount=32;require(compileRoads(broad).lanes.size()>32,"Roads still have a four-lane limit");
    std::cout<<"Seven-lane matrix, two-way numbering, bounds, splitting and persistence passed\n";
}
void nativeRoute(){
    auto g=sevenLanes();auto lanes=compileRoads(g).lanes;Map map(500,500,std::vector<Tile>(250000,Tile::Road));Simulation sim(map,42);sim.setLanes(lanes);sim.setTrafficFlowSettings({true,true,.25,.1});
    auto p=defaultProfiles()[2];p.length.stdDev=p.width.stdDev=0;p.motion.speedStdDev=p.motion.accelerationStdDev=0;sim.setProfile(VehicleKind::Car,p);
    int source=physical(lanes,0,0,0),allowed=physical(lanes,1,1,0),forbidden=physical(lanes,2,0,0);
    require(!sim.spawnOnLane(source,VehicleKind::Car,forbidden),"Native routing used a forbidden lane exit");
    require(sim.spawnOnLane(source,VehicleKind::Car,allowed),"Native routing rejected the permitted leftmost turn");
    std::cout<<"Native lane-specific route selection passed\n";
}
NodeGraph signalRoad(){NodeGraph g;g.explicitDestinations=true;for(Vec2 p:std::vector<Vec2>{{20,100},{140,100},{260,100}})g.addNode(p);g.nodes[1].roadStation=true;for(int i:{0,2}){g.nodes[i].destination=g.nodes[i].spawn=true;g.nodes[i].excludeOwnDestination=true;}
    g.connect(0,1,3.5,8);g.connect(1,2,3.5,8);for(auto& road:g.connections)road.roadType=2;return g;}
void signalTest(bool sumo){
    auto g=signalRoad();auto lanes=compileRoads(g).lanes;Map map(280,200,std::vector<Tile>(56000,Tile::Road));Simulation sim(map,42);sim.setLanes(lanes);sim.setTrafficFlowSettings({true,true,.25,.1});sim.setStuckVehicleSettings({false,30,false,60});
    auto p=defaultProfiles()[2];p.length.stdDev=p.width.stdDev=0;p.motion.speedStdDev=p.motion.accelerationStdDev=0;sim.setProfile(VehicleKind::Car,p);
    auto geometry=roadSignalAtNode(g,0,1);require(geometry.outDirection.x>.99,"Road signal is not aligned with travel");require(std::abs((geometry.b-geometry.a).x)<1e-8,"Road signal is not perpendicular");require(length(geometry.b-geometry.a)>7,"Two-way signal does not span both travel directions");
    TrafficLight light;light.a={140,90};light.b={140,110};light.outDirection={1,0};light.anchorNode=1;light.greenSeconds=1;light.yellowSeconds=1;light.redSeconds=1000;light.offsetSeconds=2;sim.setTrafficLights({light});
    require(light.controlsDirection({1,0}) && !light.controlsDirection({-1,0}),"Direction filter does not separate IN/OUT");
    require(light.controlsMovement({130,100},{131,100}) && !light.controlsMovement({150,100},{149,100}),"IN movement was treated as a controlled approach");
    require(!light.controlsMovement({141,100},{142,101}),"A departing vehicle can be stopped by its rear sweeping the line");
    sim.saveTrafficLights("directional-signals.signals");sim.loadTrafficLights("directional-signals.signals");require(sim.trafficLights()[0].anchorNode==1 && sim.trafficLights()[0].outDirection.x>.99,"Signal direction/anchor did not persist");
    #ifdef _WIN32
    std::unique_ptr<SumoBackend> backend;
    if(sumo)backend=std::make_unique<SumoBackend>(sim,g,42);
#endif
    require(sim.spawnOnLane(physical(lanes,0,0,0),VehicleKind::Car,physical(lanes,1,0,0)),"OUT vehicle could not spawn");
    require(sim.spawnOnLane(physical(lanes,1,1,0),VehicleKind::Car,physical(lanes,0,1,0)),"IN vehicle could not spawn");
    for(int i=0;i<700;++i)sim.tick(.1);
    require(sim.completed()==1,"IN traffic did not pass red, or OUT traffic ignored red");
    require(sim.entities().size()==1 && sim.entities()[0].center.x<140,"Controlled traffic crossed the red stop line");
    light.permissive=true;light.allowedA={200,90};light.allowedB={200,110};sim.setTrafficLights({light});
    for(int i=0;i<700;++i)sim.tick(.1);require(sim.completed()==2,"Directional allowed-exit permission did not release OUT traffic");
    #ifdef _WIN32
    if(backend)require(!backend->collisions() && !backend->teleports(),"Signal test used collision or teleport recovery");
    #endif
    std::cout<<(sumo?"SUMO":"Native")<<" directional red-light and road-station signal passed\n";
}
void nativeDirectionChanges(){
    for(bool freeJunction:{false,true})for(bool predictive:{false,true})for(int rotation=0;rotation<4;++rotation){
        // The legacy free-mask router may detour around the finite stop line
        // on this all-road fixture. Free-junction checks use the committed
        // predictive controller selected by the application's Native mode.
        if(freeJunction && !predictive)continue;
        auto rotate=[&](Vec2 p){p=p-Vec2{140,100};for(int i=0;i<rotation;++i)p={-p.y,p.x};return p+Vec2{160,160};};
        auto g=signalRoad();if(freeJunction){g.nodes[1].roadStation=false;g.nodes[1].openRadius=30;}
        for(auto& n:g.nodes)n.position=rotate(n.position);
        auto lanes=compileRoads(g).lanes;Simulation sim(Map(320,320,std::vector<Tile>(102400,Tile::Road)),42);
        sim.setLanes(lanes);sim.setTrafficFlowSettings({predictive,false,.25,.1});sim.setStuckVehicleSettings({false,30,false,60});
        auto p=defaultProfiles()[2];p.length.stdDev=p.width.stdDev=0;p.motion.speedStdDev=p.motion.accelerationStdDev=0;sim.setProfile(VehicleKind::Car,p);
        auto line=roadSignalAtNode(g,0,1);TrafficLight light;light.a=line.a;light.b=line.b;light.outDirection=line.outDirection;light.anchorNode=1;
        light.greenSeconds=1;light.yellowSeconds=1;light.redSeconds=1000;light.offsetSeconds=2;sim.setTrafficLights({light});
        require(sim.spawnOnLane(physical(lanes,0,0,0),VehicleKind::Car,physical(lanes,1,0,0)),"Live flip OUT spawn failed");
        require(sim.spawnOnLane(physical(lanes,1,1,0),VehicleKind::Car,physical(lanes,0,1,0)),"Live flip IN spawn failed");
        for(int i=0;i<650;++i)sim.tick(.1);
        if(sim.completed()!=1)throw std::runtime_error("Native IN blocked at road signal: free="+std::to_string(freeJunction)+" predictive="+std::to_string(predictive)+" rotation="+std::to_string(rotation)+" completed="+std::to_string(sim.completed()));
        light.outDirection=light.controlledDirection()*-1;sim.setTrafficLights({light});
        for(int i=0;i<650;++i)sim.tick(.1);
        if(sim.completed()!=2)throw std::runtime_error("Native IN stayed stopped after live flip: predictive="+std::to_string(predictive)+" rotation="+std::to_string(rotation));
    }
    std::cout<<"Native road-mounted IN/OUT and live flips passed in four orientations, with both routing modes\n";
}
void nativeCustomSignals(){
    const auto root=std::string(TRAFFIC_TEST_ROOT)+"/tests/fixtures/directional-signals/";
    auto graph=NodeGraph::load(root+"custom.nodes");auto lanes=compileRoads(graph).lanes;
    for(int signals:{0,1,2})for(auto kind:{VehicleKind::Motorbike,VehicleKind::Car}) {
        Simulation sim(Map::fromPng(root+"custom-map.png",1),42);sim.setLanes(lanes);
        sim.setTrafficFlowSettings({true,false,.4,.15});sim.setStuckVehicleSettings({false,30,false,60});
        auto profile=defaultProfiles()[static_cast<int>(kind)];profile.length.stdDev=profile.width.stdDev=0;
        profile.motion.speedStdDev=profile.motion.accelerationStdDev=0;sim.setProfile(kind,profile);
        sim.setTrafficLights({}); // PNG blue markings otherwise import default lights.
        if(signals){
            sim.loadTrafficLights(root+"custom-map.png.signals");auto north=sim.trafficLights().at(2);
            require(north.controlledDirection().y>.99,"Custom north signal no longer controls southbound OUT");
            north.greenSeconds=north.yellowSeconds=1;north.redSeconds=1000;north.offsetSeconds=2;
            if(signals==1)sim.setTrafficLights({north});
        }
        // West lane 1 turns north through the IN side of the signal station.
        // A keep-right change on the preceding road used to strand motorbikes
        // at N7: the new lane had no continuation into the committed next lane.
        int source=physical(lanes,0,0,0),target=physical(lanes,1,1,0);
        require(sim.spawnOnLane(source,kind,target),"Custom-map IN trip failed to spawn");
        for(int i=0;i<2000 && !sim.entities().empty();++i){sim.tick(.05);
            if(!sim.entities().empty() && sim.entities().front().currentStopSeconds>2 && sim.entities().front().next>=sim.entities().front().path.size())break;
        }
        require(sim.completed()==1,signals?"Custom-map IN traffic stalled at a red signal station":"Custom-map lane change lost its physical continuation with signals removed");
        require(sim.physicalCollisionEvents()==0,"Custom-map IN regression used a collision to progress");
    }
    std::cout<<"Custom map IN traffic clears the signal station with lights removed and with OUT held red\n";
}
#ifdef _WIN32
void sumoRules(bool closed=false){
    auto g=sevenLanes();if(closed)g.setLaneTurnRule(0,{0,6,{}});auto lanes=compileRoads(g).lanes;Map map(500,500,std::vector<Tile>(250000,Tile::Road));Simulation sim(map,42);sim.setLanes(lanes);
    auto output=std::filesystem::current_path()/(closed?"sumo-closed-lane":"sumo-lane-rules");SumoBackend backend(sim,g,42,"",output.string());
    std::ifstream net(output/"network.net.xml");std::string line;int west=0;
    auto attribute=[](const std::string& s,const std::string& key){std::regex r(key+"=\"([^\"]*)\"");std::smatch m;return std::regex_search(s,m,r)?m[1].str():std::string();};
    while(std::getline(net,line))if(line.find("<connection ")!=std::string::npos && attribute(line,"from")=="r0f"){
        int source=6-std::stoi(attribute(line,"fromLane")),target=std::stoi(attribute(line,"toLane"));auto to=attribute(line,"to");++west;
        require(!(closed && source==6),"SUMO reopened an explicitly closed lane");
        require(source==0?(to=="r1b" && target==2):source==6?(to=="r3f" && target==0):(to=="r2f" && target>=0 && target<7),"SUMO added an unapproved lane connection or reversed lane numbering");
    }
    require(west==(closed?36:37),"SUMO did not preserve the complete seven-lane matrix");
    std::cout<<"Actual SUMO network has exactly the 37 requested lane connections\n";
}
#endif
}
int main(int argc,char** argv){try{std::string mode=argc>1?argv[1]:"model";if(mode=="model"){modelTests();nativeRoute();}else if(mode=="native-signals"){signalTest(false);nativeDirectionChanges();nativeCustomSignals();}else if(mode=="custom-signals")nativeCustomSignals();
#ifdef _WIN32
else if(mode=="sumo-signals")signalTest(true);else if(mode=="sumo-rules")sumoRules();else if(mode=="sumo-closed")sumoRules(true);
#endif
if(mode=="write-example"){
    auto g=sevenLanes();g.nodes[0].rate=.3;Map map(500,500,std::vector<Tile>(250000,Tile::Wall));
    for(int y=0;y<500;++y)for(int x=0;x<500;++x){auto p=map.center({x,y});auto d=p-Vec2{250,250};if((x>14 && x<486 && y>233 && y<267) || (y>14 && y<486 && x>228 && x<267) || length(d)<80)map.tiles[map.index({x,y})]=Tile::Road;}
    std::string prefix=argc>2?argv[2]:"seven-lane-routing";g.save(prefix+".nodes");map.savePng(prefix+".png");
}
return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
