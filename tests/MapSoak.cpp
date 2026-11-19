#include "traffic/Demo.hpp"
#include "traffic/Geometry.hpp"
#include "traffic/SpawnClock.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <algorithm>
#include <unordered_set>
using namespace traffic;
int main(int argc,char** argv){try{
 if(argc<6)throw std::runtime_error("map_soak MAP NODES M_PER_PIXEL SEED OUTPUT_PREFIX [SECONDS] [detailed|drain]");
 unsigned seed=std::stoul(argv[4]);double duration=argc>6?std::stod(argv[6]):900;bool detailed=argc>7 && std::string(argv[7])=="detailed",drain=argc>7 && std::string(argv[7])=="drain";int durationSteps=static_cast<int>(duration*120),runSteps=durationSteps+(drain?240*120:0);std::string prefix=argv[5];std::ofstream log(prefix+".log"),csv(prefix+".csv");log<<std::unitbuf;std::cout<<std::unitbuf;
 auto map=Map::fromPng(argv[1],std::stod(argv[3]));auto graph=NodeGraph::load(argv[2]);auto roads=compileRoads(graph);
 log<<"seed="<<seed<<" scale="<<map.metersPerPixel<<" pixels="<<map.width<<"x"<<map.height<<" lanes="<<roads.lanes.size()<<"\n";
 try{validateRoadMask(roads.lanes,map);log<<"network mask validation passed\n";}catch(const std::exception& e){log<<"NETWORK WARNING: "<<e.what()<<'\n';}
 Simulation s(map,seed);s.setLanes(roads.lanes);s.setTrafficFlowSettings({true,true,.4,.15});s.setCollisionWorkers(1);s.setStuckVehicleSettings({false,30});s.setTrafficRecoverySettings({true,.6});
 auto urban=s.urbanTrafficSettings();urban.enabled=true;s.setUrbanTrafficSettings(urban);auto routing=s.routingSettings();routing.localInterval=.35;routing.movingRefreshInterval=1.2;s.setRoutingSettings(routing);auto tug=s.tugSettings();tug.enabled=true;s.setTugSettings(tug);auto turns=s.turnNegotiationSettings();turns.enabled=true;s.setTurnNegotiationSettings(turns);
 auto profiles=vietnameseCityProfiles();for(int i=0;i<vehicleKindCount;++i)s.setProfile(static_cast<VehicleKind>(i),profiles[i]);s.loadTrafficLights(std::string(argv[2])+".signals");log<<"lights="<<s.trafficLights().size()<<" deletion=off cap=100 dt=1/120\n";
 std::vector<std::vector<int>> sources(graph.nodes.size());for(std::size_t i=0;i<roads.originNodes.size();++i)if(roads.originNodes[i]>=0)sources[roads.originNodes[i]].push_back(static_cast<int>(i));
 std::mt19937 rng(seed);std::vector<double> clocks(graph.nodes.size());std::vector<int> offered(graph.nodes.size()),inserted(graph.nodes.size());std::unordered_set<std::uint64_t> stalled,freeStalled,outside,colliders;double minGap=1e9,maxFreeStop=0;int admitted=0,requests=0;auto start=std::chrono::steady_clock::now();
 csv<<"seconds,offered,inserted,completed,active,junction_entries,mean_stop,max_stop,free_max_stop,distinct_stalls,distinct_free_stalls,mask_violators,clearance_pairs,deleted\n";
 for(int step=0;step<runSteps;++step){if(step<durationSteps)for(std::size_t n=0;n<graph.nodes.size();++n){auto& node=graph.nodes[n];if(!node.spawn || node.rate<=0 || !consumeSpawnClock(clocks[n],1./120,node.rate))continue;++offered[n];++requests;if(!sources[n].empty() && s.entities().size()<100){int lane=sources[n][std::uniform_int_distribution<std::size_t>(0,sources[n].size()-1)(rng)];if(s.spawnRandomOnLane(lane)){++inserted[n];++admitted;}}}
 s.tick(1./120);
 if(step%120==119){for(const auto& e:s.entities()){if(!s.map().fits(e.footprint()))outside.insert(e.id);if(e.inOpenJunction)maxFreeStop=std::max(maxFreeStop,e.currentStopSeconds);if(detailed && step%1200==1199 && e.currentStopSeconds>5)log<<"SNAP t="<<s.seconds()<<" id="<<e.id<<" changing="<<e.changingLanes<<" coop="<<e.cooperativeRecovery<<" lane_changes="<<e.laneChanges<<" status="<<e.junctionStatus<<" body="<<e.length<<","<<e.width<<" pref="<<e.preferredDistance<<" next="<<e.next<<"/"<<e.path.size()<<" remaining_goal="<<length(e.goal-e.center)<<"\n";if(e.currentStopSeconds>=(detailed?5:60) && stalled.insert(e.id).second)log<<"STALL t="<<s.seconds()<<" id="<<e.id<<" kind="<<vehicleName(e.kind)<<" xy="<<e.center.x<<','<<e.center.y<<" free="<<e.inOpenJunction<<" lane="<<(e.networkRoute.empty()?-1:e.networkRoute[e.networkStage])<<" reason="<<stopReasonName(s.stopReason(e))<<" reserved="<<e.reservesSublaneGap()<<" heading="<<e.heading<<" junction_status="<<e.junctionStatus<<" lateral="<<e.lateralOffset<<" exit="<<(e.networkStage+2<e.networkRoute.size()?e.networkRoute[e.networkStage+2]:-1)<<" goal="<<e.goal.x<<','<<e.goal.y<<"\n";if(e.inOpenJunction && e.currentStopSeconds>=30)freeStalled.insert(e.id);
 for(const auto& o:s.entities())if(e.id<o.id){double gap=clearance(e.footprint(),o.footprint());minGap=std::min(minGap,gap);if(gap+1e-5<std::min(e.preferredDistance,o.preferredDistance) || (std::min(e.preferredDistance,o.preferredDistance)<=1e-7 && geometry::violatesClearance(corners(e.footprint()),corners(o.footprint()),0)))colliders.insert((e.id<<32)^o.id);}}
 }
 if(step%7200==7199 || step+1==durationSteps || step+1==runSteps){auto stats=s.stopStatistics(true);double freeMax=0;for(const auto& e:s.entities())if(e.inOpenJunction)freeMax=std::max(freeMax,e.currentStopSeconds);csv<<s.seconds()<<','<<requests<<','<<admitted<<','<<s.completed()<<','<<s.entities().size()<<','<<s.junctionAdmissions()<<','<<stats.meanSeconds<<','<<stats.maxSeconds<<','<<freeMax<<','<<stalled.size()<<','<<freeStalled.size()<<','<<outside.size()<<','<<colliders.size()<<','<<s.removedStuckVehicles()<<std::endl;log<<"PROGRESS t="<<s.seconds()<<" inserted="<<admitted<<" completed="<<s.completed()<<" active="<<s.entities().size()<<" max_free_stop="<<freeMax<<" wall="<<std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<<"\n";}
 if(step+1==durationSteps) log<<"AT_DURATION seconds="<<s.seconds()<<" completed="<<s.completed()<<" admitted="<<admitted<<" max_free_stop="<<maxFreeStop<<"\n";
 if(drain && step+1>=durationSteps && s.entities().empty())break;
 }
 for(std::size_t n=0;n<graph.nodes.size();++n)if(graph.nodes[n].spawn)log<<"SOURCE "<<n<<" rate="<<graph.nodes[n].rate<<" offered="<<offered[n]<<" inserted="<<inserted[n]<<"\n";
 for(const auto& e:s.entities())log<<"REMAIN id="<<e.id<<" xy="<<e.center.x<<','<<e.center.y<<" stop="<<e.currentStopSeconds<<" free="<<e.inOpenJunction<<" reason="<<stopReasonName(s.stopReason(e))<<"\n";
 if(detailed || (drain && !s.entities().empty())) for(const auto& e:s.entities()) {
 log<<std::setprecision(17)<<"POSE id="<<e.id<<" kind="<<static_cast<int>(e.kind)<<" xy="<<e.center.x<<","<<e.center.y<<" h="<<e.heading<<" lateral="<<e.lateralOffset<<" size="<<e.length<<","<<e.width<<" pref="<<e.preferredDistance<<" next="<<e.next<<"/"<<e.path.size()<<" progress="<<e.segmentProgress<<" changing="<<e.changingLanes<<" coop="<<e.cooperativeRecovery<<" signal="<<e.signalLight<<" desired="<<e.desiredSpeed<<" accel="<<e.acceleration<<" stage="<<e.networkStage<<" goal="<<e.goal.x<<","<<e.goal.y<<" status="<<e.junctionStatus<<"\n";
 log<<"NETWORK id="<<e.id;for(auto lane:e.networkRoute)log<<" "<<lane;log<<"\n";
 for(std::size_t i=e.next?e.next-1:0;i<e.path.size();++i)log<<"PATH id="<<e.id<<" i="<<i<<" p="<<e.path[i].x<<","<<e.path[i].y<<" h="<<e.pathHeadings[i]<<"\n";
 }
 log<<"DONE seconds="<<s.seconds()<<" completed="<<s.completed()<<" admitted="<<admitted<<" min_clearance="<<minGap<<" max_free_stop="<<maxFreeStop<<"\n";std::cout<<seed<<" completed="<<s.completed()<<" inserted="<<admitted<<" active="<<s.entities().size()<<" violations="<<colliders.size()<<"\n";
 if(drain && !s.entities().empty())return 3;
 return outside.empty()&&colliders.empty()?0:2;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
