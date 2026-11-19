#include "traffic/NodeGraph.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace traffic;
void need(bool b,const char* msg){if(!b)throw std::runtime_error(msg);}
int main(){try{NodeGraph g;for(auto p:std::vector<Vec2>{{15,160},{90,100},{150,100},{190,140},{120,100}})g.addNode(p);g.nodes[4].openRadius=40;g.nodes[3].destination=true;g.explicitDestinations=true;g.connect(0,1,6,8);g.connections[0].roadType=1;g.connections[0].shape={{15,160},{60,140},{80,110},{90,100}};g.connections[0].shapeWidths={6,7,8,8};g.connect(2,3,8,8);g.connections[1].roadType=1;
Simulation s(Map(210,190,std::vector<Tile>(39900,Tile::Road)),7);s.setLanes(g.lanes());s.setTrafficFlowSettings({true,true,.4,.15});s.setStuckVehicleSettings({false,30});s.setProfile(VehicleKind::Motorbike,{{2,0},{.7,0},{.3,0},{6,0,2,0},1});
for(int i=0;i<3;++i){need(s.spawnOnLane(0,VehicleKind::Motorbike),"parallel curved-approach spawn failed");const auto& e=s.entities().back();need(e.junctionPlans.size()>1 && !e.junctionPlans[1].points.empty(),"missing junction preview");const auto& preview=e.junctionPlans[1];
auto approachEnd=g.lanes()[0].points.back();Vec2 tangent{std::cos(e.pathHeadings.back()),std::sin(e.pathHeadings.back())};
need((approachEnd.x-e.path.back().x)*tangent.x+(approachEnd.y-e.path.back().y)*tangent.y>=e.length*.5+.19,"approach front bumper projects past its endpoint");need(length(preview.points.front()-e.path.back())<1e-7,"preview uses a different entry position");need(std::abs(angleDelta(preview.headings.front(),e.pathHeadings.back()))<1e-7,"preview uses a different entry heading");}
need(std::abs(s.entities()[1].lateralOffset)>.1,"fixture did not exercise an offset sublane");
for(int i=0;i<18000 && !s.entities().empty();++i){s.tick(.01);for(const auto& e:s.entities())for(const auto& o:s.entities())if(e.id<o.id)need(clearance(e.footprint(),o.footprint())>=.3-1e-5,"handoff spacing violation");}need(s.completed()==3,"curved approach did not drain");
NodeGraph straight;straight.addNode({10,30});straight.addNode({80,30});straight.connect(0,1,6,6);straight.connections[0].roadType=1;
Simulation escape(Map(100,60,std::vector<Tile>(6000,Tile::Road)),42);escape.setLanes(straight.lanes());escape.setTrafficFlowSettings({true,true,.4,.15});escape.setStuckVehicleSettings({false,30});escape.setProfile(VehicleKind::Motorbike,{{2,0},{.7,0},{.3,0},{6,0,2,0},1});need(escape.spawnOnLane(0,VehicleKind::Motorbike)&&escape.spawnOnLane(0,VehicleKind::Motorbike),"escape fixture insertion");auto& actors=const_cast<std::vector<Entity>&>(escape.entities());
for(int i=0;i<2;++i){auto& e=actors[i];e.center={i?13.8:14.,i?28.8:30.};e.heading=0;e.path={e.center,{50,e.center.y}};e.pathHeadings={0,0};e.next=1;e.segmentProgress=0;e.goal=e.path.back();e.speed=0;e.sublaneAfter=e.laneChangeAfter=1e9;}
auto& owner=actors[1];owner.desiredSpeed=0;PathManeuver claim;claim.begin=0;claim.end=1;claim.sweep={{15.01,29.3},{21,29.3},{21,30.7},{15.01,30.7}};owner.sublaneReservation={claim};owner.sublaneReservationRevision=owner.routeRevision;owner.sublaneReservationEnd=1;owner.sublaneReservationProgress=10;
for(int i=0;i<200;++i){escape.tick(.01);need(clearance(actors[0].footprint(),actors[1].footprint())>=.3-1e-5,"escape violated physical spacing");}need(actors[0].center.x>15,"slightly leading parallel neighbor was trapped by a future merge claim");

Simulation pinch=escape;auto& packed=const_cast<std::vector<Entity>&>(pinch.entities());
for(int i=0;i<2;++i){auto& e=packed[i];e.center={14.,i?31.01:30.};e.heading=0;e.path={e.center,{14.1,e.center.y+(i?0:.02)},{50,e.center.y}};e.pathHeadings={0,i?0:.2,0};e.next=1;e.segmentProgress=0;e.goal=e.path.back();e.speed=0;e.currentStopSeconds=3;e.cooperativeRecovery=false;e.sublaneReservation.clear();e.trafficRecoveryAfter=0;}
packed[0].desiredSpeed=6;packed[1].desiredSpeed=0;packed[0].changingLanes=true;packed[0].changingFrom=packed[0].changingTo=0;packed[0].laneChangeEnd={40,30};packed[0].laneChangeDirection={1,0};
for(int i=0;i<400;++i){pinch.tick(.01);need(clearance(packed[0].footprint(),packed[1].footprint())>=.3-1e-5,"forward clearance recovery violated the preferred gap");}
need(packed[0].center.x>15 && pinch.trafficRecoveryRepairs()>0,"a clear forward escape did not untangle a rotation pinch");
Simulation shortPinch=pinch;auto& shortQueue=const_cast<std::vector<Entity>&>(shortPinch.entities());auto repairsBefore=shortPinch.trafficRecoveryRepairs();
for(int i=0;i<2;++i){auto& e=shortQueue[i];e.changingLanes=false;e.changingFrom=e.changingTo=-1;e.center={14.,i?31.01:30.};e.heading=0;e.path={e.center,{14.1,e.center.y+(i?0:.02)},{18,e.center.y}};e.pathHeadings={0,i?0:.2,0};e.next=1;e.segmentProgress=0;e.goal=e.path.back();e.speed=0;e.currentStopSeconds=3;e.cooperativeRecovery=false;e.sublaneReservation.clear();e.trafficRecoveryAfter=0;e.sublaneAfter=e.laneChangeAfter=1e9;}
for(int i=0;i<200;++i){shortPinch.tick(.01);need(shortQueue.size()==2 && clearance(shortQueue[0].footprint(),shortQueue[1].footprint())>=.3-1e-5,"short escape violated physical spacing");}need(shortQueue[0].center.x>15 && shortPinch.trafficRecoveryRepairs()>repairsBefore,"short remaining approach prevented a clearance escape");

Simulation edge=escape;auto& edgeActors=const_cast<std::vector<Entity>&>(edge.entities());edgeActors.resize(1);
auto& turning=edgeActors[0];turning.center={14,27.631};turning.heading=-.3;turning.path={turning.center,{14.1,27.59},{25,30},{50,30}};
turning.pathHeadings={-.3,-.5,0,0};turning.next=1;turning.segmentProgress=0;turning.goal={50,30};turning.desiredSpeed=6;turning.speed=0;
turning.currentStopSeconds=3;turning.cooperativeRecovery=false;turning.sublaneReservation.clear();turning.trafficRecoveryAfter=0;turning.sublaneAfter=turning.laneChangeAfter=1e9;
Vec2 edgeStart=turning.center;bool backed=false;
for(int i=0;i<1000;++i){edge.tick(.01);if(edgeActors.empty())break;backed=backed || edgeActors[0].center.x<edgeStart.x-.01;need(edge.map().fits(edgeActors[0].footprint()),"reverse clearance recovery left the mask");}
need(backed && (edge.completed()==1 || edgeActors[0].center.x>edgeStart.x+1),"lane-edge pinch did not reverse and realign");

Simulation boundary(Map(210,190,std::vector<Tile>(39900,Tile::Road)),123);boundary.setLanes(g.lanes());boundary.setTrafficFlowSettings({true,true,.4,.15});boundary.setStuckVehicleSettings({false,30});boundary.setProfile(VehicleKind::Motorbike,{{2,0},{.7,0},{.3,0},{6,0,2,0},1});
need(boundary.spawnOnLane(0,VehicleKind::Motorbike),"boundary fixture insertion failed");auto& boundaryActors=const_cast<std::vector<Entity>&>(boundary.entities());auto& atBoundary=boundaryActors[0];
auto stop=atBoundary.path.back();auto stopHeading=atBoundary.pathHeadings.back();Vec2 stopForward{std::cos(stopHeading),std::sin(stopHeading)};
atBoundary.center=stop-stopForward*.8;atBoundary.heading=stopHeading;atBoundary.path={atBoundary.center,stop};atBoundary.pathHeadings={stopHeading,stopHeading};atBoundary.next=1;atBoundary.segmentProgress=0;atBoundary.goal=stop;atBoundary.currentStopSeconds=3;atBoundary.waitingForJunction=true;
auto actualBoundary=atBoundary.center;
Simulation sourceBlocked=boundary;auto& sourceActors=const_cast<std::vector<Entity>&>(sourceBlocked.entities());Entity stoppedAhead=sourceActors[0];stoppedAhead.id=999;stoppedAhead.center=stoppedAhead.center+stopForward*2.35;stoppedAhead.goal=stoppedAhead.center+stopForward*.1;stoppedAhead.path={stoppedAhead.center,stoppedAhead.goal};stoppedAhead.pathHeadings={stopHeading,stopHeading};stoppedAhead.desiredSpeed=0;sourceActors.push_back(stoppedAhead);
sourceBlocked.tick(.01);need(!sourceActors[0].inOpenJunction,"junction admission ignored a road vehicle occupying its entry prefix");sourceActors.erase(sourceActors.begin()+1);for(int i=0;i<200 && !sourceActors[0].inOpenJunction;++i)sourceBlocked.tick(.01);need(sourceActors[0].inOpenJunction,"source-clearance wait did not release after the blocker left");
Simulation downstreamBlocked=boundary;auto& downstreamActors=const_cast<std::vector<Entity>&>(downstreamBlocked.entities());Entity downstream=downstreamActors[0];downstream.id=997;downstream.center=downstream.center+stopForward*8;downstream.goal=downstream.center+stopForward*.1;downstream.path={downstream.center,downstream.goal};downstream.pathHeadings={stopHeading,stopHeading};downstream.desiredSpeed=0;downstreamActors.push_back(downstream);
downstreamBlocked.tick(.01);need(!downstreamActors[0].inOpenJunction,"junction admission checked only its first step and committed into a downstream road queue");
downstreamActors.erase(downstreamActors.begin()+1);for(int i=0;i<200 && !downstreamActors[0].inOpenJunction;++i)downstreamBlocked.tick(.01);need(downstreamActors[0].inOpenJunction,"crossing corridor did not reopen after downstream road traffic cleared");
Simulation adjacentStopped=boundary;auto& adjacentActors=const_cast<std::vector<Entity>&>(adjacentStopped.entities());Entity stoppedCrossing=adjacentActors[0];stoppedCrossing.id=998;stoppedCrossing.center=stoppedCrossing.center-stopForward*2.35;stoppedCrossing.heading=stopHeading;stoppedCrossing.inOpenJunction=true;stoppedCrossing.networkStage=1;stoppedCrossing.activeJunction=g.lanes()[stoppedCrossing.networkRoute[1]].openJunction;stoppedCrossing.junctionCenter=g.lanes()[stoppedCrossing.networkRoute[1]].junctionCenter;stoppedCrossing.junctionRadius=g.lanes()[stoppedCrossing.networkRoute[1]].junctionRadius;stoppedCrossing.junctionEnteredAt=-10;stoppedCrossing.laneRoute.clear();stoppedCrossing.waitingForJunction=false;stoppedCrossing.desiredSpeed=0;stoppedCrossing.goal=stoppedCrossing.center+stopForward*.1;stoppedCrossing.path={stoppedCrossing.center,stoppedCrossing.goal};stoppedCrossing.pathHeadings={stopHeading,stopHeading};adjacentActors.push_back(stoppedCrossing);
adjacentStopped.tick(.01);need(adjacentActors[0].inOpenJunction,"a physically clear departure was blocked by an extra static admission cushion");
for(int i=0;i<200;++i){adjacentStopped.tick(.01);need(clearance(adjacentActors[0].footprint(),adjacentActors[1].footprint())>=.3-1e-5,"static-adjacent departure violated the preferred gap");}
need(length(adjacentActors[0].center-actualBoundary)>1,"a separating admitted vehicle did not clear its stopped neighbor");
boundary.tick(.01);
need(boundaryActors[0].inOpenJunction && length(boundaryActors[0].path.front()-actualBoundary)<1e-7,"near-end stalled approach was not handed off from its actual pose");

NodeGraph staggered;for(auto p:std::vector<Vec2>{{15,70},{80,70},{15,73},{88,73},{125,105},{170,150},{105,85}})staggered.addNode(p);
staggered.nodes[6].openRadius=35;staggered.nodes[5].destination=true;staggered.explicitDestinations=true;
staggered.connect(0,1,3,6);staggered.connect(2,3,3,6);staggered.connect(4,5,3,6);for(auto& c:staggered.connections)c.roadType=1;
Simulation stagger(Map(200,170,std::vector<Tile>(34000,Tile::Road)),42);stagger.setLanes(staggered.lanes());stagger.setTrafficFlowSettings({true,true,.4,.15});stagger.setProfile(VehicleKind::Motorbike,{{2,0},{.7,0},{.3,0},{6,0,2,0},1});
need(stagger.spawnOnLane(0,VehicleKind::Motorbike,2),"staggered approach crossing is unavailable");
const auto& staggerActor=stagger.entities().front();const auto& staggerPlan=staggerActor.junctionPlans[1];
need(staggerPlan.points.size()>1 && staggerPlan.points[1].x>89.1 && std::abs(staggerPlan.headings[1])<1e-7,"turn starts before rear bumper clears the staggered neighboring stop line");

Simulation nativeSignal(Map(100,60,std::vector<Tile>(6000,Tile::Road)),42);nativeSignal.setLanes(straight.lanes());nativeSignal.setTrafficFlowSettings({true,true,.4,.15});nativeSignal.setStuckVehicleSettings({false,30});nativeSignal.setProfile(VehicleKind::Motorbike,{{2,0},{.7,0},{.3,0},{4,0,2,0},1});
TrafficLight nativeLight{{30,25},{30,35},1,1,10,2};nativeSignal.setTrafficLights({nativeLight});need(nativeSignal.spawnOnLane(0,VehicleKind::Motorbike),"native red-light insertion failed");auto committedRevision=nativeSignal.entities()[0].routeRevision;
for(int i=0;i<900;++i){nativeSignal.tick(.01);const auto& e=nativeSignal.entities().front();need(e.signalLight==-1 && e.routeRevision==committedRevision,"legacy signal planner replaced the native committed route");need(e.center.x+e.length*.5<30,"native vehicle crossed a red stop line");}
need(nativeSignal.entities()[0].center.x>28 && nativeSignal.entities()[0].speed<.1,"native vehicle did not brake close to its red light");nativeLight.greenSeconds=100;nativeLight.yellowSeconds=nativeLight.redSeconds=1;nativeLight.offsetSeconds=0;nativeSignal.setTrafficLights({nativeLight});for(int i=0;i<2500 && !nativeSignal.entities().empty();++i)nativeSignal.tick(.01);need(nativeSignal.completed()==1,"native red-light stop did not release on green");

Simulation permitted(Map(100,60,std::vector<Tile>(6000,Tile::Road)),42);permitted.setLanes(straight.lanes());permitted.setTrafficFlowSettings({true,true,.4,.15});permitted.setStuckVehicleSettings({false,30});permitted.setProfile(VehicleKind::Motorbike,{{2,0},{.7,0},{.3,0},{4,0,2,0},1});
TrafficLight permission{{30,25},{30,35},1,1,10,2};permission.permissive=true;permission.allowedA={45,25};permission.allowedB={45,35};permitted.setTrafficLights({permission});need(permitted.spawnOnLane(0,VehicleKind::Motorbike),"permitted native insertion failed");
for(int i=0;i<1000;++i)permitted.tick(.01);need(permitted.entities().size()==1 && permitted.completed()==0 && permitted.entities()[0].center.x>45,"native red permission stopped the trip or despawned it at the allowed-exit line");
for(int i=0;i<1500 && !permitted.entities().empty();++i)permitted.tick(.01);need(permitted.completed()==1,"permitted native trip did not reach its true destination");

std::cout<<"Curved/tapered sublane previews match the actual entry pose and all vehicles finish\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
