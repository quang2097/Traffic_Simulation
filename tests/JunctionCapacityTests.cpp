#include "traffic/Demo.hpp"
#include <iostream>
#include <stdexcept>
using namespace traffic;
void need(bool b,const char* message) { if(!b) throw std::runtime_error(message); }
int road(const Simulation& s,int node,bool entry) {
    for(std::size_t i=0;i<s.lanes().size();++i) {
        const auto& lane=s.lanes()[i];
        if(lane.roadGroup>=0 && (entry?lane.sourceNode:lane.destinationNode)==node) return static_cast<int>(i);
    }
    throw std::runtime_error("missing road");
}
int main() {
 try {
    for(bool compact:{false,true}) {
        Simulation s(makeJunctionDemoMap(),42); s.setLanes(makeJunctionDemoGraph().lanes());
        s.setTrafficFlowSettings({true,true,.4,.15,compact}); s.setStuckVehicleSettings({false,30});
        s.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{7,0,2,0},1});
        need(s.spawnOnLane(road(s,1,true),VehicleKind::Car,road(s,2,false)),"first crossing spawn"); s.tick(8);
        auto& stopped=const_cast<std::vector<Entity>&>(s.entities()).front();
        need(stopped.inOpenJunction,"fixture must stop inside the junction");
        auto stoppedId=stopped.id; auto stoppedPoint=stopped.center;
        stopped.speed=stopped.desiredSpeed=0; stopped.currentStopSeconds=2; stopped.junctionEnteredAt=4;
        need(s.spawnOnLane(road(s,2,true),VehicleKind::Car,road(s,1,false)),"disjoint crossing spawn");
        for(int i=0;i<2000;++i) {
            s.tick(.01);
            for(const auto& e:s.entities()) {
                need(s.map().fits(e.footprint()),"crossing left mask");
                for(const auto& o:s.entities()) if(e.id<o.id) need(clearance(e.footprint(),o.footprint())>=.5-1e-5,"concurrent crossing violated clearance");
                if(e.id==stoppedId) need(length(e.center-stoppedPoint)<1e-8,"blocked vehicle was moved to manufacture throughput");
            }
        }
        need(s.removedStuckVehicles()==0,"capacity must not rely on deletion");
        if(compact) need(s.completed()==1 && s.junctionAdmissions()==2,"a blocked stream must not close a physically disjoint crossing");
        else need(s.completed()==0 && s.junctionAdmissions()==1,"baseline admission behavior changed");
    }
    std::cout<<"Independent crossings continue around a stopped stream without moving or removing the blocker\n";
 } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
