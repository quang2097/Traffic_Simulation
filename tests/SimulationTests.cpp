#include "traffic/Simulation.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
using namespace traffic;
void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
VehicleProfile fixed(double l=1,double w=.4,double gap=.3,double speed=4,double accel=2) {
    return {{l,0},{w,0},{gap,0},{speed,0,accel,0},1};
}
Map openMap(int w=60,int h=30,double scale=1) { return Map(w,h,std::vector<Tile>(w*h,Tile::Road),scale); }
void invariants(const Simulation& s) {
    for(std::size_t i=0;i<s.entities().size();++i) {
        const auto& a=s.entities()[i];
        require(s.map().fits(a.footprint()),"vehicle intersects mask");
        for(std::size_t j=i+1;j<s.entities().size();++j) {
            const auto& b=s.entities()[j];
            require(clearance(a.footprint(),b.footprint())+1e-5>=std::min(a.preferredDistance,b.preferredDistance),"pairwise minimum clearance violated");
        }
    }
}
void geometryTests() {
    std::mt19937 random(273);
    std::uniform_real_distribution<double> coordinate(-15,15),angle(-3.14,3.14),size(.2,8),gap(.001,4);
    for(int i=0;i<2000;++i) {
        Footprint first{{coordinate(random),coordinate(random)},angle(random),size(random),size(random)};
        Footprint second{{coordinate(random),coordinate(random)},angle(random),size(random),size(random)};
        double required=gap(random);
        require(geometry::closerThan(corners(first),corners(second),required)==(clearance(first,second)<required),"fast convex check agrees with exact rectangle distance");
    }
    Footprint a{{5,5},0,4,2},b{{10,5},0,4,2};
    require(std::abs(clearance(a,b)-1)<1e-9,"edge distance");
    b.heading=1.5707963267948966;
    require(std::abs(clearance(a,b)-2)<1e-9,"oriented distance");
    b.center={6,5}; require(clearance(a,b)==0,"overlapping boxes");
    auto map=openMap(20,20,.5);
    require(map.center({4,4}).x==2.25,"physical map scaling");
    require(!map.contains(map.pixel({1e300,5})),"extreme world coordinate safely rejected");
    require(map.fits({{5,5},0,4,2}),"valid footprint");
    require(!map.fits({{.5,5},0,4,2}),"footprint off mask boundary");
    map.tiles[map.index({10,10})]=Tile::Wall;
    require(!map.fits({{5,5},.4,4,2}),"rotated footprint hits wall");
    Map thin=openMap(); thin.tiles[thin.index({20,15})]=Tile::Wall;
    double pad; auto sweep=geometry::swept({{18,15.5},0,1,.4},{{23,15.5},0,1,.4},pad);
    require(!geometry::fits(thin,sweep,pad),"swept box tunnels through a thin wall");
    require(geometry::intersectsLine(sweep,{20,10},{20,20}),"stop line intersection");
    // Pixel diagonal corner contact must not be cut through by a body.
    Map corner=openMap(10,10); corner.tiles[corner.index({4,5})]=Tile::Wall;
    auto diagonal=geometry::swept({{4.5,4.5},.785,1,1},{{5.5,5.5},.785,1,1},pad);
    require(!geometry::fits(corner,diagonal,pad),"diagonal corner cutting");
}
void motionTests() {
    Simulation s(openMap(),123); s.setProfile(VehicleKind::Car,fixed());
    require(s.spawnAt({5,15},{50,15},VehicleKind::Car,0),"spawn");
    s.tick(.5); const auto& e=s.entities().front();
    require(std::abs(e.speed-1)<1e-9 && std::abs(e.center.x-5.25)<1e-9,"SI acceleration integration");
    require(s.setEntityMotion(e.id,5,3),"individual motion");
    s.setMotionDistribution({6,0,7,0}); s.resampleMotion();
    require(s.entities()[0].desiredSpeed==6 && s.entities()[0].acceleration==7,"resampling");
    bool rejected=false;
    try { s.tick(std::numeric_limits<double>::quiet_NaN()); } catch(const std::invalid_argument&) { rejected=true; }
    require(rejected,"reject NaN dt");
    rejected=false;
    try { auto p=fixed(); p.width.stdDev=-1; s.setProfile(VehicleKind::Bike,p); } catch(const std::invalid_argument&) { rejected=true; }
    require(rejected,"reject invalid body distribution");
    Simulation scaled(openMap(120,60,.5),123); scaled.setProfile(VehicleKind::Car,fixed());
    require(scaled.spawnAt({5,15},{50,15},VehicleKind::Car,0),"scaled spawn"); scaled.tick(.5);
    require(std::abs(scaled.entities()[0].center.x-5.25)<1e-9,"speed must not depend on pixel scale");
}
void arcTests() {
    auto map=openMap(60,40);
    auto motion=curvePath(map,{{5,10},0,4.5,1.8},{{5,10},{20,10},{20,30}});
    require(!motion.points.empty() && motion.turns.size()==1,"rounded ninety-degree corner");
    auto turn=motion.turns.front();
    std::mt19937 random(591);
    std::uniform_real_distribution<double> shift(-15,15),gap(.05,4);
    for(int trial=0;trial<100;++trial) {
        auto translated=turn.sweep; Vec2 offset{shift(random),shift(random)};
        for(auto& p:translated) p=p+offset;
        double required=gap(random);
        require(geometry::closerThan(turn.sweep,translated,required)==(geometry::distance(turn.sweep,translated)<required),
            "fast curved-reservation check agrees with exact polygon distance");
    }
    Vec2 center{16.4,13.6};
    for(std::size_t i=turn.begin;i<=turn.end;++i) {
        require(std::abs(length(motion.points[i]-center)-3.6)<1e-8,"corner samples lie on a circular arc");
        require(map.fits({motion.points[i],motion.headings[i],4.5,1.8}),"arc footprint stays inside mask");
    }
    auto uturn=curvePath(map,{{30,15},0,1,.4},{{30,15},{10,15}});
    require(!uturn.points.empty() && !uturn.turns.empty(),"heading-changing replan has a translating connector");
    for(std::size_t i=1;i<uturn.points.size();++i)
        require(length(uturn.points[i]-uturn.points[i-1])>0,"connector never pivots at a fixed center");
    Map corner(40,40,std::vector<Tile>(1600,Tile::Wall));
    for(int y=5;y<36;++y) for(int x=5;x<31;++x)
        if(y<19 || x>=20) corner.tiles[corner.index({x,y})]=Tile::Road;
    Simulation s(corner); s.setProfile(VehicleKind::Car,fixed(1,.4,.2,4,4));
    require(s.spawnAt({10,12},{25,31},VehicleKind::Car,0),"L-mask arc spawn");
    bool turned=false;
    for(int step=0;step<1800 && !s.entities().empty();++step) {
        Entity before=s.entities()[0]; s.tick(.01); invariants(s);
        if(s.entities().empty()) break;
        const auto& after=s.entities()[0];
        if(std::abs(angleDelta(before.heading,after.heading))>1e-6) {
            turned=true;
            require(length(after.center-before.center)>1e-6,"turn must translate rather than pivot");
            require(after.speed>=3.999,"unobstructed corner preserves cruising momentum");
        }
        require(after.speed+1e-8>=before.speed,"clear turns never reset speed");
    }
    require(turned && s.completed()==1 && s.recoveries()==0,"curved route arrives without a recovery");
}
void gaussianTests() {
    Simulation s(openMap(),321),replica(openMap(),321);
    auto p=fixed(); p.length={4,.3}; p.width={1.8,.1}; p.distance={1.2,.2}; p.motion={8,1,5,.5};
    s.setProfile(VehicleKind::Car,p); replica.setProfile(VehicleKind::Car,p);
    double sum[5]{},sum2[5]{};
    constexpr int count=2000;
    for(int i=0;i<count;++i) {
        require(s.spawnAt({10,15},{10,15},VehicleKind::Car,0),"sample spawn");
        require(replica.spawnAt({10,15},{10,15},VehicleKind::Car,0),"replica spawn");
        auto a=s.entities()[0],b=replica.entities()[0];
        double values[]{a.length,a.width,a.preferredDistance,a.desiredSpeed,a.acceleration};
        require(a.length==b.length && a.width==b.width && a.desiredSpeed==b.desiredSpeed,"seed reproducibility");
        for(int j=0;j<5;++j) { sum[j]+=values[j]; sum2[j]+=values[j]*values[j]; }
        s.tick(.01); replica.tick(.01);
    }
    double means[]{4,1.8,1.2,8,5},variances[]{.09,.01,.04,1,.25};
    for(int j=0;j<5;++j) {
        double mean=sum[j]/count,var=sum2[j]/count-mean*mean;
        require(std::abs(mean-means[j])<std::sqrt(variances[j])*.1,"Gaussian mean");
        require(std::abs(var-variances[j])<variances[j]*.15,"Gaussian variance");
    }
}
void gapTests() {
    Simulation s(openMap());
    s.setProfile(VehicleKind::Car,fixed(1,.5,3));
    s.setProfile(VehicleKind::Bike,fixed(1,.5,1));
    require(s.spawnAt({10,10},{50,10},VehicleKind::Car,0),"first gap vehicle");
    require(s.spawnAt({12.2,10},{50,10},VehicleKind::Bike,0),"smaller preferred distance must win");
    require(!s.spawnAt({11,10},{50,10},VehicleKind::Bike,0),"overlapping spawn rejected");
    Map narrow=openMap(50,28);
    for(int x=22;x<=25;++x) for(int y=0;y<28;++y)
        if(y<13 || y>14) narrow.tiles[narrow.index({x,y})]=Tile::Wall;
    Simulation fit(narrow); fit.setProfile(VehicleKind::Bike,fixed(1.8,.6,.2));
    fit.setProfile(VehicleKind::Truck,fixed(8,2.5,.2));
    require(fit.spawnAt({10,14},{40,14},VehicleKind::Bike,0),"bike fits two-meter gap");
    require(!fit.spawnAt({10,20},{40,20},VehicleKind::Truck,0),"truck must not fit narrow gap");
    // Bike starts directly behind a slow truck, then finds a lateral gap.
    Simulation passing(openMap(55,28),3);
    passing.setProfile(VehicleKind::Truck,fixed(8,2.5,.2,.05,.05));
    passing.setProfile(VehicleKind::Bike,fixed(1.8,.6,.2,5,2));
    require(passing.spawnAt({27,14},{48,14},VehicleKind::Truck,0),"slow truck");
    require(passing.spawnAt({15,14},{45,14},VehicleKind::Bike,0),"following bike");
    bool passed=false;
    for(int i=0;i<3000;++i) {
        passing.tick(.01); invariants(passing);
        for(const auto& e:passing.entities()) if(e.kind==VehicleKind::Bike && e.center.x>33) passed=true;
    }
    if(!passed) {
        std::cerr<<"Replans "<<passing.replans()<<'\n';
        for(const auto& v:passing.entities()) {
            std::cerr<<vehicleName(v.kind)<<" at "<<v.center.x<<','<<v.center.y<<" heading "<<v.heading<<" next "<<v.next<<" blocked "<<v.blockedSeconds<<" path ";
            std::cerr<<v.path.size()<<" curve samples\n";
        }
    }
    require(passing.replans()>0 && passed,"bike must slip around slower truck into fitting gap");
    require(passing.localDetours()>0,"RTS avoidance uses a local route patch");
}
void navigationTests() {
    Map map=openMap(40,24);
    for(int y=0;y<24;++y) map.tiles[map.index({20,y})]=Tile::Wall;
    Navigation split(map);
    require(!split.connected({10,10},{30,10}),"hierarchy detects disconnected destinations");
    Simulation unreachable(map);
    require(!unreachable.spawnAt({10,10},{30,10}),"disconnected spawn rejected");
    require(unreachable.globalPlans()==0,"disconnected map avoids an expensive detailed search");
    map.tiles[map.index({20,12})]=Tile::Road;
    Navigation joined(map);
    require(joined.connected({10,10},{30,10}),"sector graph retains a one-pixel portal");
    require(joined.regions().size()>1,"map has a multi-area hierarchy");
    int goal=joined.regionAt({30,10});
    auto costs=joined.goalField(goal);
    require(std::isfinite(costs[joined.regionAt({10,10})]),"global field crosses the portal");
    joined.goalField(goal);
    require(joined.fieldBuilds()==1 && joined.cacheHits()==1,"nearby units share a cached global field");
    Simulation shared(openMap()); shared.setProfile(VehicleKind::Bike,fixed(1,.4,.2));
    require(shared.spawnAt({5,8},{50,10},VehicleKind::Bike,0),"first shared-goal unit");
    require(shared.spawnAt({5,16},{50,10},VehicleKind::Bike,0),"second shared-goal unit");
    require(shared.navigation().cacheHits()>0,"simulation reuses destination-area guidance");
    auto settings=shared.routingSettings(); settings.lookAhead=-1; bool rejected=false;
    try { shared.setRoutingSettings(settings); } catch(const std::invalid_argument&) { rejected=true; }
    require(rejected,"invalid local-routing settings rejected");
    Simulation legacy(openMap()); auto old=legacy.routingSettings(); old.rtsStyle=false; legacy.setRoutingSettings(old);
    legacy.setProfile(VehicleKind::Car,fixed()); require(legacy.spawnAt({5,15},{50,15},VehicleKind::Car,0),"classic routing remains available");
    legacy.tick(.5); require(std::abs(legacy.entities()[0].center.x-5.25)<1e-9,"classic option preserves SI motion");
    Simulation lane(openMap(60,1));
    lane.setProfile(VehicleKind::Bike,fixed(1,.4,.5,1,2));
    lane.setProfile(VehicleKind::Car,fixed(1,.4,.5,6,3));
    require(lane.spawnAt({14,.5},{50,.5},VehicleKind::Bike,0),"narrow-lane leader");
    require(lane.spawnAt({6,.5},{50,.5},VehicleKind::Car,0),"narrow-lane follower");
    for(int i=0;i<1500;++i) { lane.tick(.01); invariants(lane); }
    require(lane.entities().size()==2 && lane.entities()[1].following,"narrow-lane unit follows the leader");
    require(std::abs(lane.entities()[1].speed-1)<.1,"following approaches leader speed without collision-stop jitter");
    require(lane.localDetours()==0 && lane.recoveries()==0,"an unfitting gap keeps an orderly queue");
}
void signalTests() {
    TrafficLight light{{20,0},{20,29},1,.5,3,1.5,true};
    require(light.phase(0)==LightPhase::Red && light.phase(3)==LightPhase::Green,"phase offset");
    require(light.phase(4)==LightPhase::Yellow,"yellow phase");
    Simulation s(openMap()); s.setProfile(VehicleKind::Car,fixed(4,1.8,.5,10,10));
    s.setTrafficLights({light});
    require(s.spawnAt({15,15},{45,15},VehicleKind::Car,0),"signal spawn");
    for(int i=0;i<290;++i) {
        s.tick(.01); invariants(s);
        require(s.entities()[0].center.x+2<20+1e-7,"front bumper crossed a red light");
    }
    bool crossed=false;
    for(int i=0;i<500;++i) {
        s.tick(.01);
        if(s.entities().empty() || s.entities()[0].center.x>20) crossed=true;
    }
    require(crossed,"green must release waiting vehicle");
    // High-speed crossings in both directions are tested using swept bodies.
    for(int direction:{-1,1}) {
        Simulation fast(openMap()); fast.setProfile(VehicleKind::Bike,fixed(1,.3,0,50,100));
        auto red=light; red.greenSeconds=.1; red.yellowSeconds=0; red.redSeconds=100; red.offsetSeconds=1;
        fast.setTrafficLights({red});
        Vec2 start{direction==1?10.:30.,15},goal{direction==1?45.:5.,15};
        require(fast.spawnAt(start,goal,VehicleKind::Bike,0),"fast signal spawn");
        fast.tick(2);
        require(direction==1?fast.entities()[0].center.x+.5<20:fast.entities()[0].center.x-.5>20,"swept red stop in both directions");
    }
    // Do not trap a vehicle already straddling the line on phase change.
    Simulation clearing(openMap()); clearing.setProfile(VehicleKind::Car,fixed(4,1,.2,5,5));
    auto red=light; red.offsetSeconds=2; clearing.setTrafficLights({red});
    require(clearing.spawnAt({20,15},{45,15},VehicleKind::Car,0),"straddling spawn");
    clearing.tick(1); require(clearing.entities()[0].center.x>20,"straddler clears red");
    Simulation queue(openMap()); queue.setProfile(VehicleKind::Car,fixed(4,1.8,.5,10,10));
    auto longRed=light; longRed.greenSeconds=1; longRed.yellowSeconds=0; longRed.redSeconds=20; longRed.offsetSeconds=1;
    queue.setTrafficLights({longRed});
    require(queue.spawnAt({15,15},{45,15},VehicleKind::Car,0),"red queue leader");
    require(queue.spawnAt({8,15},{45,15},VehicleKind::Car,0),"red queue follower");
    for(int i=0;i<1200;++i) { queue.tick(.01); invariants(queue); }
    require(queue.recoveries()==0,"legitimate red-light queue must not trigger reverse recovery");
    require(queue.entities()[0].waitingForLight && queue.entities()[1].waitingForLight,"signal queue waiting propagates to followers");
}
void signalSpreadingTests() {
    Simulation wide(openMap(60,24),91);
    wide.setProfile(VehicleKind::Car,fixed(4,1.8,.5,6,3));
    wide.setProfile(VehicleKind::Bike,fixed(1.8,.6,.2,4,3));
    TrafficLight red{{30,1},{30,23},.5,0,30,.5,true};
    wide.setTrafficLights({red});
    for(double x:{24.,16.,8.}) require(wide.spawnAt({x,12},{50,12},VehicleKind::Car,0),"wide red approach spawn");
    require(wide.spawnAt({3,12},{50,12},VehicleKind::Bike,0),"mixed-size red approach bike");
    bool filling=false;
    for(int i=0;i<1000;++i) {
        std::vector<Vec2> previous;
        for(const auto& e:wide.entities()) previous.push_back(e.center);
        wide.tick(.02); invariants(wide);
        require(wide.entities().size()==4 && wide.completed()==0,"signal slots are holding positions, not destinations");
        bool stopped=false,moving=false;
        for(std::size_t j=0;j<wide.entities().size();++j) {
            const auto& e=wide.entities()[j];
            require(length(e.center-previous[j])<=50*.02+1e-6,"red spreading preserves continuous motion");
            for(Vec2 p:corners(e.footprint())) require(p.x<30+1e-8,"spreading body crosses the red stop line");
            stopped=stopped || (e.waitingForLight && e.speed<.01);
            moving=moving || e.speed>.1;
        }
        filling=filling || (stopped && moving);
    }
    int frontRow=0; double minY=100,maxY=-100;
    for(const auto& e:wide.entities()) if(e.center.x>26.5) { ++frontRow; minY=std::min(minY,e.center.y); maxY=std::max(maxY,e.center.y); }
    if(frontRow<4) for(const auto& e:wide.entities()) std::cerr<<"signal vehicle "<<e.id<<" at "<<e.center.x<<','<<e.center.y<<" slot "<<e.signalLight<<" remaining "<<e.signalRemaining<<'\n';
    require(frontRow==4 && maxY-minY>4,"wide approach fills multiple mixed-size positions along the red line");
    require(filling && wide.signalPlacements()>=4,"followers keep advancing into usable red-light space");
    wide.setTrafficLights({}); // Removing a queued signal must also release its vehicles.
    for(int i=0;i<6000 && !wide.entities().empty();++i) { wide.tick(.02); invariants(wide); }
    require(wide.completed()==4 && wide.entities().empty(),"spread vehicles resume their original destinations");
    Simulation narrow(openMap(60,3),7); narrow.setProfile(VehicleKind::Car,fixed(4,1.8,.5,6,3));
    red.a={30,.01}; red.b={30,2.99}; narrow.setTrafficLights({red});
    require(narrow.spawnAt({24,1.5},{50,1.5},VehicleKind::Car,0),"narrow red leader");
    require(narrow.spawnAt({16,1.5},{50,1.5},VehicleKind::Car,0),"narrow red follower");
    for(int i=0;i<700;++i) { narrow.tick(.02); invariants(narrow); }
    frontRow=0;
    for(const auto& e:narrow.entities()) if(e.center.x>26.5) ++frontRow;
    require(frontRow==1 && narrow.entities().size()==2,"narrow mask keeps one queue when side-by-side boxes cannot fit");
}
void liveSignalTests() {
    Simulation s(openMap(70,12),42);
    s.setProfile(VehicleKind::Car,fixed(4,1.8,.5,6,3));
    auto routing=s.routingSettings(); routing.spreadAtSignals=false; s.setRoutingSettings(routing);
    TrafficLight first{{20,.01},{20,11.99},.1,0,100,1,true};
    TrafficLight second{{45,.01},{45,11.99},.1,0,100,1,true};
    s.setTrafficLights({first,second});
    require(s.spawnAt({10,6},{62,6},VehicleKind::Car,0),"live light spawn");
    for(int i=0;i<500;++i) s.tick(.01);
    require(s.entities().size()==1 && s.entities()[0].center.x+2<20,"first light stops vehicle");
    auto before=s.entities()[0]; double clock=s.seconds(); auto ticks=s.ticks();
    first.greenSeconds=100; first.redSeconds=.1;
    s.setTrafficLights({first,second});
    require(s.seconds()==clock && s.ticks()==ticks && s.entities()[0].id==before.id &&
        length(s.entities()[0].center-before.center)<1e-12 && s.entities()[0].speed==before.speed,
        "live timing edit preserves clock and vehicle state");
    require(s.trafficLights()[0].phase(clock)==LightPhase::Green &&
        s.trafficLights()[1].phase(clock)==LightPhase::Red,"independent light timings");
    for(int i=0;i<1200;++i) { s.tick(.01); invariants(s); }
    require(s.entities().size()==1 && s.entities()[0].center.x>25 && s.entities()[0].center.x+2<45,
        "retimed first light releases vehicle while second stays red");
    auto lights=s.trafficLights(); lights.push_back({{55,.01},{55,11.99},100,0,.1,0,true});
    clock=s.seconds(); before=s.entities()[0]; s.setTrafficLights(lights);
    require(s.trafficLights().size()==3 && s.seconds()==clock && s.entities()[0].id==before.id,
        "adding multiple lights preserves active traffic");
    lights.erase(lights.begin()+1); s.setTrafficLights(lights);
    for(int i=0;i<2000 && !s.entities().empty();++i) { s.tick(.01); invariants(s); }
    require(s.completed()==1,"removing a light while running releases traffic");
}
int main() {
    try {
        geometryTests(); motionTests(); arcTests(); gaussianTests(); gapTests(); navigationTests(); signalTests(); signalSpreadingTests(); liveSignalTests();
        std::cout<<"All geometry, SI motion, Gaussian dimensions, gap passing and signal tests passed\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
