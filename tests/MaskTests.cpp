#include "traffic/Simulation.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>
using namespace traffic;
int main(int argc,char** argv) {
    std::cout<<std::unitbuf;
    try {
        if(argc<2) throw std::runtime_error("Expected sample mask path");
        if(argc>2) {
            Map colors=Map::fromPng(argv[2],.1,MapPalette::Legacy);
            std::vector<Tile> expected{Tile::Wall,Tile::Road,Tile::Wall,Tile::Spawn,Tile::Destination,Tile::Spawn,Tile::Destination,Tile::Road};
            if(colors.tiles!=expected || colors.metersPerPixel!=.1) throw std::runtime_error("Painted mask threshold, alpha, markers, or scale failed");
        }
        Map map=Map::fromPng(argv[1],.5);
        for(int i=0;i<vehicleKindCount;++i) {
            auto start=std::chrono::steady_clock::now();
            Simulation s(map,42); auto kind=static_cast<VehicleKind>(i);
            if(!s.spawn({16,50},{133,37},3,kind)) throw std::runtime_error(std::string("Failed mask routing: ")+vehicleName(kind));
            auto elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
            std::cout<<vehicleName(kind)<<" route: "<<elapsed<<"s, "<<s.entities()[0].path.size()<<" waypoints\n";
            for(int step=0;step<6000 && !s.entities().empty();++step) {
                s.tick(.01);
                if(!s.entities().empty() && !s.map().fits(s.entities()[0].footprint())) throw std::runtime_error("Body left painted mask");
            }
            if(s.completed()!=1) throw std::runtime_error(std::string("Failed mask arrival: ")+vehicleName(kind));
        }
        Simulation mixed(map,42); int spawned=0;
        // Keep the all-arrivals regression independent of automatic removal.
        mixed.setStuckVehicleSettings({false,30});
        mixed.setTrafficLights({{{30,22},{30,38},10,2,12,0,true}});
        auto before=std::chrono::steady_clock::now();
        for(int step=0;step<1200;++step) {
            if(step%40==0 && mixed.spawnRandom(3)) ++spawned;
            mixed.tick(.025);
            for(std::size_t i=0;i<mixed.entities().size();++i) {
                const auto& a=mixed.entities()[i];
                if(!mixed.map().fits(a.footprint())) throw std::runtime_error("Mixed vehicle left mask");
                for(std::size_t j=i+1;j<mixed.entities().size();++j) {
                    const auto& b=mixed.entities()[j];
                    if(clearance(a.footprint(),b.footprint())+1e-5<std::min(a.preferredDistance,b.preferredDistance))
                        throw std::runtime_error("Mixed traffic violates clearance");
                }
            }
        }
        std::cout<<"Mixed traffic: "<<spawned<<" spawned, "<<mixed.completed()<<" completed, "<<mixed.replans()
                 <<" replans in "<<std::chrono::duration<double>(std::chrono::steady_clock::now()-before).count()<<" wall seconds\n";
        if(spawned<5 || mixed.completed()<1) throw std::runtime_error("Mixed traffic makes no progress");
        for(int step=0;step<12000 && !mixed.entities().empty();++step) {
            mixed.tick(.025);
            if(step%4==0) for(std::size_t i=0;i<mixed.entities().size();++i) {
                const auto& a=mixed.entities()[i];
                if(!mixed.map().fits(a.footprint())) throw std::runtime_error("Drain recovery left mask");
                for(std::size_t j=i+1;j<mixed.entities().size();++j) {
                    const auto& b=mixed.entities()[j];
                    if(clearance(a.footprint(),b.footprint())+1e-5<std::min(a.preferredDistance,b.preferredDistance))
                        throw std::runtime_error("Drain recovery violates clearance");
                }
            }
        }
        if(!mixed.entities().empty()) {
            std::cerr<<"Undrained mixed traffic: "<<mixed.entities().size()<<'\n';
            for(const auto& e:mixed.entities()) {
                std::cerr<<e.id<<' '<<vehicleName(e.kind)<<" at "<<e.center.x<<','<<e.center.y<<" heading "<<e.heading
                         <<" target "<<e.path[e.next].x<<','<<e.path[e.next].y<<" blocked "<<e.blockedSeconds<<'\n';
            }
            throw std::runtime_error("Mixed traffic permanently stuck after spawning stops");
        }
        if(mixed.completed()!=static_cast<unsigned>(spawned)) throw std::runtime_error("Every admitted vehicle must arrive");
        std::cout<<"Drain completed at "<<mixed.seconds()<<"s: all "<<mixed.completed()<<" arrived, "<<mixed.recoveries()<<" yielding recoveries\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
