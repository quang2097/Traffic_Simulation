#include "traffic/SumoBackend.hpp"
#include "traffic/Demo.hpp"
#include "traffic/SpawnClock.hpp"
#include <filesystem>
#include <iostream>
#include <stdexcept>

#ifdef _WIN32
int main(int argc,char** argv) {
    try {
        using namespace traffic;
        if(argc!=1 && argc<4)throw std::invalid_argument("Usage: traffic_demo [map.png network.nodes meters-per-pixel [seconds [seed]]]");
        auto map=argc>1?Map::fromPng(argv[1],std::stod(argv[3])):makeJunctionDemoMap();
        auto graph=argc>1?NodeGraph::load(argv[2]):makeJunctionDemoGraph();
        double duration=argc>4?std::stod(argv[4]):120;
        unsigned seed=argc>5?static_cast<unsigned>(std::stoul(argv[5])):42;
        Simulation simulation(map,seed);auto compiled=compileRoads(graph);simulation.setLanes(compiled.lanes);
        simulation.setTrafficFlowSettings({true,true,.25,.1});
        if(argc>1 && std::filesystem::exists(std::string(argv[2])+".signals"))simulation.loadTrafficLights(std::string(argv[2])+".signals");
        SumoBackend engine(simulation,graph,seed);
        std::vector<double> clocks(graph.nodes.size());unsigned offered=0;
        for(double time=0;time<duration;time+=.1){
            for(std::size_t n=0;n<graph.nodes.size();++n)if(graph.nodes[n].spawn && consumeSpawnClock(clocks[n],.1,graph.nodes[n].rate)){
                if(simulation.entities().size()+engine.pending()>=100)continue;
                for(std::size_t l=0;l<compiled.originNodes.size();++l)if(compiled.originNodes[l]==static_cast<int>(n)){offered+=simulation.spawnRandomOnLane(static_cast<int>(l));break;}
            }
            simulation.tick(.1);
        }
        std::cout<<engine.status()<<"\nSeconds: "<<simulation.seconds()<<" | admitted requests: "<<offered<<" | completed: "<<simulation.completed()
                 <<" | active: "<<simulation.entities().size()<<" | pending: "<<engine.pending()<<" | SUMO collision involvements: "<<engine.collisions()
                 <<" | teleports: "<<engine.teleports()<<" | free handoffs: "<<engine.handoffs()<<"\n";
    } catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}

#else
int main(){std::cerr<<"The current SUMO process bridge requires Windows.\n";return 1;}
#endif
