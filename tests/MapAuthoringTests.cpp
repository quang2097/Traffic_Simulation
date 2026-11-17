#include "traffic/Simulation.hpp"
#include <iostream>
#include <stdexcept>
using namespace traffic;
void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
int main(int argc,char** argv) {
    try {
        require(argc==3,"Expected example and roundtrip paths");
        auto map=Map::fromPng(argv[1],.5);
        int spawns=0,ends=0,signals=0,dividers=0;
        for(const auto& f:map.features()) {
            spawns+=f.type==Tile::Spawn; ends+=f.type==Tile::Destination;
            signals+=f.type==Tile::Signal; dividers+=f.type==Tile::Divider;
            if(f.type==Tile::Destination) require(f.line,"Exit line not recognized");
        }
        require(spawns==2 && ends==1 && signals==2 && dividers==1,"Annotated components incorrect");
        map.savePng(argv[2]); auto copy=Map::fromPng(argv[2],.5);
        require(map.tiles==copy.tiles,"PNG roundtrip changed semantic pixels");
        Simulation imported(copy); require(imported.trafficLights().size()==2,"Signals not imported");
        Map divided(40,20,std::vector<Tile>(800,Tile::Road));
        divided.paintStroke({20.5,.5},{20.5,19.5},Tile::Divider,.6);
        divided.paintStroke({10.5,10.5},{30.5,10.5},Tile::Signal,.6);
        divided.prepareCollision(); divided.rebuildFeatures();
        require(!divided.walkable({20,10}),"Marker erased divider");
        require(!divided.fits({{20.5,10.5},0,1,1}),"Body crossed divider");
        require(!Navigation(divided).connected({10,10},{30,10}),"Routing crossed solid divider");
        Map exit(40,20,std::vector<Tile>(800,Tile::Road));
        exit.paintStroke({39.5,.5},{39.5,19.5},Tile::Destination,.6); exit.rebuildFeatures();
        Simulation traffic(exit,42);
        require(traffic.spawnAt({5.5,10.5},{39.5,10.5}),"Border exit rejected");
        for(int i=0;i<3000 && !traffic.entities().empty();++i) {
            traffic.tick(.01);
            for(const auto& e:traffic.entities()) require(traffic.map().fits(e.footprint()),"Exit vehicle left mask");
        }
        require(traffic.completed()==1,"Border exit never completed");
        Map diagonal(20,20,std::vector<Tile>(400,Tile::Road));
        diagonal.paintStroke({3.5,3.5},{16.5,16.5},Tile::Signal,.6); diagonal.rebuildFeatures();
        require(diagonal.features().size()==1 && diagonal.features()[0].line,"Diagonal line split");
        diagonal.eraseFeature(0); require(diagonal.cells(Tile::Signal).empty(),"Feature erase failed");
        std::cout<<"Map authoring checks passed\n";
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
