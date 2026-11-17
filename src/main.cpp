#include "traffic/Simulation.hpp"
#include <iostream>
#include <stdexcept>

int main(int argc,char** argv) {
    try {
        using namespace traffic;
        if(argc==3 && std::string(argv[1])=="--annotated-example") {
            Map example(120,60,std::vector<Tile>(7200,Tile::Wall),.5);
            for(int y=8;y<=51;++y) for(int x=4;x<=115;++x) example.tiles[example.index({x,y})]=Tile::Road;
            example.paintStroke({10.25,9.75},{10.25,9.75},Tile::Spawn,.3);
            example.paintStroke({10.25,20.75},{10.25,20.75},Tile::Spawn,.3);
            example.paintStroke({56.25,4.25},{56.25,25.75},Tile::Destination,.3);
            example.paintStroke({35.25,4.25},{35.25,25.75},Tile::Signal,.3);
            example.paintStroke({15.25,15.25},{50.25,15.25},Tile::Divider,.3);
            example.rebuildFeatures(); example.savePng(argv[2]);
            std::cout<<"Saved annotated example: "<<argv[2]<<'\n'; return 0;
        }
        if(argc==4 && std::string(argv[1])=="--convert-legacy") {
            auto legacy=Map::fromPng(argv[2],.5,MapPalette::Legacy);
            legacy.savePng(argv[3]);
            std::cout<<"Saved traffic-palette PNG: "<<argv[3]<<'\n'; return 0;
        }
        Map map(70,30,std::vector<Tile>(2100,Tile::Road));
        map.tiles[map.index({8,15})]=Tile::Spawn;
        map.tiles[map.index({61,15})]=Tile::Destination;
        if (argc>1) map=Map::fromPng(argv[1],argc>3?std::stod(argv[3]):.5);
        int ticks=argc>2 ? std::stoi(argv[2]) : 100;
        if (ticks<0) throw std::invalid_argument("Ticks must be nonnegative");
        Simulation simulation(std::move(map),42);
        int spawned=0;
        for (int i=0;i<ticks;++i) {
            if (i%3==0 && simulation.spawnRandom()) ++spawned;
            simulation.tick();
        }
        const auto& grid=simulation.map();
        for (int y=0;y<grid.height;++y) {
            for (int x=0;x<grid.width;++x) {
                auto tile=grid.tiles[grid.index({x,y})];
                char c=tile==Tile::Wall?'#':tile==Tile::Spawn?'S':tile==Tile::Destination?'D':'.';
                for (const auto& e:simulation.entities()) if (grid.pixel(e.center)==Cell{x,y}) c='o';
                std::cout<<c;
            }
            std::cout<<'\n';
        }
        std::cout<<"Ticks: "<<simulation.ticks()<<" Spawned: "<<spawned
                 <<" Completed: "<<simulation.completed()<<" Active: "<<simulation.entities().size()<<'\n';
    } catch (const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
