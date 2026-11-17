#include "traffic/Demo.hpp"
#include <algorithm>
#include <cmath>
namespace traffic {
Map makeJunctionDemoMap() {
    Map map(130,130,std::vector<Tile>(16900,Tile::Wall));
    for(int y=0;y<130;++y) for(int x=0;x<130;++x) {
        auto p=map.center({x,y}); auto d=p-Vec2{65,65};
        if((std::abs(d.x)<8 && p.y>5 && p.y<125) || (std::abs(d.y)<8 && p.x>5 && p.x<125) || length(d)<28) map.tiles[map.index({x,y})]=Tile::Road;
    }
    map.rebuildFeatures(); return map;
}
NodeGraph makeJunctionDemoGraph() {
    NodeGraph g; g.explicitDestinations=true;
    for(Vec2 p:std::vector<Vec2>{{65,65},{10,65},{120,65},{65,10},{65,120}}) g.addNode(p);
    g.nodes[0].openRadius=24;
    for(std::size_t n=1;n<g.nodes.size();++n) { g.nodes[n].spawn=true; g.nodes[n].rate=.5; g.nodes[n].destination=true; }
    for(auto pair:std::vector<std::pair<int,int>>{{1,0},{0,2},{3,0},{0,4}}) {
        g.connect(pair.first,pair.second,5,8); auto& road=g.connections.back(); road.roadType=2; road.laneCount=1;
    }
    return g;
}
std::vector<DirectedLane> makeDemoLanes() {
    DirectedLane east,west;
    for(int i=0;i<=59;++i) {
        double x=8+i,y=25+6.5*std::sin((2*x-15)*.04),slope=.52*std::cos((2*x-15)*.04),norm=std::sqrt(1+slope*slope);
        east.points.push_back({x-1.9*slope/norm,y+1.9/norm}); west.points.push_back({x+1.9*slope/norm,y-1.9/norm});
    }
    std::reverse(west.points.begin(),west.points.end()); return {east,west};
}
Map makeDemoMap() {
    Map m(150,100,std::vector<Tile>(150*100,Tile::Wall),.5);
    for(int y=3;y<97;++y) for(int x=3;x<147;++x) {
        double center=50+13*std::sin((x-15)*.04);
        bool main=std::abs(y-center)<15;
        bool branch=(x>54 && x<82 && y>14 && y<86);
        bool loop=(y>14 && y<38 && x>30 && x<130) || (x>104 && x<133 && y>14 && y<76);
        if(main || branch || loop) m.tiles[m.index({x,y})]=Tile::Road;
    }
    for(int y=46;y<=54;y+=2) {
        m.tiles[m.index({16,y})]=Tile::Spawn;
        int destinationY=static_cast<int>(50+13*std::sin((133-15)*.04))+(y-50);
        m.tiles[m.index({133,destinationY})]=Tile::Destination;
    }
    m.paintStroke({30,22},{30,38},Tile::Signal,.5); m.rebuildFeatures(); return m;
}
}
