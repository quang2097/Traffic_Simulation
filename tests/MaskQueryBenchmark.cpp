#include "traffic/Simulation.hpp"
#include "traffic/Geometry.hpp"
#include <random>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <cmath>
namespace traffic::geometry {
bool referenceFits(const Map& map,const Polygon& poly,double padding) {
    if(poly.empty()) return false;
    double minX=poly[0].x,maxX=minX,minY=poly[0].y,maxY=minY;
    for(Vec2 p:poly) { minX=std::min(minX,p.x); maxX=std::max(maxX,p.x); minY=std::min(minY,p.y); maxY=std::max(maxY,p.y); }
    minX-=padding; minY-=padding; maxX+=padding; maxY+=padding;
    double s=map.metersPerPixel;
    if(minX<0 || minY<0 || maxX>map.width*s || maxY>map.height*s) return false;
    if(!map.hasWalls(std::max(0,static_cast<int>(std::floor(minX/s))),std::max(0,static_cast<int>(std::floor(minY/s))),
        std::min(map.width-1,static_cast<int>(std::floor(maxX/s))),std::min(map.height-1,static_cast<int>(std::floor(maxY/s))))) return true;
    for(int y=std::max(0,static_cast<int>(std::floor(minY/s)));y<=std::min(map.height-1,static_cast<int>(std::floor(maxY/s)));++y)
        for(int x=std::max(0,static_cast<int>(std::floor(minX/s)));x<=std::min(map.width-1,static_cast<int>(std::floor(maxX/s)));++x) {
            if(map.walkable({x,y})) continue;
            Polygon wall{{x*s,y*s},{(x+1)*s,y*s},{(x+1)*s,(y+1)*s},{x*s,(y+1)*s}};
            if(distance(poly,wall)<=padding+1e-9) return false;
        }
    return true;
}
}
int main(int argc,char**argv){if(argc<2){std::cerr<<"Usage: mask_query_benchmark MAP.png\n";return 1;}using namespace traffic;auto map=Map::fromPng(argv[1],.074);std::mt19937 rng(7);std::vector<geometry::Polygon> cases;while(cases.size()<2000){Vec2 p{std::uniform_real_distribution<double>(0,map.width*.074)(rng),std::uniform_real_distribution<double>(0,map.height*.074)(rng)};if(!map.walkable(map.pixel(p)))continue;cases.push_back(corners({p,std::uniform_real_distribution<double>(-3.14,3.14)(rng),std::uniform_real_distribution<double>(1.8,12)(rng),std::uniform_real_distribution<double>(.6,2.5)(rng)}));}std::vector<bool> expected;auto start=std::chrono::steady_clock::now();for(const auto& p:cases)expected.push_back(geometry::referenceFits(map,p,.001));double old=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();start=std::chrono::steady_clock::now();for(std::size_t i=0;i<cases.size();++i)if(geometry::fits(map,cases[i],.001)!=expected[i]){std::cerr<<"mask mismatch "<<i;return 1;}double now=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();std::cout<<"reference_s="<<old<<" optimized_s="<<now<<" speedup="<<old/now<<" exact_matches="<<cases.size()<<'\n';}
