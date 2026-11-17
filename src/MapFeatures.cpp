#include "traffic/Simulation.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace traffic {
void Map::rebuildFeatures() {
    features_.clear(); destinationLabels_.assign(tiles.size(),-1);
    std::vector<unsigned char> visited(tiles.size(),0);
    std::vector<Cell> pending;
    for(int y=0;y<height;++y) for(int x=0;x<width;++x) {
        Cell start{x,y}; int i=index(start); Tile type=tiles[i];
        if(visited[i] || type==Tile::Wall || type==Tile::Road) continue;
        MapFeature feature; feature.type=type;
        pending={start}; visited[i]=1;
        for(std::size_t p=0;p<pending.size();++p) {
            Cell cell=pending[p]; feature.pixels.push_back(cell); feature.center=feature.center+center(cell);
            for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx) {
                Cell next{cell.x+dx,cell.y+dy};
                if(contains(next) && !visited[index(next)] && tiles[index(next)]==type) {
                    visited[index(next)]=1; pending.push_back(next);
                }
            }
        }
        feature.center=feature.center*(1.0/feature.pixels.size());
        double xx=0,xy=0,yy=0;
        for(Cell cell:feature.pixels) { Vec2 d=center(cell)-feature.center; xx+=d.x*d.x; xy+=d.x*d.y; yy+=d.y*d.y; }
        double angle=.5*std::atan2(2*xy,xx-yy);
        Vec2 axis{std::cos(angle),std::sin(angle)},side{-axis.y,axis.x};
        double low=std::numeric_limits<double>::infinity(),high=-low,minorLow=low,minorHigh=-low;
        for(Cell cell:feature.pixels) {
            Vec2 d=center(cell)-feature.center;
            double u=d.x*axis.x+d.y*axis.y,v=d.x*side.x+d.y*side.y;
            low=std::min(low,u); high=std::max(high,u); minorLow=std::min(minorLow,v); minorHigh=std::max(minorHigh,v);
        }
        feature.line=feature.pixels.size()>=2 && high-low+metersPerPixel>=2*(minorHigh-minorLow+metersPerPixel)-1e-8;
        feature.a=feature.center+axis*(low-metersPerPixel*.5);
        feature.b=feature.center+axis*(high+metersPerPixel*.5);
        auto clampPoint=[&](Vec2 p) { return Vec2{std::clamp(p.x,1e-6,width*metersPerPixel-1e-6),std::clamp(p.y,1e-6,height*metersPerPixel-1e-6)}; };
        feature.a=clampPoint(feature.a); feature.b=clampPoint(feature.b);
        int label=static_cast<int>(features_.size());
        if(type==Tile::Destination) for(Cell cell:feature.pixels) destinationLabels_[index(cell)]=label;
        features_.push_back(std::move(feature));
    }
}
int Map::destinationAt(Vec2 p) const {
    Cell cell=pixel(p);
    if(!contains(cell) || destinationLabels_.size()!=tiles.size()) return -1;
    return destinationLabels_[index(cell)];
}
bool Map::touchesDestination(const Footprint& box,int feature) const {
    if(feature<0 || feature>=static_cast<int>(features_.size())) return false;
    auto polygon=corners(box);
    double minX=polygon[0].x,maxX=minX,minY=polygon[0].y,maxY=minY;
    for(Vec2 p:polygon) { minX=std::min(minX,p.x); maxX=std::max(maxX,p.x); minY=std::min(minY,p.y); maxY=std::max(maxY,p.y); }
    int left=std::max(0,static_cast<int>(std::floor(minX/metersPerPixel))),right=std::min(width-1,static_cast<int>(std::floor(maxX/metersPerPixel)));
    int top=std::max(0,static_cast<int>(std::floor(minY/metersPerPixel))),bottom=std::min(height-1,static_cast<int>(std::floor(maxY/metersPerPixel)));
    for(int y=top;y<=bottom;++y) for(int x=left;x<=right;++x) if(destinationLabels_[index({x,y})]==feature) {
        double s=metersPerPixel;
        geometry::Polygon pixelBox{{x*s,y*s},{(x+1)*s,y*s},{(x+1)*s,(y+1)*s},{x*s,(y+1)*s}};
        if(geometry::distance(polygon,pixelBox)<=1e-8) return true;
    }
    return false;
}
void Map::paintStroke(Vec2 from,Vec2 to,Tile type,double widthMeters) {
    if(!std::isfinite(from.x) || !std::isfinite(from.y) || !std::isfinite(to.x) || !std::isfinite(to.y) ||
       !std::isfinite(widthMeters) || widthMeters<=0) throw std::invalid_argument("Invalid map stroke");
    double radius=std::max(widthMeters*.5,metersPerPixel*.5);
    int left=std::max(0,static_cast<int>(std::floor((std::min(from.x,to.x)-radius)/metersPerPixel)));
    int right=std::min(width-1,static_cast<int>(std::ceil((std::max(from.x,to.x)+radius)/metersPerPixel)));
    int top=std::max(0,static_cast<int>(std::floor((std::min(from.y,to.y)-radius)/metersPerPixel)));
    int bottom=std::min(height-1,static_cast<int>(std::ceil((std::max(from.y,to.y)+radius)/metersPerPixel)));
    Vec2 delta=to-from; double norm=delta.x*delta.x+delta.y*delta.y;
    for(int y=top;y<=bottom;++y) for(int x=left;x<=right;++x) {
        Vec2 p=center({x,y}); double t=norm>0?std::clamp(((p.x-from.x)*delta.x+(p.y-from.y)*delta.y)/norm,0.0,1.0):0;
        if(length(p-(from+delta*t))<=radius+1e-8) {
            if(type!=Tile::Road && type!=Tile::Wall && type!=Tile::Divider && !walkable({x,y})) continue;
            tiles[index({x,y})]=type;
        }
    }
    wallIntegral_.clear(); features_.clear(); destinationLabels_.clear();
}
void Map::eraseFeature(std::size_t feature) {
    if(feature>=features_.size()) throw std::invalid_argument("Invalid map feature");
    for(Cell cell:features_[feature].pixels) tiles[index(cell)]=Tile::Road;
    wallIntegral_.clear(); rebuildFeatures();
}
}
