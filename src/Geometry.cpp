#include "traffic/Geometry.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace traffic {
Vec2 operator+(Vec2 a,Vec2 b) { return {a.x+b.x,a.y+b.y}; }
Vec2 operator-(Vec2 a,Vec2 b) { return {a.x-b.x,a.y-b.y}; }
Vec2 operator*(Vec2 a,double s) { return {a.x*s,a.y*s}; }
double length(Vec2 a) { return std::hypot(a.x,a.y); }
double angleDelta(double from,double to) { return std::remainder(to-from,2*3.141592653589793); }
std::vector<Vec2> corners(const Footprint& b) {
    Vec2 forward{std::cos(b.heading)*b.length/2,std::sin(b.heading)*b.length/2};
    Vec2 side{-std::sin(b.heading)*b.width/2,std::cos(b.heading)*b.width/2};
    return {b.center+forward+side,b.center-forward+side,b.center-forward-side,b.center+forward-side};
}
double clearance(const Footprint& a,const Footprint& b) { return geometry::distance(corners(a),corners(b)); }
bool Map::fits(const Footprint& b) const { return geometry::fits(*this,corners(b)); }
namespace geometry {
namespace {
double dot(Vec2 a,Vec2 b) { return a.x*b.x+a.y*b.y; }
double cross(Vec2 a,Vec2 b) { return a.x*b.y-a.y*b.x; }
double pointSegment(Vec2 p,Vec2 a,Vec2 b) {
    Vec2 d=b-a;
    double t=dot(d,d)>0?std::clamp(dot(p-a,d)/dot(d,d),0.0,1.0):0;
    return length(p-(a+d*t));
}
bool separated(const Polygon& a,const Polygon& b) {
    for(std::size_t i=0;i<a.size();++i) {
        Vec2 edge=a[(i+1)%a.size()]-a[i],axis{-edge.y,edge.x};
        double loA=std::numeric_limits<double>::infinity(),hiA=-loA,loB=loA,hiB=-loA;
        for(Vec2 p:a) { double v=dot(p,axis); loA=std::min(loA,v); hiA=std::max(hiA,v); }
        for(Vec2 p:b) { double v=dot(p,axis); loB=std::min(loB,v); hiB=std::max(hiB,v); }
        if(hiA<loB-1e-10 || hiB<loA-1e-10) return true;
    }
    return false;
}
Polygon hull(Polygon points) {
    std::sort(points.begin(),points.end(),[](Vec2 a,Vec2 b) { return a.x<b.x || (a.x==b.x && a.y<b.y); });
    points.erase(std::unique(points.begin(),points.end(),[](Vec2 a,Vec2 b){return length(a-b)<1e-10;}),points.end());
    if(points.size()<3) return points;
    Polygon result;
    for(Vec2 p:points) {
        while(result.size()>1 && cross(result.back()-result[result.size()-2],p-result.back())<=0) result.pop_back();
        result.push_back(p);
    }
    std::size_t lower=result.size();
    for(int i=static_cast<int>(points.size())-2;i>=0;--i) {
        Vec2 p=points[i];
        while(result.size()>lower && cross(result.back()-result[result.size()-2],p-result.back())<=0) result.pop_back();
        result.push_back(p);
    }
    result.pop_back(); return result;
}
}
double distance(const Polygon& a,const Polygon& b) {
    if(a.empty() || b.empty()) return std::numeric_limits<double>::infinity();
    if(!separated(a,b) && !separated(b,a)) return 0;
    double result=std::numeric_limits<double>::infinity();
    for(std::size_t i=0;i<a.size();++i) for(std::size_t j=0;j<b.size();++j) {
        result=std::min(result,pointSegment(a[i],b[j],b[(j+1)%b.size()]));
        result=std::min(result,pointSegment(b[j],a[i],a[(i+1)%a.size()]));
    }
    return result;
}
Polygon convexHull(Polygon points) { return hull(std::move(points)); }
bool closerThan(const Polygon& a,const Polygon& b,double clearance) {
    if(a.empty() || b.empty()) return false;
    auto bounds=[](const Polygon& p) {
        std::array<double,4> result{p[0].x,p[0].y,p[0].x,p[0].y};
        for(Vec2 v:p) { result[0]=std::min(result[0],v.x); result[1]=std::min(result[1],v.y);
            result[2]=std::max(result[2],v.x); result[3]=std::max(result[3],v.y); }
        return result;
    };
    auto x=bounds(a),y=bounds(b);
    double dx=std::max({0.0,x[0]-y[2],y[0]-x[2]}),dy=std::max({0.0,x[1]-y[3],y[1]-x[3]});
    if(dx*dx+dy*dy>=clearance*clearance) return false;
    // Near contact, tiny body steps can make the iterative disk test report a
    // false overlap. Exact edge distances are cheap for two small footprints.
    if(a.size()<=12 && b.size()<=12) return distance(a,b)<clearance;
    // GJK on (A - B) expanded by a clearance disk. Curved reservations can
    // have hundreds of hull vertices: SAT + all edge pairs is quadratic,
    // while support queries here scan each hull only a handful of times.
    auto dot=[](Vec2 p,Vec2 q) { return p.x*q.x+p.y*q.y; };
    auto triple=[&](Vec2 p,Vec2 q,Vec2 r) { return q*dot(p,r)-p*dot(q,r); };
    auto support=[&](Vec2 direction) {
        Vec2 pa=a[0],pb=b[0];
        for(Vec2 p:a) if(dot(p,direction)>dot(pa,direction)) pa=p;
        for(Vec2 p:b) if(dot(p,direction)<dot(pb,direction)) pb=p;
        double n=length(direction);
        return pa-pb+(n>0?direction*(clearance/n):Vec2{});
    };
    Vec2 direction{1,0};
    std::vector<Vec2> simplex{support(direction)};
    direction=simplex[0]*-1;
    for(int iteration=0;iteration<64;++iteration) {
        double norm=length(direction);
        if(norm<1e-12) return true;
        direction=direction*(1/norm);
        Vec2 point=support(direction);
        if(dot(point,direction)<0) return false;
        simplex.push_back(point);
        Vec2 ao=point*-1,ab=simplex[simplex.size()-2]-point;
        if(simplex.size()==2) {
            if(dot(ab,ao)>0) direction=triple(ab,ao,ab);
            else { simplex={point}; direction=ao; }
        } else {
            Vec2 ac=simplex[0]-point;
            Vec2 outsideAB=triple(ac,ab,ab),outsideAC=triple(ab,ac,ac);
            if(dot(outsideAB,ao)>0) { simplex={simplex[1],point}; direction=outsideAB; }
            else if(dot(outsideAC,ao)>0) { simplex={simplex[0],point}; direction=outsideAC; }
            else return true;
        }
    }
    return distance(a,b)<clearance; // Exact fallback for numerical degeneracy.
}
bool violatesClearance(const Polygon& a,const Polygon& b,double clearance) {
    if(a.empty() || b.empty()) return false;
    constexpr double tolerance=1e-7;
    if(clearance>tolerance) return closerThan(a,b,clearance-tolerance);
    // Zero preferred gaps still forbid overlapping bodies. The usual distance
    // predicate cannot distinguish touching from overlapping at zero distance.
    for(const auto* poly:{&a,&b}) for(std::size_t i=0;i<poly->size();++i) {
        Vec2 edge=(*poly)[(i+1)%poly->size()]-(*poly)[i]; double norm=length(edge);
        if(norm<1e-12) continue;
        Vec2 axis{-edge.y/norm,edge.x/norm};
        double amin=1e300,amax=-1e300,bmin=1e300,bmax=-1e300;
        for(Vec2 p:a) { double value=p.x*axis.x+p.y*axis.y; amin=std::min(amin,value); amax=std::max(amax,value); }
        for(Vec2 p:b) { double value=p.x*axis.x+p.y*axis.y; bmin=std::min(bmin,value); bmax=std::max(bmax,value); }
        if(std::min(amax,bmax)-std::max(amin,bmin)<=1e-9) return false;
    }
    return true;
}
namespace {
bool clearMotion(const Footprint& from,const Footprint& to,const Polygon& obstacle,double required,int depth) {
    double pad; auto polygon=swept(from,to,pad);
    if(!violatesClearance(polygon,obstacle,required+pad)) return true;
    if(violatesClearance(corners(from),obstacle,required) || violatesClearance(corners(to),obstacle,required) || depth>=14) return false;
    Footprint middle{from.center+(to.center-from.center)*.5,from.heading+angleDelta(from.heading,to.heading)*.5,from.length,from.width};
    return clearMotion(from,middle,obstacle,required,depth+1) && clearMotion(middle,to,obstacle,required,depth+1);
}
}
bool motionClearance(const Footprint& from,const Footprint& to,const Footprint& obstacle,double required,bool allowSeparating) {
    auto body=corners(obstacle),start=corners(from),end=corners(to);
    if(allowSeparating && violatesClearance(start,body,required)) {
        if(violatesClearance(start,body,0)) return false;
        double initial=distance(start,body),final=distance(end,body);
        if(final<=initial+1e-9) return false;
        required=initial; // Permit only an escape that never worsens the existing gap.
    }
    return clearMotion(from,to,body,required,0);
}
Polygon swept(const Footprint& from,const Footprint& to,double& padding) {
    double angle=angleDelta(from.heading,to.heading);
    int steps=std::max(1,static_cast<int>(std::ceil(std::abs(angle)/0.08)));
    Polygon points;
    for(int i=0;i<=steps;++i) {
        double t=static_cast<double>(i)/steps;
        Footprint pose{from.center+(to.center-from.center)*t,from.heading+angle*t,from.length,from.width};
        auto box=corners(pose); points.insert(points.end(),box.begin(),box.end());
    }
    padding=0.5*std::hypot(from.length,from.width)*(1-std::cos(angle/(2*steps)));
    return hull(std::move(points));
}
bool fits(const Map& map,const Polygon& poly,double padding) {
    if(poly.empty()) return false;
    double minX=poly[0].x,maxX=minX,minY=poly[0].y,maxY=minY;
    for(Vec2 p:poly) { minX=std::min(minX,p.x); maxX=std::max(maxX,p.x); minY=std::min(minY,p.y); maxY=std::max(maxY,p.y); }
    minX-=padding; minY-=padding; maxX+=padding; maxY+=padding;
    double s=map.metersPerPixel;
    if(minX<0 || minY<0 || maxX>map.width*s || maxY>map.height*s) return false;
    if(!map.hasWalls(std::max(0,static_cast<int>(std::floor(minX/s))),std::max(0,static_cast<int>(std::floor(minY/s))),
        std::min(map.width-1,static_cast<int>(std::floor(maxX/s))),std::min(map.height-1,static_cast<int>(std::floor(maxY/s))))) return true;
    // Empty rectangles and wall regions outside the swept polygon can be
    // skipped as blocks. At a leaf this uses the original exact pixel test.
    auto clearRegion=[&](auto&& self,int x0,int y0,int x1,int y1)->bool {
        if(!map.hasWalls(x0,y0,x1,y1)) return true;
        Polygon region{{x0*s,y0*s},{(x1+1)*s,y0*s},{(x1+1)*s,(y1+1)*s},{x0*s,(y1+1)*s}};
        double separation=distance(poly,region);
        if(x0==x1 && y0==y1) return separation>padding+1e-9;
        if(separation>padding+1e-8) return true;
        if(x1-x0>=y1-y0) {int middle=(x0+x1)/2;return self(self,x0,y0,middle,y1) && self(self,middle+1,y0,x1,y1);}
        int middle=(y0+y1)/2;return self(self,x0,y0,x1,middle) && self(self,x0,middle+1,x1,y1);
    };
    return clearRegion(clearRegion,std::max(0,static_cast<int>(std::floor(minX/s))),std::max(0,static_cast<int>(std::floor(minY/s))),
        std::min(map.width-1,static_cast<int>(std::floor(maxX/s))),std::min(map.height-1,static_cast<int>(std::floor(maxY/s))));
}
bool intersectsLine(const Polygon& polygon,Vec2 a,Vec2 b) { return distance(polygon,{a,b})<=1e-8; }
}
}
