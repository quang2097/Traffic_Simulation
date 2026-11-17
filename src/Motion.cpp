#include "traffic/Simulation.hpp"
#include "traffic/Geometry.hpp"
#include <algorithm>
#include <cmath>

namespace traffic {
namespace {
constexpr double pi=3.141592653589793;
double cross(Vec2 a,Vec2 b) { return a.x*b.y-a.y*b.x; }
Vec2 unit(Vec2 v) { double n=length(v); return n>1e-10?v*(1/n):Vec2{}; }
double heading(Vec2 v) { return std::atan2(v.y,v.x); }
struct Builder {
    const Map& map; Footprint body; MotionPath path;
    bool append(Vec2 p,double h) {
        if(!path.points.empty() && length(p-path.points.back())<1e-10) return true;
        auto to=body; to.center=p; to.heading=h;
        double pad; auto sweep=geometry::swept(body,to,pad);
        if(!geometry::fits(map,sweep,pad)) return false;
        path.points.push_back(p); path.headings.push_back(h); body=to; return true;
    }
    void mark(std::size_t first) {
        if(path.points.size()<=first+1) return;
        PathManeuver turn; turn.begin=first; turn.end=path.points.size()-1;
        std::vector<Vec2> points;
        for(std::size_t i=first;i<=turn.end;++i) {
            auto pose=body; pose.center=path.points[i]; pose.heading=path.headings[i];
            auto box=corners(pose); points.insert(points.end(),box.begin(),box.end());
            if(i>first) turn.padding=std::max(turn.padding,.5*std::hypot(body.length,body.width)*
                (1-std::cos(angleDelta(path.headings[i-1],path.headings[i])/2)));
        }
        turn.sweep=geometry::convexHull(std::move(points)); path.turns.push_back(std::move(turn));
    }
    bool line(Vec2 end) {
        Vec2 delta=end-body.center; double distance=length(delta);
        if(distance<1e-9) return true;
        double h=heading(delta);
        if(std::abs(angleDelta(body.heading,h))>.002) return false;
        // Straight segments need only one sample; physics consumes any leftover
        // travel across boundaries, including all subsequent curve samples.
        return append(end,h);
    }
    bool connector(Vec2 end,double endHeading,double scale) {
        Vec2 start=body.center; double distance=length(end-start);
        if(distance<1e-7) return true;
        Vec2 forward{std::cos(body.heading),std::sin(body.heading)},out{std::cos(endHeading),std::sin(endHeading)};
        double handle=std::min(distance*.4,std::max(body.length,1.0))*scale;
        auto bezier=[&](Vec2 a,Vec2 b,Vec2 c,Vec2 d) {
            double controlLength=length(b-a)+length(c-b)+length(d-c);
            int steps=std::max(100,static_cast<int>(std::ceil(controlLength/.025)));
            for(int i=1;i<=steps;++i) {
                double t=static_cast<double>(i)/steps,s=1-t;
                Vec2 p=a*(s*s*s)+b*(3*s*s*t)+c*(3*s*t*t)+d*(t*t*t);
                Vec2 tangent=(b-a)*(3*s*s)+(c-b)*(6*s*t)+(d-c)*(3*t*t);
                if(length(tangent)<1e-8 || !append(p,heading(tangent))) return false;
            }
            return true;
        };
        std::size_t begin=path.points.size()-1;
        Vec2 direction=unit(end-start);
        if(std::abs(cross(forward,direction))<.05 && forward.x*direction.x+forward.y*direction.y<-.5) {
            // A U-turn requires room for translation; avoid a collinear cubic
            // with a zero derivative (which would recreate an in-place pivot).
            Vec2 side{-forward.y,forward.x};
            Vec2 middle=(start+end)*.5+side*std::max(handle,distance*.25);
            if(!bezier(start,start+forward*handle,middle-side*handle,middle) ||
               !bezier(middle,middle+side*handle,end-out*handle,end)) return false;
        } else if(!bezier(start,start+forward*handle,end-out*handle,end)) return false;
        mark(begin); return true;
    }
};
}
MotionPath curvePath(const Map& map,const Footprint& pose,const std::vector<Vec2>& input,double initialConnectorLimit,const std::vector<double>& smoothing) {
    if(input.empty() || !map.fits(pose)) return {};
    std::vector<Vec2> points{pose.center};
    for(Vec2 p:input) if(length(p-points.back())>1e-7) points.push_back(p);
    for(std::size_t i=1;i+1<points.size();) {
        if(std::abs(angleDelta(heading(points[i]-points[i-1]),heading(points[i+1]-points[i])))<.002)
            points.erase(points.begin()+i);
        else ++i;
    }
    Builder builder{map,pose,{}};
    builder.path.points.push_back(pose.center); builder.path.headings.push_back(pose.heading);
    if(points.size()==1) return builder.path;
    struct Fillet { Vec2 entry,exit,center; double angle=0,radius=0,startAngle=0; };
    std::vector<Fillet> fillets(points.size());
    for(std::size_t i=1;i+1<points.size();++i) {
        Vec2 incoming=unit(points[i]-points[i-1]),outgoing=unit(points[i+1]-points[i]);
        double angle=angleDelta(heading(incoming),heading(outgoing));
        if(std::abs(angle)<.002 || std::abs(angle)>pi-.05) return {};
        double tangent=std::tan(std::abs(angle)/2);
        double smooth=1;
        for(std::size_t j=0;j<input.size() && j<smoothing.size();++j) if(length(points[i]-input[j])<1e-6) { smooth=std::clamp(smoothing[j],.2,4.0); break; }
        double trim=std::min(std::max(1.0,pose.length*.8)*smooth*tangent,
            .45*std::min(length(points[i]-points[i-1]),length(points[i+1]-points[i])));
        bool fit=false;
        for(int attempt=0;attempt<9 && trim>1e-5;++attempt,trim*=.5) {
            double radius=trim/tangent,sign=angle>0?1.0:-1.0;
            Vec2 entry=points[i]-incoming*trim,exit=points[i]+outgoing*trim;
            Vec2 normal{-incoming.y*sign,incoming.x*sign};
            Vec2 center=entry+normal*radius;
            double startAngle=heading(entry-center);
            auto previous=pose; previous.center=entry; previous.heading=heading(incoming);
            int count=std::max(2,static_cast<int>(std::ceil(std::max(std::abs(angle)*radius/.05,std::abs(angle)/.01745329252))));
            fit=map.fits(previous);
            for(int sample=1;fit && sample<=count;++sample) {
                double t=static_cast<double>(sample)/count;
                auto next=pose; double a=startAngle+angle*t;
                next.center=center+Vec2{std::cos(a),std::sin(a)}*radius;
                next.heading=heading(incoming)+angle*t;
                double pad; auto sweep=geometry::swept(previous,next,pad);
                fit=geometry::fits(map,sweep,pad); previous=next;
            }
            if(fit) { fillets[i]={entry,exit,center,angle,radius,startAngle}; break; }
        }
        if(!fit) return {};
    }
    for(std::size_t i=1;i<points.size();++i) {
        Vec2 end=i+1<points.size()?fillets[i].entry:points[i];
        double targetHeading=heading(end-builder.body.center);
        if(std::abs(angleDelta(builder.body.heading,targetHeading))>.002) {
            Vec2 connectorEnd=end;
            if(i==1 && initialConnectorLimit>0 && length(end-builder.body.center)>initialConnectorLimit)
                connectorEnd=builder.body.center+unit(end-builder.body.center)*initialConnectorLimit;
            bool connected=false;
            for(double scale:{1.0,.5,.25,.125,.0625}) {
                auto saved=builder.path; auto savedBody=builder.body;
                if(builder.connector(connectorEnd,targetHeading,scale)) { connected=true; break; }
                builder.path=std::move(saved); builder.body=savedBody;
            }
            if(!connected) return {};
            if(length(end-builder.body.center)>1e-8 && !builder.line(end)) return {};
        } else if(!builder.line(end)) return {};
        if(i+1<points.size()) {
            auto f=fillets[i]; std::size_t begin=builder.path.points.size()-1;
            double incoming=builder.body.heading;
            int count=std::max(2,static_cast<int>(std::ceil(std::max(std::abs(f.angle)*f.radius/.05,std::abs(f.angle)/.01745329252))));
            for(int sample=1;sample<=count;++sample) {
                double t=static_cast<double>(sample)/count,a=f.startAngle+f.angle*t;
                Vec2 p=sample==count?f.exit:f.center+Vec2{std::cos(a),std::sin(a)}*f.radius;
                if(!builder.append(p,incoming+f.angle*t)) return {};
            }
            builder.mark(begin);
        }
    }
    return builder.path;
}
}
