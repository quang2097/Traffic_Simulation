#include "traffic/SumoBackend.hpp"
#include "traffic/TraCI.hpp"
#include "traffic/Geometry.hpp"
#include <winsock2.h>
#include <windows.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <queue>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace traffic {
namespace {
namespace fs=std::filesystem;
constexpr double pi=3.14159265358979323846,stepLength=.1;
using traci::Buffer;
std::string number(double n){std::ostringstream s;s<<std::setprecision(15)<<n;return s.str();}
std::string xml(const std::string& s){std::string r;for(char c:s){if(c=='&')r+="&amp;";else if(c=='\"')r+="&quot;";else if(c=='<')r+="&lt;";else r+=c;}return r;}
std::wstring quote(const std::wstring& s){std::wstring r=L"\"";int slashes=0;for(auto c:s){if(c==L'\\'){++slashes;continue;}if(c==L'\"'){r.append(slashes*2+1,L'\\');r+=c;}else{r.append(slashes,L'\\');r+=c;}slashes=0;}r.append(slashes*2,L'\\');return r+L'\"';}
struct Process {
    HANDLE handle=nullptr;
    ~Process(){stop();}
    void stop(){if(handle){if(WaitForSingleObject(handle,2000)==WAIT_TIMEOUT)TerminateProcess(handle,1);CloseHandle(handle);handle=nullptr;}}
    void start(const fs::path& exe,const std::vector<std::string>& args,const fs::path& log){
        SECURITY_ATTRIBUTES sa{sizeof(sa),nullptr,TRUE};HANDLE out=CreateFileW(log.wstring().c_str(),GENERIC_WRITE,FILE_SHARE_READ|FILE_SHARE_WRITE,&sa,CREATE_ALWAYS,FILE_ATTRIBUTE_NORMAL,nullptr);
        if(out==INVALID_HANDLE_VALUE)throw std::runtime_error("Cannot write SUMO log: "+log.string());
        std::wstring line=quote(exe.wstring());for(auto& a:args)line+=L" "+quote(fs::path(a).wstring());
        STARTUPINFOW si{};si.cb=sizeof(si);si.dwFlags=STARTF_USESTDHANDLES;si.hStdOutput=out;si.hStdError=out;si.hStdInput=GetStdHandle(STD_INPUT_HANDLE);
        PROCESS_INFORMATION process{};BOOL ok=CreateProcessW(exe.wstring().c_str(),line.data(),nullptr,nullptr,TRUE,CREATE_NO_WINDOW,nullptr,nullptr,&si,&process);CloseHandle(out);
        if(!ok)throw std::runtime_error("Cannot start "+exe.string()+" (Windows error "+std::to_string(GetLastError())+")");
        handle=process.hProcess;CloseHandle(process.hThread);
    }
    void wait(const fs::path& log){if(WaitForSingleObject(handle,30000)!=WAIT_OBJECT_0)throw std::runtime_error("SUMO network conversion timed out");DWORD code;GetExitCodeProcess(handle,&code);if(code){std::ifstream in(log);std::string text((std::istreambuf_iterator<char>(in)),{});throw std::runtime_error("SUMO network conversion failed: "+text);}stop();}
};
fs::path runtimePath(const std::string& supplied){
    std::vector<fs::path> roots;if(!supplied.empty())roots.emplace_back(supplied);
    if(auto home=std::getenv("SUMO_HOME"))roots.emplace_back(home);
    wchar_t exe[32768]{};GetModuleFileNameW(nullptr,exe,32768);auto folder=fs::path(exe).parent_path();
    roots.push_back(folder/"sumo-runtime/sumo");roots.push_back(folder/"sumo");roots.push_back(fs::path(TRAFFIC_SOURCE_ROOT)/"build/sumo-runtime/sumo");
    for(auto& p:roots)if(fs::exists(p/"bin/sumo.exe") && fs::exists(p/"bin/netconvert.exe"))return fs::absolute(p);
    throw std::runtime_error("SUMO runtime missing. Set SUMO_HOME or run scripts/install-sumo.ps1. No legacy driving fallback is used.");
}
int freePort(){SOCKET s=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);if(s==INVALID_SOCKET)throw std::runtime_error("Cannot reserve TraCI port");sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);if(bind(s,reinterpret_cast<sockaddr*>(&a),sizeof(a))){closesocket(s);throw std::runtime_error("Cannot bind TraCI port");}int n=sizeof(a);getsockname(s,reinterpret_cast<sockaddr*>(&a),&n);int p=ntohs(a.sin_port);closesocket(s);return p;}
std::map<std::string,std::string> attributes(const std::string& line){
    static const std::regex attr(R"re(([\w.:-]+)="([^"]*)")re");std::map<std::string,std::string> a;
    for(std::sregex_iterator i(line.begin(),line.end(),attr),end;i!=end;++i)a[(*i)[1]]=(*i)[2];return a;
}
Vec2 unit(Vec2 p){auto n=length(p);return n>1e-9?p*(1/n):Vec2{1,0};}
double polyLength(const std::vector<Vec2>& p){double n=0;for(std::size_t i=1;i<p.size();++i)n+=length(p[i]-p[i-1]);return n;}
bool crosses(const std::vector<Vec2>& p,Vec2 a,Vec2 b){for(std::size_t i=1;i<p.size();++i){auto d=p[i]-p[i-1],e=b-a;double den=d.x*e.y-d.y*e.x;if(std::abs(den)<1e-9)continue;auto v=a-p[i-1];double t=(v.x*e.y-v.y*e.x)/den,u=(v.x*d.y-v.y*d.x)/den;if(t>=0 && t<=1 && u>=0 && u<=1)return true;}return false;}
}

struct SumoBackend::Impl {
    Simulation& s;
    NodeGraph graph;
    traci::Connection client;
    Process process;
    fs::path project,runtime;
    struct Edge {std::string id;int from,to,road=0,direction=0,open=-1;std::vector<int> compiled;std::vector<std::string> laneIDs;std::vector<double> lengths;};
    std::vector<Edge> edges;
    std::map<std::string,int> edgeIndex,laneCompiled;
    std::map<std::string,std::vector<Vec2>> sumoShapes;
    std::vector<int> compiledEdge;
    struct Link {std::string tls;int index,in,out,inLane,outLane;std::vector<Vec2> shape;};
    std::vector<Link> links;
    std::map<std::string,int> stateSizes;
    std::map<std::string,std::string> lastStates;
    struct Trip {Entity e;std::vector<std::string> route;int stage=0,baseStage=0;bool free=false,ghost=false,pending=true,returning=false,entryRestricted=false,teleporting=false;int exit=-1,exitLane=0;std::string applied;std::map<int,int> freeStopLanes;};
    std::map<std::uint64_t,Trip> trips;
    std::map<std::string,std::vector<std::string>> routeCache;
    std::array<bool,vehicleKindCount> sampleTypes{};
    double accumulator=0;
    std::uint64_t collisionCount=0,teleportCount=0,handoffCount=0;
    std::string message="SUMO 1.28 | Krauss + SL2015 | native open junctions";
    unsigned seed;
    Impl(Simulation& simulation,const NodeGraph& g,unsigned seedValue,const std::string& home,const std::string& output):s(simulation),graph(g),runtime(runtimePath(home)),seed(seedValue){
        if(graph.connections.empty())throw std::runtime_error("Draw and connect a lane network before starting SUMO");
        project=output.empty()?fs::path(TRAFFIC_SOURCE_ROOT)/"build/sumo-live"/(std::to_string(GetCurrentProcessId())+"-"+std::to_string(GetTickCount64())):fs::absolute(output);
        fs::create_directories(project);exportNetwork();
        int port=freePort();
        process.start(runtime/"bin/sumo.exe",{"--net-file",(project/"network.net.xml").string(),"--route-files",(project/"types.rou.xml").string(),"--remote-port",std::to_string(port),"--step-length","0.1","--lateral-resolution","0.25","--seed",std::to_string(seed),"--no-step-log","true","--duration-log.disable","true","--collision.check-junctions","true","--collision.action","teleport","--time-to-teleport","300","--xml-validation","never"},project/"sumo.log");
        bool connected=false;for(int i=0;i<100;++i){try{client.connect(port);connected=true;break;}catch(...){if(WaitForSingleObject(process.handle,0)==WAIT_OBJECT_0)break;std::this_thread::sleep_for(std::chrono::milliseconds(30));}}
        if(!connected)throw std::runtime_error("Cannot connect to SUMO; see "+(project/"sumo.log").string());
        auto version=client.commands({{0,{}}}).front();version.commandLength();version.expect(0);version.integer();
        message=version.string()+" | Krauss + SL2015 | native open junctions";
        importGeometry();parseLinks();
        s.entities_.clear();s.seconds_=0;
    }
    ~Impl(){client.close();}
    void exportNetwork(){
        compiledEdge.assign(s.lanes_.size(),-1);
        auto junctionAt=[&](int n){return graph.junctionAt(n);};
        for(std::size_t r=0;r<graph.connections.size();++r){auto& road=graph.connections[r];
            for(int dir=0;dir<(road.roadType==2?2:1);++dir){Edge e;e.road=static_cast<int>(r);e.direction=dir;e.id="r"+std::to_string(r)+(dir?"b":"f");e.from=junctionAt(dir?road.to:road.from);e.to=junctionAt(dir?road.from:road.to);
                if(e.from==e.to)continue; // Entirely inside one native open area.
                for(std::size_t l=0;l<s.lanes_.size();++l)if(s.lanes_[l].roadGroup==static_cast<int>(r) && s.lanes_[l].travelDirection==dir && s.lanes_[l].openJunction<0 && s.lanes_[l].internalJunction<0)e.compiled.push_back(static_cast<int>(l));
                if(e.compiled.empty()) { // Legacy single-direction editor roads.
                    if(road.roadType==0 && r<s.lanes_.size())e.compiled.push_back(static_cast<int>(r));else continue;
                }
                // Native numbers lanes from the median outward; SUMO numbers from right inward.
                std::reverse(e.compiled.begin(),e.compiled.end());
                for(int l:e.compiled)compiledEdge[l]=static_cast<int>(edges.size());
                edgeIndex[e.id]=static_cast<int>(edges.size());edges.push_back(std::move(e));
            }
        }
        std::ofstream nodes(project/"nodes.nod.xml"),roads(project/"roads.edg.xml");nodes<<std::setprecision(15)<<"<nodes>\n";roads<<std::setprecision(15)<<"<edges>\n";
        for(std::size_t n=0;n<graph.nodes.size();++n){auto& node=graph.nodes[n];bool signal=node.signal;std::set<int> neighbors;
            for(auto& edge:edges){if(edge.from==static_cast<int>(n))neighbors.insert(edge.to);if(edge.to==static_cast<int>(n))neighbors.insert(edge.from);}
            signal=signal || neighbors.size()>1;
            for(auto& e:edges)if(e.to==static_cast<int>(n))for(int l:e.compiled)for(auto& light:s.lights_)if(crosses(s.lanes_[l].points,light.a,light.b) || length((light.a+light.b)*.5-s.lanes_[l].points.back())<std::max(15.,node.openRadius))signal=true;
            nodes<<"<node id=\"n"<<n<<"\" x=\""<<node.position.x<<"\" y=\""<<-node.position.y<<"\" type=\""<<(signal?"traffic_light":"priority")<<"\"";
            if(node.roadStation && node.openRadius<=0 && node.internalRadius<=0 && !node.allowUTurn)nodes<<" radius=\"0\"";
            else if(node.openRadius>0 || node.internalRadius>0)nodes<<" radius=\""<<std::max(node.openRadius,node.internalRadius)<<"\"";
            nodes<<"/>\n";
        }
        for(auto& e:edges){auto& road=graph.connections[e.road];
            auto shape=road.shape;if(shape.empty())shape={graph.nodes[road.from].position,graph.nodes[road.to].position};if(e.direction)std::reverse(shape.begin(),shape.end());
            roads<<"<edge id=\""<<e.id<<"\" from=\"n"<<e.from<<"\" to=\"n"<<e.to<<"\" numLanes=\""<<e.compiled.size()<<"\" speed=\""<<road.speed<<"\" spreadType=\""<<(road.roadType==2?"right":"center")<<"\" shape=\"";
            for(auto p:shape)roads<<p.x<<","<<-p.y<<" ";roads<<"\">\n";
            for(std::size_t l=0;l<e.compiled.size();++l){auto& lane=s.lanes_[e.compiled[l]];roads<<"<lane index=\""<<l<<"\" width=\""<<lane.width<<"\" allow=\"all\"/>\n";}
            roads<<"</edge>\n";
        }nodes<<"</nodes>\n";roads<<"</edges>\n";nodes.close();roads.close();
        // Authoritative turn permissions come from the authored connectivity, including U-turn openings.
        std::ofstream connections(project/"turns.con.xml");connections<<"<connections>\n";
        for(auto& in:edges)for(std::size_t il=0;il<in.compiled.size();++il){int source=in.compiled[il];
            std::set<int> targets;
            for(int next:s.lanes_[source].successors){if(compiledEdge[next]>=0)targets.insert(next);else for(int target:s.lanes_[next].successors)if(compiledEdge[target]>=0)targets.insert(target);}
            for(int target:targets){int out=compiledEdge[target];if(edges[out].from!=in.to)continue;auto& ls=edges[out].compiled;int ol=static_cast<int>(std::find(ls.begin(),ls.end(),target)-ls.begin());
                connections<<"<connection from=\""<<in.id<<"\" to=\""<<edges[out].id<<"\" fromLane=\""<<il<<"\" toLane=\""<<ol<<"\"/>\n";
            }
        }connections<<"</connections>\n";connections.close();
        Process convert;convert.start(runtime/"bin/netconvert.exe",{"--node-files",(project/"nodes.nod.xml").string(),"--edge-files",(project/"roads.edg.xml").string(),"--connection-files",(project/"turns.con.xml").string(),"--output-file",(project/"network.net.xml").string(),"--offset.disable-normalization","true","--no-turnarounds","true","--junctions.corner-detail","12","--junctions.internal-link-detail","12","--junctions.limit-turn-speed","1.8","--xml-validation","never"},project/"netconvert.log");convert.wait(project/"netconvert.log");
        std::ofstream types(project/"types.rou.xml");types<<"<routes>\n";
        const char* classes[]={"bicycle","motorcycle","passenger","truck","bus"};
        for(int k=0;k<vehicleKindCount;++k){auto p=s.profile(static_cast<VehicleKind>(k));types<<"<vType id=\"type"<<k<<"\" vClass=\""<<classes[k]<<"\" length=\""<<p.length.mean<<"\" width=\""<<p.width.mean<<"\" maxSpeed=\""<<p.motion.speedMean<<"\" accel=\""<<p.motion.accelerationMean<<"\" decel=\""<<p.deceleration<<"\" emergencyDecel=\""<<p.emergencyDeceleration<<"\" tau=\""<<std::max(.1,p.tau)<<"\" minGap=\""<<p.minGap<<"\" carFollowModel=\"Krauss\" laneChangeModel=\"SL2015\" sigma=\"0.5\" speedFactor=\"1\" minGapLat=\""<<p.lateralMinGap<<"\" lcStrategic=\"1\" lcCooperative=\"1\" lcSpeedGain=\"1\" lcKeepRight=\"1\" lcSublane=\"1\" lcPushy=\"0\" lcAssertive=\"1\"/>\n";}
        types<<"</routes>\n";
    }
    void importGeometry(){
        for(auto& e:edges){
            e.open=graph.nodes[e.to].openRadius>0?e.to:-1;
            for(int lane:e.compiled)for(int next:s.lanes_[lane].successors)if(s.lanes_[next].openJunction>=0)e.open=s.lanes_[next].openJunction;
            for(std::size_t l=0;l<e.compiled.size();++l){std::string id=e.id+"_"+std::to_string(l);e.laneIDs.push_back(id);laneCompiled[id]=e.compiled[l];
                auto values=client.get({{0xa3,0x4e,id,{}},{0xa3,0x44,id,{}}});auto& p=values[0].polygon;
                auto& lane=s.lanes_[e.compiled[l]];lane.points.clear();for(std::size_t i=0;i<p.size();i+=2)lane.points.push_back({p[i],-p[i+1]});
                sumoShapes[id]=lane.points;
                lane.pointWidths.assign(lane.points.size(),lane.width);e.lengths.push_back(values[1].value);
            }
        }
    }
    void parseLinks(){
        std::ifstream input(project/"network.net.xml");std::string line;std::vector<std::map<std::string,std::string>> rows;
        while(std::getline(input,line))if(line.find("<connection ")!=std::string::npos)rows.push_back(attributes(line));
        for(auto a:rows){if(!a.count("tl") || !edgeIndex.count(a["from"]) || !edgeIndex.count(a["to"]))continue;
            Link l;l.tls=a["tl"];l.index=std::stoi(a["linkIndex"]);l.in=edgeIndex.at(a["from"]);l.out=edgeIndex.at(a["to"]);l.inLane=std::stoi(a["fromLane"]);l.outLane=std::stoi(a["toLane"]);
            // A permission must be ahead of this stop line. An intersection
            // with the already-traversed incoming road cannot grant permission.
            l.shape={s.lanes_[edges[l.in].compiled[l.inLane]].points.back()};
            std::string via=a["via"];std::set<std::string> visited;
            while(!via.empty() && visited.insert(via).second){
                if(!sumoShapes.count(via)){auto polygon=client.get(0xa3,0x4e,via).polygon;auto& shape=sumoShapes[via];for(std::size_t k=0;k<polygon.size();k+=2)shape.push_back({polygon[k],-polygon[k+1]});}
                auto& shape=sumoShapes.at(via);l.shape.insert(l.shape.end(),shape.begin(),shape.end());
                auto split=via.rfind('_');std::string edge=via.substr(0,split),lane=via.substr(split+1),next;
                for(auto& row:rows)if(row.at("from")==edge && row.at("to")==a["to"] && row.at("fromLane")==lane){auto it=row.find("via");if(it!=row.end())next=it->second;break;}
                via=next;
            }
            auto& tail=s.lanes_[edges[l.out].compiled[l.outLane]].points;l.shape.insert(l.shape.end(),tail.begin(),tail.end());
            stateSizes[l.tls]=std::max(stateSizes[l.tls],l.index+1);links.push_back(std::move(l));
        }
        for(auto& [tls,size]:stateSizes){int node=std::stoi(tls.substr(1));if(!graph.nodes[node].signal){Buffer b;b.typedString("off");client.set(0xc2,0x23,tls,b);}}
    }
    std::vector<std::string> findRoute(int from,int to,int kind){
        auto key=edges[from].id+">"+edges[to].id+":"+std::to_string(kind);auto it=routeCache.find(key);if(it!=routeCache.end())return it->second;
        if(std::any_of(s.lanes_.begin(),s.lanes_.end(),[](const auto& lane){return lane.semanticTransfer;})){
            std::vector<double> cost(s.lanes_.size(),1e100);std::vector<int> parent(s.lanes_.size(),-1);std::priority_queue<std::pair<double,int>,std::vector<std::pair<double,int>>,std::greater<std::pair<double,int>>> queue;
            for(int lane:edges[from].compiled){cost[lane]=0;queue.push({0,lane});}int sink=-1;
            while(!queue.empty()){auto [distance,lane]=queue.top();queue.pop();if(distance!=cost[lane])continue;if(compiledEdge[lane]==to){sink=lane;break;}
                for(int next:s.lanes_[lane].successors){if(s.lanes_[next].width<s.profile(static_cast<VehicleKind>(kind)).width.mean+.05)continue;double weight=0;auto& points=s.lanes_[next].points;for(std::size_t i=1;i<points.size();++i)weight+=length(points[i]-points[i-1]);if(distance+weight<cost[next]){cost[next]=distance+weight;parent[next]=lane;queue.push({cost[next],next});}}}
            std::vector<int> chain;for(int lane=sink;lane>=0;lane=parent[lane])chain.push_back(lane);std::reverse(chain.begin(),chain.end());std::vector<std::string> route;
            for(int lane:chain)if(compiledEdge[lane]>=0 && (route.empty() || route.back()!=edges[compiledEdge[lane]].id))route.push_back(edges[compiledEdge[lane]].id);
            routeCache[key]=route;return route;
        }

        Buffer b;b.compound(7);b.typedString(edges[from].id);b.typedString(edges[to].id);b.typedString("type"+std::to_string(kind));b.typedReal(-1);b.typedInt(0);b.typedReal(0);b.typedReal(-1073741824.);
        auto result=client.get(0xab,0x86,"",b);auto& r=result.raw;if(result.type!=0x0f || r.integer()!=13)throw std::runtime_error("Unexpected SUMO route result");
        r.expect(0x09);r.integer();for(int i=0;i<3;++i){r.expect(0x0c);r.string();}r.expect(0x0e);auto route=r.strings();routeCache[key]=route;return route;
    }
    void setNumber(const std::string& id,int variable,double value){Buffer b;b.typedReal(value);client.set(0xc4,variable,id,b);}
    void setInteger(const std::string& id,int variable,int value){Buffer b;b.typedInt(value);client.set(0xc4,variable,id,b);}
    void configure(Trip& t,bool beforeBirth=false){
        auto& e=t.e;auto id=beforeBirth?"sample"+std::to_string(static_cast<int>(e.kind)):std::to_string(e.id);
        auto signature=number(e.length)+","+number(e.width)+","+number(e.acceleration)+","+number(e.deceleration)+","+number(e.emergencyDeceleration)+","+number(e.tau)+","+number(e.minGap)+","+number(e.lateralMinGap)+","+number(e.desiredSpeed);
        if(signature==t.applied)return;
        for(auto v:{std::pair<int,double>{0x44,e.length},{0x4d,e.width},{0x46,e.acceleration},
                    {0x47,e.deceleration},{0x7b,e.emergencyDeceleration},{0x48,std::max(.1,e.tau)},
                    {0x4c,e.minGap},{0xbb,e.lateralMinGap},{0x41,e.desiredSpeed}})
        {Buffer b;b.typedReal(v.second);client.set(beforeBirth?0xc5:0xc4,v.first,id,b);}
        t.applied=signature;
    }
    void add(Trip& t,bool returning=false){
        t.baseStage=t.stage;t.applied.clear();t.entryRestricted=false;
        int kind=static_cast<int>(t.e.kind);auto type="sample"+std::to_string(kind);
        if(!sampleTypes[kind]){Buffer copy;copy.typedString(type);client.set(0xc5,0x88,"type"+std::to_string(kind),copy);sampleTypes[kind]=true;}
        // Configure sampled dynamics before insertion. Changing maxSpeed only
        // after add can invalidate SUMO's already selected departure speed.
        configure(t,true);
        std::string id=std::to_string(t.e.id),rid="trip"+id+"_"+std::to_string(t.stage);
        std::size_t segmentEnd=t.stage;while(segmentEnd+1<t.route.size() && edges[edgeIndex.at(t.route[segmentEnd])].open<0)++segmentEnd;
        std::vector<std::string> route(t.route.begin()+t.stage,t.route.begin()+segmentEnd+1);Buffer rb;rb.byte(0x0e);rb.strings(route);client.set(0xc6,0x80,rid,rb);
        double position=0;
        if(returning){auto& lane=s.lanes_[edges[t.exit].compiled[t.exitLane]];auto nose=t.e.center+Vec2{std::cos(t.e.heading),std::sin(t.e.heading)}*(t.e.length*.5);double best=1e100,station=0;
            for(std::size_t i=1;i<lane.points.size();++i){auto d=lane.points[i]-lane.points[i-1];double span=length(d),fraction=std::clamp(((nose.x-lane.points[i-1].x)*d.x+(nose.y-lane.points[i-1].y)*d.y)/std::max(1e-9,span*span),0.,1.);double gap=length(nose-(lane.points[i-1]+d*fraction));if(gap<best){best=gap;position=station+span*fraction;}station+=span;}
            position=std::clamp(position,t.e.length,edges[t.exit].lengths[t.exitLane]-.1);
        }
        Buffer b;b.compound(14);std::vector<std::string> args{rid,type,"now",returning?std::to_string(t.exitLane):"best",returning?number(position):"base",returning?number(std::min(t.e.speed,t.e.desiredSpeed)):"max","current","max","current","","",""};for(auto& a:args)b.typedString(a);b.typedInt(0);b.typedInt(0);client.set(0xc4,0x85,id,b);
        // A vehicle-level setter asks SUMO for a singular type. The five
        // reusable sampling templates can then be changed for the next birth
        // without changing pending/active vehicles or growing a type catalog.
        setNumber(id,0x44,t.e.length);t.pending=true;t.returning=returning;
        // A SUMO stop safely holds an approach if the native crossing cannot be admitted.
        for(std::size_t i=t.stage;i<=segmentEnd && i+1<t.route.size();++i){auto& e=edges[edgeIndex.at(t.route[i])];if(e.open<0)continue;
            auto& target=edges[edgeIndex.at(t.route[i+1])];int stopLane=t.freeStopLanes.count(static_cast<int>(i))?t.freeStopLanes.at(static_cast<int>(i)):-1;
            for(std::size_t l=0;l<e.compiled.size() && stopLane<0;++l)for(int out:target.compiled)if(freeLink(e.compiled[l],out)>=0){stopLane=static_cast<int>(l);break;}
            if(stopLane<0)throw std::runtime_error("Trip has no permitted lane for its free-junction exit");
            Buffer stop;stop.compound(7);stop.typedString(e.id);stop.typedReal(std::max(.1,e.lengths[stopLane]-.1));stop.byte(0x08);stop.byte(stopLane);stop.typedReal(1e9);stop.byte(0x08);stop.byte(0);stop.typedReal(-1073741824.);stop.typedReal(-1073741824.);client.set(0xc4,0x12,id,stop);
        }
    }
    int freeLink(int from,int out){
        for(int link:s.lanes_[from].successors)if(s.lanes_[link].openJunction>=0 && std::find(s.lanes_[link].successors.begin(),s.lanes_[link].successors.end(),out)!=s.lanes_[link].successors.end())return link;
        return -1;
    }
    bool planFree(Entity& e,int from,int out,int link){
        auto& corridor=s.lanes_[link];corridor.points={s.lanes_[from].points.back(),s.lanes_[out].points.front()};
        corridor.junctionRadius=std::max({corridor.junctionRadius,length(corridor.points.front()-corridor.junctionCenter),length(corridor.points.back()-corridor.junctionCenter)});
        e.networkRoute={from,link,out};e.networkStage=0;
        auto& entry=s.lanes_[from].points;auto direction=unit(entry.back()-entry[entry.size()-2]);Vec2 side{-direction.y,direction.x};
        e.lateralOffset=(e.center.x-entry.back().x)*side.x+(e.center.y-entry.back().y)*side.y;
        double yaw=std::abs(angleDelta(e.heading,std::atan2(direction.y,direction.x)));
        if(std::abs(e.lateralOffset)>.01 || yaw>.001){
            // SL2015 may finish a lateral movement at the gate with a slight
            // yaw. Clear the rear of the entry row along that same sublane,
            // smoothly recovering the lane tangent before the main turn.
            // This is a driven curve, never a pose correction or teleport.
            Entity start=e,settled=e;
            settled.center=entry.back()+direction*(e.length*.5+.5)+side*e.lateralOffset;
            settled.heading=std::atan2(direction.y,direction.x);settled.junctionEntryCleared=true;
            double span=length(settled.center-start.center),handle=span/3;
            Vec2 a=start.center,b=a+Vec2{std::cos(start.heading),std::sin(start.heading)}*handle;
            Vec2 d=settled.center,c=d-direction*handle;
            if(!s.installNetworkStage(settled,1))return false;
            MotionPath prefix;prefix.points={a};prefix.headings={start.heading};auto previous=start.footprint();
            int samples=std::max(10,static_cast<int>(std::ceil(span/.1)));
            for(int k=1;k<=samples;++k){double t=static_cast<double>(k)/samples,u=1-t;
                auto p=a*(u*u*u)+b*(3*u*u*t)+c*(3*u*t*t)+d*(t*t*t);
                auto tangent=(b-a)*(3*u*u)+(c-b)*(6*u*t)+(d-c)*(3*t*t);double h=std::atan2(tangent.y,tangent.x);
                auto pose=previous;pose.center=p;pose.heading=h;double distance=length(p-previous.center);
                if(distance>1e-9 && std::abs(angleDelta(previous.heading,h))/distance>1/std::max(1.,e.length*.65))return false;
                double padding;auto sweep=geometry::swept(previous,pose,padding);if(!geometry::fits(s.map_,sweep,padding))return false;
                prefix.points.push_back(p);prefix.headings.push_back(h);previous=pose;
            }
            prefix.points.insert(prefix.points.end(),settled.path.begin()+1,settled.path.end());
            prefix.headings.insert(prefix.headings.end(),settled.pathHeadings.begin()+1,settled.pathHeadings.end());
            settled.center=start.center;settled.heading=start.heading;settled.path=std::move(prefix.points);settled.pathHeadings=std::move(prefix.headings);
            settled.next=1;settled.segmentProgress=0;e=std::move(settled);return true;
        }
        return s.installNetworkStage(e,1);
    }
    bool freeRouteFeasible(Trip& trip){
        trip.freeStopLanes.clear();
        // SUMO can route through a topological connection which the retained
        // free-space controller cannot physically turn through. Validate every
        // handoff before departure rather than discovering that at the queue head.
        for(std::size_t stage=0;stage+1<trip.route.size();++stage){int in=edgeIndex.at(trip.route[stage]),out=edgeIndex.at(trip.route[stage+1]);if(edges[in].open<0)continue;
            for(double distance:edges[out].lengths)if(distance<trip.e.length+.2)return false;
            bool eligible=false;
            for(int from:edges[in].compiled){auto& p=s.lanes_[from].points;auto forward=unit(p.back()-p[p.size()-2]);Vec2 side{-forward.y,forward.x};
                double limit=std::max(0.,(s.lanes_[from].width-trip.e.width)*.5-.01);bool allOffsets=true;
                for(double offset:{0.,-limit,limit}){bool feasible=false;
                    for(int to:edges[out].compiled){int link=freeLink(from,to);if(link<0)continue;Entity e=trip.e;e.center=p.back()-forward*(e.length*.5+.1)+side*offset;e.heading=std::atan2(forward.y,forward.x);
                        if(planFree(e,from,to,link)){feasible=true;break;}}
                    if(!feasible){allOffsets=false;break;}
                }
                if(allOffsets){eligible=true;trip.freeStopLanes[static_cast<int>(stage)]=static_cast<int>(std::find(edges[in].compiled.begin(),edges[in].compiled.end(),from)-edges[in].compiled.begin());break;}
            }
            if(!eligible)return false;
        }return true;
    }
    bool spawn(int lane,VehicleKind kind,int destination){
        if(lane<0 || lane>=static_cast<int>(compiledEdge.size()) || compiledEdge[lane]<0)return false;int from=compiledEdge[lane],k=static_cast<int>(kind);std::vector<int> goals;
        if(destination>=0){if(destination>=static_cast<int>(compiledEdge.size()) || compiledEdge[destination]<0)return false;goals.push_back(compiledEdge[destination]);}
        else for(std::size_t i=0;i<edges.size();++i){auto& e=edges[i];bool target=graph.nodes[e.to].destination;if(!graph.explicitDestinations){target=true;for(auto& next:edges)if(next.from==e.to && next.to!=e.from)target=false;}if(target && !(s.lanes_[lane].excludeOwnDestination && e.to==edges[from].from))goals.push_back(static_cast<int>(i));}
        std::shuffle(goals.begin(),goals.end(),s.random_);
        Trip t;t.e.kind=kind;t.e.spawnLane=lane;t.e.spawnNode=edges[from].from;const auto& p=s.profile(kind);
        t.e.length=s.sample(p.length,.2,25);t.e.width=s.sample(p.width,.15,5);t.e.minGap=p.minGap;t.e.tau=std::max(.1,p.tau);t.e.deceleration=p.deceleration;t.e.emergencyDeceleration=p.emergencyDeceleration;t.e.lateralMinGap=p.lateralMinGap;s.sampleMotion(t.e);
        for(int goal:goals){t.route=findRoute(from,goal,k);if(!t.route.empty() && freeRouteFeasible(t))break;t.route.clear();}if(t.route.empty())return false;
        t.e.id=s.nextId_++;t.e.massKg=defaultVehicleMass(kind);t.e.destinationPoint=graph.nodes[edges[edgeIndex.at(t.route.back())].to].position;
        add(t);trips.emplace(t.e.id,std::move(t));return true;
    }
    void updateLights(){
        // Uncontrolled approaches retain SUMO's priority rules ('o'). A gate
        // grants permissive 'g', which still yields to conflicting traffic.
        for(auto& [tls,size]:stateSizes){bool manual=false;auto original=client.get(0xa2,0x20,tls).text;std::string state(size,'o');
            for(auto& link:links)if(link.tls==tls){char phase='g';bool controlled=false;
                auto& lane=s.lanes_[edges[link.in].compiled[link.inLane]];
                for(auto& light:s.lights_){auto mid=(light.a+light.b)*.5;
                    if(!light.controlsDirection(lane.points.back()-lane.points[lane.points.size()-2]))continue;
                    if(light.anchorNode>=0 && graph.junctionAt(light.anchorNode)!=edges[link.in].to)continue;
                    if(!crosses(lane.points,light.a,light.b) && length(mid-lane.points.back())>std::max(3.,lane.width))continue;
                    manual=true;bool permit=light.permissive && crosses(link.shape,light.allowedA,light.allowedB);
                    auto p=light.phase(s.seconds_);char next=permit?'g':p==LightPhase::Red?'r':p==LightPhase::Yellow?'y':'g';
                    if(!controlled || next=='r' || (next=='y' && phase!='r'))phase=next;
                    controlled=true;
                }
                if(controlled)state[link.index]=phase;
            }
            if(manual && (lastStates[tls]!=state || original!=state)){Buffer b;b.typedString(state);client.set(0xc2,0x20,tls,b);lastStates[tls]=state;}
            else if(!manual && lastStates.count(tls)){Buffer b;b.typedString(graph.nodes[std::stoi(tls.substr(1))].signal?"0":"off");client.set(0xc2,0x23,tls,b);lastStates.erase(tls);}
        }
    }
    void remove(const std::string& id){Buffer b;b.byte(0x08);b.byte(3);client.set(0xc4,0x81,id,b);}
    void mirror(){s.entities_.clear();for(auto& [id,t]:trips)if(!t.teleporting && (!t.pending || t.free || t.returning))s.entities_.push_back(t.e);}
    bool enterFree(Trip& t,int edge,int lane){
        if(s.seconds_<t.e.junctionRetryAfter)return false;
        t.e.junctionRetryAfter=s.seconds_+.25;
        int next=edgeIndex.at(t.route[t.stage+1]);int from=edges[edge].compiled[lane],out=-1,link=-1;Entity e;
        for(const auto& controlled:links)if(controlled.in==edge && controlled.out==next && controlled.inLane==lane){
            auto state=client.get(0xa2,0x20,controlled.tls).text;char phase=state.at(controlled.index);
            if(phase=='r' || phase=='y' || phase=='u'){t.e.junctionStatus="Waiting for SUMO traffic signal";return false;}
        }
        for(int target:edges[next].compiled){int candidate=freeLink(from,target);if(candidate<0)continue;e=t.e;if(planFree(e,from,target,candidate)){link=candidate;out=target;break;}}
        if(link<0){t.e.junctionStatus="Waiting for a feasible free-junction curve";return false;}
        t.e.admissionPlan={e.path,e.pathHeadings,e.maneuvers};
        Entity approach=t.e;approach.networkRoute={from,link,out};approach.networkStage=0;
        // Reserve only through the physical gate; the native free controller handles crossing conflicts.
        auto direction=Vec2{std::cos(e.heading),std::sin(e.heading)};auto probe=e.footprint();probe.center=probe.center+direction*.2;
        if(!s.feasible(e.footprint(),probe,e,true,false)){t.e.junctionStatus="Waiting for physical entry space";return false;}
        approach.admissionPlan={e.path,e.pathHeadings,e.maneuvers};
        if(!s.junctionAdmission(approach,link,e.path)){t.e.junctionStatus="Waiting for a crossing gap or exit space";return false;}
        // Free trajectories are not protected by SUMO's internal-link foe
        // matrix. Keep intersecting committed corridors mutually exclusive;
        // nonintersecting movements can still use the open area concurrently.
        for(const auto& other:s.entities_)if(other.id!=e.id && other.inOpenJunction && (other.activeJunction==e.activeJunction || s.lanes_[link].semanticTransfer || (other.networkStage<other.networkRoute.size() && s.lanes_[other.networkRoute[other.networkStage]].semanticTransfer))){
            auto region=corners(other.footprint());
            for(std::size_t k=other.next;k<other.path.size();++k){auto pose=other.footprint();pose.center=other.path[k];pose.heading=other.pathHeadings[k];auto body=corners(pose);region.insert(region.end(),body.begin(),body.end());}
            region=geometry::convexHull(std::move(region));auto from=e.footprint();
            for(std::size_t k=1;k<e.path.size();++k){auto to=from;to.center=e.path[k];to.heading=e.pathHeadings[k];double padding;auto sweep=geometry::swept(from,to,padding);
                if(geometry::violatesClearance(sweep,region,padding+.1)){t.e.junctionStatus="Waiting for a committed free-junction corridor";return false;}from=to;}
        }
        t.e=std::move(e);t.free=true;t.ghost=true;t.exit=next;t.exitLane=static_cast<int>(std::find(edges[next].compiled.begin(),edges[next].compiled.end(),out)-edges[next].compiled.begin());
        auto id=std::to_string(t.e.id);
        // This SUMO object is now an invisible entry guard, not the moving
        // vehicle. Freeze it so neither braking nor sublane drift opens a gap
        // in the queue before the real native vehicle's rear has cleared.
        setInteger(id,0xb6,0);setInteger(id,0xb3,0);setNumber(id,0x40,0);
        ++handoffCount;++s.junctionAdmissions_;return true;
    }
    void finish(std::uint64_t id,bool removed=false){auto it=trips.find(id);if(it==trips.end())return;s.recordFinishedStopTime(it->second.e);if(removed){++s.removedStuckVehicles_;++s.removedJunctionVehicles_;s.lastRemovedStuckVehicle_=id;}else ++s.completed_;trips.erase(it);}
    void advanceFree(){
        mirror();s.drivingStep_=stepLength;s.prepareDrivingSnapshot(stepLength);
        for(auto& e:s.entities_) {auto it=trips.find(e.id);if(it==trips.end())continue;auto& t=it->second;
            if(t.returning && !t.free){t.e.speed=0;t.e.ageSeconds+=stepLength;t.e.stoppedSeconds+=stepLength;t.e.currentStopSeconds+=stepLength;t.e.longestStopSeconds=std::max(t.e.longestStopSeconds,t.e.currentStopSeconds);continue;}
            if(!t.free)continue;
            auto before=e.center;
            if(!t.returning){
                // Preserve the existing open-junction controller, without running any legacy lane logic.
                int substeps=10;for(int n=0;n<substeps;++n)s.advancePredictiveVehicle(e,stepLength/substeps);
                t.e=e;
                if(t.ghost){int in=edgeIndex.at(t.route[t.stage]);auto& p=s.lanes_[edges[in].compiled[0]].points;Vec2 tangent=unit(p.back()-p[p.size()-2]);
                    if((t.e.center.x-p.back().x)*tangent.x+(t.e.center.y-p.back().y)*tangent.y>t.e.length*.5+.3){remove(std::to_string(e.id));t.ghost=false;}}
                if(e.next>=e.path.size()){
                    if(t.ghost){remove(std::to_string(e.id));t.ghost=false;}
                    ++t.stage;t.free=false;t.e.inOpenJunction=false;t.e.internalJunction=-1;t.e.networkRoute.clear();t.e.laneRoute.clear();
                    add(t,true);
                }
            }
            double moved=length(t.e.center-before);t.e.ageSeconds+=stepLength;
            if(moved<=Simulation::stoppedSpeedThreshold*stepLength){t.e.stoppedSeconds+=stepLength;t.e.currentStopSeconds+=stepLength;t.e.junctionStationarySeconds+=stepLength;t.e.longestStopSeconds=std::max(t.e.longestStopSeconds,t.e.currentStopSeconds);}else {t.e.currentStopSeconds=0;t.e.junctionStationarySeconds=0;}
        }
        std::vector<std::uint64_t> expired;for(auto& [id,t]:trips)if(t.free && s.stuckVehicleSettings_.junctionEnabled && t.e.junctionStationarySeconds>=s.stuckVehicleSettings_.junctionTimeoutSeconds)expired.push_back(id);
        for(auto id:expired){auto& t=trips.at(id);if(t.ghost)remove(std::to_string(id));finish(id,true);}
        mirror();
    }
    void tick(){
        // UI edits update the presentation entities. Forward only dynamics to SUMO;
        // never feed a rendered position back into the lane engine.
        for(auto& e:s.entities_){auto it=trips.find(e.id);if(it==trips.end())continue;auto& t=it->second;
            t.e.desiredSpeed=e.desiredSpeed;t.e.acceleration=e.acceleration;t.e.minGap=e.minGap;t.e.tau=e.tau;
            t.e.deceleration=e.deceleration;t.e.emergencyDeceleration=e.emergencyDeceleration;t.e.lateralMinGap=e.lateralMinGap;
            if(!t.free)configure(t);
        }
        updateLights();client.step();
        auto events=client.get({{0xab,0x7a,"",{}},{0xab,0x80,"",{}},{0xab,0x75,"",{}},{0xab,0x76,"",{}},{0xab,0x78,"",{}}});collisionCount+=events[1].number;teleportCount+=events[2].number;
        for(auto& id:events[3].list){auto it=trips.find(static_cast<std::uint64_t>(std::stoull(id)));if(it!=trips.end() && !it->second.free)it->second.teleporting=true;}
        for(auto& id:events[4].list){auto it=trips.find(static_cast<std::uint64_t>(std::stoull(id)));if(it!=trips.end())it->second.teleporting=false;}
        for(auto& id:events[0].list){auto key=static_cast<std::uint64_t>(std::stoull(id));auto it=trips.find(key);if(it!=trips.end() && !it->second.free)finish(key);}
        auto ids=client.get(0xa4,0x00).list;std::vector<traci::Query> queries;std::vector<std::uint64_t> live;
        for(auto& id:ids){auto key=static_cast<std::uint64_t>(std::stoull(id));auto it=trips.find(key);if(it==trips.end() || it->second.free)continue;live.push_back(key);for(int v:{0x42,0x43,0x40,0x50,0x51,0x56,0x69})queries.push_back({0xa4,v,id,{}});}
        std::vector<traci::Value> values;if(!queries.empty())values=client.get(queries);
        for(std::size_t n=0;n<live.size();++n){auto& t=trips.at(live[n]);auto& e=t.e;std::size_t base=n*7;auto before=e.center;bool first=t.pending;t.pending=false;t.returning=false;t.teleporting=false;
            auto laneID=values[base+4].text;
            if(!sumoShapes.count(laneID)){auto polygon=client.get(0xa3,0x4e,laneID).polygon;auto& shape=sumoShapes[laneID];for(std::size_t k=0;k<polygon.size();k+=2)shape.push_back({polygon[k],-polygon[k+1]});}
            // SUMO's longitudinal/lateral occupancy is aligned to the lane.
            // Its displayed angle may include a lateral-velocity tilt, even
            // at a standstill; that is not a rotated collision rectangle.
            // Transfer the occupied pose, not that decorative tilt, to the
            // native rigid-body controller.
            auto& shape=sumoShapes.at(laneID);double station=values[base+5].value;Vec2 tangent{1,0};
            for(std::size_t k=1;k<shape.size();++k){auto delta=shape[k]-shape[k-1];double distance=length(delta);tangent=unit(delta);if(station<=distance)break;station-=distance;}
            e.heading=std::atan2(tangent.y,tangent.x);e.center={values[base].x-tangent.x*e.length*.5,-values[base].y-tangent.y*e.length*.5};e.speed=values[base+2].value;
            e.ageSeconds+=stepLength;if(e.speed<=Simulation::stoppedSpeedThreshold){e.stoppedSeconds+=stepLength;e.currentStopSeconds+=stepLength;e.longestStopSeconds=std::max(e.longestStopSeconds,e.currentStopSeconds);}else e.currentStopSeconds=0;
            auto road=values[base+3].text;e.internalJunction=road.size() && road[0]==':'?0:-1;e.inOpenJunction=false;t.stage=t.baseStage+values[base+6].number;
            e.laneChangeStatus="SUMO SL2015";if(laneCompiled.count(laneID)){int l=laneCompiled.at(laneID);if(!e.laneRoute.empty() && e.laneRoute.front()!=l && compiledEdge[e.laneRoute.front()]==compiledEdge[l] && !first){++e.laneChanges;++s.totalLaneChanges_;}e.laneRoute={l};e.networkRoute={l};e.networkStage=0;}
            (void)before;
        }
        mirror();
        for(std::size_t n=0;n<live.size();++n){auto it=trips.find(live[n]);if(it==trips.end())continue;auto& t=it->second;auto road=values[n*7+3].text,laneID=values[n*7+4].text;
            if(!edgeIndex.count(road) || !laneCompiled.count(laneID) || t.stage+1>=static_cast<int>(t.route.size()))continue;
            int edge=edgeIndex.at(road);auto& incoming=edges[edge];if(incoming.open<0)continue;int lane=static_cast<int>(std::find(incoming.laneIDs.begin(),incoming.laneIDs.end(),laneID)-incoming.laneIDs.begin());
            if(!t.entryRestricted && incoming.lengths[lane]-values[n*7+5].value<std::max(20.,t.e.length*3)){
                // Stabilize the existing sublane before the ownership gate.
                // Mode 512 keeps SUMO collision avoidance and gap checks,
                // while preventing a new lateral maneuver at the handoff.
                setInteger(std::to_string(t.e.id),0xb6,512);t.entryRestricted=true;
            }
            if(incoming.lengths[lane]-values[n*7+5].value<=std::max(.5,t.e.speed*stepLength+.3))if(enterFree(t,edge,lane))mirror();
        }
        advanceFree();s.recordDrivingStatistics();s.seconds_+=stepLength;++s.ticks_;s.updateTrafficMonitoring(stepLength);
    }
};
SumoBackend::SumoBackend(Simulation& s,const NodeGraph& graph,unsigned seed,const std::string& runtime,const std::string& project):impl_(std::make_unique<Impl>(s,graph,seed,runtime,project)){s.sumoBackend_=this;}
SumoBackend::~SumoBackend(){if(impl_)impl_->s.sumoBackend_=nullptr;}
bool SumoBackend::spawn(int lane,VehicleKind kind,int destination){return impl_->spawn(lane,kind,destination);}
void SumoBackend::step(double seconds){if(!std::isfinite(seconds) || seconds<=0 || seconds>10)throw std::invalid_argument("Invalid SUMO step duration");impl_->accumulator+=seconds;while(impl_->accumulator+1e-10>=stepLength){impl_->tick();impl_->accumulator-=stepLength;}}
std::string SumoBackend::status()const{return impl_->message+" | logs: "+impl_->project.string();}
std::uint64_t SumoBackend::collisions()const{return impl_->collisionCount;}
std::uint64_t SumoBackend::teleports()const{return impl_->teleportCount;}
std::uint64_t SumoBackend::handoffs()const{return impl_->handoffCount;}
std::size_t SumoBackend::pending()const{std::size_t n=0;for(auto& [id,t]:impl_->trips)n+=t.teleporting || (t.pending && !t.returning);return n;}
std::vector<Vec2> SumoBackend::routePreview(std::uint64_t id)const{
    auto it=impl_->trips.find(id);if(it==impl_->trips.end())return {};auto& trip=it->second;std::vector<Vec2> result{trip.e.center};
    for(std::size_t stage=trip.stage;stage<trip.route.size();++stage){auto& edge=impl_->edges[impl_->edgeIndex.at(trip.route[stage])];auto& shape=impl_->s.lanes_[edge.compiled.front()].points;std::size_t start=0;
        if(stage==static_cast<std::size_t>(trip.stage)){double best=1e100;for(std::size_t k=1;k<shape.size();++k){auto d=shape[k]-shape[k-1];double span=d.x*d.x+d.y*d.y;double t=std::clamp(((trip.e.center.x-shape[k-1].x)*d.x+(trip.e.center.y-shape[k-1].y)*d.y)/std::max(1e-9,span),0.,1.);double distance=length(trip.e.center-(shape[k-1]+d*t));if(distance<best){best=distance;start=k;}}}
        result.insert(result.end(),shape.begin()+start,shape.end());
    }return result;
}
}
