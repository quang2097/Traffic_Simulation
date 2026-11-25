#include "traffic/TraCI.hpp"
#include <winsock2.h>
#include <ws2tcpip.h>
#include <cstring>
#include <stdexcept>
#include <limits>

namespace traffic::traci {
namespace {
void need(const Buffer& b,std::size_t n) { if(n>b.data.size()-b.at) throw std::runtime_error("Truncated SUMO TraCI response"); }
void transfer(SOCKET s,unsigned char* p,int n,bool sendData) {
    while(n) { int done=sendData?::send(s,reinterpret_cast<char*>(p),n,0) : ::recv(s,reinterpret_cast<char*>(p),n,0);
        if(done<=0) throw std::runtime_error("SUMO disconnected or timed out; inspect the SUMO session log"); p+=done;n-=done; }
}
}
void Buffer::byte(int v){data.push_back(static_cast<unsigned char>(v));}
void Buffer::integer(int v){auto u=static_cast<std::uint32_t>(v);for(int i=3;i>=0;--i)byte(u>>(8*i));}
void Buffer::real(double v){std::uint64_t u;std::memcpy(&u,&v,8);for(int i=7;i>=0;--i)byte(static_cast<int>(u>>(8*i)));}
void Buffer::string(const std::string& v){integer(static_cast<int>(v.size()));data.insert(data.end(),v.begin(),v.end());}
void Buffer::strings(const std::vector<std::string>& v){integer(static_cast<int>(v.size()));for(auto& s:v)string(s);}
void Buffer::typedString(const std::string& v){byte(0x0c);string(v);}
void Buffer::typedReal(double v){byte(0x0b);real(v);}
void Buffer::typedInt(int v){byte(0x09);integer(v);}
void Buffer::compound(int n){byte(0x0f);integer(n);}
int Buffer::byte(){need(*this,1);return data[at++];}
int Buffer::integer(){need(*this,4);std::uint32_t v=0;for(int i=0;i<4;++i)v=(v<<8)|byte();return static_cast<int>(v);}
double Buffer::real(){need(*this,8);std::uint64_t u=0;for(int i=0;i<8;++i)u=(u<<8)|byte();double v;std::memcpy(&v,&u,8);return v;}
std::string Buffer::string(){int n=integer();if(n<0)throw std::runtime_error("Invalid TraCI string");need(*this,n);std::string v(data.begin()+at,data.begin()+at+n);at+=n;return v;}
std::vector<std::string> Buffer::strings(){int n=integer();if(n<0 || n>1000000)throw std::runtime_error("Invalid TraCI list");std::vector<std::string> v;while(n--)v.push_back(string());return v;}
void Buffer::expect(int t){if(byte()!=t)throw std::runtime_error("Unexpected SUMO TraCI value type");}
void Buffer::commandLength(){if(byte()==0)integer();}
Connection::Connection(){WSADATA w;if(WSAStartup(MAKEWORD(2,2),&w))throw std::runtime_error("Cannot initialize TraCI sockets");}
Connection::~Connection(){try{close();}catch(...){}WSACleanup();}
void Connection::connect(int port){
    auto s=::socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);if(s==INVALID_SOCKET)throw std::runtime_error("Cannot open SUMO socket");
    sockaddr_in a{};a.sin_family=AF_INET;a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);a.sin_port=htons(static_cast<u_short>(port));
    if(::connect(s,reinterpret_cast<sockaddr*>(&a),sizeof(a))) {closesocket(s);throw std::runtime_error("SUMO not ready");}
    DWORD timeout=5000;setsockopt(s,SOL_SOCKET,SO_RCVTIMEO,reinterpret_cast<char*>(&timeout),sizeof(timeout));
    setsockopt(s,SOL_SOCKET,SO_SNDTIMEO,reinterpret_cast<char*>(&timeout),sizeof(timeout));
    BOOL yes=TRUE;setsockopt(s,IPPROTO_TCP,TCP_NODELAY,reinterpret_cast<char*>(&yes),sizeof(yes));socket_=s;
}
std::vector<Buffer> Connection::commands(const std::vector<Command>& commands){
    Buffer payload;
    for(auto& c:commands){int n=static_cast<int>(c.body.data.size())+2;
        if(n<256)payload.byte(n);else {payload.byte(0);payload.integer(n+4);}payload.byte(c.id);
        payload.data.insert(payload.data.end(),c.body.data.begin(),c.body.data.end());}
    Buffer packet;packet.integer(static_cast<int>(payload.data.size())+4);packet.data.insert(packet.data.end(),payload.data.begin(),payload.data.end());
    transfer(socket_,packet.data.data(),static_cast<int>(packet.data.size()),true);
    Buffer head;head.data.resize(4);transfer(socket_,head.data.data(),4,false);int size=head.integer()-4;
    if(size<0 || size>128*1024*1024)throw std::runtime_error("Invalid TraCI packet length");
    Buffer result;result.data.resize(size);transfer(socket_,result.data.data(),size,false);
    std::vector<Buffer> replies;
    for(auto& c:commands){result.commandLength();if(result.byte()!=c.id)throw std::runtime_error("TraCI status command mismatch");
        int status=result.byte();auto message=result.string();if(status)throw std::runtime_error("SUMO: "+message);
        Buffer body;
        if(c.id==2) {int count=result.integer();if(count)throw std::runtime_error("Unexpected TraCI subscriptions");}
        else if(c.id>=0xa0 && c.id<=0xaf){auto begin=result.at;int length=result.byte();if(!length){length=result.integer();begin=result.at-5;}need(result,length-(result.at-begin));
            body.data.assign(result.data.begin()+begin,result.data.begin()+begin+length);result.at=begin+length;}
        else if(c.id==0){auto begin=result.at;int length=result.byte();if(!length){length=result.integer();begin=result.at-5;}need(result,length-(result.at-begin));body.data.assign(result.data.begin()+begin,result.data.begin()+begin+length);result.at=begin+length;}
        replies.push_back(std::move(body));
    }
    if(result.at!=result.data.size())throw std::runtime_error("Unconsumed TraCI response bytes");
    return replies;
}
std::vector<Value> Connection::get(const std::vector<Query>& queries){
    std::vector<Command> cs;for(auto& q:queries){Buffer b;b.byte(q.variable);b.string(q.object);b.data.insert(b.data.end(),q.extra.data.begin(),q.extra.data.end());cs.push_back({q.domain,std::move(b)});}
    auto replies=commands(cs);std::vector<Value> values;
    for(std::size_t i=0;i<replies.size();++i){auto b=std::move(replies[i]);b.commandLength();if(b.byte()!=queries[i].domain+0x10 || b.byte()!=queries[i].variable || b.string()!=queries[i].object)throw std::runtime_error("Mismatched TraCI query response");
        Value v;v.type=b.byte();switch(v.type){case 0x01:v.x=b.real();v.y=b.real();break;case 0x0b:v.value=b.real();break;case 0x09:v.number=b.integer();break;case 0x0c:v.text=b.string();break;case 0x0e:v.list=b.strings();break;
        case 0x06:{int n=b.byte();if(!n)n=b.integer();if(n<0 || n>1000000)throw std::runtime_error("Invalid TraCI polygon");while(n--){v.polygon.push_back(b.real());v.polygon.push_back(b.real());}break;}
        case 0x0f:v.raw.data.assign(b.data.begin()+b.at,b.data.end());b.at=b.data.size();break;
        default:throw std::runtime_error("Unsupported TraCI response type "+std::to_string(v.type));}
        if(b.at!=b.data.size())throw std::runtime_error("Invalid TraCI value length");values.push_back(std::move(v));}
    return values;
}
Value Connection::get(int d,int v,const std::string& o,Buffer e){return get(std::vector<Query>{{d,v,o,std::move(e)}}).front();}
void Connection::set(int d,int v,const std::string& o,Buffer value){Buffer b;b.byte(v);b.string(o);b.data.insert(b.data.end(),value.data.begin(),value.data.end());commands({{d,std::move(b)}});}
void Connection::step(){Buffer b;b.real(0);commands({{2,b}});}
void Connection::close(){if(socket_!=~std::uintptr_t(0)){try{commands({{0x7f,{}}});}catch(...){}closesocket(socket_);socket_=~std::uintptr_t(0);}}
}
