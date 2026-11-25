#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace traffic::traci {
// TraCI wire format, independent of the compiler ABI used by SUMO.
struct Buffer {
    std::vector<unsigned char> data;
    std::size_t at=0;
    void byte(int value); void integer(int value); void real(double value);
    void string(const std::string& value);
    void strings(const std::vector<std::string>& value);
    void typedString(const std::string& value); void typedReal(double value);
    void typedInt(int value); void compound(int count);
    int byte(); int integer(); double real(); std::string string();
    std::vector<std::string> strings();
    void expect(int type); void commandLength();
};
struct Command { int id; Buffer body; };
struct Query { int domain,variable; std::string object; Buffer extra; };
struct Value { int type=0,number=0; double value=0,x=0,y=0; std::string text; std::vector<std::string> list; std::vector<double> polygon; Buffer raw; };
class Connection {
public:
    Connection(); ~Connection();
    Connection(const Connection&)=delete;
    void connect(int port);
    std::vector<Buffer> commands(const std::vector<Command>& commands);
    std::vector<Value> get(const std::vector<Query>& queries);
    Value get(int domain,int variable,const std::string& object="",Buffer extra={});
    void set(int domain,int variable,const std::string& object,Buffer value);
    void step(); void close();
private:
    std::uintptr_t socket_=~std::uintptr_t(0);
};
}
