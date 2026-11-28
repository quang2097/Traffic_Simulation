#include "traffic/PolicyRuntime.hpp"
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <windows.h>

namespace traffic {
namespace {
std::wstring wide(const std::string& value){if(value.empty())return {};int size=MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,value.data(),static_cast<int>(value.size()),nullptr,0);if(!size)throw std::invalid_argument("Invalid UTF-8 path");std::wstring out(size,0);MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,value.data(),static_cast<int>(value.size()),out.data(),size);return out;}
std::wstring quote(const std::string& text){std::wstring out=L"\"";unsigned slashes=0;for(wchar_t c:wide(text)){if(c==L'\\'){++slashes;continue;}if(c==L'\"')out.append(slashes*2+1,L'\\');else out.append(slashes,L'\\');slashes=0;out+=c;}out.append(slashes*2,L'\\');return out+L"\"";}
PolicyReply parse(const std::string& line){
    if(line.rfind("READY ",0)==0){std::istringstream fields(line.substr(6));PolicyReply result;result.kind=PolicyReply::Ready;fields>>result.message;std::string schema,action;if(fields>>schema>>action){if(schema!="V2" || action!="HOLD")return {PolicyReply::Error,"Unsupported policy contract",{}};result.training=true;}return result;}
    if(line.rfind("SELECT ",0)==0)return {PolicyReply::SelectModel,line.substr(7),{}};
    if(line.rfind("ERROR ",0)==0)return {PolicyReply::Error,line.substr(6),{}};
    if(line.rfind("ACTION ",0)==0){PolicyReply reply;reply.kind=PolicyReply::Action;std::istringstream values(line.substr(7));values.imbue(std::locale::classic());if(values>>reply.green[0]>>reply.green[1]){std::string extra;if(!(values>>extra) && std::isfinite(reply.green[0]) && std::isfinite(reply.green[1]) && reply.green[0]>=5 && reply.green[0]<=120 && reply.green[1]>=5 && reply.green[1]<=120)return reply;}}
    if(line.rfind("REQUEST ",0)==0){std::istringstream values(line.substr(8));PolicyReply result;result.kind=PolicyReply::HoldAction;int a=-1,b=-1;std::string extra;if(values>>a>>b && !(values>>extra) && (a==0 || a==1) && (b==0 || b==1)){result.green={static_cast<double>(a),static_cast<double>(b)};return result;}}
    return {PolicyReply::Error,"Invalid response from policy worker",{}};
}
}
struct PolicyRuntime::Worker {
    HANDLE process=nullptr,input=nullptr,output=nullptr;
    std::atomic<bool> cancelled{false};
    ~Worker(){if(process){if(WaitForSingleObject(process,0)==WAIT_TIMEOUT)TerminateProcess(process,1);CloseHandle(process);}if(input)CloseHandle(input);if(output)CloseHandle(output);}
    void start(const std::string& python,const std::string& script,const std::string& weights,const std::string& algorithm){
        SECURITY_ATTRIBUTES security{sizeof(SECURITY_ATTRIBUTES),nullptr,TRUE};HANDLE childInput=nullptr,childOutput=nullptr,error=nullptr;
        auto cleanup=[&](){if(childInput)CloseHandle(childInput);if(childOutput)CloseHandle(childOutput);if(error && error!=INVALID_HANDLE_VALUE)CloseHandle(error);};
        if(!CreatePipe(&childInput,&input,&security,0) || !CreatePipe(&output,&childOutput,&security,0)){cleanup();throw std::runtime_error("Cannot create policy worker pipes");}
        SetHandleInformation(input,HANDLE_FLAG_INHERIT,0);SetHandleInformation(output,HANDLE_FLAG_INHERIT,0);
        error=CreateFileW(L"NUL",GENERIC_WRITE,FILE_SHARE_READ|FILE_SHARE_WRITE,&security,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr);
        STARTUPINFOW startup{};startup.cb=sizeof(startup);startup.dwFlags=STARTF_USESTDHANDLES;startup.hStdInput=childInput;startup.hStdOutput=childOutput;startup.hStdError=error;
        PROCESS_INFORMATION info{};auto command=quote(python)+L" -u "+quote(script)+L" "+quote(weights)+L" --algorithm "+quote(algorithm);
        bool ok=CreateProcessW(nullptr,command.data(),nullptr,nullptr,TRUE,CREATE_NO_WINDOW,nullptr,nullptr,&startup,&info);cleanup();
        if(!ok)throw std::runtime_error("Cannot start the selected Python interpreter. Choose its python.exe path.");
        process=info.hProcess;CloseHandle(info.hThread);
    }
    std::string readLine(){
        auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(30);std::string line;
        while(!cancelled){DWORD available=0;if(!PeekNamedPipe(output,nullptr,0,nullptr,&available,nullptr))throw std::runtime_error("Policy worker closed unexpectedly");
            if(available){char c;DWORD count=0;if(!ReadFile(output,&c,1,&count,nullptr) || !count)throw std::runtime_error("Cannot read policy response");if(c=='\n')return line;if(c!='\r')line+=c;if(line.size()>65536)throw std::runtime_error("Policy response exceeded the size limit");}
            else {if(WaitForSingleObject(process,0)!=WAIT_TIMEOUT)throw std::runtime_error("Policy worker exited without a response. Verify the Python interpreter and dependencies.");if(std::chrono::steady_clock::now()>deadline)throw std::runtime_error("Policy inference timed out after 30 seconds");Sleep(5);}
        }
        throw std::runtime_error("Policy request cancelled");
    }
    void write(const SignalObservation& observation){std::ostringstream text;text.imbue(std::locale::classic());text<<std::setprecision(17)<<"OBS "<<observation.density[0]<<' '<<observation.density[1]<<' '<<observation.mask[0]<<' '<<observation.mask[1]<<'\n';auto data=text.str();DWORD written=0;if(!WriteFile(input,data.data(),static_cast<DWORD>(data.size()),&written,nullptr) || written!=data.size())throw std::runtime_error("Cannot send policy observation");}
    void writeTraining(const TrainingSnapshot& observation,std::array<double,2> mask){std::ostringstream text;text.imbue(std::locale::classic());text<<std::setprecision(17)<<"OBS2";for(double value:trainingFeatures(observation))text<<' '<<value;text<<' '<<mask[0]<<' '<<mask[1]<<'\n';auto data=text.str();DWORD written=0;if(!WriteFile(input,data.data(),static_cast<DWORD>(data.size()),&written,nullptr) || written!=data.size())throw std::runtime_error("Cannot send training observation");}
};
PolicyRuntime::~PolicyRuntime(){shutdown();}
void PolicyRuntime::shutdown(){loaded_=training_=false;if(worker_)worker_->cancelled=true;if(pending_.valid())pending_.wait();pending_={};worker_.reset();}
void PolicyRuntime::load(const std::string& python,const std::string& script,const std::string& weights,const std::string& algorithm){
    shutdown();worker_=std::make_shared<Worker>();auto worker=worker_;
    pending_=std::async(std::launch::async,[=](){try{worker->start(python,script,weights,algorithm);return parse(worker->readLine());}catch(const std::exception& e){return PolicyReply{PolicyReply::Error,e.what(),{}};}});
}
void PolicyRuntime::request(const SignalObservation& observation){if(!loaded_ || busy())return;auto worker=worker_;pending_=std::async(std::launch::async,[worker,observation](){try{worker->write(observation);return parse(worker->readLine());}catch(const std::exception& e){return PolicyReply{PolicyReply::Error,e.what(),{}};}});}
void PolicyRuntime::requestTraining(const TrainingSnapshot& observation,std::array<double,2> mask){if(!loaded_ || busy() || !training_)return;auto worker=worker_;pending_=std::async(std::launch::async,[worker,observation,mask](){try{worker->writeTraining(observation,mask);return parse(worker->readLine());}catch(const std::exception& e){return PolicyReply{PolicyReply::Error,e.what(),{}};}});}
bool PolicyRuntime::poll(PolicyReply& reply){if(!pending_.valid() || pending_.wait_for(std::chrono::seconds(0))!=std::future_status::ready)return false;reply=pending_.get();if(reply.kind==PolicyReply::Ready){loaded_=true;training_=reply.training;}else if(reply.kind==PolicyReply::Error || reply.kind==PolicyReply::SelectModel){loaded_=false;worker_.reset();}return true;}
}
