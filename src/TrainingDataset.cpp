#include "traffic/TrainingDataset.hpp"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <regex>
#include <stdexcept>
#include <vector>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#endif
namespace traffic {
namespace {
namespace fs=std::filesystem;
std::string quoted(const std::string& value){std::string out="\"";for(unsigned char c:value){if(c=='"' || c=='\\'){out+='\\';out+=c;}else if(c<32)throw std::runtime_error("Invalid training session name");else out+=c;}return out+'"';}
// Check the exported transition contract before reusing an existing dataset.
void validate(const fs::path& path){
    std::ifstream in(path,std::ios::binary);if(!in)throw std::runtime_error("Cannot read training transitions: "+path.u8string());
    static const std::regex schema("\"schema\"\\s*:\\s*2\\s*[,}]");
    static const std::regex terminal("\"done\"\\s*:\\s*(true|false)\\s*[,}]");
    static const std::regex finished("\"done\"\\s*:\\s*true\\s*[,}]");
    bool sawRow=false,lastDone=false;
    std::string line;while(std::getline(in,line)){
        if(!line.empty() && line.back()=='\r')line.pop_back();
        if(line.empty() || line.front()!='{' || line.back()!='}' || !std::regex_search(line,schema) || !std::regex_search(line,terminal) || line.find("\"observation\"")==std::string::npos || line.find("\"next_observation\"")==std::string::npos)
            throw std::runtime_error("Dataset contains incompatible or incomplete transitions; choose a different dataset folder. Existing data was not changed.");
        sawRow=true;lastDone=std::regex_search(line,finished);
    }
    if(in.bad())throw std::runtime_error("Cannot read training transitions");
    if(sawRow && !lastDone)throw std::runtime_error("Finish the existing training episode before appending; its final transition is not terminal. Existing data was not changed.");
    if(fs::file_size(path)){in.clear();in.seekg(-1,std::ios::end);char last=0;in.get(last);if(last!='\n')throw std::runtime_error("Dataset has an incomplete final line; existing data was not changed.");}
}
void replace(const fs::path& source,const fs::path& target){
#ifdef _WIN32
    if(!MoveFileExW(source.c_str(),target.c_str(),MOVEFILE_REPLACE_EXISTING|MOVEFILE_WRITE_THROUGH))throw std::runtime_error("Cannot publish combined transitions; close programs holding the dataset open and retry. The per-run recording is preserved.");
#else
    fs::rename(source,target);
#endif
}
}
struct TrainingDataset::Impl {
    fs::path root,directory,temporary;
    std::vector<fs::path> imports;
#ifdef _WIN32
    HANDLE lock=INVALID_HANDLE_VALUE;
    ~Impl(){if(lock!=INVALID_HANDLE_VALUE)CloseHandle(lock);}
#else
    int lock=-1;
    ~Impl(){if(lock>=0){flock(lock,LOCK_UN);::close(lock);}}
#endif
};
TrainingDataset::TrainingDataset()=default;
TrainingDataset::~TrainingDataset()=default;
bool TrainingDataset::active()const{return bool(impl_);}
void TrainingDataset::cancel(){impl_.reset();}
std::string TrainingDataset::begin(const std::string& directory,bool append,std::uint32_t seed){
    if(active())throw std::runtime_error("Finish the active training run before starting another");
    if(directory.empty())throw std::invalid_argument("Choose a training dataset folder");
    auto state=std::make_unique<Impl>();state->root=fs::absolute(fs::u8path(directory));
    if(fs::exists(state->root) && !fs::is_directory(state->root))throw std::runtime_error("Dataset path must be a folder");
    if(!append && fs::exists(state->root) && !fs::is_empty(state->root))throw std::runtime_error("New dataset needs an empty folder. Enable Append to existing dataset to add runs.");
    fs::create_directories(state->root);
    auto lock=state->root/".training-dataset.lock";
#ifdef _WIN32
    state->lock=CreateFileW(lock.c_str(),GENERIC_READ|GENERIC_WRITE,0,nullptr,OPEN_ALWAYS,FILE_ATTRIBUTE_NORMAL,nullptr);
    if(state->lock==INVALID_HANDLE_VALUE)throw std::runtime_error("This dataset is already recording or cannot be written. Stop its other recording first.");
#else
    state->lock=::open(lock.c_str(),O_CREAT|O_RDWR,0600);
    if(state->lock<0 || flock(state->lock,LOCK_EX|LOCK_NB))throw std::runtime_error("This dataset is already recording or cannot be written");
#endif
    const auto combined=state->root/"transitions.jsonl";
    if(fs::exists(combined))validate(combined);
    else {
        for(const auto& entry:fs::directory_iterator(state->root))if(entry.is_directory() && fs::is_regular_file(entry.path()/"transitions.jsonl"))state->imports.push_back(entry.path()/"transitions.jsonl");
        std::sort(state->imports.begin(),state->imports.end());for(const auto& path:state->imports)validate(path);
    }
    for(unsigned n=0;;++n){auto stamp=std::chrono::system_clock::now().time_since_epoch().count();state->directory=state->root/("run-"+std::to_string(stamp)+"-"+std::to_string(n)+"-seed-"+std::to_string(seed));if(fs::create_directory(state->directory))break;}
    root_=state->root.u8string();auto result=state->directory.u8string();impl_=std::move(state);return result;
}
void TrainingDataset::finish(){
    if(!impl_)return;auto& state=*impl_;const auto target=state.root/"transitions.jsonl",current=state.directory/"transitions.jsonl";
    validate(current);if(fs::exists(target))validate(target);
    for(const auto& path:state.imports)validate(path);
    state.temporary=state.root/(".transitions-"+state.directory.filename().u8string()+".tmp");
    try {
        std::ofstream out(state.temporary,std::ios::binary|std::ios::trunc);out.exceptions(std::ios::badbit|std::ios::failbit);
        if(fs::exists(target) && fs::file_size(target)){std::ifstream previous(target,std::ios::binary);out<<previous.rdbuf();if(previous.bad())throw std::runtime_error("Cannot copy existing training dataset");}
        auto append=[&](const fs::path& path){std::ifstream in(path,std::ios::binary);if(!in)throw std::runtime_error("Cannot read training run");std::string line;while(std::getline(in,line)){if(!line.empty() && line.back()=='\r')line.pop_back();out<<"{\"session_id\":"<<quoted(path.parent_path().filename().u8string())<<','<<line.substr(1)<<'\n';}if(in.bad())throw std::runtime_error("Cannot read training run");};
        for(const auto& path:state.imports)append(path);append(current);out.close();replace(state.temporary,target);
        impl_.reset();
    }catch(...){std::error_code error;fs::remove(state.temporary,error);throw;}
}
}
