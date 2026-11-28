#include "traffic/TrainingDataset.hpp"
#include "traffic/TrainingData.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
using namespace traffic;
namespace fs=std::filesystem;
void need(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
std::string read(const fs::path& p){std::ifstream in(p,std::ios::binary);return {std::istreambuf_iterator<char>(in),{}};}
void record(const std::string& path,unsigned seed){TrainingRecorder recorder;recorder.open(path,seed,{}, {},60,8,3,0);TrainingSnapshot a,b;b.time=1;recorder.sample(a);recorder.transition(a,-1,b,true,false);recorder.close();}
int main(){try{
    auto root=fs::temp_directory_path()/("traffic-dataset-test-"+std::to_string(std::random_device{}()));
    auto combined=root/"transitions.jsonl";TrainingDataset dataset;
    auto first=dataset.begin(root.u8string(),false,7);record(first,7);auto original=read(fs::u8path(first)/"transitions.jsonl");dataset.finish();
    auto prefix=read(combined);need(prefix.find("\"session_id\"")!=std::string::npos && prefix.find("\"seed\":7")!=std::string::npos,"Combined file omitted run identity");
    auto second=dataset.begin(root.u8string(),true,8);need(first!=second,"Repeated runs reused a folder");
    TrainingDataset competing;bool rejected=false;try{competing.begin(root.u8string(),true,9);}catch(...){rejected=true;}need(rejected,"Concurrent recordings could interleave dataset writes");
    record(second,8);dataset.finish();auto both=read(combined);
    need(both.substr(0,prefix.size())==prefix && both.size()>prefix.size(),"Appending changed earlier transition bytes");
    need(read(fs::u8path(first)/"transitions.jsonl")==original,"Appending changed original run data");
    dataset.finish();need(read(combined)==both,"Repeated finish duplicated transitions");
    rejected=false;try{dataset.begin(root.u8string(),false,8);}catch(...){rejected=true;}need(rejected && read(combined)==both,"New mode overwrote an existing dataset");
    auto third=dataset.begin(root.u8string(),true,8);record(third,8);
    {std::ofstream out(combined,std::ios::binary|std::ios::app);out<<"incomplete";}
    auto damaged=read(combined);rejected=false;try{dataset.finish();}catch(...){rejected=true;}
    need(rejected && read(combined)==damaged && dataset.active(),"Failed append changed prior data or discarded the retry");
    {std::ofstream out(combined,std::ios::binary);out<<both;}dataset.finish();need(read(combined).size()>both.size(),"Retry did not publish the preserved recording");
    auto legacy=root/"legacy";record(legacy.u8string(),21);auto old=read(legacy/"transitions.jsonl"),meta=read(legacy/"metadata.json");
    auto extra=dataset.begin(legacy.u8string(),true,22);record(extra,22);dataset.finish();
    need(read(legacy/"transitions.jsonl").substr(0,old.size())==old && read(legacy/"metadata.json")==meta,"Appending to a legacy single-run dataset changed original metadata or transitions");
    auto collection=root/"collection";record((collection/"episode-old").u8string(),31);auto next=dataset.begin(collection.u8string(),true,32);record(next,32);dataset.finish();
    auto imported=read(collection/"transitions.jsonl");need(imported.find("\"seed\":31")!=std::string::npos && imported.find("\"seed\":32")!=std::string::npos,"Existing episode folders were not included in the combined dataset");
    auto cancelled=dataset.begin(root.u8string(),true,77);dataset.cancel();need(!dataset.active(),"Cancelled session kept the writer lock");
    std::cout<<"Training append: byte preservation, run IDs, concurrency, retries, legacy datasets and episode import passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
