#include "traffic/SimulationResults.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <locale>
#include <stdexcept>
using namespace traffic;
void need(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
std::string read(const std::filesystem::path& path){std::ifstream in(path,std::ios::binary);return {std::istreambuf_iterator<char>(in),{}};}
std::vector<std::vector<std::string>> parse(const std::string& csv){
    std::vector<std::vector<std::string>> rows;std::vector<std::string> row;std::string cell;bool quoted=false;
    for(std::size_t i=0;i<csv.size();++i){char c=csv[i];
        if(c=='"'){if(quoted && i+1<csv.size() && csv[i+1]=='"'){cell+='"';++i;}else quoted=!quoted;}
        else if(!quoted && c==','){row.push_back(cell);cell.clear();}
        else if(!quoted && c=='\n'){row.push_back(cell);rows.push_back(row);row.clear();cell.clear();}
        else if(quoted || c!='\r')cell+=c;
    }
    need(!quoted && cell.empty() && row.empty(),"CSV ended inside a field or row");return rows;
}
std::string value(const SimulationResult& r,const std::string& key){auto at=std::find(r.columns.begin(),r.columns.end(),key);need(at!=r.columns.end(),"Result column missing");return r.values[at-r.columns.begin()];}
struct CommaDecimal:std::numpunct<char>{char do_decimal_point() const override{return ',';}};
int main(){try{
    auto root=std::filesystem::temp_directory_path()/("traffic-results-"+newSimulationRunId());
    auto file=root/"comparison.csv";
    Simulation s(Map(80,30,std::vector<Tile>(2400,Tile::Road)),42);s.setCollisionWorkers(1);
    s.setStuckVehicleSettings({false,30,false,60});s.setTrafficRecoverySettings({false,1,25});
    ResultContext c;c.runId=newSimulationRunId();c.label="Baseline, \"morning\"";c.notes="First line\r\nSecond line — comparison";c.seed=42;
    auto zero=captureSimulationResult(s,c);need(value(zero,"completed_trips_per_hour")=="0" && value(zero,"all_trips_mean_stopped_s")=="0","Empty snapshot divided by zero");
    appendSimulationResult(file.u8string(),zero);
    s.setProfile(VehicleKind::Car,{{4,0},{1.8,0},{.5,0},{3,0,3,0},1});
    need(s.spawnAt({10,15},{15,15},VehicleKind::Car,0),"Result fixture did not spawn");s.tick(5);
    auto escape=s.gridlockEscapeSettings();escape.allowOverlap=true;s.setGridlockEscapeSettings(escape);
    c.label="Overlap enabled";
    auto savedLocale=std::locale();std::locale::global(std::locale(savedLocale,new CommaDecimal));
    auto snapshot=captureSimulationResult(s,c);std::locale::global(savedLocale);
    need(s.completed()==1 && value(snapshot,"completed_trips")=="1","Completed trips missing");
    need(value(snapshot,"completed_trips_per_hour").find(',')==std::string::npos,"CSV numbers followed system decimal locale");
    need(std::abs(std::stod(value(snapshot,"completed_trips_per_hour"))-720)<1e-6,"Throughput did not use simulated time");
    need(value(snapshot,"native_overlap_allowed")=="1" && value(snapshot,"all_trips_count")=="1","Snapshot omitted settings or finished trip population");
    appendSimulationResult(file.u8string(),snapshot);
    c.runId=newSimulationRunId();c.engine="SUMO";c.sumoCollisions=3;c.sumoTeleports=2;c.pending=4;
    auto sumo=captureSimulationResult(s,c);appendSimulationResult(file.u8string(),sumo);
    need(value(sumo,"sumo_collision_involvements")=="3" && value(sumo,"pending_vehicles")=="4","SUMO counters lost");
    auto rows=parse(read(file));need(rows.size()==4,"Appending overwrote saved rows or repeated the CSV header");
    for(const auto& row:rows)need(row.size()==zero.columns.size(),"CSV column count changed");
    auto label=std::find(zero.columns.begin(),zero.columns.end(),"run_label")-zero.columns.begin();
    auto notes=std::find(zero.columns.begin(),zero.columns.end(),"notes")-zero.columns.begin();
    need(rows[1][label]==zero.values[label] && rows[1][notes]==c.notes,"Quoted labels, Unicode or multiline notes did not round trip");
    auto bad=root/"unrelated.csv";{std::ofstream out(bad);out<<"different,header\n1,2\n";}
    auto before=read(bad);bool rejected=false;try{appendSimulationResult(bad.u8string(),snapshot);}catch(const std::exception&){rejected=true;}
    need(rejected && read(bad)==before,"Incompatible CSV was modified");
    auto broken=root/"incomplete.csv";{std::ofstream out(broken,std::ios::binary);out<<read(file)<<"partial row";}
    before=read(broken);rejected=false;try{appendSimulationResult(broken.u8string(),snapshot);}catch(const std::exception&){rejected=true;}
    need(rejected && read(broken)==before,"Incomplete CSV was modified");
    rejected=false;try{appendSimulationResult((file/"blocked.csv").u8string(),snapshot);}catch(const std::exception&){rejected=true;}
    need(rejected,"Unwritable destination did not report failure");
    for(auto path:{file,bad,broken})std::filesystem::remove(path);std::filesystem::remove(root);
    std::cout<<"Results: snapshots, append, CSV escaping, locale, engine counters and failure preservation passed\n";
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
