#include "traffic/SimulationResults.hpp"
#include <atomic>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace traffic {
namespace {
std::string timestamp() {
    auto now=std::chrono::system_clock::now();auto time=std::chrono::system_clock::to_time_t(now);
    std::tm utc{};
#ifdef _WIN32
    gmtime_s(&utc,&time);
#else
    gmtime_r(&time,&utc);
#endif
    std::ostringstream out;out.imbue(std::locale::classic());out<<std::put_time(&utc,"%Y-%m-%dT%H:%M:%SZ");return out.str();
}
template<class T> std::string number(T value) {
    std::ostringstream out;out.imbue(std::locale::classic());out<<std::setprecision(17)<<value;return out.str();
}
std::string row(const std::vector<std::string>& cells,bool quote) {
    std::string out;
    for(std::size_t i=0;i<cells.size();++i) {
        if(i)out+=',';
        if(!quote){out+=cells[i];continue;}
        out+='"';for(char c:cells[i]){if(c=='"')out+='"';out+=c;}out+='"';
    }
    return out;
}
}
std::string newSimulationRunId() {
    static std::atomic<unsigned long long> sequence{0};
    return number(std::chrono::system_clock::now().time_since_epoch().count())+"-"+number(++sequence);
}
SimulationResult captureSimulationResult(const Simulation& s,const ResultContext& c) {
    SimulationResult r;
    auto text=[&](const char* key,const std::string& value){r.columns.emplace_back(key);r.values.push_back(value);};
    auto field=[&](const char* key,auto value){text(key,number(value));};
    field("schema_version",3);text("saved_utc",timestamp());text("run_id",c.runId);text("run_label",c.label);text("notes",c.notes);
    text("engine",c.engine);text("controller",c.controller);field("seed",c.seed);
    text("map_path",c.mapPath);text("network_path",c.networkPath);
    field("map_width_px",s.map().width);field("map_height_px",s.map().height);field("meters_per_pixel",s.map().metersPerPixel);
    field("simulated_seconds",s.seconds());field("accepted_spawn_requests",c.acceptedSpawnRequests);
    field("active_vehicles",s.entities().size());field("pending_vehicles",c.pending);field("completed_trips",s.completed());
    field("completed_on_green",s.completedOnGreen());field("manually_completed_trips",s.manuallyCompleted());
    field("destination_completed_trips",s.completed()-s.completedOnGreen()-s.manuallyCompleted());
    field("removed_vehicles",s.removedStuckVehicles());field("removed_open_junction_vehicles",s.removedJunctionVehicles());
    field("completed_trips_per_hour",s.seconds()>0?s.completed()*3600./s.seconds():0.);
    const auto all=s.stopStatistics(true),active=s.stopStatistics(false);
    field("all_trips_count",all.vehicles);field("all_trips_that_stopped",all.vehiclesThatStopped);
    field("all_trips_total_stopped_s",all.totalSeconds);field("all_trips_mean_stopped_s",all.meanSeconds);
    field("all_trips_min_stopped_s",all.minSeconds);field("all_trips_max_stopped_s",all.maxSeconds);
    field("active_trips_mean_stopped_s",active.meanSeconds);
    double speed=0;std::size_t stopped=0;
    for(const auto& e:s.entities()){speed+=e.speed;if(e.speed<=Simulation::stoppedSpeedThreshold)++stopped;}
    field("active_mean_speed_m_per_s",s.entities().empty()?0.:speed/s.entities().size());field("active_stopped_vehicles",stopped);
    field("open_junction_vehicles",s.openJunctionVehicles());field("lane_changes",s.totalLaneChanges());
    field("native_physical_collision_events",s.physicalCollisionEvents());field("overlap_escapes",s.overlapEscapes());field("overlap_pairs",s.escapeOverlapPairs());
    field("gridlock_escapes",s.gridlockEscapes());field("replans",s.replans());field("recovery_repairs",s.trafficRecoveryRepairs());
    field("emergency_brake_events",s.emergencyBrakeEvents());field("contact_limit_events",s.contactLimitEvents());
    field("sumo_collision_involvements",c.sumoCollisions);field("sumo_teleport_starts",c.sumoTeleports);field("sumo_free_handoffs",c.sumoHandoffs);
    field("automatic_spawning",c.automaticSpawning);field("maximum_vehicles",c.maximumVehicles);
    const auto& escape=s.gridlockEscapeSettings();const auto& flow=s.trafficFlowSettings();
    field("native_overlap_allowed",escape.allowOverlap);field("native_reservation_release_enabled",escape.enabled);
    field("native_reservation_timeout_s",escape.afterSeconds);field("native_extra_overlap_wait_s",escape.overlapAfterSeconds);field("native_escape_speed_m_per_s",escape.speed);
    field("native_predictive_junctions",flow.predictiveJunctions);field("native_sublanes",flow.sublanes);field("native_swap_starved_priorities",flow.swapStarvedPriorities);field("native_priority_wait_s",flow.priorityWaitSeconds);
    field("native_replan_stopped_traffic",s.trafficRecoverySettings().enabled);
    field("remove_stuck_queue_leaders",s.stuckVehicleSettings().enabled);field("queue_blocker_timeout_s",s.stuckVehicleSettings().timeoutSeconds);
    field("remove_stopped_open_junctions",s.stuckVehicleSettings().junctionEnabled);field("open_junction_timeout_s",s.stuckVehicleSettings().junctionTimeoutSeconds);
    text("spawn_node_rates_vehicles_per_s",c.nodeDemand);text("signal_timings_at_save",c.signalTimings);
    for(int k=0;k<vehicleKindCount;++k) {
        const auto& p=s.profile(static_cast<VehicleKind>(k));
        auto profileField=[&](const char* name,double value){auto key="vehicle_"+number(k)+"_"+name;text(key.c_str(),number(value));};
        profileField("weight",p.weight);profileField("speed_mean_m_per_s",p.motion.speedMean);profileField("speed_stddev",p.motion.speedStdDev);
        profileField("length_mean_m",p.length.mean);profileField("width_mean_m",p.width.mean);profileField("min_gap_m",p.minGap);profileField("headway_s",p.tau);
    }
    return r;
}
void appendSimulationResult(const std::string& path,const SimulationResult& r) {
    if(path.empty() || r.columns.empty() || r.columns.size()!=r.values.size())throw std::invalid_argument("Invalid result file or snapshot");
    auto file=std::filesystem::u8path(path);const auto header=row(r.columns,false);
    bool existing=std::filesystem::exists(file) && std::filesystem::file_size(file)>0;
    if(existing) {
        std::ifstream input(file,std::ios::binary);if(!input)throw std::runtime_error("Cannot read results CSV: "+path);
        std::string actual;std::getline(input,actual);if(!actual.empty() && actual.back()=='\r')actual.pop_back();
        if(actual!=header)throw std::runtime_error("This CSV has a different format. Choose a new results CSV; the existing file was not changed.");
        input.clear();input.seekg(-1,std::ios::end);char last=0;input.get(last);
        if(last!='\n')throw std::runtime_error("Results CSV has an incomplete final row. Choose a new file; the existing file was not changed.");
    }
    if(!file.parent_path().empty())std::filesystem::create_directories(file.parent_path());
    std::ofstream output(file,std::ios::binary|std::ios::app);if(!output)throw std::runtime_error("Cannot write results CSV: "+path);
    if(!existing)output<<header<<"\r\n";
    output<<row(r.values,true)<<"\r\n";output.flush();
    if(!output)throw std::runtime_error("Saving results CSV failed: "+path);
}
}
