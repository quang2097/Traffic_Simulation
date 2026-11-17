#include "traffic/Simulation.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace traffic;
void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
int main() {
    try {
        Map map(20,20,std::vector<Tile>(400,Tile::Road)); map.tiles[map.index({5,10})]=Tile::Spawn; map.tiles[map.index({6,10})]=Tile::Destination;
        Simulation s(map,42);
        for(int i=0;i<vehicleKindCount;++i) s.setProfile(static_cast<VehicleKind>(i),{{1,0},{.4,0},{.1,0},{5,0,10,0},1});
        std::array<double,vehicleKindCount> mix{10,20,30,0,40}; s.setSpawnPercentages(mix);
        std::array<int,vehicleKindCount> counts{};
        for(int i=0;i<2000;++i) {
            require(s.spawnRandom(0),"spawn mix sample accepted"); ++counts[static_cast<int>(s.entities()[0].kind)]; s.tick(1);
            require(s.entities().empty(),"spawn mix probe finished");
        }
        for(int i=0;i<vehicleKindCount;++i) require(std::abs(counts[i]/20.-mix[i])<4,"observed free-road class percentage differs from requested mix");
        require(counts[3]==0,"zero percent excludes trucks");
        mix={0,0,100,0,0}; s.setSpawnPercentages(mix);
        for(int i=0;i<25;++i) { require(s.spawnRandom(0) && s.entities()[0].kind==VehicleKind::Car,"100 percent cars"); s.tick(1); }
        auto current=s.spawnPercentages();
        for(auto invalid:std::array<std::array<double,vehicleKindCount>,3>{{{0,0,0,0,0},{10,10,10,10,10},{-1,0,101,0,0}}}) {
            bool rejected=false; try { s.setSpawnPercentages(invalid); } catch(const std::invalid_argument&) { rejected=true; }
            require(rejected && s.spawnPercentages()==current,"invalid mix rejected atomically");
        }
        require(s.spawnRandom(0),"active mix edit spawn"); auto before=s.entities()[0]; double clock=s.seconds();
        s.setSpawnPercentages({100,0,0,0,0});
        require(s.seconds()==clock && s.entities()[0].id==before.id && s.entities()[0].kind==VehicleKind::Car,"live mix edit retains existing vehicle");
        std::cout<<"Percentage mixes, zero/100 shares, sampling and live edits passed\n";
    } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
