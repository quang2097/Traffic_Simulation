"""Reconstruct stopped poses/routes from MapSoak's terminal diagnostic log.

This is a geometry/liveness replay, not a serialized engine checkpoint. Speeds
start at zero; unrecorded claims, cooldowns and cached plans are cleared. New
sublane/lane-change searches are disabled. An in-progress change infers its
neighboring source lane and uses a four-metre completion point.
"""
import argparse
import json
import math
from pathlib import Path
import re
import shutil
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("log", type=Path)
parser.add_argument("map", type=Path)
parser.add_argument("nodes", type=Path)
parser.add_argument("--scale", type=float, default=.074)
parser.add_argument("--seconds", type=float, default=120)
parser.add_argument("--library", type=Path, default=Path("build/libtraffic.a"))
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
raw = args.log.read_text()
def number(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("Snapshot contains a non-finite number")
    return repr(value)

def point(value):
    coordinates = value.split(',')
    if len(coordinates) != 2:
        raise ValueError("Expected a two-coordinate point")
    return ','.join(number(v) for v in coordinates)

states = {}
current = None
for line in raw.splitlines():
    if line.startswith("POSE "):
        fields = dict(re.findall(r"(\w+)=([^ ]+)", line))
        current = int(fields["id"])
        for key in ('h', 'pref', 'desired', 'accel', 'lateral', 'progress'):
            fields[key] = number(fields[key])
        for key in ('xy', 'goal', 'size'):
            fields[key] = point(fields[key])
        for key in ('kind', 'stage', 'coop', 'changing'):
            fields[key] = str(int(fields[key]))
        states[current] = [fields, [], []]
    elif line.startswith("NETWORK "):
        states[current][1] = list(map(int, line.split()[2:]))
    elif line.startswith("PATH "):
        fields = dict(re.findall(r"(\w+)=([^ ]+)", line))
        states[current][2].append((point(fields["p"]), number(fields["h"])))
if not states:
    parser.error("Log has no terminal POSE/NETWORK/PATH records")
time = float(re.search(r"DONE seconds=(\S+)", raw)[1])
quoted = lambda p: json.dumps(p.resolve().as_posix())
cpp = ['#include "traffic/NodeGraph.hpp"', '#include <algorithm>', '#include <cmath>',
       '#include <iostream>', '#include <stdexcept>', 'using namespace traffic;int main(){try{',
       f'auto map=Map::fromPng({quoted(args.map)},{args.scale});',
       f'auto lanes=compileRoads(NodeGraph::load({quoted(args.nodes)})).lanes;',
       'Simulation s(map,1024);s.setLanes(lanes);s.setTrafficFlowSettings({true,true,.4,.15});',
       's.setStuckVehicleSettings({false,30});s.setTrafficRecoverySettings({true,.6});',
       'auto urban=s.urbanTrafficSettings();urban.enabled=true;s.setUrbanTrafficSettings(urban);',
       f's.loadTrafficLights({quoted(Path(str(args.nodes)+".signals"))});',
       f'while(s.seconds()+1e-7<{time})s.tick(std::min(10.,{time}-s.seconds()));',
       'auto& actors=const_cast<std::vector<Entity>&>(s.entities());']
for key, (a, network, path) in states.items():
    size, width = a['size'].split(',')
    cpp += ['{Entity e;', f'e.id={key};e.kind=static_cast<VehicleKind>({a["kind"]});',
            f'e.center={{{a["xy"]}}};e.goal={{{a["goal"]}}};e.heading={a["h"]};',
            f'e.length={size};e.width={width};e.preferredDistance={a["pref"]};',
            f'e.desiredSpeed={a["desired"]};e.acceleration={a["accel"]};e.lateralOffset={a["lateral"]};',
            f'e.networkStage={a["stage"]};e.networkRoute={{{",".join(map(str,network))}}};',
            'e.laneRoute={e.networkRoute[e.networkStage]};',
            'if(e.networkStage)e.laneRoute.push_back(e.networkRoute[e.networkStage-1]);',
            'if(e.networkStage+1<e.networkRoute.size())e.laneRoute.push_back(e.networkRoute[e.networkStage+1]);',
            'e.destinationPoint=lanes[e.networkRoute.back()].points.back();',
            f'e.cooperativeRecovery={a["coop"]};e.reservationEnd=1;e.next=1;e.segmentProgress={a["progress"]};',
            'e.currentStopSeconds=e.stoppedSeconds=500;e.sublaneAfter=e.laneChangeAfter=1e9;']
    if a['changing'] == '1':
        cpp += ['e.changingLanes=true;e.changingTo=e.networkRoute[e.networkStage];',
                'const auto& target=lanes[e.changingTo];',
                'for(std::size_t i=0;i<lanes.size();++i)if(static_cast<int>(i)!=e.changingTo && lanes[i].roadGroup==target.roadGroup && lanes[i].travelDirection==target.travelDirection && lanes[i].openJunction<0){e.changingFrom=static_cast<int>(i);break;}',
                'auto direction=target.points[1]-target.points[0];e.laneChangeDirection=direction*(1/length(direction));e.laneChangeEnd=e.center+e.laneChangeDirection*4;']
    cpp += ['e.path={'+','.join('{'+p+'}' for p, h in path)+'};',
            'e.pathHeadings={'+','.join(h for p, h in path)+'};', 'actors.push_back(e);}']
cpp += [f'for(int i=0;i<{int(args.seconds*120)} && !s.entities().empty();++i){{s.tick(1./120);',
        'if(i%120==0)for(const auto& e:s.entities()){if(!s.map().fits(e.footprint()))throw std::runtime_error("mask violation");for(const auto& o:s.entities())if(e.id<o.id && clearance(e.footprint(),o.footprint())+1e-5<std::min(e.preferredDistance,o.preferredDistance))throw std::runtime_error("clearance violation");}',
        'if(i%1200==0)std::cout<<std::unitbuf<<"t="<<(i+1)/120.<<" active="<<s.entities().size()<<" completed="<<s.completed()<<"\\n";}',
        'std::cout<<"DONE replay_seconds="<<s.seconds()-'+str(time)+'<<" completed="<<s.completed()<<" remaining="<<s.entities().size()<<" violations=0 deleted="<<s.removedStuckVehicles()<<"\\n";return s.entities().empty()?0:3;',
        '}catch(const std::exception& e){std::cerr<<e.what();return 1;}}']
source = args.output.with_suffix('.cpp')
source.parent.mkdir(parents=True, exist_ok=True)
source.write_text('\n'.join(cpp))
compiler = shutil.which('g++') or 'C:/msys64/ucrt64/bin/g++.exe'
binary = args.output.with_suffix('.exe')
subprocess.run([compiler, '-std=c++17', '-O3', '-DNDEBUG', '-I'+str(root/'include'),
                str(source), str(args.library), '-lwindowscodecs', '-lole32', '-o', str(binary)], check=True)
with args.output.with_suffix('.log').open('w') as log:
    result = subprocess.run([str(binary.resolve())], stdout=log, stderr=subprocess.STDOUT)
print(args.output.with_suffix('.log').read_text())
raise SystemExit(result.returncode)
