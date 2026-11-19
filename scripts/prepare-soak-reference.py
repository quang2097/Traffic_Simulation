"""Prepare the retained pre-Wassup6 controller for a paired local soak.

Run from the repository root before configuring CMake. The reference restores
known controller failures solely for measurement; both builds use the same
exact mask-query implementation. It is never selected by the viewer.
"""
from pathlib import Path
import argparse
root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, default=root / "build" / "soak-baseline")
out = parser.parse_args().output
out.mkdir(parents=True, exist_ok=True)
hybrid = (root / "src/HybridTraffic.cpp").read_text()
a = hybrid.index("        // A curved/tapered")
b = hybrid.index("        if(!installNetworkStage(preview", a)
hybrid = hybrid[:a] + hybrid[b:]
a = hybrid.index("double mouth=0;")
end = "if(along<0 || along>n+mouth)continue;"
b = hybrid.index(end, a) + len(end)
hybrid = hybrid[:a] + "if(along<0 || along>n)continue;" + hybrid[b:]
a = hybrid.index("                Entity settled=e;settled.changingLanes=false;")
b = hybrid.index("                e.changingLanes=false;", a)
hybrid = hybrid[:a] + hybrid[b:]
a = hybrid.index("        // An approach ends at its stop line")
b = hybrid.index("            double margin=e.length*.5+.2;", a)
hybrid = hybrid[:a] + "        if(stage+1==e.networkRoute.size()) {\n" + hybrid[b:]
a = hybrid.index("                    Vec2 sideways{-direction.y")
b = hybrid.index(" return false;", a) + len(" return false;")
hybrid = hybrid[:a] + "                    if(std::abs(dot(p-lane.points[i-1],{-direction.y,direction.x}))<(width+e.width)*.5+.1) return false;" + hybrid[b:]
a = hybrid.index("                    // A free crossing may traverse")
b = hybrid.index("                    if(dot(direction,bodyForward)>=-.25)continue;", a) + len("                    if(dot(direction,bodyForward)>=-.25)continue;")
hybrid = hybrid[:a] + "                    if(dot(direction,{std::cos(h),std::sin(h)})>=-.25)continue;" + hybrid[b:]
a = hybrid.index("    if(e.networkRoute.empty() || e.networkStage+1>=e.networkRoute.size()) return false;", hybrid.index("bool Simulation::advanceNetworkStage"))
b = hybrid.index("    if(lanes_[next].openJunction>=0)", a)
hybrid = hybrid[:a] + "    if(e.networkRoute.empty() || e.next<e.path.size() || e.networkStage+1>=e.networkRoute.size()) return false;\n    int next=e.networkRoute[e.networkStage+1];\n" + hybrid[b:]
a = hybrid.index("    // Stop lines on neighboring lanes may be staggered.")
b = hybrid.index("    Vec2 a=e.center+forward*lead,end=target.goal;", a)
hybrid = hybrid[:a] + hybrid[b:]
a = hybrid.index("        // Cache this exact motion for retries")
b = hybrid.index("        if(!junctionAdmission(e,next,candidate.path))", a)
hybrid = hybrid[:a] + hybrid[b:]
flow = (root / "src/TrafficFlow.cpp").read_text().replace(
    "Vec2 endSide{-std::sin(track.h.back()),std::cos(track.h.back())};\n        guide.push_back(e.goal+(freeExit?endSide*bestOffset:Vec2{})); Entity candidate=e;",
    "guide.push_back(e.goal+(freeExit?side*bestOffset:Vec2{})); Entity candidate=e;")
a = flow.index("    // Use the same body tangents")
b = flow.index("    Track own(candidate);", a)
flow = flow[:a] + "    Entity candidate=e; candidate.path=movement; candidate.pathHeadings.resize(movement.size()); candidate.next=1;\n    for(std::size_t i=1;i<movement.size();++i) { auto d=movement[i]-movement[i-1]; candidate.pathHeadings[i]=std::atan2(d.y,d.x); }\n    candidate.pathHeadings[0]=e.heading;\n" + flow[b:]
a = flow.index("                // Continuous sweeps need no extra")
b = flow.index("                continue;", a)
flow = flow[:a] + "                for(double d=0;d<=own.s.back()+.25;d+=.25)\n                    if(conflict(own.at(e,d),other.footprint(),std::min(e.preferredDistance,other.preferredDistance)+.325)) return false;\n" + flow[b:]
simulation = (root / "src/Simulation.cpp").read_text().replace(
    "bool clearingAhead=(ahead>.02 || (std::abs(ahead)<=.02 && e.id<other.id)) &&",
    "bool clearingAhead=ahead>e.length*.5 &&")
a = simulation.index("    // Resolve a lane-side contact")
b = simulation.index("    bool replanned=recoveryPreview_", a)
simulation = simulation[:a] + simulation[b:]
simulation = simulation.replace(" && !(e.cooperativeRecovery && !e.laneRoute.empty())", "")
simulation = simulation.replace("        if(e.cooperativeRecovery && !e.laneRoute.empty() && e.next<=e.reservationEnd) targetSpeed=std::min(.5,e.desiredSpeed);\n", "")
simulation = simulation.replace("#include <chrono>\n", "")
simulation = simulation.replace('        // Desired lateral gaps guide placement; motion enforces the preferred minimum.\n', '        if(flow_.sublanes && !e.networkRoute.empty() && !other.networkRoute.empty()) {\n            Vec2 forward{std::cos(e.heading),std::sin(e.heading)};\n            auto delta=other.center-e.center;\n            if(std::abs(delta.x*forward.x+delta.y*forward.y)<(e.length+other.length)*.5) gap=std::max(gap,flow_.lateralGap);\n        }\n')
flow = flow.replace('std::max(std::min(e.preferredDistance,other.preferredDistance),flow_.lateralGap)+.02', 'std::min(e.preferredDistance,other.preferredDistance)+.02')
lanes = (root / "src/Lanes.cpp").read_text().replace('std::max(std::min(e.preferredDistance,other.preferredDistance),flow_.lateralGap)+.02', 'std::min(e.preferredDistance,other.preferredDistance)+.02')
a = lanes.index("                // Nominal lane edges are not physical walls.")
b = lanes.index(" inside=true;", a) + len(" inside=true;")
lanes = lanes[:a] + "                if(length(p-(lane.points[i-1]+d*t))<=width*.5+1e-7) inside=true;" + lanes[b:]

a = simulation.index("    // Finish or expire a bounded clearance prefix.")
b = simulation.index("    planSignalQueues();", a)
simulation = simulation[:a] + "    for(auto& e:entities_) if(e.cooperativeRecovery && e.next>e.reservationEnd) e.cooperativeRecovery=false;\n" + simulation[b:]
a = simulation.index("    // Reject a lane trajectory before committing")
b = simulation.index("    e.path=std::move(motion.points);", a)
simulation = simulation[:a] + "    if(motion.points.empty()) return false;\n" + simulation[b:]
geometry = (root / "src/Geometry.cpp").read_text()
a = geometry.index("    // Near contact, tiny body steps")
b = geometry.index("    // GJK on (A - B)", a)
geometry = geometry[:a] + geometry[b:]
a = simulation.index("        double followingTarget=followingSpeed(e);\n        // Native signals")
b = simulation.index("        double targetSpeed=", a)
simulation = simulation[:a] + "        double followingTarget=followingSpeed(e),bendTarget=urbanSpeedLimit(e,followingTarget,brakingForBend,&bendBraking);\n" + simulation[b:]
a = flow.index("    // Match lane-bound red-light braking")
b = flow.index("    double bendCap=cap;", a)
flow = flow[:a] + flow[b:]
signal_queue = (root / "src/SignalQueue.cpp").read_text()
a = signal_queue.index("    // Native lanes and crossings own")
b = signal_queue.index("    if(e.signalLight==-2", a)
signal_queue = signal_queue[:a] + signal_queue[b:]
signal_approach = (root / "src/SignalApproach.cpp").read_text()
a = signal_approach.index("    // Look through the lane's temporary endpoint")
b = signal_approach.index("    for(std::size_t k=0;k<lights_.size();++k)", a)
signal_approach = signal_approach[:a] + signal_approach[b:]
for name, content in {"HybridTraffic.cpp": hybrid, "TrafficFlow.cpp": flow, "Simulation.cpp": simulation, "Lanes.cpp": lanes, "Geometry.cpp": geometry, "SignalQueue.cpp": signal_queue, "SignalApproach.cpp": signal_approach}.items():
    (out / name).write_text(content)
print("Reference controller prepared in", out)
