"""Build and run real SUMO projects from Traffic Lab .nodes files (no shell commands)."""
import argparse
import json
import math
import os
from pathlib import Path
import random
import shutil
import subprocess
import sys
import struct
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "build" / "sumo-runtime"
sys.path.insert(0, str(RUNTIME))


def binary(name):
    candidates = [RUNTIME / "sumo" / "bin" / (name + ".exe")]
    if os.environ.get("SUMO_HOME"):
        candidates.append(Path(os.environ["SUMO_HOME"]) / "bin" / (name + ".exe"))
    for path in candidates:
        if path.exists():
            return str(path)
    found = shutil.which(name)
    if found:
        return found
    raise RuntimeError("SUMO not installed. Run python -m pip install --target build/sumo-runtime eclipse-sumo traci")


def read_graph(path):
    tokens = iter(Path(path).read_text(encoding="utf-8-sig").split())
    if next(tokens) != "TRAFFIC_NODES":
        raise ValueError("Expected a Traffic Lab .nodes network")
    version, nn, ne = int(next(tokens)), int(next(tokens)), int(next(tokens))
    if version not in (1, 2):
        raise ValueError("Unsupported node-network version")
    nodes, roads = [], []
    for _ in range(nn):
        x, y, spawn, rate, braking = (float(next(tokens)) for _ in range(5))
        signal = int(next(tokens)) if version >= 2 else 0
        nodes.append(dict(x=x, y=y, spawn=bool(spawn), rate=rate, braking=braking, signal=signal))
    for _ in range(ne):
        a, b = int(next(tokens)), int(next(tokens))
        width, speed, smoothing = (float(next(tokens)) for _ in range(3))
        kind, lanes, count = (int(next(tokens)) for _ in range(3)) if version >= 2 else (0, 1, 0)
        shape = [(float(next(tokens)), float(next(tokens))) for _ in range(count)]
        if not shape:
            shape = [(nodes[a]["x"], nodes[a]["y"]), (nodes[b]["x"], nodes[b]["y"])]
        roads.append(dict(a=a, b=b, width=width, speed=speed, smoothing=smoothing, kind=kind, lanes=lanes, shape=shape))
    return nodes, roads


def write_xml(root, path):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def build(network, output, duration=900, seed=42, background=None, meters_per_pixel=.73, profiles_file=None):
    import sumolib
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    nodes, roads = read_graph(network)
    nd, ed = ET.Element("nodes"), ET.Element("edges")
    for i, node in enumerate(nodes):
        incident=[road["smoothing"] for road in roads if road["a"]==i or road["b"]==i]
        radius=12.5*sum(incident)/len(incident) if incident else 12.5
        ET.SubElement(nd, "node", id=f"n{i}", x=str(node["x"]), y=str(-node["y"]), type="traffic_light" if node["signal"] else "priority",radius=str(radius))
    origins = {}
    for i, road in enumerate(roads):
        for reverse in range(2 if road["kind"] == 2 else 1):
            a, b = (road["b"], road["a"]) if reverse else (road["a"], road["b"])
            shape = list(reversed(road["shape"])) if reverse else road["shape"]
            eid = f"r{i}{'b' if reverse else 'f'}"
            ET.SubElement(ed, "edge", id=eid, attrib={"from": f"n{a}", "to": f"n{b}", "numLanes": str(road["lanes"]), "width": str(road["width"]), "speed": str(road["speed"]), "spreadType": "right" if road["kind"] == 2 else "center", "shape": " ".join(f"{x},{-y}" for x, y in shape)})
            origins[eid] = a
    write_xml(nd, output / "network.nod.xml")
    write_xml(ed, output / "network.edg.xml")
    environment=os.environ.copy()
    environment["SUMO_HOME"]=str(Path(binary("sumo")).parent.parent)
    conversion = [binary("netconvert"), "--node-files", str(output / "network.nod.xml"), "--edge-files", str(output / "network.edg.xml"), "--output-file", str(output / "network.net.xml"), "--no-turnarounds", "true", "--junctions.corner-detail", "12", "--junctions.internal-link-detail", "12", "--tls.default-type", "static", "--tls.green.time", "25", "--tls.red.time", "2"]
    conversion.extend(["--junctions.limit-turn-speed","1.8"])
    subprocess.run(conversion, check=True, env=environment, stdout=(output / "netconvert.log").open("w"), stderr=subprocess.STDOUT)
    net = sumolib.net.readNet(str(output / "network.net.xml"))
    exits = [e for e in net.getEdges() if not e.getOutgoing()]
    sources = [e for e in net.getEdges() if nodes[origins[e.getID()]]["spawn"]]
    routes = ET.Element("routes")
    distribution = ET.SubElement(routes, "vTypeDistribution", id="mixed")
    rng = random.Random(seed)
    # Sampled mixed dimensions/dynamics with stable lane following on this junction.
    profiles = [("bike", "bicycle", .03, 1.8, .6, 1.2, 4), ("motorbike", "motorcycle", .80, 2, .7, 2, 10), ("car", "passenger", .14, 4.4, 1.8, 1.6, 13.9), ("truck", "truck", .01, 8, 2.4, .9, 10), ("bus", "bus", .02, 10, 2.4, .9, 10)]
    overrides=json.loads(Path(profiles_file).read_text()) if profiles_file else None
    if overrides is not None and (len(overrides)!=5 or sum(p["weight"] for p in overrides)<=0):
        raise ValueError("Vehicle profiles must contain five classes with positive total weight")
    for index,(name, vclass, share, length, width, accel, speed) in enumerate(profiles):
        settings=overrides[index] if overrides else {}
        share=settings.get("weight",share)
        length,width,accel,speed=(settings.get(k,v) for k,v in (("length",length),("width",width),("accel",accel),("speed",speed)))
        for sample in range(8):
            ET.SubElement(distribution, "vType", id=f"{name}_{sample}", vClass=vclass, probability=str(share / 8), length=str(max(.5, rng.gauss(length, settings.get("lengthStd",length * .05)))), width=str(max(.3, rng.gauss(width, settings.get("widthStd",width * .05)))), accel=str(max(.3, rng.gauss(accel, settings.get("accelStd",accel * .1)))), decel="4.5", emergencyDecel="9", sigma="0.1", tau="1.2", minGap=str(max(.7,settings.get("gap",.7 if name in ("bike","motorbike") else 1.2))), maxSpeed=str(max(.5,rng.gauss(speed,settings["speedStd"]) if settings.get("speedStd",0)>0 else speed)), speedFactor="normc(1,0.1,0.8,1.2)", lcStrategic="1", lcCooperative="1", lcSublane="0", lcAssertive="0.8", lcPushy="0", minGapLat="0.3", latAlignment="center")
    report = {"source_nodes": [], "unreachable_sources": [], "routes": 0, "duration": duration, "seed": seed, "sumo_version": subprocess.check_output([binary("sumo"), "--version"], text=True).splitlines()[0]}
    for source in sources:
        candidates = []
        for destination in exits:
            if destination == source or destination.getToNode() == source.getFromNode():
                continue
            path, _ = net.getShortestPath(source, destination)
            if path:
                candidates.append(path)
        if not candidates:
            report["unreachable_sources"].append(source.getID())
            continue
        dist_id = "routes_" + source.getID()
        dist = ET.SubElement(routes, "routeDistribution", id=dist_id)
        for path in candidates:
            ET.SubElement(dist, "route", edges=" ".join(e.getID() for e in path), probability="1")
        origin = origins[source.getID()]
        outgoing_count = sum(1 for e in sources if origins[e.getID()] == origin)
        rate = nodes[origin]["rate"] / outgoing_count
        if rate > 0:
            ET.SubElement(routes, "flow", id="flow_" + source.getID(), type="mixed", route=dist_id, begin="0", end=str(duration), vehsPerHour=str(rate * 3600), departLane="best", departSpeed="max", reroute="true")
        report["source_nodes"].append(origin)
        report["routes"] += len(candidates)
    (output / "network-report.json").write_text(json.dumps(report, indent=2))
    if report["unreachable_sources"]:
        raise ValueError("Spawn approaches have no reachable exit: "+", ".join(report["unreachable_sources"]))
    if not sources or not any(node["spawn"] and node["rate"]>0 for node in nodes):
        raise ValueError("Mark at least one connected spawn node with a positive rate")
    isolated=[i for i,node in enumerate(nodes) if node["spawn"] and node["rate"]>0 and i not in origins.values()]
    if isolated:
        raise ValueError("Spawn nodes have no outgoing road: "+str(isolated))
    write_xml(routes, output / "demand.rou.xml")
    config = ET.Element("configuration")
    inp = ET.SubElement(config, "input")
    ET.SubElement(inp, "net-file", value="network.net.xml")
    ET.SubElement(inp, "route-files", value="demand.rou.xml")
    time = ET.SubElement(config, "time")
    ET.SubElement(time, "end", value=str(duration + 1200))
    processing = ET.SubElement(config, "processing")
    for key, value in {"step-length": ".1", "lateral-resolution": "-1", "collision.action": "warn", "collision.check-junctions": "true", "time-to-teleport": "-1", "device.rerouting.probability": "1", "device.rerouting.period": "20", "seed": str(seed)}.items():
        # SUMO config accepts options in a configuration group independent of category.
        ET.SubElement(processing, key, value=value)
    out = ET.SubElement(config, "output")
    ET.SubElement(out, "tripinfo-output", value="tripinfo.xml")
    ET.SubElement(out, "summary-output", value="summary.xml")
    if background is None:
        background=Path(network).with_suffix(".png")
    if Path(background).exists():
        data=Path(background).read_bytes()
        if data[:8]!=b"\x89PNG\r\n\x1a\n":
            raise ValueError("Background must be PNG")
        width,height=struct.unpack(">II",data[16:24])
        shutil.copyfile(background,output / "map.png")
        offset=net.getLocationOffset()
        settings=ET.Element("viewsettings")
        ET.SubElement(settings,"decal",file="map.png",centerX=str(width*meters_per_pixel/2+offset[0]),centerY=str(-height*meters_per_pixel/2+offset[1]),width=str(width*meters_per_pixel),height=str(height*meters_per_pixel),layer="-1",rotation="0")
        write_xml(settings,output / "view.settings.xml")
        gui=ET.SubElement(config,"gui_only")
        ET.SubElement(gui,"gui-settings-file",value="view.settings.xml")
    write_xml(config, output / "simulation.sumocfg")
    (output / "network-report.json").write_text(json.dumps(report, indent=2))
    return output


def run(project, gui=False):
    command = [binary("sumo-gui" if gui else "sumo"), "-c", str(project / "simulation.sumocfg"), "--log", str(project / "simulation.log"), "--error-log", str(project / "warnings.log")]
    if gui:
        subprocess.run(command, cwd=project, check=True)
    else:
        subprocess.run(command, check=True, cwd=project, stdout=(project / "run.log").open("w"), stderr=subprocess.STDOUT)
        trips = ET.parse(project / "tripinfo.xml").getroot()
        last=ET.parse(project / "summary.xml").getroot()[-1]
        validation={"completed":len(trips),"max_waiting_seconds":max((float(t.get("waitingTime","0")) for t in trips),default=0),"final":dict(last.attrib),"collisions":int(last.get("collisions","0")),"teleports":int(last.get("teleports","0"))}
        (project / "validation.json").write_text(json.dumps(validation,indent=2))
        print(json.dumps(validation,indent=2))
        if validation["collisions"] or validation["teleports"] or int(last.get("running","0")) or int(last.get("waiting","0")):
            raise RuntimeError("Validation detected collisions, teleports or traffic that did not drain; inspect logs")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("network", type=Path)
    parser.add_argument("--output", type=Path, default=ROOT / "build" / "sumo-project")
    parser.add_argument("--duration", type=float, default=300)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--background",type=Path)
    parser.add_argument("--meters-per-pixel",type=float,default=.73)
    parser.add_argument("--profiles",type=Path)
    args = parser.parse_args()
    try:
        project = build(args.network, args.output, args.duration, args.seed,args.background,args.meters_per_pixel,args.profiles)
        if args.run or args.gui:
            run(project, args.gui)
        print(project / "simulation.sumocfg")
    except Exception as error:
        print(error, file=sys.stderr)
        sys.exit(1)
