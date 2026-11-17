"""Authored road interpretation for Wassup5.png; coordinates below are image pixels."""
from pathlib import Path
import json

ROOT = Path(__file__).resolve().parents[1]
SCALE = .73


def prepare():
    # Explicit junctions and boundary ports; the paired carriageways never share an edge.
    points = [(1110, 740), (1740, 145), (555, 40), (760, 70), (1850, 40), (2050, 140),
              (1615, 35), (315, 1210), (595, 1210), (45, 520), (1400, 1210), (2060, 920)]
    roads = [
        ("NW inbound", 2, 0, 1, [(555,40),(775,365),(850,590),(1110,740)]),
        ("NW outbound", 0, 3, 1, [(1110,740),(1100,560),(1040,450),(905,280),(760,70)]),
        ("NE outbound", 0, 1, 1, [(1110,740),(1220,490),(1410,365),(1740,145)]),
        ("NE outbound boundary", 1, 4, 1, [(1740,145),(1850,40)]),
        ("NE inbound", 5, 0, 1, [(2050,140),(1750,365),(1440,600),(1110,740)]),
        ("North spur, two-way", 6, 1, 2, [(1615,35),(1680,110),(1740,145)]),
        ("SW outbound", 0, 7, 1, [(1110,740),(860,850),(570,1040),(315,1210)]),
        ("SW inbound", 8, 0, 1, [(595,1210),(860,1010),(980,915),(1110,740)]),
        ("West, two-way", 9, 0, 2, [(45,520),(370,675),(650,810),(850,800),(1110,740)]),
        ("SE bottom, two-way", 10, 0, 2, [(1400,1210),(1290,1120),(1110,985),(1060,870),(1110,740)]),
        ("SE east, two-way", 11, 0, 2, [(2060,920),(1870,1200),(1770,1200),(1530,955),(1460,820),(1110,740)]),
    ]
    sources = {2,5,6,8,9,10,11}
    out = [f"TRAFFIC_NODES 2 {len(points)} {len(roads)}"]
    for i, (x,y) in enumerate(points):
        out.append(f"{x*SCALE:.6f} {y*SCALE:.6f} {int(i in sources)} 0.06 3 {int(i==0)}")
    for name,a,b,kind,shape in roads:
        out.append(f"{a} {b} 3.5 13.9 2 {kind} 2 {len(shape)}")
        out.extend(f"{x*SCALE:.6f} {y*SCALE:.6f}" for x,y in shape)
    path = ROOT / "examples" / "Wassup5.nodes"
    path.write_text("\n".join(out)+"\n")
    manifest = dict(meters_per_pixel=SCALE, driving_side="right", spawn_vehicles_per_second_per_marked_node=.06,
                    roads=[dict(name=name,from_node=a,to_node=b,two_way=kind==2,lanes_per_direction=2) for name,a,b,kind,shape in roads])
    (ROOT / "examples" / "Wassup5-network.json").write_text(json.dumps(manifest,indent=2))
    return path


if __name__ == "__main__":
    print(prepare())
