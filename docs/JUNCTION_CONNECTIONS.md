# Connecting roads and lanes

Use `build/traffic_car_following.exe` and the Map design workspace.

## One-way main road with a two-way side road

1. Draw the main road as a one-way road in its travel direction.
2. Choose **Insert junction node on road**. Centerlines become visible; hover the main road until a yellow point appears, then click where the side road should join. The road splits there, preserving its direction, lane counts, shape and interpolated widths.
3. Place a node at the other end of the side road. Click the inserted main-road node, choose **Connect two-way arm...**, then click the side-road node. Set the new arm's lane count in the node popup before connecting. Existing width/speed controls in Map design determine the new arm's width and speed.
4. Keep destination flags at actual trip endpoints. The junction node itself should usually not be a destination.

Vehicles approaching on the main road can continue straight or turn into the side road. Vehicles returning along the side road can join the main road in its permitted direction. A one-way road never gains a reverse lane from connecting a two-way arm. The new junction does not remove the through route.

## Other junctions

- **T-junction:** three road connections at one shared node.
- **Four-way intersection:** four road connections at one shared node. Split any existing through road first; connect the additional arms to the same node.
- **Merge, branch or skew junction:** connect any number of roads, at their actual angles, to the shared node. Each arm can independently be one-way or two-way and have a different lane count and width. Use **Connect one-way arm...** for a connection directed from the selected node to the clicked target. For an incoming one-way arm, start from the outside node and connect toward the junction.
- **U-turn:** enable **Allow U-turn connections** at the node where a vehicle should be allowed to return into an available opposing lane. This does not create a reverse lane on a one-way road. Free junctions already allow U-turn routes and retain their existing behavior.

Simply crossing two drawn roads does not join them. They must share a node. A three/four-road junction is identified in the node popup. Junction edits reset traffic, like other network geometry edits. Save the node network to retain connections, U-turn permissions and lane sections (node format 8; earlier versions still load).

Paint sufficient red road space before adding an arm or enabling a U-turn. Finite-width connectors must fit the road mask; invalid edits are rejected. For a large intersection that should use free navigation, keep using its open-junction radius. The new tools also work with ordinary lane-guided junctions.

At **radius 0**, generated internal connectors use junction admission while retaining committed connections. Body-aware curves clear the rear from the approach before turning and carry the trip into its exit lane. Lateral positions are preserved where the exit width permits. Conflicting guided sweeps wait before entry; followers on the same connection use car-following. Full bodies can swing inside the generated junction footprint, constrained by the road mask. This follows the internal-link structure described in [SUMO's intersection documentation](https://sumo.dlr.de/docs/Simulation/Intersections.html), while remaining a native implementation. The earlier overlap escape was removed; a timeout may release stale reservations but cannot disable body collision checks. See [the replacement model](CAR_FOLLOWING.md).

For restrictions along a road and optional U-turn openings in its middle, see [the interface guide](INTERFACE.md#restricted-lane-sections-and-optional-u-turns). A U-turn opening preserves straight-through routes, lane widths and existing section rules when splitting the road.

## Verification

`junction_connections` checks a tapered one-way road split, straight/side-turn/return-merge route availability, absence of reverse one-way lanes, actual trip completion, concurrent through/turn/merge drainage with physical clearance and no deletion, a fourth arm, opt-in U-turn completion, and saved U-turn permissions.

The car-following release's `zero_radius_junction` test drains mixed vehicles including buses, then verifies that a solid obstacle inside either junction type is respected and traffic resumes when that obstacle clears. `gridlock_escape` now explicitly rejects ghost movement through a sealed blockage. `road_sections` tests actual passing with and without a prohibition, directional meter ranges, split/re-stationing, both save formats and completion of optional U-turn/straight trips. Full validation and the paired controller comparison are linked from [CAR_FOLLOWING.md](CAR_FOLLOWING.md). The previous release's superseded overlap-policy evidence remains archived under `docs/junction-sections/`.
