# Native lanes and open intersections

Use `build/traffic_capacity.exe`. All simulation, pathfinding, collision checks, lane changes, recovery and editing run in the C++ application. Predictive junction movement and continuous sublane packing are enabled by default in the viewer. Allowed-exit lines only grant signal permission; destination nodes determine where trips finish. [See the sustained startup fix](STARTUP_JUNCTION_FIX.md) and [saved signal links](TRAFFIC_FLOW_COMPARISON.md). The earlier Python/SUMO exporter remains an optional legacy tool.

## Draw lanes that occupy space

**Draw individual lane** traces one directional lane exactly along your clicked centreline. **Sketch road** generates multiple adjacent lanes, with either one-way or two-way traffic. Click shape points, snap the endpoints to junction nodes, then finish. Before each click, set **Width at next node (m)**. The click records that width at the new point. The preview between points is a quadrilateral with the previous point's width at one end and the next width at the other. Each section can widen or narrow independently, so a single lane can expand, contract and expand again. The sketch lists widths for points already placed; adjust them before finishing.

After finishing, left-click a junction/endpoint node to edit each incident road's width there, or Ctrl-select its path and open **Widths at every sketch node** to edit interior points. Widths are stored per road shape point, not globally on a shared junction. Geometry edits reset traffic, rather than squeezing existing bodies into a resized lane.

Road lanes are shown as filled ribbons with edges and direction arrows. They are also finite-width corridors in the collision model; a vehicle cannot leave them to overtake into an opposing lane. Mouse-wheel zoom is anchored at the cursor. Middle-drag pans; **Fit map view** restores the overview. At the supplied 0.73 m/pixel scale, zooming is necessary to inspect a 3.5 m lane on the 1.5 km-wide map.

Ctrl-click connected junction nodes in travel order to select road sections. Selected-path controls include:

- Forward and opposing lane counts, independently.
- The width of each forward and opposing lane.
- A lateral offset to position an asymmetric carriageway.
- An end-width ratio for gradual widening/narrowing along the road.
- Turn smoothing and road direction.

Geometry changes pause and reset traffic. The compiler checks guided lane strips against the PNG mask. Shapes that cross black regions or pink dividers are rejected. Draw separate individual lanes when their shapes differ substantially; a multi-lane road uses a shared reference shape with offsets.

## Explicit turn nodes

Shape/width points are **not turns by default**. Left-click a node and enable **This node is a turn** to reveal **Turn deceleration (m/s²)** and **Turn influence radius**. Both can be changed live without resetting vehicles. Marked nodes carry a **T** label. The selected-path shape-point editor also offers a turn checkbox; this creates a selectable control node at that shape point.

Only curved motion inside a marked node's influence area receives authored turn braking. An unmarked shape point and a straight-through connection do not impose turn slowdown. Straight-through traffic at a marked junction still need not brake just because it passes a node. When marking a large open junction in its node popup, the initial influence radius expands to the open-area radius; adjust it to cover the actual turning area. **Smooth mixed traffic** must be enabled for this anticipatory turn controller. Signal, following and collision braking remain active independently.

Older node files load their shape points as unmarked, rather than guessing which points represent turns. New version 5 node files and version 7 engine snapshots preserve flags, rates and radii. Importing engine snapshots retains their explicit turn controls.

## Lane changes

The native controller uses the published [SUMO lane-change motivation categories](https://eclipse.dev/sumo/docs/TraCI/Change_Vehicle_State.html): speed gain, strategic route preparation, cooperation and keep-right. It is a native approximation, not a port of LC2013.

It compares available same-direction adjacent lanes using a short look-ahead traffic forecast. It can change before a slower leader has forced severe braking, prepare for the selected exit, and return right when clear. It ranks alternatives rather than accepting the first lane. A blocked merge can request a moving target-lane follower to ease off and form a gap. Cooperation expires if unused and never overrides physical clearance, signals or lane-width limits.

The merge is a tangent-preserving cubic arc. Its length can span several shape segments, and route-tail trimming removes all points beyond the final stopping position rather than accidentally inserting a backward segment. Those two dense-node bugs previously prevented valid changes. The controller checks current and predicted target gaps, swept body clearance, active crossing merge paths and the remaining target corridor. Opposing lanes are not passing candidates. A cooldown and completion check discourage oscillation and overlapping requests.

The current road's speed limit is applied to its current phase; an already-cleared slow connector no longer caps the entire outgoing road. The old fixed 6 m/s junction cap has also been removed, so straight unmarked nodes do not cause an unexplained speed dip.

Use **Load lane-change demonstration** for a reproducible native example: a 2.5 m/s leader and an 8 m/s follower start in the right lane on a densely sampled two-lane road. Automatic spawning is off. The faster car passes and returns right. The top counter records completed changes, candidate checks and gap requests even after the cars finish. Select a vehicle to see the current decision reason or the precise refusal reason (unsafe gap, no same-direction neighbor, incompatible exit, insufficient merge length, or tight corridor).

The original curved startup demo has only one lane in each direction: its neighboring lane is opposing traffic, so passing there is intentionally unavailable. For normal projects, draw or generate two adjacent lanes in the same direction to allow passing.

## Open red junctions

Left-click a junction node and set **Open junction radius**. A radius of zero retains guided junction links. A positive radius ends the guided approach lanes at that boundary and enables free-space travel inside it. The red PNG remains the physical area: black regions and dividers still block movement. The circle is a planning boundary clipped by that mask; an exit funnel allows vehicles to align before rejoining their exit lane.

Independently drawn approach and exit lane endpoints lying inside the same open area are connected automatically. They do not need to be welded to the circle's central node. Intermediate lane endpoints inside an open junction are not trip destinations.

For newly drawn networks, left-click the intended end node and mark **Trip destination (vehicles finish here)**. These nodes are labelled **D**. Vehicles choose among reachable destinations and finish only after reaching the selected destination, not simply because an approach lane ends. You can also place a standalone D node inside an open area and vehicles will navigate directly to it without needing an exit lane. An unmarked dead end cannot produce a completed trip. Older files keep their legacy outer-boundary destinations for compatibility; upgrading/editing a network preserves those existing endpoints.

A trip is divided into native phases:

1. Follow a finite-width approach lane, changing lanes when a safe useful merge is available.
2. Request admission at the boundary. Wait upstream if the chosen exit buffer is occupied or the proposed crossing conflicts in space/time with an admitted vehicle.
3. Navigate the open road mask toward the selected exit. Vehicles can refresh their paths, avoid bodies, make local detours, and use the existing cooperative recovery/back-up logic. They are not confined to the old thin connector corridor.
4. Align through the exit gate and rejoin the outgoing guided lane with momentum retained.

Admission uses a cached physically valid crossing plan and predicted conflict times. It allows disjoint compatible movements concurrently. An active crossing reserves its exit against new insertion; another vehicle targeting the same exit waits outside. Admission stops when an occupant has stalled for more than two seconds, allowing recovery to work without feeding more traffic into the knot. Vehicles inside open junctions are excluded from the optional stuck-vehicle deletion policy.

The entrance controller does not make congestion physically impossible. An obstructed network or excessive demand can require long upstream queues. No vehicle is teleported or deleted to disguise an unresolved open-junction jam.

## Wassup5

**Load Wassup5 road preset** loads the original mask at 0.73 m/pixel with the confirmed one-way/two-way interpretation. The main junction receives a 160 m open-navigation radius. The approach boundaries and free-area circle are visible in the editor. Sources start at 0.015 vehicles/s per marked node; adjust these live in node popups after checking how your desired traffic mix handles the junction. The old SUMO preset used a different demand/model and is not a performance claim for this native mode.

The red road geometry is preserved. Blue markings are not automatically turned into independent controllers by this preset; use the existing native traffic-light tools if you want signals. Signal timings remain editable in the C++ interface. Traffic should wait before entering a stopped approach, rather than filling the intersection.

Save to `.nodes` to retain the road layout, individual widths, opposing counts, offset, taper, spawn settings and open-area radius. Version 6 node files and version 8 engine lane snapshots also retain per-origin destination exclusion; older files remain readable. See [typed controls and junction capacity](JUNCTION_CAPACITY.md). Node popups still control spawn flags/rates and local braking.

## Verification

Trips select their directed lane sequence and prepare every free-junction crossing at spawn. Only the approach exit → outgoing entry/merge needs free-space planning; existing road geometry supplies the rest. Lane changes rebuild the affected crossing plans while preserving the destination. Signal allowed-exit checks follow the complete remaining trip in order, so a return lane after a U-turn can qualify during red/yellow. The associated stop line must be crossed before its allowed-exit line. U-turn curves carry the vehicle into the junction before turning to avoid swinging its rear into its own queue.

With the previous controller selected, **Junction replan radius (m)** controls nearby crossing recovery (default 25 m). It previews coordinated movement while preserving the live clock and physical positions. The new predictive native controller instead uses prepared curves, admission forecasts and braking-aware following; it does not invoke the previous RTS, tug or fleet-replanning systems. An unavailable exit remains a legitimate upstream wait.

`trip_planning_tests` verifies spawn-time crossing plans, a two-way U-turn during red, rejection of unrelated allowed exits, continuous unblocked junction motion, and local recovery that replans a nearby moving vehicle while preserving distant routes and the live clock.

`native_hybrid_tests` checks lane → open-space → lane handoff, blocked-exit hold/restart, lane metadata round-trips, asymmetric width/taper persistence, useful safe overtaking, physical spacing and complete multi-approach queue drainage with stuck deletion disabled. `native_wassup_tests` runs the native engine against the actual supplied PNG and checks arrivals, mask containment, pairwise clearance and stationary time inside the free area. These are finite regression scenarios, not guarantees for every hand-drawn layout or demand level.

Earlier native crossing runs completed 27, 28 and 28 vehicles for seeds 7, 42 and 123, with no spacing violations and no deletion. The actual Wassup5 PNG run completed its three test trips with no collisions/deletion and zero stationary seconds inside the open area. The red-light approach regression also verifies that a temporary signal queue slot cannot prematurely trigger a junction handoff. Existing lane, turn, signal, urban-flow and recovery checks passed alongside these native tests.


`variable_lane_tests` additionally verifies a 3 → 7 → 2 m width profile and its quadrilateral interpolation, fitting a truly narrowing road mask, saved point widths/destination flags, crossing between detached lanes through a free area, reaching a standalone free-space destination, rejecting unmarked dead ends, and safe overtaking between separately drawn parallel lanes.


`turn_lane_decision_tests` verifies that an unmarked corner remains at 8 m/s while the same corner marked as a turn slows to about 2.4 m/s, live flag/rate updates retain vehicle state, a straight-through node has no fixed junction speed dip, and turn metadata survives both save formats. It also checks moving speed-gain passing across dense shape points, strategic exit positioning, keep-right return and cooperative rear yielding with physical spacing intact. The in-app demonstration was visually checked: both cars finished and the counter recorded two changes.
