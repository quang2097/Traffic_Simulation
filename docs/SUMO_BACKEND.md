# Actual SUMO backend

`build/traffic_lab.exe` is the current ImGui application, with separate Native and SUMO sections. This document describes SUMO mode. `traffic_demo` also uses this backend. SUMO remains a separate headless C++ process connected over loopback TCP using the TraCI protocol. This avoids mixing the MSVC ABI of the official runtime with the project's MinGW ABI. The application, exporter, protocol client, handoff and viewer are C++17; there is no Python simulation bridge and no SUMO GUI.

## Ownership

| Area | Controller |
| --- | --- |
| Directed road lanes and lane selection | SUMO |
| Fastest route to a randomly selected reachable destination | SUMO `findRoute` |
| Longitudinal motion | SUMO Krauss |
| Lane changes, cooperative gaps and parallel two-wheelers | SUMO SL2015, 0.25 m sublane resolution |
| Guided T/cross/U-turn junction connections | SUMO internal links and conflict logic |
| Signals on guided movements | SUMO, with authored phases supplied by C++ |
| Explicitly open junctions | Retained native curve planner and free-space movement |
| Editing, rendering, stopped-time and queue-line telemetry | Current C++ application |

SUMO steps at 0.1 simulated seconds. Free motion uses ten 0.01-second substeps per SUMO step. Both clocks pause together. Gaussian dimensions, desired speed and acceleration are sampled once per new vehicle and applied to a SUMO type before departure. Five reusable sampling templates are isolated into vehicle-specific types on birth, so later samples cannot change earlier vehicles. Live edits update its SUMO parameters; positions are read from SUMO, never recomputed by the old lane controller. Headway is at least the 0.1-second engine step.

In SUMO mode, native preferred-distance bubbles, tugging, periodic RTS replanning, lane-side recovery and native lane deletion do not run. Historical code remains as dependencies of the retained free-space planner and as optional archived tests. `SimulationView` attaches `SumoBackend` only when SUMO is selected. Native mode runs the retained controller directly. The console entry point still uses SUMO.

## Free-junction handoff

Before requesting a departure, the exporter checks every open crossing on the SUMO route against the retained planner and the sampled vehicle body, including centered and offset sublane positions. An impossible native turn is rejected before the vehicle can block a lane. If a randomly chosen destination has no feasible crossing, another reachable destination is tried. An explicitly requested impossible destination is rejected.

On approach, a normal SUMO stop protects the gate. A vehicle enters only when its signal permits it, entry space and the exit buffer are clear, and the native admission check accepts the crossing. Conflicting committed free-space corridors are mutually exclusive; disjoint crossings may proceed concurrently. This is deliberately conservative for this first hybrid baseline.

Near a free gate, SUMO mode 512 prevents starting a new lateral maneuver while retaining its collision avoidance and gap checks. It preserves the current sublane position. Normal lane changing resumes after reinsertion. The handoff uses the lane-aligned occupancy pose used by the longitudinal/lateral model, rather than interpreting the display angle from a lateral movement as a rotated rigid collision rectangle. Off-center entries use a continuous prefix that clears the rear of the entry row before turning.

The incoming SUMO vehicle temporarily acts as a queue guard until the native vehicle's rear clears its entry. That guard is then removed without recording an arrival. At the exit, the same ID is safely reinserted on the remaining SUMO route at the actual front-bumper station, with its speed retained. The native body remains visible while insertion is pending. A handoff never completes a trip, changes the destination, or treats an allowed-exit line as an arrival gate. Repeated open crossings retain the full route stage.

## Signals and monitoring

Manual signal lines bind to incoming movements. The linked allowed-exit line grants a permissive green state to the corresponding movement through red/yellow; SUMO still checks conflicts. It is not a destination. Changes to durations are supplied on the next 0.1-second step. Uncontrolled movements retain SUMO priority rules. Signal nodes without manual lines can use SUMO's generated program.

Stopped-time statistics use simulated time. Detector occupancy uses the rendered oriented body. Pending initial departures and vehicles in SUMO teleport recovery are not counted as road occupancy. Pending exit reinsertion remains represented by its native body and continues accumulating stopped time. None of these intermediate states completes a trip. Open-junction timeout deletion is retained as its own optional policy and counter.

The UI separately reports SUMO collision involvements and teleport starts. SUMO collision handling uses `collision.action=teleport`, junction collision checking is enabled, and the normal 300-second gridlock teleport timeout is enabled. These are engine recovery actions, not permission to stack bodies or successful arrivals. Native overlap telemetry is available separately.

## Network import boundaries

This is the actual SUMO engine, not a claim that every historical editor feature has an exact SUMO equivalent:

- SUMO's generated lane shapes and right-hand lane numbering replace native lane strings. The viewer uses those actual shapes. Native coordinates are converted between screen-down Y and SUMO-up Y without normalization offsets.
- Multiple authored ports inside the same open-junction radius are combined into that junction for SUMO topology.
- Widths are currently constant per SUMO lane. An old taper is imported conservatively at its minimum width; the old continuously varying quadrilateral is not a varying-width SUMO lane.
- Native per-position lane-change prohibition sections are not imported in this baseline. SUMO controls normal lane changes. The editor labels this limitation; these restrictions need an explicit SUMO edge-splitting implementation before they can be restored.
- Native per-node braking affects free junctions only. SUMO computes guided-junction turn speeds from its own geometry and turn-speed rules.
- A manual line selects movements at a SUMO junction; the precise SUMO stop position is the generated lane end. It does not split a road at an arbitrary painted signal position.
- Road lanes follow the authored SUMO geometry; the PNG remains the background and the hard free-junction mask. Free-space collision and recovery behavior remains a separate controller, so the hybrid is not identical to a wholly SUMO-controlled network.

Generated network files and SUMO/netconvert logs are kept under `build/sumo-live/<session>/`. Runtime errors pause the simulation. No shell is used to execute either tool; each session owns and closes only its own headless SUMO process.

## Validation

`sumo_integration_tests` exercises actual runtime traffic and fails on incomplete drainage, collisions or teleports. Cases cover a guided four-way network, native open crossings, mixed vehicle classes, repeated handoffs, live red-to-green edits, and permissive gates on both guided and free movements. The tests keep deletion disabled, so drainage cannot be credited to removing stuck vehicles. See `docs/sumo-validation/` for captured results from this implementation.

Historical native-controller benchmarks are not SUMO comparisons. This change replaces that controller instead of claiming numerical equivalence to it.

## Upstream references and licensing

- [TraCI protocol](https://sumo.dlr.de/docs/TraCI/Protocol.html)
- [SUMO sublane model](https://sumo.dlr.de/docs/Simulation/SublaneModel.html)
- [SUMO vehicle-state commands](https://sumo.dlr.de/docs/TraCI/Change_Vehicle_State.html)
- [SUMO collision handling](https://sumo.dlr.de/docs/Simulation/Safety.html)

The runtime is Eclipse SUMO, distributed under EPL-2.0 with its stated secondary-license terms. Keep its license and notices with the runtime when distributing the application. This project does not relabel SUMO as a newly written simulation engine.
