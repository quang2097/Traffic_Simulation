# Native traffic controller comparison

These measurements describe the earlier controller revision. The current sustained-startup fixes and Gaussian mixed-traffic regression results are documented in [Startup junction queue fix](STARTUP_JUNCTION_FIX.md).

The viewer now selects the predictive controller with sublane packing. It runs inside the C++ engine. It does not launch SUMO and is not an implementation of every SUMO model.

## What changed

Free-junction trips follow their prepared, continuous curves. A controller checks the outgoing merge buffer before admitting a vehicle and compares sampled future oriented footprints with active crossings. A braking-aware follower looks along the curve, including around bends, instead of treating every nearby vehicle as a leader. Actual movement still passes swept-body, road-mask and signal checks. Native junction actors do not invoke RTS detours, periodic destination reissues, gravity tugging, or whole-turn ownership. Optional authored turn controls still affect braking.

Tracks are binary-searchable arc-length samples. Per-step following copies only the braking horizon, rather than the entire remaining crossing. Full crossing tracks are used at admission. Ordinary road geometry and destination topology are reused; only free crossings need path preparation.

Sublanes are continuous lateral positions within a lane. Spawn insertion searches physically fitting lateral positions. Moving vehicles compare reachable side gaps and validate the entire swept connector and the narrower road farther ahead. A bike cannot jump across another vehicle to occupy an otherwise empty space. Approaches retain gradual positioning; ordinary road passing uses a shorter connector that reaches the side gap before encountering the leader. Side-by-side vehicles retain their positions into the free area, with a straight departure before turning; forcing them to merge at the lane endpoint caused symmetric standstills and was rejected. Terminal lanes also retain lateral positions. Regressions fit three bikes in one 3.5 m lane, reject a fourth blocked insertion, and verify a moving bike passing a slow car within one 4.6 m lane.

The implementation retains the sampled smaller preferred distance because the selected model improved throughput without weakening it. An additional lateral minimum defaults to 0.15 m. Bodies remain finite oriented rectangles, not occupied grid squares. No collision teleport/removal policy was used to improve the measurements.

The reference ideas are [SUMO's sublane model](https://sumo.dlr.de/docs/Simulation/SublaneModel.html), [junction keep-clear checks](https://sumo.dlr.de/userdoc/Simulation/Intersections.html), and [braking-aware car following](https://sumo.dlr.de/userdoc/Car-Following-Models/). The custom controller uses these ideas with the existing authored lane graph and open road mask. It is not an ORCA pedestrian controller or a SUMO backend.

## Controlled comparison

Run on November 15, 2026. Raw results: [flow-benchmark.csv](flow-benchmark.csv).

All runs used the same 130 × 130 m mask, four two-way approaches, 5 m lane widths, 7 m/s desired speed, 2 m/s² acceleration, vehicle dimensions, 80% motorbikes / 20% cars, and fixed per-source origin/destination manifests. Each source offered one vehicle every 2 s during the first 40 s: 80 requests per seed. Blocked insertions stayed in source queues and retried every 0.5 s; they were not discarded or counted as completed. Seeds were 7, 42 and 123. Each run ended at 90 simulated seconds; all used the engine's 0.01 s physical substeps. Signals were absent in all of these runs. Stuck-vehicle deletion was disabled in every model.

| Model | Completed at 60 s, summed over 3 seeds | Completed at 90 s / 240 offered | Mean stopped seconds per inserted vehicle, averaged over seeds |
|---|---:|---:|---:|
| Previous hybrid movement | 55 | 94 / 240 | 16.49 |
| Predictive hybrid, center insertion | 179 | 240 / 240 | 1.16 |
| Predictive hybrid with sublanes | 180 | 240 / 240 | 2.95 |
| Fully lane-guided junction with sublanes | 38 | 38 / 240 | 49.18 |

The lane-guided comparison used the same road graph, endpoints and mask with the open-junction area disabled; its internal connectors constrained movement through the center. It is a comparison with this application's guided-junction model, not with SUMO itself. Removing the open area necessarily changes the allowed trajectory geometry, while demand and trip endpoints remain identical.

Both predictive variants completed every offered trip. Packing was selected because the requested completion-count criterion was slightly higher at the earlier checkpoint (180 versus 179), while it also increases lane storage. This is a near tie, not evidence that packing always wins: center insertion had lower stopped time. Both options remain available in the viewer for other maps and demand levels. The initial packed prototype failed a seed; its endpoint-centering and old crossing-yield interaction were fixed before these final runs.

All measured runs preserved the tested minimum clearances (at least 0.35 m for this vehicle mix), had zero out-of-mask vehicles and removed zero vehicles. Source queues and active unfinished vehicles are included in the CSV so a throughput comparison cannot hide failed insertion or uncompleted trips. Wall times are machine-dependent; each recorded synthetic variant ran sequentially. The CSV includes wall times, but completion counts in simulated time determine the selected model.

## Supplied PNG check

Wassup5 used the original PNG at 0.73 m/pixel, its authored lane graph and a 160 m open-junction radius. Identical fixed requests were offered at nodes 2, 8 and 9 every 5 seconds for the first 15 seconds: nine vehicles, 80% motorbike selection by the same manifest rule. Runs ended at 300 simulated seconds. The manifest's three first requests were cars, so this small sample contains three cars and six motorbikes. Signals and deletion were disabled for every model.

| Model | Completed / offered at 300 s | Mean stopped seconds |
|---|---:|---:|
| Previous hybrid | 9 / 9 | 8.48 |
| Predictive hybrid | 9 / 9 | 3.17 |
| Predictive hybrid with sublanes | 9 / 9 | 3.17 |

All three preserved the mask and clearances and removed no vehicles. This smaller actual-map run checks integration, not capacity saturation. A larger 36-request previous-controller run was interrupted before finishing after several minutes of wall time; it contributes no throughput result. Wall times for the smaller PNG runs were collected alongside regression tests and are not used for CPU comparisons.

## Reproduce

Build with CMake, then run from the project root:

```powershell
.\build\flow_benchmark.exe legacy 42 90
.\build\flow_benchmark.exe predictive 42 90
.\build\flow_benchmark.exe packed 42 90
.\build\flow_benchmark.exe guided 42 90
.\build\flow_benchmark.exe legacy 42 300 wassup 15
.\build\flow_benchmark.exe predictive 42 300 wassup 15
.\build\flow_benchmark.exe packed 42 300 wassup 15
ctest --test-dir build --output-on-failure -j 3
```

Repeat the synthetic commands with seeds 7 and 123. CSV columns are model, seed, simulated seconds, offered, inserted, completed, active, source queue, mean stopped seconds, wall seconds, minimum sampled clearance, removed, completed at 60 seconds. The engine enforces swept movement checks at every physical step; the benchmark additionally checks all pairs and mask containment every simulated second.

## Editing and saving destinations

Green PNG destination strokes are treated as road by the viewer and are no longer drawn by an editor tool. Mark trip destinations using **Trip destination (vehicles finish here)** in node settings. **Allowed exit (selected light)** and **Draw permissive light + exit** create permission lines only: a vehicle whose planned route crosses the linked exit after that light may pass regardless of color, then continues to its actual trip destination. Crossing an allowed-exit line never completes a trip or changes its destination.

Every controller has a stable numeric identity. The allowed-exit geometry and timings are serialized in the same record as its owning light. Saving a PNG or node network also saves `<filename>.signals`; keep the files together. Loading restores exact controller identities and links rather than guessing them from nearby blue pixels. Timing and permission edits remain live without changing trip destinations. Version 2 signal files remove the mistaken destination flag; version 1 files remain readable and their old flag is ignored. Regression coverage includes save/load, controller reordering/removal, permission edits, passing a red light and surviving the linked exit crossing, and migration of old files with the destination flag enabled.
