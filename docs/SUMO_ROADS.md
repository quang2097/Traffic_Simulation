# Roads, junctions and real SUMO

Open `build/traffic_roads.exe`. In **Nodes and paths**, choose **Load Wassup5 road preset**. The supplied PNG remains unchanged in `examples/Wassup5.png`; its authored network is `examples/Wassup5.nodes` and its road interpretation is recorded in `examples/Wassup5-network.json`.

The preset uses the requested **0.73 m/pixel**, right-hand traffic, two lanes per direction, 3.5 m lane widths, and a 13.9 m/s road limit. Northwest, northeast and southwest pairs are separate one-way carriageways. The west, northern spur and both southeast branches are two-way. Seven boundary nodes are explicitly marked as sources at 0.06 vehicles/s each. Other boundary ports are exits. The central junction is signalized in SUMO. The PNG's blue markings are retained visually; they are not treated as independent, conflicting SUMO traffic controllers.

## Run the supplied road

**Build and open real SUMO** saves the current network and vehicle profiles, invokes `netconvert`, generates random-destination demand, and opens `sumo-gui`. Press SUMO's Play button to start. The PNG is a correctly scaled background decal; zoom in to see vehicles, since this image represents an area 1533 × 912.5 meters.

**Run SUMO validation (300 s demand)** runs without a GUI, then allows 1200 more seconds for traffic to drain. Inspect `build/sumo-live/validation.json`, `warnings.log` and `launcher.log`. The validation command fails if collisions, teleports or leftover traffic are detected. Teleport-based jam removal is disabled. A blocked source remains queued in SUMO rather than being silently counted as a successful insertion.

SUMO runs in its own process. Traffic Lab's canvas remains a native simulation/editor preview, and its live controls do not steer an already running SUMO process. Rebuild/open SUMO after changing the network, spawn settings or vehicle distributions. Closing the native preview does not stop an open SUMO GUI. The native preview's manually placed lights remain separate from SUMO's junction signal programs.

## Author another PNG

1. Load the PNG and set **Meters / pixel** to the correct value.
2. Choose **One-way road** or **Two-way road**, **Lanes / direction**, connection width and speed. For parallel carriageways, sketch each separately in its travel direction. For an unpaired road, select Two-way; the compiler offsets opposing lanes onto their respective right-hand sides.
3. Choose **Sketch road**, click shape points along the road centreline, then **Finish road**. The first and last points define junctions. Snap them to existing nodes to join roads. Intermediate clicks shape the road without creating spawns or intersections. A preview follows the cursor. A lane corridor crossing the mask boundary or a divider is rejected; add shape points or reduce lane count/width.
4. Left-click a boundary node and mark **Spawn point** with its rate. The clock belongs to the node; branching does not multiply its demand. Mark **SUMO signalized junction** on intersections that need lights.
5. Ctrl-click connected junctions to select roads. Use **Selected road type**, **Selected lanes / dir** and **Path turn smoothness** to revise them. A two-way road is selectable from either end. SUMO maps incident road smoothing to junction radius; native junction connectors use the incoming road's smoothing.
6. **Connect to node** creates a road between existing nodes without moving them. **Sever outgoing connection** removes the chosen connection. For a two-way road, severing removes both directions of that road; separately authored one-way carriageways remain independent.
7. Save a `.nodes` network, then build/run SUMO. Keep the PNG and `.nodes` file together. You can also load a saved network or import older `.lanes` files.

## Routing and vehicle behavior

SUMO edges are unidirectional; two-way roads generate one edge in each direction. `netconvert` builds lanes, allowed lane-to-lane movements, junction internal links, right-of-way and static traffic-light phases. U-turn generation is disabled. Destination alternatives are chosen uniformly from reachable terminal edges; routes initially use shortest travel cost and SUMO's rerouting device refreshes costs every 20 seconds. Departures choose a suitable lane and begin aligned with it. SUMO handles strategic and cooperative lane changes.

The editor exports its five vehicle classes, mixture, dimensional distributions, desired speeds and acceleration distributions as sampled SUMO types. Standard SUMO car-following applies, with 1.2 s reaction headway and an export minimum gap of 0.7 m. This is different from the native engine's pairwise smaller-preferred-distance rule and mass/tug model. Lane following is used for this map: experimental within-lane sublane filtering produced junction collisions in validation and is disabled. Mixed vehicle types still use and change between the authored lanes.

This is an actual SUMO workflow, not an assertion that the custom C++ engine duplicates every SUMO model. The mask does not uniquely encode road direction, lane count, traffic rules or demand: author those explicitly. It also does not calibrate the model to a specific city. Higher demand or altered geometry can still create legitimate congestion.

## Command line and dependencies

The project-local runtime is excluded from git. To install it on another machine:

```powershell
python -m pip install --target build/sumo-runtime -r requirements-sumo.txt
```

An existing SUMO installation is also supported through `SUMO_HOME` or PATH; `sumolib` must be importable by Python. The graphical launcher uses the Python executable found by CMake. Generated projects use relative references and can be opened independently in SUMO.

```powershell
python scripts/sumo_project.py examples/Wassup5.nodes --output build/sumo-wassup5 --duration 300 --seed 42 --run
python scripts/sumo_project.py examples/Wassup5.nodes --output build/sumo-wassup5 --gui
```

The exporter accepts `--background`, `--meters-per-pixel` and `--profiles` for custom projects. The in-app launcher supplies them automatically.

## Validation

Seeds 7, 42 and 123, each with 300 seconds of demand, completed all 126 trips with zero collisions, zero teleports and no vehicles left waiting or running. Maximum waiting times were approximately 143, 123 and 125 seconds, so legal signal queues remain present. The native road regression verifies two-way lane offsets, explicit spawning, reverse path selection, persistence, stable lane indices and complete lane-strip containment in the PNG. These finite tests are evidence for this preset, not a guarantee for every traffic volume or network.

SUMO's model definitions: [network elements](https://eclipse.dev/sumo/docs/Netedit/elementsNetwork.html), [road networks and junction internal lanes](https://eclipse.dev/sumo/docs/Networks/SUMO_Road_Networks.html).
