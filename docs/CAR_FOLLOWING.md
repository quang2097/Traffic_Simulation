# Car-following replacement

Use `build/traffic_car_following.exe`. This release retires the preferred-distance bubble from the normal controller and removes the overlap escape bypass.

## What was wrong

The previous engine enforced `min(A.preferredDistance, B.preferredDistance)` as an all-direction physical exclusion zone. That mixed up driver spacing, lateral placement and body collision. A failed movement or reservation often discarded the entire step and set speed to zero. Forecasts assumed vehicles would keep accelerating even when following traffic would make them slow down. Short guided connectors could start turning while the rear still occupied an approach, and funnel parallel motorcycles into a single exit position. Several overlapping planners then tried to repair these stops.

The earlier timeout policy that allowed bodies to pass through each other was an incorrect shortcut. It is no longer available in the normal controller.

## What SUMO actually does

SUMO retains a longitudinal `minGap` and a desired time headway `tau`; it does not replace spacing with an absence of spacing. The car-following model selects a safe speed using the leader and braking capability. See [car-following parameters](https://sumo.dlr.de/docs/Car-Following-Models/index.html) and [road capacity/headways](https://sumo.dlr.de/docs/Simulation/RoadCapacity.html).

SUMO can register a gap below `minGap` as a collision even without body contact. `collision.mingap-factor=0` changes detection to physical contact. Its collision actions include teleport, warn, none and remove; the default is teleport. These options are collision handling, not a normal driving policy of stacking vehicles. [SUMO safety documentation](https://sumo.dlr.de/docs/Simulation/Safety.html)

The sublane model handles lateral occupancy separately and supports motorcycles riding in parallel. Internal connections and junction rules govern competing streams. [Sublanes](https://sumo.dlr.de/docs/Simulation/SublaneModel.html), [intersections](https://sumo.dlr.de/docs/Simulation/Intersections.html)

## Implemented behavior

- **One longitudinal model:** deterministic Krauss-style safe speed, using a follower's standstill gap and time headway, its braking and its leader's apparent braking. The discrete stopping-distance inequality was implemented independently and checked against its closed-form solution in 180 cases. It follows the semi-implicit Euler formulation used by [SUMO's safe-follow-speed core](https://github.com/eclipse-sumo/sumo/blob/main/src/microsim/cfmodels/MSCFModel.cpp); stochastic dawdling is not implemented.
- **Synchronous observations:** longitudinal decisions read the same pre-movement snapshot. Ordered final collision commits remain a safety check. Route-aware leader selection excludes moving vehicles on unrelated junction connections.
- **Spacing is directional:** standstill gap affects following speed, not the physical size of a vehicle. Lateral placement has a separate planning gap. The old preferred-distance field is retained only for compatibility and a controlled legacy comparison branch, not as a production movement constraint. Tests vary that field between 0 and 20 m and verify identical new-model trajectories.
- **Contact handling:** the final check prohibits body penetration. When only part of a proposed step fits, the engine executes that fraction instead of losing all motion. Emergency braking and these safety limits are counted. Existing imported overlap is detected and counted; it does not enable ghost movement. Collision reporting uses actual body overlap, with compressed longitudinal gaps reported separately. There is no automatic SUMO teleport implementation.
- **Junctions:** vehicles brake before a blocked approach endpoint. Forecasts include following-related slowing. Guided curves clear the approach with the rear before turning, preserve usable lateral offsets into the exit lane, and hold conflicting swept connections outside. Followers on a shared connection use car-following rather than reserving the whole junction one vehicle at a time. Guided crossings are checked for geometric feasibility before insertion. The automatic T/four-way extent is at least 12 m when road length permits; explicit extents still override it.
- **Recovery:** the optional timeout can release stale reservations. It cannot disable body collision, road-mask, divider, red-light or lane-section checks. A physically sealed blockage must remain stopped, obtain a genuine route around the obstacle, or use the separately configured deletion fallback. It cannot be made collision-free merely by reducing a gap.

## Controls and counters

In **Vehicles**, edit **Standstill gap**, **Time headway**, **Comfortable deceleration**, **Emergency deceleration** and **Lateral planning gap** for each class. Following parameters update active vehicles of that class. The global lateral planning floor still applies to insertion and sublane searches. Gaussian body sizes, speeds and accelerations remain supported.

The map shows compressed-gap episodes, emergency-braking episodes, final safety-step limits and physical collision events. An episode increments on entry into the condition, rather than every frame. Deletion counters and queue detector lines remain separate. The following overlay is now a forward distance indicator, not an all-direction clearance bubble. **Release stale reservations after timeout** is optional and off by default; its former overlap setting has been removed.

## Validation scope

The comparison and exact regression evidence are in [car-following-validation](car-following-validation/). It compares this controller with the retained legacy branch of this native engine, **not with a running SUMO executable**. Both branches use the same revised compiled network and physical-only collision auditing, with ghost escape and deletion disabled. The archived previous viewer is separate; the legacy branch is not claimed to reproduce that entire binary byte for byte.

Each synthetic four-arm map uses one 3.5 m lane per direction, a fully drivable mask, and either a 22 m free junction or a radius-zero guided junction. There are no traffic lights in this benchmark. Seeds 7, 42 and 123 each offer 60 trips over 30 seconds, then run to 120 simulated seconds. The externally generated origin/destination/class/body/motion request sequence is identical for each paired run. Blocked insertions are skipped. Realized vehicle counts therefore differ. Vietnamese-class shares are demonstration assumptions, not field calibration. Reported mean stopped time includes both finished and still-active admitted vehicles at the same 120-second horizon.

See [results](car-following-validation/results.md) for the six paired cases. All accepted new-controller trips drain without collision or deletion in these cases. This does not establish universal jam prevention. Early throughput and worst individual waiting time do not improve in every case.

This remains a native C++ simulator. Its hand-drawn road masks, body sweeps, free-junction virtual paths and lane-change planner differ from SUMO's lane-coordinate engine and complete LC2013/SL2015 models. The implemented safe-speed core follows SUMO's principles and equations; calling the entire program an exact SUMO implementation would be inaccurate. The full five-seed, 15-minute Wassup6 soak has not been rerun for this release.

Reproduce a paired benchmark:

```powershell
cmake --build build --target car_following_benchmark car_following_tests traffic_ui
.\build\car_following_benchmark.exe new guided 7
.\build\car_following_benchmark.exe legacy guided 7
```

Use `free` instead of `guided` and repeat with seeds 42 and 123.
