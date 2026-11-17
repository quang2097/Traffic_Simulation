# Junction capacity and typed controls

Open `build/traffic_capacity.exe`. The older viewer executable is left available.

## Controls

- Left-click a spawn node. Type its **Vehicles / s** value and press Enter (six decimal places, 0–100 vehicles/s). The rate applies while running; insertion still requires a safe gap. Zero disables births from that source.
- On a node marked **Spawn point**, enable **Exclude this node from its own spawn trips** to prevent new vehicles born there from choosing it as their destination. It remains a destination for other origins. Existing vehicles retain their trips. Other reachable destinations are still randomly chosen. If none remain, insertion fails instead of silently choosing the forbidden origin.
- **Maximum active vehicles** is a typed limit, committed with Enter, from 1–10,000 (default 100). It pauses new births at the limit and never removes existing vehicles. Raising it increases demand and CPU use; it is not a guarantee that a junction can carry that demand.
- **Meters / pixel** is a typed map setting. Use **Apply scale and reset** to apply it. Authored node, road-shape and signal coordinates remain aligned with the PNG. Lane widths, vehicle dimensions and junction radii retain their meter values. The proposed network is validated before replacing the current map; an invalid scale leaves it intact.
- Signal durations are typed seconds, committed with Enter while running. Spawn percentages are typed and use **Apply mix**. Width and motion previews retain their interactive controls.

Save the node network to retain the origin rule and exact rates. Node files are now version 6; engine lane snapshots are version 8 and retain source/destination identities. Older versions remain readable and continue allowing return trips until the exclusion is enabled.

## Admission behavior

**Performance → Higher junction throughput** is enabled by default with the predictive junction controller. Forecasts sample every 0.10 s rather than 0.25 s, retaining the original speed-based temporal clearance cushion. A stopped vehicle holds its occupied corridor rather than closing every approach, allowing physically disjoint movements to proceed. Exit-space checks, road-mask containment, signal rules and swept vehicle clearance remain enforced. The workspace reports cumulative **Junction entries** and the number currently crossing.

An initial variant reduced the temporal cushion and achieved larger short-run gains, but failed sustained mixed-traffic and crossing tests. That variant was rejected. The retained version favors the smaller measured gain with the original cushion.

## Matched comparison

Same map, 80 offered trips per seed, explicit origin/destination schedule, spawn timing, dimensions, speed/acceleration, 80% motorbikes / 20% cars, sublane packing and exit checks. Stuck-vehicle deletion was disabled. `packed` uses the previous admission checks; `compact` uses the retained update. Seeds 7, 42 and 123 were each simulated for 90 seconds.

| Aggregate over three seeds | Previous | Updated |
|---|---:|---:|
| Junction entries by 60 s | 227 | 234 |
| Completed trips by 60 s | 186 | 189 |
| Completed trips by 90 s | 240 | 240 |
| Mean stopped seconds per vehicle, averaged over runs | 3.205 | 2.923 |
| Vehicles deleted | 0 | 0 |

Entries improved 3.1% and early completions 1.6%; one seed admitted one fewer vehicle at 60 s, so this is an aggregate improvement, not an improvement in every run. Clearance and mask checks passed. Wall times in the [raw CSV](junction-capacity.csv) were collected alongside regression tests and should not be interpreted as isolated CPU measurements. These finite scenarios do not guarantee jam-free behavior for arbitrary geometry or oversaturated demand.

From `build`, reproduce a row with `./flow_benchmark.exe packed 42 90` and `./flow_benchmark.exe compact 42 90`. Repeat for seeds 7 and 123. The new `junction_capacity` regression separately checks a stopped crossing and an independent opposite-direction stream, preserving the stopped vehicle's position and preferred clearance. `destination_policy` checks per-origin exclusions, random alternatives, live edits, no-alternative behavior and both save formats.

All 37 targeted regressions passed, including startup mixed traffic (472/472 trips for seed 42, 316/316 for seed 7 and 305/305 for seed 123), native/free-junction handoffs, predictive multi-approach drainage, Wassup5, signals, lane changes and persistence. No vehicle deletion was used in the startup tests. Run the same suite with `ctest --test-dir build -E 'gridlock_|continuous_flow|urban_traffic|^mask$' --output-on-failure -j 3`.
