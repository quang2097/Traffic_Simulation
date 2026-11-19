# Wassup6: five-seed, 15-minute soak at 0.074 m/pixel

Completed 900 simulated seconds per seed on the supplied PNG and node network using the native C++ engine. The previous controller completed **854** trips across the five runs; the measured repair build completed **3584** (**4.20x**). Original input files and saved signal links were left unchanged and their SHA-256 hashes were verified after testing.

The five-seed repair batch below preceded a final lane-edge follow-up. It still left 51 vehicles across two road queues. **The final app includes the follow-up**, which cleared all 51 vehicles in paired reconstructed replays; those trials are described separately below. The final follow-up has not been rerun for the full five-by-900-second batch.

## Conditions

- Inputs: `C:\Users\Uokbr\Downloads\Wassup6.png` and `Node_network_2.nodes`, with its `.nodes.signals` file.
- Scale: **0.074 m/pixel**; 2100 x 1250 pixels = **155.4 x 92.5 m**.
- Seeds: **7, 42, 123, 256, 1024**. Each run has 900 seconds of demand. The repaired runs add up to 240 seconds without new arrivals to test drainage; the baseline runs end at 900 seconds.
- 29 nodes, 9 authored roads, 95 compiled directed lanes/links. Road-mask validation passes. Six saved signal controllers and their allowed-exit links are loaded.
- Saved source rates: nodes 0/4 at 0.25/s, node 6 at 0.16/s, nodes 10/15/16 at 1/s: **3.66 insertion attempts/s, 3,294 attempts per run**. Blocked insertion is skipped as in the viewer; active cap is 100.
- Vietnamese city profiles: 80% motorbikes, 14% cars, 3% bikes, 2% buses, 1% trucks, with Gaussian dimensions and motion parameters. These are demonstration assumptions, not a calibrated field survey.
- Physics: fixed 1/120 s, one collision worker for deterministic comparison; native predictive junctions and sublanes enabled. **Stuck-vehicle deletion disabled** in every run.
- Identical settings and random seeds were used for both controllers. Changed admissions affect the realized population and random draws, so this is not an identical pre-recorded trip manifest.

## Results

Counts below use exactly the 900-second CSV row. Junction stop clock is the largest vehicle stop timer sampled while that vehicle was in the free-junction phase during those 900 seconds. It can include a road wait immediately before admission, so it is an upper bound on uninterrupted junction-only waiting. Drain starts after demand stops and is not included in trip-throughput totals.

| Seed | Before: trips | Measured repair: trips | Max stop clock on junction vehicle, before / repaired (s) | Measured repair trips in last 120 s | Extra drain (s) | Vehicles left after drain |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 7 | 147 | 738 | 737.5 / 37.5 | 94 | 154.3 | 0 |
| 42 | 341 | 703 | 3.5 / 38.3 | 87 | 157.3 | 0 |
| 123 | 164 | 702 | 722.2 / 81.5 | 94 | 240.0 | 28 |
| 256 | 101 | 713 | 812.1 / 24.2 | 89 | 164.7 | 0 |
| 1024 | 101 | 728 | 861.7 / 40.9 | 99 | 240.0 | 23 |

Across the repaired runs, **4024** vehicles were admitted and **3973** completed by the end of the drain checks; **51** remained. Raw counts distinguish admission capacity, completion throughput and drainage rather than treating every long queue as an engine deadlock.

## Problems found and repairs

1. **Competing signal and junction planners.** The legacy red-light spreader could overwrite a committed native crossing with an RTS detour. Native traffic now keeps that route and brakes for signals along it, including the cached crossing beyond a temporary lane endpoint. Legal red-light stops are excluded from lane-escape recovery.
2. **Incoming-mouth head-on conflicts.** A crossing could nose into an incoming stream just beyond its lane endpoint. Crossing geometry now protects incoming lane corridors behind their stop lines and the approach mouth. It accounts for the full rotated body of an angled vehicle, including its nose, rather than only its nominal width. Turns use the open junction area instead of cutting through another incoming queue. The straight entry prefix clears the farthest nearby parallel stop line with the rear bumper, including staggered/asymmetric approaches. Admission checks the entire committed crossing against actual road vehicles, rather than just its first step. A long entry lead cannot enter and then stop across the neighboring queue’s departure corridor.
3. **Front-bumper stop alignment.** Approaches to a free junction now end with the front bumper before the approach endpoint, preventing long queued vehicles from projecting halfway into the crossing and trapping a shorter turning neighbor.
4. **Curved/tapered sublane handoff mismatch.** Junction previews used a nominal road endpoint and heading instead of the installed vehicle trajectory's actual endpoint and tangent. Previews now use the committed pose, and lateral endpoint offsets use the end tangent.
5. **Unusable lane trajectories.** Native lane paths are checked with full footprints along their interpolated trajectory before acceptance. Alternative curve scales are tried when the initial curve leaves the lane envelope; an unusable path is rejected before commitment.
6. **False spacing and reservation blocks.** A desired lateral planning buffer was being imposed as an extra hard movement gap. Insertion and slot planning retain their buffer; physical movement enforces the smaller pairwise preferred distance. A following merge's future claim cannot trap a slightly leading parallel vehicle.
7. **Packed rotation and unfinished lane changes.** Native lane traffic can execute a short, checked forward or reverse clearance prefix before rejoining its committed route, including when a lane change is still active. The full body must fit the target lane before merge state ends. Static junction admission now checks continuous swept bodies at the smaller preferred gap, using the actual committed tangents, instead of inflating sampled poses by an additional 0.325 m. Recovery leases expire instead of reserving a neighborhood forever. Every executed step still checks the mask, lane envelope, signals and physical bodies.
8. **Tiny approach-end pinches.** After a verified two-second stall within half a vehicle length of the approach endpoint (bounded to 0.5-2 m), junction planning can hand off from the actual pose instead of requiring an artificial final alignment. Admission, mask, signal and physical-body checks still apply.
9. **Near-contact numerical collision error.** A seed-7 footprint replay rejected a 0.1 mm separating move while accepting a 15 cm move. Small polygons now use exact edge distances instead of the iterative clearance-disk calculation. The regression also verifies that moving closer remains rejected.
10. **Road-mask query cost.** Exact integral-mask pruning skips empty or distant pixel rectangles while keeping the original leaf test. All 2,000 sampled queries on this PNG match the original pixel loop. This is a mask-query optimization, not a measured claim about total UI FPS.

## Final lane-edge follow-up

Both residual queues were on the northeast approach, outside the free junction. The full repair batch drained seeds 7, 42 and 256 completely; seeds 123 and 1024 retained 28 and 23 vehicles.

A nominal curved/tapered lane envelope could accept a 15 cm departure endpoint while rejecting its first 1 mm physics step. Recovery kept selecting that unexecutable departure. The final code:

- Gives native road envelopes half a mask pixel of tolerance, capped at 5 cm (3.7 cm on this map). Physical road-mask boundaries, solid dividers and the smaller pairwise preferred clearance remain enforced.
- Checks recovery departures at 2 mm / 0.001 rad resolution, bounded to the initial departure region, and can rebuild toward the lane centre instead of rejoining an unusable lateral tail.
- Includes a portable `lane_edge_recovery` regression reduced from the captured coordinates. The measured controller fails it; the final controller passes it.

Paired replays use the same captured terminal poses, body dimensions, preferred gaps, remaining routes and paths. They start all vehicles stationary, clear unrecorded claims/cooldowns, disable new lateral searches, and infer the unfinished lane change's source lane and completion point. These are **reconstructed geometry/liveness tests, not complete engine checkpoints** or a new throughput benchmark. The saved map, scale and signals are reused. Each pair permits 120 simulated seconds with no spawning or deletion.

| Captured seed | Vehicles | Measured controller: completed / remaining | Final controller: completed / remaining | Final clearance time |
| --- | ---: | ---: | ---: | ---: |
| 123 | 28 | 0 / 28 | 28 / 0 | 96.22 s |
| 1024 | 23 | 0 / 23 | 23 / 0 | 77.34 s |

No audited mask/clearance violations or deletions occurred in these paired trials. See [follow-up validation](wassup6-soak/followup-validation.json) and the four replay logs. `scripts/replay-soak-snapshot.py` reconstructs and runs these cases; it documents the state omitted from the log.

## Network configuration issue

**Destination node 2 is unreachable.** Its one-way road points **2 -> 20** and no road enters node 2. Give that destination an incoming connection, or unmark it as a destination if it is intended only as an entry. Destinations 6, 9, 10, 13 and 16 are reachable. No speculative direction changes were applied to your network.

## Validation and limits

- Across all five measured repair runs including drainage: **0 mask violators, 0 violating pairs, 0 deleted stuck vehicles**.
- Vehicle footprints, preferred pair clearances and zero-preference physical overlaps are audited every simulated second; see each CSV's cumulative violation columns. Normal engine movement also performs its collision checks per physics step.
- High offered demand can still produce capacity queues and skipped admissions. A finite five-seed soak cannot guarantee that every future network, seed or demand level is deadlock-free.
- The retained pre-repair controller uses the exact mask acceleration to keep the comparison practical. **45 shared CSV checkpoints** across all five seeds match the original unaccelerated executable in every recorded field; see `reference-validation.json`. Intermediate diagnostic runs were excluded from this comparison.
- Final regression suite: **49 of 50 tests passed under CTest**. `gridlock_long_run` reached its unchanged 240-second wall-clock timeout. The exact same test, run directly with unchanged assertions, **passed in 252.5 seconds**: all 57 admitted vehicles arrived, none remained, and the simulation drained at 150.725 simulated seconds. This establishes a functional pass, but the suite is not fully green under its existing runtime budget. See [CTest log](wassup6-soak/final-regressions.log), [standalone log](wassup6-soak/long-run-standalone.log) and [regression validation](wassup6-soak/regression-validation.json).
- Final executable/controller hashes and the unchanged original input hashes were rechecked; see [integrity validation](wassup6-soak/final-integrity-validation.json).

## Reproduce

In the native viewer (`build/traffic_wassup6.exe`), set **Files -> Meters / pixel** to **0.074**, then use **File -> Open map PNG** for `Wassup6.png` and **File -> Open node network** for `Node_network_2.nodes`, in that order. The companion `.nodes.signals` file loads with the network. Set the scale before importing the node network, so a later scale change does not rescale its already-authored world coordinates.

The commands below run the **final follow-up build**, whose random admissions and totals can differ from the measured repair table. Its archived measured runner is `build/wassup6-soak/corridor-map_soak.exe`; the measured viewer is `build/traffic_wassup6_soak.exe`.

Build the native viewer or headless runner:

```powershell
cmake --build build --target traffic_ui map_soak
.\build\map_soak.exe 'C:\Users\Uokbr\Downloads\Wassup6.png' 'C:\Users\Uokbr\Downloads\Node_network_2.nodes' 0.074 7 build/wassup6-seed7 900 drain
```

Repeat with seeds 42, 123, 256 and 1024. `detailed` in place of `drain` adds pose/path diagnostics. To build the retained reference, run `python scripts/prepare-soak-reference.py`, reconfigure CMake, and build `map_soak_baseline_fast`; it is never selected by the viewer.

The [manifest](wassup6-soak/manifest.json), [summary](wassup6-soak/summary.json), per-seed CSVs and logs under `docs/wassup6-soak/` preserve the measured results. CSVs contain drain rows after 900 seconds; use the 900-second row for throughput comparisons. The final empty state may fall between CSV samples; each log's `DONE` record provides the exact drainage outcome.

Replay a captured residual group against the final engine (MSYS2/UCRT g++):

```powershell
python scripts/replay-soak-snapshot.py docs/wassup6-soak/repaired-123.log 'C:/Users/Uokbr/Downloads/Wassup6.png' 'C:/Users/Uokbr/Downloads/Node_network_2.nodes' --output build/replay-123
```

Use `repaired-1024.log` for the other group. Add `--library build/wassup6-soak/corridor-libtraffic.a` to run the retained measured-controller control; it is expected to return nonzero because the queue remains stuck.
