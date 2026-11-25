# Native map review, lane exits and priorities

Reviewed 2026-11-22 using copies of `Downloads/custom-map.png`, `custom.nodes`, and `custom.nodes.signals` at **1.0 m/pixel**. Original files were not modified.

## Findings

- Light L0 at the west station (node 5) controls **westbound** travel (`OUT = -X`). It therefore stops vehicles leaving the junction toward the west on red. Eastbound entry is its free IN direction. The other three lights control travel toward the junction. This is an inconsistent approach orientation, not a failure of Native's reverse-direction phase check. Use Reverse controlled direction if the west light should govern entering traffic.
- Every incoming lane was using legacy automatic exits. The new run gate reports missing assignments at **nodes 5, 7, 6 and 8**. Set the outgoing nodes, target lanes and priorities in each node's Lane exits tab. Declared trip destinations can terminate without a continuing exit. Play, Step and manual spawning refuse incomplete networks in both engines.
- A split shortened the road adjacent to the free junction. The old `40% of segment length` cap then shortened its junction setback, extending physical lane ribbons into the open area. Free-junction setbacks no longer shrink with a signal split. Signal stations within half a lane width of the boundary share the existing free junction, and redundant internal road stubs do not compile into physical lanes. The four station approaches in this map keep their intended ends.

## Changes

IN and OUT are printed directly on two opposing arrows, each on the right-hand side for its own travel direction. Both arrows remain visible and keep their display size across zoom levels.

Each source-lane → outgoing-node → target-lane connection has a saved priority. **1 is highest**. The Native free-junction controller compares remaining committed paths, makes conflicting lower-priority traffic yield, and lets nonconflicting movements continue. Approaches held by a red light do not claim precedence. Existing following and body-clearance checks remain active.

**Native simulation → Junction priority → Swap priority after a long yield** defaults to **off**. When enabled, the oldest eligible vehicle yielding inside the free area receives a temporary grant after the chosen timeout. Other conflicting movements yield until it leaves; vehicles immediately obstructing its departure may first clear. Legal priority waits are excluded from queue-leader deletion, open-junction timeout deletion and stale-reservation escape.

Optional sublane filtering, smooth mixed traffic, RTS routing/refresh, red-light spreading, tug assistance, turn negotiation, fleet recovery, vehicle deletion, reservation escape and selected-route visualization default to off. Predictive routing, collision-safe following, ordinary lane changing, signal enforcement and automatic spawning stay active.

Networks save as `.nodes` **version 10**; versions 1–9 still load, with priority 1 for connections lacking a saved priority. Existing files require manual exit configuration before running. Signals remain version 4.

## Validation and limits

All **18 CTest cases passed**: [summary](priority-ctest.log), [detailed output](priority-cases.log). New checks cover priority ordering even when the lower-priority vehicle has the older ID, an actual crossing with high-priority arrival first, no physical collision, nonconflicting paths, default-off starvation relief, a temporary grant, grant release, required-exit rejection, station geometry and priority persistence. UI checks cover the refusal notification and all node panels.

An additional replay used the new optional-off settings and Vietnamese vehicle mix, preserving the supplied signal directions. Three seeds ran for 90 simulated seconds at the network's 0.5 vehicles/s per source, subject to insertion availability:

| Seed | Arrivals | Active at end | Physical collisions | Deleted |
|---|---:|---:|---:|---:|
| 7 | 62 | 52 | 0 | 0 |
| 42 | 25 | 58 | 0 | 0 |
| 123 | 21 | 56 | 0 | 0 |

Logs: [mixed replay summary](priority-mixed-native.log), [case output](priority-mixed-native-cases.log). A preceding car-only replay yielded 17/31/40 arrivals with no collisions or deletions. The mixed replay took about 140 wall-clock seconds for the combined three runs. Throughput remains seed-sensitive; these short runs do not establish jam-free behavior. The core diagnostic replay intentionally permits the original automatic routes so the unchanged case can be investigated; the interactive UI correctly refuses that network until its exits are assigned. The replay does not invent the user's desired priorities or silently change their lights.

Build: `build/traffic_priority.exe`. An older `traffic_lab.exe` was still open and was left untouched. The new preview contains the supplied map, paused, without saving edits to Downloads.
