# Semantic routing and node labels

Implemented 2026-11-22. New executable: `build/traffic_semantic.exe`. Existing running builds were left untouched.

The previous implementation required target roads to depart the current canonical junction and addressed them by far-end node ID. This prevented semantic connections between separately drawn approaches. New targets store entry-node ID, physical road ID, direction, target lanes and per-lane priorities. Node adjacency, common junction and distance are no longer permission constraints. Geometry and body-fit checks still apply.

The editor expands each typed entry node into all its departing physical carriageways. Blank lane lists leave those roads unlinked. Multiple roads at one entry node remain independent. Existing version 1–10 files preserve their old connection meaning; editing resolves it to explicit directed-road targets. Version 11 persists that identity. Target identities are maintained when roads split or their indices change after deletion.

The compiler produces native free-space transfers for semantic links. Native birth uses directed reachability plus physical free-crossing preflight, so rejected targets never insert a vehicle. SUMO computes semantic trip segments from the same graph, stops at native handoff gates, traverses free space and rejoins the selected target lane. It does not send a disconnected multi-edge trip to SUMO. Priorities and admission consider overlapping semantic movements from different junction groups.

Node IDs are now fixed-size, white-on-dark N-number badges with contrasting borders and leader lines. They are drawn after signals and vehicles, shifted away from hosted lights, and placed to avoid other badges. They remain readable with zoom. The actual marker remains the click target.

## Validation

The full **20-case CTest suite passed**: [summary](semantic-ctest.log), [case output](semantic-cases.log). This includes the original three-seed Native map replay and all existing Native/SUMO signal, handoff, route-rule, UI and engine-switch cases.

After final safety changes, all **19 affected cases were rerun successfully**, with the long unchanged map soak excluded: [summary](semantic-final-ctest.log), [case output](semantic-final-cases.log). The priority test's model/geometry/crossing cases were also rerun directly with `junction_priority_tests rules`, including a semantic conflict between separate junction groups.

New Native and actual SUMO cases use roads that do not share endpoint nodes or an open-junction radius. A car must traverse the free gap into lane 2 of the selected two-way carriageway, then reach the destination. The opposite direction and an isolated trip destination must reject spawning without inserting a vehicle. Native random births across 12 seeds must choose the sole reachable target lane. Roundtrips and pruning preserve road, direction, node, lane and priority. A blocked Native map must reject the same semantic trip rather than cross a wall. Tests record no physical collision or SUMO teleport in the successful transfer.

These checks establish permission, direction, reachability and handoff behavior for the tested cases; they do not guarantee feasibility or jam-free movement for arbitrary drawings.

A Windows visual pass checked the fixed-size badges in the running map and at a newly placed signal station. The selected station ID remained clear above the light arrows and queue annotation. The temporary preview stayed paused and was not saved to user files.
