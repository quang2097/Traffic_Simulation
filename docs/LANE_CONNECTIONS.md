# Semantic lane targets and directional signals

Roads support 1–32 lanes per direction. Lane 1 is leftmost facing travel, with separate numbering for each direction of a two-way road.

## Link lanes semantically

Click the node where a source road ends, open **Lane exits**, then **Edit lane connections on map**. The editor opens in the sidebar so the map remains visible and clickable.

1. Select the incoming road and click a **lane number**. Lane 1 is leftmost facing travel. Only this source lane is highlighted in gold; only its outgoing connections are drawn.
2. Click outgoing lanes directly on the map to toggle them. Hover previews the target and identifies its entry node, direction and lane number. Teal ribbons and arrows show the chosen exits from the gold source lane.
3. Alternatively, choose an outgoing road (`N entry -> N end`) and toggle its numbered lane buttons. You can choose lanes on several roads without typing node IDs or number lists.
4. Click **Apply lane exits** to commit and reset traffic. Apply or **Discard changes** before changing the incoming lane. **Done** returns to map tools; Escape closes the editor and discards unfinished changes.

Expand **Movement priorities (optional)** to set priority for each chosen target lane; **1 is highest**. Choosing a different source lane replaces the preview, keeping other lanes' connections out of view.

Targets need not share a junction, radius, node connection or nearby position with the source. A semantic link allows the native free-space controller to drive from the source lane's end to the chosen directed target lane's entry, then continue along that lane toward the trip destination. Multiple target roads at the same entry node can be selected independently. A two-way road's reverse direction is a separate directed choice.

Existing nodes, roads and lane numbers must exist. Permission does not make walls passable or a physically impossible turn feasible. Birth searches the permitted lane graph, selects only reachable trip destinations and validates its free crossings before insertion. An invalid or physically impossible explicit destination inserts no vehicle. SUMO uses the same semantic connectivity with native transfers between SUMO lane segments; it does not require a SUMO connector across every free-space gap.

Play, Step and manual spawning refuse networks with an unassigned non-destination lane. The notification lists the missing node, road, direction and lane. A declared trip destination can terminate without a continuing exit. An empty target list is not a valid continuing exit.

## Identify nodes

Node IDs appear in dark, bordered **N-number badges**, with white text at a fixed display size and a leader to the actual marker. They are drawn after signals and vehicles. Badges near signal hosts are displaced away from arrows, and badge placement avoids other node badges. The lane editor identifies roads with these same node numbers and their travel direction. Click the actual node marker to open its settings.

## Directional signals

**Place signal node on road** inserts a station on its longitudinal guide. In its **Signals** tab, choose the OUT approach and click **Create perpendicular light**. Select the node's light before **Draw allowed exit**; its permission line remains linked to that particular light and never acts as a trip destination.

Each light shows two arrows on the right-hand side for its own travel direction, with IN/OUT written directly on their shafts. OUT obeys the phase and linked-exit permission; reverse IN passes this light on any color. Collision checks and other traffic rules still apply. **Reverse controlled direction** swaps them. Timing and detector controls remain in the Traffic lights workspace.

Signal nodes near a free-junction boundary do not extend physical lane ribbons into the free area. The fitted stop line reflects the road width at creation; adjust or recreate it after changing widths.

## Priorities and defaults

Overlapping semantic paths participate in Native movement priority even when they originate in different junction groups. Smaller priority numbers take precedence; nonconflicting movements continue. A red-held approach does not reserve precedence over free traffic. **Native simulation -> Junction priority -> Swap priority after a long yield** is off by default. When enabled, it temporarily promotes the oldest eligible vehicle yielding inside the free area after the typed timeout, while immediately obstructing vehicles may clear first. Physical collision checks remain active.

Optional sublane filtering, smooth mixed traffic, RTS refresh, signal spreading, tug assistance, turn negotiation, fleet recovery, vehicle deletion, reservation escape and selected-route visualization default to off. Predictive routing, collision-safe following, ordinary lane changing, signals and automatic spawning remain active.

## Files and checks

Networks save as **.nodes version 11**; signals remain version 4. Versions 1–10 still load. Old connections preserve their old target direction, lane numbers and priority; opening the editor displays their corresponding entry-node addresses, and applying stores explicit target-road identities. Old files still need manual exits where they had none. Earlier application builds cannot read version 11.

The seven-lane example remains available under File -> Examples. Its existing rules are preserved on import.

See [semantic-routing validation](lane-routing-validation/SEMANTIC_REVIEW.md) for test coverage and logs.
