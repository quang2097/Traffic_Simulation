# Native / SUMO interface

Run `build/traffic_lab.exe`. The application has six workspaces: Native simulation, SUMO simulation, Map design, Traffic lights, Vehicles and Files.

## Choosing the implementation

Each simulation workspace has its own activation button. Selecting a workspace only opens its controls. **Use Native** or **Use SUMO** changes the active implementation, clears traffic/statistics and pauses. The toolbar always names the active engine. Map geometry, signals, spawn rates and vehicle profiles are shared and preserved across switches.

Native runs the existing C++ controller directly, with native driving, recovery and advanced controls in collapsible groups. It does not initialize or start SUMO. Native tapered widths and lane-change restriction sections are editable again in this mode. SUMO attaches the actual SUMO backend; its importer limitations remain unchanged. A SUMO failure stays visible as a status error, with Native available as an explicit choice.

Clock/spawn controls belong to the active engine. An inactive engine's controls are disabled until activation. Engine changes reset the seed's random sequence; browsing sections does not reset the simulation.

## Cleaner workspace

- Explanatory paragraphs are in **Help → User manual**, the toolbar's manual button, or **F1**. The guide has a topic list and full-text search and stays inside the application.
- Native driving/recovery/advanced controls, SUMO diagnostics and monitoring are collapsible.
- The map has a compact live summary; expanded statistics remain available on demand.
- The vehicle inspector opens when a vehicle is selected and can be toggled through View.
- Result/error messages occupy one status line; hovering reveals the complete message.
- Numeric controls include units, with long typed-field labels above their inputs.

## Canceling map operations

Whenever an editor tool is active, a fixed **Cancel (Esc)** bar appears at the bottom-left of the map. It works for connecting nodes, severing a connection, sketching roads, inserting nodes, placing nodes, painting/erasing, stop lines, allowed exits, dividers and queue detectors.

Cancel clears the pending source node, connection/sever mode, unfinished sketch and width samples, incomplete line stages and active drawing flags. It keeps committed nodes, roads, signals and paint. The overlay excludes its rectangle from canvas hit testing, so clicking Cancel does not also paint or place a point behind it. Escape invokes the same cancellation path. With no active tool, Escape closes the manual.

Sketch road also has Undo last point. Node placement remains active after adding a node, until canceled or another action is selected. Starting playback returns to inspect mode.

## Verification

UI tests run actual ImGui frames with both controllers. They check Native startup, SUMO startup, switching in both directions, preserving map/profiles and pausing/resetting correctly, browsing without switching, manual opening/search/closing, and actual mouse clicks on the Cancel overlay for nine editor modes. The cancellation tests also check unchanged committed map data and Escape behavior. Existing SUMO routing, Gaussian profiles, signal, free-handoff and Map_demo regressions remain in the default test suite.
## Shared Tr for normal traffic lights

Open **Traffic lights**, select a light and expand **Normal lights: shared Tr**. Select the junction, set **Tr: shared all-red (s)**, then click **Enable shared Tr for junction**. Signals with road-station links are associated automatically; check the junction assignment for manually drawn signals. The action includes enabled linked lights at that junction and pauses traffic without resetting vehicles or time. Press Play to continue.

Opposing horizontal approaches initially use Group 1; vertical approaches use Group 2. Adjust **Shared cycle group** when the junction needs another pairing. Each group takes its green and yellow, followed by the next group's full Tr before green. All participating lights remain red during each Tr, including otherwise permissive OUT movements. IN-direction rules remain unchanged. A yellow guard precedes startup clearance if any participating light was green/yellow. Pausing freezes the countdown.

Green/yellow edits apply to matching group members. Red time and phase offsets are calculated by the shared cycle; the original independent red/offset values are retained for **Disable shared Tr**. For imported files with inconsistent group durations, the group's longest green, yellow and Tr values are used. Routine signal updates and reordering preserve the running clock; changing shared timing or membership starts a guarded cycle.

The phase display shows **ALL RED: Tr remaining**. `.signals` version 6 saves the Tr duration, junction and group; older files retain independent cycles until enabled. Native and SUMO use the same schedule. RL/training control temporarily owns signal timing and does not add the normal Tr on top of its own clearance. Stopping those controllers restores shared normal timing with a guarded restart.
