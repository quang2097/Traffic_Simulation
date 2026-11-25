# Native directional signals and editor cleanup

Validated 2026-11-22.

The displayed IN/OUT labels were positioned at arrow destinations. This put the IN label on the controlled OUT approach side, making the visual meaning contradict the movement direction. Labels now identify arrow origins and explicitly say `OUT: signal` / `IN: free`. Selected arrows and labels keep a readable size across map zoom levels.

Native's swept-body stop check also tested a red line when a vehicle center was already beyond it and moving farther away. It now checks the controlled approach side before testing the swept footprint. Departing or reverse IN movements cannot acquire a new stop from their rear sweep. An OUT approach remains blocked before its front crosses the line.

Expanded Native regression coverage uses the actual generated perpendicular road-station signal, two-way traffic, live reversal of a red light, four road orientations, and predictive routing both enabled and disabled. IN vehicles must complete while OUT vehicles remain stopped; reversing the direction must release the waiting vehicle. Directional movement-side checks cover approaching, reverse and departing motion. Existing linked-exit release and persistence tests remain active.

Additional free-junction checks passed in four orientations using the application's predictive Native controller, including reversal while the OUT vehicle waits. The all-road fixture deliberately does not constrain legacy free-mask detours around a finite signal line, so that combination is excluded from the free-junction assertion. The final Native and editor rerun is recorded in [targeted checks](native-direction-ui-final.log).

The broader reported IN stoppage was not reproduced in these fixtures. The verified display mismatch and departing-body check were corrected; unrelated physical queues still yield to traffic normally.

Node properties are grouped into General, Roads, Lane exits, Signals and Junction tabs. The Signals panel selects an OUT approach, creates a perpendicular light, then exposes the selected light's reverse direction and linked-exit actions. Traffic-light sidebar sections separate creation, travel direction, timing, allowed exits, queue detectors and Native queue planning. Automatic lane exits use a compact summary; road creation settings precede sketch tools, with node tools grouped separately.

The full 17-case suite passed with local SUMO process connectivity enabled. See [CTest summary](native-direction-ui-ctest.log) and [case output](native-direction-ui-cases.log). The initial sandbox run could not connect to local SUMO processes; the full suite was rerun outside that sandbox. UI checks additionally render every node panel, the selected-node signal controls and the signal sidebar. A native Windows visual pass checked tab selection, creation and direction labels, and the grouped signal settings.

Build output: `build/traffic_lab.exe`. The previous `traffic_lane_connections.exe` entry point is refreshed to the same build for continuity.
