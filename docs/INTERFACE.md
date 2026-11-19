# Traffic Lab interface

Launch `build/traffic_parallel_departure.exe` for the latest build. It uses a separate filename so the previously opened viewer can keep running.

The top workspace strip selects the properties shown beside the central map. Switching workspaces preserves the current simulation and layout.

- **Simulation:** clock mode, step duration, speed, seed, source limits, traffic behavior, collision workers, recovery and stopped-time measurements.
- **Map design:** road sketching, nodes, connections, lane widths, selected paths, painting and markings. Click a map node to open its properties; Ctrl-click connected nodes to select a path.
- **Traffic lights:** controller selection, timing, signal queues, permission lines and queue detector lines.
- **Vehicles:** class distributions and spawn percentages.
- **Files:** map scale, explicit file paths, legacy lane import and map dimensions.

The persistent toolbar provides Play/Pause, Step, Reset traffic and Fit view. Manual clock mode uses Step. The map remains visible in every workspace. Expand **Live statistics and diagnostics** above the map for detailed counters; select a vehicle to inspect it below the map. View also toggles route and clearance overlays.

**File** offers New map, Open map PNG, Open node network, Save node network, Save node network as, and Export map PNG. Open and Save As use Windows file dialogs. New map asks for dimensions and scale before replacing the current layout. PNG masks and node networks remain separate files: open the mask first, then its network. Save both when editing both geometry and mask. Keep each generated `.signals` companion beside its corresponding file to retain traffic-light settings and linked permission lines. Opening another mask or creating a new map replaces the layout; save your current work first.

The Files workspace retains explicit path controls for existing workflows. Help provides a compact interaction reference. Existing simulation algorithms and data formats are unchanged by this interface reorganization.

## Junction deletion and queue monitoring

The map always displays the current **Open junction** vehicle count and **Deleted** counters: total, stuck queue leaders, and stationary junction vehicles. These count removals separately from completed trips and reset with **Reset traffic**. In **Simulation**, enable **Delete stationary junction vehicles** and type the **Junction stationary timeout (s)**. The viewer starts with this policy enabled at 60 seconds. It counts consecutive simulated time at or below 0.05 m/s after a vehicle enters the free junction; movement or leaving the junction resets its timer. Approach lanes are excluded. Unlike the stuck-leader policy, this junction policy also removes vehicles blocked or waiting at a signal inside the junction. The inspector shows each junction vehicle's timer. Pausing adds no time.

In **Traffic lights**, select a controller and click **Draw queue detector line**, then click its two endpoints on the map. Repeat for as many lines as needed. Draw them nearest to farthest upstream: line 1, line 2, etc. Use **Move nearer**, **Move farther**, or **Delete line** to edit that order. Escape or **Cancel detector drawing** cancels placement. Detectors can be added while traffic runs.

Each detector reports **vehicle YES/NO** using the actual oriented body touching its finite segment. The map highlights occupied detectors in orange. **Occupancy window (s)** and **Occupied threshold (%)** control that light's rolling measurements. For example, with a 30-second window and 60% threshold, line 2 qualifies when occupied for at least 18 of the last 30 simulated seconds. The light's **Queue quality** is the highest-numbered qualifying line, or 0 if none qualifies. Quality waits for a full observation window; the displayed percentage during warmup uses the observed time. Moving traffic also activates a detector, so choose window/threshold values that distinguish brief passages from a queue. Detectors measure traffic; they do not affect routing, signal permissions or destinations.

Saving the PNG or node network writes detector geometry, ordering and thresholds into its `.signals` companion (format version 3). Older versions 1 and 2 still load. Live occupancy histories and deletion counters restart on reset/load. Signal timing edits preserve detector history; changing a detector's geometry or its light's window/threshold restarts the affected history. Detector identities keep their ownership when controllers or detector lines are reordered or other lines are deleted.

## Parallel junction departures

Lane-spawned vehicles retain their original spawn lane and source node throughout their trip, including lane changes and junction crossings. Select a vehicle to see **Trip origin** in the inspector. Vehicles spawned directly on the PNG without a lane have unknown origins (`-1`).

At a free-junction entrance, vehicles from the same source that are beside each other and aligned on the same current approach use relative motion for the prediction cushion. A shared forward speed no longer makes them wait one at a time. The complete predicted crossing is still checked, and the conservative crossing cushion returns wherever their predicted headings diverge. Actual body clearances, red-light rules, occupied exit buffers, U-turn conflicts and stopped-vehicle protection remain enforced. A shared source does not give priority to a vehicle directly ahead, returning on another approach, or taking an intersecting path.
