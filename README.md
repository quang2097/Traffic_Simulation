# Traffic Lab — Native and SUMO

The C++/ImGui application includes both implementations. Open **Native simulation** or **SUMO simulation**, then click **Use Native** / **Use SUMO**. Browsing a section does not change engines. Activation resets vehicles and counters, preserves the common map, signals and vehicle profiles, and pauses.

- **Native** runs the previous C++ traffic controller, including native lanes, tapered widths, restriction sections, recovery controls and free junctions.
- **SUMO** runs actual headless Eclipse SUMO 1.28.0 through C++ TraCI for lane traffic, while keeping native free-junction handoffs. See [SUMO import boundaries](docs/SUMO_BACKEND.md).
- **Help → User manual**, the toolbar's **User manual** button, or **F1** opens the searchable in-app guide. Instruction paragraphs have moved there; the sidebars contain controls and live data.
- An active map tool shows **Cancel (Esc)** at the bottom-left of the map. It cancels connection, sever, sketch, insertion, painting and line-placement modes without changing committed geometry. Sketches also support Undo last point.
- Roads support up to **32 lanes per direction**. In a node popup, **Lane exits** assigns each incoming lane to specific outgoing node IDs and lane numbers. **File → Examples → Seven-lane routing** demonstrates this. See [lane connections and directional signals](docs/LANE_CONNECTIONS.md).
- Lane exits opens a visual editor beside the map: select one incoming lane by number, then click outgoing lanes or their numbered buttons. Gold marks the source; teal arrows show only its chosen exits. Apply or discard before switching source lanes. Links cross free road-mask space even when roads have no common junction. Birth chooses only reachable trip destinations. Fixed-size node badges stay visible above lights and traffic.
- Play, Step and manual spawning require explicitly assigned exits for every non-destination lane. The notification lists missing node/road/lane settings. Set per-target-lane priority in **Lane exits**; **Native simulation → Junction priority** offers optional starvation relief, off by default.
- **Place signal node on road** inserts a station on the road guide. Its popup creates a perpendicular light and links an allowed-exit line. All lights now have an **OUT** direction that obeys the signal and a reverse **IN** direction that passes freely; use **Reverse IN / OUT** to reverse them.

```powershell
# Needed only for SUMO mode if its runtime is not already installed:
powershell -ExecutionPolicy Bypass -File scripts/install-sumo.ps1
cmake -S . -B build -DTRAFFIC_UI_OUTPUT_NAME=traffic_semantic
cmake --build build --target traffic_ui sumo_ui_smoke sumo_integration_tests lane_connection_tests semantic_lane_tests junction_priority_tests -j 4
ctest --test-dir build --output-on-failure
.\build\traffic_semantic.exe
```

Native mode does not start a SUMO process. SUMO searches `SUMO_HOME`, `sumo-runtime/sumo` beside the executable, then this project's `build/sumo-runtime/sumo`. A missing runtime does not silently switch engines: choose Native explicitly if desired. Python/pip is only the runtime installer; the running application and bridge are C++.

Use File for opening/saving PNG maps and .nodes networks. Keep their .signals companions together. The common vehicle profiles, independent spawn-node rates, light timings and detector controls work with the active implementation. The toolbar identifies that engine. Model-specific native controls appear only in its own simulation section.

[UI and engine-selection guide](docs/DUAL_ENGINE_UI.md) · [SUMO backend](docs/SUMO_BACKEND.md) · [Earlier native documentation](docs/LEGACY_NATIVE_README.md)
