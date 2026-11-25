# Dual-engine UI validation — 2026-11-20

The full default suite passed **12/12** tests. After the final label cleanup, the four UI regressions passed again.

| Check | Result |
| --- | --- |
| SUMO startup through actual ImGui frames | Pass |
| Native startup without attaching SUMO | Pass |
| SUMO → Native → SUMO switching | Pass |
| Switching resets traffic, pauses and preserves design/profiles | Pass |
| Browsing a section does not switch engines | Pass |
| Mouse cancellation for nine editor modes | Pass |
| Cancel preserves committed map data and clears stale connection/sketch state | Pass |
| Escape cancellation and in-app manual search/open/close | Pass |
| Existing SUMO lane, Gaussian mix, signal, free-handoff and Map_demo regressions | Pass |

`ctest.log` is the full-suite summary, `cases.log` contains detailed output, and `ui-final.log` records the final rebuilt UI checks.

The rebuilt Windows application was also inspected interactively: the engine sections, explicit Native activation, manual, map-design popup, bottom-left Cancel bar and connection cancellation were verified. The inspection identified clipped labels, which were corrected. No original map files were changed.

Current executable: `build/traffic_lab.exe`.
