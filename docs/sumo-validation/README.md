# SUMO backend validation

All **9 tests passed** with the actual Eclipse SUMO 1.28.0 runtime. The build is `build/traffic_sumo.exe`; the UI smoke test also runs the real backend through 50 ImGui frames.

| Scenario | Admitted | Completed | Body overlaps | SUMO collisions | Teleports | Free handoffs |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Guided four-way | 240 | 240 | 0 | 0 | 0 | 0 |
| Map_demo, 1.0 m/pixel, supplied signals | 240 | 240 | 0 | 0 | 0 | 0 |
| Free junction, cars | 48 | 48 | 0 | 0 | 0 | 48 |
| Free junction, Gaussian bike/motorbike/car/truck/bus mix | 48 | 48 | 0 | 0 | 0 | 48 |
| Two consecutive free junctions | 12 | 12 | 0 | 0 | 0 | 24 |

Three further cases verify live red-to-green timing, a permissive gate on a guided movement, and a permissive gate through a free junction. Each verifies completion only at the destination. The ninth case checks actual backend startup and nonempty ImGui draw data.

Demand lasts 120 simulated seconds. Guided cases offer 0.5 vehicles/second from each of four origins; free cases offer 0.1/second per origin. Drainage continues until all admitted trips complete, with a 1,200-second upper bound. Deletion is disabled. Conservation is checked every step, so handoffs cannot duplicate or lose vehicles. This establishes a conservative hybrid baseline; it is not a high-demand capacity guarantee.

`cases.log` contains per-case output, `ctest.log` the full suite result, `results.json` machine-readable outcomes, and `runtime.txt` the runtime version. Main network tests use seed 42; the signal tests use seed 7. Original files in Downloads were not modified; a fixture copy is in `tests/fixtures/map-demo/`.
