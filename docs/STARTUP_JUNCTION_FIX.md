# Startup junction queue fix

Use `build/traffic_junction_fix.exe`. Allowed-exit lines remain permission-only; trip destinations are separate nodes.

The startup freeze was reproduced with the viewer's actual road mask, seed 42, 120 Hz updates, 0.5 spawn attempts/s at each of its four sources, the 100-active-vehicle cap, and `vietnameseCityProfiles()` including sampled dimensions, speed and acceleration. Vehicle deletion was disabled. The earlier short startup regression used fixed cars/motorbikes and stopped spawning too soon to expose these failures.

Changes:

- Consume a full unit of spawn credit when the floating-point tolerance accepts a spawn. Applying `fmod` to a value just below one retained almost all its credit and could spawn a second vehicle on the next frame.
- Offset lane geometry using the road's tangent, rather than a chord from an already displaced vehicle. Cached crossing paths are reused only when their starting pose matches; their cursor anchor is set to the exact live pose. The predictive movement gate also rejects movement exceeding the current step's travel budget.
- Carry turning vehicles forward until they clear the incoming row before steering across its sublanes. This prevents later arrivals parking in the space needed for the initial turn.
- Use the actual preferred clearance when identifying a leader. The extra 0.05 m look-ahead cushion could label a legally spaced side-by-side neighbor as a stopped leader and command zero speed even with a clear forward path. Actual swept body/gap checks are retained.
- Leave 0.02 m of steering room when selecting new insertion/merge gaps and stagger parallel neighbors that would otherwise meet exactly at their preferred lateral distance. This keeps tiny continuous heading changes from trapping a packed row; it does not reduce the hard minimum clearance.
- Protect only the bounded lateral transition of a within-lane merge. Short swept pieces prevent a follower or another merging vehicle from occupying the same gap midway through the maneuver. They expire with cursor progress or route revision; they do not reserve an entire trip or free junction. Piecewise checks avoid the false obstruction caused by enclosing a curved pass in one large convex polygon.
- Report vehicles waiting at a junction boundary as waiting for an exit, rather than as already at their trip destination.

## Sustained regression results

| Seed | Continuous spawning | Requests offered | Vehicles inserted | Vehicles completed after drain | Deletions |
|---|---:|---:|---:|---:|---:|
| 42 | 300 simulated seconds | 600 | 476 | 476 | 0 |
| 7 | 180 simulated seconds | 360 | 303 | 303 | 0 |
| 123 | 180 simulated seconds | 360 | 306 | 306 | 0 |

All 1,085 inserted vehicles completed. The runs preserved the road mask and tested preferred clearances. Displacement was checked every physics update against the vehicle's speed/acceleration travel budget. The test rejects a persistent stop inside the free junction and requires the whole fleet to drain within 180 simulated seconds after spawning stops. All 32 targeted startup, lane, routing, signal, permission and persistence regressions passed, including the three dense fixed-demand benchmarks.

Requests rejected by occupied insertion space or the viewer's population cap are not counted as inserted or completed. In seed 42, the fleet reached 100 active vehicles while arrivals continued; completed trips continued increasing through the five-minute spawning window. The fix prevents the reproduced permanent lock; it does not make an oversaturated approach queue-free.

Reproduce:

```powershell
cmake --build build -j 4
ctest --test-dir build -R '^startup_mixed_flow' --output-on-failure -j 3
```

The saved signal-link compatibility and permission-only behavior remain covered by `traffic_flow`, `trip_planning` and the signal suites. The earlier throughput comparison is historical; it used a different, fixed-size demand fixture and should not be substituted for these sustained startup checks.
