# Paired native-controller results

Each row uses the same offered 60-trip manifest for the pair: 30 seconds of demand followed by 90 seconds of drainage. These synthetic cases compare native controllers, not SUMO performance. See [model and conditions](../CAR_FOLLOWING.md).

| Junction | Seed | Arrived by 120 s, legacy / new | Remaining, legacy / new | Mean stopped seconds, legacy / new |
| --- | ---: | ---: | ---: | ---: |
| free | 7 | 54 / 55 | 0 / 0 | 4.66 / 2.44 |
| guided | 7 | 43 / 55 | 11 / 0 | 20.76 / 10.24 |
| free | 42 | 54 / 55 | 0 / 0 | 5.13 / 2.94 |
| guided | 42 | 11 / 55 | 43 / 0 | 71.89 / 16.50 |
| free | 123 | 50 / 50 | 0 / 0 | 2.39 / 2.09 |
| guided | 123 | 17 / 50 | 33 / 0 | 58.63 / 10.83 |

Total: **229 -> 320** completed trips. Legacy admitted 316 and retained 87; the new model admitted 320 and retained 0. Each controller received 360 identical offered trip requests. Weighted mean stopped time across all admitted vehicles was **27.16 -> 7.54 s**.

All 12 runs had **zero physical collision events and zero deletions**. The new controller drained every admitted vehicle in all six cases. Early throughput at 30 seconds was lower in some guided cases, and maximum individual stopped time was slightly higher in some free cases; these are not uniform improvements of every metric. New-only intervention counters in the logs are not comparable with the legacy branch's uninstrumented values.

**17 targeted regressions passed**, including 180 discrete stopping-equation cases. Physical contact assertions replace the retired preferred-radius assertions in the relevant packing tests. Blocked-exit tests also accept anticipatory stopping before the endpoint, while continuing to forbid entering a blocked junction. Solid-obstacle tests now require waiting and resuming after clearance, rather than ghosting through the obstacle.

[Validation and hashes](validation.json) · [CTest log](regressions.log)
