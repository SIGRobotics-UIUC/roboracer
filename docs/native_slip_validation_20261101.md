# Native simulator slip validation — 2026-11-01

Installed fork: `thegiraffe1/AutoDRIVE`, branch `windows-il2cpp-slip`, revision
`070b15ff16f07ee9cab22b15b7fe5815d5d81ebe`. Built with .NET 8 and existing
BepInEx 6.0.0-be.791 / Unity 2022.3.52f1 Windows IL2CPP bindings.

Dedicated simulator directory: `D:\Downloads\autodrive_slip_validation_20261101`.
The local `autodrive_plugin/` checkout contains unrelated work on `windows-extensions`;
it was preserved. Source was exported from the requested revision into
`/tmp/autodrive-slip-build` for this build. The installed DLL includes the small
[headless Autonomous-mode patch](autodrive_slip_autonomous.patch): passing
`-autonomous` explicitly sets `VehicleController.DrivingMode = 1`. The slip
extraction code is unchanged.

Installed DLL SHA256: `a3f40f23359aa82a20011a683beab295565c4df728b21f722156d99ff942b5fe`.

The dedicated ROS container is `f1tenth-slip-validation`, ROS domain 58, bridge
host port 4568, Foxglove port 8766. Simulator arguments:

```text
-batchmode -nographics -autonomous -ip 127.0.0.1 -port 4568
```

## Recorded outcome

Successful artifacts: `ws/log/native_slip_isolated_retry_20261101/`. The native
topic was present at every selected frame in both flying laps.
Speed scale 0.5, steering rate 3.2 rad/s, unchanged map/raceline and controller settings.

| Trial | Native flying lap | Collision change | ROS clock jumps | Native report frames |
|---|---:|---:|---:|---:|
| 1 | 14.0781 s | 0 | 0 | 430 |
| 2 | 14.0551 s | 0 | 0 | 247 |

Rear longitudinal signed correlation was 0.962–0.970 in trial 1 and
0.851–0.885 in trial 2. Lateral signed correlations were negative (about
−0.73 to −0.79), with larger kinematic magnitudes, especially at the rear.
These comparisons retain original signs without fitting coefficients.

In trial 1, one instantaneous native sample per front wheel exceeded the configured
0.01 lateral threshold (maxima 0.01021 and 0.01245); no front native endpoint
exceeded it in trial 2. Native interval means suppressed those isolated trial-1
crossings, so the report shows endpoint values and endpoint peak counts separately.
Both trials had native longitudinal peaks above the configured 0.15 threshold.
The thresholds describe slip coordinates, not measured force utilization.

Failed attempts remain in `ws/log/native_slip_validation_20261101` and
`ws/log/native_slip_isolated_20261101`. They do not contain completed measured laps.
The initial bridge startup frame can lack lidar; the bridge now replies normally
while waiting for the first lidar payload, preventing a startup deadlock.

## Verification

Plugin build: zero warnings/errors. Focused bridge/report regression run: 28 passed;
final report-analysis checks: 12 passed. Both finalized bags generated native
comparison HTML/PNG/CSV/JSON. Old recordings cannot supply native slip retrospectively.

Native `WheelHit.forwardSlip` and `WheelHit.sidewaysSlip` are simulator contact
telemetry. They have different definitions from our rigid-body/encoder estimates;
they are not real tire-force measurements. Front longitudinal telemetry is now
available, but no front encoder estimate exists for comparison.

## Primary-source update

Both successful reports were regenerated with `--slip-source unity`. The primary
profiles, maps, peak summaries and crossing events now use instantaneous Unity
readings, including front longitudinal slip. The comparison section preserves
the original kinematic estimates. Their earlier estimated threshold counts should
not be interpreted as native Unity crossings. Source metadata prevents pooling
these native metrics with earlier kinematic reports.

Primary Unity front-lateral maximum/peak ratios are 1.2449 and 0.9417 in trials
1 and 2, with one brief axle-level crossing event in trial 1 and none in trial 2.
Rear-lateral maxima are about 0.14 of the configured peak in both laps. No native
lateral event meets the sustained classification. These replace the earlier
kinematic metrics in the main repeatability overlays and summary.
