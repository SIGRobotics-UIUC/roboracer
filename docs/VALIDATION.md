# MAP timing and tire-slip validation

## Decision

Use native Unity WheelHit slip for simulator tire-limit diagnostics. The current
kinematic lateral approximation is too inaccurate for threshold decisions and is
retained only for historical comparisons. Its synchronized, calibrated use on a real
vehicle remains a [TODO](tire_lap_report.md#todo-real-vehicle-slip-estimation).
The tool locates peak-coordinate crossings and relates them to the track, speed,
steering and tracking error; it does not enforce a limit or measure tire forces.

## Recorded experiments

| Configuration / recording | Completed attempts | Collisions | Flying-lap times | Interpretation |
|---|---:|---:|---|---|
| Original repeatability baseline | 1/4 | 3 | Successful lap 15.0925 s | Timestamp steps preceded false speed, heading errors and extreme steering. |
| Fixed monotonic clock + guarded recovery | 4/4 | 0 | 14.0180–14.2051 s | Normal-run repeatability; no ROS jumps or host wall steps occurred. |
| Isolated native-slip capture | 2/2 | 0 | 14.0781, 14.0551 s | Native slip present at all selected frames; no ROS jumps. |

These are small observational samples. The laptop changed between earlier and newer
runs, so lap-time changes cannot be attributed solely to the timing fix. No host
clock step occurred in the successful fixed-stack driving runs; direct fault-path
evidence comes from the automated clock-step/recovery checks.

The two native runs showed rear longitudinal signed correlation 0.85–0.97 with
encoder-based estimates. Lateral signed correlations were negative (about −0.73 to
−0.79), and estimated magnitudes were substantially larger, especially at the rear.
A sign convention change cannot fix that magnitude discrepancy.

Unity front-lateral maxima were 1.2449 and 0.9417 times the configured peak. The
first lap had one brief axle-level event; the second had none. Neither had a
sustained native lateral event. Both had brief longitudinal crossings. The primary
[portable report](reports/unity_slip_20261101/report.html) contains only Unity slip;
[installation/comparison notes](native_slip_validation_20261101.md) preserve details.
Peak-coordinate crossings indicate sampled slip exposure, not proven saturation.

Failed native capture attempts are retained locally and excluded from measured-lap
results. Earlier kinematic reports remain historical estimates; native values cannot
be recovered retrospectively from bags that never recorded them.

## Automated verification

From a sourced ROS container in `/root/ws`:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/validate_stack.sh
```

The command runs bridge, control, perception and analysis suites in separate Python
processes. Each package owns its own ROS context; combining all package directories
in one pytest process causes conflicting fixture initialization. Tests use domain
42 by default, with bridge/process-test isolation as defined in their fixtures.

Final verification on 2026-11-01:

| Suite | Result |
|---|---|
| Bridge | 52 passed |
| Control | 97 passed |
| Perception | 105 passed; one CPU-time-sensitive test failed, then passed after the test-clock correction below |
| Analysis/reporting | 24 passed |
| Corrected perception case | 1 passed on targeted rerun (included among the 106 distinct perception tests) |

The perception failure was
`test_a_burst_of_re_anchors_pauses_them_and_checks_in_the_background`.
Fifteen synthetic updates took over two seconds of real CPU time, letting the
production hold expire before the assertion. The test now controls only the
MCL module's monotonic clock, advances synthetic time during the hold, and also
asserts that local re-anchors resume after expiry. Production localization logic
was not changed. The other 105 perception results remain from the full suite;
the full perception suite was not repeated after this test-only correction.

The initial combined invocation is retained in `ws/log/prepush_validation_20261101.log`; its context
collisions are a test-invocation failure, not a successful verification run.
The separate-package log is `ws/log/prepush_separate_validation_20261101.log`.

Checks cover native frame parsing/stamps, malformed and startup lidar frames,
missing/airborne contact, instantaneous peak preservation, clean exports, consistent
slip-source grouping, clock steps, recovery and steering continuity. The final report
was regenerated from a finalized bag and its map was visually inspected. Python
compilation, shell syntax and `git diff --check` are also checked.

## Push contents and local assets

Commit the bridge/report code, tests, concise documentation, plugin source revision
and optional headless-mode patch, wheel-setting evidence, and portable report.
Raw bags, generated tracks, colcon outputs and the separate `autodrive_plugin/` clone
remain ignored. The plugin clone has unrelated uncommitted `windows-extensions`
work and must be managed as a separate repository. No upstream fork was pushed.
