# MAP and tire diagnostics handoff

Current results and test commands are consolidated in [VALIDATION.md](VALIDATION.md).
The main tool guide is [tire_lap_report.md](tire_lap_report.md); a portable
[Unity-only report](reports/unity_slip_20261101/report.html) is included for review.

## Conclusions

- The original hairpin failure involved a timestamp step, false near-zero wheel
  speed, incorrect localized heading and extreme steering. See the recorded
  [diagnosis](hairpin_failure.md) and [timing/recovery fix](hairpin_timing_fix.md).
- Four normal fixed-stack repeats completed with no collision or ROS-clock jump.
  No host-clock step occurred, so live driving did not directly reproduce that fault.
- Two further runs captured native per-wheel Unity slip, also without collisions.
  Their flying laps were 14.0781 and 14.0551 seconds.
- The existing kinematic lateral estimate is too inaccurate for tire-limit decisions.
  Retain it only as a diagnostic comparison. Revisiting synchronized live estimation
  on the real vehicle is an explicit TODO in the tool guide.
- Use Unity telemetry for simulator slip profiles, track maps and crossing statistics.
  A peak-slip coordinate is not measured force utilization or a hard safe-speed bound.

## Reproduction and artifacts

The simulator plugin is from `thegiraffe1/AutoDRIVE` branch `windows-il2cpp-slip`,
revision `070b15ff16f07ee9cab22b15b7fe5815d5d81ebe`. Installation, the optional
headless Autonomous-mode patch and DLL hash are in
[native_slip_validation_20261101.md](native_slip_validation_20261101.md).
The local `autodrive_plugin/` checkout has separate work on `windows-extensions`;
it is ignored here and must be managed as its own repository.

Raw local artifacts (git-ignored):

- `ws/log/map_fixed_validation_20261031_run1`: four fixed-stack repeats.
- `ws/log/native_slip_isolated_retry_20261101`: two native-slip runs and comparisons.
- `ws/log/unity_slip_final_20261101`: clean report regenerated from the first native lap.
- Failed native attempts remain in `native_slip_validation_20261101` and
  `native_slip_isolated_20261101`; neither supplied a completed measured lap.

Raw bags and generated maps/racelines are not included in the push. The portable
report includes the selected native samples, settings, summary and input hashes;
regenerating from bags requires the original local recordings and Porto map assets.

## Runtime notes

Simulator operation uses the bridge's monotonic transport `/clock` consistently.
For hardware, pass `use_sim_time:=false` consistently. A transport clock is not
simulator physics time. Restart time-dependent consumers after changing the clock.
Use new trace/output paths and preserve every failed attempt.

The dedicated slip simulator was installed at
`D:\Downloads\autodrive_slip_validation_20261101`, connecting to host port 4568.
Its ROS container is `f1tenth-slip-validation`, domain 58, Foxglove port 8766.
That experiment pair was stopped after capture. Check current processes before
restarting; another simulator or bridge may be in use by unrelated work.
