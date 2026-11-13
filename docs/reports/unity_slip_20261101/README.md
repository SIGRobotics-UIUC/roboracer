# Unity-only tire slip report

Open [report.html](report.html) in a browser; plots are embedded. This is the first
measured flying lap from `native_slip_isolated_retry_20261101`: 14.0781 seconds,
speed scale 0.5, steering slew limit 3.2 rad/s, no collision or clock jump.

All slip profiles, maps and threshold events use native Unity WheelHit telemetry.
No kinematic slip estimates or estimate/native comparison appear in this report.
The axle metric requires both wheels to have contact. `E1`–`E7` on the maps identify
sampled event peaks in the event table. Larger outlined markers draw over ordinary
dots so a one-frame crossing is visible. Native slip is not measured tire force.

`summary.json`, `settings.json`, `samples.csv`, and `events.csv` preserve the selected
analysis. `provenance.json` records the plugin revision and source hashes. Raw bags
and the generated Porto map/raceline are not included; reproduce using the original
local assets and the command in the report. See [the tool guide](../../tire_lap_report.md).
