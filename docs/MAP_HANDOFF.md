# MAP handoff — 2026-10-31

## User objective and immediate next step

The user wants the recorded hairpin slowdown/wrong-way steering error eliminated, not
merely a gain comparison. A timing/recovery fix is implemented and regression-tested.
**Live validation of the fix is still pending. Do not claim the hairpin error is eliminated
or mark the overall objective complete yet.** The latest user instruction was to preserve
intermediate work and commit it to branch `MAP`.

The updated bridge is running, but no AutoDRIVE frames arrived after its restart. An
asynchronous question asked the user to disconnect/reconnect AutoDRIVE on Porto to
`127.0.0.1:4567` in Autonomous mode. The simulator runs on the user's Windows host, not
inside the ROS container; local Linux process inspection cannot establish its state.
The executable had previously been quarantined by Windows security and was restored.

When streaming resumes, run four fixed-configuration repeats and inspect every outcome:

```bash
# From repository root:
docker compose exec ros bash
# Inside container:
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/map_control_experiment.py --repeat-current 4 --output log/NEW_FIXED_VALIDATION
python3 scripts/compare_map_experiment.py log/NEW_FIXED_VALIDATION
```

Use a NEW output directory. `log/map_hairpin_fixed_20261031` already exists and contains
only a failed preflight (no live trials), with older source hashes; do not resume it after
code changes. The target is four successful warm-up/flying-lap repeats at unchanged
speed scale 0.5 and steering rate 3.2 rad/s, no collisions/wrong-way hairpin event, and no
ROS-clock jumps even if host wall time still steps. Preserve failures, not just successes.

## Workspace and runtime

- Repository: `/home/thegiraffe1/f1tenth_uiuc`; current branch: `MAP`.
- Docker service: `ros`, container `f1tenth_uiuc-ros-1`; mounted workspace `/root/ws`.
- Only normal bridge components were running at handoff: `fast_bridge`, `adapter`,
  `wheel_odom`, `foxglove_bridge`, plus ROS CLI daemon. No controller, localizer or recorder.
- Bridge launch used:
  `ros2 launch autodrive_bridge autodrive_launch.py frame_trace_path:=/root/ws/log/hairpin_fixed_clock_trace.jsonl`.
- Last observed launch process group was 5822 (verify with `ps`; never blindly reuse PIDs).
  Stop a launch with `kill -INT -- -PGID`, not SIGINT to only its parent. Wait for children
  to exit before launching another bridge. Stock simulator may require manual reconnect.
- Adapter and wheel odometry `use_sim_time` parameters were verified true. The bridge
  publishes the new monotonic `/clock`; source trace had zero frames at the last check.
- The active process predates the final *socket-ownership publication guard* and adapter
  duplicate-SIGINT cleanup-only patch. Its clock logic is already updated. A restart loads
  those last changes; it also requires restarting consumers and possibly reconnecting GUI.
- Frame trace files open exclusively. If restarting with tracing, use a NEW filename;
  reusing `hairpin_fixed_clock_trace.jsonl` causes startup failure, even if it is empty.
- `colcon build --symlink-install --packages-up-to control autodrive_bridge` succeeded.
  Source edits are symlinked into installed packages, but already-running Python processes
  do not reload them automatically.

## Diagnosis established from recorded data

See `docs/hairpin_failure.md` and `ws/scripts/diagnose_map_failure.py`.

In repeat 4, near the hairpin:

- ROS stamps advance 2.493 s while monotonic arrivals advance 0.068 s.
- Actual forward speed remains 1.051 → 1.038 m/s, but wheel odometry becomes 0.0284 m/s.
- Localized heading is wrong by about 119 degrees. An intermediate pose extrapolates
  2.685 m while the actual car moves approximately 7 cm.
- MAP rejects stale inputs briefly, then resumes once stamps are fresh even though the
  heading is still wrong. Steering changes -16.22 → +30.02 degrees.
- `MAP.stop()` previously cleared steering-rate history, so the first resumed command
  bypassed the normal slew limit. Collision follows about 0.91 s after the post-step frame.
- Isolated offline queries reproduce +30.02 degrees. Correcting heading alone produces
  -4.01 degrees; full GT pose/speed produces -17.61 degrees. These are isolated guidance/LUT
  queries, not claims about a complete counterfactual closed-loop drive.
- The other two failed repeats show the same clock/false-speed/heading-error/extreme-steer
  chain, including a different track section. It is not exclusive to one hairpin.

Why the host clock jumps remains undiagnosed. Do not blame low FPS alone: source/wall
steps and genuine monotonic frame stalls are different observations. The implementation
isolates sensor/control timing from host wall corrections, without modifying OS clocks.

## Fix implemented

Details: `docs/hairpin_timing_fix.md`.

1. `autodrive_bridge/steady_time.py`: epoch-shaped time sampled from wall time once, then
   advanced exclusively by monotonic nanoseconds. It is a **transport clock**, not exported
   simulator physics time or a correction to the remaining physics/transport rate mismatch.
2. `fast_bridge.py`: publishes this `/clock` at 100 Hz from a STEADY_TIME timer and before
   sensor frames; sensor stamps use that same clock. Clock writes from timer and websocket
   threads are serialized. Main activates publication only after successfully owning the
   server socket, preventing a failed overlapping launch from injecting a competing epoch.
3. Bridge, perception and control launches default to `use_sim_time:=true`; race launch
   forwards it into included launches. Mapping/racing wrappers propagate `USE_SIM_TIME`.
   Hardware/legacy users must consistently set `use_sim_time:=false` or `USE_SIM_TIME=false`.
   Direct `ros2 run` nodes need `-p use_sim_time:=true` explicitly in simulator operation.
4. `control/input_health.py`: clocks checked against monotonic time; heading checked against
   a reference propagated with fresh odometry yaw rate. Fault recovery requires 0.25 s and
   3 distinct updates from both pose and odometry. Default heading tolerance 0.25 rad;
   clock tolerance 0.25 s. Repeated bad poses do not become the reference. Explicit simulator
   reset signals clear references and cached inputs.
5. `map_node.py`: safety stop requests zero speed while retaining last steering; no clearing
   of slew reference. Slew limiting uses monotonic dt capped at 0.1 s. Bad heading remains
   stopped even with fresh stamps. This can conservatively require relocalization rather
   than guaranteeing autonomous recovery after arbitrary outages.
6. `map_control_experiment.py`: nodes/localizer/recorder use shared time by default;
   recorder includes `/clock` and `--use-sim-time`. `--wall-time` selects legacy behavior.
   Reports distinguish ROS-clock discontinuities from host wall steps. No gains, LUT,
   raceline, geometry or speed scale were retuned for this fix.
7. Adapter cleanup catches a second SIGINT during `destroy_node`, matching wheel odometry's
   existing handling. Launch tests exposed this race; it does not change driving logic.

Cautions: do not run a second `/clock` source or replay a bag's clock into the live domain.
Restart time-dependent consumers when restarting/changing the bridge clock. A real long
sensor/CPU outage still cannot be driven through at arbitrary speed; safety stop/recovery
remains necessary.

## Test record and intermediate failures

All tests use isolated domains; keep `ROS_DOMAIN_ID=42` when running repository tests.
Bridge conftest additionally forces domain 97; process lifecycle tests use domain 77.

- Initial focused timing/bridge/odometry/MAP math/recovery run: 24 passed.
- MAP ROS command/recovery integration: 2 passed after updating assertions for the deliberate
  recovery wait and retained steering.
- Full bridge suite: 48 passed after fixing a duplicate-SIGINT cleanup race.
- Full control run: 95 passed, 2 failed because isolated pure-pursuit tests had no `/clock`
  after launch defaults changed. Test launch helpers now explicitly request wall time.
  All affected freshness/watchdog tests plus wall-follower tests were rerun: 9 passed.
  This was a test-environment mismatch, not a weakened freshness requirement.
- Perception suite: 106 passed.
- Report/analysis suites: 19 passed.
- Final targeted clock-publication/socket-ownership/process-lifecycle/recovery run: 12 passed.
- Python compilation, shell syntax (`common.sh`, `race.sh`, `map_track.sh`) and
  `git diff --check` passed.

An early launch test caught that Humble does not export `qos_profile_clock`; fixed by using
explicit BEST_EFFORT/depth-1 QoS before starting the updated live bridge. Do not revert to
that unavailable import. The only unresolved validation is live driving with the fix.

Useful commands:

```bash
colcon build --symlink-install --packages-up-to control autodrive_bridge
source install/setup.bash
ROS_DOMAIN_ID=42 python3 -m pytest src/autodrive_bridge/test -q
ROS_DOMAIN_ID=42 python3 -m pytest src/control/test -q
ROS_DOMAIN_ID=42 python3 -m pytest src/perception/test -q
ROS_DOMAIN_ID=42 python3 -m pytest scripts/test_tire_lap_report.py scripts/test_compare_map_experiment.py scripts/test_characterize_wheel_slip.py scripts/test_analyze_straight_trace.py -q
```

## Existing reports, artifacts and earlier work

The whole session's reporting/characterization tools are included in this commit. Raw bags,
tracks and generated HTML/PNG/CSV/JSON are git-ignored and remain only in this workspace;
a fresh clone needs the original recordings/maps to regenerate them.

- `docs/tire_lap_report.md`: report usage, assumptions and repeated-run history.
- `docs/map_characterization.md`: initial MAP/timing/slip observations.
- `ws/log/porto_tire_report/report.html`: initial complete-circuit tire/track-error report.
- `ws/log/porto_tire_report_all/report.html`: whole-recording report, explicitly not a clean lap.
- `ws/log/map_repeatability_20261031/comparison.html`: four-repeat baseline reliability report.
- `ws/log/map_repeatability_20261031/repeatability_profiles.png`: three completed flying-lap
  overlays (includes the completed flying laps from two earlier attempts that failed later).
- `ws/log/hairpin_investigation/report.html`: three failures, exact snapshots and replay results.
- `ws/log/straight_frames_20261030.jsonl`, `straight_15*`, `straight_20*`, `straight_timing.*`:
  straight acceleration/cruise/braking tests and raw-clock evidence.
- Redundant `porto_odom_mapping` bag failed SQLite locking when inspected live early in
  the session; excluded. Use finalized `map_odom_investigation` instead. Never read an
  actively written SQLite bag for analysis.

Baseline repeats were 1/4 complete, with 3 warm-up collisions. The successful new flying
lap was 15.0925 s. Supplementing with two earlier *matching-configuration* completed laps:
15.0925–15.3106 s lap times, 0.0535–0.0836 m tracking RMS, front lateral P95 peak-coordinate
ratio 0.595–0.714, front maximum 1.083–1.447. Those three completed laps are conditional
samples and do not override the poor run-success rate.

Reports are **kinematic slip relative to documented peak-slip coordinates**, not measured
force utilization. Defaults: longitudinal peak 0.15, lateral peak tan(alpha)=0.01, radius
0.0585 m, track width 0.236 m, wheelbase 0.33 m. Installed-binary tire definitions remain
unverified. The body velocity reference is empirically about 0.1555 m ahead of the rear
axle, consistent with documented COM 0.15532 m; ignoring this creates false large lateral
slip. Rear longitudinal estimates are distance-based and correct left/right turning travel.
Only rear encoders exist. Tracking uses GT rear-axle position versus mapped raceline;
localization and map/world calibration can contribute.

The last report changes add steering/rate, signed tracking and heading error, event durations,
blue below 1 versus orange/red at/above 1, and brief/sustained/uncertain classifications.
Sustained is a configurable 0.3 wall seconds plus 3 valid intervals, not proof of physical
saturation. Figure z-order was corrected so gray trajectory does not obscure colored dots.

## What to report to the user next

Keep progress concrete: implementation and automated regressions are complete; fixed-stack
live repeatability is not yet established. Once connected, run the repeats, preserve all
attempts, inspect clock/pose/speed/steering around the former failure, and summarize completion
counts and errors alongside before/after plots. Do not assert success based only on tests.
