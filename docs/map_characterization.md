# AutoDRIVE MAP characterization

The [2026 RoboRacer technical guide](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-guide-2026/#13-vehicle) describes a two-piece cubic tire-friction spline. Its listed lateral peak is at slip 0.01 with force 1.00, followed by an asymptote at slip 0.10 with force 0.50. These are **not** Pacejka B/C/D/E coefficients. The guide also lists 0.324 m wheelbase, 3.906 kg mass, center of mass x=0.15532 m, steering limit 0.5236 rad, and steering rate 3.2 rad/s. Confirm the installed simulator build before copying these numbers into configuration; the repo currently uses 0.330 m wheelbase measured from its vehicle frame transforms and a provisional Pacejka LUT.

## Reproduce

Connect AutoDRIVE to the running ROS bridge on `127.0.0.1:4567` in Autonomous mode. In the ROS container, from `/root/ws`:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/characterize_map.py record --track map_trial --scale 0.5 --laps 2
python3 scripts/characterize_map.py analyze log/map_characterization_<timestamp>.csv
```

The recorder resets the car, launches MAP, collects simulator body velocity/yaw rate, wheel odometry, IMU, actual steering feedback, `/drive`, and lap/collision counters, then stops at the requested lap count or first collision. It writes CSV and a launch log under `log/`. `--laps 2` includes a complete flying lap because the first count can occur shortly after the car starts from its reset pose. Ground-truth topics are used **only** for offline calibration; the controller continues to use `/odom`. The analysis uses samples with fresh paired data, speed in LUT coverage, small speed and steering changes, and nontrivial steering. Its observed lateral acceleration is `d(v_y)/dt + v_x*yaw_rate`; the model value is interpolated from the LUT at the simulator's reported steering angle and ground-truth forward speed. It does not fit or overwrite the model.

## Observations on `map_trial`

| Speed scale | Result | Paired / steady samples | Median absolute lateral acceleration error | 95th percentile absolute wheel versus ground-truth speed error |
|---|---|---:|---:|---:|
| 0.5 | Two lap-count increments, no collision | 639 / 112 | 0.101 m/s² | 0.883 m/s |
| 0.6 | Two lap-count increments, no collision | 747 / 118 | 0.095 m/s² | 1.148 m/s |

The median reported steering divided by commanded steering was 0.995 in both runs. `V1 Steering` is already an angle in radians; the extra multiplication by steering lock in `wheel_odom.py` was wrong. That bridge correction is submitted separately in [PR #20](https://github.com/SIGRobotics-UIUC/roboracer/pull/20). The bridge normally uses IMU yaw rate, so its incorrect steering conversion affected the kinematic fallback, not the measured-speed signal used for these runs.

The median wheel-speed error was near zero (-0.046 and -0.030 m/s), but its tails were large. Peak wheel speeds were 4.04 and 5.05 m/s while peak ground-truth body speeds were 3.52 and 4.37 m/s. The peaks were not necessarily simultaneous; do not interpret their differences as instantaneous slip. Check time-aligned wheel/ground-truth errors during acceleration and cornering before changing controller speed inputs. The LUT agrees reasonably in the observed steady region, which reached about 4 m/s and 4.6 m/s² lateral acceleration. It has **not** been validated near the guide's tire peak, the model's stability boundary, or speeds near the 7 m/s LUT ceiling. The simulator's friction spline and simplified single-track/Pacejka model remain different models, even if their effective response agrees over this limited range.

## Porto odometry investigation (2026-10-30)

Remapped the running Porto simulator as `tracks/porto_odom_trial` with two wall-following
laps at 1 m/s. The first mintime solve failed (`Restoration Failed`); the configured
mintime retry succeeded, with its speeds capped afterwards by the safe profile. This
fallback is recorded in the generated raceline header. Rebuilt the control package in
Docker because its installed launch files did not yet include MAP.

The clean MAP run used speed scale 0.5, Cartographer localization, and the normal wheel
odometry speed input. It completed two simulator lap-count increments with **zero new
collisions**. The first increment need not be a full lap. Logs and telemetry are under
`ws/log/porto_map_05_clean.csv`, `porto_map_05_clean.launch.log`, and the ROS bag
`porto_map_clean/` (generated artifacts, not committed). An earlier run had an orphaned
localization gate from a failed launch; the clean repeat had only one gate and localized
successfully. All trial controllers and recorders were stopped; the bridge remains running.

Use a sourced ROS container to inspect finalized bags:

```bash
python3 scripts/analyze_odom_bag.py log/porto_map_clean
python3 scripts/characterize_map.py analyze log/porto_map_05_clean.csv
```

The bag analyzer pairs `/odom` and `/ground_truth/odom` by **exact header stamp**, rather
than latest callback arrival. Stop recording first; it requires finalized metadata and
opens SQLite read-only. Moving samples require ground-truth forward speed above 0.5 m/s.

| Measurement | Wall-follow mapping | Clean MAP, scale 0.5 |
|---|---:|---:|
| Moving exact-stamp pairs | 954 | 408 |
| Median wheel minus reported GT speed | +0.046 m/s | +0.112 m/s |
| 95th percentile absolute speed error | 0.179 m/s | 0.525 m/s |
| Same error with GT averaged over approximately 0.1 s | 0.154 m/s | 0.495 m/s |
| Median wheel pose step / GT position step | 0.9998 | 1.0095 |
| Median GT position speed / reported GT speed | 1.049 | 1.057 |

For mapping, use bag `log/map_odom_investigation/`. A redundant secondary mapping bag
failed with SQLite locking during live inspection and is excluded. The clean MAP bag
has 571 exact-stamp pairs including stationary samples, median interval 0.068 s (~15 Hz).
GT position-step metrics exclude gaps outside 1–200 ms and steps of at least 1 m.
Ratios use planar displacement and forward speed, so turning and lateral motion also
contribute; these are diagnostic comparisons, not a fitted calibration.

The displacement agreement argues against a large wheel-radius scale error at these
speeds. However, differentiating **GT position itself using bridge wall time** also
exceeds the simulator's reported physics velocity by about 5–6% at the median. Thus the
speed discrepancy cannot be assigned entirely to wheel slip or callback pairing.
`fast_bridge` stamps each received frame with the ROS wall clock; `wheel_odom` divides
encoder travel by that elapsed time, while GT velocity comes directly from the simulator.
A mismatch between simulated physics time and wall time is a supported hypothesis, not
yet proven without physics timestamps. The 0.1 s smoothing window explains only part of
the error. At MAP speed, the additional median distance ratio is about 1%, consistent
with a smaller wheel-travel contribution but insufficient to identify slip uniquely.

No estimator, tire model, or controller gains were changed. A next experiment should
capture simulator physics time per frame (if available), check simulator time scaling
and frame pacing, and compare encoder travel and IMU yaw integration using that time
base. Do not tune wheel radius or substitute ground-truth velocity to hide a timing error.
The existing steering feedback multiplication in `wheel_odom` remains a separate known
issue affecting its kinematic fallback, not this default IMU-yaw-rate speed comparison.
Validation: Docker build of packages through `control` succeeded; four MAP math tests
passed on isolated ROS domain 42; the analyzer ran successfully on both finalized bags.

## Wheel-slip characterization (2026-10-30)

Implemented `scripts/characterize_wheel_slip.py` and analyzed the finalized Porto
mapping and both half-speed MAP bags. This is a longitudinal **encoder travel excess**
measurement, not a direct tire-contact slip sensor. It does not differentiate encoder
angles using wall time or compare against instantaneous reported speed:

```
wheel travel = wheel radius * encoder angle change
excess = (mean rear-wheel travel - GT forward travel) / GT forward travel
```

Triples of left encoder, right encoder, and GT pose are joined by exact integer-nanosecond
header stamps. GT planar displacement is projected onto midpoint heading and corrected
from chord length to arc length using the heading change (constant-curvature assumption).
Positive excess means over-rotation; negative means under-rotation. GT pose is assumed
centered laterally on the vehicle, so mean left/right travel cancels ordinary differential
turning. Individual wheel numbers are **not** tire slip in corners without a rear track
width; `--rear-track-width METRES` optionally corrects expected wheel paths by
`forward travel ∓ half track width * heading change`. No track width was assumed in these
results. The configured radius is 0.0585 m; a radius error remains a possible constant bias.

Filtering rejects intervals with wall gaps outside 1–200 ms, forward travel at most 2 cm,
reported mean forward speed at most 0.5 m/s, position/wheel steps of at least 0.8 m,
heading changes of at least 0.3 rad, or invalid expected travel. These thresholds reject
large resets and teleports, not every possible small discontinuity. Frame intervals are
about 68 ms, so transient numerical/sampling effects remain possible.

Acceleration grouping uses `(vx_end² - vx_start²) / (2 * forward_travel)`, avoiding wall
clock differentiation. This is a proxy for physical longitudinal acceleration, not a
measured tire force: accelerating >= +0.5 m/s²; braking <= -0.5 m/s²; steady between them.
Cornering means absolute mean reported yaw rate >= 0.3 rad/s. Groups overlap (e.g.
accelerating and cornering). Active driving requires a positive `/drive` command above
0.1 m/s and less than 0.3 s old at interval start. Explicit stop commands are retained for
up to 2 s; otherwise command state is unavailable and excluded from driving groups.

| Regime | Clean MAP median excess | Clean MAP intervals | Earlier MAP median excess |
|---|---:|---:|---:|
| Active driving, all | +1.00% | 398 | +0.95% |
| Active driving, accelerating | +2.28% | 144 | +1.95% |
| Active driving, braking | -1.48% | 102 | -1.63% |
| Steady straight (all command states) | +0.32% | 44 | +0.49% |

The earlier MAP recording had competing localization gates, so it is a repeatability
check rather than an equally clean control experiment. In the clean run, active-driving
5th–95th percentiles were -3.27% to +4.56%. Acceleration alone spanned +1.09% to +6.36%;
braking alone spanned -6.36% to +0.46%. Distance-weighted net excess over 52.90 m of
active driving was only **+0.56%**, since acceleration and braking partly cancel.
The 1 m/s mapping baseline was -0.34% median in 448 steady straight intervals; the MAP
baseline was +0.32%. That limits the case for a large constant radius correction but
does not identify pure tire slip independently of effective rolling radius and sampling.

Both MAP recordings also contain shutdown motion with **both reported encoder angles
unchanged**: 0.325 m over four accepted intervals in the clean run, and 0.310 m over three
in the earlier run. These intervals have stale/unavailable commands and are excluded
from active-driving results. This is consistent with wheel lock while the vehicle slides,
but encoder-reporting behavior cannot be distinguished from actual physical lock using
these signals alone. In particular, calling the entire braking distribution tire slip
without this qualification would overstate what was measured.

This establishes a repeatable acceleration/braking-dependent wheel-travel effect of a
few percent at the tested speeds. It does not explain away the separate wall-time versus
physics-velocity inconsistency. No live driving, model tuning, radius changes, or controller
input substitutions were needed for this offline characterization. Coverage is limited to
1 m/s mapping and MAP scale 0.5 (GT peak about 3.12 m/s), not the tire-limit regime.

Reproduce inside the sourced Docker container:

```bash
python3 scripts/characterize_wheel_slip.py log/porto_map_clean --output log/porto_slip_map05
python3 scripts/characterize_wheel_slip.py log/map_odom_investigation --output log/porto_slip_mapping
python3 scripts/characterize_wheel_slip.py log/porto_map_05 --output log/porto_slip_map05_repeat
python3 -m pytest scripts/test_characterize_wheel_slip.py -q
```

Outputs are JSON group summaries and CSV accepted intervals, including each wheel's
travel, GT forward/lateral travel, command state, acceleration proxy, and signed excess.
Five synthetic tests pass: no-slip invariance to wall-clock pacing, known 15%
over-rotation, left/right paths in a turn, reset/stationary exclusion, and final-stop
classification after the command stream ends.

## Straight-line timing experiment (2026-10-30)

Ran `straight_line_trial.py` at requested speeds 1.5 and 2.0 m/s, without MAP or
localization. The ordinary adapter remained in the loop, using wheel odometry for speed
feedback. Ground truth was used only to position and bound this diagnostic experiment.
Both trials completed with zero new collisions. They started near map x=-7.33 m on the
long straight, held steering at zero, requested speed until 3.5 m forward travel, and
then sent zero speed for braking. End positions were x=-3.36 and -3.14 m; lateral drift
was about 2 cm. Observed peak forward speeds were 1.75 and 2.06 m/s. The 1.5 m/s speed
loop transient demonstrates why the request is not an exact imposed vehicle velocity.

Added optional raw-frame tracing to `fast_bridge`:

- `frame_trace_path` defaults to an empty string (disabled). A nonempty path opens a new
  JSONL file exclusively, preserving existing experiments.
- Records monotonic arrival time, ROS stamp, bridge processing time, raw scalar/vector
  state, all payload field names, and the reply commands. Large camera/range payloads
  are omitted. A bounded queue and separate writer thread keep disk writes off the
  frame handler. Queue overflow and writer errors are reported at shutdown.
- `frame_trace_enabled` can pause recording after launch. Use a clean bridge shutdown
  to finalize the trace. This experiment closed with zero dropped trace records and no
  writer errors. The ordinary bridge was restored with tracing disabled afterward.

The incoming payload has **no explicit physics timestamp, frame counter, or FPS field**.
It does have `V1 Lap Time`. We tested that field as an empirical candidate simulation
clock, without asserting that its source is the physics timestep. No lap boundary or
reset is included in the analyzed straight-line intervals.

| Measurement | 1.5 m/s request | 2.0 m/s request |
|---|---:|---:|
| Accepted moving intervals | 38 | 31 |
| Median frame-arrival interval | 66.35 ms | 68.37 ms |
| Maximum frame-arrival interval | 146.06 ms | 95.87 ms |
| Median bridge frame processing | 1.81 ms | 1.85 ms |
| Median position speed / reported speed, wall time | 1.0697 | 1.0700 |
| Same ratio using lap-timer increments | 1.0013 | 1.0008 |
| 5th–95th percentile of lap-based ratio | 0.9803–1.0174 | 0.9886–1.0262 |
| Median cruise wheel-travel excess | -0.29% | +0.25% |

Cruise is conservatively defined by adjacent reported speeds differing by less than
0.02 m/s within the accelerate/cruise phase. It has only 4 and 9 accepted intervals;
these are short bounded tests, not a steady-state calibration. Other accepted intervals
require mean forward speed above 0.5 m/s and forward motion above 1.5 cm. Phase boundaries,
large pose/heading jumps, and gaps over 0.5 s are excluded. Raw position and encoder
values are used directly, so ROS callback pairing and `/odom` filtering do not cause
these position-versus-reported-velocity comparisons.

The 146.06 ms arrival gap is especially informative:

- Lap timer advanced **33.0 ms**.
- Vehicle moved **0.02662 m** forward at mean reported velocity **0.7963 m/s**.
- Position/wall interval gives **0.1822 m/s** (77% below reported velocity).
- Position/lap interval gives **0.8065 m/s** (1.3% above reported velocity).
- Bridge processing for that frame was **5.49 ms**.

Thus a long received-frame gap did not correspond to an equally long interval of vehicle
motion. The source could involve simulator update scheduling, transport, or OS scheduling;
arrival data cannot distinguish these or count actual dropped rendering/physics frames.
These runs did not reproduce every severe frame drop reported by the user. However,
matching lap-time increments to displacement largely removes the velocity mismatch in
both trials, strongly supporting a timing/sampling contribution independent of MAP.

The braking trace also shows transient encoder under-rotation, including a brief reversed
encoder increment in the 1.5 m/s run while body velocity was still positive. Do not label
all of that wheel lock: the observed behavior can involve motor/braking dynamics and
encoder implementation. These aggressive zero-speed stops differ from the speed changes
during ordinary laps. Only three accepted braking intervals per test remain above the
speed cutoff, so a braking distribution would be poorly supported.

**Interpretation:** the prior few-percent wheel-travel slip proxy remains meaningful,
but wheel velocity computed from arrival/wall time mixes it with substantial timing
variation. It is premature to retune MAP or tire coefficients around that error. A robust
follow-up should verify how the installed simulator generates lap time and sensor updates,
then supply unrestricted physics timestamps or a simulator clock to all consumers. Using
competition-restricted lap telemetry as a production controller clock would be inappropriate.
No estimator behavior was changed in this experiment.

Artifacts (generated, uncommitted) are in `ws/log/`: `straight_frames_20261030.jsonl`,
`straight_15{.csv,.json,_bag/}`, `straight_20{.csv,.json,_bag/}`, and
`straight_timing.{json,csv,png}`. The plotting script compares both clocks and encoder
travel. Example commands inside the ROS container:

```bash
ros2 launch autodrive_bridge autodrive_launch.py frame_trace_path:=/root/ws/log/NEW_TRACE.jsonl
# Separate shell, no other /drive controller:
python3 scripts/straight_line_trial.py --speed 1.5 --output log/NEW_TRIAL
# Stop tracing bridge cleanly, restore normal bridge, then analyze the finalized trace:
python3 scripts/analyze_straight_trace.py log/NEW_TRACE.jsonl log/NEW_TRIAL.csv --output log/NEW_REPORT
```

The positioning script assumes the saved `porto_odom_trial` map/world transform and
Porto geometry; it must not be used on another track without adaptation. It aborts on
stale telemetry, new collisions, invalid starting pose, and positioning/profile timeouts;
position/heading/depth bounds terminate the measured profile with zero speed.
Validation: package build succeeded; 20 focused bridge, asynchronous writer, slip-geometry,
and candidate-clock tests passed. The live ordinary bridge is the only remaining stack
launch; no experiment controller or recorder remains.

## Reusable lap report

See [tire_lap_report.md](tire_lap_report.md) for regeneration commands, metric definitions,
and the initial complete-circuit report in `ws/log/porto_tire_report/report.html`.
The report's velocity-reference consistency check found that reported velocity behaves
as a center-of-mass velocity (about 0.1555 m ahead of the rear axle), rather than rear-axle
velocity. That offset must be accounted for in front/rear lateral-slip estimates; it does
not affect the earlier rear longitudinal distance comparison through a forward x offset.
