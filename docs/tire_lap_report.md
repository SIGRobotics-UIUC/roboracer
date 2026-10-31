# Regenerating a tire operating-region report

Run from `/root/ws` in the ROS Docker container after sourcing `install/setup.bash`:

```bash
python3 scripts/tire_lap_report.py log/porto_map_clean \
  --track-dir tracks/porto_odom_trial \
  --output log/porto_tire_report
```

Open `log/porto_tire_report/report.html`. The HTML includes its plots as embedded images,
so it can be copied on its own. The same directory contains `report.md`, two PNG figures,
`samples.csv`, `summary.json`, and `settings.json`. Regeneration replaces these generated
files. The saved JSON records the input paths, selected interval, thresholds, assumptions,
quality counts, and exact regeneration command.

`--lap auto` selects the first complete geometric circuit in a continuous recording segment.
The start/finish is the first moving position in that segment, not necessarily the simulator's
finish line. `--lap 2` selects the second complete circuit. `--lap all` includes the whole
recording, including startup/stopping. If auto cannot find a complete circuit, it generates
a clearly labeled partial report; an explicit unavailable lap number fails. A circuit can
start from rest, so this is not necessarily a flying-lap comparison.

The track directory must contain `raceline.csv` (s, x, y, heading, curvature, speed,
acceleration) and `map_to_world.yaml` matching the simulator world. The report projects
position onto the closed raceline and unwraps progress. Teleports, implausible association
jumps, and gaps over one second split segments. Shorter plausible gaps retain geometric
continuity, but slip estimates are unavailable when the frame interval exceeds 0.2 s.
This is offline diagnostics only; no controller is launched and no vehicle commands are sent.

## Record the inputs

With the bridge already connected, record while your chosen controller drives the lap:

```bash
ros2 bag record -o log/NEW_LAP \
  /ground_truth/odom /autodrive/roboracer_1/left_encoder \
  /autodrive/roboracer_1/right_encoder /autodrive/roboracer_1/steering /drive
```

Stop recording cleanly before analysis. Only finalized SQLite ROS bags are supported.
Ground truth is required for these offline estimates. Both rear encoders are joined to
GT by exact integer-nanosecond source stamps. Steering has no header timestamp; its nearest
bag-receipt time is used, within 30 ms at each interval endpoint. Missing encoders or
steering leave the affected metric unavailable, not zero. `/drive` is optional context;
complete-lap selection does not depend on receiving command messages.

## What is plotted

- Front lateral: maximum absolute estimated left/right lateral slip, divided by the lateral
  peak-slip coordinate.
- Rear lateral: the corresponding rear-wheel estimate.
- Rear longitudinal: maximum absolute left/right encoder-travel excess, divided by the
  longitudinal peak-slip coordinate. Per-wheel turning travel is accounted for.
- Reported forward speed and wall-clock frame interval, to expose low-speed and timing effects.
- Three track maps colored by the same slip ratios. Colors stop changing above 2; numeric
  values in profiles, CSV, and summaries remain unclipped. Gray indicates unavailable data.

The lateral estimate computes wheel-point velocity from rigid-body velocity and yaw rate,
then compares its direction with inferred Ackermann wheel steering. Rear longitudinal
slip compares encoder travel with pose displacement projected onto midpoint heading,
corrected from chord to arc and for left/right turning paths. It does not divide wheel
travel by wall-clock time. Estimates are interval-scale and cannot resolve brief peaks
between frames. Neither metric estimates tire force, normal load, or combined-slip capacity.
Front longitudinal slip is unavailable because front-wheel encoders are absent.

The threshold line at 1 is the **documented peak-slip coordinate**, not a hard physical
slip limit or proof of saturation in the installed binary. The orange line at 0.8 is an
explicit reporting heuristic. Fraction of distance above either threshold uses valid
interval distance as its denominator; missing estimates do not count as below the threshold.
Summaries also expose valid interval counts and distance so coverage can be assessed.

## Settings and velocity reference

Defaults are editable with `--settings YOUR_SETTINGS.json` (JSON keys override defaults).
The complete effective configuration is always saved as `settings.json`:

| Setting | Default | Basis |
|---|---:|---|
| wheelbase | 0.33 m | Repository vehicle geometry/frame transforms |
| track_width | 0.236 m | Documented RoboRacer geometry |
| wheel_radius | 0.0585 m | Existing bridge calibration |
| velocity_reference_x | 0.15532 m | Documented center of mass; empirically supported below |
| longitudinal_peak | 0.15 | Documented longitudinal force-peak slip |
| lateral_peak | 0.01 | Documented lateral force-peak slip, expressed as tan(alpha) |

Source: [RoboRacer technical guide](https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-guide-2026/#132-vehicle-dynamics).
These defaults are not asserted to be verified settings of the installed simulator binary.
In particular, the guide's dynamics table lists 0.324 m wheelbase and 0.059 m wheel radius,
whereas the repository currently uses the values above. Geometry and model assumptions
are explicit so alternate configurations can be compared without changing code.

Reported body velocity must be transformed from its actual reference point before computing
wheel velocities. In the clean Porto bag, a time-independent consistency check estimates
this reference about **0.1555 m ahead of the rear axle**, matching the documented center
of mass. This is evidence from recorded motion, not direct inspection of the simulator
source. The earlier conceptual rear-axle velocity assumption was incorrect for this data.
Ignoring the offset spuriously inflates lateral slip at both axles.

The diagnostic derives the reference from
`(reported_vy - reported_vx * lateral_displacement / forward_displacement) / yaw_rate`
in moving intervals with |yaw rate| > 0.4 rad/s. It also compares displacement direction
against velocity transformed to the rear axle. Results are saved in the summary and CSV.
Lateral estimates are suppressed when that angular disagreement exceeds 0.02 rad.
Use a different reference configuration for builds whose signals behave differently.

## Initial Porto report

Generated from `log/porto_map_clean`: one continuous geometric circuit, about 27.34 m,
with 206 valid intervals for all three metrics. The 2.26 s telemetry interruption elsewhere
in the recording was excluded by circuit selection. One 159 ms interval inside the selected
circuit is flagged; its estimates remain available under the 200 ms cutoff.

| Metric | Median peak-coordinate ratio | P95 | Distance at/above 1 |
|---|---:|---:|---:|
| Front lateral | 0.243 | 0.827 | 3.22% |
| Rear lateral | 0.151 | 0.527 | 0.00% |
| Rear longitudinal | 0.287 | 0.880 | 2.68% |

These numbers are model-relative estimates, not measured saturation percentages. Front
spikes exceed twice the documented coordinate; steering pairing, inferred wheel angles,
and installed tire-model definitions need checking before concluding physical saturation.
The reference-direction residual has median absolute value 0.00050 rad and P95 0.00231 rad.
No estimator, controller, or simulator parameters were changed to generate this report.

## Validation

```bash
python3 -m pytest scripts/test_tire_lap_report.py -q
```

Synthetic checks cover Ackermann geometry, front/rear separation, center-of-mass reference
transformation, known longitudinal over-rotation, invariance to wall-time scaling,
missing sensors, discontinuities, and complete-lap selection. The generator was also run
against the finalized Porto bag, and its figures inspected.

## Brief versus sustained threshold crossings

The report now includes steering feedback in degrees, its interval rate in degrees per
**wall-clock** second, and a threshold-event table. `events.csv` and `summary.json` contain
each event's peak ratio, covered interval count, wall duration, source stamps, selected-row
indices, and maximum absolute steering-feedback rate. `samples.csv` labels every estimate
as below peak, brief, sustained, timing-uncertain, or unavailable for each metric.

Default persistence settings (overridable through `--settings`):

- `sustained_crossing_seconds`: 0.3 wall-clock seconds.
- `sustained_crossing_min_intervals`: 3 consecutive valid intervals.
- Every interval in the run must be at or above ratio 1. Missing estimates, segment changes,
  and intervals above `frame_gap_warning` (0.12 s) break runs. A crossing in such a long
  interval is timing-uncertain and cannot count as sustained.

Orange shading/markers identify brief runs, red identifies sustained runs, and gray
identifies timing-uncertain runs on the slip profiles. The track maps retain the distinct
blue-below-1/orange-red-at-or-above-1 palette and show peak-coordinate ratio regardless of
event duration. Interval values are not smoothed or discarded based on the classification.

The durations sum the recorded interval lengths, including the first above-threshold
interval. Crossing times within an interval are unknown, so these are sampled interval
coverage durations, not precise physical event durations. The 0.3 s criterion is a reporting
heuristic; neither a sustained crossing nor a brief crossing proves tire saturation.
Clock discrepancies can affect duration and steering-rate estimates. Event tables enable
comparison with fast steering adjustments without asserting that they caused the slip.

For the initial complete Porto circuit, there are four brief front lateral events, no
rear lateral events, and three brief rear longitudinal events. No event meets the default
sustained criterion. The largest front event has ratio 2.164 over two intervals covering
0.136 wall seconds, with maximum steering-feedback rate about 78.9 degrees/wall second.
The longest rear longitudinal event covers 0.292 wall seconds over five intervals: it
remains brief under the explicitly chosen 0.3 s criterion.

## Raceline tracking error

The lap profiles now include signed geometric tracking error and heading error. Ground-truth
rear-axle position is transformed through the saved map/world origin and projected onto the
nearest segment of the mapped raceline. Error magnitude is Euclidean distance to that
segment; left of the segment's travel direction is positive. Heading error is relative to
the geometric segment tangent, not the raceline file's convention-dependent heading column.
This differs from MAP's error to a lookahead target and does not change when the controller's
lookahead setting changes.

CSV columns are `track_distance_m` (absolute error), `signed_tracking_error_m`, and
`heading_error_rad`. JSON/HTML/Markdown summaries report distance-weighted RMS, sample P95
absolute error, maximum absolute error, distance-weighted signed mean, and sample P95
absolute heading error. The weighting distance and valid interval counts are saved.
Intervals with no valid displacement weight are excluded from these summaries. Map/world
calibration, map geometry, and localization performance can contribute to tracking error;
this measurement does not isolate steering-control error by itself.

For the original complete Porto circuit, tracking RMS is 0.161 m, P95 absolute error
0.344 m, maximum 0.466 m, and P95 absolute heading error 10.70 degrees. These are results
from the original run, not from the new steering-rate experiment.

## Fixed-speed-profile steering-rate experiment

With the simulator connected to the normal bridge in Autonomous mode on the saved Porto
track, and no other controller publishing `/drive`, run inside the sourced container:

```bash
python3 scripts/map_control_experiment.py --output log/NEW_RATE_EXPERIMENT
python3 scripts/compare_map_experiment.py log/NEW_RATE_EXPERIMENT
```

The runner performs the balanced order 3.2, 1.6, 0.8, 0.8, 1.6, 3.2 rad/s. It holds the
raceline, speed scale (0.5), speed cap (none), MAP lookahead, vehicle geometry, LUT and
adapter speed-loop settings fixed. Hashes and actual controller arguments are saved in
`manifest.json`. A steering change can still affect the measured path and actual speed;
fixed speed profile means identical reference configuration, not imposed identical motion.

Each trial resets the simulator, launches ordinary Cartographer localization and MAP,
records raw evaluation topics, and stops after three native lap-count increments. The
first increment is excluded from native flying-lap timing. The second complete geometric
circuit is used for tracking/slip comparisons, avoiding the initial acceleration circuit.
If that circuit cannot be extracted, the fallback report is explicitly excluded from grouped
comparisons. Native lap times and geometric report windows are different definitions and
are labeled separately.

The runner stops on a new collision, a ground-truth gap over one second, excessive vehicle
speed, excessive track distance, early process exit, timeout, changed fixed input hashes,
or interruption. It stops the controller, sends zeros, then stops localization and finalizes
the bag. An incomplete trial stops the sequence for inspection. Existing experiment output
directories are never overwritten. No global tuning defaults are modified.

The comparison generator creates `comparison.html`, Markdown/JSON/CSV data, and a figure
when results exist. Per-trial reports remain linked. It excludes incomplete trials and
non-comparable lap windows from grouped statistics. Two repeats per setting are exploratory;
frame timing, localization variation, and actual speed must be checked before claiming a
lap-time improvement.

## Reconnected experiment attempts and clock discontinuities

After the simulator was restored from quarantine, the balanced experiment was attempted
in `log/map_rate_experiment_connected` and rerun in `log/map_rate_experiment_rerun_01`.
Both stopped during the baseline (3.2 rad/s) after a collision-counter change on the lap
following a completed flying lap. Native flying-lap times were 15.0995 and 15.3106 s,
respectively. Neither is a successful full-protocol trial, and no lower-rate result was
obtained. These attempts do not establish a controller-tuning improvement.

The rerun records simultaneous ROS source-stamp and receiver monotonic-clock intervals:

| ROS stamp interval | Monotonic arrival interval | Native lap counter |
|---|---|---:|
| 2.40655 s | 0.07115 s | 0 |
| 2.52605 s | 0.07686 s | 2 |

These are clock discontinuities, not equivalent gaps in arriving sensor callbacks. The
older raw straight-line trace independently contains the same pattern: a 2.14255 s ROS
interval with a 0.07581 s monotonic interval, and a 2.01330 s ROS interval with a 0.06399 s
monotonic interval. This does not rule out additional genuine simulator freezes; it shows
that a long ROS-header interval alone cannot identify one. The earlier 146 ms monotonic
arrival stall remains a distinct observation.

Such steps can affect the current stack because wheel-speed estimation, controller input
freshness, acceleration ramping and steering-rate limiting use ROS wall time. They are
therefore a confound for steering-gain experiments, independently of tire-slip modeling.
We have not identified or changed the operating-system/time-synchronization cause.

The experiment runner now records clock-discontinuity events when ROS and monotonic
intervals disagree by more than 0.5 s and stops active trials on detection. Comparison
reports explicitly show clock-event counts and exclude affected trials from grouped
comparisons. Excluded measurements are shown as gray crosses rather than successful
results. Existing attempts and their logs are preserved; neither global clocks nor
production controller defaults were changed. The usual bridge remains running, and all
experiment controllers/recorders were stopped after each attempt.

## Current-configuration repeatability check (2026-10-31)

Four fresh attempts were run with identical MAP settings: speed scale 0.5, steering-rate
limit 3.2 rad/s, the same raceline, lookahead, LUT, vehicle geometry and speed adapter.
All fixed controller/input hashes are saved in `log/map_repeatability_20261031/manifest.json`.
No control tuning was applied. The protocol was one warm-up lap and one measured native
flying lap, followed by a stop.

| Attempt | Outcome | Measured flying lap | Tracking RMS |
|---|---|---:|---:|
| 1 | Completed, no collision | 15.0925 s | 0.0535 m |
| 2 | Collision during warm-up | unavailable | not a measured lap |
| 3 | Collision during warm-up | unavailable | not a measured lap |
| 4 | Collision during warm-up | unavailable | not a measured lap |

Attempts 3 and 4 were resumed after the runner's stream-health check was corrected to
refresh its snapshot after report generation. All earlier outcomes were retained; no
failed trial was overwritten. Only orchestration/analysis changed, not controller inputs.
The primary repeatability result is **1/4 successful attempts**, with only one measured
lap. Standard deviations for primary measured-lap metrics are therefore unavailable,
not zero. Clock discontinuities were recorded in every attempt. This is not sufficient
to validate report repeatability, and the current stack did not reliably complete this
short protocol under the observed timing conditions.

For additional context, the two earlier baseline attempts were verified to have identical
fixed controller/configuration hashes and speed scale. Each completed a flying lap before
failing later. Their `native:2` windows were regenerated in their `native_flying_report/`
directories and included in a **separate supplementary completed-lap comparison**. They
do not change the four fresh attempts' failure counts:

| Metric across three completed flying laps | Minimum | Maximum | Sample standard deviation |
|---|---:|---:|---:|
| Native lap time | 15.0925 s | 15.3106 s | 0.1239 s |
| Tracking RMS | 0.0535 m | 0.0836 m | 0.0160 m |
| Front lateral P95 peak-coordinate ratio | 0.595 | 0.714 | 0.061 |
| Front lateral maximum peak-coordinate ratio | 1.083 | 1.447 | 0.183 |

Completed-lap pace is comparatively similar in these samples, while individual peak
values and tracking error vary. These are conditional-on-completion observations with
selection bias, not evidence of overall reliability or absolute tire-model accuracy.
The three completed lap traces are overlaid against mapped circuit progress in
`repeatability_profiles.png`. Failed runs' partial reports remain linked and clearly labeled.

Reproduce the same protocol with a **new output directory**:

```bash
python3 scripts/map_control_experiment.py --repeat-current 4 --output log/NEW_REPEATABILITY
# If interrupted after completed/failed attempts, preserve them and continue the remainder:
python3 scripts/map_control_experiment.py --repeat-current 4 --resume --output log/NEW_REPEATABILITY
python3 scripts/compare_map_experiment.py log/NEW_REPEATABILITY
```

`--repeat-current` records clock issues rather than aborting merely because of a clock
step. Collision, currently stale telemetry, excessive speed/track offset and process
failures still stop an individual attempt. A fresh simulator stream allows the next
identical repeat to reset and run. Normal gain-comparison mode retains its stricter clock
abort. The current detector compares local wall-clock and monotonic intervals sampled
together; source-stamp intervals are recorded separately, avoiding false clock detections
caused solely by queued callbacks during report generation.

Native report selection is available with `--lap native:2`. It joins the interval between
the first and second native counter increments to GT using bag-receipt times. Counter
messages lack source stamps, so endpoints have approximately one frame of uncertainty.
Resets/skipped counters are rejected, and gaps within a counter-defined lap remain flagged;
a full native lap boundary does not imply complete sensor coverage.

Regenerate the saved primary and supplementary summary:

```bash
python3 scripts/compare_map_experiment.py log/map_repeatability_20261031 \
  --earlier log/map_rate_experiment_connected \
  --earlier log/map_rate_experiment_rerun_01
```

The current results are in `log/map_repeatability_20261031/comparison.html` with JSON/CSV,
per-attempt reports, plots, native timing, all failures and actual controller arguments.
All experiment controllers and recorders were stopped afterward; the normal bridge remains.
