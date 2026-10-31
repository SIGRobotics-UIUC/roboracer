# Hairpin slowdown and wrong-way steering investigation

The recorded failures explain the observed behavior: a timestamp step corrupts the
estimated state, MAP briefly stops for stale data, then resumes with a fresh timestamp
but an incorrect heading and wheel speed. It commands extreme steering in the wrong
direction. This investigation did not change production control settings or perform
additional driving.

Artifacts: `ws/log/hairpin_investigation/report.html`, per-run JSON, and plots. All inputs
are finalized bags from the four-repeat current-configuration experiment.

## Primary hairpin case: repeat 4

| Quantity | Before timestamp step | First resumed control sample |
|---|---:|---:|
| Simulator body speed | 1.051 m/s | 1.038 m/s |
| Wheel odometry speed | 1.099 m/s | 0.0284 m/s |
| Requested speed | 1.089 m/s | 1.089 m/s |
| Steering command | -16.22 degrees | +30.02 degrees |
| Localized heading error | small before event | -118.77 degrees |

The ROS source-stamp interval is 2.49335 s, but the runner's independently sampled
wall/monotonic-clock diagnostic gives a monotonic callback interval of only 0.06827 s.
Ground-truth position advances approximately 7 cm and yaw changes only -3.31 degrees
across that interval. The localized pose instead briefly extrapolates about **2.685 m**
and turns about **120 degrees** away from reality. After the sensor frame arrives, its
position partly recovers, but heading remains wrong. At the first nonzero resumed
command, localized position is still 0.212 m from ground truth.

The collision counter changes **0.911 s after the first post-step GT stamp**. Source-clock
time after the jump is used for this delay; it is not a claim that the preceding 2.493 s
was physical elapsed time. Timeline plots use sensor-frame index to avoid displaying the
clock step as a real multi-second motion pause.

## Failure chain and code evidence

1. **Wheel velocity is divided by elapsed ROS-stamp time.**
   In `wheel_odom.integrate`, cumulative encoder travel is differentiated over its history
   using source timestamps. Its nominal 0.1 s speed window retains at least two samples.
   Across a positive clock step, that pair can span 2.493 s even though wheel travel is
   only one normal frame. About 0.071 m / 2.493 s produces the recorded 0.0284 m/s.
   Thus the near-zero speed is initially an estimator artifact, not the actual car stopping.

2. **Localization extrapolation behaves as if the artificial interval were physical time.**
   The extrapolated displacement and yaw are consistent with prior velocity/yaw-rate
   integrated across the clock step. Before the step the yaw rate is -0.859 rad/s;
   multiplying by 2.493 s gives about -123 degrees, close to the erroneous extrapolation.
   The physical frame-to-frame yaw change is only about -3.3 degrees. This identifies an
   extrapolation signature; the installed Cartographer C++ binary was not instrumented
   internally, so its precise internal state evolution is inferred from outputs.

3. **MAP's freshness check briefly commands zero speed/steering.**
   `MAP._tick` rejects stale pose/odometry using ROS wall time. The log records
   `MAP stop: stale or future input`. Two zero `/drive` messages precede resumption.
   Physical body speed then decreases further over subsequent frames, but the initial
   discontinuity in wheel speed is much larger than the actual slowdown.

4. **Fresh timestamps permit resumption with a bad heading.**
   New pose timestamps become current before heading has recovered. The positional jump
   guard cannot guarantee recovery: the extreme extrapolated point can be rejected for
   stale odometry, then its position partly recovers while heading is still wrong.
   There is no independent angular-consistency check in the current MAP input gate.

5. **Wrong heading changes the sign of the lateral guidance demand.**
   The actual car is continuing a right turn, but the erroneous estimated heading makes
   the next raceline point appear to require the opposite steering direction. The false
   low wheel speed also switches guidance/lookup to the LUT's 0.9 m/s minimum column.
   The requested acceleration reaches that column's support and the steering query is
   clamped to positive lock.

6. **The first resumed command bypasses the controller's slew history.**
   `MAP.stop()` clears `last_steer` and `last_command_time`. The first resumed query is
   therefore not bounded relative to the pre-stop steering command. The simulator still
   has its own actuator dynamics, but the controller requests the full reversal immediately.

This is not the normal speed profile deliberately steering into the wall. The nonzero
requested speed before and after the event is unchanged in this case. Nor does the
recording establish a tire-saturation event as the initiating cause. Timing-induced
state corruption precedes the erroneous steering command.

## Isolated offline counterfactuals

Using the same model/LUT, raceline and local guidance calculation:

| Query inputs | Steering result |
|---|---:|
| Recorded localized pose and wheel speed | +30.02 degrees |
| Same position/speed; ground-truth heading only | -4.01 degrees |
| Same position; ground-truth heading and body speed | -4.35 degrees |
| Ground-truth position, heading and body speed | -17.61 degrees |

The recorded-input query reproduces the actual first resumed command. Correcting heading
alone changes its direction; correcting the position as well restores a substantial
right-turn request. This isolates the immediate steering mechanism, not the outcome of a
full closed-loop alternative. Nearest raceline index is used to initialize offline target
history; the report records that assumption. No ground truth was fed to the live controller.

## Other failed repeats

| Repeat | ROS step | Body / odom speed after step | Heading error at resume | Steering before → after | Collision delay after post-step frame |
|---|---:|---:|---:|---:|---:|
| 2 | 1.658 s | 1.746 / 0.105 m/s | -42.39 degrees | -4.26 → +30.02 degrees | 0.679 s |
| 3 | 2.080 s | 2.519 / 0.081 m/s | +73.66 degrees | +2.66 → -27.02 degrees | 0.312 s |
| 4 | 2.493 s | 1.038 / 0.028 m/s | -118.77 degrees | -16.22 → +30.02 degrees | 0.911 s |

Repeat 3 fails in a different section of the track. It exhibits the same input-corruption
mechanism, so the problem is not exclusive to one hairpin or one planned curvature.
Turning makes the consequence more visible because heading errors immediately change
where the controller believes the path lies.

## What should be addressed before tuning this turn

- Establish a consistent clock across sensor stamps, wheel odometry, localization and
  controller timing. Changing bridge stamps alone while other nodes use jumping wall time
  would not be a complete solution. The host/time-synchronization source is not yet fixed.
- Treat a clock discontinuity as an estimator recovery event: hold the car stopped until
  velocity and localized heading are revalidated, rather than resuming merely on fresh stamps.
- Add recovery checks for implausible heading changes and explicitly bound resumed steering
  relative to actual/recent steering. These are proposed changes, not implemented fixes.

The evidence supports prioritizing timing and recovery handling over reducing corner speed
or tuning tire-limit penalties. Those may change symptoms while leaving the initiating
state-estimation failure intact.

## Regenerate

Inside the sourced ROS container:

```bash
python3 scripts/diagnose_map_failure.py \
  --bag log/map_repeatability_20261031/04_rate_3p2/bag \
  --bag log/map_repeatability_20261031/02_rate_3p2/bag \
  --bag log/map_repeatability_20261031/03_rate_3p2/bag \
  --output log/hairpin_investigation
```

The generator opens finalized SQLite bags read-only, reconstructs nearby observations,
replays isolated guidance queries, and writes JSON, PNGs and a self-contained HTML report.
Plots mark the timestamp step and collision counter change. No ROS node is started.
Relevant production code is `ws/src/autodrive_bridge/autodrive_bridge/wheel_odom.py`,
`ws/src/control/control/map_node.py`, and `ws/src/control/control/map_guidance.py`.
