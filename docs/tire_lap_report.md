# Tire slip diagnostic tool

Use this offline tool to locate where the simulated tires approach or cross their
configured peak-slip coordinates, relate those events to speed, steering and track
position, and compare repeated laps before changing a speed profile or controller.
It observes a recorded run; it does not impose a slip limit or change driving commands.

**Use Unity telemetry for simulator tire-limit analysis.** The existing kinematic
lateral estimator is too inaccurate for that purpose: it differs in sign and often
substantially in magnitude from native WheelHit slip. Its historical comparisons
remain available for diagnosis, not as a substitute for native slip measurements.

- [Final Unity-only report](reports/unity_slip_20261101/report.html)
- [Consolidated validation](VALIDATION.md)
- [Plugin installation and native comparison](native_slip_validation_20261101.md)
- [Serialized wheel settings](friction_curve_empirical_check.md)

## Record and regenerate

Install the `thegiraffe1/AutoDRIVE` `windows-il2cpp-slip` plugin and launch the bridge
with `publish_wheel_slip:=true`. From the sourced ROS container in `/root/ws`:

```bash
ros2 bag record --use-sim-time -o log/NEW_LAP /clock \
  /ground_truth/odom /drive /autodrive/roboracer_1/wheel_slip \
  /autodrive/roboracer_1/steering /autodrive/roboracer_1/lap_count
# Stop the recorder cleanly, then:
python3 scripts/tire_lap_report.py log/NEW_LAP \
  --track-dir tracks/porto_odom_trial --lap native:2 --unity-only \
  --output log/NEW_REPORT
```

The automated `map_control_experiment.py --repeat-current N --output log/NEW_RUN`
records the native topic along with controller/localizer diagnostics. Use a new
output directory per experiment. `native:2` selects the flying lap after warm-up;
`auto` selects a continuous geometric circuit, `all` includes the entire recording.
Maps/racelines and raw bags remain local generated assets; preserve them separately
if regeneration from another machine is needed.

The JSON `std_msgs/String` topic `/autodrive/roboracer_1/wheel_slip` contains an
integer `stamp_ns` and four named `wheels` entries: `grounded`, `forward`, `sideways`.
The bridge uses the same timestamp as the other sensors in that frame. No driving
node consumes this topic. Ungrounded wheels are unavailable rather than zero slip.

## Reading the report

The self-contained `report.html` embeds lap profiles and track maps. CSV/JSON exports
save samples, crossing events, summary metrics, settings and the regeneration command.

- Front/rear lateral ratios use the larger absolute `sidewaysSlip` on each axle.
- Front/rear longitudinal ratios use the larger absolute `forwardSlip` on each axle.
- Both wheels must have valid ground contact for an axle-level ratio.
- Primary native metrics use instantaneous source-stamp-matched values, preserving
  observed single-frame peaks. Missing native values never fall back to estimates.
- Profile context includes steering, steering rate, tracking/heading error, speed
  and sensor intervals. Ground-truth position supplies track projection.
- Maps draw larger, outlined crossing markers above ordinary samples. Diamonds and
  `E` labels identify event peaks and correspond to the event table.

The default peak coordinates are lateral **0.01** and longitudinal **0.15**; the
serialized Windows scene agrees with these values. Runtime overrides have not been
measured. A ratio of 1 means the configured peak-slip coordinate, **not** 100% tire
force, a hard safety boundary or proof of saturation. The 0.8 line is a reporting
heuristic. Colors cap at 2; exported values remain unclipped. Event duration and
valid-distance exposure are sampled approximations; peaks between frames can be missed.

A crossing is classified as sustained after 0.3 transport-clock seconds and three
consecutive valid samples. Missing data, segment changes and long frame gaps break
runs; long-gap crossings are timing-uncertain. These labels describe sampled
persistence, not measured physical force saturation. Native lap boundaries use
counter receipt times, with approximately one frame uncertainty.

## Sources and historical estimates

`--slip-source auto` prefers Unity for the whole selection when available.
`--slip-source unity` requires telemetry but retains the diagnostic comparison.
`--unity-only` requires Unity and omits estimated-slip comparisons and fields.
`--slip-source kinematic` explicitly reproduces the older estimate-based analysis.
Old bags without native telemetry cannot be upgraded retrospectively. Cross-run
comparison rejects pooling Unity and kinematic slip metrics.

The old lateral method derives wheel-point velocity from body velocity/yaw rate,
assumes a velocity reference and wheel geometry, infers Ackermann wheel angles,
and computes `tan(alpha)`. It is sensitive to small velocity/reference/angle errors
and does not establish Unity's internal slip definition. Native-versus-estimate
plots preserve raw signs; reversing a convention does not fix magnitude error.
Rear longitudinal encoder-travel estimates agree more closely with Unity in the
recorded laps, but also remain approximations. Optional `--settings file.json`
overrides thresholds and estimator parameters; effective values are saved.

## TODO: real-vehicle slip estimation

Before using lateral estimates live on the car, establish synchronized acquisition
stamps, actual per-wheel steering/geometry and the body-velocity reference; validate
wheel/contact-axis signs and slip definitions; and evaluate a lateral-velocity or
sideslip estimator against an independent reference across speed and steering sweeps.
An IMU and wheel encoders alone do not directly measure body lateral velocity.
Do not enable a lateral slip feedback limit using the current approximation.
