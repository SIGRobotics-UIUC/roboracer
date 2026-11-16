# Wheel-contact telemetry investigation and validation plan

Status (2026-11-16): additive plugin/bridge/benchmark software is implemented;
see [implementation and validation limits](wheel_contact_telemetry.md). The source
inspection below describes the pre-implementation baseline. A first live read-only
[stationary Porto check](wheel_contact_static_validation_20261116.md) completed;
static force semantics and sampling-phase acceptance remain unresolved. No physical
tire-utilization metric or saturation detector was implemented. Later authorized
runner deployments, verified net6.0 implementation hashes, native sleep-Boolean
correction and repeated stationary mass/wheel-mass tests are documented in the
[current assessment](tire_utilization_assessment.md). The static support residual
and dynamic acceptance gates remain unresolved; historical inspection details
below are not the current deployed-version record.
Do not use these evaluation signals in driving decisions.

## Inspected sources and installed build

- Plugin source is the separate Git checkout `autodrive_plugin/`, origin
  `thegiraffe1/AutoDRIVE`, branch `windows-wheel-slip-hud`, revision `f890810`.
  It was clean during inspection. The installed simulator directory
  `D:\Documents\f1tenth\autodrive_win_il2cpp_test` is not itself a Git checkout.
- Its BepInEx log identifies Unity `2022.3.52f1` and the wheel-slip plugin.
  Installed `BepInEx/plugins/TrackSelect.Windows.dll` SHA256:
  `101afe0460ed3804949c1fbf086165c16619dd82803af345da26312d93f5c783`.
  Do not assume the binary exactly matches the current source revision without
  a reproducible build comparison.
- Installed `Data/level0` SHA256:
  `5edbd1bacc15647746e6e89d9bcf1bb9e202d62d47e76ae352497be1404b3f8d`.
  This matches `docs/autodrive_wheel_settings_20261031.json`. That extraction is
  evidence for serialized defaults, not for runtime overrides.
- `WheelSlipData.Read()` calls the existing unsafe IL2CPP `GroundHit` wrapper
  for all four named VehicleController WheelColliders. It reads contact status,
  signed forward/sideways slip, friction extremum-slip settings, and RPM.
  The wrapper deliberately allocates native WheelHit storage because the generated
  out-parameter wrapper is unsafe. Preserve this workaround and use generated
  WheelHit field accessors rather than introducing guessed struct offsets.
- `WheelSlipPlugin` patches Socket.IO `Emit("Bridge", data)` and appends
  `V1 Wheel Slip`: four grounded/forward/sideways triples. The bridge expects
  exactly 12 finite numbers and publishes stamped JSON when
  `publish_wheel_slip:=true`. RPM/peak settings appear in the plugin diagnostic
  response, not the existing ROS slip payload.
- Benchmark detailed logging inventories the slip topic; its numerical signal map
  does not yet decode the proposed contact stream. It must not import or invoke
  the old `ws/scripts/tire_lap_report.py` to implement a future module.

## Vehicle configuration evidence and gaps

All four serialized wheels have radius 0.059 m and the following curves:

| Direction | extremumSlip | extremumValue | asymptoteSlip | asymptoteValue | stiffness |
|---|---:|---:|---:|---:|---:|
| Forward | 0.15 | 0.90 | 0.25 | 0.58 | 0.80 |
| Sideways | 0.01 | 1.00 | 0.10 | 0.50 | 1.00 |

The resulting configured value scales at the extrema are 0.72 and 1.00,
respectively. These are not measured forces in newtons or validated friction
coefficients. Capture runtime settings before using them as experiment evidence.

`planning/config/racecar.ini` has mass 3.906 kg, wheelbase 0.330 m, track width
0.236 m, CoM-to-front/rear distances 0.175/0.155 m and CoM height 0.074 m.
Its comments explicitly describe several assumptions: guide wheelbase differs,
CoM reference is assumed, height is derived, and roll distribution is estimated.
These are planner inputs, not authoritative installed Rigidbody properties.

The prerequisite runtime snapshot must contain Rigidbody mass, local/world CoM,
pose/scale, gravity, inertia tensor/orientation, constraints and drag; actual
wheel-center geometry and steering axes; radius/mass, sprungMass, suspension
spring/damper/target position/travel, forceAppPointDistance, wheel damping,
steerAngle, motorTorque, brakeTorque and RPM. Include other rigidbodies, extra
forces/anti-roll logic, contact-surface identity and any runtime setting changes.
Do not add WheelCollider.mass to Rigidbody.mass as extra gravitational mass
without establishing the engine's mass accounting.

Unity calls sprungMass the mass supported by a WheelCollider and describes its
configured rest distribution. It is not an instantaneous load sensor.
[Unity sprungMass](https://docs.unity3d.com/2022.3/Documentation/ScriptReference/WheelCollider-sprungMass.html).
Force application height must also be recorded because it affects vehicle
moments: [forceAppPointDistance](https://docs.unity3d.com/2022.3/Documentation/ScriptReference/WheelCollider-forceAppPointDistance.html).

## Minimal additive telemetry extension

Keep `V1 Wheel Slip` unchanged. Add an opt-in, versioned `V1 Wheel Contact`
JSON payload and a separate ROS String topic
`/autodrive/roboracer_1/wheel_contact`. Old plugins, bridges, bags and reports
continue to work. This avoids changing the existing fixed 12-number decoder.

Reuse one contact reading per wheel for the old and new payloads. Proposed
per-wheel fields, with the original names retained under a `wheel_hit` object:

- Stable name and WheelCollider/vehicle identity; `grounded` (GetGroundHit return).
- `wheel_hit.force`, unchanged scalar; label **raw WheelHit.force**.
- Signed `wheel_hit.forwardSlip` and `wheel_hit.sidewaysSlip`, without conversion
  to conventional slip ratio or slip angle.
- `wheel_hit.normal`, `point`, plus `forwardDir`/`sidewaysDir` for axis validation.
  Declare Unity world coordinates/handedness and units explicitly; retain raw
  Unity vectors even if downstream analysis creates converted ROS copies.
- Both runtime friction curves: `extremumSlip`, `extremumValue`, `asymptoteSlip`,
  `asymptoteValue`, `stiffness`. Initially including all ten numbers per wheel in
  each contact message is simpler and makes every sample self-contained. A later
  configuration revision/hash optimization must preserve the full snapshots.
- Contact collider identity, sampling error/validity and configuration revision.
  When ungrounded, set contact-dependent fields unavailable/null; do not interpret
  default/stale WheelHit memory as zero force or valid zero slip. Preserve raw
  nonfinite values as explicit tagged representations and mark them invalid,
  rather than replacing them silently with zero or invalid JSON numbers.

Unity documents `force` as a scalar contact-force magnitude. Its public WheelHit
API supplies no separate longitudinal/lateral force components. Neither that
name nor the contact normal establishes `force == Fz`; multiplying it by the
normal would be a model assumption, not an exposed vector measurement.
[WheelHit](https://docs.unity3d.com/2022.3/Documentation/ScriptReference/WheelHit.html),
[force](https://docs.unity3d.com/2022.3/Documentation/ScriptReference/WheelHit-force.html).

Sample on Unity's main thread. Include a sample sequence, reset generation,
observed physics-step identity/time, fixedDeltaTime and rendering frame number.
The current Emit hook is not proof of post-physics acquisition: establish and
label its phase, cache one coherent sample, and identify repeated observations
of the same solved state. For validation, optionally record every solved physics
step locally, comparing it with the transported/downsampled stream. A Unity-side
sample clock must not be equated with the bridge's host-monotonic `/clock`.

Record the runtime vehicle/configuration snapshot and build/plugin hashes at
startup, track changes, resets and settings changes. Keep bridge arrival stamps
as well as source-step evidence. Missing new telemetry remains unavailable in
old collections. These additions are evaluation-only and never feed a controller
or localizer.

## Controlled validation experiments

Use a new isolated run for every trial, flat uniform ground and an independent
stop mechanism. Keep the race stack stopped during controlled manual experiments.
Repeat at least three times, retain transients and compare settled windows
separately. Define acceptance tolerances from baseline repeatability, solver-step
sensitivity and instrument precision before interpreting dynamic results.

1. **Ground-contact and ABI checks.** At rest, check finite vectors/force and
   near-unit normals; compare surface contact points to wheel geometry. Lift one
   wheel in a separate diagnostic scene: verify grounded=false, unavailable contact
   fields, and consistent remaining contact behavior. No negative-force clipping.
   Ensure reading telemetry does not alter reset, slip or command behavior.
2. **Static balance.** On level ground, with commands zero and suspension settled,
   compare the four raw force values, their sum, axle sums and side sums with
   expected gravity support. First establish absence of chassis-ground contact,
   non-wheel supports, external forces and vertical acceleration. For a rigid,
   planar idealization, if a is CoM-to-front and b CoM-to-rear, L=a+b,
   expected front support is mg*b/L and rear support mg*a/L. Symmetric halves are
   justified only with symmetric geometry and lateral CoM. Compare independently
   with runtime sprungMass*g. Test controlled mass and CoM shifts in a diagnostic
   copy: proportional scaling and front/rear redistribution help discriminate a
   load-like quantity from a friction resultant or fixed configuration value.
3. **Longitudinal transfer.** Straight steering, smooth acceleration and braking
   plateaus, initially below substantial wheel slip; include coast-down for drag
   identification. Use measured Rigidbody CoM acceleration in a documented body
   frame, not commanded acceleration or wheel speed. The ideal planar prediction
   transfers delta=m*ax*h/L from front to rear during forward acceleration;
   braking reverses it. Check slopes, intercepts, symmetry, total support, pitch
   transients and solver-step dependence. The moment arm h must agree with actual
   force-application geometry; an artificial force application height can change
   this prediction. Never silently fit an effective height and call it measured
   CoM height. Countercheck against springs/damping and any non-wheel contacts.
4. **Lateral transfer.** Run steady constant-radius left/right circles at matched
   speeds below large slip, then increase cautiously. Compare body lateral
   acceleration, roll and outside/inside sums. For equal track width t, the ideal
   amount shifted from inner to outer is delta=m*abs(ay)*h/t: outer increases by
   delta, inner decreases by delta, so outer-minus-inner changes by 2*delta.
   With unequal tracks, compare roll moments using each axle's own half-track.
   Do not assume a 50/50 front/rear roll-transfer split: spring stiffness,
   geometry, anti-roll logic and force application points determine it. Mirror
   directions and remove baseline lateral-CoM bias. Departures from the ideal
   balance are evidence to explain, not grounds for renaming raw force as load.
5. **Friction sweeps, only after the above.** Quasi-steady straight traction/braking
   sweeps for longitudinal behavior and constant-speed steering/skidpad sweeps for
   lateral behavior. Hold surface, load/configuration and timestep controlled;
   test separate axes before combined slip. Log actual torque, steer, speed,
   acceleration, angular acceleration and curve changes. Aggregate force inferred
   from rigidbody momentum requires gravity, drag and other forces accounted for;
   it does not identify individual tire Fx/Fy without additional assumptions.

## What the friction curves can and cannot establish

Unity describes separate slip-input, force-output curves with two spline pieces,
zero tangents at the extremum and asymptote, and a stiffness multiplier on their
values. This supports investigating a **configured curve-response estimate**.
It does not by itself validate Newton-valued forces, the load normalization,
combined-slip coupling, low-speed regularization or the actual peak under a
particular runtime contact state.
[WheelFrictionCurve](https://docs.unity3d.com/2022.3/Documentation/ScriptReference/WheelFrictionCurve.html).

Keep three evidence categories separate:

| Category | Examples | Allowed interpretation |
|---|---|---|
| Direct simulator telemetry | grounded, raw force/slips/vectors, runtime curves | Report original API semantics and validity |
| Model-derived estimates | ideal static/transfer predictions; candidate spline response; body force balance | State equations, assumptions, uncertainty and tested regime |
| Heuristic slip proxies | abs(forwardSlip)/forward extremumSlip; lateral equivalent; sampled time above threshold | Position relative to configured slip coordinate only |

A ratio near one identifies a configured peak-slip coordinate. It is not a
percentage of available grip. Beyond the extremum, larger slip may correspond to
lower force, so a clipped monotonically increasing utilization gauge is wrong.
An assumed cubic interpolant matching the documented knots/tangents is still a
model reconstruction until checked against the installed engine. Runtime settings
must be valid (positive ordered slip knots, finite values, appropriate stiffness)
for even that proxy. Do not assume a friction circle/ellipse combines the axes.

Acceptance gate: first resolve force semantics and time alignment; then establish
runtime configuration and force/load mapping; then validate axis sweeps and
combined behavior against independent dynamics evidence. Until those gates pass,
do not implement physical tire utilization, saturation flags or grip feedback.
A future benchmark module can initially report raw contact diagnostics and
clearly named slip-to-configured-peak proxies only.

## WSL versus a native Windows Codex session

No separate session is required for this investigation or source design. WSL can
read the installed build via `/mnt/d/...`; the source repo is separately available
at `autodrive_plugin/`. The installed simulator is a binary deployment directory.
A Windows .NET host exists at `C:\Program Files\dotnet\dotnet.exe`, but no Linux
`dotnet` was found on PATH; the simulator's bundled `dotnet/` is a runtime, not
proof of an installed SDK. No build was attempted here.

For implementation, a native Windows session opened on the AutoDRIVE source repo
is convenient for Windows SDK/BepInEx references, installer PowerShell, native
launch/log inspection and Unity testing. WSL is suitable for bridge/benchmark
changes and recording analysis. Either workflow is viable; use temporary build
outputs and preserve the installed DLL until installation is explicitly wanted.
Keep plugin and RoboRacer changes in their respective Git repos, record both
revisions, and avoid simultaneous edits to the same checkout. No deployment or
vehicle motion was performed in this investigation.
