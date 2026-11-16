# Evaluation-only wheel-contact telemetry

The additive implementation follows `wheel_contact_telemetry_plan.md`. Physical
force/load interpretation, tire utilization, saturation detection and grip
feedback remain gated on its simulator experiments. None of these signals feeds
localization, planning, control, the benchmark command gate, or reset decisions.
Existing localization-gate/bridge ground-truth dependencies remain review debt;
this change does not make the default race stack ground-truth independent.

## Components and opt-in

The **separate** `autodrive_plugin/` checkout contains the Windows plugin changes.
The RoboRacer checkout contains the ROS bridge, benchmark module, synthetic
contract fixture, tests and this documentation. Do not commit the plugin checkout
into RoboRacer or replace an installed DLL as an incidental build step.

Build the plugin using its existing Windows installer/build workflow against the
target simulator's BepInEx bindings and .NET 6 targeting pack. Preserve the old
installed DLL before explicitly installing a new build. The existing installer
has unrelated local changes; they are not part of this implementation.

After installation, enable the following in
`BepInEx/config/uiuc.autodrive.wheel-slip.cfg`, while the simulator is stopped:

```ini
[Telemetry]
WheelContact = true
```

Default is false. Sampling does not edit the installed configuration or DLL.
Authorized deployment can use the separately reviewed personal Windows runner.
The read-only control command `contact` returns the latest cached JSON,
or an unavailable response if disabled or no sample exists. `slip` is unchanged.

Plugin 0.3.6 also exposes **mutating**, stationary-only diagnostic commands:
`contact-trial mass_090|mass_110|wheel_mass_080|wheel_mass_120|step_0005|step_0020`
and `contact-trial restore`. These require the designated
`autodrive_with_plugin_codex` directory, enabled telemetry, grounded wheels,
rest and zero motor torque. Body-mass cases scale by 0.9/1.1; wheel-mass cases
scale all four by 0.8/1.2; timestep cases set 0.0005/0.002 seconds. They restore
the original value explicitly, on reset, on actuation/horizontal movement, or
after 12 real-time seconds at the next main-thread callback. A blocked main
thread can delay restoration. Lifecycle state is in `runtime_snapshot.stationary_trial`.
These are isolated evaluation interventions, never a controller input or a
substitute for the benchmark command gate. Verify applied and restored values
and fresh ROS streams; retain aborts as failures. See the current assessment for
actual outcomes and limitations.

Rebuild/source the ROS packages, then explicitly enable contact publication:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select autodrive_bridge benchmark
source install/setup.bash
ros2 launch autodrive_bridge autodrive_launch.py publish_wheel_contact:=true
```

`publish_wheel_slip:=true` remains independent. Neither switch enables plugin
sampling by itself. An old plugin produces no contact topic samples; the bridge
never invents missing data. An old bridge ignores the added Socket.IO key.

Set `logging.level: detailed` in a benchmark configuration to inventory and record
`/autodrive/roboracer_1/wheel_contact` as `std_msgs/msg/String`. It is optional:
absence does not block readiness, motion or finalization. Existing benchmark
ownership and stop rules still apply; do not start another controller.

Analyze a sealed run into a new report directory with the existing CLI:

```bash
ros2 run benchmark benchmark --log-root benchmark_runs \
  analyze RUN_ID --analysis-id contact-review --modules wheel_contact
```

`--modules all` also includes `wheel_contact`. Old/basic bags explicitly report
contact metrics unavailable. Reports do not import or execute `ws/scripts/` and
never write inside raw runs. The signal-map version is now 3.

## Wire and sampling contract

`V1 Wheel Slip` retains its four grounded/forward/sideways triples and order
FL, FR, RL, RR. With contact disabled it retains the existing acquisition point.
With contact enabled both transported payloads reuse the same cached contact
reading. The HUD/legacy diagnostic may still acquire its own display samples.
The existing native allocated/unboxed WheelHit ABI workaround is unchanged;
new fields use generated accessors, not guessed memory offsets.

`V1 Wheel Contact` transports `base64:` followed by UTF-8 JSON encoded as base64,
so the stock Unity JSONObject serializer never needs to escape embedded quotes.
The bridge also accepts the earlier plain JSON string form. Published ROS String
messages contain ordinary JSON, schema version 1, evaluation-only. See
`docs/fixtures/wheel_contact_v1.json` for a synthetic example. Its snapshot is
explicitly synthetic, not evidence for actual installed configuration.

The opt-in injected MonoBehaviour counts observed FixedUpdate callbacks and
samples once in LateUpdate after new callbacks. Sampling and Unity API access
stay on Unity's main thread. The Emit hook reads the cache and does not call
Unity APIs in contact mode. Multiple Emit observations can carry the same
`session_id/reset_generation/sample_sequence`; repeated samples are counted and
excluded from numerical distributions. There is no fresh sample during a
physics pause. A whole-sample acquisition failure clears the cache and logs an
error instead of transmitting a stale hit as a new sample.

LateUpdate normally follows automatic physics, but the installed player's loop,
manual simulation and actual solved-state acquisition have **not** been
validated. `phase_verified` and `physics_step_identity_verified` remain false.
`observed_fixed_callback`, `observed_fixed_time`, rendering frame,
`fixedDeltaTime`, and `skipped_fixed_callbacks` are observations, not a guaranteed
solver-step ID or a complete physics trace. Intermediate fixed steps can be
missed at low render rates. No every-solved-step local recorder is implemented.
Sampling-phase validation remains an explicit prerequisite for physical metrics.

A process-session ID disambiguates plugin restarts. Observed completed
ResetManager requests, scene loads and track changes invalidate the cache and advance reset
generation; these are diagnostic boundaries only. The reset hook is not installed
when contact mode is off. Scene/automatic reset coverage still requires runtime
verification; position/velocity are never used to infer resets here.

Each sample contains:

- Four named wheels, collider/vehicle instance IDs (scoped to the session),
  grounded and validity flags, sampling error, raw force/slips/contact vectors,
  contact collider identity/material name, both runtime friction curves, RPM,
  steering degrees and applied motor/brake torques.
- A complete runtime snapshot with Rigidbody mass, local/world CoM, pose/scale,
  gravity, inertia, drag, constraints, simulation/solver settings, wheel-center
  geometry and steering axes, wheel mass/radius/sprungMass, suspension properties,
  force application height and damping. Other active rigidbodies and vehicle-root
  component types are inventoried. This does not prove absence of other supports,
  forces or anti-roll behavior; external-force inventory is marked unverified.
- A configuration revision for observed stable Rigidbody/wheel settings, plus
  the full snapshot in every sample. Snapshot changes outside that revision's
  fields remain visible in the full sample. World pose alone does not advance
  the revision. Missing/invalid numeric snapshot values set
  `runtime_snapshot_valid=false`.
- Startup hashes of the actual plugin, native GameAssembly.dll, installed level0
  and generated game binding; Unity/plugin version. Missing files yield null
  hashes, not fabricated provenance. The binding hash is not the native game
  executable hash. Record both Git revisions and dirty diffs with trial results;
  DLL hashes identify binaries but do not prove reproducible source correspondence.

Ungrounded `wheel_hit` is null, not zero force or zero slip. Nonfinite API values
use `{"nonfinite":"NaN"}`, `+Infinity` or `-Infinity` tags; affected wheel data is
marked invalid. A tagged wheel cannot claim valid=true. Raw negative forces are
retained. Whole native-read failures yield unavailable samples, not a fabricated
grounded=false observation. Unity vectors remain in raw left-handed world
coordinates (x right, y up, z forward); no implicit ROS conversion is performed.

The bridge validates bounded JSON envelopes (256 KiB, nesting limit 32), adds
`stamp_ns` from the same arrival stamp used for that bridge frame, and retains
`arrival_monotonic_ns` from callback entry separately. Malformed telemetry is dropped with throttled
warnings without interrupting normal sensors/commands. Source clock evidence is
preserved; it is never equated with transport `/clock` or Windows wall time.

## Analysis semantics and remaining validation

The module reports signed raw slip/force distributions, normal magnitudes,
grounded sample fractions, invalid/ungrounded counts, raw four-grounded-force
sums, session/configuration counts, observed sequence gaps, repeated source
samples, skipped callbacks and provenance availability. `contacts.jsonl` retains
all selected envelopes, including invalid-wheel tags and full runtime snapshots,
with recording time and alignment segment.

Ratios are `abs(raw slip)/runtime extremumSlip`, independently for each axis.
All five curve values must be finite; slip knots must be positive and ordered,
values nonnegative and stiffness positive. Ratios are not clipped. The
at-or-above-one statistic is a **fraction of valid observed samples**, not elapsed
time, saturation or a fraction of available grip. There is no reconstructed
friction spline, force vector, friction circle or combined-slip model.

Run the controlled experiments in the plan on new isolated trials, with an
independent stop mechanism and race stack stopped. Ground-contact/ABI checks,
wheel lift, runtime overrides, reset/track coverage, static balance, mass/CoM
shifts, solver/phase sensitivity, acceleration/braking, mirrored circles, and
axis/combined sweeps remain unperformed. Do not promote raw force to Fz or
implement physical tire utilization until those acceptance gates pass.

## Read-only stationary capture across isolated containers

If the development container cannot discover the bridge's ROS graph, run this
from a second shell **inside the bridge container**, on the bridge's ROS domain:

```bash
cd /root/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH=/root/ws/src/benchmark:"${PYTHONPATH}" \
  /usr/bin/python3 -m benchmark.analysis.contact_probe --seconds 30
```

Leave the vehicle stationary, without a controller/teleop running. This helper
creates no drive/reset publishers. It records contact payloads and observes drive
and low-level commands into a new exclusive directory under
`/root/ws/log/wheel_contact_checks/` (shared as `ws/log/wheel_contact_checks/`).
Malformed contact payloads are retained with decode errors. Missing live contact
data causes a bounded discovery timeout, not a fabricated success. Raw snapshots,
source revisions/diffs when available, graph publisher names and recording counts
are saved for analysis. Captures are diagnostic artifacts, not sealed benchmark
race runs. One continuous capture is not three independent trials, and the helper
does not establish ground flatness, stationarity or absence of other supports.


The first live stationary Porto capture and its unresolved static-support/timing
findings are documented in [stationary validation](wheel_contact_static_validation_20261116.md).
It does not validate physical load semantics or dynamic driving behavior.

## Offline lap diagnostics (wheel_contact 1.1)

The benchmark's module-owned **Wheel Slip Diagnostics** viewer uses measured
contact envelopes and preserved localized map-frame poses/assets. It introduces
no driving or collection changes. `contacts.jsonl` retains retransmissions;
`slip_samples.jsonl` keeps the earliest eligible arrival for each
`(session_id, reset_generation, sample_sequence)`, records conflicting duplicate
counts, and exports per-wheel signed raw slip, runtime peaks, validity and
nonnegative peak-slip ratios with pose/progress/lap association evidence.

The four plots show forward slip, sideways slip, forward peak-slip ratio and
sideways peak-slip ratio. Raw units remain Unity slip coordinates, never slip
angle/degrees or conventional slip ratio. Ratio 1 marks the configured peak-slip
coordinate; larger values remain visible. Map markers locate the vehicle, not
individual wheel contacts. Exact timestamps are preferred; bounded bracketed
interpolation estimates only position/yaw in verified transport time. It never
interpolates slip, extrapolates, or crosses reset/session/clock boundaries.
Unmatched observations still contribute to the full-window summary below the
plots. That summary uses separate raw/ratio counts and sample-weighted absolute
magnitudes/tails; `Samples at/above configured peak-slip coordinate` is a count
and fraction of valid ratios, not time saturated or grip used. Configuration
revision counts include possible numerical noise; denominator-change counts
and ratios use observed runtime curves.

Missing data stays unavailable. This reporting extension does not resolve the
existing localization-gate ground-truth/map-origin dependency or bridge automatic
respawn-detection review debt. It adds no ground-truth input to the slip viewer
and makes no claim that the collection stack is ground-truth independent.


The viewer is integrated into Race Performance: use its Track overlay dropdown
for a wheel's slip/peak-slip color map, or expand Wheel slip on the plot side.
It reuses the original SVG track, default overlay, lap controls and selection;
there is no separate slip map. See [the modular viewer API](benchmark_viewer_extensions.md).
