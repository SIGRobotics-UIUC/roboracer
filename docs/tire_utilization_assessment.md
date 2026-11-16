# Tire-utilization estimate: evidence and investigation conclusion

Date: 2026-11-16. The investigation reached the bounded conclusion below; physical
tire-utilization validation remains incomplete. Initial inspection was read-only.
Later sections record authorized
deployment through the fixed Windows runner, stationary resets and bounded
parameter interventions in the designated diagnostic simulator copy. No driving
commands were sent. Plugin code executes on Windows; container-side inspection
of installed bindings uses metadata rather than loading their executable code.
Historical checkpoints below are superseded where later evidence says so.

Current conclusion: useful per-wheel slip/contact diagnostics are available;
physical percentage-of-grip estimates have no validated accuracy. Static raw
force follows body-mass changes but is strongly timestep-sensitive, and wheel
mass does not explain its weight discrepancy. See the final two sections for
the controlled experiments, failed-case disclosure, current 0.3.6 deployment
identity and remaining physical-validation boundary.

## What is meaningful now

| Candidate | Supported interpretation now | Principal limitation |
|---|---|---|
| Per-wheel `abs(slip)/extremumSlip`, separately per axis | Position relative to the runtime configured peak-slip coordinate, grounded/valid only | Not force fraction, available grip or saturation; depends on valid curves and adequate sampling |
| Time history of signed slips, contact state and runtime settings | Direct simulator diagnostics at transported observations | Intermediate physics states are missed; source phase is not verified |
| Configured friction-curve response normalized by its configured peak | At most a model response coordinate, after stating the reconstruction | The installed interpolation, force/load mapping and combined-slip law have not been established |
| Per-axle/whole-vehicle force demand from dynamics | Potentially useful for low-frequency, quasi-steady maneuvers after calibration | Requires external-force accounting, appropriate mass/inertia, source-time derivatives and explicit allocation assumptions |
| Per-wheel physical force/capacity ratio or remaining grip | Not supported by present evidence | No exposed separate tire Fx/Fy, unresolved raw-force semantics, unknown validated combined-slip capacity |

There is currently no evidence-based accuracy percentage for a physical
utilization estimate. The 0.00948 raw-unit standard deviation of the static force
sum describes repeatability of one measurement window, not accuracy of a force
model or predictive grip margin. A visually precise gauge would overstate the
available evidence.

"Force fraction used", "distance from a peak-slip coordinate", and "remaining
stable grip margin" are different targets. Post-peak slip can increase while
force decreases. A lower reconstructed force fraction can therefore coexist
with a worse sliding state; it must not be displayed as recovered safety margin.
Likewise, a slip ratio of 0.5 is not necessarily 50% force or 50% remaining grip.

For a hypothetical validated anisotropic ellipse one might define
`u = sqrt((Fx/(mu_x*Fz))^2 + (Fy/(mu_y*Fz))^2)`, but neither those per-wheel
forces nor that capacity/combined-slip model is validated here. This formula is
not implemented or used as a benchmark metric. A single nonnegative scalar
cannot describe both force demand and post-peak stability without additional
regime information.

## Current physical discrepancy

The first stationary Porto recording has Rigidbody mass 3.47 kg, gravity
9.81 m/s², all four wheels grounded, near-zero body motion and no /drive
publisher. Raw force sum averages 30.8122 versus conditional mg of 34.0407 N,
a 9.484% difference. Front and rear raw sums are both low relative to the
conditional support predictions. No correction factor is fitted.

Possible explanations still include additional support, external/co-simulation
behavior, API force semantics, solver/phase behavior or native-read issues.
Current measurements do not select among them. The reported normal/tangent
vectors and flat contact points pass basic geometric sanity checks, but that
does not establish correctness of force interpretation.

The planner's 3.906 kg must not replace runtime Rigidbody mass, and wheel masses
must not simply be added. Runtime world CoM is approximately 0.06866 m above the
reported plane in this snapshot, whereas planning assumes 0.074 m. Their mass-
height products differ: 0.23824 versus 0.28904 kg·m. Those are different model
inputs, not an explanation for the measured raw-force discrepancy.

## New evidence from read-only inspection

Artifacts are in
`ws/log/wheel_contact_reports/investigation-20261116T055724.506936Z/`.

- The existing local control API at `host.docker.internal:4580` accepted read-only
  `status`, `tree Roboracer 2` and `bounds Chassis-1-solid1` commands. Status
  identifies Porto, Autonomous mode and a connected bridge.
- Metadata was read with System.Reflection.Metadata, without loading/executing
  the target assembly. `AntiRollBar` and `AutomobileController` contain anti-roll
  and downforce fields/methods respectively; these are separate types from the
  active `VehicleController`. Their presence in the assembly does **not** establish
  that RoboRacer uses either behavior. An earlier broad assembly-string search
  was insufficient for attributing those features to the active car.
- Active root components include Rigidbody, VehicleController, LapTimer and
  **CoSimManager**. The latter exposes Teleport, CoSimTimer and CoSimSmoothness;
  its native behavior is not reconstructed by the generated bindings. Presence
  alone is not proof that it changes this car's motion or support balance.
- The chassis `bounds` command reports render-mesh geometry in the root frame;
  the `tree` command exposes some children only as generic Component wrappers.
  Neither query establishes physical collider clearance or contact forces.

## Why dynamics alone cannot resolve every tire force

For illustration, suppose normal loads and all other forces were already known,
and tire tangent forces act at the recorded coplanar contact points. Form a
6-by-8 matrix whose columns are each wheel's two tangent unit directions followed
by the moments `r cross direction` about the measured CoM.

For the recorded geometry this matrix has **rank 3**, leaving a **5-dimensional
nullspace**. A nonzero redistribution of tire tangent forces can therefore leave
the total body force/moment unchanged. The saved numerical example has wrench
residuals below 2e-16 for arbitrary force-unit normalization. See
`tangent_force_identifiability.json` in the investigation artifact directory.

This is an inverse-problem example, not a claim that arbitrary nullspace forces
satisfy the actual engine's tire law. It demonstrates why exact acceleration,
yaw acceleration, inertia and Fz measurements would still need additional tire,
torque or allocation assumptions to recover per-wheel Fx/Fy. In nonplanar
settings the rank can differ; eight free tangent components still exceed six
body-wrench equations. A fully validated constitutive model could supply the
missing constraints, but it would be a model-derived estimate, not direct API
force measurement.

A bicycle model can estimate axle lateral demand from net lateral force and yaw
moment under suitable low-slip/steady assumptions. It does not independently
resolve inside/outside wheel forces or combined-slip margin. Wheel torque/RPM
could add constraints only after effective rotational inertia, brake behavior,
reaction torque and solver regularization have been established.

## Diagnostic source prepared for the next measurement

Plugin **0.3.1** adds optional read-only observations within existing
`Telemetry.WheelContact=true` mode:

- A collision observer attached to the vehicle Rigidbody object, recording raw
  OnCollisionEnter/Stay/Exit aggregate impulses, contact geometry/separation,
  collider IDs and callback fixed time/frame. Impulses are not converted into
  per-tire forces, and WheelCollider inclusion/exclusion is not assumed.
- A native collider inventory with type, trigger/enabled/body membership and
  world AABB bounds, plus body sleep state. Inventory is bounded at 64 colliders;
  events retain at most 8 other-collider entries with at most 16 contact points.
  Truncation and sampling errors remain explicit.
- Current controller drive/brake/steer configuration and explicit scalar getter
  observations of CoSimManager state. No getters outside that whitelist or
  arbitrary native methods are invoked.

These appear under `runtime_snapshot.collision_diagnostics`,
`controller_configuration` and `cosimulation_component`; the ROS schema remains
additive version 1. No forces, collider settings, mass, pose, time step or command
state are changed. Latest collision events may be stale or absent on sleeping
bodies; their time/phase/sleep state must be checked before interpretation.
A collision callback is an additional observed endpoint, not proof of the whole
physics acquisition phase. Full raw snapshots and validity tags remain intact.

Source/API compilation against installed bindings uses a temporary .NET 10
target override, zero warnings/errors for the source check. The source project
still targets .NET 6. A real .NET 6 build and runtime callback verification are
pending; no test DLL is a deployment artifact and no installed DLL was changed.

## Next decisive tests and achievable conclusion

1. Install the diagnostic build and repeat the stationary capture. Identify
   whether non-wheel collider contacts exist and whether their *fresh* vertical
   impulse/time-step contribution could explain the missing support. Check
   collider identity to avoid double counting and sign/phase before summing.
   If this does not explain the residual, inspect co-simulation and engine/read
   semantics rather than rescaling force to mg.
2. In an isolated diagnostic copy with an independent stop, verify wheel lift,
   support-only static balance, three independent repetitions, controlled mass
   and longitudinal/lateral CoM shifts. Change solver step/settings separately
   and compare telemetry-enabled/disabled overhead. These interventions have
   not been performed or silently enabled by the new diagnostic observer.
3. Only after load and time gates pass, use separate-axis, quasi-steady sweeps
   to identify actual force-response peaks. Derive aggregate/axle forces from
   properly accounted dynamics, then test load/speed changes and held-out trials.
4. Test combined slip and transients separately. Publish model residuals and
   validity regimes. Use an empirically validated envelope, not an assumed
   circle/ellipse or a clipped sum of the two slip-coordinate ratios.

If those tests succeed, a useful **calibrated axle/vehicle demand estimate** and
an empirical near-peak/sliding indicator are plausible. A credible per-wheel
force/capacity fraction needs stronger constitutive/torque information and
validation. No numerical error bound or instantaneous remaining-grip percentage
is supportable yet. The unresolved 9.484% support residual and phase/downsampling
issues are material limitations, not details to hide behind a utilization label.

## Deployment checkpoint required by the user

When further progress requires a plugin installation or Windows simulator restart:
finish all independent builds/offline checks; export the deployment DLL into the
shared repository with version and SHA-256; print `ACTION REQUIRED: INSTALL PLUGIN`
with exact installation steps; then pause the goal under the user's explicit
standing authorization. Do not poll/retry during this checkpoint or mark the
investigation complete. Wait for user confirmation, then verify the running
plugin version before resuming simulator experiments. A .NET 10 compile-check
DLL must never be presented as a .NET 6 deployment DLL.

## Follow-up: deployed 0.3.1 and three reset baselines

Version 0.3.1 was observed running after deployment, SHA256
`e0c43e84cfb5e0c5b6b1c3c07724ba66f5d8e9d43ea6da82bf1d5152beb1daf9`.
New diagnostics were present. Three separate stationary reset recordings were
then made without drive publishers or driving commands. Each sent one ROS reset
request, followed by the bridge's existing repeated reset frames; diagnostic
reset generation therefore advanced several times per user request. It is not
a count of independent reset commands.

| Trial directory under `ws/log/wheel_contact_reset_trials/` | Valid messages | Mean raw force sum, >10 seconds after request | Sample standard deviation |
|---|---:|---:|---:|
| `20261116T061659.712849Z` | 480 | 30.80272 | 0.00789 |
| `20261116T062351.596218Z` | 395 | 30.80297 | 0.00869 |
| `20261116T062820.917407Z` | 478 | 30.80969 | 0.00987 |

The second recording began after the reset had already happened, so it supports
post-reset repeatability but is weaker evidence for the immediate transient.
The support discrepancy persists across resets. An exploratory source-time
finite-difference comparison during settling leaves an approximately 3.24–3.25 N
residual between conditional `m*(g+ay)` and raw force sum. This is not a calibrated
force model: sampling phase, intervening states and differentiation error remain
unverified. Details and reproducing script are saved in
`ws/log/wheel_contact_reports/reset-analysis-20261116T062821.256859Z/`.

No collision events were recorded. There are 22 colliders in the native inventory;
active non-wheel collider AABBs stayed at least about 3.83 mm above the reported
floor plane. This does not exclude speculative/contact-offset behavior, and
missing callbacks still do not establish zero support.

Crucially, `body_is_sleeping` reported true in every observation, **including while
pose, velocity and wheel force changed during settling**. It cannot be used as
proof that WheelHit values are stale, or proof that the car is not being updated.
The generated IsSleeping wrapper uses a native Boolean icall delegate; metadata
inspection alone does not settle its runtime ABI correctness. An independent
runtime-invoke Boolean read is needed before interpreting that flag. CoSimManager
reported disabled/inactive, so it is not currently established as the source of
the discrepancy.

The new offline `static_support_observation` utility retains raw force and mg
comparison while accepting a candidate collision impulse/time-step projection
only for identified non-wheel contacts with matching observed step/body metadata.
It refuses stale, ambiguous, truncated, duplicate, sleeping/unknown and invalid
inputs rather than manufacturing zero support. Even a numerically closed
conditional comparison does not mark physical tire-load semantics accepted.

## Installation checkpoint: diagnostic 0.3.2

A genuine .NET 6 Release DLL was built using only explicitly authorized downloads
of Microsoft.NETCore.App.Ref and Microsoft.AspNetCore.App.Ref 6.0.36 into temporary
reference-pack directories. Restore sources were cleared afterwards; no runtime,
SDK or system package was installed. Build passed with zero warnings/errors.
Metadata-only verification confirms target `.NETCoreApp,Version=v6.0`, assembly
version 0.3.2.0, and System.Runtime/System.Text.Json references 6.0.0.0.

Offline benchmark suite: **108 passed**, three existing numerical-library
deprecation warnings. Browser and live ROS collection files were excluded;
those tests are not claimed by this run. This includes 18 conditional support
regressions and the existing contact decoder/probe/module/bag/report tests.

Export:
`ws/log/plugin_exports/wheel-contact-0.3.2-6c337d1124b2/TrackSelect.Windows.dll`

SHA256:
`6c337d1124b27e92be0d50a93f3ca90467fc0e033f4d7c46cac870df94dc75fd`

The export also contains installation instructions/script, metadata/build logs,
reference-binding hashes, source archive and manifest. The DLL was not executed
in this container and is not installed by the agent.

Version 0.3.2 adds an independent IL2CPP runtime-invoke sleeping Boolean alongside
the original icall result, explicit collider/floor/default contact offsets and
raw LateUpdate force/torque accumulators. Missing/unavailable native calls are
reported. Accumulators may have been cleared already and are not tire forces.
These measurements target the unresolved discrepancy; they do not enable a
physical utilization metric. The goal pauses at this checkpoint pending user
installation/restart confirmation. No polling or simulator tests resume until
confirmation, followed first by version/hash verification.

## Fixed-action runner deployment of 0.3.2

The user subsequently authorized a reviewed, fixed-action Windows runner for the
separate simulator copy `D:\Documents\f1tenth\autodrive_with_plugin_codex`.
Runner source/configuration are personal and Git-ignored; they were read and
preserved, not modified. No running service was inferred from source/queue files.
An explicit authorized launch request returned a successful live response, then
rest was confirmed before installation: no /drive publisher or controller/
benchmark node, 86 recent zero-speed odometry observations, zero actual throttle,
near-zero body speed from the simulator's diagnostic, and no gamepad.

The runner completed installation request
`7f0ad22ece5e4467ac2d878ea1801b5a`, returning the expected SHA256
`6c337d1124b27e92be0d50a93f3ca90467fc0e033f4d7c46cac870df94dc75fd`.
Its backup is in the trusted Windows state directory under that request ID,
`artifacts\7f0ad22ece5e4467ac2d878ea1801b5a\previous-TrackSelect.Windows.dll`.
The subsequent launch request `b40eceb921eb49638ef80b7b01a6056b` completed with
fixed arguments `-track Porto -autoconnect`. No request timed out or was replayed.

Live contact telemetry independently reported version
`0.3.2+f890810a22381d5335ea8cd441777a5ba294df8a` and exactly the installed expected
hash. Twenty fresh messages were received on each of scan, IMU, odometry,
contact and actual throttle. The simulator control status confirmed connection,
Porto and Autonomous mode. There was still no /drive publisher and no localized
pose publisher: sensor streams were fresh, but race localization readiness was
not established and no motion was enabled. Personal client protocol tests passed
all six checks in temporary files; these were not real install/launch tests.

Deployment response and readiness evidence:
`ws/log/windows_runner_deployments/7f0ad22ece5e4467ac2d878ea1801b5a/`.
The trusted runner response reports its script SHA256 as
`2ad5ab2e5aa6234b386f11bf8605e931e0b3397e67328b8f4ac1184c3eca80d7`.
Queue responses are operational evidence, not tamper-resistant audit records.

The new diagnostic values narrow the next work:

- The independent sleeping read returned **unavailable: IsSleeping method not
  found**. It did not silently substitute a value or prove the original Boolean
  correct. The icall return remains true; native method exposure/Boolean ABI is
  still unresolved.
- LateUpdate force/torque accumulators returned zero vectors. This is not proof
  of absent applied forces: buffers may already have been cleared.
- Default contact offset is 0.01 m. This strengthens the warning that positive
  AABB clearances of about 0.0038 m do not, by themselves, exclude contact-offset
  behavior. It does not prove an extra supporting force.
- No collision events were recorded in the readiness observations. Callback
  behavior and support/force semantics remain unvalidated.

The current development container has no `/opt/autodrive-bepinex` reference mount.
The exported 0.3.2 DLL remains a verified implementation artifact; future builds
must restore the documented read-only reference mount rather than substitute
unverified references or a reference-assembly DLL. Official dependency downloads
are now explicitly authorized by the user, superseding the audit-only restriction.

## Restored references and independent native-Boolean check (0.3.3)

The user restored the read-only reference mount. Metadata-only inspection of
that installed Rigidbody binding's static initializer gives the exact icall key
`UnityEngine.Rigidbody::IsSleeping`. The 0.3.2 attempt to find a normal IL2CPP
method was unavailable; it is not treated as a successful measurement.

Prepared 0.3.3 retains both existing readings and adds `sleep_state_icall_byte`,
resolving that exact internal call with a Cdecl delegate whose return is a byte.
It rejects noncanonical byte values above 1 and reports resolution errors.
It neither wakes nor sleeps the body. The purpose is comparison with the
existing generated Boolean-return delegate, not assuming its result beforehand.

A controlled x86-64 fixture returned a one-byte zero with deliberately nonzero
upper return-register bits. The default .NET Boolean delegate reported true;
the byte-return delegate reported zero. This demonstrates a real marshaling
failure mode but does not prove that the Unity function exhibits it. Only the
running side-by-side observation can answer that.

0.3.3 built as genuine net6.0 Release with zero warnings/errors. Target metadata
and framework references were verified without loading the plugin. The focused
contact/probe/support suite passed 35 tests. Exported implementation DLL:
`ws/log/plugin_exports/wheel-contact-0.3.3-d0bedc999b63/TrackSelect.Windows.dll`.
SHA256: `d0bedc999b63f20a5dd7bfae223fb8d2e075a4739731180d442b6132a0d1b692`.

## Relative response can cancel a load factor, conditionally

Unity's [2022.3 curve API](https://docs.unity3d.com/2022.3/Documentation/ScriptReference/WheelFrictionCurve.html)
describes two spline sections, configured knots and zero tangents at the peak
and asymptote. This does not, by itself, establish the installed interpolation
or the relationship between an axis response and combined grip margin.
The [manual](https://docs.unity3d.com/2022.3/Documentation/Manual/wheel-colliders-friction.html)
also describes the reduction in curve response beyond the configured peak.

An important conditional result is that for `F=A(load,state)*c(slip)`, with the
same positive factor A at the reference peak, `F/F_peak=c/c_peak`. Uncertainty
in absolute load can therefore cancel in a *relative, same-load axis response*.
The static force discrepancy does not logically invalidate every such estimate.
Separable normalization, actual interpolation, axis coupling and transient/low-
speed behavior still require validation; none is assumed by a shipped metric.

An illustrative first-segment cubic family is
`f(u;m)=-2u^3+3u^2+m*(u^3-2u^2+u)`, for u in [0,1], m in [0,3]. All members
start at zero, reach one at u=1 with zero tangent, and are monotone nonnegative.
At u=0.5 their responses range from 0.5 to 0.875. This is a mathematical example,
not an asserted family of actual Unity implementations or an accuracy bound.
It shows why a half-peak slip coordinate cannot simply be labeled 50% force.
Configured asymptote/peak value ratios are 0.64444 forward and 0.5 sideways;
the corresponding slip-coordinate ratios are 1.66667 and 10. Larger slip need
not mean greater force fraction or greater remaining stable margin.

The arithmetic/cancellation checks and example data are preserved under
`ws/log/wheel_contact_reports/normalized-response-20261116T155139.584599Z/`.
No curve-reconstruction or physical-utilization metric was added to control or
benchmark outputs from this argument.

## Live ABI comparison: the original sleeping getter is unreliable

The runner installed 0.3.3 through request `8734b7c838b145618899214d615f2969`,
then launched it through `493f3c5c959d4c2781f22f9bff327ea5`. Rest and absence of
controllers/drive publishers were checked before installation. Live telemetry
matched the exported implementation hash
`d0bedc999b63f20a5dd7bfae223fb8d2e075a4739731180d442b6132a0d1b692`.
Scan, IMU, odometry and contact streams were fresh. No motion or reset commands
were sent for this comparison.

All **100** accepted observations reported generated Boolean `true` and explicit
native byte `0`/Boolean `false`. The earlier always-true result must not be used
as evidence that the body is asleep or that wheel force is stale. This is a
specific native-return marshaling discrepancy, supported both by the controlled
ABI fixture and by repeated live side-by-side reads. It does not imply that all
other getters are wrong, or validate the remaining force semantics.

Evidence is saved in
`ws/log/wheel_contact_sleep_abi_checks/20261116T161032.122597Z/`.
Raw force still averages 30.81059, mass remains 3.47 kg, floor contact offsets
are 0.01 m, and no collision events or observer errors were recorded in those
observations. Correcting the sleep interpretation therefore does **not** resolve
the static support residual or establish absence of other support.

The offline conditional-support utility now prefers a consistent explicit native
sleep byte when present, retains the raw getter/disagreement, and refuses
noncanonical/inconsistent byte payloads. It still never marks physical tire-load
interpretation accepted. Older captures are not rewritten or retroactively given
an invented corrected sleep state.

Primary-source cross-checks also support keeping the interpretations separate:

- [Unity 2022.3 WheelHit.force](https://docs.unity3d.com/2022.3/Documentation/ScriptReference/WheelHit-force.html)
  describes a scalar contact-force magnitude; it does not supply separate tire
  Fx/Fy or an explicit identity with Fz.
- [Unity collision impulse](https://docs.unity3d.com/2022.3/Documentation/ScriptReference/Collision-impulse.html)
  is an aggregate over a collision pair's contact points. Dividing it by the
  corresponding fixed step is a force comparison, not automatically per-tire
  force, and freshness/identity/sign still matter.
- [PhysX 3.4 vehicle source](https://github.com/NVIDIAGameWorks/PhysX-3.4/blob/master/PhysX_3.4/Source/PhysXVehicle/src/PxVehicleUpdate.cpp)
  distinguishes suspension force, normalized/filtered tire load and tangent tire
  force computation. This is useful architectural evidence, not proof of the
  installed Unity engine's exact implementation or load filter.

The next discriminating physical checks remain controlled mass/CoM and solver-
step comparisons, plus verified support/force acquisition. No fitted static
correction or percentage-of-grip claim is introduced by fixing the Boolean.

## Controlled stationary mass sweep: the residual is approximately additive

The designated diagnostic copy ran 0.3.4, implementation SHA256
`7b47f1d5168e5c72f53214c30f23d153fc9b43a46cbe75fbbffdb85acc6f1760`.
The genuine net6.0 Release build and metadata checks passed with zero build
warnings/errors. Managed policy/lifecycle fixtures passed 49 checks, and 35
focused Python contact/support tests passed. The fixed Windows runner completed
installation `899d34ff62ea41efba62a53d07c7a7ec` and launch
`78e62afedd984fddad8d6faea6d2253a` without timeouts/replay. Rest and no competing
controller/drive publisher were verified first; expected plugin hash and fresh
scan/IMU/odometry/contact streams were verified before any parameter intervention.

The explicit trial API is restricted to the designated diagnostic-copy folder,
requires zero torque, rest and grounded hits through the existing native reading
path, and supports fixed ±10% mass cases only. It does not publish drive commands.
An original-value snapshot, explicit restore, reset restore, actuation/motion
restore and 12-second main-thread deadline bound each intervention. Managed tests
exercise these paths, including setter failures. The main-thread deadline is not
an independent real-time safety system, and does not change the plan's requirement
for independent stopping before moving experiments.

Six alternating cases (three per level) observed the actual target mass, its
automatic restoration to **3.47 kg**, and return to rest. The final explicit
restore/readback confirmed no active intervention and mass 3.47 kg. No driving
or reset commands were sent for this sweep. Raw data and intervention/cleanup
records are under `ws/log/wheel_contact_mass_trials/20261116T162439.726483Z/`.

Settled comparison windows use unique, grounded/valid observations 4–10 local
receipt seconds after command acknowledgment; all transients remain in raw files.
This is not an assumed Windows/container clock alignment or solved-step proof.

| Applied mass | Repetition | Settled samples | Raw force sum mean | Conditional mg | Missing amount |
|---|---:|---:|---:|---:|---:|
| 3.123 kg | 1 | 113 | 27.39677 | 30.63663 N | 3.23986 N |
| 3.123 kg | 2 | 118 | 27.40339 | 30.63663 N | 3.23324 N |
| 3.123 kg | 3 | 112 | 27.40408 | 30.63663 N | 3.23255 N |
| 3.8170002 kg | 1 | 113 | 34.23710 | 37.44477 N | 3.20768 N |
| 3.8170002 kg | 2 | 121 | 34.23907 | 37.44477 N | 3.20570 N |
| 3.8170002 kg | 3 | 126 | 34.23778 | 37.44477 N | 3.20699 N |

SprungMass sums followed actual body mass at both levels. No collision events
were recorded in the selected windows. An exploratory two-level regression of
raw sum against body mass has slope **9.85096 raw-force units/kg**, versus gravity
9.81 m/s², and intercept -3.36313 raw-force units. The near-weight slope is useful
evidence that changes in this raw signal track changes in static support demand.
Two distinct levels are not validation of a global affine law, dynamic accuracy,
per-wheel normal force, or an additive correction to publish as physical load.

The deficit is about 10.56% at low mass and 8.56% at high mass, but approximately
3.2 N at both. A universal 9.5% rescaling inferred from the original baseline is
therefore not supported. The mechanism behind the offset is not yet identified.
In particular, this result does not authorize adding WheelCollider masses to the
body mass or assigning unexplained residual force to a collider.

Analysis is preserved at
`ws/log/wheel_contact_reports/mass-sweep-20261116T162439.726483Z/analysis.json`.
The next discriminating interventions are wheel-mass accounting and CoM shifts,
followed by solver-step/phase sensitivity. Physical tire utilization and moving
friction sweeps remain unvalidated.

## Controlled stationary wheel-mass sweep: no corresponding force offset

Version 0.3.5, net6.0 implementation SHA256
`ab2e4a8179e5148f85bc15fa13fab90c075f1f7550bb0dacc5a0bd65e5754f22`, passed
86 managed policy/lifecycle checks before runner installation
`993367c2149e4f2893f62bb6ca632ed3` and launch
`c182d35393b44e8891c263c748be412c`. Expected running hash, fresh ROS sensor/contact
streams, zero drive publishers and rest were verified. Six alternating cases
changed all four WheelCollider masses to 80%/120% of their original 0.109 kg,
while Rigidbody mass stayed 3.47 kg. Actual target values and automatic restoration
were observed in all six cases. Final readback confirmed all four masses at
0.109 kg, body mass 3.47 kg and no active intervention. No drive/reset commands
were sent.

| Per-wheel mass | Repetition | Settled samples | Raw force sum mean | Conditional mg minus raw |
|---|---:|---:|---:|---:|
| 0.0872 kg | 1 | 118 | 30.81074 | 3.22996 N |
| 0.13080001 kg | 1 | 132 | 30.80977 | 3.23093 N |
| 0.0872 kg | 2 | 105 | 30.80743 | 3.23327 N |
| 0.13080001 kg | 2 | 96 | 30.80753 | 3.23317 N |
| 0.0872 kg | 3 | 125 | 30.80445 | 3.23625 N |
| 0.13080001 kg | 3 | 121 | 30.80537 | 3.23533 N |

Selection again uses unique grounded/valid observations 4–10 receipt seconds
after acknowledgment, retaining all raw transients. Paired high-minus-low means
are -0.000969, +0.000105 and +0.000914 raw-force units. This does not support a
simple static residual proportional to WheelCollider mass: for example, the
coincidental baseline match to three wheel masses times gravity would predict
a 1.28315 N change across these levels, which was not observed. The experiment
does not establish dynamic wheel-inertia behavior or prove the missing support
mechanism. No wheel-mass correction is introduced.

Raw data: `ws/log/wheel_contact_wheel_mass_trials/20261116T163622.230501Z/`.
Reproducible analysis and input hashes:
`ws/log/wheel_contact_reports/wheel-mass-sweep-20261116T163622.230501Z/`.

## Timestep intervention: the static raw signal is strongly step-sensitive

Version **0.3.6** adds two closed, stationary-only timestep cases to the existing
bounded trial lifecycle. Managed fixtures passed **120 checks**; focused Python
decoder/contact/support/probe tests passed **61 tests**. A genuine net6.0 Release
implementation build passed with zero warnings/errors and target/version metadata
checks. Export: `ws/log/plugin_exports/wheel-contact-0.3.6-588e57cebdb9/`.
Implementation SHA256:
`588e57cebdb9318d6ed7e5bd2e1cc452dd1f158ac1a7012ac86081fadcb1d442`.

The fixed runner completed install `2a8d387ec49147b98a0a433f728c0602` and launch
`d7416e935ca54669a92ac903aecc6836`. Rest/no controllers were verified before
installation. Fresh scan/IMU/odometry/contact streams and the expected running
hash were verified before intervention. Body mass stayed 3.47 kg and all four
wheel masses stayed 0.109 kg. The global fixed step is restored even if the
trial's vehicle is destroyed; as with other cases, main-thread blockage can
still delay its callback-based restore.

**Failed case retained:** the first 0.5 ms case aborted after about 3.7 receipt
seconds because a required ROS stream exceeded the unchanged 0.6-second freshness
limit. It did not reach the planned 4–10-second window. Explicit restore and
readback confirmed 1 ms, original masses and inactive state. Its 79 raw messages
show a transient increase toward roughly 32.4, but are not a passed settled
comparison. The capture did not record which stream triggered the failure;
contact gaps in the retained pre-abort messages were at most 0.126 seconds.
Neither a specific overload mechanism nor a permanent connection failure is
established. The case was not replayed or counted as one of three repetitions.
Raw failed run: `ws/log/wheel_contact_step_trials/20261116T164224.648759Z/`.

After restoration, a read-only 12-second check found all required streams fresh,
with maximum interarrival gaps around 0.11 seconds, no drive publisher, and the
expected baseline step/rest state. A distinct **2 ms** comparison then completed
three trials without relaxing freshness limits. Each observed the actual target,
automatic restoration to 1 ms, original masses and return to rest; final explicit
restore/readback confirmed no active intervention. No drive/reset commands were
sent in either timestep experiment.

| Step | Case | Valid observations | Raw force sum mean | Conditional mg minus raw |
|---|---|---:|---:|---:|
| 1 ms | Initial 5-second baseline | 84 | 30.81064 | 3.23006 N |
| 2 ms | Repetition 1, 4–10 s | 97 | 27.43028 | 6.61042 N |
| 2 ms | Repetition 2, 4–10 s | 100 | 27.43188 | 6.60882 N |
| 2 ms | Repetition 3, 4–10 s | 94 | 27.43314 | 6.60756 N |

The 2 ms cases' maximum observed body speeds in those windows were below
0.000030 m/s. World CoM height changed by about 0.000006 m relative to baseline.
No collision events were recorded in these selected observations; missing events
still do not prove no other support. The baseline and trial windows have different
selection durations and remain receipt-time windows, not verified solved-step
sampling. All transients and source identity are retained.

This establishes a large, reproducible **timestep sensitivity of the observed
stationary scalar**, under the tested settings. The conditional weight gap changes
from about 9.49% to 19.41% despite unchanged body weight. It is inconsistent with
calibrating all configurations by one 9.5% scale or a fixed 3.2 N additive offset.
The residual is roughly twice as large at twice the step, but these two completed
levels do not validate linear extrapolation to zero step or a correction law.
Possible engine/solver/read-phase/support mechanisms remain unresolved. This
experiment identifies a material dependency, not the exact native implementation
or a proof that the scalar equals the force actually applied to the body.

Raw successful trials and post-trial readiness:
`ws/log/wheel_contact_step_high_trials/20261116T164406.603795Z/`.
Reproducible analysis, input hashes and standalone comparison figure:
`ws/log/wheel_contact_reports/step-high-sweep-20261116T164406.603795Z/`.

## Investigation conclusion and remaining validation boundary

The current implementation supports **per-wheel contact/slip diagnostics and a
configured peak-slip coordinate**, with validity, build identity and sampling
limitations retained. Report the forward and sideways ratios separately, retain
signed slip and contact/regime context, and label them as slip coordinates. They
are useful for comparing repeatable sampled behavior at matched settings. They
are not a measured percentage of force capacity or remaining grip. No numerical
physical-utilization error bound can be assigned from these tests.

The experiments materially narrow the static discrepancy: the sleeping Boolean
was an ABI-marshaling artifact; raw force changes approximately with added body
weight; changing WheelCollider masses did not explain the static offset; and the
remaining weight discrepancy strongly depends on timestep. Those are distinct
findings. The exact support/force/solver mechanism is still unidentified, so no
static fit, inferred Fz, saturation detector or grip feedback is published.

A calibrated low-frequency **axle/vehicle demand estimate** remains a plausible
future target after source-phase/time, load/external-force and model validation.
A credible individual-tire combined-utilization fraction additionally needs
constraints beyond the body wrench and a validated constitutive/capacity model.
The inverse-problem nullspace and unestablished curve/coupling laws do not disappear
by correcting one force offset. Relative response normalization could cancel a
common load factor only under the stated separability assumptions; those
assumptions have not been verified in this simulator.

Wheel lift, verified non-wheel support accounting, longitudinal/lateral CoM
shifts, complete solved-step tracing, enabled/disabled overhead comparison,
separate-axis moving force-response sweeps, combined slip, transients and held-out
accuracy tests remain **unperformed/unvalidated**. They are requirements for
claiming the stronger physical estimates, not successes inferred from managed
checks. In particular, the static/phase acceptance gate did not pass, so moving
friction sweeps were not enabled. Benchmark command ownership and the inhibited
gate remain unchanged. The evidence supports the bounded conclusion above;
it does not establish physical tire-utilization validation or complete all
experiments in the original plan.
