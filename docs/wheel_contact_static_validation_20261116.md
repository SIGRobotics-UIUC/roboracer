# First live stationary contact check — Porto, 2026-11-16

A read-only 30-second capture completed after ROS discovery became available
between the development session and the live bridge on domain 0. No driving,
reset, track-switch, settings or simulator-launch commands were sent. The graph
contained fast_bridge, adapter, wheel_odom and Foxglove, with no /drive publisher.
No drive or low-level command messages were observed during the recording.

Raw capture: `ws/log/wheel_contact_checks/20261116T054118.743665Z/`.
Analysis: `ws/log/wheel_contact_reports/20261116T054118.743665Z/`, including
`report.html`, `metrics.json`, `static_diagnostics.json`, the reproducible
`analyze_static.py` and module `contacts.jsonl`. Raw capture files were not
rewritten. These are diagnostic artifacts, not a managed/sealed race collection.

## Observed telemetry

- 840 messages; all envelopes valid; all four wheels grounded and marked valid.
  All runtime snapshots valid. 840 unique source samples, no repeated source
  observations, no sequence regressions, one session and reset generation 2.
- Contact normals and tangent directions have unit length within reported
  floating precision and are orthogonal. All 3,360 contact points have world
  y=0, on collider `Floor`. This is a useful ABI sanity observation, not proof
  that every native accessor or sampling phase is correct.
- Runtime Rigidbody mass is **3.47 kg**, gravity 9.81 m/s²; do not substitute the
  planner's 3.906 kg or add WheelCollider masses without mass-accounting evidence.
  Wheel radius is 0.059 m. Runtime friction curves match the serialized defaults
  listed in the investigation plan. Configured sprungMass values sum to 3.47 kg.
- Motor torque, steering angle, RPM and both raw slips remain zero. Brake torque
  is **428 N·m API value per wheel** throughout, so the baseline includes the
  simulator's applied brake behavior.
- Reported body speed averages 0.00002894 m/s and peaks at 0.00006924 m/s.
  Maximum body displacement from the first observation is 0.000720 m. The car is
  nearly stationary, with small residual drift rather than mathematically fixed.
- Plugin reports Unity 2022.3.52f1, version
  `0.3.0+f890810a22381d5335ea8cd441777a5ba294df8a`, plugin SHA256
  `5d66e273dcf5b7c34cad6f4e7a50c5634ebf2e1855f53d066c43c9934c24537b`.
  Runtime payload includes native game/binding/level0 hashes. These identify
  reported binaries; no reproducible source-to-binary comparison was performed.

## Conditional static comparison

These predictions assume a rigid body on a flat plane with wheel-only support,
no additional forces and negligible acceleration. Non-wheel support/forces have
**not** been ruled out. Wheel contact force is not renamed as vertical load.

| Quantity | Raw force mean | Conditional support prediction |
|---|---:|---:|
| Four-wheel sum | 30.8122 | mg = 34.0407 N |
| Front axle sum | 14.6302 | 15.9723 N |
| Rear axle sum | 16.1821 | 18.0684 N |

The raw sum is **9.484% below mg**. Its sample standard deviation is 0.00948 raw
force units, so the discrepancy is far larger than variation within this single
recording. Ten-second contiguous windows have means 30.81159, 30.81251 and
30.81266. They show short-term repeatability, not three independent trials or
solver sensitivity, and no physical acceptance tolerance has been established.

Predicted axle support uses runtime world CoM and the horizontal projection of
wheel-center axle geometry; projected wheelbase is approximately 0.330 m.
Per-wheel sprungMass*g predictions are 8.01668, 8.00514, 9.01532 and 9.00356 N;
raw means are 7.32093, 7.30923, 8.09696 and 8.08511, respectively.
The mismatch needs investigation of supports/extra forces, applied-brake behavior,
force semantics and actual sampling phase. It is not corrected with a scale
factor or attributed to a particular cause without evidence.

## Timing and configuration findings

The observed fixedDeltaTime is **0.001 s**, not the usual assumed 0.02 s. Mean
bridge/collector arrival interval is about 0.0358 s (approximately 28 Hz). Mean
advance of observed Unity fixed time between transported samples is 0.02186 s.
There are 853 unobserved source sequence IDs within the recording's source span.
This is evidence of transport/downsampling coverage, not proof of network packet
loss. Many physics states are not represented; LateUpdate and solved-state
identity flags remain false/unverified. No clock alignment or latency is inferred.

There are **840 configuration revisions for 840 samples**. Inspection shows tiny
variations in inertia, sprungMass, transform-derived wheel-center coordinates
and lossy scale (typically 1e-7 to 1e-6). Exact serialized comparison treats these
numerical variations as new revisions. Reports must not interpret this count as
840 deliberate settings changes. Raw snapshots are preserved; a future revision
identity needs an explicit comparison policy and regression tests, without
rounding away raw observations or silently ignoring meaningful small changes.

## Acceptance status and next steps

Live transport, strict envelope decoding, raw snapshot export and basic contact
vector sanity have been exercised. A user-installed plugin runs successfully;
this supersedes the earlier lack of live evidence, not the stated compile-check
limitations or source/binary reproducibility uncertainty.

Physical load/force interpretation is **not accepted**. Before driving sweeps:

1. Explain the static support mismatch, including checking chassis/other ground
   contacts and external-force behavior in the diagnostic simulator scene.
2. Establish actual acquisition phase/step evidence at the installed 1 ms step,
   and compare solver-step sensitivity and disabled/enabled telemetry overhead.
3. Improve configuration-revision identity while preserving full raw snapshots.
4. Perform wheel-lift/null-field, reset/scene/track and isolated mass/CoM checks;
   establish independent repeatability/tolerances before dynamic experiments.

No wheel lift, reset, mass/CoM change, acceleration/braking, circles, friction
sweep, collisions or racing laps were tested. There is no new physical tire
utilization, saturation flag or grip feedback.

The follow-up [tire-utilization assessment](tire_utilization_assessment.md)
records the inverse-problem limit, local control/metadata inspection, and prepared
0.3.1 collision diagnostics for resolving the support residual. That diagnostic
build is not yet installed or runtime-validated in this record.
