# V1 implementation validation

Validated in the existing ROS 2 Humble container. Synthetic test data was
written to temporary directories; actual simulator artifacts are under `ws/log/`.
Validated tracks and existing run artifacts were not modified. Runtime code
neither imports nor invokes `ws/scripts/`.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select perception benchmark
source install/setup.bash
```

Both packages built successfully. The existing container was not recreated and
the Docker image was not rebuilt. The Dockerfile now declares the bag reader,
SQLite3 and MCAP packages; Compose adds a read-only `/repository` mount for Git
snapshots. Apply those changes before real collection when these capabilities
are needed. Without a readable Git checkout, source snapshot availability is
explicitly reported rather than invented.

## Framework checks

From `/root/ws`, the final full suite passed **32 tests with one skip**:

```bash
ROS_DOMAIN_ID=68 python3 -m pytest src/benchmark/test -q \
  --basetemp=/tmp/benchmark-v1-accepted
```

The skip is the **MCAP native-bag fixture**: the existing container exposes
SQLite3 but does not have the MCAP storage plugin. MCAP writing/reading is not
claimed validated. Three deprecation warnings originate in installed numerical
dependencies used by the planning map loader; they are not test failures.

Coverage includes strict configuration/typed overrides, managed launch mappings,
exclusive IDs, hashes/read-only sealing, explicit non-overwriting recovery,
active-writer recovery locks, process-group cleanup, monotonic latching stop
policies, freshness/rest confirmation, deterministic numeric metrics, duplicate
and out-of-order stamps, storage sorting across clock resets, frame checks,
angle interpolation, unavailable timing evidence, malformed-module/exception
isolation, generic static figures, and coexisting analyses.
Monotonic receipt-time boundaries prevent startup/stopping data from leaking into
measurement when recording timestamps freeze. Completion inhibits commands as
soon as the target lap count is observed; lap-time association finishes while
the vehicle is stopping.

Synthetic ROS integration verifies collection completion, stop requests,
logging-off lifecycle, transient pose loss versus persistent-loss abort,
recorder exit, disk reserve failure, stop confirmation and bag-count integrity.
The managed-startup case runs the **real pure pursuit node**, with only the
localizer process stubbed and synthetic poses supplied by the fixture. It does
not establish actual Cartographer/MCL convergence. Preflight rejection also
checks that an existing stack receives no benchmark enable/inhibit commands.
The frozen-clock fixture verifies gate silence selects the existing bridge's
monotonic command timeout, with rest explicitly unconfirmed because fresh
odometry cannot be established. This fixture models that timeout, not physics.

The offline CLI generated another report on unused domain 69 without creating a
ROS node:

```bash
ROS_DOMAIN_ID=69 ros2 run benchmark benchmark \
  --log-root /tmp/benchmark-v1-accepted/test_complete_collection_offli0/log \
  analyze complete --analysis-id cli-final --modules all
```

`ros2 launch perception mcl.launch.py --show-args` confirms the existing MCL
seed is exposed with unchanged default `-1` (nondeterministic). A managed
`components.localization.params.seed` explicitly configures that argument;
`metadata.seeds` alone remains an operator declaration.

## Existing-stack regressions

```bash
ROS_DOMAIN_ID=42 python3 -m pytest \
  src/control/test/test_input_health.py src/control/test/test_map_ros.py -q
# 8 passed

ROS_DOMAIN_ID=70 python3 -m pytest \
  src/perception/test/test_mcl.py src/perception/test/test_mcl_node.py -q
# 87 passed

ROS_DOMAIN_ID=97 python3 -m pytest \
  src/autodrive_bridge/test/test_fast_bridge.py \
  src/autodrive_bridge/test/test_steady_time.py -q
# 19 passed
```

The MAP callback/stop tests choose domain 42 themselves; the MCL tests use the
specified domain 70. No default-domain driving tests were run.
Bridge fixtures force domain 97. These existing-stack checks total 114 passes.

## Remaining external validation

An actual Porto experiment was performed as described below. No injected
pause/disconnect test or independent physical stopping measurement was performed.
Stop confirmation means fresh wheel odometry at rest plus gate inhibition.
Validate the provisional thresholds on each supported stack.

The current `/clock` is the existing host-monotonic transport clock, not physics
simulation time. There is no continuous native physics-time loss measurement,
cross-Windows/container clock synchronization, callback/solver instrumentation,
or command application endpoint correlation in v1. Reports carry explicit
unavailable metrics and the evidence needed for v2. Control smoothness, tire
dynamics, online-planner adapters and MPC/MPCC adapters remain outside this v1.

## Actual AutoDRIVE Porto test (2026-11-15 UTC)

The user started the normal bridge and selected Porto. The test used the saved
`Porto` track, MAP, Cartographer, speed scale 0.3, maximum command speed 2 m/s,
one warm-up lap and two requested measured laps. Basic SQLite3 capture required
ground truth for evaluation. The existing localization gate's ground-truth
dependencies remain unchanged; this was not a ground-truth-independent stack.

First attempt `porto-map-20261115T015408Z` stayed inhibited and aborted at
readiness. Native `last_lap_time` is positive infinity before a completed lap;
requiring a finite duration before motion created a startup deadlock. The fix
recognizes this specific sentinel as stream presence while treating its duration
as unavailable. Other nonfinite commands remain invalid. Both raw runs remain
sealed, including this readiness failure.

Retry `porto-map-20261115T015408Z-retry1` enabled motion and completed one native
warm-up lap in **31.8052 s**. About two seconds into measurement, the bridge
reported **`collision count 0 -> 1`** and emitted `sim_reset`. The manager aborted
with `unexpected_sim_reset`, inhibited commands and confirmed wheel-odometry rest.
**Zero of two measured laps completed; one collision occurred.** This is a real
collision-aborted validation, not a successful race or evidence of collision-free
control. The collision's underlying control/localization cause was not established.

Bag message counts match recorder acknowledgements. Only the user's bridge,
adapter, wheel odometry and Foxglove nodes remained; `/drive` had zero publishers
after cleanup. The bridge was left running. No map/raceline was regenerated.

The first offline report exposed another bug: the signal map combined parent
`world` and child `ground_truth` into a fictitious header frame. These contracts
are now separate and checked independently (signal-map version 2). Raw telemetry
was not changed; corrected analysis was written under a new ID:

- Raw: `ws/log/runs/porto-map-20261115T015408Z-retry1/`
- Corrected HTML/JSON: `ws/log/reports/porto-map-20261115T015408Z-retry1/actual-run-frames-fixed/`
- Config/source checkouts/console: `ws/log/benchmark_sessions/porto-map-20261115T015408Z/`

Collection metadata preserves commit `2ce99c9` and the local sentinel-fix diff.
The corrected report separately preserves the analysis diff. A Git bundle and
local checkout supplied provenance without restarting/recreating the user's
existing bridge container. The roughly two-second measured interval is too short
for full-lap algorithm comparisons; unsupported timing metrics remain unavailable.

After both fixes:

```bash
ROS_DOMAIN_ID=68 python3 -m pytest src/benchmark/test -q \
  --basetemp=/tmp/benchmark-actual-fixes-validation
# 34 passed, 1 skipped (MCAP plugin unavailable)
```

## Metadata and native lap-time validation audit (2026-11-15)

Inspected the immutable run `benchmark_report_v1.1-20261115T052144.462167Z`
and its reports (including `v1.4`). Its recorded Git metadata contains commit
`6b8438a57f21ece6403c4ba895935885d784e301` and `dirty: true`, but no branch.
It predates branch capture. Analysis does not infer its branch from today's
checkout. New snapshots record branch, branch state (attached/detached/
unavailable), commit and dirty state; old missing branch fields are marked
unrecorded. Available Git identity survives a separate source-snapshot failure.

The native reset command contains only `V1 Reset: True`, not a pose. The track
selector's `TrackSelector.Select` uses scene `Spawn 1`; `Switch` updates the
ResetManager's saved position/rotation. The selected Porto scene transform is
not present in this run's configuration or saved simulator metadata. The
Cartographer localization gate's map-frame identity initializer is not a
simulator spawn transform. Initial-pose metadata therefore records no explicit
request and an unresolved default, with no fabricated numerical pose or use of
first-observed/ground-truth pose. Previous collections are normalized in memory;
raw metadata is preserved.

Direct MCAP inspection of all 2,394 headerless `std_msgs/Float32`
`/autodrive/roboracer_1/last_lap_time` messages found:

- 615 positive infinities, all while the native lap counter was zero after reset.
- 1,779 finite positive durations; no NaNs, negative infinities or nonpositive
  durations. Frame rejections: zero; timestamp rejections: zero.
- Representative sentinel: recording timestamp `1794720107433021190`,
  CDR bytes `000100000000807f` (`+inf`), latest native counter `0`.
- The previous session's finite duration (`13.62090015411377 s`, counter `6`)
  precedes reset. Infinity starts after counter reset at `1794720107433021190`;
  finite duration resumes with counter `1` at `1794720137986133317`.

`fast_bridge` passes through the simulator's `V1 Last Lap Time` unchanged.
Collection already explicitly recognized positive infinity as stream presence
before a completed lap. Offline decoding instead applied a generic finite-value
check and mislabeled all 615 normal uninitialized values as invalid.

Offline validation now retains these samples with `availability: uninitialized`
when corroborated by counter zero; they are not numerical durations. Unexplained
infinity, NaN and nonpositive durations remain numeric rejections. Per-signal
validation counts, independent reason counts (frame/timestamp/numeric/payload),
and indexed examples are stored in analysis `signal_validation`. No lap times
are interpolated or rewritten. If counter evidence is absent, infinity is not
silently accepted as normal.

A new analysis was generated under
`ws/benchmark_runs/reports/benchmark_report_v1.1-20261115T052144.462167Z/metadata-validation-20261115/`.
It reports 615 uninitialized, 1,779 available, zero invalid lap-time samples and
no warnings. Race Performance remains partial because some optional metrics
lack evidence; that is separate from recording validity. Existing raw runs and
reports were preserved. Native MCAP and SQLite fixtures were tested using
`/usr/bin/python3` and the sourced ROS `PYTHONPATH` (not an overriding bare
`PYTHONPATH=src/benchmark`, which would skip ROS fixtures).

Final focused regression command (inside the sourced Humble container):

```bash
PYTHONPATH=src/benchmark:src/planning:$PYTHONPATH /usr/bin/python3 -m pytest \
  src/benchmark/test/test_metadata_validation.py \
  src/benchmark/test/test_bag_loading.py \
  src/benchmark/test/test_config_artifacts.py \
  src/benchmark/test/test_signals_modules_reports.py \
  src/benchmark/test/test_cli_progress.py -q -rs
# 53 passed; no skips; three existing third-party deprecation warnings.
```

This includes native SQLite3/MCAP fixtures and old-run compatibility. No live
simulator collection or browser test was performed for this metadata change.

### Follow-up: authoritative simulator spawn located

Read-only Unity scene inspection found the numeric source in the local stock
simulator at
`/mnt/d/Downloads/autodrive_simulator_practice_windows/autodrive_simulator/Data/level0`
(SHA256 `5edbd1bacc15647746e6e89d9bcf1bb9e202d62d47e76ae352497be1404b3f8d`).
The scene's root `Roboracer` Transform (path ID 550) has Unity-world position
`(-3, 0.009999999776482582, 0.800000011920929)` and quaternion
`(0, 0.7071068286895752, 0, 0.7071068286895752)` in xyzw order.
`Infrastructure / Porto Spawn Points / Spawn 1` (Transform path ID 914) has
local position `(7, 0.009999999776482582, 14.800000190734863)`; the Infrastructure
parent is translated `(-10, 0, -14)` with identity rotation/unit scale. This
agrees with the scene vehicle start pose to float precision.

`race.sh:35` calls `common.sh:10`'s Boolean reset. TrackSelector's `Select`
places the vehicle at the selected track's Spawn 1. Its `Switch` explicitly
writes vehicle transforms into `ResetManager.init_VehiclePositions` and
`init_VehicleRotations`; those saved arrays are the runtime reset destination.
Thus the numeric pose is discoverable in simulator assets/runtime state, not
in ROS map YAML. The ROS interface currently omits that saved reset state.
These coordinates describe the Unity vehicle root, not the ROS rear axle.

This local scene inspection does not establish the simulator build or mutable
ResetManager arrays used by an older run whose simulator version was not
recorded. Do not backfill old collections from a different/local build. For
runtime collection the authoritative source is the active ResetManager's saved
arrays, with selected track, coordinate convention, vehicle root identity and
simulator/build provenance, rather than the moving vehicle transform.
