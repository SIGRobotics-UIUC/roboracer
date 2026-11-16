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

## Wheel-contact telemetry implementation (2026-11-16 UTC)

Implemented against RoboRacer base `f33e1845fb8c355fcccc16dd0a8a89f764385aff`
and the separate AutoDRIVE plugin base
`f890810a22381d5335ea8cd441777a5ba294df8a`. Both checkouts contain uncommitted
implementation changes. Pre-existing `.gitignore`, tire-report documentation and
plugin installer changes were preserved. No installed plugin/configuration,
validated track or raw benchmark run was changed. No external network access,
package installation, simulator launch or plugin DLL execution was performed.

Both Python packages built successfully into temporary workspace outputs:

```bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
colcon --log-base /tmp/wheel-contact-colcon-log build --symlink-install \
  --packages-select autodrive_bridge benchmark \
  --build-base /tmp/wheel-contact-colcon-build \
  --install-base /tmp/wheel-contact-colcon-install
source /tmp/wheel-contact-colcon-install/setup.bash
```

The existing installed planning, perception, control and native exporter packages
served as the underlay; colcon warned that they were not rebuilt in the temporary
overlay. No changes in those packages were needed.

The complete non-browser benchmark suite passed **94 tests**, including synthetic
ROS collection, command gating/process cleanup, SQLite3/MCAP bag fixtures, raw
contact arrival/source-clock separation, nonfinite tags, invalid/ungrounded
samples, runtime curve changes, source repetition/reset/session boundaries,
old-bag unavailability and the standardized report contract:

```bash
PATH=/usr/bin:"${PATH}" ROS_DOMAIN_ID=68 /usr/bin/python3 -m pytest \
  ws/src/benchmark/test \
  --ignore=ws/src/benchmark/test/test_race_view_browser.py -q -rs
# 94 passed, 3 existing numerical-library deprecation warnings
```

The initial default-directory invocation reported one collection skip and no
collected tests due to missing `playwright.sync_api`; it is not counted as a
passing suite. Browser tests were explicitly excluded on the successful rerun.
No Playwright/browser dependency was installed, and no browser validation is
claimed. A separate offline synthetic contact report smoke test passed, including
strict JSON export, report rendering and `contacts.jsonl` creation.

The full bridge suite passed **74 tests**, on fixture-selected domains 97 and 77
with ephemeral launch-test ports. After moving contact's monotonic observation
to callback entry, the affected decoder/frame tests were rerun: **37 passed**.
The benchmark ROS fixture uses domain 68; no default-domain motion was commanded.

```bash
PATH=/usr/bin:"${PATH}" /usr/bin/python3 -m pytest \
  ws/src/autodrive_bridge/test -q
PATH=/usr/bin:"${PATH}" /usr/bin/python3 -m pytest \
  ws/src/autodrive_bridge/test/test_fast_bridge.py \
  ws/src/autodrive_bridge/test/test_wheel_contact_decoder.py -q
```

The Windows plugin sources compiled against `/opt/autodrive-bepinex` bindings
with **zero warnings and zero errors**, using temporary copies excluding existing
`bin/obj`, an explicit temporary NuGet configuration with all sources cleared,
NuGet audit disabled, and the installed .NET 10 reference pack. Final compile
outputs are under `/tmp/wheel-contact-checked-compile-787uuj2o/`. No DLL was
executed or installed. The source project continues to target **net6.0**; the
container lacks that targeting pack, so this net10.0 override is a source/API
compile check only and its DLL is **not a deployment artifact**. A real .NET 6
build is still required on the installation machine.

No live AutoDRIVE telemetry, Unity phase/ABI/reset validation, physics-step trace,
telemetry-overhead baseline, wheel-lift/static balance, controlled mass/CoM change,
acceleration/braking, circles or friction sweeps were performed. The new sampling
phase/solver-step flags remain unverified and external-force inventory remains
unknown. Physical force/load interpretation, tire utilization and saturation
metrics remain deferred behind the original plan's acceptance gates. See
`docs/wheel_contact_telemetry.md` for opt-in and wire semantics.

### Socket.IO contact-string transport correction

After a user-reported outer Socket.IO JSONDecodeError, the plugin's contact field
was changed to `base64:` plus base64-encoded UTF-8 JSON. This avoids relying on
stock JSONObject escaping for embedded JSON quotes/backslashes. The bridge
accepts both the new encoding and the earlier plain JSON form; published ROS
messages and benchmark inputs remain ordinary JSON. Regression coverage includes
quotes, backslashes, Unicode, malformed base64, invalid UTF-8 and size bounds.
Focused bridge/decoder checks: **42 passed**. Live confirmation is pending.
The attempted updated C# compile check was blocked by missing mounted BepInEx/
Unity reference DLLs; the earlier successful compile applies to the preceding
version, not this transport correction. No reference packages were installed.

### 0.3.1 investigation diagnostics — not yet deployed

Follow-up local control queries and metadata-only inspection are documented in
`docs/tire_utilization_assessment.md`. No driving/reset/configuration changes were
sent. The active VehicleController must not be confused with AntiRollBar or
AutomobileController types elsewhere in the same assembly. A numerical
flat-contact tangent-force inverse example has rank 3, nullity 5; this is an
identifiability demonstration, not a reconstructed engine tire law.

Prepared plugin 0.3.1 adds read-only collision callback impulses/contact IDs,
collider bounds/type/body membership, sleep state, controller fields and a
whitelist of scalar CoSimManager getters. Event/inventory bounds and raw
nonfinite tags remain explicit. Nothing converts aggregate impulses to tire
forces or changes physics/command settings.

Final .NET 10 source/API compile check against installed references succeeded
with zero warnings/errors in `/tmp/contact-collision-build-15qqvugn/`; this is not
a deployment DLL. A parser smoke check confirmed additional diagnostic fields
and invalid snapshot tags survive both existing decoders independently of wheel
validity. No Python runtime/decoder algorithm was changed by this investigation.
The .NET 6 deployment and native callback behavior still require user-installed
runtime validation. Current source diagnostics have not yet been observed live.

### Actual 0.3.1 reset observations and 0.3.2 deployment artifact

Three stationary reset baselines, strict interpretation limits, and the new
0.3.2 export are recorded in `docs/tire_utilization_assessment.md`. No /drive
publisher was present and no driving commands were sent. Source-time body motion
and force changed despite an always-true reported sleeping Boolean; the Boolean
is now explicitly an unresolved measurement rather than evidence of stale force.

With the user's authorization, only .NET 6 reference packs were downloaded to
/tmp. The genuine net6.0 Release 0.3.2 build passed with zero warnings/errors;
metadata-only inspection verified the target framework, assembly version and
framework reference versions. The target DLL was not executed. Offline benchmark
checks, excluding browser and live ROS collection: **108 passed**, three existing
numerical-library deprecation warnings. New support diagnostics have 18 regression
cases covering same-observed-step conditional projection and refusal of stale,
ambiguous, invalid, duplicate, truncated or sleeping inputs; native/runtime Boolean
disagreement is retained and does not validate force interpretation.

DLL: `ws/log/plugin_exports/wheel-contact-0.3.2-6c337d1124b2/TrackSelect.Windows.dll`
SHA256: `6c337d1124b27e92be0d50a93f3ca90467fc0e033f4d7c46cac870df94dc75fd`

Installation is a user action. Windows PowerShell installation and native callback
behavior have not been executed/tested in this Linux container. The goal pauses
until user confirmation; verification of the running 0.3.2 version/hash is the
first action when resuming simulator tests.

### Bounded mass interventions, plugin 0.3.4

The new native-parameter trial source was net6.0 Release compiled and metadata
checked; managed Unity-stub policy/lifecycle checks passed 49 assertions, and
focused Python contact/support checks passed 35 tests. These are distinct from
the actual simulator checks. Six stationary ±10% mass trials on the designated
codex copy observed applied mass and verified automatic restoration and rest;
final explicit restore/readback confirmed baseline 3.47 kg, no active case.
No drive commands or reset requests were issued in that sweep. It yielded an
approximately additive ~3.2 N raw-force deficit across tested masses, not a
validated percentage scale correction. Full data and limits are recorded in
`docs/tire_utilization_assessment.md`. CoM, wheel-mass, solver-step and moving
force/grip behavior are still not validated by these checks.

### Current stationary diagnostic build, 0.3.6

Historical missing-pack/deployment statements above are superseded by the current
assessment in `docs/tire_utilization_assessment.md`. The read-only installed
binding mount is restored; cached official .NET 6 reference packs permit genuine
net6.0 Release builds with the .NET 10 SDK. The 0.3.6 implementation DLL (not an
obj/ref assembly) built with zero warnings/errors, metadata-verified target and
version, and SHA256
`588e57cebdb9318d6ed7e5bd2e1cc452dd1f158ac1a7012ac86081fadcb1d442`.
Its export directory retains compiled source, input hashes, build commands/log,
metadata and manifest:
`ws/log/plugin_exports/wheel-contact-0.3.6-588e57cebdb9/`.

Managed stub tests linking the actual fixed-case policy/lifecycle source passed
**120 checks**, including refusal, mass/wheel-mass/timestep apply/restore, deadline,
nonfinite input and setter failures. They execute no Unity/plugin DLL and do not
establish native engine behavior. Focused Python checks passed **61 tests**:

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=/repository/ws/src/benchmark:/repository/ws/src/autodrive_bridge:"${PYTHONPATH}" \
  /usr/bin/python3 -m pytest \
  ws/src/benchmark/test/test_contact_support.py \
  ws/src/benchmark/test/test_contact_probe.py \
  ws/src/benchmark/test/test_wheel_contact.py \
  ws/src/autodrive_bridge/test/test_wheel_contact_decoder.py -q
/opt/dotnet/dotnet run \
  --project autodrive_plugin/tests/StationaryTrialChecks/Checks.csproj --no-restore
```

One initial Python invocation accidentally replaced the sourced ROS PYTHONPATH
and failed during conftest import (`rclpy` unavailable). The command above retains
it and passed. This is environment correction, not a skipped test.

Six live stationary wheel-mass cases on 0.3.5 observed all target values and
restoration; their raw force sums changed negligibly despite ±20% wheel masses.
The first 0.3.6 0.5 ms timestep case was **aborted** on required ROS-stream
staleness before the settled window. Explicit cleanup/readback restored 1 ms,
all original masses and inactive state. It is not counted as a passed trial.
No freshness threshold was relaxed and the failed case was not automatically
replayed. These diagnostics send no driving commands. Full outcomes and later
comparisons belong to the current assessment; moving tire-force/grip experiments
remain behind the static/phase gates.

Three subsequent 2 ms stationary cases completed on verified 0.3.6 with automatic
restore/readback in every case. At fixed 3.47 kg body mass, the raw sum changed
from baseline 30.81064 at 1 ms to 27.43028–27.43314 at 2 ms, while conditional
weight remained 34.0407 N. Final values were 1 ms, original body/wheel masses,
no active case. This is strong timestep sensitivity, not a validated correction
law or physical Fz interpretation. The failed 0.5 ms case remains failed.
Source-phase/load acceptance did not pass; no moving friction sweeps, physical
utilization metric or grip feedback were enabled. See the assessment's conclusion
for supported slip diagnostics and all remaining validation boundaries.

## Wheel-slip reporting 1.1 (2026-11-16)

Offline preparation/reporting checks:

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=/repository/ws/src/benchmark:"${PYTHONPATH}" /usr/bin/python3 -m pytest \
  ws/src/benchmark/test/test_signals_modules_reports.py \
  ws/src/benchmark/test/test_bag_loading.py \
  ws/src/benchmark/test/test_wheel_slip.py \
  ws/src/benchmark/test/test_wheel_contact.py -q
# 53 passed; 3 existing scipy/skimage deprecation warnings
PYTHONPATH=/repository/ws/src/benchmark:"${PYTHONPATH}" /tmp/wheel-slip-browser/bin/python -m pytest \
  ws/src/benchmark/test/test_wheel_slip_browser.py \
  ws/src/benchmark/test/test_race_view_browser.py -q
# 4 passed (Playwright Chromium, isolated temporary browser environment)
```

Coverage includes signed values and cancellation, changing peaks/ratios above 1,
separate raw/ratio denominators, invalid/ungrounded wheels, source retransmission
and conflicts, exact/bracketed position matching and yaw wrap, duplicate stamps,
stale/missing poses, incompatible segments/domains, reset splits, ambiguous
projection with retained XY, progress wrap, complete/partial laps, strict JSON,
HTML escaping and slip exports. Browser tests check four four-wheel plots, ratio
reference lines, selection/map synchronization, pin/keyboard controls, lap/wheel/
quantity controls, gaps, invariant overall summaries, responsive stacking and
page errors. Existing Race Performance browser tests continue to pass.

Reanalyzed preserved sealed collection
`wheel_contact_i1-20261116T183616.331674Z` using both `wheel_contact` alone and
`all`, into new analysis IDs `wheel_slip_v1_1_verified_wheel_contact` and
`wheel_slip_v1_1_verified_all`. The measured window contains 702 unique observations,
all with exact localized-pose associations, five complete laps, no conflicting
duplicates, and recorded collision baseline/end counts of zero. The raw seal was
verified again after analysis. Results live under `ws/benchmark_runs/reports/`;
no raw collection or preserved track was modified. These are offline checks of
an existing collection, not new simulator validation or evidence of physical
load/grip semantics. Synthetic tests cover interpolation because this collection
requires none. Rotated map backgrounds are omitted with an explicit diagnostic;
XY and progress remain available. No runtime dependency was added.

Packaging validation: from `/repository/ws`, sourced ROS Humble and ran
`colcon build --symlink-install --packages-select benchmark` (one package
finished). After sourcing `install/setup.bash`, verified the installed renderer's
`wheel_contact_view.js` is present. The JavaScript is declared in `package_data`.
The generated combined real report also opened in Chromium with four slip plots,
five lap choices, Race Performance before Wheel Slip, and no page errors.

## Shared viewer integration (supersedes the separate slip viewer)

Wheel contact now contributes an overlay menu and collapsed Wheel slip plot group
to the original Race Performance SVG track/viewer. The original overlay remains
the default; wheel overlays use fixed numeric color ranges across laps/wheels.
The four-wheel plots share the original lap, pin, keyboard, progress cursor and
zoom controls. Selected contact observations retain their own namespaced identity;
base telemetry and truth require an exact unique timestamp/segment/domain match.
An expandable observation readout shows all wheel values and alignment evidence.
The full-window summary remains a separate static card below the shared viewer.
The original SVG renderer handles preserved rotated map backgrounds as before;
the earlier standalone-renderer limitation no longer applies. The standalone
`wheel_contact_view.js` was removed from source and package data.

Current checks: **54 Python tests passed** (the previous focused command plus
`ws/src/benchmark/test/test_report_extensions.py`), with the same three upstream
deprecation warnings. Generic-composition tests cover multiple contributors,
module-only fallback hosting, and reversed CLI module order. The four browser
checks cover existing Race Performance behavior plus shared-track slip overlays,
all four wheel plots/reference lines, namespaced nonmatching contact selection,
exact-only truth availability, controls, invariant summary and responsive layout.
An initial browser run exposed map viewport sizing after adding the menu; its
height was adjusted to retain navigation visibility and the original tests then
passed. The safe-script test also caught and corrected literal Unicode escaping
in the shared renderer.

Reanalyzed the same sealed contact-enabled collection into new analysis IDs
`shared_slip_view_final_wheel_contact` and `shared_slip_view_final_all`. Opened both real HTML
reports in Chromium: one SVG track each, four added slip plots, 160 colored vehicle
observations on the first lap, no page errors. The collection's 702 unique samples
and five complete laps are unchanged, and the raw seal was verified again.
No map/raceline generation, driving, bridge restart or simulator changes occurred.
See `docs/benchmark_viewer_extensions.md` for the extension contract and an example.

### Compact slip overlay and position card

The overlay menu now has Default and Slip. Slip selects the largest available
peak-slip ratio across the four wheels and both forward/sideways directions at
each retained contact observation. Observation details identify the winning
wheel/direction. Colors are anchored to green at 0 and yellow at 1; values above
1 are red, and fully unavailable observations are gray. Values and full-window
statistics remain unchanged. The plot-group explanation paragraph is omitted.
The position readout uses four fixed columns in one row, with stacked progress
(m/%), position (x/y), and heading (value/unit). Text cannot wrap into another
card row; full values remain in tooltips.

Focused Python checks: 28 passed using
`test_wheel_slip.py test_wheel_contact.py test_report_extensions.py` with the
sourced ROS environment and benchmark PYTHONPATH. The browser fixtures exercise
0/1/>1/missing colors, the two-option overlay menu, absent explanation, and
single-row nonoverflowing cards at 600 px and 360 px widths, alongside the
existing race-view navigation checks. An initial browser invocation collected
the old two-row controls assertion while that test file was being updated;
the corrected test retains two rows for controls and requires one for the card.
A separate initial plot-start timeout was not counted as a passing check.

New real-run output: `compact_slip_overlay` under the preserved contact-enabled
collection's reports directory. The collection seal was verified again; no
track assets or prior analyses were overwritten.
Fresh browser run completed: **4 passed** using
`test_race_view_browser.py test_wheel_slip_browser.py`. The generated real report
opened with exactly two overlay options, one position-card row and no page errors.

### Plot expansion across laps

The shared viewer now preserves open/closed state for every plot group and nested
plot panel when changing laps. State keys include the contributing module,
group and panel, so future modules have independent state. State is captured
from the current DOM before rebuilding plots, including nested panels in closed
groups and changes made immediately before navigation. The browser regression
opens Wheel slip, closes its first plot and Motion and commands, opens Tracking,
and checks the same expansion state after switching laps in both directions.
Validation: `test_race_view_browser.py test_wheel_slip_browser.py` passed all four
Chromium tests. New report ID: `persistent_plot_sections`; the original collection
seal was verified after analysis. Existing reports remain untouched.
