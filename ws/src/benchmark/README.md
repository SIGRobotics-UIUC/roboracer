# Reproducible race benchmarks (v1)

This package separates collection from offline numerical analysis. It never
imports or runs `ws/scripts/`. Existing algorithm launches and native AutoDRIVE
interfaces remain in use. Generated artifacts live under **`ws/benchmark_runs/`**.

## Build and commands

Rebuild the Docker image to install MCAP support and Plotly (`docker compose build ros`);
`docker compose down` / `up` alone does not rebuild an existing image. Packages
installed manually inside a container disappear when that container is recreated.
Recreate the service to
apply the read-only `/repository` source mount. Do this with the simulator
disconnected and existing launches stopped. The additional mount enables Git
snapshots; `BENCHMARK_REPO_ROOT` may identify another readable checkout.

Inside the Humble container, from `/root/ws`:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select benchmark
source install/setup.bash
ros2 run benchmark benchmark run --config src/benchmark/config/example.yaml \
  --run-id example-01 --set components.controller.params.speed_scale=0.5
ros2 run benchmark benchmark status
ros2 run benchmark benchmark stop
ros2 run benchmark benchmark analyze example-01 --analysis-id baseline --modules all
```

`run --config porto` resolves `src/benchmark/config/porto.yaml` from the
workspace root (or `ws/src/benchmark/config/porto.yaml` from the repository
root), then the installed package's config directory. Existing explicit file
paths take precedence. Setup/readiness, warm-up versus measured lap completion,
stopping and final results are printed live. Process log paths and the sealed
raw collection directory are labeled; the final JSON also includes the failure
reason when a run aborts.

Add `--analyze` to `run` to generate the report after the collection has been
sealed and owned processes cleaned up:

```bash
ros2 run benchmark benchmark run --config porto --run-id porto --analyze
```

This also analyzes finalized aborted/stopped runs for diagnosis; collection
failure still returns a nonzero exit status. Incomplete/unsealed runs are not
analyzed automatically. Without `--analyze`, the terminal prints a ready-to-copy
`benchmark analyze <id>` command with the full generated run ID. Custom
artifact roots are called out separately with the required `--log-root` option. Analysis prints
labeled paths for the report directory, HTML report, and metrics JSON.

Global `--log-root` and `--tracks-dir` options precede the subcommand. Defaults
are `./benchmark_runs` and `./tracks`; run from the workspace root. `run --run-id porto`
generates an ID such as `porto-20261115T045626.159100Z`. Without `--run-id`,
the ID is just the UTC timestamp. Generated IDs use microseconds and add a
numeric suffix on collision. Run ID prefixes may contain up to 99 characters.
Analysis lookups use the full generated run ID; explicit analysis and recovery
destination IDs remain exact and exclusive.
Multiple analysis IDs coexist; neither raw runs nor reports are overwritten.

Compose bind-mounts `./ws` to `/root/ws`, including `benchmark_runs/`. A report at
`/root/ws/benchmark_runs/reports/<run_id>/<analysis_id>/report.html` in the
container is available at `ws/benchmark_runs/reports/<run_id>/<analysis_id>/report.html`
on your laptop. Open that host file in a browser; no image rebuild or additional
mount is needed. Existing runs under `ws/log/` stay there; analyze them with
`ros2 run benchmark benchmark --log-root /root/ws/log analyze <run_id>`.

Start the normal bridge separately; the simulator must be connected on the
selected track and in Autonomous mode. The benchmark cannot switch simulator
tracks, select Autonomous mode, pause physics, or place an arbitrary initial
pose. It uses `/autodrive/reset_command` and requires the reset event.

## Managed and metadata-only components

The managed adapters support MAP and pure pursuit with Cartographer or MCL.
Controller parameters are explicitly typed/mapped in `collector/config.py`;
MCL maps `particles` and `seed` (-1 retains its nondeterministic default), while Cartographer has no extra managed
parameters. Unknown managed types/parameters fail validation. Optional
components with `managed: false` may have arbitrary JSON-compatible parameters;
these are metadata, not instructions to configure ROS. There is no general
component launcher. Online avoidance and MPC/MPCC launch adapters are currently
unsupported; they must not be represented as managed components.

`experiment.mode: manual` requires all components to be metadata-only. Start
the external localizer/controller yourself, remapping the controller's `/drive`
to `/benchmark/requested_drive`. The manager still owns the command gate, reset,
recording, readiness and stopping. External processes are not terminated, but
their commands cannot pass the inhibited gate. A direct `/drive` publisher is
rejected. Manual parameters are never treated as an applied speed profile.

## Lifecycle and stopping

Preflight checks the transport clock's bridge parameters/owner, track assets,
command ownership, and existing managed nodes. The gate starts inhibited;
recording starts before reset/localization/controller startup. Readiness checks
valid fresh inputs and recorder accepted-write counts before enabling motion.
The recorder's acknowledgement means rosbag2 accepted the write, **not** an
individual fsync guarantee. Closing the writer flushes the bag before sealing.

Managed controllers publish only to `/benchmark/requested_drive`. A separate
gate is the sole `/drive` publisher, forwards unchanged while enabled, and
latches closed on a fault or lifecycle stop. It monitors manager heartbeat,
command freshness, required driving inputs and clock progression with monotonic
timers. The forwarding hop is benchmark overhead, including in logging-off runs.
It does not introduce ground truth into controller feedback.

On completion, stop or abort: inhibit the gate, SIGINT owned algorithm process
groups, repeatedly send zero-speed commands at 20 Hz, and confirm fresh wheel
odometry below 0.05 m/s for 0.5 s after at least 1.5 s of stopping. The default
confirmation timeout is 5 s. Controller groups are checked/escalated if needed;
unconfirmed stops are recorded explicitly. Recording continues through stopping,
then the bag is closed, metadata/hashes written, and the collection sealed.

The adapter ramps speed; one zero command is insufficient. The bridge's
monotonic 0.5 s command timeout is a fallback, not proof of rest. Disconnects
cannot establish physical stopping. If the gate dies, the bridge stops receiving
commands, but rest is reported unconfirmed. These mechanisms require AutoDRIVE
validation before relying on physical stopping. They are not complete collision
protection; existing controller/localization safety checks remain unchanged.

If `/clock` stops advancing, the gate withholds `/drive` instead of repeatedly
sending zeros: the adapter's ROS-time speed ramp could otherwise retain positive
throttle while its time is frozen. This selects the existing bridge's monotonic
command-timeout fallback. The chosen stop mode is recorded, and without fresh
odometry the stop remains unconfirmed.

Default warm-up is one native lap-counter increment, excluded from measured
laps; subsequent configured lap increments are measured. A zero-warm-up run
includes a potentially partial/standing first lap and must be interpreted that
way. Separate lap/time messages have no shared simulator frame ID; associations
are time-bounded and labelled accordingly. Counter jumps/regressions abort rather
than inventing laps. Collisions abort by default (`collision_policy: continue`
can retain a collision-contaminated run, but respawn/reset events still abort).

## Clocks, loss and QoS

V1 requires the existing bridge's `/clock` with `publish_clock` and
`use_sim_time` enabled. **This is elapsed host monotonic time with a fixed epoch,
not simulator physics time.** It continues through simulator stalls. Native
lap-time values retain their native simulator units. No Windows/container clock
equivalence, physics step, or cross-domain latency is inferred.

Essential topic valid-data loss defaults to 1 s, configurable by exact topic
under `watchdogs.signals`; manager/gate heartbeat timeout also defaults to 1 s.
Separate monotonic checks catch stalls even when ROS time freezes. Jitter and
isolated drops below the limits continue with gap reporting. Pauses/disconnects
during motion abort; silence is never inferred to mean an intentional pause.
Startup remains inhibited with a 30 s readiness deadline. Recorder exit/write
errors and disk reserve exhaustion abort immediately. Disk reserve defaults to
256 MiB, so stopping/final metadata have headroom; abrupt exhaustion may still
leave an incomplete run. Unexpected resets or major source-clock discontinuities
abort measurement.

Basic capture includes localization, odometry, scan/IMU, requested/final drive,
normalized actuator commands and feedback, native lap/collision/reset events,
clock, TF, and reference/provenance evidence where available. Detailed adds map,
particles, encoders and optional diagnostics. Ground truth is optional unless
`logging.require_ground_truth: true`. Essential subscriptions and writes are
checked separately. Subscriptions adapt to offered reliability; `/clock` is
best effort; `/tf_static`, map and provenance request transient-local durability.
Endpoint types/QoS and recorder subscription contracts are preserved.

`logging.level: off` disables the telemetry recorder, not lifecycle monitoring,
the gate, reset or stopping. Minimal lifecycle/configuration metadata remains.
Telemetry-dependent metrics are unavailable. MCAP is selected when installed;
otherwise automatic storage explicitly records SQLite3 fallback. Explicit MCAP
selection fails if unavailable.

## Preservation and analysis

`benchmark_runs/runs/<run_id>/` contains configuration, source snapshots, assets, processes,
events, bag, recording-order index, final metadata, and integrity manifest.
Finalization atomically creates `complete.json` and immediately removes write
permissions. The framework refuses modification/reuse and verifies content
hashes on analysis. Read-only modes are not a security boundary against the
file owner deliberately changing permissions. Preserve a finalized run intact.

Git snapshots include commit, dirty state, binary tracked diff and copies/hashes
of untracked nonignored files. Installed algorithm configuration and track assets
are copied. Parameter-service snapshots preserve effective ROS settings when
available. Simulator version/physics step/seeds/vehicle settings are explicit
operator metadata or unknown. The existing localization gate's ground-truth/map
origin dependency and bridge privileged respawn detection remain review debt.
The default stack is not claimed to be ground-truth independent.

Analysis requires no active ROS graph or simulator. It uses rosbag2 readers and
message definitions in the container, a versioned signal map, shared alignment,
and independent registered modules. The capture index restores writer order
even if the storage reader sorts timestamps; duplicates remain identifiable.
It also preserves receipt monotonic timestamps from the collector container.
Measurement boundaries use that same verified domain so stopping/warm-up samples
stay excluded even when ROS recording timestamps freeze. These receipt stamps
are not callback execution profiling or simulator acquisition timestamps.
Source samples are stably sorted only inside continuous clock segments. Large
ambiguous regressions, recorder regressions and resets break alignment; no
extrapolation/interpolation across unsupported gaps or resets. Angles wrap and
TUM `psi` converts to ROS yaw by adding pi/2. Ground truth comparisons require
preserved map/world alignment. Missing/invalid inputs produce null with reasons.

Race metrics include native lap times/completion, collisions, signed estimated
and true tracking errors, wheel speed versus spatial reference, localization
error, reference progress and optional map center clearance. Raster clearance
is not physical body clearance. Exact controller target state and exact
collision instants are unavailable. Computational metrics cover publication
spacing/frequency/jitter and verified source-stamp age at recording. Callback,
solver, localization execution, consumption age and end-to-end application
latencies are unavailable without the endpoint evidence listed in every JSON
result. Arrival intervals are never substituted for computation durations.
Control smoothness and tire dynamics remain deferred.

Reports live in `benchmark_runs/reports/<run_id>/<analysis_id>/` with strict JSON and HTML.
Modules return generic metrics, warnings and static PNG/JPEG figures; custom
HTML callbacks/interactive dependencies are excluded. Adding a registered module
does not change collection or rendering. Exceptions are isolated per module.
Each module receives its own artifact directory. Register a new `Module` in
`analysis/modules/__init__.py` (or import its registration there); declare
required/optional canonical signals, return metrics using `core.results.metric`
and figures as relative paths with captions. Required missing signals skip the
module. Malformed results and nonfinite JSON values fail only that module.

An incomplete run cannot be analyzed implicitly. After all collection processes
are stopped, `benchmark recover OLD --run-id NEW` copies its artifacts to a new
finalized `recovered_incomplete` run, with unknown stop status. It never repairs
a bag or overwrites the original automatically. Bag repair, if needed, is a
separate explicit action on a copy. Finalized runs cannot be recovered.

## Validation

```bash
ROS_DOMAIN_ID=68 python3 -m pytest src/benchmark/test -q
```

ROS integration fixtures use isolated domains 68/69 and temporary artifacts.
They do not connect to AutoDRIVE. Full simulator acceptance must still record
track/controller/localizer/parameters, laps/collisions, stop confirmation,
input stalls, disconnects and recording failures. Synthetic fixtures do not
establish collision-free laps or physical stopping. See `VALIDATION.md` for the
implementation's actual checks and limitations.

Race Performance owns its offline viewer: SVG trajectories over an embedded
occupancy-map PNG, and Plotly signal panels. The library is embedded in the
HTML (no CDN or network required). Its map and signal plots share lap/time
selection; click to pin and use Unpin to resume hover inspection. Signal legend clicks
hide traces; selections persist across laps. Drag horizontally to zoom; signal
panels share horizontal ranges. Groups and individual panels can collapse.
Trajectory markers and readouts use aligned localized-pose timestamps, never
nearest spatial neighbors. Gaps and ambiguous reference projections remain
unavailable. Odometry yaw is not plotted in map coordinates without validated
TF alignment; normalized actuator evidence remains in the core inventory.

Modules can supply `Module.render_report(result)` to return their section HTML
and scripts. Plotly is a Race Performance dependency, not a core requirement.
`window.benchmarkReport` exposes `getSelection()`, `subscribe(callback)` (returns
an unsubscribe function), and `requestSelection({lap, timestamp})`. Lap indices
are zero-based; Race Performance timestamps are seconds relative to its stated
source timestamp origin. Changing laps clears the timestamp. No track overlay
extension is exposed until another implemented module requires it.

The map stays beside the plots on wide displays. Trajectory source labels are
fixed; only signal legends toggle traces. Map wheel zoom is independent of
linked horizontal signal zoom. Cursor lines and trajectory markers update as
DOM/SVG overlays instead of redrawing Plotly on each hover. Reference-distance
wraps break signal lines. Signed lateral distributions retain signed means and
medians, with explicitly named absolute P95/max magnitudes; vehicle-center
clearance distributions report P5/min. Computation Timing is currently hidden
from reports and excluded from the default `all` module selection.
