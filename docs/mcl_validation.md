# MCL independent-sampling and recovery update — 2026-11-08

This update supersedes the sampler/recovery findings in the historical report
below. Production changes are limited to independent KLD draws and an adaptive
recovery population floor. No simulator, movement, reset, commit or push occurred.
The prior `mcl_validation_results.json` is preserved unchanged. Current raw results
are in [mcl_validation_results_independent.json](mcl_validation_results_independent.json).

## Implemented behavior

Adaptive resampling now takes one fresh independent uniform variate per draw and
selects its ancestor using the inverse weighted CDF. Source weights/cloud stay
fixed throughout the operation. Occupied **output** bins update the KLD population
requirement after each draw. Minimum/maximum limits, circular yaw binning, pf_err,
probability-to-quantile pf_z conversion, and ESS triggering are unchanged. Fixed
mode retains its original systematic sampler. Independent sampling removes the
permuted-block dependence concern. The KLD bound remains approximate and capped;
missing support and small unsampled modes remain limitations. This is KLD
resampling of the existing weighted cloud, not a new localization algorithm.

`recovery_min_particles: 1000` is a new positive-integer ROS parameter in
`perception/config/mcl.yaml`. The effective recovery count is
`max(current_count, min(recovery_min_particles, max_particles))`, only in adaptive
mode. The existing minimum is respected by current_count. A recovery floor never
shrinks a larger cloud, and fixed mode ignores the floor. Override via a node ROS
parameter (or edit the source YAML and rebuild); this is not a new launch argument.
For example, with no other localizer running, from a sourced /root/ws shell:

```bash
ros2 run perception mcl_node --ros-args --params-file /root/ws/install/perception/share/perception/config/mcl.yaml -p track:=test_track_v2 -p adaptive_particles:=true -p recovery_min_particles:=1000
```

All init_filter callers were reviewed. The explicit recovery flag is set for:

| Caller | Purpose | Recovery floor |
|---|---|---|
| on_find | Initial/reset/loss global-search result | yes |
| on_verify | Accepted relocation by full-map verification | yes |
| on_check | Confirmed global relocation | yes |
| validate | Switch to an alternative hypothesis | yes |
| initialpose_cb | Broad manual pose initialization | yes |
| reanchor | Ordinary local correction/recheck | no |

Matched/global results still use the **existing** tighter Gaussian where they did
before; only population changes. keep_localized, matched, Gaussian widths,
convergence thresholds, recovery state transitions and topics are unchanged.
Explicit flags avoid inferring policy from log-reason text or keep_localized (which
is also true during global relocation). A subsequent ESS-triggered resample can
shrink the cloud again: this is an initialization floor, not a persistent recovery
mode. No privileged simulator inputs were added. Existing ground-truth review debt
remains outside this change.

## Regression results and recovery count measurements

Complete suites: **177 passed in 96.22 seconds, no skips**. New tests compare the
exact independent inverse-CDF draw sequence, reject a zero-weight ancestor, and
verify normalized output weights. Existing population adaptation, confidence/yaw,
seed reproducibility, synthetic LiDAR, fixed sampler, recovery and frame-jump tests
passed. Callback tests exercise every caller above in both modes, including a
floor above max_particles, and preserve localization state behavior.

The following counts were directly asserted in callback tests after forcing a
concentrated cloud to contract:

| Mode / configuration | Before | Global recovery or /initialpose | Local re-anchor |
|---|---:|---:|---:|
| adaptive, floor 1000, max 2000 | 200 | 1000 | 200 |
| adaptive, floor 1200, max 500 | 200 | 500 | 200 |
| fixed, particles 1000 (either config) | 1000 | 1000 | 1000 |

Algorithm tests also verify that requesting a lower floor never shrinks an
already larger population. These are count/state callback measurements, not
simulator recovery-success measurements. The trajectory benchmark below never
calls init_filter, so its performance results isolate the sampler change and do
not measure recovery-initialization cost or success rate.

Exact regression command from repository root:

```bash
docker compose run --rm --no-deps ros bash -lc 'source /opt/ros/humble/setup.bash; export PYTHONPATH=/root/ws/src/perception:$PYTHONPATH; ROS_DOMAIN_ID=42 python3 -m pytest src/perception/test/test_mcl.py src/perception/test/test_mcl_node.py -q'
```

Host Python compilation and `git diff --check` also passed.

## Five-seed benchmark comparison

The benchmark script, parameters, trajectory/sensor generator, seeds 1–5 and ESS
trigger were unchanged. The existing ROS Humble image was used in a temporary
container after tests completed, without concurrent test load. There are 210
updates per mode/seed/scenario, with separate warm-up and alternating mode order.
All numbers are measurements. Means below are averages of per-run means; p95/p99
columns are **averages of per-run percentiles**, not pooled percentiles. Position
maximum and count maximum are the worst across five runs. Resamples are totals
across five runs. PF timing excludes scan matching, sensor generation and ROS;
fit-only time is recorded separately in the JSON. This is one timing repeat per
seed, with no confidence intervals or isolated CPU scheduling guarantee.

| Scenario | Mode | Version | Position mean/max m | Heading mean rad | Count min/mean/max | PF mean/p95/p99 ms | Resamples |
|---|---|---|---|---:|---|---|---:|
| baseline | fixed | prior strata | 0.09047/0.27451 | 0.00439 | 1000/1000.00/1000 | 1.341/1.471/1.542 | 383 |
| baseline | fixed | independent update | 0.09047/0.27451 | 0.00439 | 1000/1000.00/1000 | 1.317/1.450/1.482 | 383 |
| baseline | adaptive | prior strata | 0.09082/0.28443 | 0.00459 | 200/201.08/263 | 0.465/0.747/0.871 | 384 |
| baseline | adaptive | independent update | 0.09109/0.34718 | 0.00456 | 200/201.01/263 | 0.514/0.903/0.950 | 379 |
| uncertain | fixed | prior strata | 0.08638/0.53020 | 0.00561 | 1000/1000.00/1000 | 1.292/1.438/1.540 | 1045 |
| uncertain | fixed | independent update | 0.08638/0.53020 | 0.00561 | 1000/1000.00/1000 | 1.237/1.332/1.348 | 1045 |
| uncertain | adaptive | prior strata | 0.08828/0.52473 | 0.00618 | 200/292.89/675 | 1.050/1.770/2.011 | 1045 |
| uncertain | adaptive | independent update | 0.08676/0.53565 | 0.00641 | 200/287.24/713 | 1.246/2.153/3.101 | 1045 |

Fixed-mode accuracy, counts and resampling statistics match the previous JSON
**exactly for every seed**, checked programmatically. Its timing changed despite
unchanged computation (baseline mean 1.341 to 1.317 ms, uncertain 1.292 to 1.237 ms),
showing that historical runtime comparisons contain environment noise.

Adaptive baseline mean position error changed .09082 to .09109 m, while worst
error increased .28443 to .34718 m (seed 3). Baseline heading mean improved slightly.
Under uncertainty mean position error improved .08828 to .08676 m, but heading mean
increased .00618 to .00641 rad and worst position error increased .52473 to .53565 m.
These small averages do not establish statistical equivalence; the increased peak
error warrants bag/simulator validation. No accuracy thresholds were weakened.

Adaptive mean latency increased about 10.5% in baseline and 18.6% under uncertainty.
Baseline p95/p99 rose .747/.871 to .903/.950 ms; uncertain p95/p99 rose 1.770/2.011
to 2.153/3.101 ms. The new implementation makes a scalar RNG/CDF lookup per particle,
whereas the prior sampler amortized work using vectorized blocks. Independent draws
also have higher sampling variance. Both are expected tradeoffs; timing/scheduling
noise prevents attributing every change to them. Uncertain seed 2 had p99 5.579 ms
(other uncertain adaptive seeds 2.432–2.588 ms), retained in results rather than
excluded. Its fit-only overhead was also higher, consistent with possible system
noise but insufficient to prove its cause. No parameters were tuned to hide this.
In the new uncertain runs adaptive mean latency is approximately equal to fixed
(1.246 versus 1.237 ms), with higher tail latency despite fewer particles.

Uncertain adaptive runs show **79–88 population increases per seed**, with peaks
650–713; prior peaks were 600–675. Adaptation still grows after contraction. Counts
do not need to reach 2000 in this stress scenario; the broad-distribution unit test
separately checks the cap. The uncertainty scenario is synthetic (initial .8 m/.5
rad and elevated motion noise); it is not a new production parameter recommendation.

### Current individual runs

| Scenario | Seed | Mode | Position mean/max m | Heading mean rad | Count min/mean/max | PF mean/p95/p99 ms | Resamples | Count increases |
|---|---:|---|---|---:|---|---|---:|---:|
| baseline | 1 | fixed | 0.09674/0.27370 | 0.00444 | 1000/1000.00/1000 | 1.716/1.896/1.927 | 77 | 0 |
| baseline | 1 | adaptive | 0.09135/0.30022 | 0.00453 | 200/201.27/263 | 0.509/0.910/0.961 | 75 | 3 |
| baseline | 2 | adaptive | 0.09318/0.23751 | 0.00460 | 200/200.70/263 | 0.515/0.894/0.934 | 78 | 2 |
| baseline | 2 | fixed | 0.09287/0.27451 | 0.00429 | 1000/1000.00/1000 | 1.213/1.336/1.379 | 78 | 0 |
| baseline | 3 | fixed | 0.08728/0.25601 | 0.00447 | 1000/1000.00/1000 | 1.219/1.339/1.364 | 75 | 0 |
| baseline | 3 | adaptive | 0.09233/0.34718 | 0.00489 | 200/200.65/233 | 0.514/0.913/0.955 | 76 | 3 |
| baseline | 4 | adaptive | 0.08771/0.21961 | 0.00394 | 200/200.50/248 | 0.512/0.899/0.934 | 74 | 1 |
| baseline | 4 | fixed | 0.08761/0.22774 | 0.00375 | 1000/1000.00/1000 | 1.214/1.320/1.361 | 77 | 0 |
| baseline | 5 | fixed | 0.08788/0.24868 | 0.00499 | 1000/1000.00/1000 | 1.222/1.360/1.381 | 76 | 0 |
| baseline | 5 | adaptive | 0.09089/0.26223 | 0.00486 | 200/201.95/248 | 0.522/0.899/0.963 | 76 | 1 |
| uncertain | 1 | fixed | 0.08536/0.48393 | 0.00535 | 1000/1000.00/1000 | 1.232/1.324/1.346 | 209 | 0 |
| uncertain | 1 | adaptive | 0.08066/0.46133 | 0.00582 | 200/290.86/713 | 1.217/2.143/2.588 | 209 | 82 |
| uncertain | 2 | adaptive | 0.08634/0.47759 | 0.00676 | 200/288.64/700 | 1.416/2.374/5.579 | 209 | 87 |
| uncertain | 2 | fixed | 0.09429/0.48523 | 0.00576 | 1000/1000.00/1000 | 1.224/1.329/1.343 | 209 | 0 |
| uncertain | 3 | fixed | 0.08051/0.42154 | 0.00560 | 1000/1000.00/1000 | 1.263/1.366/1.384 | 209 | 0 |
| uncertain | 3 | adaptive | 0.09119/0.46336 | 0.00665 | 200/287.84/650 | 1.214/2.210/2.463 | 209 | 79 |
| uncertain | 4 | adaptive | 0.08694/0.53565 | 0.00602 | 200/283.37/713 | 1.192/2.009/2.443 | 209 | 88 |
| uncertain | 4 | fixed | 0.08219/0.45113 | 0.00546 | 1000/1000.00/1000 | 1.241/1.320/1.331 | 209 | 0 |
| uncertain | 5 | fixed | 0.08953/0.53020 | 0.00589 | 1000/1000.00/1000 | 1.227/1.321/1.336 | 209 | 0 |
| uncertain | 5 | adaptive | 0.08870/0.48977 | 0.00679 | 200/285.49/675 | 1.191/2.031/2.432 | 209 | 83 |

Reproduce (repository root; runs offline code only):

```bash
docker compose run --rm --no-deps ros bash -lc 'source /opt/ros/humble/setup.bash; export PYTHONPATH=/root/ws/src/perception:$PYTHONPATH; python3 src/perception/test/benchmark_mcl.py --seeds 1 2 3 4 5 --scenario all'
```

The simulator startup/stationary procedure below remains prepared, not executed.
The sampler-selection blocker is resolved by independent draws; the initialization-
count concern is addressed by the floor. Remaining validation includes real recovery
quality with that floor, map/scene compatibility, stationary/bag integration,
node telemetry and latency tails. No further tuning or live testing was performed.

---

The following is the preserved prior review. Its statements about stratified
sampling, retained recovery counts, and 148 tests describe the previous version;
the update above supersedes those statements. Simulator commands remain a prepared
procedure, not evidence of live validation.

## Historical stratified-sampler review and prepared simulator procedure — 2026-11-08

Only the benchmark and this report/results were changed during this review. Runtime
localization code was retained. No live stack, simulator, motion, reset, commit,
or push was started. Existing generated track artifacts were preserved.

## Review findings

`git status --short` showed seven modified tracked implementation/config/launch/test
files and two untracked benchmark/documentation files from the initial work.
`git diff --check` passed before review.

Occupied bins are counted from the **output samples**, using a set, not from source
particles or covariance. Each newly occupied bin updates the KLD requirement. The
stopping check occurs after adding the sample and counts the sample just added.
The first bin uses the configured floor; populations cannot exceed the allocated
maximum. p, n and w are updated together, with uniform normalized weights. All
predict/weight/estimate/spread code and display sampling use the current n/arrays.
Yaw uses 63 equal bins at the default setting, actual width 2pi/63 = 0.099733 rad,
with one bin centered at +/-pi. Negative x/y use floor. pf_z is correctly converted
from .99 to 2.32635; the bound is evaluated in log space.

**Statistical concern:** randomized permuted strata give each pre-stopping draw the
correct weighted categorical marginal. Within a block the draws are dependent;
only a complete block has full stratification. Distribution-dependent stopping
can correlate final population size and the retained empirical distribution.
There is no established KLD confidence guarantee for this particular sampler.
I recommend replacing it with independent uniform inverse-CDF categorical draws
for a conventional sequential KLD baseline in a separately reviewed change. Such
a change would increase sampling variance and require renewed benchmarks. The
present sampler was deliberately retained to measure the implementation under
review. Fox's original derivation/algorithm is the reference:
https://papers.nips.cc/paper_files/paper/2001/file/c5b2cebf15b205503560c4e8e6d1ea78-Paper.pdf
Even conventional draws would only bound approximation of the represented discrete
cloud, subject to the asymptotic approximation and configured maximum; they do
not establish pose accuracy or recover missing support.

**Recovery interaction:** Gaussian initialization/re-anchors retain the current
population. Recovery after shrinkage can start with 200 instead of 1000 particles.
No recovery decisions or convergence thresholds changed, but hypothesis coverage,
Monte Carlo estimates, spread and time-to-convergence can differ. Resampling only
copies existing poses; growing to 2000 cannot create lost support. Matched
re-anchors intentionally create a tighter cloud, so low counts there are expected.
The global matcher and Alternatives remain separate mechanisms, not encoded by the
particle population. Global search overhead may dominate overall node latency.

Fixed mode retains the original systematic positions, RNG usage, weights and fixed
population for valid configurations, with an exact-draw regression. New KLD
configuration validation runs even in fixed mode, so invalid unused KLD settings
can now fail construction; this is a configuration compatibility caveat. ROS
mode defaults are intentionally adaptive, while the reusable constructor defaults
to fixed. ESS compares to half the current count and can leave a broad uniform
cloud unchanged until weights degenerate. Once stationary and localized, motion
gating skips further PF updates, including forced resampling.

**Numerical edge debt:** extreme finite but microscopic bin widths can overflow
pose/bin division or 2pi/bin_yaw before integer conversion; current validation only
checks finite positive widths. Nonfinite particles explicitly raise ValueError
rather than publishing an invalid estimate. Practical configured values are covered
by tests, but arbitrary extreme configurations are not hardened. The adaptive
weight sanitation silently substitutes uniform weights for an invalid population;
this avoids NaNs but is not evidence of valid localization.

## Measured benchmarks

Existing ROS Humble image `sigrobotics_f1tenth:humble`, temporary container, seeds
1–5, 210 updates per run, ESS enabled, no concurrent regression workload.
Inputs are generated once per seed/scenario and shared by the two modes. Five
unmeasured warm-up updates use a separate filter. Mode order alternates by seed.
Baseline retains the original .15 m/.1 rad initial uncertainty and motion noise.
Uncertain uses .8 m/.5 rad initial uncertainty and motion noise_x/noise_y=(.5,.1),
noise_yaw=(.2,.3); sensor inputs and odometry perturbations remain identical across
modes. These are artificial stress parameters, not proposed ROS defaults.

All values below are measurements, not estimates. Particle counts are post-update
counts (both start at 1000). Time includes predict, weighting, resampling, estimate,
and spread; excludes synthetic sensor generation, ROS and scan matching. Heading
is mean absolute wrapped error. Percentiles are per run across 210 updates, so
p99 uses only a few tail samples. Fit-only timing is retained in the JSON; neither
global nor local search cost is represented. CPU scheduling and VM timing still
limit conclusions. No confidence intervals or accuracy guarantees are inferred.

| Scenario | Seed | Mode | Position mean/max m | Heading mean rad | Particles min/mean/max | PF mean/p95/p99 ms | Resamples | Count increases |
|---|---:|---|---|---:|---|---|---:|---:|
| baseline | 1 | fixed | 0.0967/0.2737 | 0.00444 | 1000/1000.0/1000 | 1.755/1.952/2.021 | 77 | 0 |
| baseline | 1 | adaptive | 0.0912/0.2844 | 0.00450 | 200/201.0/263 | 0.467/0.763/0.797 | 77 | 2 |
| baseline | 2 | adaptive | 0.0939/0.2405 | 0.00459 | 200/200.8/233 | 0.485/0.748/1.158 | 80 | 3 |
| baseline | 2 | fixed | 0.0929/0.2745 | 0.00429 | 1000/1000.0/1000 | 1.239/1.361/1.534 | 78 | 0 |
| baseline | 3 | fixed | 0.0873/0.2560 | 0.00447 | 1000/1000.0/1000 | 1.205/1.307/1.323 | 75 | 0 |
| baseline | 3 | adaptive | 0.0859/0.2311 | 0.00452 | 200/201.2/233 | 0.447/0.730/0.772 | 75 | 2 |
| baseline | 4 | adaptive | 0.0921/0.2516 | 0.00424 | 200/201.0/233 | 0.458/0.737/0.821 | 75 | 4 |
| baseline | 4 | fixed | 0.0876/0.2277 | 0.00375 | 1000/1000.0/1000 | 1.253/1.372/1.427 | 77 | 0 |
| baseline | 5 | fixed | 0.0879/0.2487 | 0.00499 | 1000/1000.0/1000 | 1.254/1.366/1.404 | 76 | 0 |
| baseline | 5 | adaptive | 0.0910/0.2694 | 0.00510 | 200/201.4/248 | 0.470/0.756/0.808 | 77 | 2 |
| uncertain | 1 | fixed | 0.0854/0.4839 | 0.00535 | 1000/1000.0/1000 | 1.262/1.364/1.382 | 209 | 0 |
| uncertain | 1 | adaptive | 0.0823/0.4837 | 0.00613 | 200/290.7/600 | 1.011/1.727/1.882 | 209 | 94 |
| uncertain | 2 | adaptive | 0.1016/0.4847 | 0.00685 | 200/293.8/663 | 1.047/1.787/2.148 | 209 | 95 |
| uncertain | 2 | fixed | 0.0943/0.4852 | 0.00576 | 1000/1000.0/1000 | 1.264/1.367/1.376 | 209 | 0 |
| uncertain | 3 | fixed | 0.0805/0.4215 | 0.00560 | 1000/1000.0/1000 | 1.281/1.464/1.717 | 209 | 0 |
| uncertain | 3 | adaptive | 0.0841/0.4975 | 0.00596 | 200/291.8/663 | 1.030/1.739/2.015 | 209 | 91 |
| uncertain | 4 | adaptive | 0.0896/0.4668 | 0.00582 | 200/291.2/675 | 0.999/1.604/1.875 | 209 | 98 |
| uncertain | 4 | fixed | 0.0822/0.4511 | 0.00546 | 1000/1000.0/1000 | 1.327/1.508/1.599 | 209 | 0 |
| uncertain | 5 | fixed | 0.0895/0.5302 | 0.00589 | 1000/1000.0/1000 | 1.327/1.488/1.629 | 209 | 0 |
| uncertain | 5 | adaptive | 0.0838/0.5247 | 0.00613 | 200/297.0/625 | 1.165/1.994/2.135 | 209 | 86 |

The uncertain adaptive runs show 86–98 count increases, so growth was observed
throughout tracking rather than inferred from initialization. Peaks of 600–675
remain below the initial 1000: this scenario demonstrates growth from a reduced
population, not a need to hit the 2000 cap. The broad-distribution unit test covers
the cap separately. High-uncertainty adaptive tails can exceed fixed-mode tails.
No scan matching or recovery corrected these synthetic tracking estimates.

Reproduce from repository root (runs only offline code):

```bash
docker compose run --rm --no-deps ros bash -lc 'source /opt/ros/humble/setup.bash; export PYTHONPATH=/root/ws/src/perception:$PYTHONPATH; python3 src/perception/test/benchmark_mcl.py --seeds 1 2 3 4 5 --scenario all'
```

Raw results: [mcl_validation_results.json](mcl_validation_results.json).

## Stationary simulator procedure — prepared, not executed

Installed app found at the following actual host path. Its Info.plist reports
0.3.0, which does not establish the AutoDRIVE release/scene. README specifies
2026-iros. Verify the build and selected track match test_track_v2 before proceeding.
There is another app copy under Downloads/autodrive_simulator; do not launch both.

Host terminal:

```bash
cd /Users/rohankhanna/roboracer
open '/Users/rohankhanna/Downloads/AutoDRIVE Simulator [macOS]/AutoDRIVE Simulator.app'
docker compose build
docker compose up -d
docker compose exec ros bash
```

Inside container, /root/ws:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 node list
ros2 launch autodrive_bridge autodrive_launch.py publish_restricted_topics:=false publish_camera:=false
```

Bridge launch includes fast_bridge, adapter, wheel_odom and foxglove_bridge.
Wheel odometry uses rear encoders and IMU gyro, publishes /odom and odom->base_link;
adapter provides scans, raw IMU and static sensor mounts. Do not launch a second
wheel_odom. Connect the host simulator to 127.0.0.1:4567 and select Autonomous
mode. Start with the car at rest. No teleop, race.sh, race.launch.py, gap/wall
follower, reset request or /drive command is part of this procedure.

Every additional container shell:

```bash
docker compose exec ros bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
```

The existing optional EKF pipeline command is:

```bash
ros2 launch perception ekf.launch.py
```

It publishes /odometry/filtered_accl and /odometry/corrected and has publish_tf=false.
It does not replace wheel /odom or feed MCL. Its default corrector subscribes to
privileged raw odometry solely for velocity-error diagnostics. With restricted
telemetry off that diagnostic receives no data. To explicitly disable even that
subscription, use these two commands in separate sourced shells **instead of**
ekf.launch.py:

```bash
ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_filter_node_accl --params-file /root/ws/install/perception/share/perception/config/ekf_config_accl.yaml -r odometry/filtered:=/odometry/filtered_accl
ros2 run perception corrector --ros-args -p publish_error:=false
```

For the first MCL test the optional EKF can be omitted. Keep its outputs separate
and do not remap /odom to /odometry/corrected.

Use test_track_v2: map.yaml, map.pgm, map.pbstream, raceline.csv, centerline.csv,
preview.png and trajectory.csv exist. No map_to_world.yaml was found there.
MCL does not require it or the pbstream. Other saved names are slam_toolbox_test
(native graph/map/trajectory, no raceline) and slam_toolbox_test_v2 (native graph,
map/raceline/trajectories). Prefer the Cartographer-origin test_track_v2 for the
first comparison; native SLAM map conventions need separate validation.

Run exactly one of these, stopping it with terminal Ctrl-C before switching:

```bash
ros2 launch perception mcl.launch.py track:=test_track_v2 adaptive_particles:=false particles:=1000
ros2 launch perception mcl.launch.py track:=test_track_v2 adaptive_particles:=true particles:=1000
```

Do not run Cartographer localization/mapping, SLAM Toolbox mapping, localization_gate,
another MCL, or race launch alongside this test. MCL alone owns map->odom;
wheel_odom alone owns odom->base_link. Verify publishers before each run:

```bash
ros2 node list
ros2 node info /mcl
ros2 topic info /odom --verbose
ros2 topic info /localized_pose --verbose
ros2 topic info /tf --verbose
ros2 topic info /drive --verbose
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
```

Multiple /tf publishers are normal for different edges; inspect nodes/child frames,
not just total publisher count. There should be no /drive publisher. MCL subscribes
to scan, wheel odom, initialpose, TF and sim_reset; it does not subscribe to ground
truth. Restricted telemetry off disables privileged automatic crash/teleport reset
events; explicit reset-command events still work, but no resets are requested here.
Existing localization_gate/map-origin and default-bridge review debt is not fixed.

## Foxglove and monitoring

Connect Foxglove to `ws://localhost:8765` (the bridge launch starts the server and
Compose exposes this port). Add a 3D panel, set its fixed/display frame to map,
and enable /map, /mcl/particles and /localized_pose. Display TF axes for map, odom,
base_link, laser and imu; optionally display /scan to inspect wall alignment. Add
Raw Messages panels for /localized_pose and /mcl/particles. The map is transient
local: select transient-local durability if a subscriber requires explicit QoS,
or reconnect after MCL starts. If a newly opened view misses the map, restart the
localizer only after stopping its previous instance. No world/ground_truth transform
is necessary for this view.

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /localized_pose
ros2 topic echo /localized_pose --once
ros2 topic echo /mcl/particles --once
ros2 param get /mcl adaptive_particles
ros2 param get /mcl particles
```

The particles parameter is the **initial/fixed setting**, not live adaptive N.
The displayed PoseArray is decimated (step=max(1,n//200)), may exceed 200 poses,
and is published only every fifth eligible update. At rest after convergence,
PF updates cease and the cloud can stop publishing while pose/TF continue on odom.
Neither cloud length nor topic rate proves the live count. Startup logs report
only initial N. A live-count diagnostic topic does not exist.

For exact live N without changing production code, the following is an alternative
instrumented process, **instead of either MCL launch**, in a sourced shell:

```bash
python3 - <<'MONITOR'
import os
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from perception.mcl_node import MclNode
config = os.path.join(get_package_share_directory('perception'), 'config', 'mcl.yaml')
rclpy.init(args=['--ros-args', '--params-file', config, '-p', 'track:=test_track_v2',
                 '-p', 'adaptive_particles:=true', '-p', 'particles:=1000', '-p', 'seed:=1'])
node = MclNode()
node.create_timer(1.0, lambda: node.get_logger().info(
    f'PF n={node.pf.n} ESS={node.pf.neff():.1f} updates={node.updates} localized={node.localized}'))
try:
    rclpy.spin(node)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    node.pool.shutdown(wait=False, cancel_futures=True)
    node.destroy_node()
    rclpy.try_shutdown()
MONITOR
```

Change adaptive_particles to false for the instrumented fixed run. This timer
adds overhead and is for diagnostics, not precision runtime benchmarking. It is
prepared code, not a verified live simulator run.

First stationary test: use the same scene and unchanged car pose for each mode.
Observe sensors first, then start fixed MCL, allow up to 60 seconds for search and
verification, log time to first /localized_pose, spread/convergence logs, map-scan
alignment, TF continuity, warnings, pose jitter and live count. Stop only MCL;
repeat adaptive with the car still at exactly the same pose. Do not send a guessed
simulator/world pose as /initialpose. An ambiguous track may legitimately remain
unresolved while stationary; do not force motion or relax thresholds to pass.
Check that published poses are finite, frame_id map, fresh and stationary; fixed
N must remain 1000 and adaptive N must remain in [200,2000]. Pass criterion is
correct map alignment and stable gated pose, not merely receiving messages. A
stationary test cannot establish moving accuracy, collision safety or lost recovery.

## Same-input replay comparison

No .mcap, .db3, .bag or metadata.yaml was found in the repository (including ignored
track data, excluding generated build/install directories; ws/log was also checked separately). A targeted search
of Downloads also found none. This is not a whole-disk inventory. No existing
recording is assumed.

For a future stationary recording, run **bridge only**, restricted telemetry off,
no localizer or mapper (so /tf contains no map->odom). In a sourced /root/ws shell:

```bash
mkdir -p log
ros2 bag record -o log/mcl_stationary_20261108 /scan /odom /sensors/imu/raw /tf /tf_static /autodrive/roboracer_1/sim_reset
```

Use a new output directory if that proposed recording name already exists. Record
60 seconds and Ctrl-C the recorder. Inspect `ros2 bag info log/mcl_stationary_20261108`
for topic coverage, counts, duration and storage plugin. Do not record /drive,
localized output, full raw privileged topics or a map->odom TF edge in the input
bag. /odom is already wheel/IMU based; replaying it does not need the EKF or raw
encoders. Verify /tf_static contains sensor mounts. Preserve any recorded bag.

Disconnect simulator and stop the live bridge/odometry/EKF. For each mode start a
fresh localizer with the same map, config and seed. Example fixed replay process:

```bash
ros2 run perception mcl_node --ros-args --params-file /root/ws/install/perception/share/perception/config/mcl.yaml -p track:=test_track_v2 -p particles:=1000 -p adaptive_particles:=false -p seed:=1 -p use_sim_time:=true
```

In another sourced shell, once the localizer is ready:

```bash
ros2 bag play log/mcl_stationary_20261108 --clock --rate 1.0
```

After playback, Ctrl-C the localizer. Run a fresh localizer with
adaptive_particles:=true and the identical replay command. Do not replay twice
into the same running filter or loop the bag. Live TF/sensor publishers must be
stopped. A bag clock here is **replay time**, not support for a live simulator clock.
If viewing replay in Foxglove, start only its bridge, rather than autodrive_launch:
`ros2 run foxglove_bridge foxglove_bridge`.

Repeat paired seeds, alternate order, warm the machine, record per-update N,
resamples, PF timing separately from search/ROS and time-to-localize. The diagnostic
wrapper above can also use use_sim_time:=true. Exact per-update timing/resample
telemetry for a live ROS node is not exposed yet; add scoped instrumentation in a
follow-up before claiming a ROS runtime comparison. Node recovery uses monotonic
wall time and asynchronous search, so even identical bag inputs/seed do not ensure
bit-identical node-level results. Keep playback rate 1 and record scheduling/search
outcomes. Offline PF benchmarking is more deterministic.

No absolute map-frame error can be measured from this bag alone. Use a separate
reference, independent of localization inputs. Ground truth may be evaluated only
in an isolated tool; test_track_v2 has no saved map-to-world alignment. Do not
invent one, feed simulator poses into /initialpose, or enable ground-truth-derived
reset hints. Map/scan agreement and stationary jitter are observables, not ground-
truth accuracy. No simulator movement recording is authorized in this task.

## Safe shutdown and remaining driving blockers

Ctrl-C recording/monitoring commands, then the MCL terminal, optional EKF/corrector
terminals, and disconnect the simulator **before** Ctrl-C in the bridge terminal.
For detached launches send SIGINT to the verified process group, not SIGTERM or
SIGINT to the launch PID alone. Check `ros2 node list` until fast_bridge, adapter,
wheel_odom, MCL and optional EKF nodes are gone. Never restart into stale instances.
Quit the host simulator normally. If stopping the container is desired, after
node shutdown run `docker compose stop ros` on the host. Do not use volume deletion
or clean generated tracks. No movement is commanded, so a drive stop publisher is
not part of this stationary procedure.

Driving blockers: resolve/document sampler statistical choice; validate simulator
build/scene and saved-map match; execute stationary localization and same-input
replay; provide actual count/timing telemetry for node comparisons; test reduced-
population recovery, latency tails, timestamp gaps and input failure behavior;
confirm TF and /drive ownership; arrange an independent evaluation reference if
absolute simulator errors are needed. Existing reset/ground-truth policy debt and
controller collision-protection limitations remain. Driving and autonomous racing
are outside this prepared procedure.

## Regression verification

After benchmarks completed, the following ran in a fresh temporary ROS image
container (with no live simulator):

```bash
docker compose run --rm --no-deps ros bash -lc 'source /opt/ros/humble/setup.bash; export PYTHONPATH=/root/ws/src/perception:$PYTHONPATH; ROS_DOMAIN_ID=42 python3 -m pytest src/perception/test/test_mcl.py src/perception/test/test_mcl_node.py -q'
```

Result: **148 passed in 95.58 seconds**, no skips. Host benchmark compilation and
final `git diff --check` passed. Startup, Foxglove, bag capture/replay and the live
monitor wrapper are prepared instructions, not executed integration validation.
