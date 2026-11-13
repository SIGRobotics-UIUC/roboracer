# MCL diagnostics

The MCL node publishes `/mcl/diagnostics` as
`diagnostic_msgs/msg/DiagnosticArray`, with one status named `mcl/localization`.
It runs automatically in fixed and adaptive modes. No simulator ground truth is
used, and no diagnostic value changes the localization estimate or gating.

From a built and sourced ROS workspace:

```bash
ros2 topic echo /mcl/diagnostics
ros2 topic hz /mcl/diagnostics
```

Rebuild/source perception after updating its package dependency:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select perception
source install/setup.bash
```

The publisher uses reliable, volatile QoS with depth 1. A steady-clock timer
publishes approximately once per second, including without new scans, while
stationary and when replay ROS time is paused. Long node callbacks can delay
publication. The message header uses ROS time, so it can remain unchanged while
ROS time is paused. Status level is OK when `localized` is true and WARN otherwise;
this mirrors the existing declaration, not an independent accuracy or sensor-health
check.

All measurements are `DiagnosticStatus.values` key/value strings:

| Key | Meaning |
|---|---|
| `particle_count` | Actual current `pf.n`, including before initialization; not the decimated visualization count or configured initial population. |
| `effective_sample_size` | N_eff from the current weights at publication time; after resampling this is approximately N. |
| `localized` | Existing localization declaration, `true` or `false`. |
| `position_std_m` | Existing weighted cloud spread: sqrt(E[(x-mean_x)^2 + (y-mean_y)^2]), in metres; not a per-axis deviation. |
| `yaw_std_rad` | Existing circular yaw standard deviation, in radians. |
| `resampling_count` | Number of actual resampling operations performed by node PF updates since startup. Skipped ESS resampling does not increment it. Recovery/reset does not clear it. |
| `pf_update_duration_ms` | Most recent completed PF update duration in milliseconds. Includes prediction when required, sensor weighting, resampling, estimate/map-to-odom calculation and spread. Excludes scan selection, odometry interpolation, scan matching, convergence handling, TF publication and visualization. |
| `initialized` | Whether a pose cloud has been initialized, `true` or `false`; independent of successful localization. |
| `pf_update_age_sec` | Steady elapsed seconds since the last completed PF update. |

Unavailable statistics are the literal string `unavailable`, never a substitute
zero. Before cloud initialization, N_eff and both standard deviations are
unavailable: the constructor's placeholder cloud is not a pose measurement.
Duration and age remain unavailable until the first completed PF update. A
Gaussian initialization can therefore have available cloud statistics but no
update duration. Nonfinite cloud statistics are also unavailable. Genuine numeric
zeros remain numeric strings such as `0.0`. Counts and flags are available from
startup.

Cloud statistics are recomputed from the **current retained cloud** at publication
time, including after a re-anchor or recovery population change. After loss/reset
they may describe an old retained cloud; they do not imply that its pose is valid.
Check `localized` alongside them. A localized stationary car normally skips PF
updates: diagnostics continue, retain the last duration, and show increasing update
age. This is expected motion gating, not proof of missing sensors. Conversely,
low spread and a true localization flag do not establish correct map alignment.

Timing uses two `perf_counter` reads per eligible PF update. Resampling uses the
existing return value for its lifetime counter. Array checks, ESS/spread calculations,
message construction and publication occur at 1 Hz, outside the timed PF work.
There is no per-particle logging, extra scan matching, or new diagnostic parameter.
The motion/sensor models, KLD sampling, population/recovery parameters, convergence
thresholds, TF and existing topics are unchanged.

The reported stable simulator runs at bounds 300–2000 and 500–2000, and the earlier
failure at 200–2000, do not establish the cause of that failure. Diagnostics enable
recording these observations alongside sensor/pose data. The experimental ROS
default is now 300–2000, reflecting the validated configuration; the cause of the
earlier 200–2000 failure remains unestablished. Other filter settings are unchanged.

## Verification

The complete MCL and node suites passed in the existing ROS Humble Docker image:

```bash
docker compose run --rm --no-deps ros bash -lc 'source /opt/ros/humble/setup.bash; export PYTHONPATH=/root/ws/src/perception:$PYTHONPATH; ROS_DOMAIN_ID=42 python3 -m pytest src/perception/test/test_mcl.py src/perception/test/test_mcl_node.py -q'
```

Result: **183 passed in 102.88 seconds**, no skips. Diagnostics regressions cover
fixed/adaptive values, unavailable versus zero measurements, nonuniform weights,
invalid numeric statistics, current counts after recovery, lifetime resampling
counts, skipped updates/resampling, read-only behavior (including RNG state), and
real ROS publication while stationary with a paused ROS clock. Existing filter,
recovery and frame-jump regressions passed. Python compilation and
`git diff --check` also passed. No live simulator validation was performed.

The targeted build `colcon build --symlink-install --packages-select perception`
also passed in the ROS container (one package finished).
