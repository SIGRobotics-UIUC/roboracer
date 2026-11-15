# V1 implementation validation

Validated in the existing ROS 2 Humble container. All generated test data was
written to temporary directories; validated tracks and existing run artifacts
were not modified. Runtime code neither imports nor invokes `ws/scripts/`.

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

No AutoDRIVE experiment, lap, collision scenario, pause/disconnect test or
physical stopping measurement was performed. Stop confirmation currently means
fresh wheel odometry at rest plus gate inhibition, not independently established
physical rest. Validate the provisional thresholds on each supported stack.

The current `/clock` is the existing host-monotonic transport clock, not physics
simulation time. There is no continuous native physics-time loss measurement,
cross-Windows/container clock synchronization, callback/solver instrumentation,
or command application endpoint correlation in v1. Reports carry explicit
unavailable metrics and the evidence needed for v2. Control smoothness, tire
dynamics, online-planner adapters and MPC/MPCC adapters remain outside this v1.
