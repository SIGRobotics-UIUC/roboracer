# Unseen-track pipeline

The stack is built to race a track it has never seen. Nothing track-specific is committed:
every map and raceline is generated on the day, into `ws/tracks/<name>/` (git-ignored).

```
new track ─► Cartographer mapping ─► centerline ─► TUM mintime raceline ─► localization + pure pursuit
             (keyboard teleop or
              wall follower lap)
```

## Commands

Inside the container (`docker compose exec ros bash`, working directory `~/ws`), with the simulator connected:

```bash
ros2 launch autodrive_bridge autodrive_launch.py     # shell 1: bridge (always running)          alias: sim
scripts/map_track.sh <name>                         # shell 2: map -> save -> centerline -> mintime   map-track
scripts/map_track.sh <name> --wall-follow           #   ... or let the wall follower drive 2 laps (--laps N), the default
scripts/map_track.sh <name> --gap-follow            #   ... or follow the gap (experimental: planning can fail, control/README.md)
ros2 run autodrive_bridge teleop_keyboard           # shell 3: drive the mapping lap(s)            kb
scripts/race.sh <name> [speed_scale:=0.5]           # localization + pure pursuit                  race
scripts/race.sh <name> avoid:=true                  #   ... passing obstacles the map did not have
ros2 launch control follow_the_gap.launch.py [speed:=1.0]   # no-map fallback, instead of race.sh   ftg
```

Restart the bridge with Ctrl-C in shell 1 (SIGINT to the whole process group; or `kill -INT -- -<pgid>`),
then wait until `ros2 node list` is empty of its nodes. A SIGINT to the launch process alone
(`pkill -INT -f "ros2 launch autodrive_bridge"`) reaches no node: they run on ~5 s until launch sends
SIGTERM. A bridge relaunched in that window stops once `fast_bridge` fails to bind :4567
(`autodrive_launch.py` shuts down with it). A second `wheel_odom` / `adapter` holds all its
output while the first runs, and for 2 s at every start (`/odom` + TF odom -> base_link; `/scan`,
`/sensors/imu/raw`, ground truth and the commands), so Cartographer never gets
duplicate `/odom` or IMU stamps. After a SIGKILL or crash the dead node stays in the graph until its DDS
lease expires (~20 s): a relaunch holds that long, logs it and recovers. Two copies started together
outside one launch hold each other until one is stopped.

`map_track.sh` resets the sim car, starts Cartographer mapping, waits for you to finish driving and
press Enter (with `--wall-follow` or `--gap-follow`: starts `control wall_follow.launch.py` or `disparity_gap.launch.py` and waits for its laps), runs `save_map`, stops mapping, then runs `ros2 run planning build_raceline <name>`.
It needs a terminal (it refuses to start without one) and exits early if `cartographer_node` dies on startup.
If `save_map` fails, mapping keeps running and it asks to retry (Enter) or quit (`q`, the map is lost; delete `tracks/<name>` before reusing the name).
`build_raceline` never leaves a mapped track without a raceline: if TUM mintime fails it falls back to
mintime without `safe_traj` (speeds then held to its limits), then the centerline (with a speed profile within
those limits, laterally at most `mue * g`). Anything but a clean mintime line ends in a WARNING block and exit
status 2 (1: no raceline at all), which `map_track.sh` repeats. `race.sh` prints which line the car drives;
`pure_pursuit_node` logs it and warns unless it is a clean mintime line (also latched on `/raceline/provenance`).
`race.sh` resets the sim car (localization starts at the map origin = where mapping started) and
launches `control race.launch.py` (perception `localization.launch.py` + control `pure_pursuit.launch.py`).
With `avoid:=true` it also starts planning's `obstacle_spliner_node`, and pure pursuit steers on its
`/local_path` (the raceline shifted sideways around an obstacle on `/scan` that the map did not have)
instead of the raceline, at the raceline's speeds. Each scan is placed with the pose at its own stamp
(interpolated; a scan newer than the poses waits for them, at most 0.2 s) and aligned to the map's walls
before anything counts as an obstacle, so neither the pose running ahead of the scan in a hairpin nor
~0.15 m / 2 deg of localization error turns a wall into a roadblock. The alignment moves the scan at most
`max_align_shift` (0.3 m) / `max_align_turn` (6 deg), leaves out returns short of the map's own ray (an
obstacle hiding a wall), and its correction is carried from tick to tick across the localizer's changes of
TF `map -> odom` (it jumps only when that fits clearly better). With no fresh collision-free path the car stops.
With `adaptive_lookahead:=true` (the default) pure pursuit's lookahead is also capped by the raceline's curvature (so it
cuts curves by at most about `adaptive_lookahead_cut` m), which is what makes `speed_scale:=1.0` drive
without hitting the inside walls (control/README.md).
If mapping fails on race day, `follow_the_gap.launch.py` drives on `/scan` alone (no map); run it instead of
`race.sh`, never alongside it, since both publish `/drive`.

### Stopping a launch

Stop `ros2 launch` with Ctrl-C, or from a script with SIGINT to its whole process group:
`kill -INT -- -<pgid>` (`map_track.sh` does this). SIGTERM to the `ros2 launch` process alone
(e.g. `timeout ros2 launch ...`, `kill <pid>`) exits the launch but leaves every node it started running.
Check with `ros2 node list` and kill leftovers before the next launch.

## Packages (`ws/src/`)

| Package | Contents |
|---|---|
| `autodrive_bridge` | `fast_bridge`, `adapter`, `wheel_odom`, `teleop_keyboard`; `autodrive_launch.py`. Sim only: replaced by f1tenth_system on the car. |
| `perception` | Cartographer configs, `mapping.launch.py`, `localization.launch.py`, `save_map`, `map_origin`. Particle filter localization instead of Cartographer's: `mcl.launch.py` (`mcl_node`, `mcl.py`, `config/mcl.yaml`). Experimental, not in the pipeline: `ekf.launch.py` (robot_localization IMU EKF + `corrector`, a drift-corrected forward speed on `/odometry/corrected`, sim only) |
| `planning` | `build_raceline` = `centerline` extraction + TUM `mintime` optimization. `obstacle_spliner_node`: stationary-obstacle avoidance, `/local_path` (`race.launch.py avoid:=true`) |
| `control` | `pure_pursuit_node`, `pure_pursuit.launch.py`, `race.launch.py`. `follow_the_gap_node` (`follow_the_gap.launch.py`): reactive `/scan` -> `/drive`, needs no map, the fallback. `wall_follow_node` (`wall_follow.launch.py`): drives the mapping laps (`map_track.sh --wall-follow`), the default. `disparity_gap_node` (`disparity_gap.launch.py`): follow the gap with disparity extension, also drives the mapping laps (`map_track.sh --gap-follow`, experimental: see control/README.md) |

## Interfaces

| Topic / frame | Type | Producer |
|---|---|---|
| `/scan` (frame `laser`) | `sensor_msgs/LaserScan` | adapter (sim) / urg_node (car) |
| `/odom`, TF `odom -> base_link` | `nav_msgs/Odometry` | wheel_odom (sim) / vesc_to_odom (car) |
| `/sensors/imu/raw` (frame `imu`) | `sensor_msgs/Imu`, accel includes gravity; Cartographer uses it | adapter (sim) / vesc (car) |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | teleop_keyboard, wall_follow or disparity_gap (mapping), pure_pursuit, follow_the_gap |
| TF `map -> odom` | | Cartographer (mapping and localization), or `mcl_node` with `localizer:=mcl` |
| `/tracked_pose` | `geometry_msgs/PoseStamped`, frame `map` | Cartographer (`publish_tracked_pose`) |
| `/localized_pose` | `geometry_msgs/PoseStamped`, frame `map` | localization_gate (or `mcl_node`): `/tracked_pose` (or the filter pose) relayed once localized; withheld after a sim reset (`sim_reset`) until relocalized (`mcl_node`: also while it searches for the car when lost) (pure pursuit's default `pose_topic`; with none for `max_pose_age` s it stops the car) |
| `/mcl/diagnostics` | `diagnostic_msgs/DiagnosticArray` | MCL: current population, ESS, localization flag, cloud spread, resampling count and latest PF timing, at 1 Hz even while stationary; see [MCL diagnostics](mcl_diagnostics.md) |
| `/autodrive/roboracer_1/sim_reset` | `std_msgs/String`, the reason | fast_bridge (sim only): one event per sim reset. Reset commands always; respawns and other teleports (collision count change, position jump) only with restricted topics. fast_bridge also replies zeros for `reset_frames` frames |
| `/local_path` | `nav_msgs/Path`, frame `map`, one pose per raceline point | obstacle_spliner_node: the raceline, shifted around the nearest obstacle in the way; pure_pursuit follows it with `avoid:=true` |
| `/local_path/valid` | `std_msgs/Bool` | obstacle_spliner_node: false when its pose or scan is stale (or no pose covers a scan's stamp for `max_scan_age`) or no side is collision-free (pure_pursuit then stops) |
| `/ground_truth/pose`, TF `world -> ground_truth` | `geometry_msgs/PoseStamped`, frame `world` (sim) | adapter, debugging only |

Static TF `base_link -> laser, imu, front_camera` comes from the adapter. `base_link` is the rear axle.
Cartographer runs with `use_imu_data = true`, which requires `imu` at the origin of `base_link`
(rotation allowed). On the car, publish `base_link -> imu` with zero translation and the VESC's
mounting rotation, or Cartographer stops waiting for an IMU it cannot transform.

## Track directory `ws/tracks/<name>/`

| File | Written by | Format |
|---|---|---|
| `map.pbstream` | `save_map` | Cartographer state, loaded by `localization.launch.py` |
| `map.pgm`, `map.yaml` | `save_map` | ROS map_server occupancy grid, `map` frame |
| `trajectory.csv` | `save_map` | `# t,x_m,y_m,yaw_rad`: the mapping lap's optimized poses (`/trajectory_query`), gives driving direction |
| `map_to_world.yaml` | `save_map` | ground-truth pose of the `map` origin in the sim world frame (debugging) |
| `centerline.csv` | `build_raceline` | TUM input: `# x_m,y_m,w_tr_right_m,w_tr_left_m`, closed loop, driving direction |
| `raceline.csv` | `build_raceline` | TUM output: `# s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2`, `map` frame. Second header line `# raceline: {json}`: method (`mintime`, `mintime-retry`, `centerline`), `clean`, settings `changed`, `notes` (failures, clearance), solver, lap, min wall clearance, build time |
| `preview.png` | `build_raceline` | map, centerline, track bounds, raceline coloured by speed |
| `runs/<time>.csv` | `lap_logger_node` (`race.launch.py`) | one row per lap of a race run: time, status, speeds, offset from the raceline, collisions; first line `# run: {json}` with the race settings. Compare runs: `ros2 run control lap_report <name>` |

## Car parameters (AutoDRIVE RoboRacer)

| | |
|---|---|
| wheelbase, max steering | `ws/src/control/config/vehicle.yaml` (0.33 m, 0.524 rad), loaded by every node that uses them |
| wheel radius | 0.0585 m (calibrated against ground truth) |
| throttle | motor torque (sim VehicleController; ~25 m/s per unit throttle is only its steady state), speed loop in `adapter.yaml` |
| acceleration limit | 2.5 m/s^2 up, 6.9 m/s^2 down (`adapter.yaml` `max_acceleration`, `max_deceleration`): the adapter ramps the /drive speed as the car's speed controller would, so a launch from a standstill does not spin the wheels; after a /drive gap the ramp restarts from the measured speed |

## Optional SLAM Toolbox mapping (Milestone 1)

Cartographer remains the default. The opt-in asynchronous backend uses the same
`/scan` and wheel-odometry TF (`odom -> base_link`); SLAM Toolbox alone owns
`map -> odom`. It does not subscribe to privileged simulator pose/IPS/odometry,
and does not start `map_origin`. `use_sim_time` is false because the bridge stamps
sensor frames with the arrival clock. Resolution remains 0.05 m; ranges are
0.15–9.5 m, matching the Cartographer configuration.

After rebuilding the Docker image and workspace:

```bash
# Repository root (does not start/restart your running bridge):
docker compose build
# When ready to replace the container: disconnect the simulator and stop the old
# bridge cleanly with Ctrl-C first; then recreate the container.
docker compose up -d --force-recreate
# Enter a container shell:
docker compose exec ros bash
colcon build --symlink-install
source install/setup.bash
ros2 launch autodrive_bridge autodrive_launch.py publish_restricted_topics:=false
```

Start AutoDRIVE manually on the host, select the track, connect to
`127.0.0.1:4567`, and select Autonomous mode. In another sourced container shell,
with no mapper/localizer/controller already running:

```bash
scripts/map_track.sh slam_toolbox_test_v2 --slam-toolbox --wall-follow --laps 2 speed:=0.5
```

Use a **new name** each time: the script refuses any existing track directory,
including partial/validated maps from either backend. Stop the car before saving;
inspect `tracks/slam_toolbox_test_v2/preview.png` before considering a race. As with
Cartographer, planning can exit 2 with a fallback line or 1 on failure; neither
means a clean minimum-time raceline. Mapper failure during automatic laps aborts
and stops the follower. Mapping logs stay in `ws/log/`.

For manual driving or debugging the individual components:

```bash
ros2 launch perception slam_mapping.launch.py track:=slam_toolbox_manual
# Other shell: drive 1–2 clean laps, then stop:
ros2 run autodrive_bridge teleop_keyboard
# With mapping still running and the car stopped:
ros2 run perception save_slam_map slam_toolbox_manual
# Stop mapping before starting another map/localizer:
ros2 run planning build_raceline slam_toolbox_manual
```

`slam_mapping.launch.py` starts `async_slam_toolbox_node` with
`perception/config/slam_mapping.yaml`. Native `/pose` is remapped to
`/mapping/slam_pose` (`PoseWithCovarianceStamped`). `slam_mapping_pose` validates
and relays it as `/tracked_pose` for the unchanged mapping followers and publishes
a transient-local `/mapping/trajectory` (`nav_msgs/Path`) snapshot every 0.5 s.
It broadcasts no TF and publishes no racing `/localized_pose`. `/mapping/diagnostics` reports scan/pose counts, input timestamp gaps and observed processing ages for future throughput investigations.

`save_slam_map` waits for a periodic map after startup, stages outputs in a private
`.name-saving-*` directory under `TRACKS_DIR`, validates them, and publishes a new
track directory only after all steps succeed. Failed staging directories are
retained for diagnosis/late service writes; retries use fresh staging paths. No
existing destination is overwritten. Output is:

| File | Meaning |
|---|---|
| `map.pgm`, `map.yaml` | Existing trinary encoding/thresholds (free 254, unknown 205, occupied 0; YAML 0.196/0.65), map-frame origin and 0.05 m resolution |
| `map.posegraph`, `map.data` | Native SLAM Toolbox serialization pair; load using prefix `map`, not either extension |
| `trajectory.csv` | `# t,x_m,y_m,yaw_rad`, corrected graph poses sorted by original scan time |
| `trajectory_online.csv` | Historical live pose stream, retained for correction diagnostics |
| `odometry.csv`, `graph_metadata.json` | Graph-stored sensor odometry and native sensor/encoding metadata |
| `mapping_diagnostics.json` | Occupancy fractions, discontinuities, online-to-final corrections, planner geometry result and available runtime counters |
| `mapping.json` | Backend, graph prefix, and snapshot provenance |
| `centerline.csv`, `raceline.csv`, `preview.png` | Unchanged planner outputs after `build_raceline` |

The graph pair and service API were checked against installed Humble SLAM Toolbox
2.6.10: `/slam_toolbox/serialize_map` is `slam_toolbox/srv/SerializePoseGraph`,
request `string filename`, success `result == RESULT_SUCCESS (0)`. The prefix
produces `.posegraph` and `.data` (installed `serialization.hpp`). Its native
`/slam_toolbox/save_map` service uses `slam_toolbox/srv/SaveMap` with a nested
`std_msgs/String name`; this integration instead rasterizes the serialized final graph to preserve the repository's established unknown-cell thresholds and make the map and trajectory refer to the same snapshot.
Upstream interface references: [serialization](https://github.com/SteveMacenski/slam_toolbox/blob/humble/include/slam_toolbox/serialization.hpp)
and [ROS interfaces](https://github.com/SteveMacenski/slam_toolbox/blob/humble/src/slam_toolbox_common.cpp).

**Snapshot export:** build `slam_graph_tools` before using the saver (full `colcon
build --symlink-install`, or `--packages-up-to perception planning`). Its native
`export_slam_graph` executable reads a trusted same-version graph pair using the
installed SDK's `serialization::read` and `Mapper::GetAllProcessedScans`, then
exports `GetCorrectedPose()` and `GetTime()`. It does not rerun optimization, use
visualization marker ordering, query simulator poses, or repair/snap trajectories.
Both the occupancy raster and chronological trajectory come from that snapshot;
the live map is only a readiness check. Failure to export fails saving; there is
no fallback to historical poses. Export requires a NEW output directory.

ROS services in this Humble version do not expose a timestamped final trajectory.
The public native SDK is therefore used in a small separate `ament_cmake`
package; the four autonomy/bridge packages remain Python. Native binary archives
are version-dependent and must be trusted. Export defaults are deliberately the
same as `slam_mapping.yaml`: 0.05 m, min_pass_through 2, occupancy_threshold 0.1.
Change and test both together if those raster parameters ever change.

**Reliability experiment:** the failed first run had metre-scale graph corrections
and a final geometry still rejected by the planner. SLAM mapping followers now
default to 0.5 m/s (explicit `speed:=` still wins); Cartographer defaults are
unchanged. Loop candidate radius is reduced from 3 to 1 m and coarse/fine response
thresholds raised from 0.35/0.45 to 0.55/0.65. These are conservative experimental
settings to reduce ambiguous cross-corridor associations, not a demonstrated fix.
They can also miss real closures when odometry drifts; compare new reports rather
than weakening the planner. Scan rate/queue and calibration remain unchanged:
four queue-full warnings alone do not prove sustained overload. The installed
Ceres solver hardcodes 50 threads and caps them internally; no invented ROS
thread parameter is added. See [failure analysis](slam_toolbox_failure.md).

Existing planner geometry checks remain authoritative. A map whose final poses
mostly lie in free cells can still contain collapsed loops/false shortcuts.
Keep the car stopped when saving and inspect diagnostics and preview. No real
AutoDRIVE remapping with these revised settings has yet been validated.

This milestone adds mapping only. There is no `map.pbstream`, so Cartographer
localization and `race.sh` (which still checks for a pbstream even with MCL) cannot
consume this track. Do not rename serialized files to fake a pbstream. For later
MCL experiments, stop mapping, reset the car to its mapping start, and explicitly
launch `ros2 launch control race.launch.py track:=slam_toolbox_test_v2 localizer:=mcl
speed_scale:=0.3`; localization/control/racing algorithms are unchanged.

Tests inside the built/sourced container:

```bash
ROS_DOMAIN_ID=65 python3 -m pytest src/perception/test/test_slam_mapping.py src/perception/test/test_save_map.py src/planning/test/test_centerline.py -q
python3 -m pytest src/perception/test/test_slam_mapping_integration.py -q -s
```

The integration test starts a real async mapper on isolated domain 66, feeds
synthetic room LiDAR plus odometry/TF, serializes a native graph, and inspects its
temporary occupancy image. It does not publish `/drive` or connect to AutoDRIVE.
It establishes interface/artifact plumbing, not successful real-track mapping,
loop closure, follower lap completion, or a valid optimized racing line.

### Read-only mapping diagnostics

Inside the sourced container:

```bash
ros2 run perception diagnose_mapping tracks/slam_toolbox_test --export-graph \
  --mapping-log log/map_track_slam_toolbox_test.log \
  --follower-log log/map_track_slam_toolbox_test_wall_follow.log
# Repeat after the controlled experiment, using the NEW track and its logs:
ros2 run perception diagnose_mapping tracks/slam_toolbox_test_v2 --export-graph \
  --mapping-log log/map_track_slam_toolbox_test_v2.log \
  --follower-log log/map_track_slam_toolbox_test_v2_wall_follow.log
```

A unique `ws/log/slam_diagnostics/diagnosis-*` directory receives JSON and text
reports plus the read-only graph export. Existing track files and logs are never
rewritten. Free/occupied/unknown/outside counts use map origin, rotation, image
row orientation and resolution; outside points are not clipped onto map edges.
Gross diagnostic flags (>0.5 m, >3 m/s or >0.5 rad between samples) are separate
from unchanged planner validation. Corrections match acquisition timestamps,
not CSV row numbers. Missing logs or runtime counters are unknown, not zero.
A matching graph raster tests serialization; a failed final-path planner test
shows that merely replacing online poses is insufficient. No geometry is carved,
snapped to free space, deleted or relaxed to obtain a raceline.

Additional regression checks:

```bash
ROS_DOMAIN_ID=65 python3 -m pytest src/perception/test/test_mapping_diagnostics.py src/perception/test/test_slam_mapping.py src/perception/test/test_slam_mapping_integration.py src/perception/test/test_save_map.py src/planning/test/test_centerline.py -q
```
