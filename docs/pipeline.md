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

## Car parameters (AutoDRIVE RoboRacer)

| | |
|---|---|
| wheelbase, max steering | `ws/src/control/config/vehicle.yaml` (0.33 m, 0.524 rad), loaded by every node that uses them |
| wheel radius | 0.0585 m (calibrated against ground truth) |
| throttle | motor torque (sim VehicleController; ~25 m/s per unit throttle is only its steady state), speed loop in `adapter.yaml` |
| acceleration limit | 2.5 m/s^2 up, 6.9 m/s^2 down (`adapter.yaml` `max_acceleration`, `max_deceleration`): the adapter ramps the /drive speed as the car's speed controller would, so a launch from a standstill does not spin the wheels; after a /drive gap the ramp restarts from the measured speed |
