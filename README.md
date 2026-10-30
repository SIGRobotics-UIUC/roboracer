# SIGRobotics RoboRacer Stack

![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)
![Docker](https://img.shields.io/badge/Docker-amd64%20%7C%20arm64-2496ED?logo=docker&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)
![Simulator](https://img.shields.io/badge/AutoDRIVE-RoboRacer%202026--iros-blue)

Autonomous racing stack of [SIGRobotics](https://github.com/SIGRobotics-UIUC) at the University of
Illinois Urbana-Champaign, for the RoboRacer (F1TENTH) 1:10 scale car and the
[AutoDRIVE RoboRacer simulator](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing).

The stack is built to race a track it has never seen. On the day it maps the track with Cartographer,
extracts the centerline, plans a minimum-time raceline with the TUM optimizer, and races it with
pure pursuit, localizing on the saved map. Nothing track-specific is committed: every map and
raceline is generated into `ws/tracks/<name>/`.

```
new track ─► Cartographer mapping ─► centerline ─► TUM minimum-time ─► Cartographer localization
             (keyboard or wall-follower laps)        raceline              + pure pursuit (+ obstacle avoidance)
```

> [!NOTE]
> - The stack has been developed and tested in the AutoDRIVE simulator (macOS practice build, release
>   `2026-iros`). Its topics follow [f1tenth_system](https://github.com/f1tenth/f1tenth_system) so the
>   same perception, planning and control run on the car, but it **has not yet been run on the real car**.
> - Racing has been tested up to `speed_scale` 0.5 (half the planned speed profile), and in the sim at 1.0 on
>   Porto, Berlin and IROS 2026 with the adaptive lookahead (on by default; without it pure pursuit cuts into the inside walls).
>   The braking gain and the tyre model in `racecar.ini` are not yet tuned for full speed.

## Features

| Module | Package | What it does |
|---|---|---|
| **Simulator bridge** | `autodrive_bridge` | Socket.IO bridge to AutoDRIVE (`fast_bridge`), one sensor frame per sim frame; publishes `/scan`, `/sensors/imu/raw`, `/odom` (wheel odometry) and sim ground truth under the f1tenth_system names. A VESC stand-in (`adapter`) runs a speed loop on `/drive` with an acceleration ramp, as f1tenth_system does on the car. Detects sim resets and respawns, stops the car on them and announces them on `sim_reset`. Keyboard teleop. |
| **Mapping** | `perception` | Cartographer 2D SLAM with LiDAR, wheel odometry and IMU. `save_map` waits for the final optimization and writes the pbstream, occupancy grid and the mapping lap's trajectory. |
| **Localization** | `perception` | Cartographer pure localization on the saved map. `localization_gate` lets the car drive only once Cartographer has localized, and relocalizes it after every sim reset. Alternative: a Monte Carlo localization particle filter (`mcl_node`, `localizer:=mcl`) on the saved `map.yaml` that finds the car by itself (global scan matching) and re-anchors after odometry jumps. |
| **Planning** | `planning` | Centerline and track widths extracted from the occupancy grid, checked against the mapping lap; [TUM global race trajectory optimization](https://github.com/TUMFTM/global_racetrajectory_optimization) (`mintime`) with a RoboRacer vehicle model; if it fails, a loudly reported fallback line (mintime without the acceleration limits, then the centerline; exit status 2, recorded in `raceline.csv`'s header); a preview image of the result. Stationary-obstacle avoidance (`obstacle_spliner_node`): the raceline shifted sideways around obstacles the map did not have. |
| **Control** | `control` | Pure pursuit on the raceline with a speed-dependent lookahead and safety stops on bad poses. With `avoid:=true`, it follows the obstacle-avoiding path instead, and stops when there is no collision-free one. Follow the gap as a no-map fallback. A centring wall follower (the default) and an experimental follow the gap with disparity extension, either of which drives the mapping laps by itself. |
| **Tooling** | `ws/scripts` | `map_track.sh` (map → save → centerline → raceline in one command) and `race.sh`; shell aliases; Foxglove bridge; 132 unit and integration tests. |

## Repository layout

```
Dockerfile, docker-compose.yml   ROS 2 Humble environment (Linux, macOS or Windows/WSL2; amd64 or arm64)
.bash_aliases                    shortcuts inside the container (sim, kb, map-track, race, ftg, plan, ...)
ws/src/autodrive_bridge/         AutoDRIVE <-> ROS 2 bridge, VESC stand-in, wheel odometry, teleop (sim only)
ws/src/perception/               Cartographer configs and launches, save_map, localization_gate
ws/src/planning/                 centerline extraction, TUM minimum-time raceline, racecar.ini, obstacle spliner
ws/src/control/                  pure pursuit, follow the gap, wall follower, disparity gap follower, vehicle.yaml, race.launch.py
ws/scripts/                      map_track.sh, race.sh, common.sh
ws/tracks/<name>/                generated per track (git-ignored)
docs/pipeline.md                 interfaces, frames, track file formats and car parameters
```

## Installation

1. Install [Docker](https://docs.docker.com/get-docker/).
2. Download the AutoDRIVE RoboRacer simulator for your OS from release
   [`2026-iros`](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing/releases/tag/2026-iros).
   The simulator runs natively on the host and connects to the bridge in the container on port 4567.
3. Clone and build the container:

```bash
git clone https://github.com/SIGRobotics-UIUC/roboracer.git && cd roboracer
docker compose build
```

## Getting started

Start the container and the bridge:

```bash
docker compose up -d && docker compose exec ros bash
colcon build --symlink-install && source install/setup.bash     # inside the container
ros2 launch autodrive_bridge autodrive_launch.py                # alias: sim
```

Start the simulator, **Connect** to `127.0.0.1:4567` and set **Driving Mode** to **Autonomous**.
With the stock simulator, restarting the bridge while it is connected makes it reconnect in a loop,
so **Disconnect** first.

To restart the bridge, stop it with Ctrl-C in its terminal (or `kill -INT -- -<pgid>` for the launch's
whole process group) and wait until `ros2 node list` no longer shows `fast_bridge`, `adapter` or
`wheel_odom` before launching it again. `pkill -INT -f "ros2 launch autodrive_bridge"` signals only the
launch process: ros2 launch forwards nothing on that SIGINT and its nodes keep running ~5 s, until it
sends them SIGTERM. A new bridge started meanwhile ends as soon as its `fast_bridge` fails to bind :4567
(the launch shuts down with it). Until then, and whenever a second copy runs, the newer `wheel_odom`
or `adapter` holds all its output (2 s at every start, then while another copy is seen):
`/odom` + TF odom -> base_link, `/scan`, `/sensors/imu/raw`, ground truth, the commands. So
Cartographer never gets duplicate `/odom` or IMU stamps from two copies (it aborts on them). The nodes
also exit when their launch process dies.

A node that was SIGKILLed or crashed stays visible to the others until its DDS lease runs out (~20 s by
default), so a bridge started right after that holds its output that long; it logs the wait and starts on
its own. Two copies started together outside one launch (two `ros2 run`, or two launches on different
`port:=`) each see the other and both hold until one is stopped.

### Race an unseen track

In new container shells (`docker compose exec ros bash`), with the bridge running:

```bash
scripts/map_track.sh <name> --wall-follow    # map the track: the wall follower drives 2 laps (--laps N), then
                                             # the map is saved and the centerline and raceline are planned
                                             # (--gap-follow: follow the gap drives them; experimental, see ws/src/control/README.md)
scripts/race.sh <name> speed_scale:=0.5      # reset the car, localize on the saved map, pure pursuit on the raceline
scripts/race.sh <name> avoid:=true           #   ... and pass obstacles the map did not have
```

To drive the mapping laps by hand instead, run `scripts/map_track.sh <name>`, drive 1-2 clean laps with
`ros2 run autodrive_bridge teleop_keyboard` (W/S speed, A/D steer, X stop) in another shell, stop the
car and press Enter.

Check `tracks/<name>/preview.png` before racing. To re-plan an existing map (for example after changing
the car parameters in `ws/src/planning/config/racecar.ini`): `ros2 run planning build_raceline <name>`.

### Particle filter (MCL) instead of Cartographer localization

```bash
scripts/race.sh <name> speed_scale:=0.5 localizer:=mcl     # or: ros2 launch control race.launch.py track:=<name> localizer:=mcl
```

`perception`'s `mcl_node` is a numpy particle filter with a likelihood-field lidar model (no range_libc
build) on `tracks/<name>/map.yaml`. It publishes the same `map -> odom` and `/localized_pose` as the
Cartographer path, so pure pursuit is unchanged, plus `/mcl/particles` and `/map` for Foxglove. It finds the
car by itself with a scan-matching search (standing still, any heading): near the raceline first (60-130 ms on
porto_v2, the car may go at once), verified in the background by a search of the whole track at every heading,
which also covers a car turned across the track after a crash; at the start, after every sim reset or crash,
and when lost; no sim ground truth, so it behaves as on the real car. If
two places on the track look the same to the lidar, it settles it from the last known pose (or the map origin);
when even that cannot tell, it takes its best guess after a few tries and keeps the other places, switching if
one of them fits better as the car drives (a `/initialpose` from Foxglove or RViz also settles it). While
driving, it re-anchors the pose when the scan stops fitting the map (wheel slip, a stalled sensor stream), and
a scan that keeps not fitting (something the map does not have, or the car moved far) only starts a background
search: the car stops only when the particle cloud itself spreads.
Parameters: `ws/src/perception/config/mcl.yaml`; `ros2 launch perception mcl.launch.py track:=<name> particles:=2000`.
Never run it with `localization.launch.py`: both publish `map -> odom`.

### No map

If mapping fails, `ros2 launch control follow_the_gap.launch.py [speed:=1.0]` (alias `ftg`) drives on the
LiDAR alone. Run it instead of `race.sh`, never alongside it: both publish `/drive`.

## Interfaces

| Topic | Direction | |
|---|---|---|
| `/scan` | out | LiDAR (`sensor_msgs/LaserScan`) |
| `/odom` | out | wheel-encoder odometry, TF `odom -> base_link` (`nav_msgs/Odometry`); `vesc_to_odom` on the car |
| `/sensors/imu/raw` | out | IMU including gravity (`sensor_msgs/Imu`); used by Cartographer |
| TF `base_link -> laser, imu, front_camera` | out | static sensor mounts; `base_link` is the rear axle |
| `/drive` | in | `ackermann_msgs/AckermannDriveStamped` (speed m/s, steering rad) |
| TF `map -> odom`, `/tracked_pose` | out | Cartographer pose in the `map` frame |
| `/localized_pose` | out | `/tracked_pose` relayed only once localized (`localization_gate`); what `race.launch.py` drives on |
| `/autodrive/roboracer_1/sim_reset` | out | one `std_msgs/String` event per sim reset or respawn (`fast_bridge`, sim only) |
| `/ground_truth/odom`, `/ground_truth/pose`, TF `world -> ground_truth` | out | sim ground truth in the sim `world` frame, debugging only; off with `publish_restricted_topics:=false` |

`fast_bridge`, `adapter`, `wheel_odom` and `teleop_keyboard` are sim only: on the car, f1tenth_system's
`vesc_to_odom` publishes `/odom` and the VESC does the speed control. Raw simulator topics are under
`/autodrive/roboracer_1/*`. Foxglove: `ws://localhost:8765`.
Frames, file formats and car parameters are in [`docs/pipeline.md`](docs/pipeline.md); each package's
nodes and parameters are in its README (e.g. [`ws/src/control/README.md`](ws/src/control/README.md)).

## Testing

```bash
colcon build --symlink-install && source install/setup.bash
ROS_DOMAIN_ID=42 python3 -m pytest src/control/test src/perception/test src/planning/test src/autodrive_bridge/test
```

Some tests launch nodes that publish `/drive`. With the bridge running, a separate `ROS_DOMAIN_ID`
keeps them from driving the sim car.

## Troubleshooting

- **Nodes keep running after a launch is stopped:** stop `ros2 launch` with Ctrl-C (or `kill -INT -- -<pgid>`),
  not `kill`/`timeout`. SIGTERM to `ros2 launch` alone leaves its nodes running. Check with `ros2 node list`.
- **The car does not move at the start of a race:** it waits until Cartographer has localized, usually
  2-3 s. `ros2 topic echo /localized_pose` shows when it starts.
- **`map_track.sh` refuses to start:** it needs an interactive terminal, e.g. `docker compose exec ros bash`.
- **`save_map` fails:** mapping keeps running. Press Enter to retry, or `q` to quit without a map.
- **Rebuilding from scratch:** `cbclean` deletes `build/`, `install/` and colcon's logs, and keeps the
  mapping logs in `ws/log`.

## Contributing

Open an issue, or fork the repository and open a pull request against `main`. Before opening a PR,
build the workspace and run the tests above, and keep each commit to one change.

## Acknowledgement

This stack builds on these open-source projects:

- [AutoDRIVE Ecosystem](https://github.com/Tinker-Twins/AutoDRIVE) and its
  [RoboRacer Sim-Racing](https://github.com/AutoDRIVE-Ecosystem/AutoDRIVE-RoboRacer-Sim-Racing) simulator and devkit
- [f1tenth_system](https://github.com/f1tenth/f1tenth_system): topic names and the car-side drivers
- [Cartographer](https://github.com/cartographer-project/cartographer) and
  [Cartographer ROS](https://github.com/cartographer-project/cartographer_ros)
- [global_racetrajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization) (TUM)
- [ForzaETH Race Stack](https://github.com/ForzaETH/race_stack): the acceleration limit in the adapter follows its `ActuationLimiter`
- F1TENTH / UPenn ESE 6150 pure-pursuit lab template (MIT, see [`ws/src/control/LICENSE`](ws/src/control/LICENSE))

## Citation

If this stack helps your work, please cite the projects it builds on, in particular the AutoDRIVE
simulator ([Samak et al., 2023](https://arxiv.org/abs/2212.05241)) and the TUM minimum-time optimization
([Christ et al., 2021](https://doi.org/10.1080/00423114.2019.1704804)).
