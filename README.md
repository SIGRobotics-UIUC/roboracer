# SIGRobotics F1Tenth Stack

ROS 2 Humble in Docker, connected to the AutoDRIVE RoboRacer simulator.

```
Dockerfile, docker-compose.yml   the ROS 2 environment (any OS, amd64 or arm64)
ws/src/autodrive_bridge/         AutoDRIVE <-> ROS 2 bridge
```

## Setup (once)

Install [Docker](https://docs.docker.com/get-docker/) and download the AutoDRIVE RoboRacer
simulator for your OS (release `2026-iros`).

```bash
git clone https://github.com/SIGRobotics-UIUC/roboracer.git && cd roboracer
docker compose build
```

## Run

```bash
docker compose up -d && docker compose exec ros bash
colcon build --symlink-install && source install/setup.bash     # inside the container
ros2 launch autodrive_bridge autodrive_launch.py
```

Start the simulator, **Connect** to `127.0.0.1:4567`, and set **Driving Mode** to **Autonomous**.

| Topic | Direction | |
|---|---|---|
| `/scan` | out | LiDAR (`sensor_msgs/LaserScan`) |
| `/car_state/odom` | out | ground-truth pose + speed, map frame (`nav_msgs/Odometry`) |
| TF `map -> base_link -> laser` | out | |
| `/drive` | in | `ackermann_msgs/AckermannDriveStamped` (speed m/s, steering rad) |

Raw sim topics are under `/autodrive/roboracer_1/*`. Foxglove: `ws://localhost:8765`.
