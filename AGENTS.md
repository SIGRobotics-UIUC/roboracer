# RoboRacer development guide

## Architecture

- ROS 2 Humble; five Python `ament_python` packages and the native `slam_graph_tools` snapshot exporter under `ws/src/`.
- `autodrive_bridge`: simulator-only Socket.IO `fast_bridge`, sensor/command `adapter`, encoder-based `wheel_odom`, and keyboard teleop. On the car, hardware drivers/VESC replace these interfaces.
- `perception`: Cartographer mapping and frozen-map localization, `localization_gate`, alternative numpy MCL, map saving, and map-origin recording. `ekf.launch.py` and `corrector` are experimental and outside the race pipeline.
- Optional mapping backend: `slam_mapping.launch.py` runs async SLAM Toolbox plus `slam_mapping_pose` (native pose relay and online trajectory). `save_slam_map` stages native `.posegraph`/`.data` and uses `slam_graph_tools` to export a consistent final map/trajectory; it does not create a Cartographer pbstream or use simulator ground truth.
- `planning`: offline centerline/width extraction and TUM minimum-time raceline generation; optional runtime `obstacle_spliner_node` for stationary obstacles absent from the map. TUM runs in a separate venv through subprocess/CSV exchange.
- `control`: pure pursuit with adaptive lookahead, reactive follow-the-gap, centering wall follower, and experimental disparity-gap follower. `race.launch.py` orchestrates localization, pure pursuit, and optional avoidance.
- `benchmark`: managed collection with an independent command gate/rosbag2 recorder, plus offline race/timing analysis. Artifacts use `ws/log/runs/` and `ws/log/reports/`; never execute/import `ws/scripts/` from this package. See its README and VALIDATION.md for supported adapters and unvalidated simulator behavior.
- Read `docs/pipeline.md`, relevant launch/config files, and neighboring tests before editing. Keep changes scoped; preserve working interfaces and validated behavior.

## Environment and commands

From the repository root:

```bash
docker compose build
docker compose up -d
docker compose exec ros bash
```

Inside the container (`/root/ws`):

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
# Targeted build, when dependencies are already built:
colcon build --symlink-install --packages-select <package>
```

- Compose bind-mounts `./ws` to `/root/ws`, mounts `.bash_aliases` read-only, and exposes 4567 (AutoDRIVE) and 8765 (Foxglove). The simulator runs separately on the host.
- The image installs ROS dependencies and the patched, pinned TUM optimizer in `/opt/tum_optimizer` and `/opt/tum_venv`. Keep its numerical dependencies isolated from ROS Python. Rebuild the image after dependency/patch changes; rebuild/source the workspace after package metadata changes.
- `.dockerignore` allowlists the planning Docker dependency directory; update it deliberately if adding image COPY sources.

## ROS interfaces and ownership

| Interface | Owner and contract |
|---|---|
| `/scan` | Adapter/car LiDAR; `sensor_msgs/LaserScan`, frame `laser` |
| `/odom` | `wheel_odom`/car odometry; `nav_msgs/Odometry` |
| `/sensors/imu/raw` | Adapter/car IMU; `sensor_msgs/Imu`, gravity included |
| `/tracked_pose` | Cartographer; `geometry_msgs/PoseStamped` in `map` |
| `/localized_pose` | `localization_gate` or MCL; valid localized pose consumed by racing |
| `/local_path`, `/local_path/valid` | Obstacle planner; `nav_msgs/Path` in `map` and `std_msgs/Bool` |
| `/drive` | One active controller/teleop; `AckermannDriveStamped`, speed m/s and steering radians |
| `/raceline/provenance` | Pure pursuit; transient-local String describing raceline origin |
| `/autodrive/reset_command` | Simulator reset request, Bool |
| `/autodrive/roboracer_1/sim_reset` | Bridge reset/respawn event, String; localizers consume it |

- TF chain: `map -> odom -> base_link -> laser/imu/front_camera`. Exactly one of Cartographer, MCL, or SLAM Toolbox mapping owns `map -> odom`; wheel odometry owns `odom -> base_link`; the sim adapter owns static mounts. `base_link` is the rear axle.
- Never run both localizers or competing `/drive` publishers. Preserve single-instance guards and bridge frame timestamps; duplicate odometry/IMU stamps can abort Cartographer.
- Cartographer requires the IMU at the `base_link` origin (mount rotation is allowed). Preserve frame IDs, units, QoS compatibility, and timestamp alignment.
- Bridge raw topics use `/autodrive/roboracer_1/*`. With `use_sim_time:=true`, the bridge publishes a host-monotonic transport `/clock` that continues during simulator stalls; sensor stamps represent transport arrival, not native simulator acquisition/physics time. Do not assume deterministic simulation time or a Windows/container clock relationship.
- During benchmark runs, controllers publish `/benchmark/requested_drive`; only `benchmark_gate` publishes `/drive`. Do not bypass the inhibited gate or start another controller/teleop. Raw runs are sealed immediately after finalization; analysis must write outside them.

## Mapping, planning, and racing

Run the bridge in its own container shell:

```bash
ros2 launch autodrive_bridge autodrive_launch.py
```

Connect the host simulator to `127.0.0.1:4567`, Autonomous mode. In another sourced container shell:

```bash
scripts/map_track.sh <new-name> --wall-follow --laps 2
scripts/race.sh <name> speed_scale:=0.5
scripts/race.sh <name> speed_scale:=0.5 localizer:=mcl avoid:=true
```

- Mapping resets the sim, starts `perception/mapping.launch.py`, drives laps, runs `save_map`, stops mapping, and runs `planning build_raceline`. Without `--wall-follow`, drive with `ros2 run autodrive_bridge teleop_keyboard` and answer the terminal prompts. `--gap-follow` is experimental; prefer wall following.
- Opt in with `scripts/map_track.sh <new-name> --slam-toolbox --wall-follow --laps 2`. Existing track directories are refused. SLAM Toolbox exports final corrected poses and map pixels from the same serialized graph; retain online poses for diagnostics and preserve planner consistency checks. Its follower speed defaults to 0.5 m/s; calibration and Cartographer defaults stay unchanged. See `docs/pipeline.md` for saving and MCL compatibility; `race.sh` still requires a pbstream.
- Individual commands: `ros2 run perception save_map <name>` while mapping; `ros2 run planning build_raceline <name>` to re-plan. Inspect `preview.png` before racing.
- Planner exit codes: 0 clean mintime; 2 raceline written using fallback/unclean result; 1 failure. Preserve warnings and the JSON provenance header; never report fallback as clean optimization.
- `race.sh` resets the sim and starts `control/race.launch.py`; direct launch does not reset it. Default localization is Cartographer; `localizer:=mcl` selects MCL. `avoid:=true` starts the obstacle planner and follows its path at raceline speeds.
- Known inconsistency: `race.sh` requires `map.pbstream` even with MCL; `mcl.launch.py` itself loads `map.yaml`. Report this when relevant; avoid unrelated fixes.
- No-map alternative: `ros2 launch control follow_the_gap.launch.py speed:=1.0`, instead of racing.
- Stop launches with terminal Ctrl-C or SIGINT to their process group; SIGTERM to only the launch process can leave nodes alive. Check `ros2 node list` before restarting; disconnect the stock simulator before restarting its bridge.

## Coding and new nodes

- Follow neighboring Python style: four-space indentation, snake_case functions/modules, descriptive classes, and short interface/unit docstrings. No repository-wide formatter/linter configuration is currently provided.
- Separate reusable geometry/algorithm code from ROS I/O where practical; keep callbacks bounded and make time, randomness, and state transitions testable.
- Put runtime modules in `<package>/<package>/`; add console scripts in `setup.py`, runtime/test dependencies in `package.xml`, and installed launch/config resources in `setup.py`. Use declared ROS parameters and installed package-share paths rather than new machine-specific paths.
- Follow existing `rclpy` initialization/spin/cleanup patterns, node logging, explicit message types, and compatible QoS. Controllers must publish stops during shutdown; preserve existing process cleanup and watchdog behavior.
- Shared wheelbase/steering geometry belongs in `control/config/vehicle.yaml`; planning mirrors it in `planning/config/racecar.ini`, checked by `test_vehicle_params.py`. Adapter dynamics/calibration belong in `autodrive_bridge/config/adapter.yaml`.

## Testing

After building and sourcing, from `/root/ws`:

```bash
ROS_DOMAIN_ID=42 python3 -m pytest src/control/test src/perception/test src/planning/test src/autodrive_bridge/test
# Example focused algorithm checks:
ROS_DOMAIN_ID=42 python3 -m pytest src/control/test/test_lookahead.py src/planning/test/test_centerline.py
```

- Unit/algorithm tests use synthetic geometry, scans, and fixture maps. Callback-level tests may still require `rclpy`; they do not prove ROS transport or launch wiring.
- ROS integration tests launch nodes/publish messages and check commands, resets, duplicate-instance handling, and process cleanup. Some process tests require Linux. Tests can publish `/drive`: isolate them from a running simulator.
- Domain selection is currently inconsistent: bridge fixtures force 97, bridge launch tests use 77, and some controller tests choose random nonzero domains. The outer `ROS_DOMAIN_ID` is not universal isolation; inspect fixtures before parallel runs and avoid those domains for live stacks.
- Optimizer tests in `planning/test/test_build_raceline.py` need the Docker TUM venv; some run for minutes. Missing optimizer dependencies cause skips. Report skips explicitly rather than claiming optimizer validation passed.
- Full-simulator testing currently requires an external AutoDRIVE instance; there is no checked-in unattended scenario runner or CI workflow. Synthetic/node tests do not establish collision-free simulated laps. For relevant behavior changes, record track, parameters, localizer, laps/collisions, and failures from simulator validation, or state it was not performed.
- Add focused behavioral regressions for algorithm/safety changes. Run affected tests, then broader integration/optimizer checks as warranted; report exact commands and results.
- SLAM mapping tests: `ROS_DOMAIN_ID=65 python3 -m pytest src/perception/test/test_slam_mapping.py -q`; `test_slam_mapping_integration.py` runs a real mapper on isolated domain 66 with synthetic sensors and temporary artifacts. This is not an AutoDRIVE mapping validation.

## Preserve artifacts and validated behavior

- `ws/tracks/<name>/` is generated and git-ignored. Preserve validated maps, pbstreams, mapping trajectories, racelines, previews, provenance, and useful logs. Do not delete/overwrite them as cleanup or regenerate them silently.
- Use new track names, temporary directories, or explicit `TRACKS_DIR`/planner `--tracks-dir` for experiments. Re-planning overwrites generated planning outputs: copy a validated track before experimenting unless replacement is requested.
- Keep generated `ws/build`, `ws/install`, `ws/log`, and track outputs out of commits. Preserve committed test fixtures; create test outputs in temporary directories.
- Preserve CSV formats, map-frame geometry, and driving direction. TUM raceline `psi_rad` uses yaw minus pi/2; do not treat it as ROS yaw. Keep existing working defaults and safety behavior unless the task explicitly changes them.

## Simulator ground truth: prohibited inputs and review debt

- Never introduce simulator ground truth as an input to racing, localization, planning, or control, including privileged raw position/odometry, ground-truth TF, or derived reset/pose hints. Use LiDAR, encoder odometry, and IMU interfaces available on the car. Ground truth may be used by isolated evaluation/visualization to measure results, never to feed driving decisions.
- Existing exceptions/dependencies below are review debt, not permission to extend them. Identify them in relevant change reports; do not silently remove or redesign them during unrelated work:
  - `perception/localization_gate.py` subscribes to `/ground_truth/odom`, uses ground-truth speed to wait for rest after resets, and combines pose with `map_to_world.yaml` to initialize relocalization. This violates the desired input policy.
  - `fast_bridge.py` detects crashes/teleports using privileged collision counts and simulator position/velocity when restricted topics are enabled; its `sim_reset` events affect localization. Explicit reset-command events work without restricted telemetry.
  - `map_origin.py` derives `/map_origin/ground_truth`; `save_map.py` stores it as `map_to_world.yaml`. Nominally debugging metadata, it currently feeds the localization-gate dependency above.
  - Experimental `corrector.py` reads raw simulator odometry for velocity-error diagnostics; keep that comparison outside the corrected estimate and race pipeline.
- `autodrive_launch.py` currently defaults `publish_restricted_topics:=true`. Setting it false removes privileged telemetry/automatic respawn detection but does not alone eliminate the localization gate's map-origin fallback assumption. Do not claim the existing default stack is ground-truth independent. Review and test migration separately.

## Safety invariants

- Preserve localization gating: racing consumes `/localized_pose`; do not bypass it with `/tracked_pose` to make the car move. Never substitute `/ground_truth/pose` (which is in `world`).
- Maintain finite pose/quaternion checks, `map` frame checks, pose freshness, raceline distance/heading checks, steering limits, nonnegative speed, and nonfinite-command stops.
- With avoidance enabled, require a fresh valid map-frame path with one point per raceline point; stop if no collision-free path exists. Preserve scan/pose timestamp interpolation and stale-input invalidation in the obstacle planner.
- Preserve pure pursuit's pose-loss watchdog, repeated stop burst, shutdown stops, and bridge command timeout. A single zero command can be insufficient because of the adapter speed ramp.
- New driving paths must stop on stale/invalid required inputs and receive regression tests for that behavior. Do not weaken thresholds, disable adaptive lookahead, or increase speed defaults merely to pass a demonstration.
- Existing checks are coarse: Cartographer can extrapolate fresh-stamped poses during a stall, and pure pursuit has no direct scan-based emergency stop. Do not represent current checks as complete collision protection.

## Context efficiency

- Start with AGENTS.md, the user-named files, and directly relevant neighboring code. Expand scope only when necessary.
- Prefer targeted `rg` searches and bounded file reads over repository-wide dumps.
- Exclude generated directories from routine searches: ws/build/, ws/install/, ws/log/, .idea/, __pycache__/, and .pytest_cache/.
- Treat ws/tracks/ as experiment data. Read only explicitly relevant track artifacts; never modify validated tracks without approval.
- When debugging a specific run, inspect its named log and report files rather than all historical runs.
- Do not read large .pbstream, .posegraph, .data, .pgm, or image files as text; use appropriate parsers or metadata inspection when needed.
- Avoid repeating unchanged repository scans or copying large tool outputs into the conversation. Summarize findings and cite file paths and relevant functions.
- Run focused tests first, then expand to broader suites when the change warrants it.
- These are default search and context rules, not restrictions on accessing files explicitly needed for an assigned task.
