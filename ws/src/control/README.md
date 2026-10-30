# control

Pure pursuit on a generated raceline, and two followers for the mapping lap(s): the wall follower (the default) and an experimental follow the gap with disparity extension (see below). `pure_pursuit_node` takes the car's pose in the `map` frame,
finds the closest raceline waypoint, picks the first waypoint one lookahead ahead (lookahead =
`lookahead_time` * drive speed, clipped to [`lookahead_min`, `lookahead_distance`]), steers on the arc
through it and drives at the raceline speed scaled by `speed_scale`. The speed is taken at the target
waypoint, not the closest one: the car brakes early into corners, but on corner exits it can exceed the
profile at its own position near `speed_scale` 1.0. With `adaptive_lookahead:=true` the lookahead is
also capped by the curvature of the raceline near the car, so at speed it does not cut corners (on by
default, `adaptive_lookahead:=false` turns it off; see Adaptive lookahead below). See `docs/pipeline.md` for where
it sits in the stack.

`follow_the_gap_node` is a reactive alternative that needs no map: it steers toward the farthest point
(the middle of the run of farthest beams, since ranges are capped at `max_range`) of the widest free
gap within `fov` (default ±90°) of straight ahead in `/scan`, at a constant `speed` (`ros2 run control follow_the_gap_node`, or
`ros2 launch control follow_the_gap.launch.py`, which loads `config/vehicle.yaml`). Its steering is
clipped to `max_steering_angle` (default 0.524 rad, the value in `config/vehicle.yaml`).
`tools/lidar_plot.py` plots its scan processing live (WebAgg, not installed).

`disparity_gap_node` is a follow the gap with disparity extension that drives the mapping lap(s) (experimental, see below).

## Run

```bash
ros2 launch control pure_pursuit.launch.py track:=<name>
ros2 launch control pure_pursuit.launch.py track:=<name> speed_scale:=0.3
ros2 launch control race.launch.py track:=<name> avoid:=true   # + obstacle avoidance
ros2 launch control race.launch.py track:=<name> speed_scale:=1.0   # adaptive lookahead is on by default
```

Localization must be running: the default pose topic is `/localized_pose` (perception's
`localization_gate` or `mcl_node`, which `race.launch.py` starts), so the car waits until it has
localized (~2-3 s at the start and after each sim reset). `pose_topic:=/tracked_pose` (Cartographer raw)
bypasses that gate. The sim's `/ground_truth/pose` is in the sim `world` frame, not `map`
(the offset is in `tracks/<name>/map_to_world.yaml`), so pure pursuit refuses it.

With `avoid:=true`, pure pursuit steers on `/local_path` instead of the raceline: planning's
`obstacle_spliner_node` (started by `race.launch.py avoid:=true`) publishes the raceline with its points
shifted sideways around an obstacle the map did not have, one point per raceline point. Speeds stay the
raceline's.

## Safety

Each pose is checked before it is used; a failed check publishes a stop (speed 0, steering 0) and a
warning (at most once a second), and driving resumes on the next good pose:

- the position must be finite and the quaternion finite with a norm within 1 ± 0.1;
- `header.frame_id` must be `map`;
- the pose must be at most `max_pose_age` s old (an unstamped pose counts as stale);
- the car must be within `max_offset` m of the raceline (with `avoid`, of `/local_path`);
- the closest raceline point must head within 90 deg of the car. If it does not (a spin, a 180 deg flipped
  pose, a teleport), the whole line is searched for the closest point heading along the car; with none
  within `flip_reacquire_distance` the car stops (and stays stopped until it heads along a line);
- with `avoid`: a `/local_path` must have arrived, be at most `max_path_age` s old, have one pose per
  raceline point in the `map` frame, and `/local_path/valid` must be true (a collision-free path exists).

A command that is not finite is replaced by a stop, the speed is never negative (a negative `vx_mps`
drives at 0, `speed_scale` must be > 0). Once a pose has arrived, a 20 Hz watchdog stops the car when no
pose has come for `max_pose_age` s (localization withheld after a sim reset or while `mcl_node`
searches, a crashed localizer): it publishes a stop on every tick for `stop_burst` s, then nothing
(`fast_bridge` holds zero throttle without `/drive`, and teleop is not fought). A burst, not one stop:
the adapter restarts its speed ramp near the measured speed after a pause in `/drive`, so a lone stop
means near-cruise throttle; only a steady stream brakes. The localizers publish at 40-100 Hz, so it does
not fire while racing. Ctrl-C publishes 0.5 s of 20 Hz stops before exiting.

These are coarse. Cartographer extrapolates `/tracked_pose` (`use_pose_extrapolator` defaults to true;
`pose_publish_period_sec` 1e-2), so its stamps stay fresh even when SLAM stalls: the age check only
catches DDS backlog or a non-Cartographer publisher. `max_offset` only catches gross errors: a wrong
pose on a nearby leg (iros2026's legs are 1.07 m apart) passes it. There is no `/scan`-based stop.

## Topics

| Topic | Type | |
|---|---|---|
| `pose_topic` (`/localized_pose`) | `geometry_msgs/PoseStamped` | subscribed, must be in the `map` frame |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | published, one command per pose (and the watchdog's stops) |
| `/local_path` | `nav_msgs/Path` | subscribed with `avoid`, the path steered on |
| `/local_path/valid` | `std_msgs/Bool` | subscribed with `avoid`, false = stop |

## Parameters

| Parameter | Default | |
|---|---|---|
| `track` | `''` | loads `$TRACKS_DIR/<track>/raceline.csv`, `TRACKS_DIR` defaults to `~/ws/tracks` |
| `raceline_path` | `''` | explicit raceline file, overrides `track` |
| `pose_topic` | `/localized_pose` | any `PoseStamped` of `base_link` in the `map` frame |
| `speed_scale` | `0.5` | speed = raceline `vx_mps` * `speed_scale`, must be > 0 |
| `max_speed` | `0.0` | m/s cap on the drive speed, 0 = no cap |
| `lookahead_time` | `0.4` | s; lookahead = `lookahead_time` * drive speed, clipped to [`lookahead_min`, `lookahead_distance`] |
| `lookahead_min` | `0.6` | m, about twice the wheelbase |
| `lookahead_distance` | `3.0` | m, maximum lookahead |
| `adaptive_lookahead` | `true` | cap the lookahead by the curvature of the raceline near the car (below); off = the speed term only |
| `adaptive_lookahead_cut` | `0.1` | m; corner cut allowed: on a curve of radius R the lookahead is at most sqrt(2 * cut * R) |
| `adaptive_lookahead_min_time` | `0.25` | s; the capped lookahead is never below this * drive speed |
| `adaptive_lookahead_slope` | `0.5` | m per m: the cap rises by this much per metre of distance from the curve (before and after it). Node parameter only (not a launch argument) |
| `adaptive_lookahead_smooth` | `0.5` | m; the raceline's curvature is averaged over this length of arc. Node parameter only (not a launch argument) |
| `search_window` | `2.0` | m; the closest point is searched this far along the line either side of the previous one |
| `reacquire_distance` | `1.0` | m; farther than this from that point, the whole line is searched (points heading within 90 deg of the car) |
| `flip_reacquire_distance` | `0.5` | m; when the closest point heads against the car, a point heading along it must be this close, else stop (iros2026's opposite legs are 1.07 m apart) |
| `max_pose_age` | `0.5` | s; an older pose gets a stop, and so does no pose for this long (same as `fast_bridge`'s `/drive` watchdog) |
| `stop_burst` | `1.5` | s of 20 Hz stops after the poses are lost, enough for the adapter to brake |
| `max_offset` | `3.0` | m; farther than this from the raceline, the pose is taken as bad and the car stops |
| `avoid` | `false` | steer on `/local_path` (obstacle avoidance) instead of the raceline |
| `max_path_age` | `0.5` | s; with `avoid`, an older `/local_path` gets a stop |
| `wheelbase` | none | m, from `config/vehicle.yaml` (required) |
| `max_steering_angle` | none | rad, steering is clipped to this; from `config/vehicle.yaml` (required) |

The raceline is the TUM output format: `;` separated, `#` header
`s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2`.

### Adaptive lookahead

Pure pursuit steers on the chord to its target, so on a curve of curvature kappa it settles inside the
line, about lookahead^2 * kappa / 2 deep (in the sim about 0.6 of that), and at a corner entry its target is
already in the turn while the car is still on the straight. The racelines pass 0.25-0.4 m from the walls,
so at `speed_scale` 1.0 (lookahead 1-2.4 m) the car cut into the inside walls of Porto's and Berlin's
corners. With `adaptive_lookahead:=true` (`control/lookahead.py`) each raceline point i gets a cap,
precomputed once from the line's geometry:

    cap_i = max(lookahead_min, min over points j of (sqrt(2 * cut / |kappa_j|) + slope * |arc distance i to j|))
    lookahead = min(clip(lookahead_time * speed, lookahead_min, lookahead_distance), max(cap_closest, min_time * speed))

so on a curve the cut is at most about `adaptive_lookahead_cut` while `min_time` * speed is below the cap;
where the floor wins it is larger (4 m/s on a radius of 1 m: lookahead 1.0 m, cut about 0.5 m), which is
the trade for enough preview at speed. The cap shrinks toward a curve (and grows back after it) by at
most `slope` m per metre driven (no steps, no chatter), with curves found by arc length, so uneven point
spacing and repeated points do not matter. On straights the cap is infinite and nothing changes; at `speed_scale` 0.5 the speed
term is mostly below the cap already. `kappa` comes from the raceline's points (heading change per metre,
averaged over `adaptive_lookahead_smooth` m), not the csv column, so it also works for hand-made lines.
With `avoid:=true` the cap is still the raceline's (per raceline point, which `/local_path` keeps one to
one), not the curvature of `/local_path`'s swerve around an obstacle.
Sim (MCL, 2026-10-29): Porto at 1.0, 0 collisions in 23 laps of 7.00 s (off: 25 collisions in 180 s);
Berlin at 1.0, 0 collisions, 12.38 s laps (off: 30 collisions); lap times at 0.5 unchanged.

## Wall follower

`wall_follow_node` drives the mapping lap(s) of an unseen track: it needs only `/scan`, no map or
localization. It measures the left and right wall distances (two beams per side, projected
`lookahead` m ahead), steers with a PD controller to stay centred, and slows down in corners. With
`laps > 0` it counts returns to its start position on `pose_topic`, then stops the car and exits.

```bash
scripts/map_track.sh <name> --wall-follow [--laps 2] [speed:=1.0]   # mapping with the wall follower
ros2 launch control wall_follow.launch.py speed:=1.0 debug:=true     # on its own, until Ctrl+C
```

| Topic | Type | |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | subscribed |
| `pose_topic` (`/tracked_pose`) | `geometry_msgs/PoseStamped` | subscribed when `laps > 0` (Cartographer, also while mapping) |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | published, one command per scan; zero on exit |

| Parameter | Default | |
|---|---|---|
| `speed` | `1.0` | m/s on straights; keep it low for clean scan matching |
| `kp`, `kd` | `1.05`, `0.10` | rad per m off centre, rad per m/s of drift |
| `lookahead` | `0.8` | m ahead at which the wall distances are projected |
| `beam_angle_deg` | `45` | angle between the side beam and the forward beam |
| `max_range` | `5.0` | m, a side with no return within this has no wall |
| `track_width` | `1.2` | m, holds half of it from the wall when only one wall is visible |
| `corner_slowdown` | `0.5` | fraction of `speed` shed at full steering |
| `max_steering_angle` | `0.524` | rad, steering is clipped to this; `wall_follow.launch.py` loads it from `config/vehicle.yaml` |
| `laps` | `0` | stop after this many laps, 0 = never |
| `lap_leave_radius`, `lap_return_radius`, `lap_min_distance` | `3.0`, `1.5`, `10.0` | m: a lap is leaving the start by the first, coming back within the second, after driving at least the third |
| `debug` | `false` | log wall distances, steering and speed |

## Follow the gap with disparity extension

`disparity_gap_node` drives the mapping lap(s) of an unseen track, like the wall follower: it needs
only `/scan`, no map or localization. At every jump of more than
`disparity_jump` between neighbouring beams it extends the nearer range sideways by half the car's
width plus `safety_margin`, so every beam that is still long is a direction the whole car fits through.
It then takes the runs of beams longer than the first of `free_thresholds` (relaxing it if there are
none) within `fov`, picks the deepest run (the widest among near-equal depths), aims at its deepest
point, shifted to the middle of the narrowest spot along the way, and never turns toward something
beside the car's body. Speed scales with the clear distance straight ahead, between `min_speed` and
`max_speed`. With `laps > 0` it counts returns to its start position on `pose_topic` (the wall
follower's `LapCounter`), then stops the car and exits.

```bash
scripts/map_track.sh <name> --gap-follow [--laps 2] [speed:=1.0]   # mapping with follow the gap
ros2 launch control disparity_gap.launch.py speed:=1.0              # on its own, until Ctrl+C
```

It never runs alongside `race.sh` or the other followers: they all publish `/drive`.
It was tuned in the F1TENTH gym (Levine) and drives Porto in AutoDRIVE (2 laps at ~1 m/s, past an obstacle at the side of the track; one dead ahead is untested).

**Experimental: use `--wall-follow` for mapping.** Known issue: the gap follower takes the inside line through
corners, so its mapping lap is ~13-15 % shorter than the centerline (Porto: 26.8 m vs 30.7 m; the wall follower
drives 30.2 m). `build_raceline` rejects a centerline more than 15 % off the lap, so planning fails about every
other run ("centerline is ... m but the mapping lap is ... m"). The map itself is fine (its centerline is within
~5 cm median of a wall-follower map). Not fixed yet: drive closer to the centre, or replace the length check.

| Topic | Type | |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | subscribed |
| `pose_topic` (`/tracked_pose`) | `geometry_msgs/PoseStamped` | subscribed when `laps > 0` (Cartographer, also while mapping) |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | published, one command per scan; a stop when no beam ahead has a return, and zero on exit |

| Parameter | Default | |
|---|---|---|
| `max_speed` | `1.0` | m/s cap; the launch file's `speed`. Keep it low for clean scan matching |
| `laps` | `0` | stop after this many laps, 0 = never |
| `lap_leave_radius`, `lap_return_radius`, `lap_min_distance` | `3.0`, `1.5`, `10.0` | m, as for the wall follower |
| `min_speed` | `0.5` | m/s, unless `max_speed` is lower |
| `speed_per_metre` | `0.6` | m/s per metre of clear road within ±8° ahead, halved at full steering |
| `max_range` | `10.0` | m, longer returns are clipped to this |
| `fov` | `π/2` | rad, only beams within ± this can be aimed at |
| `car_width`, `safety_margin` | `0.27`, `0.05` | m, the extension radius is half the width plus the margin |
| `disparity_jump` | `0.35` | m between neighbouring beams that counts as an obstacle edge |
| `free_thresholds` | `[3.0, 2.0, 1.2, 0.6]` | m, tried in order until a gap exists |
| `path_lookahead`, `path_cap` | `6.0`, `1.5` | m: how far along the path to look for its narrowest spot; room beyond the cap on a side counts as open |
| `side_clearance`, `push_gain` | `0.08`, `2.0` | m kept beside the body; rad per metre something is inside it |
| `body_behind_lidar`, `body_ahead_of_lidar` | `0.45`, `0.15` | m, the car's body along x relative to the LiDAR |
| `max_steering_angle` | `0.524` | rad, steering is clipped to this; the launch file loads it from `config/vehicle.yaml` |

## Test

```bash
python3 -m pytest ws/src/control/test   # needs a sourced ROS 2 environment and the built package
```

The tests publish `/scan`, `/tracked_pose`, `/localized_pose` and read `/drive`: with the bridge running, give them their
own `ROS_DOMAIN_ID` (e.g. `ROS_DOMAIN_ID=42 python3 -m pytest ...`) so they don't drive the sim car.

## Credit

Adapted from the F1TENTH / UPenn ESE 6150 pure-pursuit lab template (MIT, see `LICENSE`).
