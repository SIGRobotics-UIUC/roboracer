#!/usr/bin/env python3
"""Pure pursuit on a generated raceline (tracks/<track>/raceline.csv, TUM output format).

On every pose (pose_topic, map frame): find the closest raceline point, pick the first point at
least one lookahead ahead of the car, steer on the arc through it, drive at the raceline speed
* speed_scale (capped at max_speed). One /drive per pose.

  lookahead = clip(lookahead_time * drive speed, lookahead_min, lookahead_distance)

adaptive_lookahead (default on): the lookahead is also capped by the curvature of the raceline near
the car, so it does not cut corners (pure pursuit settles about lookahead^2 * kappa / 2 inside a curve;
control/lookahead.py):

  cap = max(lookahead_min, min over raceline points j of
            (sqrt(2 * adaptive_lookahead_cut / |kappa_j|) + adaptive_lookahead_slope * |arc distance to j|))
  lookahead = min(above, max(cap, adaptive_lookahead_min_time * drive speed))

kappa from the raceline geometry, averaged over adaptive_lookahead_smooth m; cap precomputed per point.

The closest point is searched within search_window m along the line of the previous one, so it
cannot jump to another stretch of track that passes close by (iros2026: two legs 1.07 m apart,
driven in opposite directions). On the first pose, or when that point is more than
reacquire_distance m away, the whole line is searched, among points heading within 90 deg of the car.

Safety: a pose not in the map frame, with a non-finite position or a quaternion that is not unit,
older than max_pose_age s, or more than max_offset m from the raceline gets a stop command (speed 0)
instead; driving resumes on the next good pose. A closest point heading more than 90 deg from the car
(after a spin, a flipped pose, a teleport) forces the whole-line search; with no point within
flip_reacquire_distance heading along the car, it stops. With no pose for max_pose_age s (localization
withheld or lost) a timer sends stops at 20 Hz for stop_burst s, and Ctrl-C sends them for 0.5 s: the
adapter's speed ramp only brakes on a steady stream of stops, a lone one after a pause restarts it near
the measured speed (close to cruise throttle).

avoid: steer on /local_path (planning's obstacle_spliner_node: the raceline's points shifted sideways
around an obstacle, one per raceline point) instead of the raceline; speeds stay the raceline's. No
path yet, a path older than max_path_age s, one of another length, or /local_path/valid false (no
collision-free path) gets a stop command.

Which line it drives: on start it logs the raceline's provenance (build_raceline's '# raceline:' header
line: method, solver, fallbacks) and WARNs unless it is a clean TUM mintime line, also when the
provenance is unknown (older or hand-made files). /raceline/provenance (latched String) repeats it.
"""
import json
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String

from perception.tracks import track_dir
from planning.raceline import read_provenance

from control.lookahead import curvature_cap, lookahead

# raceline.csv columns: s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2
X, Y, VX = 1, 2, 5


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        p = self.declare_parameter
        track = p('track', '').value                      # set by pure_pursuit.launch.py
        raceline_path = p('raceline_path', '').value      # overrides track
        pose_topic = p('pose_topic', '/localized_pose').value   # PoseStamped in the map frame, once localized
        self.speed_scale = p('speed_scale', 0.5).value
        self.max_speed = p('max_speed', 0.0).value        # m/s cap on the drive speed, 0 = no cap
        self.lookahead_distance = p('lookahead_distance', 3.0).value   # m, max
        self.lookahead_min = p('lookahead_min', 0.6).value             # m, ~2x the wheelbase
        self.lookahead_time = p('lookahead_time', 0.4).value           # s
        adaptive = p('adaptive_lookahead', True).value                # cap by the curvature ahead
        la_cut = p('adaptive_lookahead_cut', 0.1).value                # m, lookahead <= sqrt(2 cut / kappa)
        la_slope = p('adaptive_lookahead_slope', 0.5).value            # + slope * arc distance to the curve
        self.la_min_time = p('adaptive_lookahead_min_time', 0.25).value  # s, but >= min_time * speed
        la_smooth = p('adaptive_lookahead_smooth', 0.5).value          # m, curvature averaged over
        search_window = p('search_window', 2.0).value                  # m along the line, each way
        self.reacquire_distance = p('reacquire_distance', 1.0).value   # m from the line
        # m; after a heading failure (iros2026's opposite legs are 1.07 m apart)
        self.flip_reacquire_distance = p('flip_reacquire_distance', 0.5).value
        self.max_pose_age = p('max_pose_age', 0.5).value   # s, as fast_bridge's /drive watchdog
        self.stop_burst = p('stop_burst', 1.5).value       # s of 20 Hz stops once poses are lost
        self.max_offset = p('max_offset', 3.0).value       # m from the line, farther = bad pose
        self.avoid = p('avoid', False).value               # follow /local_path instead of the raceline
        self.max_path_age = p('max_path_age', 0.5).value   # s
        # vehicle geometry comes only from config/vehicle.yaml (pure_pursuit.launch.py loads it)
        self.max_steering_angle = p('max_steering_angle', Parameter.Type.DOUBLE).value
        self.wheelbase = p('wheelbase', Parameter.Type.DOUBLE).value
        if self.max_steering_angle is None or self.wheelbase is None:
            raise RuntimeError('max_steering_angle / wheelbase unset: load control/config/vehicle.yaml')
        if not self.speed_scale > 0:
            self.get_logger().fatal(f'speed_scale must be > 0, got {self.speed_scale}')
            raise ValueError(f'speed_scale {self.speed_scale}')

        path = raceline_path or os.path.join(track_dir(track), 'raceline.csv')
        if not os.path.isfile(path):
            self.get_logger().fatal(f'Raceline not found: {path} (set track or raceline_path)')
            raise FileNotFoundError(path)
        self.waypoints = np.loadtxt(path, delimiter=';', ndmin=2)
        if len(self.waypoints) > 1 and np.allclose(self.waypoints[0, [X, Y]], self.waypoints[-1, [X, Y]]):
            self.waypoints = self.waypoints[:-1]   # TUM closes the loop by repeating the first row
        if (self.waypoints[:, VX] < 0).any():
            self.get_logger().warning(f'{path}: negative vx_mps, driven as 0 (no reversing)')
        self.xy = self.waypoints[:, [X, Y]]
        # heading of each point toward the next, from the geometry (no psi convention involved)
        seg = np.roll(self.xy, -1, axis=0) - self.xy
        self.heading = np.arctan2(seg[:, 1], seg[:, 0])
        step = np.median(np.hypot(seg[:, 0], seg[:, 1]))
        self.window = max(1, int(np.ceil(search_window / step))) if step > 0 else len(self.xy)
        self.cap = (curvature_cap(self.xy, la_cut, la_slope, self.lookahead_min, self.lookahead_distance, la_smooth)
                    if adaptive else np.full(len(self.xy), np.inf))
        self.closest = None
        self.path_xy, self.path_stamp, self.path_valid = None, 0.0, False
        self.last_pose = None      # s, clock time of the last pose that got a command
        self.get_logger().info(f'Loaded {path}: {len(self.waypoints)} points; lookahead '
                               f'{self.lookahead_time} s x speed, {self.lookahead_min}-{self.lookahead_distance} m'
                               + (f'; adaptive: <= sqrt(2 x {la_cut} m / kappa) + {la_slope} x distance to the curve, '
                                  f'>= {self.la_min_time} s x speed (cap {self.cap.min():.2f} m at the tightest)'
                                  if adaptive else ''))
        self.report_provenance(path)

        self.create_subscription(PoseStamped, pose_topic, self.pose_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        if self.avoid:
            self.create_subscription(Path, '/local_path', self.path_callback, 10)
            self.create_subscription(Bool, '/local_path/valid', self.valid_callback, 10)
            self.get_logger().info('Avoiding obstacles: following /local_path')
        self.create_timer(0.05, self.watchdog)

    def report_provenance(self, path):
        """Log which raceline this is (WARN unless a clean mintime line) and latch it on /raceline/provenance."""
        try:
            prov = read_provenance(path)
        except ValueError as e:          # a '# raceline:' line that is not JSON
            prov, bad = None, str(e)
        else:
            bad = None
        with open(path) as f:
            comments = [ln.strip() for ln in f if ln.startswith('#')][:3]
        if prov is not None and prov.get('clean') and prov.get('method') == 'mintime':
            text = (f"Raceline: TUM mintime, converged with the default settings ({prov.get('solver')}), "
                    f"lap {prov.get('lap_s')} s, built {prov.get('built')}")
            self.get_logger().info(text)
        elif prov is not None:
            text = (f"RACELINE IS NOT A CLEAN MINTIME LINE: method {prov.get('method')}; "
                    f"{'; '.join(prov.get('notes', []))}; changed {json.dumps(prov.get('changed', {}))}; "
                    f"lap {prov.get('lap_s')} s, built {prov.get('built')} ({path})")
            self.get_logger().warning(text)
        else:
            text = (f'RACELINE OF UNKNOWN PROVENANCE: {path} has no build_raceline provenance line'
                    + (f' (unreadable: {bad})' if bad else '')
                    + ' (built before it was recorded, or written by hand), so it may not be a converged mintime '
                    f'line. Header: {" | ".join(comments)}')
            self.get_logger().warning(text)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.provenance_pub = self.create_publisher(String, '/raceline/provenance', latched)
        self.provenance_pub.publish(String(data=text))

    def path_callback(self, msg: Path):
        if msg.header.frame_id != 'map' or len(msg.poses) != len(self.xy):
            self.get_logger().warning(f"/local_path: {len(msg.poses)} poses in frame '{msg.header.frame_id}', "
                                      f"expected {len(self.xy)} in 'map'", throttle_duration_sec=2.0)
            self.path_xy = None
            return
        self.path_xy = np.array([[q.pose.position.x, q.pose.position.y] for q in msg.poses])
        self.path_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def valid_callback(self, msg: Bool):
        self.path_valid = msg.data

    def drive_speed(self, i):
        v = max(self.waypoints[i, VX] * self.speed_scale, 0.0)   # never reverse
        return min(v, self.max_speed) if self.max_speed > 0 else v

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def watchdog(self):
        """No pose for max_pose_age s: a stop on every tick for stop_burst s (so the adapter brakes), then
        silence (fast_bridge's /drive watchdog keeps the throttle at zero, teleop is not fought)."""
        if self.last_pose is None:
            return
        lost = self.now() - self.last_pose
        if self.max_pose_age < lost <= self.max_pose_age + self.stop_burst:
            self.stop(f'no pose for {lost:.1f} s')

    def stop(self, reason):
        self.get_logger().warning(f'Stopping: {reason}', throttle_duration_sec=1.0)
        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.header.frame_id = 'base_link'
        self.drive_pub.publish(drive)   # speed 0, steering 0

    def find_closest(self, dists, yaw):
        n = len(self.xy)
        if self.closest is not None:
            near = (self.closest + np.arange(-self.window, self.window + 1)) % n
            i = near[np.argmin(dists[near])]
            if dists[i] <= self.reacquire_distance:
                return int(i)
        # first pose, or off the line: the closest point the car is heading along
        along = np.flatnonzero(np.cos(self.heading - yaw) > 0)
        if not along.size:
            along = np.arange(n)
        return int(along[np.argmin(dists[along])])

    def pose_callback(self, msg: PoseStamped):
        self.last_pose = self.now()
        x, y = msg.pose.position.x, msg.pose.position.y
        q = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        norm = np.linalg.norm(q)
        if not (np.isfinite([x, y]).all() and np.isfinite(q).all() and abs(norm - 1) <= 0.1):
            return self.stop(f'invalid pose: position ({x}, {y}), quaternion {q.tolist()}')
        qx, qy, qz, qw = q / norm
        if msg.header.frame_id != 'map':
            return self.stop(f"pose in frame '{msg.header.frame_id}', the raceline is in 'map'")
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9   # 0 = unstamped, stale
        age = self.now() - stamp
        if age > self.max_pose_age:
            return self.stop(f'pose {age:.2f} s old')
        yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))

        xy = self.xy
        if self.avoid:
            if self.path_xy is None:
                return self.stop('no /local_path yet')
            path_age = self.now() - self.path_stamp
            if path_age > self.max_path_age:
                return self.stop(f'/local_path {path_age:.2f} s old')
            if not self.path_valid:
                return self.stop('no collision-free /local_path')
            xy = self.path_xy

        n = len(xy)
        dists = np.hypot(xy[:, 0] - x, xy[:, 1] - y)
        closest = self.find_closest(dists, yaw)
        if np.cos(self.heading[closest] - yaw) <= 0:
            # the line runs the other way (spin, flipped pose, teleport): search the whole line. The
            # previous point is kept on failure, so the car stays stopped until it heads along a line
            # within flip_reacquire_distance (not one farther away, e.g. across a wall)
            prev, self.closest = self.closest, None
            closest = self.find_closest(dists, yaw)
            if np.cos(self.heading[closest] - yaw) <= 0 or dists[closest] > self.flip_reacquire_distance:
                self.closest = prev
                return self.stop('car heading against the raceline')
        if dists[closest] > self.max_offset:
            return self.stop(f'pose {dists[closest]:.1f} m from the raceline')
        self.closest = closest

        # Target: first point after the closest whose distance from the car reaches the lookahead.
        # Off the line, the car's offset is added so the target stays ahead along the line instead
        # of being the closest point, which would cut straight across corners. At most one lap is
        # searched: on a tiny track every point may be within the lookahead, then the farthest
        # point (across the loop) is the target.
        # adaptive_lookahead: also capped by the curvature ahead (the raceline's, as /local_path follows it)
        lookahead_m = lookahead(self.drive_speed(closest), self.lookahead_time, self.lookahead_min,
                                self.lookahead_distance, self.cap[closest], self.la_min_time)
        ahead = (closest + np.arange(n)) % n
        far = np.flatnonzero(dists[ahead] >= lookahead_m + dists[closest])
        target = ahead[far[0]] if far.size else ahead[np.argmax(dists[ahead])]
        target_dist = dists[target]
        if target_dist < 1e-6:
            return

        # target in the vehicle frame, then the arc through it: curvature = 2 y / d^2
        dx, dy = xy[target, 0] - x, xy[target, 1] - y
        target_y = -dx * np.sin(yaw) + dy * np.cos(yaw)
        steering = np.arctan(self.wheelbase * 2 * target_y / target_dist ** 2)

        speed = self.drive_speed(target)
        if not (np.isfinite(steering) and np.isfinite(speed)):
            return self.stop(f'non-finite command: steering {steering}, speed {speed}')
        drive = AckermannDriveStamped()
        drive.header.stamp = self.get_clock().now().to_msg()
        drive.header.frame_id = 'base_link'
        drive.drive.steering_angle = float(np.clip(steering, -self.max_steering_angle, self.max_steering_angle))
        drive.drive.speed = float(speed)
        self.drive_pub.publish(drive)


def main():
    # Handle Ctrl+C ourselves (as wall_follow_node): rclpy's default handler shuts ROS down first, so
    # the stop could never be published and the car would keep its last command
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PurePursuit()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    for _ in range(10):   # always leave the car stopped: 0.5 s of 20 Hz stops, so the adapter brakes
        node.stop('shutting down')
        time.sleep(0.05)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
