#!/usr/bin/env python3
"""Follow the gap with disparity extension: drives the mapping lap(s) without a map or localization.

Optionally counts laps on pose_topic and stops the car after `laps` of them.

    ros2 launch control disparity_gap.launch.py [speed:=1.0] [laps:=2]

Subscribes /scan (sensor_msgs/LaserScan) and pose_topic (geometry_msgs/PoseStamped, for laps only),
publishes /drive (ackermann_msgs/AckermannDriveStamped). Used by scripts/map_track.sh --gap-follow.
"""

# From Roboracer Lab 4: https://github.com/f1tenth/f1tenth_lab4_template

import time

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan

from control.wall_follow_node import LapCounter

# Defaults for the ROS parameters. disparity_gap.launch.py loads max_steering_angle from config/vehicle.yaml
# and sets max_speed from its speed argument.
DEFAULTS = {
    'max_range': 10.0,          # m, longer returns are clipped to this
    'fov': np.pi / 2,           # rad, only beams within +/- this angle can be aimed at
    'car_width': 0.27,          # m, the RoboRacer; mirrors planning/config/racecar.ini
    'safety_margin': 0.05,      # m, extra room added on each side of the car
    'disparity_jump': 0.35,     # m, a jump this big between neighbours is an obstacle edge
    'free_thresholds': [3.0, 2.0, 1.2, 0.6],  # m, tried in order until a gap exists
    'max_steering_angle': 0.524,  # rad, steering lock; mirrors config/vehicle.yaml
    'max_speed': 1.0,           # m/s, the speed cap
    'min_speed': 0.5,           # m/s, unless max_speed is lower
    'speed_per_metre': 0.6,     # m/s of speed per metre of clear road ahead
    'side_clearance': 0.08,     # m, gap kept between the car's side and anything beside it
    'push_gain': 2.0,           # rad per metre that something intrudes into the side clearance
    'path_lookahead': 6.0,      # m, how far along the chosen path to look for its narrowest point
    'path_cap': 1.5,            # m, room beyond this on a side counts as open
    'body_behind_lidar': 0.45,  # m, the car's body runs from this far behind the LiDAR...
    'body_ahead_of_lidar': 0.15,  # m, ...to this far in front of it
    'laps': 0,                  # stop after this many laps, 0 = never
    'pose_topic': '/tracked_pose',  # for lap counting only
    'lap_leave_radius': 3.0,    # m, see LapCounter
    'lap_return_radius': 1.5,   # m
    'lap_min_distance': 10.0,   # m
}


class DisparityGapFollow(Node):
    """Follow the gap with disparity extension.

    1. Clean the scan (invalid beams and no returns count as far, clip long returns).
    2. Extend every obstacle edge sideways by half the car's width plus a margin,
       so every beam that is still long is a direction the whole car fits through
       (the closest-point bubble, applied to every edge instead of just one).
    3. Find runs of beams longer than a free-space threshold (relaxing it if
       there are none) and pick the deepest, the widest among near-equal depths.
    4. Aim at the deepest part of that gap, shifted to the middle of the
       narrowest spot along the way.
    5. Never turn toward something beside the car's body, and push away from it.
    6. Set speed from the clear distance straight ahead, capped at max_speed.
    """

    def __init__(self, parameter_overrides=None):
        super().__init__('disparity_gap_node', parameter_overrides=parameter_overrides)
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)

        self.laps = self.param('laps')
        self.lap_counter = LapCounter(self.param('lap_leave_radius'), self.param('lap_return_radius'),
                                      self.param('lap_min_distance'))
        self.done = False
        self.started = time.monotonic()
        self.pose_warned = False

        self.sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        max_speed = self.param('max_speed')
        if self.laps > 0:
            pose_topic = self.param('pose_topic')
            self.create_subscription(PoseStamped, pose_topic, self.pose_callback, 10)
            self.get_logger().info(f'Following the gap at up to {max_speed:.1f} m/s, stopping after '
                                   f'{self.laps} lap(s) counted on {pose_topic}')
        else:
            self.get_logger().info(f'Following the gap at up to {max_speed:.1f} m/s until stopped')

    def param(self, name):
        return self.get_parameter(name).value

    def preprocess_lidar(self, ranges, range_min, range_max):
        """Clip every return to max_range; return (ranges, valid).

        No return (inf) is far. Invalid beams (NaN, below range_min) also count as far, as in
        wall_follow_node: as 0 m each one would be an obstacle edge right at the car, and its
        extension would blank most of the scan. A real gap that narrow is covered anyway by the
        extension of its neighbours' edges."""
        limit = min(self.param('max_range'), range_max) if range_max > 0 else self.param('max_range')
        ranges = np.asarray(ranges, dtype=float)
        valid = ~np.isnan(ranges) & (ranges >= range_min)
        return np.clip(np.where(valid, ranges, limit), 0.0, limit), valid

    def extend_disparities(self, ranges, angle_increment):
        """At each obstacle edge, overwrite the beams beside it (on the far side)
        with the edge's distance, wide enough to cover half the car plus a margin."""
        radius = self.param('car_width') / 2 + self.param('safety_margin')
        extended = ranges.copy()
        n = len(ranges)
        jumps = np.flatnonzero(np.abs(np.diff(ranges)) > self.param('disparity_jump'))

        for i in jumps:
            near, far = (i, i + 1) if ranges[i] < ranges[i + 1] else (i + 1, i)
            dist = max(ranges[near], 0.05)
            width = int(np.ceil(np.arctan(radius / dist) / angle_increment))
            if far > near:
                lo, hi = far, min(n, far + width)
            else:
                lo, hi = max(0, far - width + 1), far + 1
            extended[lo:hi] = np.minimum(extended[lo:hi], dist)
        return extended

    def find_max_gap(self, free_space_ranges):
        """Return [start, end) of the best run of non-zero values, or None.

        Deepest gap first (a wide gap can be a dead-end pocket); gaps nearly as
        deep as the deepest are then compared by width."""
        free = np.concatenate(([0], (free_space_ranges > 0).astype(int), [0]))
        edges = np.flatnonzero(np.diff(free))
        if len(edges) == 0:
            return None
        starts, ends = edges[0::2], edges[1::2]
        depths = np.array([np.max(free_space_ranges[s:e]) for s, e in zip(starts, ends)])
        deep_enough = depths >= 0.9 * np.max(depths)
        widths = np.where(deep_enough, ends - starts, -1)
        best = np.argmax(widths)
        return int(starts[best]), int(ends[best])

    def find_best_point(self, start_i, end_i, ranges):
        """Index (into ranges) of the deepest point in the gap; among near-equal
        depths, the middle one, so the car stays centred on long straights."""
        gap = ranges[start_i:end_i]
        deep = np.flatnonzero(gap >= 0.95 * np.max(gap))
        return start_i + int(deep[len(deep) // 2])

    def choose_target(self, ranges, lo, hi):
        """Best gap at the strictest threshold that has one; else the deepest beam."""
        window = ranges[lo:hi]
        for threshold in self.param('free_thresholds'):
            gap = self.find_max_gap(np.where(window > threshold, window, 0.0))
            if gap is not None:
                return lo + self.find_best_point(gap[0], gap[1], window)
        return lo + int(np.argmax(window))

    def centre_on_path(self, ranges, angles, heading, depth):
        """Angle to the middle of the narrowest point along the chosen heading.

        Aiming at the deepest beam alone grazes whatever bounds the opening
        (a ray only clears an edge by the extension radius); lining up with the
        middle of the tightest spot before reaching it keeps the whole car clear."""
        x, y = ranges * np.cos(angles), ranges * np.sin(angles)
        along = x * np.cos(heading) + y * np.sin(heading)
        across = -x * np.sin(heading) + y * np.cos(heading)   # + is left of the path
        cap = self.param('path_cap')
        best = None
        for s in np.arange(0.5, min(depth, self.param('path_lookahead')), 0.25):
            here = (along > s - 0.25) & (along < s + 0.25) & (ranges > 0)
            left = across[here & (across > 0)]
            right = -across[here & (across < 0)]
            room_l = min(np.min(left), cap) if len(left) else cap
            room_r = min(np.min(right), cap) if len(right) else cap
            if best is None or room_l + room_r < best[1] + best[2]:
                best = (s, room_l, room_r)
        if best is None:
            return heading
        s, room_l, room_r = best
        shift = (room_l - room_r) / 2.0
        return heading + np.arctan2(shift, s)

    def side_gaps(self, ranges, angles):
        """Smallest sideways distance from the LiDAR to anything beside the car's
        body, (left, right)."""
        x = ranges * np.cos(angles)   # forward of the LiDAR
        y = ranges * np.sin(angles)   # left of the LiDAR
        beside = ((x > -self.param('body_behind_lidar')) & (x < self.param('body_ahead_of_lidar'))
                  & (ranges > 0))
        left, right = y[beside & (y > 0)], -y[beside & (y < 0)]
        return (np.min(left) if len(left) else np.inf,
                np.min(right) if len(right) else np.inf)

    def side_correction(self, ranges, angles, steering_angle):
        """Never turn toward something right beside the body (turning swings that
        side into it), and push away from anything inside the side clearance."""
        reach = self.param('car_width') / 2 + self.param('side_clearance')
        left, right = self.side_gaps(ranges, angles)
        if (steering_angle > 0 and left < reach) or (steering_angle < 0 and right < reach):
            steering_angle = 0.0
        push = max(0.0, reach - right) - max(0.0, reach - left)
        return steering_angle + self.param('push_gain') * push

    def choose_speed(self, ranges, angles, steering_angle):
        ahead = ranges[np.abs(angles) < np.radians(8)]
        clear = float(np.min(ahead)) if len(ahead) else 0.0
        speed = self.param('speed_per_metre') * clear
        speed *= 1.0 - 0.5 * abs(steering_angle) / self.param('max_steering_angle')
        # max_speed wins over min_speed, so speed:=0.3 drives at 0.3 at most
        return float(min(max(speed, self.param('min_speed')), self.param('max_speed')))

    def publish_drive(self, steering_angle, speed):
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = float(speed)
        self.pub.publish(drive_msg)

    def lidar_callback(self, data: LaserScan):
        """Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        if self.done:
            return
        if self.laps > 0 and self.lap_counter.start is None and not self.pose_warned \
                and time.monotonic() - self.started > 5.0:
            self.get_logger().warn('No pose received yet: laps are not being counted')
            self.pose_warned = True

        angles = data.angle_min + np.arange(len(data.ranges)) * data.angle_increment
        fov = self.param('fov')
        lo = int(np.searchsorted(angles, -fov))
        hi = int(np.searchsorted(angles, fov, side='right'))

        raw, valid = self.preprocess_lidar(data.ranges, data.range_min, data.range_max)
        if not np.any(valid[lo:hi]):
            self.get_logger().warning('No valid LiDAR returns ahead, stopping', throttle_duration_sec=1.0)
            self.publish_drive(0.0, 0.0)
            return
        ranges = self.extend_disparities(raw, data.angle_increment)

        target = self.choose_target(ranges, lo, hi)
        max_steer = self.param('max_steering_angle')
        aim = self.centre_on_path(raw, angles, angles[target], ranges[target])
        steering_angle = self.side_correction(raw, angles, aim)
        steering_angle = float(np.clip(steering_angle, -max_steer, max_steer))

        self.publish_drive(steering_angle, self.choose_speed(ranges, angles, steering_angle))

    def pose_callback(self, msg):
        if self.done or not self.lap_counter.update(msg.pose.position.x, msg.pose.position.y):
            return
        self.get_logger().info(f'Lap {self.lap_counter.laps}/{self.laps} complete')
        if self.lap_counter.laps >= self.laps:
            self.done = True

    def stop(self):
        self.get_logger().info('Stopping: sending zero speed and steering')
        for _ in range(3):   # repeat and wait, so the commands are delivered before the node exits
            self.publish_drive(0.0, 0.0)
            time.sleep(0.05)


def main():
    # Handle Ctrl+C ourselves, as wall_follow_node does: rclpy's default handler shuts ROS down first,
    # so stop() could never publish and the car would coast until the adapter's command timeout
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = DisparityGapFollow()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.stop()                 # always leave the car stopped
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
