"""Centring wall follower: drives the mapping lap(s) without a map, localization or ground truth.

Measures the distance to the left and right walls, projected a short distance ahead of the car, and
steers with a PD controller to keep the two equal (i.e. stay on the track centre), its steering changing
at most max_steer_rate. When only one wall
is in view it holds half the track width from it, the width measured from the scans that see both
(TrackWidth). When the wall straight ahead is close it turns towards the more open side (a hairpin) and
slows down; "close" is measured in track widths, so it adapts to each track. It also slows in corners. Optionally counts
laps on pose_topic and stops the car after `laps` of them.

    ros2 launch control wall_follow.launch.py [speed:=1.0] [laps:=2] [debug:=true]

Subscribes /scan (sensor_msgs/LaserScan) and pose_topic (geometry_msgs/PoseStamped, for laps only),
publishes /drive (ackermann_msgs/AckermannDriveStamped). Used by scripts/map_track.sh --wall-follow.
"""
import math
import time

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan


class LapCounter:
    """Counts returns to the start position.

    A lap is complete when the car has been more than leave_radius from the start, comes back within
    return_radius of it, and has driven at least min_distance since the last lap.
    """

    def __init__(self, leave_radius=3.0, return_radius=1.5, min_distance=10.0):
        self.leave_radius = leave_radius
        self.return_radius = return_radius
        self.min_distance = min_distance
        self.start = None
        self.last = None
        self.distance = 0.0   # driven since the start or the last lap
        self.away = False
        self.laps = 0

    def update(self, x, y):
        """Feed one position; returns True when it completes a lap."""
        if self.start is None:
            self.start = self.last = (x, y)
            return False
        self.distance += math.hypot(x - self.last[0], y - self.last[1])
        self.last = (x, y)
        r = math.hypot(x - self.start[0], y - self.start[1])
        if r > self.leave_radius:
            self.away = True
        elif self.away and r < self.return_radius and self.distance >= self.min_distance:
            self.laps += 1
            self.away = False
            self.distance = 0.0
            return True
        return False


class TrackWidth:
    """Running estimate of the track width, from scans that see both walls.

    One wall dropping out of view (a junction, an opening, the outside of a wide corner) leaves only the other
    to steer on, and the car then holds half the track width from it. A fixed width breaks on tracks that are
    wider or narrower than it (Berlin: ~2.4 m sections and a narrow hairpin, against the old fixed 1.2 m): the
    car steered at full lock towards the remaining wall. The estimate follows the walls as they are.
    """

    def __init__(self, initial=1.2, alpha=0.2, lo=0.5, hi=6.0):
        self.width = initial      # m, until both walls have been seen
        self.alpha = alpha        # weight of each new measurement (exponential smoothing)
        self.lo, self.hi = lo, hi

    def update(self, d_left, d_right):
        """Feed one scan's wall distances (None: that wall is not visible); returns the current width."""
        if d_left is not None and d_right is not None:
            w = min(max(d_left + d_right, self.lo), self.hi)
            self.width += self.alpha * (w - self.width)
        return self.width


def centring_error(d_left, d_right, width):
    """How far the car is right of the centre [m] (positive: steer left), from the wall distances
    (None: not visible). With one wall it holds half the track width from it; with none, 0 (straight on)."""
    if d_left is not None and d_right is not None:
        return (d_left - d_right) / 2.0
    if d_left is not None:
        return d_left - width / 2.0
    if d_right is not None:
        return width / 2.0 - d_right
    return 0.0


def front_speed_factor(front, slow_distance, min_factor):
    """Speed factor for the clear distance straight ahead: 1 from slow_distance on, down to min_factor at the
    wall (a hairpin's outside wall comes up fast; slower, the car turns tighter and stays off it)."""
    if slow_distance <= 0.0:
        return 1.0
    return min(1.0, max(min_factor, front / slow_distance))


def corner_turn(front, open_left, open_right, start_distance, full_distance):
    """Steering, as a fraction of full lock (positive: left), towards the more open side once the wall straight
    ahead is within start_distance, rising to full lock at full_distance; 0 farther out or with both sides alike.

    On the way into a hairpin the inside wall drops out of view, and holding a distance from the outside wall
    alone drove the car straight on until that wall, now across its path, was too close to turn away from
    (Berlin). The open side is where the track goes; the closer the wall ahead, the harder it turns there,
    and the more this takes over from centring (the caller blends the two by its magnitude). Full lock must
    come while the wall is still about a turning radius plus the car's length away (~1.2 m): ramping to it
    only at the wall turned too late and clipped the hairpin's outside wall.
    """
    if start_distance <= full_distance or front >= start_distance or open_left == open_right:
        return 0.0
    urgency = min(1.0, (start_distance - front) / (start_distance - full_distance))
    return urgency * (1.0 if open_left > open_right else -1.0)


def rate_limit(previous, target, max_rate, dt):
    """target, moved from previous by at most max_rate * dt (no limit without a previous command, a positive
    dt or a positive max_rate)."""
    if previous is None or dt <= 0.0 or max_rate <= 0.0:
        return target
    step = max_rate * dt
    return min(previous + step, max(previous - step, target))


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follow_node')
        p = self.declare_parameter
        self.speed = p('speed', 1.0).value                        # m/s on straights
        # rad of steering per m off centre, and per (m/s) of drift towards a wall. Was 1.05 / 0.10: it overcorrected
        # in tight corners (57 / 39 steering reversals a minute on Porto / Berlin, 33 / 27 now); more kd only
        # added the scan noise's jitter (73 a minute at kd 0.25)
        self.kp = p('kp', 0.8).value
        self.kd = p('kd', 0.05).value
        self.lookahead = p('lookahead', 0.8).value                # m ahead at which wall distances are projected
        self.beam_angle = math.radians(p('beam_angle_deg', 45.0).value)  # forward beam, from the side beam
        self.max_range = p('max_range', 5.0).value                # beyond this a side counts as "no wall"
        # m: initial track width, until both walls have been seen; then it is measured as the car drives
        self.width = TrackWidth(p('track_width', 1.2).value, p('width_alpha', 0.2).value)
        # Distances to the wall ahead, in track widths (the measured width): what is "close" depends on the
        # track. Tuned on Berlin (2.4 m): start turning at 2.4 m, full lock at 1.2 m, slow from 2.5 m; the same
        # metres on Porto (~1.3 m wide) put ordinary corners at full lock and made the car jerky.
        self.turn_start_widths = p('turn_start_widths', 1.0).value   # start turning to the open side
        self.turn_full_widths = p('turn_full_widths', 0.5).value     # ... at full lock
        self.front_slow_widths = p('front_slow_widths', 1.05).value  # start slowing down
        self.min_turn_full = p('min_turn_full', 0.5).value           # m: floor of the full-lock distance
        self.min_turn_span = p('min_turn_span', 0.3).value           # m: start at least this before full lock
        self.front_min_factor = p('front_min_factor', 0.4).value     # of speed, with the wall right ahead
        # rad/s the steering may change: a wall dropping in or out of view, the corner turn starting, or the
        # PD overcorrecting made the steering snap in tight corners. The sim's servo turns at ~3.2 rad/s.
        self.max_steer_rate = p('max_steer_rate', 1.5).value
        self.prev_steer = None
        self.open_angle = math.radians(p('open_angle_deg', 60.0).value)  # beams that compare the two sides
        self.corner_slowdown = p('corner_slowdown', 0.5).value    # fraction of speed shed at full steering
        self.max_steer = p('max_steering_angle', 0.524).value     # rad, steering is clipped to this; default mirrors
                                                                  # config/vehicle.yaml, which wall_follow.launch.py loads
        self.laps = p('laps', 0).value                            # stop after this many laps, 0 = never
        pose_topic = p('pose_topic', '/tracked_pose').value       # for lap counting only
        self.debug = p('debug', False).value                      # log wall distances and steering

        self.prev_error = None
        self.prev_time = None
        self.prev_walls = None   # which walls were visible last scan (left, right)
        self.lap_counter = LapCounter(p('lap_leave_radius', 3.0).value, p('lap_return_radius', 1.5).value,
                                      p('lap_min_distance', 10.0).value)
        self.done = False
        self.started = time.monotonic()
        self.pose_warned = False

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        if self.laps > 0:
            self.create_subscription(PoseStamped, pose_topic, self.pose_callback, 10)
            self.get_logger().info(f'Wall following at {self.speed:.1f} m/s, stopping after {self.laps} '
                                   f'lap(s) counted on {pose_topic}')
        else:
            self.get_logger().info(f'Wall following at {self.speed:.1f} m/s until stopped')

    def range_at(self, msg, ranges, angle):
        """Median range over +/-1 deg around angle (rad, positive = left); max_range if no return."""
        i = int(round((angle - msg.angle_min) / msg.angle_increment))
        half = max(1, int(round(math.radians(1.0) / msg.angle_increment)))
        window = ranges[max(0, i - half):i + half + 1]
        return float(np.median(window)) if window.size else self.max_range

    def wall_distance(self, msg, ranges, side):
        """Distance to the wall on one side (side = +1 left, -1 right), projected lookahead metres ahead.

        Classic two-beam method: b is perpendicular to the car, a is beam_angle further forward.
        alpha is the wall's angle relative to the car; positive when the wall falls away ahead.
        Returns None if either beam sees no wall.
        """
        theta = self.beam_angle
        b = self.range_at(msg, ranges, side * math.pi / 2)
        a = self.range_at(msg, ranges, side * (math.pi / 2 - theta))
        if a >= self.max_range or b >= self.max_range:
            return None
        alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
        # negative once the wall curves across the car's path (a hairpin's outside): it is right there
        return max(0.0, b * math.cos(alpha) + self.lookahead * math.sin(alpha))

    def scan_callback(self, msg):
        if self.done:
            return
        ranges = np.array(msg.ranges, dtype=float)
        ranges[~np.isfinite(ranges)] = self.max_range       # no return -> "far"
        ranges = np.clip(ranges, 0.0, self.max_range)

        d_left = self.wall_distance(msg, ranges, +1)
        d_right = self.wall_distance(msg, ranges, -1)

        # Error: how far the car is right of centre (positive -> steer left, which is positive steering)
        width = self.width.update(d_left, d_right)
        error = centring_error(d_left, d_right, width)

        # PD control, with the derivative from sensor timestamps. When a wall drops in or out of view
        # the error jumps; skip the derivative for that scan, or it kicks the steering to full lock.
        walls = (d_left is not None, d_right is not None)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        d_error = 0.0
        if self.prev_time is not None and t > self.prev_time and walls == self.prev_walls:
            d_error = float(np.clip((error - self.prev_error) / (t - self.prev_time), -1.0, 1.0))
        self.prev_error, self.prev_time, self.prev_walls = error, t, walls

        front = self.range_at(msg, ranges, 0.0)
        full = max(self.min_turn_full, self.turn_full_widths * width)
        start = max(full + self.min_turn_span, self.turn_start_widths * width)
        turn = corner_turn(front, self.range_at(msg, ranges, self.open_angle),
                           self.range_at(msg, ranges, -self.open_angle), start, full)
        # the closer the wall ahead, the more the turn takes over from the centring: at a hairpin the inside
        # wall's tip reappears close on the inside and centring alone steered away from the turn
        pd = float(np.clip(self.kp * error + self.kd * d_error, -self.max_steer, self.max_steer))
        steering = (1.0 - abs(turn)) * pd + turn * self.max_steer
        steering = rate_limit(self.prev_steer, steering, self.max_steer_rate, t - self.prev_steer_t
                              if self.prev_steer is not None else 0.0)
        self.prev_steer, self.prev_steer_t = steering, t
        speed = self.speed * (1.0 - self.corner_slowdown * abs(steering) / self.max_steer)
        speed *= front_speed_factor(front, self.front_slow_widths * width, self.front_min_factor)
        self.publish(speed, steering)

        if self.laps > 0 and self.lap_counter.start is None and not self.pose_warned \
                and time.monotonic() - self.started > 5.0:
            self.get_logger().warn('No pose received yet: laps are not being counted')
            self.pose_warned = True

        if self.debug:
            fmt = lambda d: f'{d:.2f}' if d is not None else ' -- '  # noqa: E731
            self.get_logger().info(f'wall L {fmt(d_left)} R {fmt(d_right)}  width {width:.2f} m  front {front:.2f} m'
                                   f'  turn {turn:+.2f}'
                                   f'  error {error:+.2f} m  d_error {d_error:+.2f}'
                                   f'  steer {steering:+.3f} rad  speed {speed:.2f} m/s', throttle_duration_sec=0.5)

    def pose_callback(self, msg):
        if self.done or not self.lap_counter.update(msg.pose.position.x, msg.pose.position.y):
            return
        self.get_logger().info(f'Lap {self.lap_counter.laps}/{self.laps} complete')
        if self.lap_counter.laps >= self.laps:
            self.done = True

    def publish(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        self.drive_pub.publish(msg)

    def stop(self):
        self.get_logger().info('Stopping: sending zero speed and steering')
        for _ in range(3):   # repeat and wait, so the commands are delivered before the node exits
            self.publish(0.0, 0.0)
            time.sleep(0.05)


def main():
    # Handle Ctrl+C ourselves: rclpy's default handler shuts ROS down first, so stop() could
    # never publish and the car would coast until the adapter's command timeout
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = WallFollower()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)   # short waits so Ctrl+C is noticed even with no scans arriving
    except KeyboardInterrupt:
        pass
    node.stop()                 # always leave the car stopped
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
