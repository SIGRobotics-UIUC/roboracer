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
import signal
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


def measures_width(alpha_left, alpha_right, max_angle):
    """Whether this scan's wall distances measure the track width: both walls seen and nearly parallel to the car
    (wall angles in rad, None: not seen).

    Approaching a hairpin's outside wall, the side beams hit that wall ahead: both "walls" closed in and the
    width read 2.0 -> 0.5 m (SRL 2025 ICRA's first hairpin). The turn distances scale with the width, so the
    corner turn faded out (half lock -> 0) as the car reached the wall and it drove straight into it.
    """
    if alpha_left is None or alpha_right is None:
        return False
    return abs(alpha_left) <= max_angle and abs(alpha_right) <= max_angle


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


def side_openness(ranges, angles, lo, hi):
    """Mean range over the beams lo..hi rad to the left and to the right (angles positive left): how open each
    side is. Single beams at +/-60 deg both hit a hairpin's rounded outside wall at about the same range, and the
    turn picked a side by noise (SRL 2025 ICRA: right, then left at 0.8 m from the wall); a sector also sees
    the opening where the inside wall ends."""
    left = ranges[(angles >= lo) & (angles <= hi)]
    right = ranges[(angles <= -lo) & (angles >= -hi)]
    return (float(left.mean()) if left.size else 0.0), (float(right.mean()) if right.size else 0.0)


def hairpin_side(active, open_side, open_range, front, width, open_widths, start_distance, armed=True,
                 turned=0.0, max_turn=math.pi, other_way=False):
    """The side (+1 left, -1 right) of a hairpin being turned into, or 0 for none.

    A hairpin starts when the inside wall has ended beside the car: the perpendicular range on the open side
    exceeds open_widths track widths, with the wall ahead within start_distance (only while armed). Its side then
    stays fixed until the way ahead is clear again: mid-turn the perpendicular beam meets the return lane's
    outside wall, and dropping the hairpin there unwound the steering halfway round. It also ends once the car has
    turned max_turn rad into it: where nothing ahead is ever start_distance away (a cramped hook), holding it
    kept the car circling at full lock (SRL 2024 CDC). And it ends when other_way: the other side has become the
    more open one, or the car is not turning as a hairpin would (the caller decides). An S-bend starts like a hairpin,
    round a wall end, but then goes the other way, and a wall curving away looks like one ending; held to the
    hairpin's side the car turned on into a dead end (SRL 2024 CDC) or the wrong way at the next corner (SRL 2025
    ICRA's last bump).
    """
    if active and (turned >= max_turn or other_way):
        return 0
    if active and front < start_distance:
        return active
    if armed and front < start_distance and open_range > open_widths * width:
        return open_side
    return 0


def hairpin_tip(ranges, angles, side, laser_x, max_range, max_ahead=0.6):
    """The closest scan point on the hairpin's side (+1 left, -1 right) no more than max_ahead m ahead of base_link,
    in base_link (rear axle) coordinates: the inside wall's end the car turns round. None if there is none."""
    x = ranges * np.cos(angles) + laser_x
    y = ranges * np.sin(angles)
    mask = (side * y > 0.0) & (ranges < max_range) & (x <= max_ahead)
    if not mask.any():
        return None
    i = np.flatnonzero(mask)[np.argmin(np.hypot(x[mask], y[mask]))]
    return float(x[i]), float(y[i])


def orbit_steering(tip, side, radius, wheelbase, gain, max_steer):
    """Steering (rad) that circles the point tip (base_link coordinates) on side at radius: the curvature of that
    circle, corrected by gain * (distance - radius). None while the tip is still ahead of the rear axle: turning
    then swings the car's inside across the wall's end.

    Full lock as soon as a hairpin's inside wall ended at the scanner put the turning circle across that wall's
    far corner (SRL 2024 CDC: the rear half of the car was still beside it, and it clipped the inside). Circling
    the end at the distance the car has to it passes it at that distance, with the return lane on the far side.
    Closer than radius it turns less, not more: round a fat island end (Porto) the nearest point is its surface,
    and turning at the curvature of the distance to it pulled the car in against it.
    """
    if tip[0] > 0.0:
        return None
    curvature = max(0.0, 1.0 / radius + gain * (math.hypot(*tip) - radius))
    return side * min(math.atan(wheelbase * curvature), max_steer)


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
        # measure the width only where both walls are within this angle of parallel to the car: corners and a
        # wall ahead otherwise read as a narrower track
        self.width_max_angle = math.radians(p('width_max_angle_deg', 20.0).value)
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
        # beams between these angles on each side compare how open the two sides are (the turn goes to the more open)
        self.open_min = math.radians(p('open_min_deg', 20.0).value)
        self.open_max = math.radians(p('open_max_deg', 110.0).value)
        # A hairpin (the inside wall ends beside the car, with a wall ahead): a U-turn round the end of that wall,
        # which has to start there, long before the usual distances, which scale with the track width (SRL 2025
        # ICRA's top-left hairpin, 1.35 m lane: the wall ahead was 3.2 m away where the turn had to start; it began
        # at 1.35 m and hit the wall). The car circles the wall's end at the distance it has to it.
        self.hairpin_open_widths = p('hairpin_open_widths', 1.0).value    # inside wall "ended": side range > this
        # m: a hairpin needs the wall ahead within this (a side opening with the way ahead clear is not one); below
        # the scan's max_range, so that "the way ahead is clear again" (which ends it) can come
        self.hairpin_max_start = p('hairpin_max_start', 4.5).value
        self.laser_x = p('laser_x', 0.2733).value                  # m, scanner ahead of base_link (sim mount)
        # m: smallest radius at which to circle the wall's end, beyond the car's turning radius at full lock
        self.orbit_margin = p('orbit_margin', 0.05).value
        self.orbit_gain = p('orbit_gain', 6.0).value               # 1/m^2: curvature per m off the orbit radius
        self.orbit_radius = None                                   # m, set when circling starts
        # deg turned into a hairpin (from the commanded speed and steering) that ends it: a U-turn is done. Commands,
        # not measured motion, so a car pinned against a wall is released too. It re-arms once the way ahead is clear.
        self.hairpin_max_turn = math.radians(p('hairpin_max_turn_deg', 180.0).value)
        # A hairpin ends once the car has turned hairpin_min_turn_deg into it and the other side has been the more
        # open one for hairpin_other_way_s (an S-bend, not a U-turn). Not before that turn: early in a deep U-turn
        # the closed end beyond the wall is the more open side, and the return lane is still behind the scan.
        self.hairpin_min_turn = math.radians(p('hairpin_min_turn_deg', 80.0).value)
        self.hairpin_other_way = p('hairpin_other_way_s', 0.2).value
        # A hairpin is dropped if after hairpin_reach m in it the car has turned less than hairpin_reach_turn_deg: a
        # wall curving away or a gap in the scan, not a wall end to turn round. Held on, it turned the car the wrong
        # way at the next corner (SRL 2025 ICRA's last bump). That side then starts no new one until it has closed.
        self.hairpin_reach = p('hairpin_reach', 1.5).value
        self.hairpin_reach_turn = math.radians(p('hairpin_reach_turn_deg', 45.0).value)
        self.hairpin_driven = 0.0     # m driven in the hairpin so far (from the commanded speed)
        self.hairpin_dropped = 0      # side of a hairpin dropped that way, until that side has closed again
        self.other_way_since = None   # scan time since which the other side has been the more open
        # m: a hairpin needs the inside wall's end this close on the open side. Beside a far wall end the usual corner
        # turn has room, and circling a far one took the car round an island into the wrong lane (SRL 2024 CDC).
        self.hairpin_tip_max = p('hairpin_tip_max', 1.2).value
        self.hairpin = 0    # side of the hairpin being turned into (+1 left, -1 right), 0 for none
        self.hairpin_turned = 0.0   # rad turned into it so far
        self.hairpin_armed = True   # False after a turn-limited hairpin, until the way ahead is clear
        self.prev_speed = 0.0
        self.corner_slowdown = p('corner_slowdown', 0.5).value    # fraction of speed shed at full steering
        self.wheelbase = p('wheelbase', 0.33).value               # m; config/vehicle.yaml, loaded by the launch
        self.max_steer = p('max_steering_angle', 0.524).value     # rad, steering is clipped to this; default mirrors
                                                                  # config/vehicle.yaml, which wall_follow.launch.py loads
        self.min_orbit = self.max_steer_radius() + self.orbit_margin
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
        """Distance to the wall on one side (side = +1 left, -1 right), projected lookahead metres ahead, the
        wall's angle relative to the car (rad, positive when the wall falls away ahead), and the distance to it
        beside the car (perpendicular to the wall).

        Classic two-beam method: b is perpendicular to the car, a is beam_angle further forward.
        Returns (None, None, None) if either beam sees no wall.
        """
        theta = self.beam_angle
        b = self.range_at(msg, ranges, side * math.pi / 2)
        a = self.range_at(msg, ranges, side * (math.pi / 2 - theta))
        if a >= self.max_range or b >= self.max_range:
            return None, None, None
        alpha = math.atan2(a * math.cos(theta) - b, a * math.sin(theta))
        beside = b * math.cos(alpha)
        # negative once the wall curves across the car's path (a hairpin's outside): it is right there
        return max(0.0, beside + self.lookahead * math.sin(alpha)), alpha, beside

    def max_steer_radius(self):
        """m, the rear axle's turning radius at full lock."""
        return self.wheelbase / math.tan(self.max_steer)

    def turn_distances(self, width):
        """Distances to the wall ahead [m] at which the corner turn starts and reaches full lock."""
        full = max(self.min_turn_full, self.turn_full_widths * width)
        return max(full + self.min_turn_span, self.turn_start_widths * width), full

    def scan_callback(self, msg):
        if self.done:
            return
        ranges = np.array(msg.ranges, dtype=float)
        ranges[~np.isfinite(ranges)] = self.max_range       # no return -> "far"
        ranges = np.clip(ranges, 0.0, self.max_range)

        d_left, alpha_left, beside_left = self.wall_distance(msg, ranges, +1)
        d_right, alpha_right, beside_right = self.wall_distance(msg, ranges, -1)
        front = self.range_at(msg, ranges, 0.0)

        # The track width, from the distances to the walls beside the car: the projected ones (lookahead m ahead)
        # read a wall falling away as a wider track (Porto: 2.95 m for ~1.8 m), and the turn distances scale with it
        width = self.width.width
        measured = measures_width(alpha_left, alpha_right, self.width_max_angle)
        if measured:
            width = self.width.update(beside_left, beside_right)
        # Error: how far the car is right of centre (positive -> steer left, which is positive steering)
        error = centring_error(d_left, d_right, width)

        # PD control, with the derivative from sensor timestamps. When a wall drops in or out of view
        # the error jumps; skip the derivative for that scan, or it kicks the steering to full lock.
        walls = (d_left is not None, d_right is not None)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        d_error = 0.0
        if self.prev_time is not None and t > self.prev_time and walls == self.prev_walls:
            d_error = float(np.clip((error - self.prev_error) / (t - self.prev_time), -1.0, 1.0))
        self.prev_error, self.prev_time, self.prev_walls = error, t, walls

        start, full = self.turn_distances(width)
        angles = msg.angle_min + msg.angle_increment * np.arange(ranges.size)
        open_left, open_right = side_openness(ranges, angles, self.open_min, self.open_max)
        open_side = 1 if open_left > open_right else -1
        hairpin_start = self.hairpin_max_start
        if self.hairpin and self.prev_steer is not None and t > self.prev_steer_t:
            # rotation into the hairpin since the last scan, at the last command (kinematic bicycle)
            yaw_rate = self.prev_speed * math.tan(self.prev_steer) / self.wheelbase
            # at most 0.2 s of it: after a gap in the scans the last command says little about the turn made
            step = min(t - self.prev_steer_t, 0.2)
            self.hairpin_turned += max(0.0, self.hairpin * yaw_rate) * step
            self.hairpin_driven += self.prev_speed * step
        if front >= hairpin_start or (not self.hairpin and measured):
            self.hairpin_armed = True    # the way ahead is clear, or back between two parallel walls
        if not self.hairpin or open_side == self.hairpin or self.hairpin_turned < self.hairpin_min_turn:
            self.other_way_since = None
        elif self.other_way_since is None:
            self.other_way_since = t
        other_way = (self.hairpin != 0 and self.other_way_since is not None
                     and t - self.other_way_since >= self.hairpin_other_way)
        no_progress = (self.hairpin != 0 and self.hairpin_driven >= self.hairpin_reach
                       and self.hairpin_turned < self.hairpin_reach_turn)
        open_range = self.range_at(msg, ranges, open_side * math.pi / 2)
        if self.hairpin_dropped and (self.range_at(msg, ranges, self.hairpin_dropped * math.pi / 2)
                                     <= self.hairpin_open_widths * width):
            self.hairpin_dropped = 0   # a wall beside the car on that side again
        end = None if self.hairpin else hairpin_tip(ranges, angles, open_side, self.laser_x, self.max_range)
        wall_end_near = end is not None and math.hypot(*end) <= self.hairpin_tip_max
        side = hairpin_side(self.hairpin, open_side, open_range, front, width, self.hairpin_open_widths, hairpin_start,
                            self.hairpin_armed and wall_end_near and open_side != self.hairpin_dropped,
                            self.hairpin_turned, self.hairpin_max_turn, other_way or no_progress)
        if self.hairpin and not side and (other_way or self.hairpin_turned >= self.hairpin_max_turn):
            self.hairpin_armed = False
        if self.hairpin and not side and no_progress:
            self.hairpin_dropped = self.hairpin
        if side != self.hairpin:
            self.hairpin_turned, self.hairpin_driven, self.orbit_radius = 0.0, 0.0, None
        self.hairpin = side
        orbit = None
        if self.hairpin:
            open_left, open_right = (1.0, 0.0) if self.hairpin > 0 else (0.0, 1.0)   # its side, held
            tip = hairpin_tip(ranges, angles, self.hairpin, self.laser_x, self.max_range)
            if tip is not None and tip[0] <= 0.0:
                if self.orbit_radius is None:
                    self.orbit_radius = max(self.min_orbit, math.hypot(*tip))
                orbit = orbit_steering(tip, self.hairpin, self.orbit_radius, self.wheelbase, self.orbit_gain,
                                       self.max_steer)
        # the closer the wall ahead, the more the turn takes over from the centring: at a hairpin the inside
        # wall's tip reappears close on the inside and centring alone steered away from the turn
        turn = corner_turn(front, open_left, open_right, start, full)
        pd = float(np.clip(self.kp * error + self.kd * d_error, -self.max_steer, self.max_steer))
        steering = (1.0 - abs(turn)) * pd + turn * self.max_steer
        if orbit is not None:   # circling a hairpin's wall end; the corner turn still tightens it near the wall ahead
            steering = self.hairpin * max(abs(orbit), abs(turn) * self.max_steer)
        steering = rate_limit(self.prev_steer, steering, self.max_steer_rate, t - self.prev_steer_t
                              if self.prev_steer is not None else 0.0)
        self.prev_steer, self.prev_steer_t = steering, t
        speed = self.speed * (1.0 - self.corner_slowdown * abs(steering) / self.max_steer)
        speed *= front_speed_factor(front, self.front_slow_widths * width, self.front_min_factor)
        self.prev_speed = speed
        self.publish(speed, steering)

        if self.laps > 0 and self.lap_counter.start is None and not self.pose_warned \
                and time.monotonic() - self.started > 5.0:
            self.get_logger().warn('No pose received yet: laps are not being counted')
            self.pose_warned = True

        if self.debug:
            fmt = lambda d: f'{d:.2f}' if d is not None else ' -- '  # noqa: E731
            self.get_logger().info(f'wall L {fmt(d_left)} R {fmt(d_right)}  width {width:.2f} m  front {front:.2f} m'
                                   f'  turn {turn:+.2f}{" hairpin" if self.hairpin else ""}'
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
    # ros2 launch forwards a second SIGINT after the terminal's: it interrupted stop() midway (a traceback, and the
    # car left without its zero commands). The first SIGINT ends the loop; this ignores any after it at once.
    def first_sigint(signum, frame):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, first_sigint)
    node = WallFollower()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)   # short waits so Ctrl+C is noticed even with no scans arriving
    except KeyboardInterrupt:
        pass
    signal.signal(signal.SIGINT, signal.SIG_IGN)   # also when the loop ended on its own (laps done)
    node.stop()                 # always leave the car stopped
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
