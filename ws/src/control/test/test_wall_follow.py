"""Tests for wall_follow_node: LapCounter unit tests, plus a functional test that launches
wall_follow.launch.py, publishes synthetic corridor scans and poses, and checks /drive.

Needs a sourced ROS 2 environment with the control package built:
    python3 -m pytest ws/src/control/test
"""
import math
import os
import random
import signal
import subprocess
import time

import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
from ackermann_msgs.msg import AckermannDriveStamped  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

from builtin_interfaces.msg import Time  # noqa: E402
from control.wall_follow_node import (LapCounter, TrackWidth, WallFollower, centring_error, corner_turn,  # noqa: E402
                                      front_speed_factor, hairpin_side, hairpin_tip, measures_width,
                                      orbit_steering, rate_limit, side_openness)

SPEED = 1.0
DOMAIN = random.randint(1, 101)  # not 0: the live stack (bridge, sim car) runs there
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'vehicle.yaml')) as f:
    MAX_STEER = yaml.safe_load(f)['/**']['ros__parameters']['max_steering_angle']


def circle_lap(radius=4.0, n=200):
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return [(radius * math.cos(a), radius * math.sin(a)) for a in t]


def test_lap_counter_counts_returns():
    lc = LapCounter()
    done = [lc.update(x, y) for _ in range(2) for x, y in circle_lap()] + [lc.update(4.0, 0.0)]
    assert lc.laps == 2
    assert sum(done) == 2


def test_lap_counter_ignores_jitter_at_start():
    lc = LapCounter()
    for i in range(500):  # sitting at the start with pose noise
        lc.update(0.05 * math.sin(i), 0.05 * math.cos(i))
    assert lc.laps == 0


def test_lap_counter_needs_min_distance():
    lc = LapCounter(leave_radius=3.0, return_radius=1.5, min_distance=10.0)
    for x in (0.0, 1.0, 2.0, 3.5, 2.0, 1.0, 0.0):  # out 3.5 m and straight back: 7 m < 10 m
        lc.update(x, 0.0)
    assert lc.laps == 0


def test_track_width_is_learned_from_both_walls():
    tw = TrackWidth(initial=1.2)
    assert tw.update(1.0, None) == 1.2                    # one wall: nothing to measure yet
    for _ in range(30):
        tw.update(1.2, 1.2)                               # a 2.4 m wide track (Berlin)
    assert tw.width == pytest.approx(2.4, abs=0.01)
    for _ in range(30):
        tw.update(0.4, 0.5)                               # into a 0.9 m hairpin
    assert tw.width == pytest.approx(0.9, abs=0.01)
    assert TrackWidth(lo=0.5, hi=6.0).update(0.1, 0.1) > 0.2   # implausible readings are clamped


def test_width_is_measured_only_beside_parallel_walls():
    deg = math.radians
    assert measures_width(deg(3), deg(-5), deg(20))               # a straight: measure
    assert not measures_width(deg(-30), deg(2), deg(20))          # a wall angled across the car's path
    assert not measures_width(None, deg(2), deg(20))              # one wall out of view


def test_one_wall_holds_half_the_learned_width():
    # 2.4 m wide, car 1.3 m from the left wall, right wall out of view (an opening): nearly centred, so a small
    # correction; with the old fixed 1.2 m it read 0.7 m off centre and steered at full lock into the left wall
    assert centring_error(1.3, None, 2.4) == pytest.approx(0.1)
    assert centring_error(1.3, None, 1.2) == pytest.approx(0.7)
    assert centring_error(None, 0.3, 1.0) == pytest.approx(0.2)
    assert centring_error(1.0, 1.4, 2.4) == pytest.approx(-0.2)
    assert centring_error(None, None, 2.4) == 0.0


def test_slows_down_for_a_wall_ahead():
    assert front_speed_factor(5.0, 2.0, 0.5) == 1.0
    assert front_speed_factor(1.5, 2.0, 0.5) == pytest.approx(0.75)
    assert front_speed_factor(1.2, 2.5, 0.4) == pytest.approx(0.48)    # the defaults at full turning lock
    assert front_speed_factor(0.2, 2.0, 0.5) == 0.5
    assert front_speed_factor(0.2, 0.0, 0.5) == 1.0       # disabled


def test_turns_towards_the_open_side_when_the_wall_ahead_is_close():
    # a left hairpin: the inside (left) is open, the outside wall comes up ahead
    assert corner_turn(3.0, 5.0, 1.5, 2.4, 1.2) == 0.0                # wall still far
    assert corner_turn(1.8, 5.0, 1.5, 2.4, 1.2) == pytest.approx(0.5)  # halfway in: half lock to the left
    assert corner_turn(1.2, 5.0, 1.5, 2.4, 1.2) == pytest.approx(1.0)  # full lock while the wall is 1.2 m away
    assert corner_turn(0.4, 1.5, 5.0, 2.4, 1.2) == pytest.approx(-1.0)  # right hairpin: full lock to the right
    assert corner_turn(0.5, 2.0, 2.0, 2.4, 1.2) == 0.0                # both sides alike: no preference
    assert corner_turn(0.5, 5.0, 1.0, 1.0, 1.2) == 0.0                # disabled (start not beyond full)


def test_side_openness_is_the_mean_range_of_each_side_sector():
    angles = np.radians(np.arange(-135.0, 135.5, 0.5))
    ranges = np.where(angles > 0, 3.0, 1.0)                    # open on the left
    left, right = side_openness(ranges, angles, math.radians(20), math.radians(110))
    assert left == pytest.approx(3.0) and right == pytest.approx(1.0)


def test_hairpin_starts_where_the_inside_wall_ends_and_holds_its_side():
    w, start = 1.35, 3.4
    assert hairpin_side(0, 1, 0.7, 3.0, w, 1.0, start) == 0     # inside wall still beside the car
    assert hairpin_side(0, 1, 2.1, 4.0, w, 1.0, start) == 0     # wall ended, but nothing ahead to turn from
    assert hairpin_side(0, 1, 2.1, 3.2, w, 1.0, start) == 1     # wall ended, wall ahead: left hairpin
    assert hairpin_side(1, -1, 0.5, 2.0, w, 1.0, start) == 1    # mid-turn: the side is held
    assert hairpin_side(1, -1, 0.5, 3.6, w, 1.0, start) == 0    # way ahead clear again: done
    assert hairpin_side(0, -1, 2.1, 3.2, w, 1.0, start) == -1   # right hairpin
    assert hairpin_side(0, 1, 2.1, 3.2, w, 1.0, start, armed=False) == 0        # not armed: none starts


def test_hairpin_ends_after_its_turn_or_when_the_track_goes_the_other_way():
    w, start = 1.35, 3.4
    assert hairpin_side(1, 1, 2.1, 2.0, w, 1.0, start, turned=math.radians(170)) == 1
    assert hairpin_side(1, 1, 2.1, 2.0, w, 1.0, start, turned=math.pi) == 0     # a U-turn's worth: over, even
    assert hairpin_side(1, 1, 2.1, 2.0, w, 1.0, start, other_way=True) == 0     # ... with the start conditions met


def test_hairpin_tip_is_the_closest_point_on_its_side_not_far_ahead():
    angles = np.radians(np.arange(-135.0, 135.5, 0.5))
    ranges = np.full(angles.size, 5.0)
    ranges[np.argmin(abs(angles - math.radians(-100)))] = 0.8     # the wall's end, behind-right of the scanner
    ranges[np.argmin(abs(angles - math.radians(-20)))] = 0.5      # something ahead-right: too far ahead to be it
    x, y = hairpin_tip(ranges, angles, -1, 0.27, 5.0)
    assert x == pytest.approx(0.27 + 0.8 * math.cos(math.radians(-100)), abs=0.01) and y < 0.0
    assert hairpin_tip(ranges, angles, 1, 0.27, 5.0) is None       # nothing on the left


def test_orbit_steering_circles_the_wall_end_at_the_radius():
    wb, lock = 0.33, 0.524
    on_circle = math.atan(wb / 0.8)
    assert orbit_steering((0.2, -0.8), -1, 0.8, wb, 6.0, lock) is None       # end still ahead of the rear axle
    assert orbit_steering((0.0, -0.8), -1, 0.8, wb, 6.0, lock) == pytest.approx(-on_circle)
    assert abs(orbit_steering((0.0, -0.9), -1, 0.8, wb, 6.0, lock)) > on_circle     # drifted out: tighter
    assert abs(orbit_steering((0.0, -0.7), -1, 0.8, wb, 6.0, lock)) < on_circle     # too close: wider
    assert orbit_steering((0.0, -0.3), -1, 0.8, wb, 6.0, lock) == 0.0               # far too close: straight on
    assert orbit_steering((0.0, 2.0), 1, 0.62, wb, 6.0, lock) == pytest.approx(lock)   # at most full lock


def test_steering_rate_is_limited():
    assert rate_limit(None, 0.5, 1.5, 0.03) == 0.5                    # first command: as asked
    assert rate_limit(0.0, 0.5, 1.5, 0.1) == pytest.approx(0.15)      # a snap to 0.5 rad moves 0.15 in 0.1 s
    assert rate_limit(0.2, -0.5, 1.5, 0.1) == pytest.approx(0.05)
    assert rate_limit(0.1, 0.12, 1.5, 0.1) == pytest.approx(0.12)     # small changes pass through
    assert rate_limit(0.0, 0.5, 0.0, 0.1) == 0.5                      # disabled


def corridor_scan(left, right, n=1080, angle_min=-2.35619, inc=0.004363323, range_max=10.0):
    """Scan of two walls parallel to the car, `left` m to its left and `right` m to its right."""
    msg = LaserScan()
    msg.header.frame_id = 'laser'
    msg.angle_min, msg.angle_increment = angle_min, inc
    msg.angle_max = angle_min + inc * (n - 1)
    msg.range_min, msg.range_max = 0.06, range_max
    s = np.sin(angle_min + inc * np.arange(n))
    with np.errstate(divide='ignore'):
        r = np.where(s > 0, left / s, np.where(s < 0, right / -s, np.inf))
    msg.ranges = np.minimum(r, range_max).astype(float).tolist()
    return msg


def scene_scan(segments, n=1080, angle_min=-2.35619, inc=0.004363323, range_max=10.0):
    """Scan of wall segments ((x1, y1), (x2, y2)) in the laser frame (x ahead, y left)."""
    msg = LaserScan()
    msg.header.frame_id = 'laser'
    msg.angle_min, msg.angle_increment = angle_min, inc
    msg.angle_max = angle_min + inc * (n - 1)
    msg.range_min, msg.range_max = 0.06, range_max
    a = angle_min + inc * np.arange(n)
    dx, dy = np.cos(a), np.sin(a)
    r = np.full(n, np.inf)
    for (x1, y1), (x2, y2) in segments:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (x1 * ey - y1 * ex) / den          # range along the beam
            u = (x1 * dy - y1 * dx) / den          # position along the segment
        hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        r[hit] = np.minimum(r[hit], t[hit])
    msg.ranges = np.minimum(r, range_max).astype(float).tolist()
    return msg


def left_hairpin(ahead, half_width=1.0, n_arc=36):
    """A left hairpin, `ahead` m from the outside wall straight ahead: a corridor 2 * half_width wide whose left
    wall is a divider ending at the tip, with the outside wall a semicircle around the tip into the return lane
    (SRL 2025 ICRA's first corner, whose rounded outside wall the side beams see on the way in)."""
    radius = 2.0 * half_width
    tip = ahead - radius                                        # the outside wall's apex is radius beyond the tip
    a = np.linspace(-math.pi / 2, math.pi / 2, n_arc + 1)
    arc = list(zip(tip + radius * np.cos(a), half_width + radius * np.sin(a)))
    return scene_scan([((-5.0, half_width), (tip, half_width)),                  # divider, ends at the tip
                       ((-5.0, -half_width), (tip, -half_width)),                # right wall
                       ((tip, half_width + radius), (-5.0, half_width + radius))]  # return lane's outside wall
                      + list(zip(arc[:-1], arc[1:])))


# SRL 2025 ICRA's top-left hairpin, simplified from a recorded scan (laser frame 1.1 s before the car hit it) and
# extended back along both lanes: a 1.5 m lane, the divider ending 0.6 m ahead, the outside wall looping round ~4 m
# ahead into the return lane on the left.
HAIRPIN_OUTSIDE = [(-6.0, -0.8), (-0.76, -0.76), (1.82, -0.63), (2.78, -0.47), (3.17, -0.32), (3.59, -0.05),
                   (4.01, 0.62), (4.03, 0.78), (3.95, 0.98), (3.64, 1.27), (3.21, 1.51), (2.82, 1.68), (1.8, 1.93),
                   (0.9, 2.3), (-1.5, 2.9), (-6.0, 2.9)]
HAIRPIN_DIVIDER = [(-6.0, 0.8), (-0.76, 0.76), (0.62, 0.67), (0.62, 0.77), (-6.0, 0.9)]
CAR = dict(wheelbase=0.33, laser_x=0.273, rear=-0.085, front=0.415, half_width=0.135)   # base_link = rear axle


def walls(*lines):
    return [seg for line in lines for seg in zip(line[:-1], line[1:])]


def scan_from(segments, x, y, yaw):
    """Scan of world wall segments from the laser of a car at (x, y, yaw) (rear axle)."""
    lx, ly = x + CAR['laser_x'] * math.cos(yaw), y + CAR['laser_x'] * math.sin(yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    to_laser = lambda p: ((p[0] - lx) * c + (p[1] - ly) * s, -(p[0] - lx) * s + (p[1] - ly) * c)  # noqa: E731
    return scene_scan([(to_laser(a), to_laser(b)) for a, b in segments])


def clearance(segments, x, y, yaw):
    """Smallest distance [m] from the car's outline (sampled every ~6 cm) to a wall."""
    c, s = math.cos(yaw), math.sin(yaw)
    u = np.r_[np.linspace(CAR['rear'], CAR['front'], 9), np.linspace(CAR['rear'], CAR['front'], 9),
              np.full(5, CAR['rear']), np.full(5, CAR['front'])]
    v = np.r_[np.full(9, -CAR['half_width']), np.full(9, CAR['half_width']),
              np.linspace(-CAR['half_width'], CAR['half_width'], 5), np.linspace(-CAR['half_width'], CAR['half_width'], 5)]
    pts = np.c_[x + u * c - v * s, y + u * s + v * c]
    best = np.inf
    for a, b in segments:
        a, e = np.array(a), np.array(b) - np.array(a)
        t = np.clip(((pts - a) @ e) / (e @ e), 0.0, 1.0)
        best = min(best, float(np.min(np.linalg.norm(pts - (a + t[:, None] * e), axis=1))))
    return best


def drive_closed_loop(node, segments, done, x, y, yaw, dt=0.025, t_max=15.0):
    """Drive a kinematic bicycle car with the node's /drive commands (servo 3.2 rad/s, the adapter's speed ramp) on
    scans ray-cast from its pose; returns (outcome, seconds, smallest wall clearance). No ROS transport."""
    commands = []
    node.publish = lambda speed, steering: commands.append((speed, steering))
    v, steer, t, min_clear = node.speed, 0.0, 0.0, np.inf
    while t < t_max:
        msg = scan_from(segments, x, y, yaw)
        ns = int(round(t * 1e9))   # whole nanoseconds: rounding sec and nanosec apart stepped the stamp back 1 s
        msg.header.stamp = Time(sec=ns // 1000000000, nanosec=ns % 1000000000)
        node.scan_callback(msg)
        v_cmd, s_cmd = commands[-1]
        steer += float(np.clip(s_cmd - steer, -3.2 * dt, 3.2 * dt))
        v += float(np.clip(v_cmd - v, -6.9 * dt, 2.5 * dt))
        x, y = x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt
        yaw += v / CAR['wheelbase'] * math.tan(steer) * dt
        t += dt
        min_clear = min(min_clear, clearance(segments, x, y, yaw))
        if min_clear < 0.02:
            return 'crash', t, min_clear
        if done(x, y, yaw):
            return 'through', t, min_clear
    return 'timeout', t, min_clear


# SRL 2024 CDC's S-bend, simplified from its saved map (map frame): west along a lane whose right wall is a thin
# peninsula ending at (2.35, 2.25); round its end to the north, then left (west) between an island's tip (1.2, 3.9)
# and the outside wall. Behind the peninsula is a dead-end wedge; the lane itself ends closed, further west.
S_BEND_LOWER = [(7.0, 1.3), (5.0, 1.0), (3.4, 0.65), (2.0, 0.7), (0.5, 1.2), (-0.5, 1.7), (-0.6, 2.1), (0.1, 3.25),
                (1.2, 3.9), (0.1, 3.4), (-1.5, 3.75), (-4.0, 4.0)]
S_BEND_PENINSULA = [(7.0, 3.45), (2.35, 2.25), (2.55, 2.5), (5.6, 3.25)]
S_BEND_OUTSIDE = [(5.6, 3.25), (3.0, 4.75), (2.0, 5.1), (1.0, 5.35), (0.2, 5.5), (-1.5, 5.75), (-4.0, 6.0)]


def wall_end_u_turn(lane, thick, depth, side=-1):
    """A U-turn round the end (x = 0) of a `thick` wall on `side` (-1 right): a `lane` m approach lane heading +x,
    an equally wide return lane beyond the wall, and the outside wall rounding off `depth` m beyond its end."""
    h, y_in, y_back = lane / 2, -lane / 2 - thick, -1.5 * lane - thick
    a = np.linspace(math.pi / 2, -math.pi / 2, 25)
    centre, semi = (h + y_back) / 2, (h - y_back) / 2
    outside = [(-6.0, h), (0.0, h)] + [(depth * math.cos(t), centre + semi * math.sin(t)) for t in a] + [(0.0, y_back), (-6.0, y_back)]
    wall = [(-6.0, -h), (0.0, -h), (0.0, y_in), (-6.0, y_in)]
    flip = lambda line: [(x, -side * -y) for x, y in line]  # noqa: E731  (side -1: as built; +1: mirrored)
    return walls(flip(outside), flip(wall)), (y_in, y_back)


@pytest.fixture(scope='module')
def ros():
    rclpy.init(domain_id=DOMAIN)
    node = rclpy.create_node('wall_follow_test')
    yield node
    node.destroy_node()
    rclpy.shutdown()


def launch(*args):
    return subprocess.Popen(
        ['ros2', 'launch', 'control', 'wall_follow.launch.py', 'use_sim_time:=false', f'speed:={SPEED}', *args],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True,
        env=dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN)))


def stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
    return proc.stdout.read()


class Driver:
    def __init__(self, node):
        self.node = node
        self.scan_pub = node.create_publisher(LaserScan, '/scan', 10)
        self.pose_pub = node.create_publisher(PoseStamped, '/tracked_pose', 10)
        self.drives = []
        self.sub = node.create_subscription(AckermannDriveStamped, '/drive', self.drives.append, 10)

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_for_node(self, proc, timeout=20.0):
        end = time.time() + timeout
        while self.scan_pub.get_subscription_count() == 0:
            assert proc.poll() is None, 'node exited:\n' + proc.stdout.read()
            assert time.time() < end, 'wall_follow_node did not subscribe to /scan'
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def drive(self, left, right, n=10):
        """Publish n corridor scans and return the last /drive."""
        return self.drive_scan(lambda: corridor_scan(left, right), n, f'corridor ({left}, {right})')

    def drive_scan(self, make_scan, n=10, what='scan'):
        """Publish n scans from make_scan() and return the last /drive."""
        self.drives.clear()
        for _ in range(n):
            msg = make_scan()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.scan_pub.publish(msg)
            self.spin(0.05)
        self.spin(0.5)
        assert self.drives, f'no /drive for {what}'
        d = self.drives[-1].drive
        assert math.isfinite(d.steering_angle) and abs(d.steering_angle) <= MAX_STEER + 1e-6
        return d

    def pose(self, x, y):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)
        self.spin(0.01)

    def close(self):
        for e in (self.scan_pub, self.pose_pub):
            self.node.destroy_publisher(e)
        self.node.destroy_subscription(self.sub)


@pytest.fixture
def run(ros):
    procs, drivers = [], []

    def start(*args):
        proc = launch(*args)
        driver = Driver(ros)
        procs.append(proc)
        drivers.append(driver)
        driver.wait_for_node(proc)
        return driver, proc

    yield start
    for d in drivers:
        d.close()
    for p in procs:
        out = stop(p)
        assert 'Traceback' not in out, out


def test_centring(run):
    driver, _ = run()
    centred = driver.drive(0.6, 0.6)
    assert centred.steering_angle == pytest.approx(0.0, abs=1e-3)
    assert centred.speed == pytest.approx(SPEED, rel=1e-3)   # float32 ranges leave a ~1e-6 rad residual
    assert driver.drive(0.9, 0.3).steering_angle > 0.0    # right of centre: steer left
    assert driver.drive(0.3, 0.9).steering_angle < 0.0    # left of centre: steer right
    assert driver.drive(0.3, 0.9).speed < SPEED           # slows while steering
    assert driver.drive(0.6, 20.0).steering_angle == pytest.approx(0.0, abs=1e-3)  # one wall at track_width / 2


def test_turns_into_a_hairpin_at_full_lock_before_the_wall(run):
    # Regression (SRL 2025 ICRA): the side beams hit the outside wall ahead, the measured width collapsed and the
    # corner turn faded out; the car drove straight into the wall. The width must hold and the turn reach full
    # lock while the wall is still about half a track width away.
    driver, _ = run()
    driver.drive(1.0, 1.0, n=30)                              # learn the 2 m width on the straight
    steering = [driver.drive_scan(lambda d=d: left_hairpin(d), n=6, what=f'hairpin {d} m').steering_angle
                for d in (2.4, 2.0, 1.6, 1.3, 1.0)]
    assert steering[0] < 0.3 * MAX_STEER, steering           # wall still far: only centring (the inside opening)
    assert all(b >= a - 0.02 for a, b in zip(steering, steering[1:])), steering   # never unwinds on the way in
    assert steering[-1] == pytest.approx(MAX_STEER, abs=1e-3), steering           # full left lock at 1.0 m


@pytest.mark.parametrize('speed', [0.8, 1.0, 1.2])
def test_drives_through_the_recorded_u_turn_hairpin(ros, speed):
    # Closed-loop regression on the real geometry (SRL 2025 ICRA, top-left hairpin): the previous follower drove
    # into the outside wall at every one of these speeds; it must come round into the return lane, clear of walls.
    node = WallFollower()
    node.speed = speed
    try:
        back_in_return_lane = lambda x, y, yaw: x < -1.0 and y > 1.2 and math.cos(yaw) < -0.8  # noqa: E731
        outcome, seconds, min_clear = drive_closed_loop(node, walls(HAIRPIN_OUTSIDE, HAIRPIN_DIVIDER),
                                                        back_in_return_lane, x=-4.5, y=0.0, yaw=0.0)
    finally:
        node.destroy_node()
    assert outcome == 'through', (outcome, seconds, min_clear)
    assert min_clear > 0.1, min_clear


@pytest.mark.parametrize('lane, thick, depth, side, speed', [
    (2.0, 0.3, 3.0, -1, 1.2),   # wide lane, thick wall: full lock at the scanner clipped the wall's far corner
    (2.0, 0.3, 3.0, 1, 1.2),
    (1.6, 0.3, 3.0, -1, 0.8),   # SRL 2024 CDC-like
    (1.6, 0.05, 4.0, -1, 1.2),  # outside wall 4 m beyond: the hairpin never started, the car drove into it
    (1.2, 0.3, 4.0, -1, 0.8),
])
def test_u_turns_round_a_wall_end(ros, lane, thick, depth, side, speed):
    # Closed-loop regression: the previous hairpin turn (full lock once the inside wall ended at the scanner, the
    # wall ahead within 3.5 m) crashed in each of these.
    segments, (y_in, y_back) = wall_end_u_turn(lane, thick, depth, side)
    node = WallFollower()
    node.speed = speed
    try:
        in_return_lane = lambda x, y, yaw: x < -2.0 and -side * y > -y_in and math.cos(yaw) < -0.8  # noqa: E731
        outcome, seconds, min_clear = drive_closed_loop(node, segments, in_return_lane, x=-4.5, y=0.0, yaw=0.0,
                                                        t_max=20.0)
    finally:
        node.destroy_node()
    assert outcome == 'through', (outcome, seconds, min_clear)
    assert min_clear > 0.1, min_clear


@pytest.mark.parametrize('speed', [0.8, 1.0, 1.2])
def test_takes_an_s_bend_round_a_wall_end_not_the_dead_end_behind_it(ros, speed):
    # Closed-loop regression (SRL 2024 CDC): the turn round the peninsula's end starts like a hairpin, but the track
    # then goes left. Held to the hairpin's side, the car turned on into the dead-end wedge behind the peninsula
    # and hit the outside wall on every lap; the followers before this crash here at each of these speeds.
    node = WallFollower()
    node.speed = speed
    try:
        in_top_lane = lambda x, y, yaw: x < -1.0 and y > 3.9  # noqa: E731
        outcome, seconds, min_clear = drive_closed_loop(
            node, walls(S_BEND_LOWER, S_BEND_PENINSULA, S_BEND_OUTSIDE), in_top_lane, x=5.0, y=2.0, yaw=math.pi,
            t_max=25.0)
    finally:
        node.destroy_node()
    assert outcome == 'through', (outcome, seconds, min_clear)
    assert min_clear > 0.05, min_clear


def test_stops_after_laps(run):
    driver, proc = run('laps:=1')
    driver.drive(0.6, 0.6, n=3)
    for x, y in circle_lap() + [(4.0, 0.0)]:
        driver.pose(x, y)
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        pytest.fail('wall_follow_node did not exit after its lap')
    driver.spin(0.5)
    assert driver.drives[-1].drive.speed == 0.0
    out = proc.stdout.read()
    assert 'Lap 1/1 complete' in out, out
