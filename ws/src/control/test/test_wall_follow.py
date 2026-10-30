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

from control.wall_follow_node import LapCounter  # noqa: E402

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


@pytest.fixture(scope='module')
def ros():
    rclpy.init(domain_id=DOMAIN)
    node = rclpy.create_node('wall_follow_test')
    yield node
    node.destroy_node()
    rclpy.shutdown()


def launch(*args):
    return subprocess.Popen(
        ['ros2', 'launch', 'control', 'wall_follow.launch.py', f'speed:={SPEED}', *args],
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
        self.drives.clear()
        for _ in range(n):
            msg = corridor_scan(left, right)
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.scan_pub.publish(msg)
            self.spin(0.05)
        self.spin(0.5)
        assert self.drives, f'no /drive for corridor ({left}, {right})'
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
