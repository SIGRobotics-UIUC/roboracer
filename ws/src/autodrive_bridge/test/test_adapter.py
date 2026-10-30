import math

import pytest
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from rclpy.time import Time

from autodrive_bridge.adapter import Adapter, move_towards

DT = 0.01   # pure pursuit sends /drive at ~100 Hz


class FakeClock:
    def __init__(self):
        self.ns = 1_000_000_000

    def now(self):
        return Time(nanoseconds=self.ns)


class Count:
    def __init__(self):
        self.n = 0

    def publish(self, msg):
        self.n += 1

    sendTransform = publish


class Recorder:
    def __init__(self):
        self.data = []

    def publish(self, msg):
        self.data.append(msg.data)


@pytest.fixture
def node():
    n = Adapter()
    n.fake_clock = FakeClock()
    n.get_clock = lambda: n.fake_clock   # deterministic dt
    n.throttle_pub, n.steer_pub = Recorder(), Recorder()
    n.guard.ok = True   # alone (the guard's own tests: test_single_instance.py)
    yield n
    n.destroy_node()


def drive(node, speed, dt=DT, steer=0.0):
    """One /drive dt seconds after the last; returns the throttle sent."""
    node.fake_clock.ns += int(round(dt * 1e9))
    msg = AckermannDriveStamped()
    msg.drive.speed = float(speed)
    msg.drive.steering_angle = float(steer)
    node.drive_cb(msg)
    return node.throttle_pub.data[-1]


def test_move_towards():
    assert move_towards(0.0, 4.0, 0.5) == pytest.approx(0.5)
    assert move_towards(4.0, 0.0, 0.5) == pytest.approx(3.5)
    assert move_towards(1.0, 1.2, 0.5) == pytest.approx(1.2)


def test_launch_ramps_the_speed(node):
    # from rest: the old loop sent kff * 4.45 = 0.18 at once, which spins the wheels
    throttle = drive(node, 4.45)
    assert node.v_cmd == pytest.approx(node.max_accel * 0.02)   # first /drive: nominal 0.02 s step
    assert throttle < 0.01
    for _ in range(100):
        drive(node, 4.45)
    assert node.v_cmd == pytest.approx(node.max_accel * (0.02 + 100 * DT))
    for _ in range(100):
        drive(node, 4.45)
    assert node.v_cmd == pytest.approx(4.45)


def test_steady_state_throttle_unchanged(node):
    # once the ramp has reached the request, the throttle is the old formula on v_ref
    node.vx = 4.0
    for _ in range(50):
        drive(node, 4.45)
    assert node.v_cmd == pytest.approx(4.45)
    i = min(node.i_max, node.integral + 0.45 * DT)
    expected = node.kff * 4.45 + node.kp * 0.45 + node.ki * i
    assert drive(node, 4.45) == pytest.approx(expected)


def test_one_frame_stop_does_not_restart_the_ramp(node):
    node.vx = 4.45
    drive(node, 4.45)                    # restart from the measured speed
    assert node.v_cmd == pytest.approx(4.45)
    throttles = [drive(node, 0.0)]
    assert node.v_cmd == pytest.approx(4.45 - node.max_decel * DT)   # dips 0.07 m/s, not to 0
    throttles += [drive(node, 4.45) for _ in range(10)]
    assert node.v_cmd == pytest.approx(4.45)
    # the old loop sent kp_brake * -4.45 = -0.45 for the zero frame; no frame brakes now
    assert min(throttles) > 0.1


def test_restart_after_gap_from_measured_speed(node):
    node.vx = 3.0
    throttle = drive(node, 4.45, dt=1.0)   # longer than cmd_timeout
    assert node.v_cmd == pytest.approx(3.0 + node.max_accel * 0.02)
    assert throttle > 0.0                  # no braking toward a ramp restarted at 0
    node.vx = 0.0                          # e.g. after a respawn
    assert drive(node, 4.45, dt=1.0) < 0.01
    assert node.v_cmd == pytest.approx(node.max_accel * 0.02)


def test_restart_ignores_negative_odom_glitch(node):
    # /odom read -10.55 m/s within 1 s after a respawn (bag_why1); a forward launch must not reverse
    node.vx = -1.0
    assert drive(node, 4.45, dt=1.0) >= 0.0
    assert node.v_cmd == pytest.approx(node.max_accel * 0.02)
    node.vx = 0.0
    assert min(drive(node, 4.45) for _ in range(20)) >= 0.0


def test_reverse_limited_like_actuation_limiter(node):
    drive(node, -1.0)                      # from rest toward -1 at max_deceleration (first step 0.02 s)
    assert node.v_cmd == pytest.approx(-node.max_decel * 0.02)
    for _ in range(20):
        drive(node, -1.0)
    assert node.v_cmd == pytest.approx(-1.0)
    drive(node, -0.5)                      # back up toward 0 at max_acceleration
    assert node.v_cmd == pytest.approx(-1.0 + node.max_accel * DT)


def test_unstick_reverse_resets_the_ramp(node):
    node.unstick = True
    node.vx = 0.0
    while node.reverse_until is None:      # stuck: the ramp keeps rising
        drive(node, 4.45)
    assert node.v_cmd > 3.0
    drive(node, 4.45)
    assert node.v_cmd == 0.0
    assert node.throttle_pub.data[-1] == pytest.approx(-node.kff * node.unstick_reverse_speed)
    while node.reverse_until is not None:
        throttle = drive(node, 4.45)
    assert node.v_cmd == pytest.approx(node.max_accel * DT)   # forward again from 0
    assert throttle < 0.01


def test_zero_at_rest(node):
    assert drive(node, 0.0) == 0.0
    assert node.v_cmd == 0.0


def test_zero_request_at_rest_clears_the_ramp(node):
    # e.g. stopped by a wall while the ramp was still coming down
    drive(node, 3.0)
    node.v_cmd = 3.0
    node.vx = 0.0
    assert drive(node, 0.0) == 0.0
    assert node.v_cmd == 0.0


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_non_finite_speed_stops(node, bad):
    node.vx = 3.0
    for _ in range(20):
        drive(node, 4.45, steer=0.2)
    assert drive(node, bad, steer=0.2) == 0.0
    assert node.steer_pub.data[-1] == 0.0
    assert node.v_cmd == 0.0 and node.integral == 0.0
    # the next finite /drive ramps from the measured speed again, as after a /drive gap
    assert drive(node, 4.45) > 0.0
    assert node.v_cmd == pytest.approx(3.0 + node.max_accel * 0.02)


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_non_finite_steering_stops(node, bad):
    # max(-1, min(1, nan)) is 1.0: full left lock
    node.vx = 3.0
    assert drive(node, 4.45, steer=bad) == 0.0
    assert node.steer_pub.data[-1] == 0.0


def test_repeated_nan_speed_does_not_ramp(node):
    # move_towards(v, nan, m) added +m every call
    for _ in range(100):
        drive(node, math.nan)
    assert node.v_cmd == 0.0
    assert set(node.throttle_pub.data) == {0.0}


@pytest.mark.parametrize('bad', [math.nan, math.inf])
def test_non_finite_odom_ignored(node, bad):
    # a NaN vx made the throttle NaN, which the clamp turned into +1.0
    msg = Odometry()
    msg.twist.twist.linear.x = 2.0
    node.odom_cb(msg)
    msg.twist.twist.linear.x = bad
    node.odom_cb(msg)
    assert node.vx == 2.0
    throttle = drive(node, 2.0)
    assert math.isfinite(throttle) and throttle < 0.2


def test_held_while_another_adapter_runs(node):
    # a second adapter republishing the old bridge's IMU and scans repeats their stamps: Cartographer aborts
    node.guard.ok = False
    pubs = {name: Count() for name in ('scan_pub', 'imu_pub', 'gt_odom_pub', 'gt_pose_pub', 'tf')}
    for name, pub in pubs.items():
        setattr(node, name, pub)
    msg = AckermannDriveStamped()
    msg.drive.speed = 4.0
    node.drive_cb(msg)
    node.imu_cb(Imu())
    node.lidar_cb(LaserScan())
    node.sim_odom_cb(Odometry())
    assert node.throttle_pub.data == [] and node.steer_pub.data == []
    assert all(p.n == 0 for p in pubs.values())
    node.guard.ok = True
    node.imu_cb(Imu())
    node.lidar_cb(LaserScan())
    node.sim_odom_cb(Odometry())
    assert all(p.n == 1 for p in pubs.values())
