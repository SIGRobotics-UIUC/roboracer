import math
import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32

from autodrive_bridge.wheel_odom import WheelOdom, reset_jump

R = 0.0585   # wheel_radius default


def stamp(t):
    return Time(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))


def frame(node, t, dist, gyro=None):
    """One sim frame: both encoders at `dist` metres of travel, then (unless None) the IMU."""
    for cb in (node.left_cb, node.right_cb):
        js = JointState()
        js.header.stamp = stamp(t)
        js.position = [dist / R]
        cb(js)
    if gyro is not None:
        imu = Imu()
        imu.header.stamp = stamp(t)
        imu.angular_velocity.z = gyro
        node.imu_cb(imu)


@pytest.fixture
def node():
    n = WheelOdom()
    yield n
    n.destroy_node()


def test_reset_jump():
    assert reset_jump((0.0, 0.0, 0.0), (0.01, -1.0 / R, -1.0 / R), R, 30.0)      # 1 m back in 10 ms
    assert not reset_jump((0.0, 0.0, 0.0), (0.01, 0.1 / R, 0.1 / R), R, 30.0)    # 10 m/s
    # bunched frames: wall-clock stamps 1 ms apart, a normal frame's travel
    assert not reset_jump((0.0, 0.0, 0.0), (0.001, 0.11 / R, 0.11 / R), R, 30.0)


def test_sim_steering_feedback_is_radians(node):
    node.use_imu = False  # exercise the bicycle-model fallback
    node.steering_cb(Float32(data=0.2))
    frame(node, 1.0, 0.0)
    frame(node, 1.1, node.wheelbase)
    assert node.yaw == pytest.approx(math.tan(0.2))


def test_short_drive_reset_not_integrated(node):
    # 1 m in 100 frames, then a reset: no IMU in the reset frames, encoders back to the start
    for i in range(101):
        frame(node, 1.0 + 0.01 * i, 0.01 * i, gyro=0.0)
    x = node.x
    assert x == pytest.approx(1.0)
    frame(node, 2.01, 1.01)
    frame(node, 2.02, 0.0)
    frame(node, 2.03, 0.0)
    frame(node, 2.04, 0.0, gyro=0.0)
    assert node.x == pytest.approx(x)   # pose continues from where it was, no jump back
    frame(node, 2.05, 0.01, gyro=0.0)
    assert node.x == pytest.approx(x + 0.01)


def test_gyro_gap_capped(node):
    frame(node, 1.0, 0.0, gyro=2.0)
    frame(node, 1.5, 0.1, gyro=2.0)     # 0.5 s stall at 2 rad/s
    assert node.yaw == pytest.approx(2.0 * node.max_gyro_dt)


def test_slip_off_by_default(node):
    assert not node.slip.active


def test_slip_parameters_degrade_odometry_and_turn_off_again(node):
    from rclpy.parameter import Parameter as P
    res = node.set_parameters([P('slip_scale_bias', P.Type.DOUBLE, 0.1)])
    assert res[0].successful and node.slip.active
    frame(node, 1.0, 0.0, gyro=0.0)
    frame(node, 1.1, 1.0, gyro=0.0)
    assert node.x == pytest.approx(1.1)         # 1 m driven, 10 % wheelspin read as 1.1 m
    node.set_parameters([P('slip_scale_bias', P.Type.DOUBLE, 0.0)])
    assert not node.slip.active
    frame(node, 1.2, 2.0, gyro=0.0)
    assert node.x == pytest.approx(2.1)         # back to exact


def test_slip_seed_restarts_the_noise(node):
    from rclpy.parameter import Parameter as P
    node.set_parameters([P('slip_scale_std', P.Type.DOUBLE, 0.1), P('slip_seed', P.Type.INTEGER, 4)])
    first = node.slip.e_s
    node.set_parameters([P('slip_seed', P.Type.INTEGER, 4)])
    assert node.slip.e_s == first


def test_slip_on_then_off_in_one_set_keeps_the_node_alive(node):
    # rclpy applies these one at a time: the slip goes on, then off, logging at two severities
    from rclpy.parameter import Parameter as P
    for v in (0.1, 0.0, 0.2, 0.0):
        res = node.set_parameters([P('slip_scale_bias', P.Type.DOUBLE, v), P('slip_lateral', P.Type.DOUBLE, 0.0)])
        assert all(r.successful for r in res)
        assert node.slip.active == (v > 0)


def test_slip_logged_once_after_the_set_settles(node):
    import rclpy
    from rclpy.parameter import Parameter as P
    node.set_parameters([P('slip_scale_bias', P.Type.DOUBLE, 0.1), P('slip_lateral', P.Type.DOUBLE, 0.02)])
    assert node.slip_text == 'simulated wheel slip off'      # not yet: the set may still be going on
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
    assert 'scale_bias=0.1' in node.slip_text and 'lateral=0.02' in node.slip_text
    assert node.slip_log_timer is None
