"""Callback checks for slip_monitor_node (no transport or simulator)."""
import math

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter  # noqa: E402
from ackermann_msgs.msg import AckermannDriveStamped  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402

from control.slip_monitor_node import SlipMonitor  # noqa: E402


class Record:
    def __init__(self):
        self.data = []

    def publish(self, msg):
        self.data.append(msg.data)


def pose(stamp, x, yaw=0.0):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
    msg.pose.position.x = x
    msg.pose.orientation.z = math.sin(yaw / 2)
    msg.pose.orientation.w = math.cos(yaw / 2)
    return msg


def test_callbacks_publish_no_slip_then_linear_and_angular_slip_and_mu():
    rclpy.init()
    node = SlipMonitor(parameter_overrides=[
        Parameter('wheelbase', value=0.33),
        Parameter('linear_threshold', value=0.4),
        Parameter('angular_threshold', value=0.4),
        Parameter('pose_smoothing', value=1.0),
        Parameter('residual_smoothing', value=1.0),
        Parameter('trigger_samples', value=1),
        Parameter('clear_samples', value=1),
    ])
    try:
        outputs = {}
        for name in ('detected_pub', 'valid_pub', 'linear_pub', 'angular_pub',
                     'linear_error_pub', 'angular_error_pub',
                     'expected_speed_pub', 'measured_speed_pub',
                     'friction_pub', 'friction_valid_pub'):
            outputs[name] = Record()
            setattr(node, name, outputs[name])

        drive = AckermannDriveStamped()
        drive.drive.speed = 2.0
        node.command_callback(drive)
        imu = Imu()
        imu.linear_acceleration.x = 3.0
        imu.linear_acceleration.y = 4.0
        node.imu_callback(imu)

        t = node.now()
        node.pose_callback(pose(t, 0.0))
        node.pose_callback(pose(t + 0.1, 0.1))
        assert outputs['valid_pub'].data[-1]
        assert not outputs['detected_pub'].data[-1]
        assert outputs['friction_pub'].data[-1] == pytest.approx(
            5 / 9.80665, rel=1e-6)

        node.pose_callback(pose(t + 0.2, 0.2))
        assert outputs['linear_pub'].data[-1]
        assert outputs['detected_pub'].data[-1]

        drive.drive.speed = 1.0
        drive.drive.steering_angle = 0.2
        node.command_callback(drive)
        node.pose_callback(pose(t + 0.3, 0.3))
        assert outputs['angular_pub'].data[-1]
        assert outputs['detected_pub'].data[-1]
    finally:
        node.destroy_node()
        rclpy.shutdown()
