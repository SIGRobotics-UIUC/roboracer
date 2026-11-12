#!/usr/bin/env python3
"""Online command-versus-localization slip and friction monitor.

Inputs are /drive, a map-frame PoseStamped, and the planar IMU acceleration.
The node is diagnostic only: it never publishes a drive command.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, String

from control.slip_detection import (
    CommandMotion, FrictionEstimator, PoseMotionEstimator, SlipDetector)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class SlipMonitor(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__(
            'slip_monitor', parameter_overrides=parameter_overrides)
        p = self.declare_parameter
        self.pose_topic = p('pose_topic', '/localized_pose').value
        self.command_topic = p('command_topic', '/drive').value
        self.imu_topic = p('imu_topic', '/sensors/imu/raw').value
        wheelbase = p('wheelbase', Parameter.Type.DOUBLE).value
        if wheelbase is None:
            raise RuntimeError(
                'wheelbase unset: load control/config/vehicle.yaml')
        linear_threshold = p('linear_threshold', 0.75).value
        angular_threshold = p('angular_threshold', 0.75).value
        pose_smoothing = p('pose_smoothing', 0.35).value
        residual_smoothing = p('residual_smoothing', 0.35).value
        trigger_samples = p('trigger_samples', 2).value
        clear_samples = p('clear_samples', 3).value
        max_acceleration = p('max_acceleration', 2.5).value
        max_deceleration = p('max_deceleration', 6.9).value
        self.min_motion_speed = p('min_motion_speed', 0.2).value
        self.max_command_age = p('max_command_age', 0.5).value
        self.max_imu_age = p('max_imu_age', 0.2).value
        self.max_pose_age = p('max_pose_age', 0.5).value
        self.reset_mu_on_sim_reset = p('reset_mu_on_sim_reset', True).value

        positive = (
            self.min_motion_speed, self.max_command_age,
            self.max_imu_age, self.max_pose_age)
        if not all(math.isfinite(v) and v > 0 for v in positive):
            raise ValueError(
                'motion/freshness parameters must be finite and positive')

        self.motion = PoseMotionEstimator(smoothing=pose_smoothing)
        self.command_motion = CommandMotion(
            max_acceleration, max_deceleration)
        self.detector = SlipDetector(
            wheelbase, linear_threshold, angular_threshold,
            residual_smoothing, trigger_samples, clear_samples)
        self.friction = FrictionEstimator()
        self.command = None       # speed, steering, arrival time
        self.imu = None           # ax, ay, arrival time
        self.last_pose_arrival = None
        self.is_valid = False

        self.detected_pub = self.create_publisher(
            Bool, '/slip/detected', 10)
        self.valid_pub = self.create_publisher(Bool, '/slip/valid', 10)
        self.linear_pub = self.create_publisher(Bool, '/slip/linear', 10)
        self.angular_pub = self.create_publisher(
            Bool, '/slip/angular', 10)
        self.linear_error_pub = self.create_publisher(
            Float32, '/slip/linear_error', 10)
        self.angular_error_pub = self.create_publisher(
            Float32, '/slip/angular_error', 10)
        self.expected_speed_pub = self.create_publisher(
            Float32, '/slip/expected_speed', 10)
        self.measured_speed_pub = self.create_publisher(
            Float32, '/slip/measured_speed', 10)
        self.friction_pub = self.create_publisher(
            Float32, '/friction/estimate', 10)
        self.friction_valid_pub = self.create_publisher(
            Bool, '/friction/valid', 10)

        self.create_subscription(
            AckermannDriveStamped, self.command_topic,
            self.command_callback, 10)
        self.create_subscription(
            PoseStamped, self.pose_topic, self.pose_callback, 10)
        self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, 10)
        self.create_subscription(
            String, '/autodrive/roboracer_1/sim_reset',
            self.reset_callback, 10)
        self.create_timer(0.05, self.watchdog)
        self.get_logger().info(
            f'slip monitor: {self.command_topic} versus {self.pose_topic}; '
            f'thresholds {linear_threshold:.3f} m/s, '
            f'{angular_threshold:.3f} rad/s (diagnostic only)')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def command_callback(self, msg):
        speed = float(msg.drive.speed)
        steering = float(msg.drive.steering_angle)
        values = (speed, steering)
        self.command = ((speed, steering, self.now())
                        if all(math.isfinite(v) for v in values) else None)

    def imu_callback(self, msg):
        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        self.imu = ((ax, ay, self.now())
                    if all(math.isfinite(v) for v in (ax, ay)) else None)

    def reset_state(self, reset_friction=False):
        self.motion.reset()
        self.command_motion.reset()
        self.detector.reset()
        self.command = None
        self.imu = None
        self.last_pose_arrival = None
        if reset_friction:
            self.friction.reset()
        self.publish_invalid()

    def reset_callback(self, _msg):
        self.reset_state(self.reset_mu_on_sim_reset)

    def publish_invalid(self):
        if self.is_valid:
            self.detected_pub.publish(Bool(data=False))
            self.linear_pub.publish(Bool(data=False))
            self.angular_pub.publish(Bool(data=False))
        self.valid_pub.publish(Bool(data=False))
        self.friction_valid_pub.publish(
            Bool(data=self.friction.estimate is not None))
        self.is_valid = False

    def watchdog(self):
        if (self.last_pose_arrival is not None and self.is_valid and
                self.now() - self.last_pose_arrival > self.max_pose_age):
            self.motion.reset()
            self.command_motion.reset()
            self.detector.reset()
            self.publish_invalid()

    def pose_callback(self, msg):
        now = self.now()
        self.last_pose_arrival = now
        q = msg.pose.orientation
        q_values = (q.x, q.y, q.z, q.w)
        norm = math.sqrt(sum(v * v for v in q_values))
        x, y = float(msg.pose.position.x), float(msg.pose.position.y)
        stamp = stamp_seconds(msg.header.stamp)
        pose_age = now - stamp
        values = (x, y, stamp, *q_values)
        if (msg.header.frame_id != 'map' or
                not all(math.isfinite(v) for v in values) or
                stamp <= 0 or abs(pose_age) > self.max_pose_age or
                abs(norm - 1.0) > 0.1):
            self.motion.reset()
            self.command_motion.reset()
            self.detector.reset()
            return self.publish_invalid()

        yaw = yaw_from_quaternion(q)
        motion = self.motion.update(stamp, x, y, yaw)
        if motion is None:
            self.detector.reset()
            return self.publish_invalid()
        if (self.command is None or
                now - self.command[2] > self.max_command_age):
            self.command_motion.reset()
            self.detector.reset()
            return self.publish_invalid()

        speed_command, steering, _ = self.command
        expected_speed = self.command_motion.update(
            speed_command, motion.speed, motion.dt)
        if expected_speed is None:
            self.detector.reset()
            return self.publish_invalid()
        if max(abs(expected_speed), abs(motion.speed)) < self.min_motion_speed:
            self.detector.reset()
            return self.publish_invalid()

        result = self.detector.update(
            expected_speed, steering, motion.speed, motion.yaw_rate)
        if result is None:
            return self.publish_invalid()

        self.valid_pub.publish(Bool(data=True))
        self.detected_pub.publish(Bool(data=result.slipping))
        self.linear_pub.publish(Bool(data=result.linear_slip))
        self.angular_pub.publish(Bool(data=result.angular_slip))
        self.linear_error_pub.publish(Float32(data=result.linear_error))
        self.angular_error_pub.publish(Float32(data=result.angular_error))
        self.expected_speed_pub.publish(Float32(data=result.expected_speed))
        self.measured_speed_pub.publish(Float32(data=motion.speed))
        self.is_valid = True

        imu_fresh = (
            self.imu is not None and now - self.imu[2] <= self.max_imu_age)
        if imu_fresh:
            above_threshold = result.linear_slip or result.angular_slip
            estimate = self.friction.update(
                self.imu[0], self.imu[1], valid=True,
                slipping=result.slipping or above_threshold)
            if estimate is not None:
                self.friction_pub.publish(Float32(data=estimate))
        self.friction_valid_pub.publish(
            Bool(data=self.friction.estimate is not None))


def main():
    rclpy.init()
    node = SlipMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
