#!/usr/bin/env python3
"""AutoDRIVE <-> stack adapter (sim only; stands in for the drivers AND localization).

AutoDRIVE gives ground-truth pose, so in sim this node publishes /car_state/odom itself
(map frame, from the sim's IPS) and the localization node isn't started. It also publishes
/scan and TF map -> base_link -> laser, and turns /drive (speed + steering angle) into
AutoDRIVE's normalized throttle/steering commands.
(Originally written for ForzaETH, so it also publishes /car_state/scan, /car_state/pose and
/vesc/sensors/imu/raw, which this stack doesn't use.)

AutoDRIVE RoboRacer (2026-iros) throttle behaves like a speed setpoint: steady-state
speed is ~25 m/s per unit throttle (measured 2026-10-04: 0.05 -> 1.25, 0.2 -> 4.92 m/s), so
the command is feedforward v_ref * speed_kff with a small PI trim on the measured speed.
Steering is linear, +-1 -> +-0.524 rad (sim feedback and measured yaw rate agree).
Speed comes from the sim's own body-frame velocity (odom twist), position from the IPS.

Recovery after a crash is the run logger's job (it resets the car over the API).
auto_unstick (off by default) is the old fallback for sims without a reset: if /drive asks
for > unstick_cmd m/s but the car stays below unstick_speed for unstick_time s, reverse
for unstick_duration s, then hand control back.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Adapter(Node):
    def __init__(self):
        super().__init__('adapter')
        p = self.declare_parameter
        self.ns = p('vehicle_ns', '/autodrive/roboracer_1').value
        self.max_steer = p('max_steering_angle', 0.5236).value   # rad at steering_command = 1
        self.laser_x = p('laser_x', 0.2733).value                 # base_link (rear axle) -> lidar
        self.kff = p('speed_kff', 0.04).value                     # throttle per m/s (1 / 25)
        self.kp = p('speed_kp', 0.006).value
        self.ki = p('speed_ki', 0.006).value
        self.i_max = p('speed_i_max', 1.0).value
        self.max_throttle = p('max_throttle', 1.0).value
        self.cmd_timeout = p('cmd_timeout', 0.5).value            # s without /drive -> stop
        self.unstick = p('auto_unstick', False).value
        self.unstick_cmd = p('unstick_cmd', 0.5).value
        self.unstick_speed = p('unstick_speed', 0.1).value
        self.unstick_time = p('unstick_time', 1.5).value
        self.unstick_duration = p('unstick_duration', 1.0).value
        self.unstick_reverse_speed = p('unstick_reverse_speed', 0.8).value

        # AutoDRIVE's bridge publishes RELIABLE / depth 1; match it.
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(Point, self.ns + '/ips', self.ips_cb, qos)
        self.create_subscription(Odometry, self.ns + '/odom', self.sim_odom_cb, qos)
        self.create_subscription(Imu, self.ns + '/imu', self.imu_cb, qos)
        self.create_subscription(LaserScan, self.ns + '/lidar', self.lidar_cb, qos)
        self.create_subscription(AckermannDriveStamped, '/drive', self.drive_cb, 10)

        self.scan_pub = self.create_publisher(LaserScan, '/car_state/scan', 10)
        # ForzaETH's MAP controller reads longitudinal accel from the VESC IMU, which is
        # mounted rotated 90 deg (it uses -linear_acceleration.y), and /scan for FTG mode.
        self.vesc_imu_pub = self.create_publisher(Imu, '/vesc/sensors/imu/raw', 10)
        self.raw_scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/car_state/odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/car_state/pose', 10)
        self.throttle_pub = self.create_publisher(Float32, self.ns + '/throttle_command', qos)
        self.steer_pub = self.create_publisher(Float32, self.ns + '/steering_command', qos)

        self.tf = TransformBroadcaster(self)
        static = TransformStamped()
        static.header.stamp = self.get_clock().now().to_msg()
        static.header.frame_id = 'base_link'
        static.child_frame_id = 'laser'
        static.transform.translation.x = self.laser_x
        static.transform.rotation.w = 1.0
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(static)

        self.yaw = None
        self.yaw_rate = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.v_ref = 0.0
        self.steer_ref = 0.0
        self.last_cmd_t = None
        self.integral = 0.0
        self.last_ctrl_t = None
        self.stuck_since = None
        self.reverse_until = None

        self.create_timer(0.02, self.control_step)   # 50 Hz command stream
        self.get_logger().info(f'AutoDRIVE adapter up on {self.ns}')

    # ---------------- sensors -> ForzaETH ----------------
    def imu_cb(self, msg):
        self.yaw = yaw_from_quat(msg.orientation)
        self.yaw_rate = msg.angular_velocity.z
        vesc = Imu()
        vesc.header.stamp = self.get_clock().now().to_msg()
        vesc.header.frame_id = 'imu'
        vesc.linear_acceleration.x = msg.linear_acceleration.y
        vesc.linear_acceleration.y = -msg.linear_acceleration.x
        vesc.linear_acceleration.z = msg.linear_acceleration.z
        vesc.angular_velocity = msg.angular_velocity
        self.vesc_imu_pub.publish(vesc)

    def sim_odom_cb(self, msg):
        self.vx = msg.twist.twist.linear.x     # body frame, from the sim
        self.vy = msg.twist.twist.linear.y

    def ips_cb(self, msg):
        if self.yaw is None:
            return
        now = self.get_clock().now()

        stamp = now.to_msg()
        qx, qy, qz, qw = quat_from_yaw(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = msg.x
        odom.pose.pose.position.y = msg.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.yaw_rate
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = msg.x
        tf.transform.translation.y = msg.y
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(tf)

    def lidar_cb(self, msg):
        msg.header.frame_id = 'laser'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.scan_pub.publish(msg)
        self.raw_scan_pub.publish(msg)

    # ---------------- ForzaETH -> AutoDRIVE ----------------
    def drive_cb(self, msg):
        self.v_ref = float(msg.drive.speed)
        self.steer_ref = float(msg.drive.steering_angle)
        self.last_cmd_t = self.get_clock().now()

    def control_step(self):
        now = self.get_clock().now()
        stale = (self.last_cmd_t is None or
                 (now - self.last_cmd_t).nanoseconds * 1e-9 > self.cmd_timeout)
        v_ref = 0.0 if stale else self.v_ref
        steer = 0.0 if stale else self.steer_ref

        dt = 0.02 if self.last_ctrl_t is None else (now - self.last_ctrl_t).nanoseconds * 1e-9
        self.last_ctrl_t = now

        t = now.nanoseconds * 1e-9
        if self.unstick:
            if self.reverse_until is not None:
                if t < self.reverse_until:
                    self.integral = 0.0
                    self.throttle_pub.publish(Float32(data=float(-self.kff * self.unstick_reverse_speed)))
                    self.steer_pub.publish(Float32(data=0.0))
                    return
                self.reverse_until = None
                self.stuck_since = None
            if v_ref > self.unstick_cmd and abs(self.vx) < self.unstick_speed:
                self.stuck_since = self.stuck_since or t
                if t - self.stuck_since > self.unstick_time:
                    self.get_logger().warn('Car stuck, reversing to free it')
                    self.reverse_until = t + self.unstick_duration
            else:
                self.stuck_since = None

        err = v_ref - self.vx
        if abs(v_ref) < 1e-3 and abs(self.vx) < 0.05:
            self.integral = 0.0
            throttle = 0.0
        else:
            self.integral = max(-self.i_max, min(self.i_max, self.integral + err * dt))
            throttle = self.kff * v_ref + self.kp * err + self.ki * self.integral
        throttle = max(-self.max_throttle, min(self.max_throttle, throttle))

        self.throttle_pub.publish(Float32(data=float(throttle)))
        self.steer_pub.publish(Float32(data=float(max(-1.0, min(1.0, steer / self.max_steer)))))


def main():
    rclpy.init()
    node = Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.throttle_pub.publish(Float32(data=0.0))
        node.destroy_node()
        rclpy.try_shutdown()
