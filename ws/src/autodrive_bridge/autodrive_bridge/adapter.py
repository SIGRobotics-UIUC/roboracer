#!/usr/bin/env python3
"""AutoDRIVE <-> stack adapter.

Republishes simulator sensor topics into the names expected by the stack, publishes
static sensor transforms, exposes simulator ground truth separately for debugging,
and converts /drive commands into AutoDRIVE normalized throttle/steering commands.

Vehicle speed for the throttle controller comes from the estimated odometry rather
than AutoDRIVE ground-truth odometry.
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
        self.odom_topic = p('odom_topic', '/odometry/filtered').value
        self.max_steer = p('max_steering_angle', 0.5236).value   # rad at steering_command = 1
        self.imu_x = p('imu_x', 0.08).value
        self.imu_z = p('imu_z', 0.055).value
        self.laser_x = p('laser_x', 0.2733).value
        self.laser_z = p('laser_z', 0.096).value
        self.camera_x = p('camera_x', -0.015).value
        self.camera_z = p('camera_z', 0.15).value
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
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(AckermannDriveStamped, '/drive', self.drive_cb, 10)

        self.scan_pub = self.create_publisher(LaserScan, '/car_state/scan', 10)
        # ForzaETH's MAP controller reads longitudinal accel from the VESC IMU, which is
        # mounted rotated 90 deg (it uses -linear_acceleration.y), and /scan for FTG mode.
        self.vesc_imu_pub = self.create_publisher(Imu, '/vesc/sensors/imu/raw', 10)
        self.raw_scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.gt_odom_pub = self.create_publisher(Odometry, '/ground_truth/odom', 10)
        self.gt_pose_pub = self.create_publisher(PoseStamped, '/ground_truth/pose', 10)
        self.throttle_pub = self.create_publisher(Float32, self.ns + '/throttle_command', qos)
        self.steer_pub = self.create_publisher(Float32, self.ns + '/steering_command', qos)

        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.publish_static_transforms()

        self.yaw = None
        self.yaw_rate = 0.0
        self.gt_vx = 0.0
        self.gt_vy = 0.0
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
    def make_static_tf(self, child, position, orientation=(0.0, 0.0, 0.0, 1.0)):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = child
        tf.transform.translation.x = position[0]
        tf.transform.translation.y = position[1]
        tf.transform.translation.z = position[2]
        tf.transform.rotation.x = orientation[0]
        tf.transform.rotation.y = orientation[1]
        tf.transform.rotation.z = orientation[2]
        tf.transform.rotation.w = orientation[3]
        return tf

    def publish_static_transforms(self):
        transforms = [
            self.make_static_tf('imu', [self.imu_x, 0.0, self.imu_z]),
            self.make_static_tf('laser', [self.laser_x, 0.0, self.laser_z]),
            self.make_static_tf('front_camera', [self.camera_x, 0.0, self.camera_z],[0.0, 0.0871557, 0.0, 0.9961947]),
        ]
        self.static_tf.sendTransform(transforms)

    def imu_cb(self, msg:Imu):
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
        # self.vesc_imu_pub.publish(msg)

    def sim_odom_cb(self, msg:Odometry):
        self.gt_vx = msg.twist.twist.linear.x
        self.gt_vy = msg.twist.twist.linear.y

    def odom_cb(self, msg:Odometry):
        self.vx = msg.twist.twist.linear.x

    #publishes the ground truth odometry and transform
    def ips_cb(self, msg:Point):
        if self.yaw is None:
            return

        stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = quat_from_yaw(self.yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'ground_truth'
        odom.pose.pose.position.x = msg.x
        odom.pose.pose.position.y = msg.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.gt_vx
        odom.twist.twist.linear.y = self.gt_vy
        odom.twist.twist.angular.z = self.yaw_rate
        self.gt_odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.gt_pose_pub.publish(pose)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = 'ground_truth'
        tf.transform.translation.x = msg.x
        tf.transform.translation.y = msg.y
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(tf)

    def lidar_cb(self, msg:LaserScan):
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
        node.steer_pub.publish(Float32(data=0.0))
        node.destroy_node()
        rclpy.try_shutdown()
