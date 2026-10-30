#!/usr/bin/env python3
"""AutoDRIVE <-> stack adapter.

Republishes simulator sensor topics under the official f1tenth_system names (/scan,
/sensors/imu/raw), publishes static sensor transforms, exposes simulator ground truth
separately for debugging, and converts /drive commands into AutoDRIVE normalized
throttle/steering commands.

The throttle loop stands in for the VESC's speed control and for f1tenth_system's acceleration
limit on the /drive speed. Its measured speed comes from /odom (wheel_odom in sim, vesc_to_odom
on the car) rather than AutoDRIVE ground truth. The sim throttle is motor torque
(VehicleController); ~25 m/s per unit throttle is its steady state.
"""
import math
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.signals import SignalHandlerOptions
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from autodrive_bridge.single_instance import DuplicateGuard, exit_with_parent, topic_publishers


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def move_towards(current, target, max_change):
    """current moved toward target by at most max_change (ActuationLimiter in ForzaETH's f1tenth_system)."""
    return current + max(-max_change, min(max_change, target - current))


class Adapter(Node):
    def __init__(self):
        super().__init__('adapter')
        p = self.declare_parameter
        self.ns = p('vehicle_ns', '/autodrive/roboracer_1').value
        self.odom_topic = p('odom_topic', '/odom').value
        self.max_steer = p('max_steering_angle', Parameter.Type.DOUBLE).value  # rad at steering_command = 1 (vehicle.yaml)
        if self.max_steer is None:
            raise RuntimeError('max_steering_angle unset: load control/config/vehicle.yaml')
        self.laser_x = p('laser_x', 0.2733).value
        self.laser_z = p('laser_z', 0.096).value
        self.camera_x = p('camera_x', -0.015).value
        self.camera_z = p('camera_z', 0.15).value
        self.kff = p('speed_kff', 0.04).value                     # throttle per m/s (1 / 25)
        self.kp = p('speed_kp', 0.006).value
        self.kp_brake = p('speed_kp_brake', 0.1).value            # kp when too fast: negative throttle brakes
        self.ki = p('speed_ki', 0.006).value
        self.i_max = p('speed_i_max', 1.0).value
        self.max_throttle = p('max_throttle', 1.0).value
        self.max_accel = p('max_acceleration', 2.5).value         # m/s^2 ramp on the /drive speed
        self.max_decel = p('max_deceleration', 6.9).value         # m/s^2 (names as ForzaETH's ActuationLimiter)
        self.cmd_timeout = p('cmd_timeout', 0.5).value            # s; a longer /drive gap restarts the integral and
                                                                  # the speed ramp from the measured speed
                                                                  # (fast_bridge's own cmd_timeout stops the car)
        self.unstick = p('auto_unstick', False).value
        self.unstick_cmd = p('unstick_cmd', 0.5).value
        self.unstick_speed = p('unstick_speed', 0.1).value
        self.unstick_time = p('unstick_time', 1.5).value
        self.unstick_duration = p('unstick_duration', 1.0).value
        self.unstick_reverse_speed = p('unstick_reverse_speed', 0.8).value

        # AutoDRIVE's bridge publishes RELIABLE / depth 1; match it.
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(Odometry, self.ns + '/odom', self.sim_odom_cb, qos)   # restricted topic
        self.create_subscription(Imu, self.ns + '/imu', self.imu_cb, qos)
        self.create_subscription(LaserScan, self.ns + '/lidar', self.lidar_cb, qos)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(AckermannDriveStamped, '/drive', self.drive_cb, 10)

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.imu_pub = self.create_publisher(Imu, '/sensors/imu/raw', 10)
        self.gt_odom_pub = self.create_publisher(Odometry, '/ground_truth/odom', 10)
        self.gt_pose_pub = self.create_publisher(PoseStamped, '/ground_truth/pose', 10)
        self.throttle_pub = self.create_publisher(Float32, self.ns + '/throttle_command', qos)
        self.steer_pub = self.create_publisher(Float32, self.ns + '/steering_command', qos)

        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.publish_static_transforms()

        self.vx = 0.0                 # measured speed from /odom, for the throttle loop
        self.v_cmd = 0.0              # /drive speed after the acceleration limit
        self.integral = 0.0
        self.last_ctrl_t = None
        self.stuck_since = None       # auto_unstick state
        self.reverse_until = None
        # A second adapter (an overlapping bridge restart) holds its output while this one runs: its
        # /scan and /sensors/imu/raw would repeat this one's stamps (Cartographer aborts on duplicate IMU
        # stamps), its commands would fight this one's.
        self.guard = DuplicateGuard(self, '/scan, /sensors/imu/raw, ground truth and commands',
                                    lambda: topic_publishers(self, self.ns + '/throttle_command'))

        # Commands go out as each /drive arrives (drive_cb), so the bridge's next reply to the sim
        # carries it. A 50 Hz command timer used to add ~10 ms and drop 2/3 of pure pursuit's
        # commands. When /drive goes quiet, fast_bridge zeroes the commands after its cmd_timeout.
        self.get_logger().info(f'AutoDRIVE adapter up on {self.ns}')

    # ---------------- sensors -> stack ----------------
    def make_static_tf(self, child, position, orientation=(0.0, 0.0, 0.0, 1.0)):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = child
        t, r = tf.transform.translation, tf.transform.rotation
        t.x, t.y, t.z = position
        r.x, r.y, r.z, r.w = orientation
        return tf

    def publish_static_transforms(self):
        transforms = [
            # The sim IMU sits at (0.08, 0, 0.055) m, but Cartographer requires the IMU frame to
            # coincide with its tracking frame, base_link. The gyro reads the same anywhere on the
            # rigid car; the accelerometer lever arm (w^2 * 0.08 m) only nudges its gravity estimate.
            self.make_static_tf('imu', [0.0, 0.0, 0.0]),
            self.make_static_tf('laser', [self.laser_x, 0.0, self.laser_z]),
            self.make_static_tf('front_camera', [self.camera_x, 0.0, self.camera_z],
                                [0.0, 0.0871557, 0.0, 0.9961947]),   # pitched down 10 deg
        ]
        self.static_tf.sendTransform(transforms)

    def imu_cb(self, msg:Imu):
        if not self.guard.ok:   # a second copy's stamps duplicate this one's (Cartographer aborts)
            return
        self.imu_pub.publish(msg)   # keeps fast_bridge's stamp, the sim frame it arrived in

    def odom_cb(self, msg:Odometry):
        vx = msg.twist.twist.linear.x
        if math.isfinite(vx):   # a NaN speed would make the throttle NaN, clamped to full throttle
            self.vx = vx

    def sim_odom_cb(self, msg:Odometry):
        """Sim ground truth, projected to 2D, in the sim world frame ('world', not Cartographer's
        'map'): /ground_truth/odom, /ground_truth/pose, TF world -> ground_truth. Debugging only."""
        if not self.guard.ok:
            return
        src = msg.pose.pose
        qx, qy, qz, qw = quat_from_yaw(yaw_from_quat(src.orientation))

        odom = Odometry()
        odom.header.stamp = msg.header.stamp   # fast_bridge's stamp, the sim frame
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'ground_truth'
        odom.pose.pose.position.x = src.position.x
        odom.pose.pose.position.y = src.position.y
        o = odom.pose.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        odom.twist.twist.linear.x = msg.twist.twist.linear.x
        odom.twist.twist.linear.y = msg.twist.twist.linear.y
        odom.twist.twist.angular.z = msg.twist.twist.angular.z
        self.gt_odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.gt_pose_pub.publish(pose)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = 'ground_truth'
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(tf)

    def lidar_cb(self, msg:LaserScan):
        if not self.guard.ok:
            return
        msg.header.frame_id = 'laser'
        self.scan_pub.publish(msg)  # keeps fast_bridge's stamp, the sim frame the scan arrived in

    # ---------------- stack -> AutoDRIVE ----------------
    def drive_cb(self, msg):
        """Throttle (speed PI + feedforward) and normalized steering for each /drive."""
        if not self.guard.ok:
            return
        v_ref = float(msg.drive.speed)
        steer = float(msg.drive.steering_angle)
        if not (math.isfinite(v_ref) and math.isfinite(steer)):
            # min/max clamps pass NaN through as the bound (full steering lock, a speed ramp that only
            # climbs), so stop instead; the next finite /drive ramps from the measured speed
            self.get_logger().error(f'/drive not finite (speed {v_ref}, steering {steer}): stopping',
                                    throttle_duration_sec=1.0)
            self.integral = 0.0
            self.v_cmd = 0.0
            self.last_ctrl_t = None
            self.throttle_pub.publish(Float32(data=0.0))
            self.steer_pub.publish(Float32(data=0.0))
            return
        now = self.get_clock().now()
        gap = None if self.last_ctrl_t is None else (now - self.last_ctrl_t).nanoseconds * 1e-9
        if gap is None or gap > self.cmd_timeout:
            # /drive restarted after fast_bridge stopped the car: ramp from the measured speed, nominal first
            # step. With zero throttle for cmd_timeout the wheels are not driven, so vx is not above the true
            # speed. Clamped at 0: /odom can read a large negative glitch after a respawn (-10.55 m/s seen),
            # which would otherwise mean full reverse torque. After a sim reset this relies on /drive staying
            # quiet longer than cmd_timeout (the localization gate withholds poses); otherwise the ramp
            # continues from before the reset.
            self.integral = 0.0
            self.v_cmd = max(0.0, self.vx)
            dt = 0.02
        else:
            dt = min(0.1, gap)
        self.last_ctrl_t = now

        t = now.nanoseconds * 1e-9
        if self.unstick:
            if self.reverse_until is not None:
                if t < self.reverse_until:
                    self.integral = 0.0
                    self.v_cmd = 0.0   # forward again ramps from 0
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

        # The sim throttle is motor torque, so kff * v_ref from a standstill is a torque step that spins
        # the wheels, and /odom counts the spin as travel. Ramp the speed as f1tenth_system does on the car
        # (stock throttle_interpolator, symmetric; ForzaETH's ActuationLimiter, accel / decel separate, copied
        # here). max_deceleration (mue * g) does not limit the planned braking at speed_scale <= 1; it keeps
        # a one-frame stop from restarting the ramp at 0 while the car is fast.
        rate = self.max_accel if v_ref > self.v_cmd else self.max_decel
        self.v_cmd = move_towards(self.v_cmd, v_ref, rate * dt)

        err = self.v_cmd - self.vx
        if abs(v_ref) < 1e-3 and abs(self.vx) < 0.05:
            self.integral = 0.0
            self.v_cmd = 0.0
            throttle = 0.0
        else:
            self.integral = max(-self.i_max, min(self.i_max, self.integral + err * dt))
            # The feedforward alone keeps the throttle positive while slowing down (8 -> 3 m/s with
            # kp 0.006: +0.09), leaving braking to drag; a larger gain on overspeed brakes.
            kp = self.kp if err >= 0 else self.kp_brake
            throttle = self.kff * self.v_cmd + kp * err + self.ki * self.integral
        throttle = max(-self.max_throttle, min(self.max_throttle, throttle))

        self.throttle_pub.publish(Float32(data=float(throttle)))
        self.steer_pub.publish(Float32(data=float(max(-1.0, min(1.0, steer / self.max_steer)))))


def main():
    # No rclpy signal handler: Ctrl+C / SIGTERM end the loop with the context still valid, so the
    # zero command below can be published (rclpy's handler shut the context down first and it
    # raised). Under ros2 launch, fast_bridge is stopping too and may not forward it; it sends the
    # sim a zero command itself when it stops. Python handles signals only between waits: wake every 0.1 s.
    exit_with_parent()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    node = Adapter()
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node.guard.ok:   # a held copy leaves the running one's commands alone
            node.throttle_pub.publish(Float32(data=0.0))
            node.steer_pub.publish(Float32(data=0.0))
        node.destroy_node()
        rclpy.try_shutdown()
