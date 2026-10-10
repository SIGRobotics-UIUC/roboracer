#!/usr/bin/env python3
"""Lean AutoDRIVE <-> ROS 2 bridge for the RoboRacer simulator (2026-iros release).

Publishes all simulator topics, including the front camera (optionally). Topics restricted during
competition can be disabled with the `publish_restricted_topics` parameter.

Published (vehicle_ns = /autodrive/roboracer_1):
  imu, lidar, front_camera, throttle, steering, left_encoder, right_encoder

Additionally, if publish_restricted_topics = True:
  ips, odom, lap_count, collision_count, lap_time, last_lap_time, best_lap_time

Subscribed:
  throttle_command, steering_command, /autodrive/reset_command
"""
import base64
import gzip
import threading
import time
from io import BytesIO

import numpy as np
import rclpy
import socketio
from cv_bridge import CvBridge
from gevent import pywsgi
from geventwebsocket.handler import WebSocketHandler
from PIL import Image as PILImage
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, JointState, LaserScan, Image
from std_msgs.msg import Bool, Float32, Int32


def vec(s):
    return np.fromstring(s, dtype=float, sep=' ')


class FastBridge:
    def __init__(self):
        self.node = rclpy.create_node('autodrive_fast_bridge')
        p = self.node.declare_parameter
        ns = p('vehicle_ns', '/autodrive/roboracer_1').value
        self.port = p('port', 4567).value
        self.reset_frames = p('reset_frames', 5).value
        self.cmd_timeout = p('cmd_timeout', 0.5).value
        self.publish_restricted_topics = p('publish_restricted_topics', True).value
        self.publish_camera = p('publish_camera', False).value
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        n = self.node

        self.pub = {
            'imu': n.create_publisher(Imu, ns + '/imu', qos),
            'lidar': n.create_publisher(LaserScan, ns + '/lidar', qos),
            'throttle': n.create_publisher(Float32, ns + '/throttle', qos),
            'steering': n.create_publisher(Float32, ns + '/steering', qos),
            'left': n.create_publisher(JointState, ns + '/left_encoder', qos),
            'right': n.create_publisher(JointState, ns + '/right_encoder', qos),
        }

        if self.publish_camera:
           self.pub['camera'] = n.create_publisher(Image, ns + '/front_camera', qos)
           self.cv_bridge = CvBridge()

        if self.publish_restricted_topics:
            self.pub.update({
                'ips': n.create_publisher(Point, ns + '/ips', qos),
                'odom': n.create_publisher(Odometry, ns + '/odom', qos),
                'lap_count': n.create_publisher(Int32, ns + '/lap_count', qos),
                'collision_count': n.create_publisher(Int32, ns + '/collision_count', qos),
                'lap_time': n.create_publisher(Float32, ns + '/lap_time', qos),
                'last_lap_time': n.create_publisher(Float32, ns + '/last_lap_time', qos),
                'best_lap_time': n.create_publisher(Float32, ns + '/best_lap_time', qos),
            })

        self.throttle_cmd = 0.0
        self.steering_cmd = 0.0
        self.last_cmd_t = 0.0
        self.reset_left = 0

        n.create_subscription(Float32, ns + '/throttle_command', self._throttle_cb, qos)
        n.create_subscription(Float32, ns + '/steering_command', self._steering_cb, qos)
        n.create_subscription(Bool, '/autodrive/reset_command', self._reset_cb, qos)

        self.frames = 0
        self.window_start = None
        self.window_frames = 0
        self.work_s = 0.0

    def _throttle_cb(self, msg):
        self.throttle_cmd = float(msg.data)
        self.last_cmd_t = time.monotonic()

    def _steering_cb(self, msg):
        self.steering_cmd = float(msg.data)
        self.last_cmd_t = time.monotonic()

    def _reset_cb(self, msg):
        if msg.data:
            self.reset_left = self.reset_frames
            self.throttle_cmd = self.steering_cmd = 0.0
            self.node.get_logger().info('Resetting the car to its start pose')

    def on_frame(self, d):
        t_in = time.perf_counter()
        stamp = self.node.get_clock().now().to_msg()
        P = self.pub

        P['throttle'].publish(Float32(data=float(d['V1 Throttle'])))
        P['steering'].publish(Float32(data=float(d['V1 Steering'])))
        enc = vec(d['V1 Encoder Angles'])
        for key, name, angle in (('left', 'left_encoder', enc[0]), ('right', 'right_encoder', enc[1])):
            js = JointState()
            js.header.stamp = stamp
            js.header.frame_id = name
            js.name = [name]
            js.position = [float(angle)]
            P[key].publish(js)

        q = vec(d['V1 Orientation Quaternion'])
        w = vec(d['V1 Angular Velocity'])
        a = vec(d['V1 Linear Acceleration'])

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu'
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = map(float, q)
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = map(float, w)
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = map(float, a)
        P['imu'].publish(imu)

        rate = float(d['V1 LIDAR Scan Rate'])
        ranges = np.fromstring(gzip.decompress(base64.b64decode(d['V1 LIDAR Range Array'])).decode('utf-8'),
                               dtype=np.float32, sep='\n')
        ls = LaserScan()
        ls.header.stamp = stamp
        ls.header.frame_id = 'lidar'
        ls.angle_min = -2.35619
        ls.angle_max = 2.35619
        ls.angle_increment = 0.004363323
        ls.time_increment = (1.0 / rate) / 1080 if rate > 0 else 0.0
        ls.scan_time = 1.0 / rate if rate > 0 else 0.0
        ls.range_min = 0.06
        ls.range_max = 10.0
        ls.ranges = ranges.tolist()
        P['lidar'].publish(ls)

        if self.publish_camera:
            camera = np.asarray(PILImage.open(BytesIO(base64.b64decode(d['V1 Front Camera Image']))).convert('RGB'))
            camera_msg = self.cv_bridge.cv2_to_imgmsg(camera, encoding='rgb8')
            camera_msg.header.stamp = stamp
            camera_msg.header.frame_id = 'front_camera'
            P['camera'].publish(camera_msg)

        if self.publish_restricted_topics:
            pos = vec(d['V1 Position'])
            lv = vec(d['V1 Linear Velocity'])
            P['ips'].publish(Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])))

            od = Odometry()
            od.header.stamp = stamp
            od.header.frame_id = 'world'
            od.child_frame_id = 'roboracer_1'
            od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = map(float, pos)
            od.pose.pose.orientation = imu.orientation
            od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = map(float, lv)
            od.twist.twist.angular = imu.angular_velocity
            P['odom'].publish(od)

            P['lap_count'].publish(Int32(data=int(float(d['V1 Lap Count']))))
            P['collision_count'].publish(Int32(data=int(float(d['V1 Collisions']))))
            P['lap_time'].publish(Float32(data=float(d['V1 Lap Time'])))
            P['last_lap_time'].publish(Float32(data=float(d['V1 Last Lap Time'])))
            P['best_lap_time'].publish(Float32(data=float(d['V1 Best Lap Time'])))

        reset = self.reset_left > 0
        if reset:
            self.reset_left -= 1
        if reset or time.monotonic() - self.last_cmd_t > self.cmd_timeout:
            self.throttle_cmd = self.steering_cmd = 0.0
        self.frames += 1
        self._stats(t_in)
        return {'V1 Throttle': str(self.throttle_cmd),
                'V1 Steering': str(self.steering_cmd),
                'V1 Reset': str(reset)}

    def _stats(self, t_in):
        # report the sim's frame rate every 30 s; it is the control loop rate
        now = time.perf_counter()
        self.work_s += now - t_in
        self.window_frames += 1
        if self.window_start is None:
            self.window_start = now
        elif now - self.window_start > 30.0 or self.frames == 150:
            dt = now - self.window_start
            self.node.get_logger().info(
                f'{self.window_frames / dt:.1f} frames/s from sim, '
                f'bridge work {1e3 * self.work_s / self.window_frames:.1f} ms/frame')
            self.window_start, self.window_frames, self.work_s = now, 0, 0.0


def main():
    rclpy.init()
    bridge = FastBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(bridge.node)
    threading.Thread(target=executor.spin, daemon=True).start()

    sio = socketio.Server(async_mode='gevent')

    @sio.on('connect')
    def connect(sid, environ):
        bridge.node.get_logger().info('AutoDRIVE simulator connected')

    @sio.on('disconnect')
    def disconnect(sid):
        bridge.node.get_logger().warn('AutoDRIVE simulator disconnected')

    @sio.on('Bridge')
    def on_bridge(sid, data):
        if data:
            sio.emit('Bridge', data=bridge.on_frame(data))

    bridge.node.get_logger().info(f'Waiting for AutoDRIVE on port {bridge.port}')
    try:
        pywsgi.WSGIServer(('', bridge.port), socketio.WSGIApp(sio),
                          handler_class=WebSocketHandler, log=None).serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        bridge.node.destroy_node()
        rclpy.try_shutdown()
