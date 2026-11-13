"""Publish a synthetic opponent on /perception/obstacles.

Real opponent detection is the largest piece of work on the overtaking roadmap
and the slowest to debug. Nothing downstream of it should wait. This node
publishes the same message a LiDAR-based detector eventually will, so planners
can be written and tested today, in the single-car simulator, against an
opponent that behaves exactly as the experiment requires. When detection lands,
this node is switched off and nothing else changes.

The motion and sensor models live in opponent_models.py, which imports no ROS
and is unit tested there. This file is only plumbing.

Modes:
    static          a cone or parked car, for stationary-obstacle planning
    constant_speed  holds a fixed offset at a fixed speed
    weaving         d is a sine in arc length, so the line repeats lap to lap
                    and a GP has something real to learn
    replay          follows a recorded line from a CSV of s,d

Every sensor parameter defaults to the ideal, which makes the opponent a
perfect oracle out of the box. That default is for bring-up only. A planner
validated against a noiseless, never-dropping, zero-latency opponent has not
been validated, and a GP given clean data is not being exercised at all.
"""

import dataclasses
import math
import os

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker

from f1t_msgs.msg import Obstacle, ObstacleArray
from planning.frenet import FrenetConverter
from planning.opponent_models import SensorModel, motion_from_params, observe


class FakeOpponentNode(Node):

    def __init__(self):
        super().__init__('fake_opponent_node')
        p = lambda n, v: self.declare_parameter(n, v).value

        tracks_dir = p('tracks_dir', os.environ.get('TRACKS_DIR',
                                                    os.path.expanduser('~/ws/tracks')))
        track_name = p('track_name', '')
        self.reference = p('reference', 'raceline')
        ego_topic = p('ego_topic', '/car_state/odom_frenet')
        out_topic = p('out_topic', '/perception/obstacles')
        marker_topic = p('marker_topic', '/opponent_marker')
        self.map_frame = p('map_frame', 'map')
        rate = float(p('rate', 40.0))

        if not track_name:
            raise RuntimeError('fake_opponent_node needs the track_name parameter')
        track_dir = os.path.join(tracks_dir, track_name)
        if self.reference == 'raceline':
            self.conv = FrenetConverter.from_track(track_dir)
        else:
            self.conv = FrenetConverter.from_centerline(
                os.path.join(track_dir, 'centerline.csv'))
        self.lap = float(self.conv.length)

        replay_file = p('replay_file', '')
        replay_s = replay_d = None
        if replay_file:
            arr = np.loadtxt(replay_file, delimiter=',', ndmin=2)
            replay_s, replay_d = arr[:, 0], arr[:, 1]

        self.motion = motion_from_params(
            mode=p('mode', 'constant_speed'),
            lap_length=self.lap,
            speed=float(p('speed', 2.0)),
            s0=float(p('s0', 0.0)),
            d=float(p('d', 0.0)),
            amplitude=float(p('amplitude', 0.4)),
            wavelength=float(p('wavelength', 12.0)),
            replay_s=replay_s,
            replay_d=replay_d,
        )
        self.sensor = SensorModel(
            max_range=float(p('max_range', 10.0)),
            rear_range=float(p('rear_range', 0.0)),
            detection_prob=float(p('detection_prob', 1.0)),
            sigma_s=float(p('sigma_s', 0.0)),
            sigma_d=float(p('sigma_d', 0.0)),
            sigma_vs=float(p('sigma_vs', 0.0)),
            sigma_vd=float(p('sigma_vd', 0.0)),
            latency=float(p('latency', 0.0)),
            length=float(p('length', 0.50)),
            width=float(p('width', 0.30)),
            inflation=float(p('inflation', 0.05)),
        )
        # Seeded so a run is reproducible; two planners must face the same noise
        # or the comparison between them measures the random number generator.
        self.rng = np.random.default_rng(int(p('seed', 0)))
        self.remember_static = bool(p('remember_static', True))

        self.ego_s = None
        self.last_obs = None
        self.warned_frame = False
        self.t0 = self.get_clock().now()

        self.pub = self.create_publisher(ObstacleArray, out_topic, 10)
        self.marker_pub = self.create_publisher(Marker, marker_topic, 10)
        self.create_subscription(Odometry, ego_topic, self.ego_cb, 10)
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'{self.motion.__class__.__name__} opponent on {track_name}, lap {self.lap:.2f} m, '
            f'publishing {out_topic} at {rate:.0f} Hz against the {self.reference}')

    def ego_cb(self, msg):
        self.ego_s = float(msg.pose.pose.position.x)
        expect = f'frenet/{self.reference}'
        if msg.header.frame_id and msg.header.frame_id != expect and not self.warned_frame:
            self.warned_frame = True
            self.get_logger().error(
                f'ego state is measured against {msg.header.frame_id} but this node uses '
                f'{expect}. d means a different curve on each side and every overtake will '
                f'pick the wrong side. Fix the reference parameter before trusting anything.')

    def tick(self):
        if self.ego_s is None:
            return   # no ego state yet, so visibility is undefined

        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        obs = observe(self.motion, t, self.ego_s, self.lap, self.sensor, self.rng)

        if obs is None and self.motion.is_static and self.remember_static and self.last_obs:
            # A cone does not cease to exist when it leaves sensor range.
            obs = dataclasses.replace(self.last_obs, is_visible=False)
        elif obs is not None:
            self.last_obs = obs

        msg = ObstacleArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'frenet/{self.reference}'
        if obs is not None:
            msg.obstacles = [self._to_msg(obs)]
        # An empty array is published deliberately: consumers must be able to
        # tell a clear track from a dead perception stack.
        self.pub.publish(msg)
        self._publish_marker(obs)

    def _to_msg(self, o):
        m = Obstacle()
        m.id = int(o.id)
        for f in ('s_start', 's_end', 'd_right', 'd_left', 's_center', 'd_center',
                  'size', 'vs', 'vd', 's_var', 'd_var', 'vs_var', 'vd_var'):
            setattr(m, f, float(getattr(o, f)))
        m.is_actually_a_gap = bool(o.is_actually_a_gap)
        m.is_static = bool(o.is_static)
        m.is_visible = bool(o.is_visible)
        return m

    def _publish_marker(self, o):
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'fake_opponent'
        m.id = 0
        m.type = Marker.CUBE
        if o is None:
            m.action = Marker.DELETE
            self.marker_pub.publish(m)
            return
        m.action = Marker.ADD
        x, y = self.conv.to_cartesian(o.s_center % self.lap, o.d_center)
        yaw = float(self.conv.heading_at(o.s_center % self.lap))
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        m.scale.x, m.scale.y, m.scale.z = self.sensor.length, self.sensor.width, 0.2
        m.color.a = 1.0 if o.is_visible else 0.3
        m.color.r, m.color.g, m.color.b = (0.6, 0.6, 0.6) if o.is_static else (1.0, 0.3, 0.0)
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = FakeOpponentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
