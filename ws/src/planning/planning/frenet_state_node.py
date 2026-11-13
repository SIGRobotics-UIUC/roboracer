"""Publish the car's own state in Frenet coordinates on /car_state/odom_frenet.

Both overtaking planners subscribe to this topic and nothing in the stack
produced it, so every planner node is blocked until it exists. The work is only
a wrapper around FrenetConverter; the value is that one node owns the
conversion and every consumer agrees about what s and d mean.

WHICH REFERENCE LINE. d is measured against the raceline by default. ForzaETH
measure it against the centerline. The choice is arbitrary but it must be the
same everywhere, because d = 0 meaning two different curves on two branches
would make any comparison between those branches meaningless while looking
entirely healthy. The reference is named in header.frame_id so a consumer can
detect the mismatch at runtime instead of discovering it in a race.

PACKING. nav_msgs/Odometry is reused with Frenet values in the Cartesian slots,
following ForzaETH so their planner code ports unedited:

    pose.pose.position.x  = s        twist.twist.linear.x = vs
    pose.pose.position.y  = d        twist.twist.linear.y = vd

pose.pose.orientation additionally carries the car's heading relative to the
reference tangent, as a yaw-only quaternion. ForzaETH leave it empty; a planner
running a bicycle model in Frenet needs it, and filling it costs nothing.

POSE SOURCE. The pose comes from /localized_pose: perception's
localization_gate (Cartographer) or mcl_node, one of which race.launch.py
always starts. AGENTS.md: racing consumes /localized_pose and must not
bypass the gate with /tracked_pose. The gate withholds the pose until the
car is localized and while Cartographer extrapolates it past the sensors:
in a sim stall /tracked_pose ran 0.8-3 m ahead of a car that stood still,
then snapped back (localization_gate.py), and every planner downstream of
this node would have planned from that. So nothing is published until the
car is localized, which is what the consumers should see. Only for a run
without a localizer, such as mapping, set pose_topic:=/tracked_pose.
"""

import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from planning.frenet import FrenetConverter


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_to_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class FrenetStateNode(Node):

    def __init__(self, **kwargs):
        super().__init__('frenet_state_node', **kwargs)   # kwargs: Node's, e.g. parameter_overrides (tests)
        p = lambda n, v: self.declare_parameter(n, v).value

        tracks_dir = p('tracks_dir', os.environ.get('TRACKS_DIR',
                                                    os.path.expanduser('~/ws/tracks')))
        track_name = p('track_name', '')
        reference = p('reference', 'raceline')       # 'raceline' | 'centerline'
        pose_topic = p('pose_topic', '/localized_pose')   # see POSE SOURCE
        odom_topic = p('odom_topic', '/odom')
        out_topic = p('out_topic', '/car_state/odom_frenet')

        if not track_name:
            raise RuntimeError('frenet_state_node needs the track_name parameter')
        track_dir = os.path.join(tracks_dir, track_name)

        if reference == 'raceline':
            self.conv = FrenetConverter.from_track(track_dir)
        elif reference == 'centerline':
            self.conv = FrenetConverter.from_centerline(
                os.path.join(track_dir, 'centerline.csv'))
        else:
            raise RuntimeError(f"reference must be 'raceline' or 'centerline', got {reference!r}")

        self.frame = f'frenet/{reference}'
        self.speed = 0.0
        self.have_odom = False

        self.pub = self.create_publisher(Odometry, out_topic, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.create_subscription(PoseStamped, pose_topic, self.pose_cb, 10)

        self.get_logger().info(
            f'{self.conv!r} on {track_name}: d measured against the {reference}, '
            f'lap {self.conv.length:.2f} m, publishing {out_topic} as {self.frame}')

    def odom_cb(self, msg):
        # nav_msgs/Odometry twist is in the body frame by convention, and
        # wheel_odom fills linear.x with forward speed only.
        self.speed = float(msg.twist.twist.linear.x)
        self.have_odom = True

    def pose_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.orientation)

        s, d = self.conv.to_frenet(x, y)
        s, d = float(s), float(d)

        vs, vd = self.conv.velocity_to_frenet(
            s, d, self.speed * math.cos(yaw), self.speed * math.sin(yaw))

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame
        out.child_frame_id = 'base_link'
        out.pose.pose.position.x = s
        out.pose.pose.position.y = d
        heading_error = wrap_to_pi(yaw - float(self.conv.heading_at(s)))
        out.pose.pose.orientation.z = math.sin(heading_error / 2.0)
        out.pose.pose.orientation.w = math.cos(heading_error / 2.0)
        out.twist.twist.linear.x = float(vs)
        out.twist.twist.linear.y = float(vd)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FrenetStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
