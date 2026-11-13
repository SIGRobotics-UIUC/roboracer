#!/usr/bin/env python3
"""Starts and restarts Cartographer's localization trajectory, and relays its pose once it is localized.

Started by localization.launch.py (cartographer_node runs with -start_trajectory_with_default_topics
false). /tracked_pose is relayed unchanged on /localized_pose only while localized, so pure pursuit
does not drive before that (no poses -> no /drive -> the bridge stops the car after its cmd_timeout),
and not while extrapolated past the sensors (stall, below).

  start:     a trajectory with the initial pose identity relative to the frozen map trajectory
             (the car starts at the mapping start pose: race.sh resets the sim car there). The initial
             pose links the new trajectory to the map, so Cartographer matches locally right away.
  localized: the first /constraint_list after the number of constraints to the frozen map first rose
             (the optimization with that constraint runs after it is added).
  stall:     /tracked_pose is extrapolated to the current time (Cartographer's
             pose extrapolator), and /clock runs on while the sim stalls (fast_bridge's steady clock), so
             with no new sensor data the pose runs on at the car's last speed: 0.8-3 m ahead of a car that
             stood still for 0.4 s on Porto, then snapped back, and pure pursuit steered on it. A pose
             stamped more than max_pose_lead s after the newest /odom is therefore not relayed (normally it
             leads by 6 ms, 99 % under 30 ms; in a stall by the stall's length). Pure pursuit coasts
             through a short gap and stops after its max_pose_age. Without /odom everything is relayed.
             (Composing TF map -> odom with /odom instead, as mcl_node builds its pose, was worse on Porto:
             Cartographer's map -> odom jumps with every correction and in stalls; 2-3x the jitter.)
  sim reset: fast_bridge's /autodrive/roboracer_1/sim_reset event (reset command, respawn after a crash,
             any other teleport; the bridge also stops the car). The relay stops at once; after
             debounce s with the car still, the trajectory is finished, and once Cartographer has
             deleted it (pure localization trimmer) a new one starts at the ground truth pose mapped
             through map_to_world.yaml.
In competition there is no ground truth or collision counter, so fast_bridge only announces reset
commands: a respawn goes undetected, and only Cartographer's global search (localization.lua) can
recover. The real car has no respawns.
"""
import math
import os
import time

import yaml
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from cartographer_ros_msgs.msg import StatusCode, TrajectoryStates
from cartographer_ros_msgs.srv import FinishTrajectory, GetTrajectoryStates, StartTrajectory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from perception.map_origin import pose2d
from perception.tracks import track_dir

INTER_NS = 'Inter constraints, different trajectories'


def count_map_constraints(markers):
    """Constraints between trajectories in a /constraint_list (LINE_LIST: two points each)."""
    return sum(len(m.points) // 2 for m in markers.markers if m.ns == INTER_NS)


def world_to_map(origin, x, y, yaw):
    """Car pose in the sim world -> map frame; origin = (x, y, yaw) of the map origin in the world."""
    ox, oy, oyaw = origin
    c, s = math.cos(oyaw), math.sin(oyaw)
    dx, dy = x - ox, y - oy
    return c * dx + s * dy, -s * dx + c * dy, math.atan2(math.sin(yaw - oyaw), math.cos(yaw - oyaw))


class LocalizationGate(Node):
    def __init__(self, **kwargs):
        super().__init__('localization_gate', **kwargs)
        p = self.declare_parameter
        track = p('track', '').value
        self.config_dir = p('configuration_directory',
                            os.path.join(get_package_share_directory('perception'), 'config')).value
        self.config_basename = p('configuration_basename', 'localization.lua').value
        # s after the last reset event: merges a reset command's event with its teleport frame's (1-2
        # frames later) and covers the reset frames until the sim releases the car (~0.1 s)
        self.debounce = p('debounce', 0.5).value
        self.still_speed = p('still_speed', 0.1).value      # m/s: the car has stopped after a reset
        self.timeout = p('timeout', 10.0).value             # s per Cartographer step
        self.max_pose_lead = p('max_pose_lead', 0.06).value  # s a relayed pose may lead the newest /odom

        self.origin = None
        path = os.path.join(track_dir(track), 'map_to_world.yaml')
        if os.path.isfile(path):
            with open(path) as f:
                w = yaml.safe_load(f)
            self.origin = (w['x_m'], w['y_m'], w['yaw_rad'])

        self.frozen = None            # frozen map trajectory id
        self.tid = None               # current localization trajectory
        self.localized = False
        self.min_count = None         # fewest map constraints seen since the start (None: unknown)
        self.rose = False
        self.pending = None           # (reason, monotonic time of the last reset signal)
        self.gt = None                # (stamp s, x, y, speed, yaw)
        self.warn_t = 0.0
        self.odom_t = None            # stamp of the newest /odom
        self.held = 0                 # poses not relayed in the current stall
        self.stall_warn_t = 0.0

        self.pub = self.create_publisher(PoseStamped, '/localized_pose', 10)
        # first: rclpy runs ready subscriptions in creation order, so a reset closes the relay before
        # a pose that arrived with it is relayed
        self.create_subscription(String, '/autodrive/roboracer_1/sim_reset', self.reset_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, '/tracked_pose', self.pose_cb, 10)
        self.create_subscription(MarkerArray, '/constraint_list', self.constraints_cb, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.gt_cb, 10)

    # ---------------- relay and settle ----------------
    def odom_cb(self, msg):
        self.odom_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def pose_cb(self, msg):
        if self.localized:
            lead = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - self.odom_t \
                if self.odom_t is not None else 0.0
            if lead > self.max_pose_lead:     # extrapolated past the sensors: the sim stalled
                self.held += 1
                if self.held == 1 and time.monotonic() - self.stall_warn_t > 2.0:
                    self.stall_warn_t = time.monotonic()
                    self.get_logger().warning(f'no new odometry for {lead:.2f} s (sim stall?): holding '
                                              'the extrapolated pose back')
                return
            self.held = 0
            self.pub.publish(msg)   # stamp and frame_id unchanged
        elif time.monotonic() - self.warn_t > 2.0:
            self.warn_t = time.monotonic()
            self.get_logger().warning(f'waiting for localization (trajectory {self.tid}), not driving')

    def constraints_cb(self, msg):
        if self.localized or self.tid is None or self.pending:
            return
        n = count_map_constraints(msg)
        if self.rose and n > self.min_count:
            self.localized = True
            self.get_logger().info(f'localized (trajectory {self.tid}, {n} constraints to the map), driving')
        elif self.min_count is not None and n > self.min_count:
            self.rose = True
        else:
            self.min_count = n if self.min_count is None else min(self.min_count, n)

    # ---------------- sim reset signals ----------------
    def trigger(self, reason):
        if self.localized or not self.pending:
            self.get_logger().warning(f'sim reset ({reason}): relocalizing, not driving')
        self.localized = False
        self.pending = (reason, time.monotonic())

    def reset_cb(self, msg):
        self.trigger(msg.data)   # the reason fast_bridge saw

    def gt_cb(self, msg):
        x, y, yaw = pose2d(msg.pose.pose)
        v = msg.twist.twist.linear
        self.gt = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, x, y, math.hypot(v.x, v.y), yaw)

    def restart_due(self, now):
        if not self.pending or now - self.pending[1] < self.debounce:
            return False
        # stopped (respawn and reset zero the velocity), or give up waiting after timeout
        return self.gt is None or self.gt[3] < self.still_speed or now - self.pending[1] > self.timeout

    # ---------------- Cartographer ----------------
    def call(self, srv_type, name, request):
        client = self.create_client(srv_type, name)
        try:
            if not client.wait_for_service(timeout_sec=self.timeout):
                raise RuntimeError(f'service {name} not available (is cartographer_node running?)')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
            if future.result() is None:
                raise RuntimeError(f'{name} timed out')
            res = future.result()
            if res.status.code != StatusCode.OK:
                raise RuntimeError(f'{name}: {res.status.message}')
            return res
        finally:
            self.destroy_client(client)

    def states(self):
        s = self.call(GetTrajectoryStates, '/get_trajectory_states', GetTrajectoryStates.Request()).trajectory_states
        return dict(zip(s.trajectory_id, s.trajectory_state))

    def start(self, pose=None):
        """Start a localization trajectory at pose (x, y, yaw) in the map, identity if None."""
        if self.frozen is None:
            frozen = [i for i, s in self.states().items() if s == TrajectoryStates.FROZEN]
            if not frozen:
                raise RuntimeError('no frozen trajectory (map.pbstream not loaded?)')
            self.frozen = frozen[0]
        old, self.tid = self.tid, None   # constraints_cb ignores the old trajectory's constraints
        self.localized = False
        deleted = True
        if old is not None:
            try:
                self.call(FinishTrajectory, '/finish_trajectory', FinishTrajectory.Request(trajectory_id=old))
            except RuntimeError:
                self.tid = old   # still active: finish it on the retry
                raise
            # its constraints to the map would count as new ones until the trimmer deletes it
            end = time.monotonic() + self.timeout
            while self.states().get(old) != TrajectoryStates.DELETED and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.1)
            deleted = self.states().get(old) == TrajectoryStates.DELETED
            if not deleted:
                self.get_logger().warning(f'trajectory {old} not deleted after {self.timeout:.0f} s')
        x, y, yaw = pose or (0.0, 0.0, 0.0)
        req = StartTrajectory.Request(configuration_directory=self.config_dir,
                                      configuration_basename=self.config_basename,
                                      use_initial_pose=True, relative_to_trajectory_id=self.frozen)
        req.initial_pose.position.x, req.initial_pose.position.y = float(x), float(y)
        req.initial_pose.orientation.z, req.initial_pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        tid = self.call(StartTrajectory, '/start_trajectory', req).trajectory_id
        self.rose, self.min_count = False, 0 if deleted else None
        self.tid = tid
        self.get_logger().info(f'started trajectory {self.tid} at x={x:.2f} y={y:.2f} yaw={yaw:.2f} '
                               f'(relative to frozen trajectory {self.frozen})')

    def relocalize(self):
        reason, _ = self.pending
        self.pending = None
        if self.gt is None or self.origin is None:
            self.get_logger().warning('no ground truth or map_to_world.yaml: assuming the car is at the map origin')
            pose = None
        else:
            pose = world_to_map(self.origin, self.gt[1], self.gt[2], self.gt[4])
        self.get_logger().info(f'relocalizing after {reason}')
        try:
            self.start(pose)
        except RuntimeError as e:
            self.get_logger().error(f'{e}; retrying')
            self.pending = (reason, time.monotonic())   # the debounce loop retries

    def try_start(self):
        """The first start; False (logged) if Cartographer is not ready yet (pbstream still loading)."""
        try:
            self.start()
            return True
        except RuntimeError as e:
            self.get_logger().error(f'{e}; retrying in 2 s')
            return False


def main():
    rclpy.init()
    node = LocalizationGate()
    try:
        while rclpy.ok() and not node.try_start():
            end = time.monotonic() + 2.0
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.restart_due(time.monotonic()):
                node.relocalize()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:   # a second Ctrl-C during cleanup
            pass


if __name__ == '__main__':
    main()
