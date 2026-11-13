#!/usr/bin/env python3
"""Logs how far a localizer's pose is from the sim's ground truth (simulation debugging only).

    ros2 run perception localization_debug --ros-args -p track:=<name> [-p pose_topic:=/localized_pose]

Works with any localizer: it only reads a PoseStamped in the map frame, /ground_truth/odom and /odom, and
changes nothing. Needs tracks/<name>/map_to_world.yaml and publish_restricted_topics (the bridge's default);
the real car and competition have no ground truth.

Each pose is compared with the newest ground truth when it arrives, so the error includes the localizer's
computation delay: it is what pure pursuit sees. long > 0 = the estimate is ahead of the car, lat > 0 = left of it.
withheld: share of the time the car was moving without a pose for max_pose_age s (not localized, or lost).
odom/truth: distance by wheel odometry over true distance; 1.0 = the odometry reports distance correctly.
"""
import math
import os
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from perception.localization_gate import world_to_map
from perception.map_origin import pose2d
from perception.tracks import track_dir

MAX_STEP = 1.0   # m between two messages: a bigger one is a teleport, not driving


def pose_error(est, truth):
    """(distance, along the car, across the car, yaw) of est from truth, in the car's frame."""
    dx, dy = est[0] - truth[0], est[1] - truth[1]
    c, s = math.cos(truth[2]), math.sin(truth[2])
    dyaw = est[2] - truth[2]
    return math.hypot(dx, dy), c * dx + s * dy, -s * dx + c * dy, math.atan2(math.sin(dyaw), math.cos(dyaw))


def summary(errors):
    """Mean, 95th percentile and maximum, as text."""
    if not errors:
        return 'no pose'
    e = np.asarray(errors)
    return f'mean {e.mean():.3f} p95 {np.percentile(e, 95):.3f} max {e.max():.2f} m'


class Path:
    """Distance driven along a stream of poses, without teleports."""

    def __init__(self):
        self.last = None
        self.length = 0.0

    def add(self, x, y):
        if self.last is not None:
            step = math.hypot(x - self.last[0], y - self.last[1])
            if step < MAX_STEP:
                self.length += step
        self.last = (x, y)


class LocalizationDebug(Node):
    def __init__(self):
        super().__init__('localization_debug')
        p = self.declare_parameter
        track = p('track', '').value
        pose_topic = p('pose_topic', '/localized_pose').value
        self.period = p('log_period', 2.0).value             # s between log lines
        self.max_pose_age = p('max_pose_age', 0.5).value     # s without a pose: withheld
        self.still_speed = p('still_speed', 0.1).value       # m/s: below this the car is not expected to have one

        path = os.path.join(track_dir(track), 'map_to_world.yaml')
        if not os.path.isfile(path):
            raise RuntimeError(f'{path} not found: mapped without ground truth?')
        with open(path) as f:
            w = yaml.safe_load(f)
        self.origin = (w['x_m'], w['y_m'], w['yaw_rad'])

        self.truth = None             # newest ground truth, in the map frame
        self.last_error = None        # newest pose_error()
        self.window = []              # errors [m] since the last log line
        self.errors = []              # errors [m] of the whole run
        self.pose_t = None            # monotonic time of the newest pose
        self.moving = 0               # ground truth messages with the car moving ...
        self.withheld = 0             # ... and no pose
        self.odom_path = Path()
        self.truth_path = Path()

        self.create_subscription(Odometry, '/ground_truth/odom', self.truth_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(PoseStamped, pose_topic, self.pose_cb, 10)
        self.create_timer(self.period, self.report)
        self.get_logger().info(f'comparing {pose_topic} with /ground_truth/odom on track {track}')

    def truth_cb(self, msg):
        x, y, yaw = pose2d(msg.pose.pose)
        self.truth = world_to_map(self.origin, x, y, yaw)
        self.truth_path.add(x, y)
        v = msg.twist.twist.linear
        if math.hypot(v.x, v.y) >= self.still_speed:
            self.moving += 1
            if self.pose_t is None or time.monotonic() - self.pose_t > self.max_pose_age:
                self.withheld += 1

    def odom_cb(self, msg):
        x, y, _ = pose2d(msg.pose.pose)
        self.odom_path.add(x, y)

    def pose_cb(self, msg):
        self.pose_t = time.monotonic()
        if self.truth is None:
            return
        self.last_error = pose_error(pose2d(msg.pose), self.truth)
        self.window.append(self.last_error[0])
        self.errors.append(self.last_error[0])

    def totals(self):
        ratio = (f'{self.odom_path.length / self.truth_path.length:.2f}'
                 if self.truth_path.length > 0.5 else 'n/a')
        withheld = 100.0 * self.withheld / self.moving if self.moving else 0.0
        return f'odom/truth {ratio} over {self.truth_path.length:.1f} m | withheld {withheld:.1f} %'

    def report(self):
        if self.truth is None:
            self.get_logger().warning('no /ground_truth/odom: is the bridge running with publish_restricted_topics?')
            return
        if not self.window:
            self.get_logger().warning(f'no pose for {self.period:.0f} s | {self.totals()}')
            return
        err, long, lat, yaw = self.last_error
        self.get_logger().info(f'err {err:.2f} m (long {long:+.2f}, lat {lat:+.2f}, yaw {math.degrees(yaw):+.1f} deg) | '
                               f'last {self.period:.0f} s: {summary(self.window)} | run: {summary(self.errors)} | '
                               f'{self.totals()}')
        self.window = []

    def final_report(self):
        self.get_logger().info(f'run total: {summary(self.errors)} over {len(self.errors)} poses | {self.totals()}')


def main():
    rclpy.init()
    node = LocalizationDebug()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.final_report()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
