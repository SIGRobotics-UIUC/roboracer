#!/usr/bin/env python3
"""Lap logger: times every lap the car drives on a track and writes one CSV row per lap.

    ros2 run control lap_logger_node --ros-args -p track:=<name>     (race.launch.py starts it, log_laps:=true)

Laps come from the car's pose along the raceline (control.lap_stats): no sim topics are needed, so it
also works on the real car. In the sim it also reads the bridge's restricted topics (when the bridge
publishes them): collisions during each lap, and sim_reset, which aborts the lap being timed (the car
was respawned or reset, so that lap is not a clean one). Localizer pose jumps are counted per lap; one
over max_jump (3 m) aborts the lap.

Writes tracks/<track>/runs/<UTC time>.csv (out_dir overrides the folder), one row per finished or aborted
lap, flushed as it happens so Ctrl-C loses nothing (no file if no lap started). Its first line is
'# run: {json}': the track, the start time, the raceline's provenance and the race settings it was given
(controller, speed_scale, max_speed, localizer, avoid), for `ros2 run control lap_report <track>` to compare runs.
Logs each lap and a summary on exit.

Subscribed:
  pose_topic (geometry_msgs/PoseStamped, map frame; default /localized_pose)
  <vehicle_ns>/collision_count (std_msgs/Int32), <vehicle_ns>/sim_reset (std_msgs/String)  sim only
"""
import csv
import json
import os
import signal
import time
from datetime import datetime, timezone

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Int32, String

from control.lap_stats import LapTracker, summary
from perception.tracks import track_dir
from planning.raceline import read_provenance

COLUMNS = ['lap', 'status', 'start', 'time_s', 'distance_m', 'mean_speed_mps', 'max_speed_mps',
           'mean_offset_m', 'max_offset_m', 'collisions', 'pose_jumps', 'max_jump_m', 'note']
SETTINGS = ('controller', 'speed_scale', 'max_speed', 'localizer', 'avoid')   # recorded in the header only


class LapLogger(Node):
    def __init__(self):
        super().__init__('lap_logger')
        p = self.declare_parameter
        track = p('track', '').value
        raceline_path = p('raceline_path', '').value       # overrides track
        pose_topic = p('pose_topic', '/localized_pose').value
        vehicle_ns = p('vehicle_ns', '/autodrive/roboracer_1').value
        out_dir = p('out_dir', '').value                   # default tracks/<track>/runs
        max_jump = p('max_jump', 3.0).value                # m: a longer pose jump aborts the lap (car moved)
        # race settings, recorded as given: any type, so speed_scale:=1 and :=1.0 both work
        any_type = ParameterDescriptor(dynamic_typing=True)
        self.settings = {k: p(k, None, any_type).value for k in SETTINGS}

        if not raceline_path and not track:
            raise ValueError('set track or raceline_path')
        path = raceline_path or os.path.join(track_dir(track), 'raceline.csv')
        if not os.path.isfile(path):
            self.get_logger().fatal(f'Raceline not found: {path} (set track or raceline_path)')
            raise FileNotFoundError(path)
        xy = np.loadtxt(path, delimiter=';', ndmin=2)[:, 1:3]
        self.tracker = LapTracker(xy, max_jump=max_jump)

        started = datetime.now(timezone.utc)
        out_dir = out_dir or os.path.join(track_dir(track) if track else os.path.dirname(path), 'runs')
        os.makedirs(out_dir, exist_ok=True)
        self.out_path = os.path.join(out_dir, started.strftime('%Y%m%d-%H%M%S') + '.csv')
        try:
            provenance = read_provenance(path)
        except ValueError:
            provenance = None
        header = {'track': track or None, 'started': started.isoformat(timespec='seconds'), 'raceline': path,
                  'raceline_provenance': provenance, 'lap_length_m': round(self.tracker.length, 3),
                  'pose_topic': pose_topic, **self.settings}
        self.file = open(self.out_path, 'w', newline='')
        self.file.write('# run: ' + json.dumps(header) + '\n')
        self.writer = csv.writer(self.file)
        self.writer.writerow(COLUMNS)
        self.file.flush()

        self.collisions = None      # the sim's count at the start of the lap being timed
        self.collision_count = None
        self.create_subscription(PoseStamped, pose_topic, self.pose_cb, 10)
        self.create_subscription(Int32, vehicle_ns + '/collision_count', self.collision_cb, 10)
        self.create_subscription(String, vehicle_ns + '/sim_reset', self.reset_cb, 10)
        self.get_logger().info(f'Logging laps of {path} ({self.tracker.length:.1f} m) on {pose_topic} '
                               f'to {self.out_path}')

    def pose_cb(self, msg):
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9 or time.time()
        timing = self.tracker.acc is not None
        lap = self.tracker.update(t, msg.pose.position.x, msg.pose.position.y)
        self.record(lap)
        if self.tracker.acc is not None and (not timing or lap is not None):
            self.collisions = self.collision_count     # a new lap started

    def collision_cb(self, msg):
        self.collision_count = msg.data

    def reset_cb(self, msg):
        self.record(self.tracker.abort(f'sim reset: {msg.data}'))

    def record(self, lap):
        if lap is None:
            return
        hits = ('' if self.collision_count is None or self.collisions is None
                else self.collision_count - self.collisions)
        self.writer.writerow([lap.number, lap.status, lap.start, f'{lap.time_s:.3f}', f'{lap.distance_m:.2f}',
                              f'{lap.mean_speed_mps:.2f}', f'{lap.max_speed_mps:.2f}', f'{lap.mean_offset_m:.3f}',
                              f'{lap.max_offset_m:.3f}', hits, lap.pose_jumps, f'{lap.max_jump_m:.2f}', lap.note])
        self.file.flush()
        text = (f'Lap {lap.number} ({lap.start}): {lap.time_s:.2f} s, mean {lap.mean_speed_mps:.2f} m/s, '
                f'off the raceline mean {lap.mean_offset_m:.2f} m / max {lap.max_offset_m:.2f} m'
                + (f', {lap.pose_jumps} pose jumps (max {lap.max_jump_m:.2f} m)' if lap.pose_jumps else ''))
        if lap.status == 'ok':
            self.get_logger().info(text)
        else:
            self.get_logger().warning(f'{text}: ABORTED ({lap.note})')

    def close(self):
        self.file.close()
        if not self.tracker.laps:      # no lap started: leave no empty run behind
            os.remove(self.out_path)
            self.get_logger().info('No laps; nothing written')
            return
        self.get_logger().info(f'{summary(self.tracker.laps)}. Written to {self.out_path}')


def main():
    # Handle Ctrl+C ourselves, so the summary is still logged before ROS shuts down
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = LapLogger()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    signal.signal(signal.SIGINT, signal.SIG_IGN)   # a second Ctrl-C (launch forwards one) must not cut the file short
    node.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
