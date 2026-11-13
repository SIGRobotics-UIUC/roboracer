#!/usr/bin/env python3
"""Bounded Porto straight-line test through /drive, with no MAP/localizer.

Ground truth is used for experiment placement and safety bounds only. The adapter
retains its normal wheel-odometry speed feedback. Requires porto_odom_trial map origin.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, Int32
    from ackermann_msgs.msg import AckermannDriveStamped
    import yaml
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--speed', type=float, default=1.5)
    ap.add_argument('--output', required=True)
    args = ap.parse_args()
    if not .5 <= args.speed <= 2.:
        ap.error('speed must be between 0.5 and 2 m/s')
    if Path(args.output + '.csv').exists():
        ap.error('output already exists')
    with open('tracks/porto_odom_trial/map_to_world.yaml') as stream:
        origin = yaml.safe_load(stream)
    rclpy.init()
    n = rclpy.create_node('straight_line_trial')
    latest, samples = {}, []
    phase = 'discovery'
    def yaw(q):
        return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    def gt(msg):
        p = msg.pose.pose.position
        dx, dy = p.x-origin['x_m'], p.y-origin['y_m']
        c, s = math.cos(origin['yaw_rad']), math.sin(origin['yaw_rad'])
        a = yaw(msg.pose.pose.orientation)-origin['yaw_rad']
        a = math.atan2(math.sin(a), math.cos(a))
        v = msg.twist.twist.linear
        latest['gt'] = (time.monotonic(), c*dx+s*dy, -s*dx+c*dy, a, v.x)
        samples.append(dict(stamp_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,
                            phase=phase, x=latest['gt'][1], y=latest['gt'][2],
                            yaw=a, vx=v.x, vy=v.y))
    def scan(msg):
        ranges = [r for i, r in enumerate(msg.ranges)
                  if abs(msg.angle_min+i*msg.angle_increment) < .14 and math.isfinite(r)]
        latest['scan'] = (time.monotonic(), min(ranges) if ranges else float('inf'))
    subscriptions = [n.create_subscription(Odometry, '/ground_truth/odom', gt, 100),
                     n.create_subscription(LaserScan, '/scan', scan, 10),
                     n.create_subscription(Int32, '/autodrive/roboracer_1/collision_count',
                                           lambda m: latest.update(collision=m.data), 10)]
    pub = n.create_publisher(AckermannDriveStamped, '/drive', 10)
    reset = n.create_publisher(Bool, '/autodrive/reset_command', 10)
    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=.01)
    def command(speed):
        msg = AckermannDriveStamped()
        msg.header.stamp = n.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = speed
        pub.publish(msg)
    def hold_stop(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            command(0.)
            spin(.025)
    def stop_process(proc):
        if proc and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=5)
    bag = follower = None
    result = dict(speed_request_mps=args.speed, status='starting')
    try:
        spin(3)
        if 'gt' not in latest:
            raise RuntimeError('No connected simulator ground truth')
        phase = 'reset'
        for _ in range(3):
            reset.publish(Bool(data=True))
            spin(.1)
        hold_stop(2)
        baseline = latest.get('collision')
        topics = ['/odom', '/ground_truth/odom', '/scan', '/sensors/imu/raw', '/drive',
                  '/autodrive/roboracer_1/left_encoder', '/autodrive/roboracer_1/right_encoder',
                  '/autodrive/roboracer_1/throttle', '/autodrive/roboracer_1/steering',
                  '/autodrive/roboracer_1/throttle_command', '/autodrive/roboracer_1/collision_count']
        with open(args.output+'.bag.log', 'w') as baglog, open(args.output+'.position.log', 'w') as log:
            bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', args.output+'_bag', *topics],
                                   stdout=baglog, stderr=subprocess.STDOUT, start_new_session=True)
            phase = 'positioning'
            follower = subprocess.Popen(['ros2', 'launch', 'control', 'wall_follow.launch.py',
                                         'speed:=1.0', 'laps:=0'], stdout=log,
                                        stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+100
            while True:
                spin(.025)
                arrival, x, y, a, v = latest['gt']
                if latest.get('collision') != baseline:
                    raise RuntimeError('Collision during positioning')
                if follower.poll() is not None or time.monotonic() > deadline or time.monotonic()-arrival > .6:
                    raise RuntimeError('Positioning stopped or telemetry timed out')
                if -7.5 < x < -6.0 and abs(y) < .2 and abs(a) < .08:
                    break
            stop_process(follower)
            phase = 'settling'
            hold_stop(2)
            _, x0, y0, a0, _ = latest['gt']
            if not (-8 < x0 < -5.5 and abs(y0) < .25 and abs(a0) < .1):
                raise RuntimeError('Invalid straight-line starting pose')
            result['start_map_pose'] = [x0, y0, a0]
            phase = 'accelerate_cruise'
            start = time.monotonic()
            while time.monotonic()-start < 12:
                arrival, x, y, a, v = latest['gt']
                travel = (x-x0)*math.cos(a0)+(y-y0)*math.sin(a0)
                cross = -(x-x0)*math.sin(a0)+(y-y0)*math.cos(a0)
                age, clearance = latest.get('scan', (0., 0.))
                if time.monotonic()-arrival > .35 or time.monotonic()-age > .5:
                    raise RuntimeError('Telemetry gap: stopping')
                if latest.get('collision') != baseline:
                    raise RuntimeError('Collision: stopping')
                if abs(cross) > .22 or abs(a-a0) > .12 or clearance < max(1., v*v/4+.4):
                    result['bound_stop'] = dict(cross=cross, yaw=a-a0, clearance=clearance)
                    result['profile_end'] = 'safety_bound'
                    break
                if travel >= 3.5 or x >= -2.0:
                    result['profile_end'] = 'distance_bound'
                    break
                command(args.speed)
                spin(.025)
            else:
                raise RuntimeError('Profile duration limit reached before distance bound')
            phase = 'braking'
            hold_stop(3)
            result.update(status='completed', end_map_pose=list(latest['gt'][1:4]),
                          collision_change=latest.get('collision', 0)-(baseline or 0))
    except Exception as exc:
        result.update(status='aborted', error=str(exc))
    finally:
        stop_process(follower)
        phase = 'final_stop'
        hold_stop(1)
        stop_process(bag)
        if samples:
            with open(args.output+'.csv', 'w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(samples[0]))
                writer.writeheader()
                writer.writerows(samples)
        Path(args.output+'.json').write_text(json.dumps(result, indent=2)+'\n')
        n.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, indent=2))
    if result['status'] != 'completed':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
