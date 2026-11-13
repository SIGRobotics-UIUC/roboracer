#!/usr/bin/env python3
"""Compare timestamp-aligned wheel odometry and simulator velocity in a ROS 2 bag.

Run inside a sourced ROS container: python3 scripts/analyze_odom_bag.py BAG_DIR
Ground truth is used offline only. No controller inputs are changed.
"""
import argparse
import json
from pathlib import Path
import sqlite3

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def analyze(path):
    if not (Path(path) / 'metadata.yaml').exists():
        raise ValueError('Stop recording first: finalized bag metadata.yaml is required')
    data = {}
    for db in sorted(Path(path).glob('*.db3')):
        with sqlite3.connect(db.resolve().as_uri() + '?mode=ro&immutable=1', uri=True) as conn:
            topics = {i: (name, get_message(kind)) for i, name, kind in
                      conn.execute('SELECT id,name,type FROM topics')}
            for topic, blob in conn.execute('SELECT topic_id,data FROM messages ORDER BY timestamp'):
                name, kind = topics[topic]
                msg = deserialize_message(blob, kind)
                if not hasattr(msg, 'header'):
                    continue
                t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                if name in ('/odom', '/ground_truth/odom'):
                    p = msg.pose.pose.position
                    v = msg.twist.twist.linear
                    values = (v.x, p.x, p.y, msg.twist.twist.angular.z)
                elif name.endswith('_encoder'):
                    values = (msg.position[0],)
                else:
                    continue
                data.setdefault(name, {})[t] = values
    wheel, gt = data['/odom'], data['/ground_truth/odom']
    times = sorted(wheel.keys() & gt.keys())
    if len(times) < 3:
        raise ValueError('Need at least three exact timestamp pairs for /odom and /ground_truth/odom')
    t = np.array(times)
    w = np.array([wheel[k] for k in times])
    g = np.array([gt[k] for k in times])
    moving = g[:, 0] > .5
    error = w[:, 0] - g[:, 0]
    def stats(a):
        a = a[np.isfinite(a)]
        return dict(samples=len(a), median=float(np.median(a)),
                    p95_abs=float(np.percentile(abs(a), 95))) if len(a) else None
    result = dict(exact_stamp_pairs=len(t), moving_speed_error_mps=stats(error[moving]),
                  frame_interval_seconds=stats(np.diff(t)))
    # Wheel speed represents a trailing ~0.1 s interval. Compare against GT
    # averaged over that same interval, rather than instantaneous GT.
    averaged = np.full(len(t), np.nan)
    for i, ti in enumerate(t):
        lo = np.searchsorted(t, ti - .1)
        if i > lo:
            averaged[i] = np.trapz(g[lo:i+1, 0], t[lo:i+1]) / (ti - t[lo])
    result['window_matched_speed_error_mps'] = stats((w[:, 0] - averaged)[moving])
    # Ground-truth displacement divided by wall time independently checks whether
    # simulator velocity and bridge wall-clock timestamps use the same time base.
    dt = np.diff(t)
    distance = np.linalg.norm(np.diff(g[:, 1:3], axis=0), axis=1)
    velocity = .5 * (g[1:, 0] + g[:-1, 0])
    valid = (dt > .001) & (dt < .2) & (velocity > .5) & (distance < 1)
    result['gt_position_speed_over_reported_speed'] = stats((distance[valid] / dt[valid]) / velocity[valid])
    result['wheel_pose_step_over_gt_pose_step'] = stats(
        np.linalg.norm(np.diff(w[:, 1:3], axis=0), axis=1)[valid] / np.maximum(distance[valid], 1e-9))
    result['speed_error_by_frame_interval'] = {
        label: stats(error[1:][valid & mask]) for label, mask in
        [('under_20ms', dt < .02), ('20_to_50ms', (dt >= .02) & (dt < .05)),
         ('over_50ms', dt >= .05)]}
    print(json.dumps(result, indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    analyze(parser.parse_args().bag)
