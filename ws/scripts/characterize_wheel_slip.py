#!/usr/bin/env python3
"""Offline encoder-travel slip proxy, independent of elapsed wall time.

Requires a finalized ROS bag with ground truth and both rear encoders. Positive
excess means wheel over-rotation; negative means under-rotation. Individual wheel
values include differential turning unless --rear-track-width is provided.
"""
import argparse
import csv
import json
from pathlib import Path
import sqlite3

import numpy as np

NS = '/autodrive/roboracer_1/'


def load_bag(path):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    path = Path(path)
    if not (path / 'metadata.yaml').exists():
        raise ValueError('Stop recording first: finalized metadata.yaml is required')
    wanted = {'/ground_truth/odom', NS + 'left_encoder', NS + 'right_encoder', '/drive'}
    data = {name: {} for name in wanted}
    for db in sorted(path.glob('*.db3')):
        with sqlite3.connect(db.resolve().as_uri() + '?mode=ro&immutable=1', uri=True) as conn:
            topics = {i: (name, get_message(kind)) for i, name, kind in
                      conn.execute('SELECT id,name,type FROM topics') if name in wanted}
            for topic, blob in conn.execute('SELECT topic_id,data FROM messages ORDER BY timestamp'):
                if topic not in topics:
                    continue
                name, kind = topics[topic]
                msg = deserialize_message(blob, kind)
                stamp = msg.header.stamp
                t = stamp.sec * 1000000000 + stamp.nanosec
                if name == '/ground_truth/odom':
                    p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
                    yaw = np.arctan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
                    values = (p.x, p.y, yaw, v.x, v.y, msg.twist.twist.angular.z)
                elif name == '/drive':
                    values = (msg.drive.speed,)
                else:
                    values = (msg.position[0],)
                data[name][t] = values
    times = sorted(set.intersection(*(set(d) for name, d in data.items() if name != '/drive')))
    if len(times) < 3:
        raise ValueError('Need three or more exact-stamp GT/left/right triples')
    gt = np.array([data['/ground_truth/odom'][t] for t in times])
    angles = np.array([[data[NS + wheel + '_encoder'][t][0] for wheel in ('left', 'right')]
                       for t in times])
    commands = sorted(data['/drive'].items())
    return np.array(times, dtype=np.int64), gt, angles, commands


def intervals(stamps, gt, angles, commands=None, radius=.0585, rear_track_width=None):
    """GT columns: x,y,yaw,vx,vy,yaw_rate; encoder columns: left,right radians."""
    dt = np.diff(stamps) * 1e-9
    dyaw = np.arctan2(np.sin(np.diff(gt[:, 2])), np.cos(np.diff(gt[:, 2])))
    heading = gt[:-1, 2] + .5 * dyaw
    dp = np.diff(gt[:, :2], axis=0)
    # Convert midpoint-heading chord projection to forward arc length. This is
    # exact for constant-curvature planar motion with constant body velocity.
    arc = np.sinc(dyaw / (2*np.pi))
    forward = (dp[:, 0]*np.cos(heading) + dp[:, 1]*np.sin(heading)) / arc
    lateral = (-dp[:, 0]*np.sin(heading) + dp[:, 1]*np.cos(heading)) / arc
    wheel = np.diff(angles, axis=0) * radius
    expected = np.column_stack((forward, forward))
    if rear_track_width is not None:
        expected[:, 0] -= rear_track_width * .5 * dyaw
        expected[:, 1] += rear_track_width * .5 * dyaw
    speed = .5 * (gt[:-1, 3] + gt[1:, 3])
    # Energy slope estimates physical longitudinal acceleration without using
    # wall-clock dt. Turning/slip can also affect it; this is a grouping proxy.
    ax = np.divide(np.diff(gt[:, 3]**2), 2*forward,
                   out=np.full(len(dt), np.nan), where=forward > .01)
    valid = ((dt > .001) & (dt < .2) & (speed > .5) &
             (forward > .02) & (np.linalg.norm(dp, axis=1) < .8) &
             (np.abs(dyaw) < .3) & (np.max(abs(wheel), axis=1) < .8) &
             (np.min(expected, axis=1) > .01) & np.isfinite(ax))
    yaw_rate = .5 * (gt[:-1, 5] + gt[1:, 5])
    corner = abs(yaw_rate) >= .3
    steady = abs(ax) < .5
    groups = {'all': valid, 'accelerating': valid & (ax >= .5),
              'braking': valid & (ax <= -.5), 'steady': valid & steady,
              'straight': valid & ~corner, 'cornering': valid & corner,
              'steady_straight': valid & steady & ~corner}
    commanded = np.full(len(dt), np.nan)
    if commands:
        ct = np.array([item[0] for item in commands], dtype=np.int64)
        cv = np.array([item[1][0] for item in commands])
        # Command must already be active at the start of the interval.
        ci = np.searchsorted(ct, stamps[:-1], side='right') - 1
        age = stamps[:-1]-ct[np.maximum(ci, 0)]
        # A final zero command remains a stop after the controller exits. Keep
        # it for up to 2 s; nonzero commands must be less than 0.3 s old.
        latest_zero = cv[np.maximum(ci, 0)] <= .1
        fresh = (ci >= 0) & (age < np.where(latest_zero, 2_000_000_000, 300_000_000))
        commanded[fresh] = cv[ci[fresh]]
        driving = valid & (commanded > .1)
        stopping = valid & (commanded <= .1)
        groups.update(driving=driving, commanded_stop=stopping,
                      command_unavailable=valid & ~np.isfinite(commanded),
                      driving_accelerating=driving & (ax >= .5),
                      driving_braking=driving & (ax <= -.5))
    mean_wheel = np.mean(wheel, axis=1)
    excess = np.divide(mean_wheel-forward, forward,
                       out=np.full(len(dt), np.nan), where=forward != 0)
    individual = np.divide(wheel-expected, expected,
                           out=np.full_like(wheel, np.nan), where=expected != 0)
    report = {}
    for name, mask in groups.items():
        if not mask.any():
            report[name] = {'intervals': 0}
            continue
        e = excess[mask]
        report[name] = dict(intervals=int(mask.sum()), gt_forward_m=float(forward[mask].sum()),
            distance_weighted_excess_pct=float(100*(mean_wheel[mask].sum()/forward[mask].sum()-1)),
            median_excess_pct=float(100*np.median(e)),
            p05_excess_pct=float(100*np.percentile(e, 5)),
            p95_excess_pct=float(100*np.percentile(e, 95)),
            left_median_excess_pct=float(100*np.median(individual[mask, 0])),
            right_median_excess_pct=float(100*np.median(individual[mask, 1])))
        frozen = mask & np.all(abs(wheel) < 1e-8, axis=1)
        report[name]['both_encoders_unchanged_intervals'] = int(frozen.sum())
        report[name]['gt_travel_with_both_encoders_unchanged_m'] = float(forward[frozen].sum())
    rows = [dict(stamp_ns=int(stamps[i+1]), dt_wall_s=dt[i], gt_forward_m=forward[i],
                 gt_lateral_m=lateral[i], left_travel_m=wheel[i, 0], right_travel_m=wheel[i, 1],
                 speed_mps=speed[i], acceleration_proxy_mps2=ax[i], yaw_rate=yaw_rate[i],
                 speed_command_mps=commanded[i],
                 mean_excess_pct=100*excess[i], left_excess_pct=100*individual[i, 0],
                 right_excess_pct=100*individual[i, 1], cornering=bool(corner[i]))
            for i in np.flatnonzero(valid)]
    return dict(exact_stamp_triples=len(stamps), rejected_intervals=int((~valid).sum()),
                wheel_radius_m=radius, rear_track_width_m=rear_track_width,
                individual_wheels_turn_corrected=rear_track_width is not None, groups=report), rows


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--wheel-radius', type=float, default=.0585)
    parser.add_argument('--rear-track-width', type=float)
    parser.add_argument('--output', required=True, help='output stem for JSON and interval CSV')
    args = parser.parse_args()
    if args.wheel_radius <= 0 or (args.rear_track_width is not None and args.rear_track_width <= 0):
        parser.error('radius and track width must be positive')
    result, rows = intervals(*load_bag(args.bag), radius=args.wheel_radius,
                            rear_track_width=args.rear_track_width)
    result['bag'] = args.bag
    Path(args.output + '.json').write_text(json.dumps(result, indent=2) + '\n')
    if rows:
        with open(args.output + '.csv', 'w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    print(json.dumps(result, indent=2))
