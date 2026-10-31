#!/usr/bin/env python3
"""Regenerate an offline tire operating-region report from a finalized ROS 2 bag.

Example (inside the sourced ROS container):
  python3 scripts/tire_lap_report.py log/porto_map_clean \
    --track-dir tracks/porto_odom_trial --output log/porto_tires

These are kinematic slip estimates relative to documented peak-slip coordinates,
not measured tire forces or percentages of grip. No ROS nodes are started.
"""
import argparse
import base64
import csv
from dataclasses import asdict, dataclass
import html
import json
from pathlib import Path
import sqlite3

import numpy as np

NS = '/autodrive/roboracer_1/'
SOURCE = 'https://autodrive-ecosystem.github.io/competitions/roboracer-sim-racing-guide-2026/#132-vehicle-dynamics'
WHEELS = ('front_left', 'front_right', 'rear_left', 'rear_right')
METRICS = ('front_lateral_peak_ratio', 'rear_lateral_peak_ratio', 'rear_longitudinal_peak_ratio')


@dataclass
class Settings:
    wheelbase: float = .33
    track_width: float = .236
    wheel_radius: float = .0585
    longitudinal_peak: float = .15
    lateral_peak: float = .01
    velocity_reference_x: float = .15532
    max_steering_pair_age: float = .03
    minimum_speed: float = .5
    maximum_interval: float = .2
    frame_gap_warning: float = .12
    max_velocity_direction_residual: float = .02
    sustained_crossing_seconds: float = .3
    sustained_crossing_min_intervals: int = 3


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def wheel_lateral_slips(vx, vy, yaw_rate, steer, cfg):
    """Planar rigid-body wheel velocities; x forward, y left, positive steer left.

    Body velocity reference is (velocity_reference_x, 0) relative to rear axle.
    Steering is the reported virtual center-wheel angle, in radians.
    """
    result = {}
    for name in WHEELS:
        x = cfg.wheelbase if name.startswith('front') else 0.
        y = cfg.track_width/2 * (1 if name.endswith('left') else -1)
        delta = (np.arctan2(cfg.wheelbase*np.sin(steer),
                           cfg.wheelbase*np.cos(steer)-y*np.sin(steer))
                 if name.startswith('front') else 0.)
        local_x = vx-yaw_rate*y
        local_y = vy+yaw_rate*(x-cfg.velocity_reference_x)
        angle = float(wrap(np.arctan2(local_y, local_x)-delta))
        result[name] = (angle, float(np.tan(angle)), float(local_x))
    return result


def longitudinal_slips(forward_travel, dyaw, encoder_change, cfg):
    expected = np.array([forward_travel-cfg.track_width/2*dyaw,
                         forward_travel+cfg.track_width/2*dyaw])
    if np.min(expected) <= .01:
        return None
    return (np.asarray(encoder_change)*cfg.wheel_radius-expected)/expected


def read_bag(path):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    path = Path(path)
    if not (path/'metadata.yaml').exists():
        raise ValueError('Stop recording first; finalized metadata.yaml is required')
    wanted = {'/ground_truth/odom', NS+'left_encoder', NS+'right_encoder', NS+'steering', '/drive'}
    gt, enc, steering, commands = {}, {'left': {}, 'right': {}}, [], []
    for db in sorted(path.glob('*.db3')):
        with sqlite3.connect(db.resolve().as_uri()+'?mode=ro&immutable=1', uri=True) as conn:
            topics = {i: (name, get_message(kind)) for i, name, kind in
                      conn.execute('SELECT id,name,type FROM topics') if name in wanted}
            for topic, received, blob in conn.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp'):
                if topic not in topics:
                    continue
                name, kind = topics[topic]
                msg = deserialize_message(blob, kind)
                if name == NS+'steering':
                    steering.append((received, float(msg.data)))
                    continue
                stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
                if name == '/drive':
                    commands.append((stamp, float(msg.drive.speed)))
                elif name.endswith('_encoder'):
                    key = 'left' if name.endswith('left_encoder') else 'right'
                    if msg.position:
                        enc[key][stamp] = float(msg.position[0])
                else:
                    p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
                    yaw = np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                    gt[stamp] = (received, p.x, p.y, yaw, v.x, v.y, msg.twist.twist.angular.z)
    if len(gt) < 3:
        raise ValueError('Need at least three ground-truth samples')
    return gt, enc, sorted(steering), sorted(commands)


def nearest_pair(values, t):
    if not values:
        return np.nan, np.inf
    times = values[0]
    j = int(np.searchsorted(times, t))
    candidates = [i for i in (j-1, j) if 0 <= i < len(times)]
    j = min(candidates, key=lambda i: abs(int(times[i])-t))
    return float(values[1][j]), abs(int(times[j])-t)*1e-9


def build_rows(gt, enc, steering, commands, cfg):
    paired = (np.array([r[0] for r in steering], dtype=np.int64),
              np.array([r[1] for r in steering])) if steering else None
    command_times = np.array([r[0] for r in commands], dtype=np.int64)
    rows, segment = [], 0
    items = sorted(gt.items())
    for (ta, a), (tb, b) in zip(items, items[1:]):
        dt = (tb-ta)*1e-9
        dyaw = float(wrap(b[3]-a[3]))
        mid_yaw = a[3]+dyaw/2
        dp = np.array(b[1:3])-a[1:3]
        ds = float((dp[0]*np.cos(mid_yaw)+dp[1]*np.sin(mid_yaw))/np.sinc(dyaw/(2*np.pi)))
        distance = float(np.linalg.norm(dp))
        lateral_travel = float((-dp[0]*np.sin(mid_yaw)+dp[1]*np.cos(mid_yaw))/np.sinc(dyaw/(2*np.pi)))
        vx, vy, rate = (np.array(a[4:7])+b[4:7])/2
        finite = np.isfinite([*a[1:], *b[1:]]).all()
        interval_ok = bool(finite and .001 < dt <= cfg.maximum_interval and distance < .8 and abs(dyaw) < .3)
        geometry_ok = bool(finite and .001 < dt <= 1. and distance < .8 and abs(dyaw) < .3)
        if not geometry_ok:
            segment += 1
        moving = bool(interval_ok and vx > cfg.minimum_speed and ds > .015)
        sa, age_a = nearest_pair(paired, a[0])
        sb, age_b = nearest_pair(paired, b[0])
        pair_age = max(age_a, age_b)
        steer = .5*(sa+sb)
        lateral_ok = moving and pair_age <= cfg.max_steering_pair_age and np.isfinite(steer) and abs(steer) <= .6
        flags = []
        if not interval_ok: flags.append('interval_discontinuity')
        if not moving: flags.append('low_speed_or_invalid_motion')
        if dt > cfg.frame_gap_warning: flags.append('long_frame_interval')
        if pair_age > cfg.max_steering_pair_age: flags.append('steering_unpaired')
        row = dict(stamp_ns=tb, received_ns=int(b[0]), segment=segment, x_m=float(b[1]), y_m=float(b[2]),
                   yaw_rad=float(b[3]), distance_step_m=distance if geometry_ok else 0.,
                   dt_wall_s=dt, vx_mps=float(vx), vy_mps=float(vy), yaw_rate_rps=float(rate),
                   steering_rad=float(steer) if np.isfinite(steer) else None,
                   steering_pair_age_s=float(pair_age) if np.isfinite(pair_age) else None,
                   steering_rate_wall_dps=float(np.degrees(sb-sa)/dt)
                   if pair_age <= cfg.max_steering_pair_age and dt > 0 else None,
                   speed_command_mps=None, moving=moving, lateral_valid=False, longitudinal_valid=False,
                   front_lateral_peak_ratio=None, rear_lateral_peak_ratio=None,
                   rear_longitudinal_peak_ratio=None)
        row['velocity_reference_x_inferred_m'] = (
            float((vy-vx*lateral_travel/ds)/rate)
            if moving and abs(rate) > .4 else None)
        row['velocity_direction_residual_rad'] = (
            float(wrap(np.arctan2(vy-rate*cfg.velocity_reference_x, vx)-np.arctan2(lateral_travel, ds)))
            if moving else None)
        if moving and abs(row['velocity_direction_residual_rad']) > cfg.max_velocity_direction_residual:
            lateral_ok = False
            flags.append('velocity_reference_inconsistent')
        j = int(np.searchsorted(command_times, ta, side='right'))-1
        if j >= 0 and 0 <= (ta-command_times[j])*1e-9 < .3:
            row['speed_command_mps'] = commands[j][1]
        for name in WHEELS:
            row[name+'_alpha_rad'] = row[name+'_lateral_slip'] = None
        for name in ('rear_left', 'rear_right'):
            row[name+'_longitudinal_slip'] = None
        if lateral_ok:
            slips = wheel_lateral_slips(vx, vy, rate, steer, cfg)
            lateral_ok = all(v[2] > .2 and abs(v[0]) < np.pi/2 for v in slips.values())
            if lateral_ok:
                for name, (alpha, slip, _) in slips.items():
                    row[name+'_alpha_rad'], row[name+'_lateral_slip'] = alpha, slip
                for axle in ('front', 'rear'):
                    row[axle+'_lateral_peak_ratio'] = max(abs(slips[axle+'_'+side][1])
                                                           for side in ('left', 'right'))/cfg.lateral_peak
                row['lateral_valid'] = True
        have_encoders = all(ta in enc[side] and tb in enc[side] for side in ('left', 'right'))
        if moving and have_encoders:
            change = np.array([enc[side][tb]-enc[side][ta] for side in ('left', 'right')])
            if np.isfinite(change).all() and np.max(abs(change)*cfg.wheel_radius) < .8:
                slip = longitudinal_slips(ds, dyaw, change, cfg)
                if slip is not None:
                    row['rear_left_longitudinal_slip'], row['rear_right_longitudinal_slip'] = map(float, slip)
                    row['rear_longitudinal_peak_ratio'] = float(np.max(abs(slip))/cfg.longitudinal_peak)
                    row['longitudinal_valid'] = True
        if not have_encoders: flags.append('encoders_unpaired')
        if moving and not row['longitudinal_valid']: flags.append('longitudinal_unavailable')
        if moving and not row['lateral_valid']: flags.append('lateral_unavailable')
        row['flags'] = ';'.join(flags)
        rows.append(row)
    return rows


def track_progress(rows, path, origin):
    """Nearest closed-polyline projection; unwrap independently after discontinuities."""
    xy = path[:, :2]
    if len(xy) > 1 and np.linalg.norm(xy[-1]-xy[0]) < 1e-8:
        xy = xy[:-1]
    delta = np.roll(xy, -1, axis=0)-xy
    lengths = np.linalg.norm(delta, axis=1)
    if np.any(lengths < 1e-8):
        raise ValueError('Track contains duplicate adjacent points; remove duplicate closing point')
    starts = np.r_[0., np.cumsum(lengths[:-1])]
    total = float(lengths.sum())
    c, s = np.cos(origin['yaw_rad']), np.sin(origin['yaw_rad'])
    previous = unwrapped = None
    previous_source_segment = None
    report_segment = -1
    for row in rows:
        source_segment = row['segment']
        if not np.isfinite([row['x_m'], row['y_m']]).all():
            report_segment += 1
            row['segment'] = report_segment
            row['track_distance_m'] = row['progress_m'] = row['unwrapped_progress_m'] = None
            row['signed_tracking_error_m'] = row['heading_error_rad'] = None
            previous_source_segment = previous = None
            continue
        dx, dy = row['x_m']-origin['x_m'], row['y_m']-origin['y_m']
        point = np.array([c*dx+s*dy, -s*dx+c*dy])
        fractions = np.clip(np.sum((point-xy)*delta, axis=1)/lengths**2, 0, 1)
        projections = xy+fractions[:, None]*delta
        j = int(np.argmin(np.sum((point-projections)**2, axis=1)))
        progress = float(starts[j]+fractions[j]*lengths[j]) % total
        row['track_distance_m'] = float(np.linalg.norm(point-projections[j]))
        residual = point-projections[j]
        cross = delta[j, 0]*residual[1]-delta[j, 1]*residual[0]
        row['signed_tracking_error_m'] = float(np.copysign(row['track_distance_m'], cross))
        row['heading_error_rad'] = float(wrap(row['yaw_rad']-origin['yaw_rad']-
                                              np.arctan2(delta[j, 1], delta[j, 0])))
        if previous_source_segment != source_segment:
            report_segment += 1
            unwrapped = progress
        else:
            step = (progress-previous+total/2) % total-total/2
            # Localization/association jumps must not fabricate a complete lap.
            if abs(step) > max(1., 3*row['distance_step_m']):
                row['flags'] += ';track_progress_jump'
                unwrapped = progress
                report_segment += 1
            else:
                unwrapped += step
        row['progress_m'], row['unwrapped_progress_m'] = progress, float(unwrapped)
        row['segment'] = report_segment
        previous, previous_source_segment = progress, source_segment
    return total


def select_lap(rows, length, selection):
    complete = []
    for segment in dict.fromkeys(r['segment'] for r in rows):
        indexed = [(i, r) for i, r in enumerate(rows)
                   if r['segment'] == segment and r['unwrapped_progress_m'] is not None]
        if not indexed:
            continue
        moving = [(i, r) for i, r in indexed if r['moving']]
        if not moving:
            continue
        # Place the geometric start/finish at the first moving sample. A
        # complete circuit need not cross the raceline file's arbitrary s=0 twice.
        start_index, start_row = moving[0]
        next_boundary = start_row['unwrapped_progress_m']+length
        for i, row in indexed:
            if row['moving'] and row['unwrapped_progress_m'] >= next_boundary:
                complete.append((start_index, i))
                start_index = i
                next_boundary += length
    if selection == 'all':
        return rows, dict(selection='all', complete_laps_available=len(complete), full_lap=False)
    if not complete:
        if selection != 'auto':
            raise ValueError('No complete geometric lap available; use --lap all for a partial recording')
        return rows, dict(selection='partial_fallback', complete_laps_available=0, full_lap=False)
    lap = 1 if selection == 'auto' else int(selection)
    if not 1 <= lap <= len(complete):
        raise ValueError(f'Lap {lap} unavailable; {len(complete)} complete lap(s) found')
    a, b = complete[lap-1]
    return rows[a:b+1], dict(selection=lap, complete_laps_available=len(complete), full_lap=True)


def read_lap_counters(path):
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import Int32
    samples = []
    for db in sorted(Path(path).glob('*.db3')):
        with sqlite3.connect(db.resolve().as_uri()+'?mode=ro&immutable=1', uri=True) as conn:
            for t, blob in conn.execute('SELECT m.timestamp,m.data FROM messages m JOIN topics t ON m.topic_id=t.id WHERE t.name=? ORDER BY m.timestamp', (NS+'lap_count',)):
                samples.append((t, deserialize_message(blob, Int32).data))
    return sorted(samples)


def select_native_lap(rows, counter_samples, number):
    if number < 2 or not counter_samples:
        raise ValueError('Native flying-lap selection requires counter telemetry and an increment number >= 2')
    previous = counter_samples[0][1]
    crossings = []
    for stamp, count in counter_samples[1:]:
        if count == previous:
            continue
        if count != previous+1:
            raise ValueError('Native counter reset/skipped within bag; lap boundaries are ambiguous')
        crossings.append((stamp, count)); previous = count
    if len(crossings) < number:
        raise ValueError(f'Native increment {number} unavailable; {len(crossings)} crossings recorded')
    start, end = crossings[number-2], crossings[number-1]
    selected = [r for r in rows if start[0] <= r['received_ns'] < end[0]]
    if len(selected) < 3:
        raise ValueError('Too few GT samples inside native lap window')
    return selected, dict(selection=f'native:{number}', full_lap=True,
        boundary_source='native counter bag-receipt times; approximately one frame uncertainty',
        start_native_count=start[1], end_native_count=end[1],
        continuous_geometry=len({r['segment'] for r in selected}) == 1,
        complete_laps_available=max(0, len(crossings)-1))


def summarize(rows):
    summary = {}
    for metric in METRICS:
        rr = [r for r in rows if r[metric] is not None and np.isfinite(r[metric])]
        distance = sum(r['distance_step_m'] for r in rr)
        values = [r[metric] for r in rr]
        summary[metric] = dict(valid_intervals=len(rr), valid_distance_m=distance,
            median=float(np.median(values)) if values else None,
            p95=float(np.percentile(values, 95)) if values else None,
            maximum=float(max(values)) if values else None,
            near_or_beyond_peak_distance_pct=100*sum(r['distance_step_m'] for r in rr if r[metric] >= .8)/distance if distance else None,
            beyond_peak_distance_pct=100*sum(r['distance_step_m'] for r in rr if r[metric] >= 1)/distance if distance else None)
    return summary


def reference_diagnostics(rows):
    inferred = [r['velocity_reference_x_inferred_m'] for r in rows
                if r['velocity_reference_x_inferred_m'] is not None]
    residual = [abs(r['velocity_direction_residual_rad']) for r in rows
                if r['velocity_direction_residual_rad'] is not None]
    return dict(inferred_reference_samples=len(inferred),
                inferred_reference_x_median_m=float(np.median(inferred)) if inferred else None,
                direction_residual_median_abs_rad=float(np.median(residual)) if residual else None,
                direction_residual_p95_abs_rad=float(np.percentile(residual, 95)) if residual else None)


def tracking_summary(rows):
    valid = [r for r in rows if r.get('track_distance_m') is not None and
             np.isfinite(r['track_distance_m']) and r['distance_step_m'] > 0]
    if not valid:
        return dict(valid_intervals=0, distance_weighted_rms_m=None,
                    p95_abs_m=None, maximum_abs_m=None, mean_signed_m=None,
                    p95_abs_heading_deg=None)
    weights = np.array([r['distance_step_m'] for r in valid])
    error = np.array([r['track_distance_m'] for r in valid])
    return dict(valid_intervals=len(valid),
        valid_distance_m=float(weights.sum()),
        distance_weighted_rms_m=float(np.sqrt(np.average(error**2, weights=weights))),
        p95_abs_m=float(np.percentile(error, 95)), maximum_abs_m=float(np.max(error)),
        mean_signed_m=float(np.average([r['signed_tracking_error_m'] for r in valid], weights=weights)),
        p95_abs_heading_deg=float(np.percentile(np.abs(np.degrees([r['heading_error_rad'] for r in valid])), 95)))


def crossing_events(rows, cfg):
    """Classify sampled threshold runs, without bridging missing data or frame gaps.

    Durations use wall time, not verified physics time. Classification is a
    persistence heuristic and cannot establish sustained physical tire saturation.
    """
    events = []
    for metric in METRICS:
        key = metric.replace('_peak_ratio', '_crossing')
        run = []

        def finish():
            if not run:
                return
            duration = sum(rows[i]['dt_wall_s'] for i in run)
            uncertain = any(rows[i]['dt_wall_s'] > cfg.frame_gap_warning for i in run)
            kind = ('timing_uncertain' if uncertain else 'sustained'
                    if duration >= cfg.sustained_crossing_seconds and
                    len(run) >= cfg.sustained_crossing_min_intervals else 'brief')
            peak = max(run, key=lambda i: rows[i][metric])
            rates = [abs(rows[i]['steering_rate_wall_dps']) for i in run
                     if rows[i].get('steering_rate_wall_dps') is not None]
            for i in run:
                rows[i][key] = kind
            events.append(dict(metric=metric, classification=kind, intervals=len(run),
                duration_wall_s=duration, peak_ratio=rows[peak][metric],
                start_stamp_ns=rows[run[0]]['stamp_ns']-round(rows[run[0]]['dt_wall_s']*1e9),
                end_stamp_ns=rows[run[-1]]['stamp_ns'],
                start_index=run[0], end_index=run[-1], peak_index=peak,
                max_abs_steering_rate_wall_dps=max(rates) if rates else None))
            run.clear()

        for i, row in enumerate(rows):
            value = row[metric]
            row[key] = 'unavailable' if value is None else 'below_peak'
            if run and (row['segment'] != rows[run[-1]]['segment'] or
                        row['dt_wall_s'] > cfg.frame_gap_warning):
                finish()
            if value is not None and np.isfinite(value) and value >= 1:
                run.append(i)
                if row['dt_wall_s'] > cfg.frame_gap_warning:
                    finish()
            else:
                finish()
        finish()
    return events


def render(rows, output, report):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import BoundaryNorm, ListedColormap
    distance = np.cumsum([r['distance_step_m'] for r in rows])
    labels = ('Front lateral: max left/right', 'Rear lateral: max left/right', 'Rear longitudinal: max left/right')
    fig, axes = plt.subplots(9, 1, figsize=(12, 20), sharex=True)
    for ax, metric, label in zip(axes, METRICS, labels):
        values = [r[metric] if r[metric] is not None else np.nan for r in rows]
        ax.plot(distance, values, linewidth=1, label=label)
        ax.axhline(1, color='red', linestyle='--', label='Documented peak-slip coordinate')
        ax.axhline(.8, color='orange', linestyle=':', label='0.8 reporting heuristic')
        ax.set_ylabel('|slip| / peak slip'); ax.legend(fontsize=8)
        seen = set()
        for event in report['crossing_events']:
            if event['metric'] != metric:
                continue
            kind = event['classification']
            color = {'brief': '#f28e2b', 'sustained': '#c62828', 'timing_uncertain': '#777777'}[kind]
            lo, hi = event['start_index'], event['end_index']
            ax.axvspan(max(0., distance[lo]-rows[lo]['distance_step_m']), distance[hi],
                       color=color, alpha=.18)
            ax.scatter([distance[event['peak_index']]], [event['peak_ratio']], color=color,
                       s=25, zorder=3, label=kind.replace('_', ' ')+' crossing' if kind not in seen else None)
            seen.add(kind)
        ax.legend(fontsize=8)
    numeric = lambda key: [r[key] if r[key] is not None else np.nan for r in rows]
    axes[3].plot(distance, np.degrees(numeric('steering_rad')), label='Steering feedback (degrees)')
    axes[3].axhline(0, color='.6', linewidth=.7); axes[3].legend(fontsize=8)
    axes[4].plot(distance, numeric('steering_rate_wall_dps'), label='Steering feedback rate (degrees / wall second)')
    axes[4].axhline(0, color='.6', linewidth=.7); axes[4].legend(fontsize=8)
    axes[5].plot(distance, numeric('signed_tracking_error_m'), label='Signed distance to raceline (m; left positive)')
    axes[5].axhline(0, color='.6', linewidth=.7)
    axes[5].legend(fontsize=8)
    axes[6].plot(distance, np.degrees(numeric('heading_error_rad')), label='Heading relative to raceline tangent (degrees)')
    axes[6].axhline(0, color='.6', linewidth=.7); axes[6].legend(fontsize=8)
    axes[7].plot(distance, [r['vx_mps'] for r in rows], label='Reported forward speed (m/s)')
    axes[7].legend(fontsize=8)
    axes[8].plot(distance, [1000*r['dt_wall_s'] for r in rows], label='ROS-stamp interval (ms)')
    axes[8].axhline(report['settings']['frame_gap_warning']*1000, linestyle=':', color='orange', label='Long-interval warning')
    axes[8].legend(fontsize=8); axes[8].set_xlabel('Distance traveled within selection (m)')
    for ax in axes: ax.grid(alpha=.25)
    fig.suptitle('Estimated tire operating region — not force utilization')
    fig.tight_layout(rect=(0, 0, 1, .97)); fig.savefig(output/'lap_profiles.png', dpi=150); plt.close(fig)
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    # A discontinuous palette makes crossing the documented peak unmistakable.
    boundaries = [0., .25, .5, .75, 1., 1.25, 1.5, 1.75, 2.]
    colors = ListedColormap(['#e0f3ff', '#9acbed', '#4a98cf', '#155a96',
                             '#ffb347', '#f47926', '#d83a25', '#920d20'])
    norm = BoundaryNorm(boundaries, colors.N, clip=True)
    for ax, metric, label in zip(axes, METRICS, labels):
        ax.plot([r['x_m'] for r in rows], [r['y_m'] for r in rows], color='.75', linewidth=3, zorder=1)
        rr = [r for r in rows if r[metric] is not None]
        points = ax.scatter([r['x_m'] for r in rr], [r['y_m'] for r in rr],
                            c=[r[metric] for r in rr], norm=norm, cmap=colors, s=16, zorder=2)
        bar = fig.colorbar(points, ax=ax, ticks=[0, .5, 1, 1.5, 2],
                           label='|slip| / peak (colors capped at 2)')
        bar.ax.set_yticklabels(['0', '0.5', '1 — peak', '1.5', '2+'])
        bar.ax.axhline(1., color='black', linewidth=2)
        ax.set_title(label); ax.set_aspect('equal'); ax.set_xlabel('World x (m)'); ax.set_ylabel('World y (m)')
    fig.tight_layout(); fig.savefig(output/'track_slip.png', dpi=150); plt.close(fig)
    lines = ['# Estimated tire operating region', '',
             '**These are kinematic slip estimates, not tire forces or percentages of available grip.**', '',
             f"Selection: {report['lap']}. Distance: {distance[-1]:.2f} m.", '',
             '| Estimate | Valid intervals | Median ratio | P95 ratio | Distance at/above peak |',
             '|---|---:|---:|---:|---:|']
    def fmt(value): return 'unavailable' if value is None else f'{value:.3f}'
    tracking = report['tracking_error']
    tracking_text = (f"Raceline tracking: distance-weighted RMS {fmt(tracking['distance_weighted_rms_m'])} m; "
                     f"P95 absolute {fmt(tracking['p95_abs_m'])} m; maximum {fmt(tracking['maximum_abs_m'])} m; "
                     f"P95 absolute heading error {fmt(tracking['p95_abs_heading_deg'])} degrees.")
    for metric, label in zip(METRICS, labels):
        v = report['metrics'][metric]
        lines.append(f"| {label} | {v['valid_intervals']} | {fmt(v['median'])} | {fmt(v['p95'])} | {fmt(v['beyond_peak_distance_pct'])}% |")
    lines += ['', tracking_text, '']
    event_description = (f"Sustained means at least {report['settings']['sustained_crossing_seconds']} wall seconds "
        f"and {report['settings']['sustained_crossing_min_intervals']} consecutive valid intervals at/above 1. "
        'Missing data, segment changes, and long frame intervals break runs. '
        'Long-interval crossings are timing-uncertain. This is a persistence heuristic, not proof of saturation.')
    lines += ['', '## Threshold-crossing events', '', event_description, '',
              '| Metric | Classification | Intervals | Wall duration (ms) | Peak ratio | Max steering rate (deg/wall s) |',
              '|---|---|---:|---:|---:|---:|']
    event_table = '<h2>Threshold-crossing events</h2><p>'+html.escape(event_description)+'</p>'
    event_table += '<table><tr><th>Metric</th><th>Classification</th><th>Intervals</th><th>Wall duration (ms)</th><th>Peak ratio</th><th>Max steering rate (deg/wall s)</th></tr>'
    for event in report['crossing_events']:
        name = event['metric'].replace('_peak_ratio', '').replace('_', ' ')
        cells = [name, event['classification'], str(event['intervals']),
                 fmt(event['duration_wall_s']*1000), fmt(event['peak_ratio']),
                 fmt(event['max_abs_steering_rate_wall_dps'])]
        lines.append('| '+' | '.join(cells)+' |')
        event_table += '<tr>'+''.join('<td>'+html.escape(c)+'</td>' for c in cells)+'</tr>'
    if not report['crossing_events']:
        lines.append('No threshold crossings in the selected data.')
        event_table += '<tr><td colspan="6">No threshold crossings in the selected data.</td></tr>'
    event_table += '</table>'
    lines += ['', '## Assumptions and limitations', '']+[f'- {w}' for w in report['warnings']]
    lines += ['', '## Velocity reference consistency', '',
              json.dumps(report['velocity_reference_diagnostics'], indent=2)]
    lines += ['', '## Regenerate', '', '```bash', report['regenerate_command'], '```', '',
              f'[Documented threshold source]({SOURCE})', '',
              '![Lap profiles](lap_profiles.png)', '', '![Track slip](track_slip.png)']
    markdown = '\n'.join(lines)+'\n'
    (output/'report.md').write_text(markdown)
    images = ''.join('<h2>'+title+'</h2><img style="max-width:100%" src="data:image/png;base64,'+
                     base64.b64encode((output/file).read_bytes()).decode()+'">'
                     for title, file in [('Lap profiles', 'lap_profiles.png'), ('Track maps', 'track_slip.png')])
    table = '<table><tr><th>Estimate</th><th>Valid intervals</th><th>Median ratio</th><th>P95 ratio</th><th>Distance at/above peak</th></tr>'
    for metric, label in zip(METRICS, labels):
        v = report['metrics'][metric]
        table += f"<tr><td>{label}</td><td>{v['valid_intervals']}</td><td>{fmt(v['median'])}</td><td>{fmt(v['p95'])}</td><td>{fmt(v['beyond_peak_distance_pct'])}%</td></tr>"
    table += '</table>'
    (output/'report.html').write_text('<!doctype html><meta charset="utf-8"><title>Tire operating region</title>'
        '<style>body{font-family:system-ui;max-width:1400px;margin:30px auto;padding:15px;color:#18232d}'
        'table{border-collapse:collapse}th,td{padding:10px;border-bottom:1px solid #ddd;text-align:left}'
        'pre{white-space:pre-wrap;background:#f2f4f5;padding:15px}li{margin:8px 0}</style>'
        '<h1>Estimated tire operating region</h1>'
        '<p><strong>Kinematic slip estimates, not tire forces or percentages of available grip.</strong></p>'
        f"<p>Selection: {html.escape(str(report['lap']))}. Distance: {distance[-1]:.2f} m.</p>"+
        '<p>'+html.escape(tracking_text)+'</p>'+table+event_table+images+'<h2>Assumptions and limitations</h2><ul>'+
        ''.join('<li>'+html.escape(w)+'</li>' for w in report['warnings'])+'</ul>'+
        '<h2>Velocity reference consistency</h2><pre>'+html.escape(json.dumps(report['velocity_reference_diagnostics'], indent=2))+
        '</pre><h2>Regenerate</h2><pre>'+html.escape(report['regenerate_command'])+'</pre>'+
        '<p><a href="'+SOURCE+'">Documented threshold source</a></p></body>')


def main():
    import shlex
    import sys
    import yaml
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bag')
    ap.add_argument('--track-dir', required=True, help='contains raceline.csv and map_to_world.yaml')
    ap.add_argument('--output', required=True, help='directory for regenerated report artifacts')
    ap.add_argument('--lap', default='auto', help='auto/integer: geometric circuit; native:2: second native increment (flying lap); all: entire recording')
    ap.add_argument('--settings', help='JSON overrides for Settings; saved in the report')
    args = ap.parse_args()
    native = args.lap.startswith('native:')
    if native:
        if not args.lap[7:].isdigit() or int(args.lap[7:]) < 2:
            ap.error('native flying lap number must be >= 2')
    elif args.lap not in ('auto', 'all') and (not args.lap.isdigit() or int(args.lap) < 1):
        ap.error('--lap must be auto, all, native:N, or a positive integer')
    cfg = Settings(**json.loads(Path(args.settings).read_text())) if args.settings else Settings()
    for key, value in asdict(cfg).items():
        if not np.isfinite(value) or (key != 'velocity_reference_x' and value <= 0):
            ap.error(f'invalid setting: {key}')
    if int(cfg.sustained_crossing_min_intervals) != cfg.sustained_crossing_min_intervals:
        ap.error('sustained_crossing_min_intervals must be an integer')
    rows = build_rows(*read_bag(args.bag), cfg)
    track_dir = Path(args.track_dir)
    path = np.loadtxt(track_dir/'raceline.csv', delimiter=';', comments='#', ndmin=2)
    if len(path) < 3: ap.error('raceline needs three or more points')
    with open(track_dir/'map_to_world.yaml') as stream: origin = yaml.safe_load(stream)
    # Raceline columns are s, x, y, heading, curvature, speed, acceleration.
    if path.shape[1] < 3: ap.error('raceline needs s, x, y columns')
    length = track_progress(rows, path[:, 1:3], origin)
    selected, lap = (select_native_lap(rows, read_lap_counters(args.bag), int(args.lap[7:]))
                     if native else select_lap(rows, length, args.lap))
    if not selected: ap.error('No report intervals')
    output = Path(args.output); output.mkdir(parents=True, exist_ok=True)
    warnings = [
        'Peak-slip thresholds are documented defaults, not verified against the installed simulator binary.',
        'Normalized slip is proximity to a peak-slip coordinate, not force utilization; 0.8 is a reporting heuristic.',
        'Wheel geometry, steering sign/scale and velocity reference point are assumptions saved in settings.json.',
        'Lateral estimates use planar rigid-body motion and inferred Ackermann wheel angles; tire forces, loads and combined-slip coupling are unobserved.',
        'Rear longitudinal slip uses per-wheel encoder travel versus pose-projected travel, corrected for turning; no wall-time differentiation.',
        'Front longitudinal slip is unavailable: the stream has only rear encoders.',
        'Steering Float32 has no source timestamp: nearest bag-receipt pairing is approximate, with a maximum age of '+str(cfg.max_steering_pair_age)+' s.',
        'Missing/low-speed/discontinuous data remain unavailable, never zero; plots break at unavailable estimates.',
        'GT body velocity is assumed referenced at x='+str(cfg.velocity_reference_x)+' m from the rear axle (documented center of mass by default); compare the inferred reference and direction residual in this report.',
        'Lateral estimates are unavailable when reported velocity transformed to the rear axle disagrees with displacement direction by more than '+str(cfg.max_velocity_direction_residual)+' rad.',
        'Geometric lap selection uses the supplied raceline and map/world transform, not simulator lap counters.',
        'Intervals are averages; frame drops can hide brief slip peaks. Whole-recording selection includes startup and stopping.',
        'Tracking error is ground-truth rear-axle position versus the mapped raceline, not controller lookahead error; map/world alignment and localization error can affect the result.',
    ]
    if not lap['full_lap']: warnings.append('This selection is not a verified complete lap; see lap selection metadata.')
    if native:
        warnings.append('Native counter boundaries identify a full lap, not complete sensor coverage. Counter messages have no source stamp; boundaries are approximate to a frame.')
        if not lap['continuous_geometry']:
            warnings.append('Native lap contains discontinuities: tracking/slip coverage is incomplete; inspect quality flags and valid distances.')
    if any(r['front_lateral_peak_ratio'] is not None and r['front_lateral_peak_ratio'] > 2 for r in selected):
        warnings.append('Some front estimates exceed twice the documented peak coordinate: verify steering pairing, model definitions and installed-build parameters before interpreting this as tire saturation.')
    if max((r['track_distance_m'] for r in selected if r['track_distance_m'] is not None), default=0) > 1.:
        warnings.append('Some samples are over 1 m from the supplied raceline: verify track/frame alignment.')
    warnings.append('Crossing durations and steering rates use wall time, not verified physics time. Brief or sustained classifications do not establish physical tire saturation.')
    events = crossing_events(selected, cfg)
    report = dict(bag=str(Path(args.bag).resolve()), track_dir=str(track_dir.resolve()),
                  lap=lap, track_length_m=length, selected_intervals=len(selected),
                  selected_distance_m=sum(r['distance_step_m'] for r in selected),
                  long_frame_intervals=sum('long_frame_interval' in r['flags'] for r in selected),
                  start_stamp_ns=selected[0]['stamp_ns'], end_stamp_ns=selected[-1]['stamp_ns'],
                  settings=asdict(cfg), threshold_source=SOURCE, warnings=warnings,
                  quality_flag_counts={flag: sum(flag in r['flags'].split(';') for r in selected)
                                       for flag in sorted({f for r in selected for f in r['flags'].split(';') if f})},
                  velocity_reference_diagnostics=reference_diagnostics(selected),
                  tracking_error=tracking_summary(selected),
                  crossing_events=events,
                  crossing_counts={metric: {kind: sum(e['metric'] == metric and e['classification'] == kind for e in events)
                                            for kind in ('brief', 'sustained', 'timing_uncertain')} for metric in METRICS},
                  metrics=summarize(selected), regenerate_command=shlex.join(['python3', *sys.argv]))
    with open(output/'samples.csv', 'w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(selected[0])); writer.writeheader(); writer.writerows(selected)
    with open(output/'events.csv', 'w', newline='') as stream:
        fields = ['metric', 'classification', 'intervals', 'duration_wall_s', 'peak_ratio',
                  'start_stamp_ns', 'end_stamp_ns', 'start_index', 'end_index', 'peak_index',
                  'max_abs_steering_rate_wall_dps']
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows(events)
    (output/'settings.json').write_text(json.dumps(asdict(cfg), indent=2)+'\n')
    (output/'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    render(selected, output, report)
    print(json.dumps(dict(output=str(output), lap=lap, metrics=report['metrics']), indent=2))


if __name__ == '__main__':
    main()
