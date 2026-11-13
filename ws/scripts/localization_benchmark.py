#!/usr/bin/env python3
"""Localization benchmark: race each localizer on a mapped track and score its pose against the sim's ground truth.

    python3 scripts/localization_benchmark.py run --track porto [--localizers cartographer,mcl] [--duration 60]
        [--speed-scale 0.5] [--repeats 1] [--controller pure_pursuit]
    python3 scripts/localization_benchmark.py analyze log/localization_benchmark/<time>/   (re-score saved runs)

Run inside the container (sourced workspace), with the bridge running and the simulator connected in
Autonomous mode on the same track (this script cannot switch the sim's track). Each run is a normal
`scripts/race.sh <track> localizer:=<name>` for --duration s, stopped with one SIGINT to the launch.

Recorded per run (log/localization_benchmark/<UTC time>/<localizer>_<n>.csv): /localized_pose,
/ground_truth/pose (sim world frame, rear axle = base_link), the sim's lap and collision counts, and /scan
arrival times (the sim's sensor rate: a slow sim degrades every localizer).

Scores per run (summary.txt, summary.json):
  error     the localized pose against ground truth, put in the map frame with tracks/<track>/map_to_world.yaml
            (the car's true pose when mapping started) and interpolated to each localized pose's stamp (also
            across a sim stall, where the car stood still: a localizer extrapolating meanwhile is scored):
            position mean / median / p95 / max [m], heading mean / p95 [deg], share of time over 0.2 m
  aligned   position error after the best rigid fit of the run's localized track onto the true one: what is
            left once a rotated or shifted map is taken out (error - aligned ~ the map's own placement error)
  jitter    RMS change of the error over 50 ms [m]: how much the estimate wobbles around the truth, what
            pure pursuit steers on (a constant offset scores 0). Cartographer's /tracked_pose: 20-110 mm
            on Porto, and the car clipped corners on it; MCL: 6-20 mm
  jumps     steps of the localized pose faster than 15 m/s (and over 0.3 m): the localizer snapping (e.g.
            Cartographer's pose extrapolation running on while the sim stalls, then snapping back)
  stalls    gaps over 0.1 s between the sim's ground-truth frames: the sim froze (a loaded Mac)
  gaps      the pose missing over 0.25 s while it had been published (e.g. withheld after a crash), total s
  laps, collisions, scan_hz

Ground truth is only used here, for scoring; the localizers never see it.
"""
import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

NS = '/autodrive/roboracer_1/'
JUMP_SPEED, JUMP_MIN = 15.0, 0.3       # m/s, m: a localized step faster than this is a jump
GAP = 0.25                             # s without a localized pose
MAX_PAIR_DT = 0.1                      # s: ground truth on both sides of a stamp, at most this far apart ...
STILL, MAX_STILL_DT = 0.1, 2.0         # m, s: ... or a longer gap (a sim stall) the car barely moved across
TELEPORT = 0.5                         # m between two ground-truth poses: a reset, never interpolated across
FAR = 0.2                              # m: "share of time over"
JITTER_DT = 0.05                       # s: the error's change over this long is the jitter


def wrap(a):
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ----------------------------------------------------------------------------------------------- scoring

def to_map(xyyaw, origin):
    """World-frame poses (N x 3) into the map frame, origin = the map origin's world pose (x, y, yaw)."""
    p = np.asarray(xyyaw, dtype=float).reshape(-1, 3)
    ox, oy, oyaw = origin
    c, s = math.cos(oyaw), math.sin(oyaw)
    dx, dy = p[:, 0] - ox, p[:, 1] - oy
    return np.column_stack([c * dx + s * dy, -s * dx + c * dy, wrap(p[:, 2] - oyaw)])


def interpolate(gt_t, gt, t):
    """Ground truth (sorted stamps gt_t, poses gt N x 3) at stamps t: (poses M x 3, valid mask). A stamp is
    valid with ground truth on both sides within MAX_PAIR_DT, or across a gap of up to MAX_STILL_DT over which
    the car moved less than STILL (the sim stalled: the car stood, and a localizer that extrapolated meanwhile
    is scored against where it stood); never across a teleport."""
    gt_t, gt, t = np.asarray(gt_t), np.asarray(gt), np.asarray(t)
    out = np.full((len(t), 3), np.nan)
    if len(gt_t) < 2:
        return out, np.zeros(len(t), bool)
    i = np.clip(np.searchsorted(gt_t, t), 1, len(gt_t) - 1)
    t0, t1, p0, p1 = gt_t[i - 1], gt_t[i], gt[i - 1], gt[i]
    moved = np.hypot(*(p1[:, :2] - p0[:, :2]).T)
    ok = (t >= t0) & (t <= t1) & (((t1 - t0 <= MAX_PAIR_DT) & (moved < TELEPORT))
                                  | ((t1 - t0 <= MAX_STILL_DT) & (moved < STILL)))
    f = np.where(t1 > t0, (t - t0) / np.where(t1 > t0, t1 - t0, 1.0), 0.0)
    out[:, :2] = p0[:, :2] + f[:, None] * (p1[:, :2] - p0[:, :2])
    out[:, 2] = wrap(p0[:, 2] + f * wrap(p1[:, 2] - p0[:, 2]))
    out[~ok] = np.nan
    return out, ok


def rigid_fit(src, dst):
    """Rotation R and translation t minimising |R src + t - dst| (2D points, Kabsch)."""
    ms, md = src.mean(axis=0), dst.mean(axis=0)
    u, _, vt = np.linalg.svd((src - ms).T @ (dst - md))
    d = np.sign(np.linalg.det(vt.T @ u.T))
    r = vt.T @ np.diag([1.0, d]) @ u.T
    return r, md - r @ ms


def jumps_and_gaps(t, xy):
    """(jump sizes, gap durations) of a localized pose stream."""
    t, xy = np.asarray(t), np.asarray(xy)
    if len(t) < 2:
        return [], []
    dt, step = np.diff(t), np.hypot(*np.diff(xy, axis=0).T)
    jumps = step[(step > JUMP_MIN) & (step > JUMP_SPEED * dt)]
    return [float(j) for j in jumps], [float(g) for g in dt[dt > GAP]]


def jitter(t, err, ok):
    """RMS of the change of the error vector over JITTER_DT (pairs ok at both ends, at most 1.4 x apart)."""
    t = np.asarray(t)
    j = np.searchsorted(t, t + JITTER_DT)
    v = ok & (j < len(t))
    v[v] &= ok[j[v]] & (t[j[v]] - t[v] < 1.4 * JITTER_DT)
    if v.sum() < 3:
        return None
    d = np.linalg.norm(err[j[v]] - err[v], axis=1)
    return float(np.sqrt((d ** 2).mean()))


def counter_delta(values):
    """Increase of a sim counter over a run, ignoring drops (a sim reset zeroes some counters)."""
    v = np.asarray(values, dtype=float)
    return int(np.clip(np.diff(v), 0, None).sum()) if len(v) > 1 else 0


def score(rows, origin):
    """Scores of one run from its recorded rows (dicts: kind, stamp, recv, a, b, c)."""
    by = {}
    for r in rows:
        by.setdefault(r['kind'], []).append(r)
    loc = sorted(by.get('loc', []), key=lambda r: r['stamp'])
    gt = sorted(by.get('gt', []), key=lambda r: r['stamp'])
    lt = np.array([r['stamp'] for r in loc])
    lp = np.array([[r['a'], r['b'], r['c']] for r in loc]).reshape(-1, 3)
    gp = to_map([[r['a'], r['b'], r['c']] for r in gt], origin) if gt else np.zeros((0, 3))
    truth, ok = interpolate([r['stamp'] for r in gt], gp, lt)
    out = {'poses': len(loc), 'paired': int(ok.sum())}
    if ok.sum() >= 3:
        e = np.hypot(*(lp[ok, :2] - truth[ok, :2]).T)
        eh = np.degrees(np.abs(wrap(lp[ok, 2] - truth[ok, 2])))
        r, tr = rigid_fit(lp[ok, :2], truth[ok, :2])
        ea = np.hypot(*((lp[ok, :2] @ r.T + tr) - truth[ok, :2]).T)
        out.update(err_mean=float(e.mean()), err_median=float(np.median(e)), err_p95=float(np.percentile(e, 95)),
                   err_max=float(e.max()), far_share=float((e > FAR).mean()),
                   head_mean=float(eh.mean()), head_p95=float(np.percentile(eh, 95)),
                   aligned_mean=float(ea.mean()), aligned_p95=float(np.percentile(ea, 95)),
                   fit_rot_deg=float(np.degrees(math.atan2(r[1, 0], r[0, 0]))), fit_shift=float(np.hypot(*tr)))
        jit = jitter(lt, lp[:, :2] - truth[:, :2], ok)
        if jit is not None:
            out['jitter'] = jit
    jumps, gaps = jumps_and_gaps(lt, lp[:, :2])
    scans = sorted(r['recv'] for r in by.get('scan', []))
    gdt = np.diff([r['stamp'] for r in gt]) if len(gt) > 1 else np.zeros(0)
    out.update(jumps=len(jumps), max_jump=max(jumps, default=0.0), gaps=len(gaps), gap_s=float(sum(gaps)),
               stalls=int((gdt > MAX_PAIR_DT).sum()), stall_s=float(gdt[gdt > MAX_PAIR_DT].sum()),
               laps=counter_delta([r['a'] for r in sorted(by.get('laps', []), key=lambda r: r['recv'])]),
               collisions=counter_delta([r['a'] for r in sorted(by.get('hits', []), key=lambda r: r['recv'])]),
               scan_hz=float((len(scans) - 1) / (scans[-1] - scans[0])) if len(scans) > 1 else 0.0)
    return out


# ----------------------------------------------------------------------------------------------- files

FIELDS = ['kind', 'stamp', 'recv', 'a', 'b', 'c']


def write_rows(path, rows):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, FIELDS)
        w.writeheader()
        w.writerows(rows)


def read_rows(path):
    with open(path, newline='') as f:
        return [{k: (v if k == 'kind' else float(v)) for k, v in r.items()} for r in csv.DictReader(f)]


def read_origin(track_dir):
    import yaml
    with open(Path(track_dir) / 'map_to_world.yaml') as f:
        o = yaml.safe_load(f)
    return o['x_m'], o['y_m'], o['yaw_rad']


COLUMNS = [('run', '{}'), ('err_mean', '{:.3f}'), ('err_median', '{:.3f}'), ('err_p95', '{:.3f}'),
           ('err_max', '{:.2f}'), ('far_share', '{:.1%}'), ('head_mean', '{:.2f}'), ('head_p95', '{:.2f}'),
           ('aligned_mean', '{:.3f}'), ('jitter', '{:.3f}'), ('jumps', '{}'), ('max_jump', '{:.2f}'),
           ('gap_s', '{:.1f}'), ('stalls', '{}'), ('laps', '{}'), ('collisions', '{}'), ('scan_hz', '{:.0f}')]


def table(results):
    lines = [[name for name, _ in COLUMNS]]
    for run, s in results:
        lines.append([fmt.format(run if name == 'run' else s[name]) if name == 'run' or name in s else '-'
                      for name, fmt in COLUMNS])
    widths = [max(len(x) for x in col) for col in zip(*lines)]
    return '\n'.join('  '.join(x.rjust(w) if i else x.ljust(w) for i, (x, w) in enumerate(zip(line, widths)))
                     for line in lines)


def analyze(out_dir):
    out_dir = Path(out_dir)
    meta = json.loads((out_dir / 'meta.json').read_text())
    origin = tuple(meta['origin'])
    results = [(p.stem, score(read_rows(p), origin)) for p in sorted(out_dir.glob('*.csv'))]
    text = (f"Localization benchmark: track {meta['track']}, speed_scale {meta['speed_scale']}, "
            f"{meta['duration']} s per run, controller {meta['controller']}\n"
            'errors in m (heading in deg) against sim ground truth in the map frame; aligned = after the best rigid '
            'fit (map placement removed)\n\n' + table(results) + '\n')
    (out_dir / 'summary.txt').write_text(text)
    (out_dir / 'summary.json').write_text(json.dumps({'meta': meta, 'runs': dict(results)}, indent=1))
    print(text)
    return results


# ----------------------------------------------------------------------------------------------- recording

def record_run(track, localizer, args, path):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Int32
    from rclpy.qos import qos_profile_sensor_data

    rows = []
    node = rclpy.create_node('localization_benchmark')

    def pose_cb(kind):
        def cb(m):
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            rows.append({'kind': kind, 'stamp': st, 'recv': time.time(), 'a': m.pose.position.x,
                         'b': m.pose.position.y, 'c': yaw_of(m.pose.orientation)})
        return cb

    def count_cb(kind):
        return lambda m: rows.append({'kind': kind, 'stamp': 0.0, 'recv': time.time(), 'a': m.data, 'b': 0, 'c': 0})

    node.create_subscription(PoseStamped, '/localized_pose', pose_cb('loc'), 100)
    node.create_subscription(PoseStamped, '/ground_truth/pose', pose_cb('gt'), 100)
    node.create_subscription(Int32, NS + 'lap_count', count_cb('laps'), 10)
    node.create_subscription(Int32, NS + 'collision_count', count_cb('hits'), 10)
    node.create_subscription(LaserScan, '/scan', lambda m: rows.append(
        {'kind': 'scan', 'stamp': 0.0, 'recv': time.time(), 'a': 0, 'b': 0, 'c': 0}), qos_profile_sensor_data)

    cmd = (f'exec scripts/race.sh {track} localizer:={localizer} speed_scale:={args.speed_scale} '
           f'controller:={args.controller} ' + ' '.join(args.extra))
    log = open(path.with_suffix('.log'), 'w')
    proc = subprocess.Popen(['bash', '-c', cmd], cwd=args.ws, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    print(f'  {path.stem}: racing {args.duration:.0f} s ...', flush=True)
    end = time.time() + args.duration
    try:
        while time.time() < end and proc.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        # one SIGINT to the launch only: it forwards it to its nodes once (signalling the whole group too
        # interrupts the nodes' own shutdown, leaving the car on its last command)
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
        log.close()
        node.destroy_node()
    write_rows(path, rows)
    if not any(r['kind'] == 'gt' for r in rows):
        print('  WARNING: no /ground_truth/pose (bridge publish_restricted_topics:=false?): cannot score',
              file=sys.stderr)
    time.sleep(args.pause)   # nodes gone and their DDS entries expired before the next race resets the car


def run(args):
    import rclpy
    track_dir = Path(args.ws) / 'tracks' / args.track
    for f in ('raceline.csv', 'map.yaml', 'map.pbstream', 'map_to_world.yaml'):
        if not (track_dir / f).exists():
            sys.exit(f'{track_dir / f} missing (map the track first: scripts/map_track.sh {args.track})')
    out_dir = Path(args.ws) / 'log' / 'localization_benchmark' / datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S')
    out_dir.mkdir(parents=True)
    localizers = args.localizers.split(',')
    meta = {'track': args.track, 'localizers': localizers, 'duration': args.duration,
            'speed_scale': args.speed_scale, 'controller': args.controller, 'repeats': args.repeats,
            'extra': args.extra, 'origin': list(read_origin(track_dir)),
            'started': datetime.now(timezone.utc).isoformat(timespec='seconds')}
    (out_dir / 'meta.json').write_text(json.dumps(meta, indent=1))
    print(f'Results in {out_dir}')
    rclpy.init()
    try:
        for n in range(1, args.repeats + 1):
            for loc in localizers:   # alternate, so a sim slowing down over time hits both alike
                record_run(args.track, loc, args, out_dir / f'{loc}_{n}.csv')
    finally:
        rclpy.shutdown()
    analyze(out_dir)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run', help='race and score')
    r.add_argument('--track', required=True)
    r.add_argument('--localizers', default='cartographer,mcl')
    r.add_argument('--duration', type=float, default=60.0, help='s of racing per run')
    r.add_argument('--speed-scale', type=float, default=0.5)
    r.add_argument('--controller', default='pure_pursuit', choices=['pure_pursuit', 'map'])
    r.add_argument('--repeats', type=int, default=1)
    r.add_argument('--pause', type=float, default=8.0, help='s between runs')
    r.add_argument('--ws', default=os.path.expanduser('~/ws'))
    r.add_argument('extra', nargs='*', help='more race launch arguments, e.g. log_laps:=false')
    a = sub.add_parser('analyze', help='re-score a results folder')
    a.add_argument('dir')
    args = ap.parse_args(argv)
    if args.cmd == 'run':
        run(args)
    else:
        analyze(args.dir)


if __name__ == '__main__':
    main()
