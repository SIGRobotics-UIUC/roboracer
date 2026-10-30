#!/usr/bin/env python3
"""ros2 run planning build_raceline <name>

Reads   <tracks>/<name>/map.pgm, map.yaml, trajectory.csv (optional: driving direction + start)
Writes  <tracks>/<name>/centerline.csv, raceline.csv, preview.png   (formats: docs/pipeline.md)

<tracks> = $TRACKS_DIR or ~/ws/tracks. The raceline is TUM's minimum-time optimization (opt_type
'mintime') with racecar.ini as is. So that a track is never left without a raceline, a failed step
falls back to the next one:
  mintime        TUM mintime, racecar.ini as is: the expected line
  mintime-retry  TUM mintime without safe_traj, its speeds then held to the safe_traj acceleration limits
  centerline     the centerline + a speed profile within those limits (no optimizer, cannot fail)
Anything but a clean mintime line (a fallback, or a line closer to a wall than half the car width)
ends in a WARNING block. raceline.csv's second header line records how it was made
(# raceline: {"method": ..., "clean": ..., "notes": [...], ...}); pure_pursuit_node logs it.

Exit status: 0 clean mintime line, 2 raceline written but NOT a clean mintime line, 1 no raceline.
"""
import argparse
import configparser
import datetime
import json
import os
import sys
import tempfile
import time

import numpy as np

from planning.centerline import extract_centerline, write_centerline
from planning.preview import save_preview
from planning.raceline import (OptimizationError, TUM_PYTHON, check_inside, lap_time, path_traj, run_mintime,
                               speed_profile, write_raceline)

EXIT_NOT_CLEAN = 2


def default_config():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('planning'), 'config', 'racecar.ini')
    except Exception:   # not installed: running from the source tree
        return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            'config', 'racecar.ini')


def ini_with(ini, changes, path):
    """Writes a copy of ini with optim_opts_mintime entries replaced to path (comments are dropped)."""
    cfg = configparser.ConfigParser()
    cfg.read(ini)
    opts = json.loads(cfg.get('OPTIMIZATION_OPTIONS', 'optim_opts_mintime'))
    opts.update(changes)
    cfg.set('OPTIMIZATION_OPTIONS', 'optim_opts_mintime', json.dumps(opts))
    with open(path, 'w') as f:
        cfg.write(f)
    return path


def reason(e):
    """One line for a failed method, with IPOPT's EXIT message if there is one."""
    if not isinstance(e, OptimizationError):
        return f'{type(e).__name__}: ' + (str(e).splitlines() or [''])[0]
    return e.summary + (f' ({e.ipopt_exit})' if e.ipopt_exit else '')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('name', help='track name, i.e. directory under the tracks dir')
    ap.add_argument('--tracks-dir', default=os.environ.get('TRACKS_DIR', os.path.expanduser('~/ws/tracks')))
    ap.add_argument('--config', default=default_config(), help='TUM racecar.ini')
    ap.add_argument('--step', type=float, default=0.1, help='centerline point spacing [m]')
    ap.add_argument('--smooth', type=float, default=0.05, help='centerline spline smoothing [m rms]')
    ap.add_argument('--carve-trajectory', type=float, nargs='?', const=0.3, default=None, metavar='RADIUS',
                    help='treat cells within RADIUS m (default 0.3) of the mapping lap as free; only for '
                         'tracks whose map shows walls across the driven path (overpass)')
    ap.add_argument('--tum-python', default=TUM_PYTHON)
    args = ap.parse_args(argv)

    d = os.path.join(args.tracks_dir, args.name)
    map_yaml = os.path.join(d, 'map.yaml')
    if not os.path.isfile(map_yaml):
        sys.exit(f'build_raceline: {map_yaml} not found')
    traj_csv = os.path.join(d, 'trajectory.csv')
    has_traj = os.path.isfile(traj_csv)
    t0 = time.monotonic()
    # never leave stale outputs next to a failed run
    for f in ('centerline.csv', 'raceline.csv', 'preview.png'):
        if os.path.exists(os.path.join(d, f)):
            os.remove(os.path.join(d, f))

    how = ('following the mapping lap in trajectory.csv' if has_traj
           else 'no trajectory.csv: counter-clockwise, start near the map origin')
    print(f'[1/3] centerline from {map_yaml} ({how})')
    try:
        track, m, mask = extract_centerline(map_yaml, traj_csv if has_traj else None,
                                            step=args.step, smooth_m=args.smooth,
                                            carve_radius_m=args.carve_trajectory)
    except RuntimeError as e:
        sys.exit(f'build_raceline: centerline extraction failed: {e}')
    cl_csv = os.path.join(d, 'centerline.csv')
    write_centerline(cl_csv, track)
    seg = np.hypot(*(np.roll(track[:, :2], -1, axis=0) - track[:, :2]).T)
    width = track[:, 2] + track[:, 3]
    print(f'      {len(track)} points, {seg.sum():.1f} m, width {width.min():.2f}-{width.max():.2f} m -> {cl_csv}')

    rl_csv = os.path.join(d, 'raceline.csv')
    cfg = configparser.ConfigParser()
    cfg.read(args.config)
    veh = json.loads(cfg.get('GENERAL_OPTIONS', 'veh_params'))
    opts = json.loads(cfg.get('OPTIMIZATION_OPTIONS', 'optim_opts_mintime'))
    width_veh, width_opt = veh['width'], opts['width_opt']
    # the fallbacks' speeds: safe_traj's acceleration ellipse, laterally at most the friction limit mue * g
    # (ay_safe 7.0 is above it; mintime stays below through its tire model: max |ay| ~6.1 on the iros2026 maps)
    a_grip = opts['mue'] * veh['g']
    limits = dict(v_max=veh['v_max'], ax_pos=opts['ax_pos_safe'] or a_grip,
                  ax_neg=abs(opts['ax_neg_safe'] or a_grip), ay_max=min(opts['ay_safe'] or a_grip, a_grip))

    def check_width():
        narrow = width < width_opt
        if narrow.any():
            i = np.argmin(width)
            raise OptimizationError(
                'track is narrower than width_opt = %.2f m over %.1f m, down to %.2f m at (%.1f, %.1f)'
                % (width_opt, narrow.sum() * seg.mean(), width[i], *track[i, :2]))

    def mintime():
        check_width()
        traj, dt, ipopt_exit = run_mintime(cl_csv, args.config, python=args.tum_python)
        return traj, dt, ipopt_exit, {}

    def mintime_retry():
        # without the acceleration ellipse; the speeds are held to it afterwards instead
        check_width()
        changed = {'safe_traj': False}
        with tempfile.TemporaryDirectory(prefix='build_raceline_') as work:
            ini = ini_with(args.config, changed, os.path.join(work, 'racecar_retry.ini'))
            traj, dt, ipopt_exit = run_mintime(cl_csv, ini, python=args.tum_python)
        changed['speeds'] = 'mintime speeds capped by a speed profile within the safe_traj limits'
        return speed_profile(traj, v_cap=traj[:, 5], **limits), dt, ipopt_exit, changed

    def centerline():
        changed = {'path': 'centerline', 'speeds': 'speed profile within the safe_traj limits'}
        return speed_profile(path_traj(track[:, :2]), **limits), 0.0, None, changed

    notes = []      # every way the result differs from a clean mintime line
    traj = None
    methods = (('mintime', mintime), ('mintime-retry', mintime_retry), ('centerline', centerline))
    for k, (method, run) in enumerate(methods, 1):
        print(f'[2/3] attempt {k}/{len(methods)}: {method}' + (' (FALLBACK)' if k > 1 else f' ({args.config})'))
        try:
            traj, dt, ipopt_exit, changed = run()
            clearance = check_inside(traj, m, mask, width_veh / 2)
            break
        except Exception as e:      # any failure moves on to the next method, never to no raceline
            print(f'build_raceline: {method}: {type(e).__name__}: {e}', file=sys.stderr)
            notes.append(f'{method} failed: {reason(e)}')
            traj = None
    if traj is None:
        save_preview(os.path.join(d, 'preview.png'), m, track, None, f'{args.name}: no raceline, all methods FAILED')
        sys.exit('build_raceline: no raceline: ' + '; '.join(notes))
    if clearance < width_veh / 2:
        notes.append(f'min wall clearance {clearance:.2f} m < half the car width {width_veh / 2:.2f} m')
    clean = method == 'mintime' and not notes

    t_lap = lap_time(traj)
    solver = f'IPOPT EXIT: {ipopt_exit}' if ipopt_exit else None
    provenance = {'method': method, 'clean': clean, 'changed': changed, 'notes': notes, 'solver': solver,
                  'lap_s': round(t_lap, 2), 'min_clearance_m': round(clearance, 3),
                  'built': datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
                  'track': args.name, 'config': os.path.abspath(args.config)}
    write_raceline(rl_csv, traj, provenance)
    print(f'      {method}: {len(traj)} points, {traj[-1, 0]:.1f} m, lap {t_lap:.2f} s, vx {traj[:, 5].min():.1f}-'
          f'{traj[:, 5].max():.1f} m/s, min wall clearance {clearance:.2f} m, solve {dt:.1f} s -> {rl_csv}')

    png = os.path.join(d, 'preview.png')
    title = {'mintime': 'TUM mintime', 'mintime-retry': 'TUM mintime RETRY without safe_traj (FALLBACK)',
             'centerline': 'CENTERLINE (FALLBACK)'}[method]
    save_preview(png, m, track, traj,
                 f'{args.name}: {title}, lap {t_lap:.2f} s, {traj[-1, 0]:.1f} m, vx max {traj[:, 5].max():.1f} m/s')
    print(f'[3/3] preview -> {png}   (total {time.monotonic() - t0:.1f} s)')

    if not clean:
        bar = '!' * 100
        lines = ([bar, f'WARNING: {rl_csv} is NOT a clean TUM mintime raceline: method {method}']
                 + [f'  - {n}' for n in notes]
                 + ([f'  - differs from a mintime line: {json.dumps(changed)}'] if changed else [])
                 + ['  The car will drive this line (pure_pursuit_node warns too). Check preview.png. Exit status 2.',
                    bar])
        print('\n'.join(lines), file=sys.stderr)
        sys.exit(EXIT_NOT_CLEAN)


if __name__ == '__main__':
    main()
