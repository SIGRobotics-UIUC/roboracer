#!/usr/bin/env python3
"""Compare the logged runs of a track (lap_logger_node's tracks/<track>/runs/*.csv), one line per run.

    ros2 run control lap_report <track> [--last N]
    ros2 run control lap_report path/to/run.csv ...       (the laps of the given runs)

Per run: its settings (controller, speed_scale, localizer, avoid), laps finished and aborted, the best lap,
the mean flying lap, and the mean distance from the raceline over the finished laps.
"""
import argparse
import csv
import glob
import json
import os
import sys

from perception.tracks import track_dir


def read_run(path):
    """(header dict, rows as dicts) of one run csv."""
    with open(path, newline='') as f:
        first = f.readline()
        header = json.loads(first[len('# run: '):]) if first.startswith('# run: ') else {}
        if not header:
            f.seek(0)
        return header, list(csv.DictReader(f))


def setting(value):
    return '-' if value is None else str(value).lower()


def run_line(path):
    header, rows = read_run(path)
    ok = [r for r in rows if r['status'] == 'ok']
    times = [float(r['time_s']) for r in ok]
    flying = [float(r['time_s']) for r in ok if r['start'] == 'flying']
    offsets = [float(r['mean_offset_m']) for r in ok]
    mean = lambda v: f'{sum(v) / len(v):.2f}' if v else '-'   # noqa: E731
    settings = [setting(header.get(k)) for k in ('controller', 'speed_scale', 'localizer', 'avoid')]
    return [os.path.basename(path)[:-4], *settings, str(len(ok)), str(len(rows) - len(ok)),
            f'{min(times):.2f}' if times else '-', mean(flying), mean(offsets)]


def table(lines, head):
    widths = [max(len(x) for x in col) for col in zip(head, *lines)]
    for line in [head, *lines]:
        print('  '.join(x.ljust(w) for x, w in zip(line, widths)).rstrip())


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('target', nargs='+', help='a track name, or run csv files')
    ap.add_argument('--last', type=int, default=0, help="only the track's last N runs")
    args = ap.parse_args(argv)

    if len(args.target) == 1 and not args.target[0].endswith('.csv'):
        runs = sorted(glob.glob(os.path.join(track_dir(args.target[0]), 'runs', '*.csv')))
        if not runs:
            print(f'no runs in {os.path.join(track_dir(args.target[0]), "runs")} '
                  f'(race.sh logs them, log_laps:=true)', file=sys.stderr)
            return 1
        if args.last:
            runs = runs[-args.last:]
        table([run_line(r) for r in runs],
              ['run', 'controller', 'speed_scale', 'localizer', 'avoid', 'laps', 'aborted', 'best_s',
               'flying_mean_s', 'offset_m'])
        return 0

    for path in args.target:
        header, rows = read_run(path)
        print(f'{path}: {json.dumps({k: header.get(k) for k in ("track", "started", "speed_scale", "localizer")})}')
        table([[r['lap'], r['status'], r['start'], r['time_s'], r['mean_speed_mps'], r['max_offset_m'],
                r['collisions'], r['note']] for r in rows],
              ['lap', 'status', 'start', 'time_s', 'mean_speed', 'max_offset_m', 'collisions', 'note'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
