#!/usr/bin/env python3
"""HTML report of localization benchmark results (scripts/localization_benchmark.py), one page, no dependencies.

    python3 scripts/localization_report.py log/localization_benchmark/<a>/ log/localization_benchmark/<b>/ ...
        [-o log/localization_benchmark/report.html] [--title TITLE] [--metrics err_mean,err_p95,...]

A folder may name its series: <folder>:cartographer=Carto before,mcl=MCL (runs of one localizer from several
folders under one name are one series; e.g. to compare versions of the code). Default names: Cartographer,
MCL (ours). Series keep the order they first appear in, or --order's (up to 4: the palette's adjacent-safe
slots).

Groups the runs by speed_scale (from each folder's meta.json) and localizer, and draws one small chart per
metric (x = speed scale, one line per localizer through the mean of its runs, a dot per run), plus a table of
every run. Open the file in a browser. --metrics picks the charts and their order (default: all; err_p95 is
only charted when asked for).
"""
import argparse
import html
import json
from pathlib import Path

METRICS = [   # key, title, unit, scale, what it means
    ('err_mean', 'Mean position error', 'cm', 100, 'average distance between the estimate and the true pose'),
    ('err_max', 'Worst position error', 'cm', 100, 'the single largest error in the run'),
    ('err_p95', 'Position error, 95th percentile', 'cm', 100, '95 % of the time the estimate was closer than this'),
    ('head_p95', 'Heading error, 95th percentile', 'deg', 1, '95 % of the time the heading was off by less'),
    ('jitter', 'Pose jitter', 'mm', 1000,
     'how much the estimate wobbles over 50 ms: what pure pursuit steers on'),
    ('lat_p95', 'Sideways error, 95th percentile', 'cm', 100,
     'error across the car\'s heading: what puts the car into a wall'),
    ('lon_p95', 'Along-track error, 95th percentile', 'cm', 100, 'error along the heading: ahead of or behind the car'),
    ('jumps', 'Pose jumps per run', '', 1, 'steps of the estimate faster than 15 m/s (snaps)'),
    ('collisions', 'Crashes per run', '', 1, 'the sim\'s collision count over the run'),
    ('odom_drift', 'Odometry drift', '%', 100,
     'how far /odom strays from the true motion per metre driven: the input both localizers share'),
]
TABLE = ['err_mean', 'err_median', 'err_p95', 'err_max', 'far_share', 'head_mean', 'head_p95', 'aligned_mean',
         'lat_mean', 'lat_p95', 'lon_p95', 'lon_bias', 'jitter', 'jumps', 'max_jump', 'gap_s', 'stalls', 'laps',
         'collisions', 'odom_drift', 'scan_hz']


DEFAULT_NAMES = {'cartographer': 'Cartographer', 'mcl': 'MCL (ours)'}


def load(dirs):
    """(runs, series names in order). dirs: 'folder' or 'folder:localizer=Name,localizer=Name'."""
    runs, series = [], []
    for arg in dirs:
        d, _, spec = arg.partition(':')
        names = dict(DEFAULT_NAMES, **dict(kv.split('=', 1) for kv in spec.split(',') if kv))
        s = json.loads((Path(d) / 'summary.json').read_text())
        meta = s['meta']
        for name, r in s['runs'].items():
            loc = name.rsplit('_', 1)[0]
            label = names.get(loc, loc)
            if label not in series:
                series.append(label)
            runs.append({'name': name, 'localizer': label, 'speed': meta['speed_scale'],
                         'track': meta['track'], 'duration': meta['duration'], 'folder': Path(d).name, **r})
    if len(series) > 4:
        raise SystemExit(f'{len(series)} series; at most 4 (fold some together or make two reports)')
    return runs, series


PAGE = (Path(__file__).parent / 'localization_report.html').read_text()


def plural(n, one, many=None):
    return f'{n} {one if n == 1 else many or one + "s"}'


def findings(runs, series):
    """Headline tiles, one per series: (label, value, note)."""
    out = []

    def pick(loc, key):
        return [r[key] for r in runs if r['localizer'] == loc and key in r]
    for label in series:
        e, n = pick(label, 'err_mean'), sum(r['localizer'] == label for r in runs)
        crashes = sum(pick(label, 'collisions'))
        if e:
            out.append((f'{label}: crashes in {n} runs', f'{crashes}',
                        f'mean error {100 * sum(e) / len(e):.1f} cm; worst {100 * max(pick(label, "err_max")):.0f} cm; '
                        + (f'jitter {1000 * sum(pick(label, "jitter")) / len(pick(label, "jitter")):.0f} mm; '
                           if pick(label, 'jitter') else '')
                        + plural(sum(pick(label, 'jumps')), 'pose jump')))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dirs', nargs='+')
    ap.add_argument('-o', '--output', default='log/localization_benchmark/report.html')
    ap.add_argument('--title', default='Localization benchmark: Cartographer vs our MCL')
    ap.add_argument('--order', help='series names in the order to show them, comma-separated')
    ap.add_argument('--metrics', help='charts to draw, in order, comma-separated keys: '
                    + ', '.join(m[0] for m in METRICS))
    args = ap.parse_args(argv)
    runs, series = load(args.dirs)
    by_key = {m[0]: m for m in METRICS}
    if args.metrics:
        unknown = [k for k in args.metrics.split(',') if k not in by_key]
        if unknown:
            raise SystemExit(f'--metrics: unknown {", ".join(unknown)}; choose from {", ".join(by_key)}')
        metrics = [by_key[k] for k in args.metrics.split(',')]
    else:
        metrics = [m for m in METRICS if m[0] != 'err_p95']
    if args.order:
        order = args.order.split(',')
        series = sorted(series, key=lambda n: order.index(n) if n in order else len(order))
    tracks = sorted({r['track'] for r in runs})
    speeds = sorted({r['speed'] for r in runs})
    reps = max(sum(1 for r in runs if r['localizer'] == loc and r['speed'] == s)
               for loc in {r['localizer'] for r in runs} for s in speeds)
    sub = (f'Track {", ".join(tracks)} in the AutoDRIVE simulator, pure pursuit on the raceline, '
           f'{runs[0]["duration"]:.0f} s per run, up to {reps} runs per localizer at speed scale '
           f'{", ".join(str(s) for s in speeds)} (fraction of the planned raceline speed). '
           'Ground truth from the simulator is used only for scoring.')
    page = (PAGE.replace('__SUB__', html.escape(sub)).replace('__RUNS__', json.dumps(runs))
            .replace('__METRICS__', json.dumps(metrics)).replace('__TABLE__', json.dumps(TABLE))
            .replace('__FINDINGS__', json.dumps(findings(runs, series))).replace('__SERIES__', json.dumps(series))
            .replace('__TITLE__', html.escape(args.title)))
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(page)
    print(f'Wrote {args.output} ({len(runs)} runs)')


if __name__ == '__main__':
    main()
