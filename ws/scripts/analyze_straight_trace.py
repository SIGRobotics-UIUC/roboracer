#!/usr/bin/env python3
"""Compare raw simulator motion with frame arrival and lap-timer intervals.

The lap timer is a candidate simulation clock, not a verified physics timestamp.
Only matching phase labels from straight_line_trial CSV files are analyzed.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def stats(values):
    a = np.array(values)
    a = a[np.isfinite(a)]
    return dict(n=len(a), median=float(np.median(a)), p05=float(np.percentile(a, 5)),
                p95=float(np.percentile(a, 95)), maximum=float(np.max(a))) if len(a) else None


def compute(frames, phases, radius=.0585):
    selected = [(r, phases[r['ros_stamp_ns']]) for r in frames if r['ros_stamp_ns'] in phases]
    rows = []
    for (a, pa), (b, pb) in zip(selected, selected[1:]):
        if pa != pb or pa not in ('accelerate_cruise', 'braking'):
            continue
        da, db = a['payload'], b['payload']
        vec = lambda d, k: np.array([float(v) for v in d[k].split()])
        qa, qb = vec(da, 'V1 Orientation Quaternion'), vec(db, 'V1 Orientation Quaternion')
        yaw = lambda q: np.arctan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]**2+q[2]**2))
        ya, yb = yaw(qa), yaw(qb)
        dyaw = np.arctan2(np.sin(yb-ya), np.cos(yb-ya))
        heading = ya+.5*dyaw
        dp = vec(db, 'V1 Position')-vec(da, 'V1 Position')
        ds = (dp[0]*np.cos(heading)+dp[1]*np.sin(heading))/np.sinc(dyaw/(2*np.pi))
        wall = (b['arrival_monotonic_ns']-a['arrival_monotonic_ns'])*1e-9
        lap = float(db['V1 Lap Time'])-float(da['V1 Lap Time'])
        va, vb = vec(da, 'V1 Linear Velocity')[0], vec(db, 'V1 Linear Velocity')[0]
        v = .5*(va+vb)
        dw = np.mean(vec(db, 'V1 Encoder Angles')-vec(da, 'V1 Encoder Angles'))*radius
        if wall <= 0 or wall > .5 or ds <= .015 or ds > .8 or v <= .5 or abs(dyaw) > .05:
            continue
        rows.append(dict(stamp_ns=b['ros_stamp_ns'], phase=pb, wall_dt=wall, lap_dt=lap,
                         forward_m=ds, wheel_m=dw, vx=v, vx_change=vb-va,
                         gt_speed_wall=ds/wall, gt_speed_lap=ds/lap if lap>0 else float('nan'),
                         wheel_excess_pct=100*(dw/ds-1),
                         work_ms=b['bridge_work_ns']*1e-6))
    report = {}
    for name, subset in [('all', rows),
                         ('cruise', [r for r in rows if r['phase']=='accelerate_cruise' and abs(r['vx_change']) < .02]),
                         ('accelerate_cruise', [r for r in rows if r['phase']=='accelerate_cruise']),
                         ('braking', [r for r in rows if r['phase']=='braking'])]:
        report[name] = dict(intervals=len(subset),
            arrival_interval_s=stats([r['wall_dt'] for r in subset]),
            bridge_work_ms=stats([r['work_ms'] for r in subset]),
            wheel_excess_pct=stats([r['wheel_excess_pct'] for r in subset]),
            position_speed_over_reported_wall=stats([r['gt_speed_wall']/r['vx'] for r in subset]),
            position_speed_over_reported_lap=stats([r['gt_speed_lap']/r['vx'] for r in subset]),
            lap_dt_over_wall_dt=stats([r['lap_dt']/r['wall_dt'] for r in subset if r['lap_dt']>0]),
            intervals_over_120ms=sum(r['wall_dt']>.12 for r in subset))
    median_dt=np.median([r['wall_dt'] for r in rows]) if rows else np.nan
    for name, subset in [('normal', [r for r in rows if r['wall_dt'] <= 1.5*median_dt]),
                         ('long_gap', [r for r in rows if r['wall_dt'] > 1.5*median_dt])]:
        report[name] = dict(intervals=len(subset),
            arrival_interval_s=stats([r['wall_dt'] for r in subset]),
            position_speed_over_reported_wall=stats([r['gt_speed_wall']/r['vx'] for r in subset]),
            position_speed_over_reported_lap=stats([r['gt_speed_lap']/r['vx'] for r in subset]))
    return report, rows


if __name__ == '__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('trace')
    ap.add_argument('trials', nargs='+', help='CSV output(s) from straight_line_trial.py')
    ap.add_argument('--output', required=True)
    args=ap.parse_args()
    frames=[json.loads(line) for line in open(args.trace)]
    all_reports={}
    all_rows=[]
    for trial in args.trials:
        with open(trial) as stream:
            phases={int(r['stamp_ns']):r['phase'] for r in csv.DictReader(stream)}
        report, rows=compute(frames, phases)
        all_reports[trial]=report
        for row in rows:
            row['trial']=trial
        all_rows.extend(rows)
    all_reports['payload_keys']=frames[0]['payload_keys'] if frames else []
    Path(args.output+'.json').write_text(json.dumps(all_reports, indent=2)+'\n')
    if all_rows:
        with open(args.output+'.csv','w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(all_rows[0]))
            writer.writeheader(); writer.writerows(all_rows)
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes=plt.subplots(3,len(args.trials),figsize=(6*len(args.trials),9),
                               sharex='col',squeeze=False)
        for j, trial in enumerate(args.trials):
            rr=[r for r in all_rows if r['trial']==trial]
            if not rr:
                continue
            t0=rr[0]['stamp_ns']
            tt=np.array([(r['stamp_ns']-t0)*1e-9 for r in rr])
            label=Path(trial).stem
            axes[0,j].set_title(label)
            axes[0,j].plot(tt,[r['vx'] for r in rr],label='reported velocity')
            axes[0,j].plot(tt,[r['gt_speed_wall'] for r in rr],'.',label='displacement / wall interval')
            axes[0,j].plot(tt,[r['gt_speed_lap'] for r in rr],'--',label='displacement / lap interval')
            axes[1,j].plot(tt,[r['wall_dt']*1000 for r in rr],'.-',label='arrival interval')
            axes[1,j].plot(tt,[r['lap_dt']*1000 for r in rr],'.',label='lap timer interval')
            axes[2,j].plot(tt,[r['wheel_excess_pct'] for r in rr],'.-',label='encoder travel excess')
            axes[2,j].set_xlabel('Wall seconds since first analyzed sample')
        axes[0,0].set_ylabel('Forward speed (m/s)')
        axes[1,0].set_ylabel('Frame interval (ms)')
        axes[2,0].set_ylabel('Wheel travel excess (%)')
        for ax in axes.flat:
            ax.grid(alpha=.3); ax.legend(fontsize=8)
        fig.tight_layout(); fig.savefig(args.output+'.png',dpi=150)
    print(json.dumps(all_reports,indent=2))
