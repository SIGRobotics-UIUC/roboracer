"""Compare native Unity WheelHit telemetry with the kinematic estimates.

The two quantities have different definitions; signed and magnitude comparisons
are both reported without fitting a scale or assuming a sign convention.
"""
import json
import sqlite3
from pathlib import Path
import numpy as np

WHEELS = ('front_left', 'front_right', 'rear_left', 'rear_right')
TOPIC = '/autodrive/roboracer_1/wheel_slip'


def read_native(bag):
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    samples = {}
    for db in sorted(Path(bag).glob('*.db3')):
        with sqlite3.connect(db.resolve().as_uri()+'?mode=ro&immutable=1', uri=True) as conn:
            ids = [r[0] for r in conn.execute('SELECT id FROM topics WHERE name=?', (TOPIC,))]
            for topic in ids:
                for (blob,) in conn.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp', (topic,)):
                    d = json.loads(deserialize_message(blob, String).data)
                    samples[int(d['stamp_ns'])] = {w['name']: w for w in d['wheels']}
    return samples


def attach(rows, samples):
    """Rows describe intervals; native telemetry is at the interval endpoint.

    Exact source-stamp matching avoids silently pairing delayed or stale frames.
    Also expose the mean of both endpoint telemetry values for the interval
    comparison, only when the wheel has contact at both endpoints.
    """
    for row in rows:
        end = samples.get(row['stamp_ns'], {})
        start_stamp = row.get('interval_start_stamp_ns', row['stamp_ns']-round(row['dt_wall_s']*1e9))
        start = samples.get(start_stamp, {})
        for wheel in WHEELS:
            e, a = end.get(wheel), start.get(wheel)
            row[wheel+'_native_grounded'] = bool(e['grounded']) if e else None
            for native, estimate in (('forward', 'longitudinal'), ('sideways', 'lateral')):
                row[wheel+'_native_'+estimate+'_slip'] = float(e[native]) if e and e['grounded'] else None
                row[wheel+'_native_mean_'+estimate+'_slip'] = (
                    .5*(float(e[native])+float(a[native]))
                    if e and a and e['grounded'] and a['grounded'] else None)


def summary(rows, lateral_peak=.01, longitudinal_peak=.15):
    result = dict(definition='Unity WheelHit slip versus kinematic estimates; definitions and signs may differ.',
                  alignment='Exact ROS source stamps; native mean of both interval endpoints; grounded at both endpoints.',
                  thresholds=dict(lateral=lateral_peak, longitudinal=longitudinal_peak),
                  available_endpoint_frames=sum(any(r.get(w+'_native_grounded') is not None for w in WHEELS) for r in rows),
                  metrics={})
    for wheel in WHEELS:
        for axis in ('lateral', 'longitudinal'):
            pairs = [(r.get(wheel+'_'+axis+'_slip'), r.get(wheel+'_native_mean_'+axis+'_slip')) for r in rows]
            pairs = np.array([(a,b) for a,b in pairs if a is not None and b is not None and np.isfinite([a,b]).all()])
            endpoint = np.array([r[wheel+'_native_'+axis+'_slip'] for r in rows if r.get(wheel+'_native_'+axis+'_slip') is not None])
            threshold = lateral_peak if axis == 'lateral' else longitudinal_peak
            m = dict(paired_intervals=len(pairs), native_endpoint_samples=len(endpoint))
            if len(endpoint):
                m.update(native_endpoint_abs_p95=float(np.percentile(abs(endpoint),95)), native_endpoint_abs_max=float(np.max(abs(endpoint))),
                         native_endpoint_above_peak_samples=int(np.sum(abs(endpoint)>=threshold)))
            if len(pairs):
                a,b = pairs.T
                corr = lambda x,y: float(np.corrcoef(x,y)[0,1]) if len(x)>2 and np.std(x)>1e-12 and np.std(y)>1e-12 else None
                threshold = lateral_peak if axis == 'lateral' else longitudinal_peak
                m.update(estimate_above_peak_intervals=int(np.sum(abs(a)>=threshold)), native_above_peak_intervals=int(np.sum(abs(b)>=threshold)),
                         estimate_abs_p95=float(np.percentile(abs(a),95)), native_abs_p95=float(np.percentile(abs(b),95)),
                         signed_rmse=float(np.sqrt(np.mean((a-b)**2))), magnitude_rmse=float(np.sqrt(np.mean((abs(a)-abs(b))**2))),
                         signed_correlation=corr(a,b), magnitude_correlation=corr(abs(a),abs(b)))
            result['metrics'][wheel+'_'+axis] = m
    return result


def render(rows, output, result):
    import matplotlib.pyplot as plt
    distance = np.cumsum([r['distance_step_m'] for r in rows])
    fig, axes = plt.subplots(4, 2, figsize=(14,12), sharex=True)
    for i,wheel in enumerate(WHEELS):
        for j,axis in enumerate(('lateral','longitudinal')):
            ax=axes[i,j]
            for key,label in ((wheel+'_'+axis+'_slip','Kinematic estimate'),
                              (wheel+'_native_mean_'+axis+'_slip','Unity WheelHit (endpoint mean)')):
                if any(r.get(key) is not None for r in rows):
                    ax.plot(distance,[r.get(key) if r.get(key) is not None else np.nan for r in rows],label=label,linewidth=1)
            ax.plot(distance, [r.get(wheel+'_native_'+axis+'_slip') if r.get(wheel+'_native_'+axis+'_slip') is not None else np.nan for r in rows], label='Unity instantaneous endpoint', linewidth=.7, alpha=.4)
            ax.set_title(wheel.replace('_',' ')+' '+axis); ax.set_ylabel('Raw signed slip'); ax.grid(alpha=.25);ax.legend(fontsize=7)
    axes[-1,0].set_xlabel('Distance (m)');axes[-1,1].set_xlabel('Distance (m)')
    fig.suptitle('Native Unity telemetry and kinematic slip: definitions may differ')
    fig.tight_layout();fig.savefig(output/'native_slip_comparison.png',dpi=150);plt.close(fig)


def select_primary(rows, cfg, requested='auto'):
    """Use a single slip source for the entire selection; never fill Unity gaps with estimates."""
    available = any(r.get(w+'_native_grounded') is not None for r in rows for w in WHEELS)
    source = 'unity' if requested == 'auto' and available else 'kinematic' if requested == 'auto' else requested
    if source == 'unity' and not available:
        raise ValueError('Unity slip requested but no native telemetry is present in this selection')
    for row in rows:
        row['slip_source'] = source
        for axle, axis, peak in (('front','lateral',cfg.lateral_peak), ('rear','lateral',cfg.lateral_peak),
                                 ('rear','longitudinal',cfg.longitudinal_peak), ('front','longitudinal',cfg.longitudinal_peak)):
            metric = axle+'_'+axis+'_peak_ratio'
            row['kinematic_'+metric] = row.get(metric)
            if source == 'unity':
                values = [row.get(axle+'_'+side+'_native_'+axis+'_slip') for side in ('left','right')]
                row[metric] = max(abs(v) for v in values)/peak if all(v is not None and np.isfinite(v) for v in values) else None
    return source
