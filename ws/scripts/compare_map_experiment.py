#!/usr/bin/env python3
"""Regenerate a comparative HTML/CSV/JSON report for map_control_experiment.py."""
import argparse
import base64
import csv
import html
import json
from pathlib import Path
import statistics


def spread(values):
    values = [v for v in values if v is not None]
    if not values:
        return dict(n=0, mean=None, minimum=None, maximum=None, sample_stddev=None)
    return dict(n=len(values), mean=statistics.mean(values), minimum=min(values), maximum=max(values),
                sample_stddev=statistics.stdev(values) if len(values) > 1 else None)


def compare(directory, earlier=()):
    directory = Path(directory)
    manifest = json.loads((directory/'manifest.json').read_text())
    repeatability = manifest.get('mode') == 'repeatability'
    supplementary = []
    supplementary_overlays = []
    for source in earlier:
        previous = json.loads((Path(source)/'manifest.json').read_text())
        if previous.get('fixed_input_sha256') != manifest.get('fixed_input_sha256') or previous['speed_scale'] != manifest['speed_scale']:
            raise ValueError('Earlier configuration hashes or speed scale do not match')
        for trial in previous['trials']:
            if trial['steering_rate_limit_radps'] != 3.2:
                continue
            report_path = Path(trial['trial_directory'])/'native_flying_report'
            if not (report_path/'summary.json').exists():
                continue
            summary = json.loads((report_path/'summary.json').read_text())
            native = [lap['lap_time_s'] for lap in trial['native_laps'] if lap['increment'] == 2]
            if not native or summary['lap']['selection'] != 'native:2':
                continue
            supplementary.append(dict(source=str(source), trial_status=trial['status'],
                flying_lap_s=native[0], tracking_rms_m=summary['tracking_error']['distance_weighted_rms_m'],
                front_p95_peak_ratio=summary['metrics']['front_lateral_peak_ratio']['p95'],
                front_maximum_peak_ratio=summary['metrics']['front_lateral_peak_ratio']['maximum']))
            supplementary_overlays.append(dict(index=f'Earlier {len(supplementary)} (failed later)',
                supplementary_native_lap=True, trial_directory=str(report_path.parent),
                report_directory=str(report_path), report_summary=summary))
    rows = []
    for trial in manifest['trials']:
        report = trial.get('report_summary', {})
        tracking = report.get('tracking_error', {})
        front = report.get('metrics', {}).get('front_lateral_peak_ratio', {})
        counts = report.get('crossing_counts', {}).get('front_lateral_peak_ratio', {})
        clock_events = trial.get('clock_discontinuities', [])
        comparable = (trial['status'] == 'completed' and
                      trial.get('report_status') in ('second_geometric_lap', 'native_flying_lap') and not clock_events)
        rows.append(dict(trial=trial['index'], rate_radps=trial['steering_rate_limit_radps'],
            status=trial['status'], comparable=comparable,
            flying_lap_mean_s=trial.get('mean_flying_lap_s'),
            tracking_rms_m=tracking.get('distance_weighted_rms_m'),
            tracking_p95_m=tracking.get('p95_abs_m'),
            heading_p95_deg=tracking.get('p95_abs_heading_deg'),
            front_above_peak_distance_pct=front.get('beyond_peak_distance_pct'),
            front_p95_peak_ratio=front.get('p95'), front_maximum_peak_ratio=front.get('maximum'),
            rear_lateral_p95_peak_ratio=report.get('metrics', {}).get('rear_lateral_peak_ratio', {}).get('p95'),
            rear_longitudinal_p95_peak_ratio=report.get('metrics', {}).get('rear_longitudinal_peak_ratio', {}).get('p95'),
            front_brief_events=counts.get('brief'), front_sustained_events=counts.get('sustained'),
            long_frame_intervals=report.get('long_frame_intervals'),
            collision_change=trial.get('collision_change'), report_status=trial.get('report_status'),
            clock_discontinuities=len(clock_events),
            host_wall_clock_steps=len(trial['host_wall_clock_steps']) if 'host_wall_clock_steps' in trial else None,
            report_path=str(Path(Path(trial['trial_directory']).name)/'report'/'report.html')))
    groups = []
    for rate in sorted({r['rate_radps'] for r in rows}, reverse=True):
        eligible = [r for r in rows if r['rate_radps'] == rate and r['comparable'] and r['flying_lap_mean_s'] is not None]
        groups.append(dict(rate_radps=rate, comparable_trials=len(eligible),
            mean_trial_lap_time_s=statistics.mean(r['flying_lap_mean_s'] for r in eligible) if eligible else None,
            minimum_trial_lap_time_s=min(r['flying_lap_mean_s'] for r in eligible) if eligible else None,
            maximum_trial_lap_time_s=max(r['flying_lap_mean_s'] for r in eligible) if eligible else None,
            median_trial_tracking_rms_m=statistics.median(r['tracking_rms_m'] for r in eligible if r['tracking_rms_m'] is not None)
                if any(r['tracking_rms_m'] is not None for r in eligible) else None))
    result = dict(status=manifest.get('status', 'not_started'), fixed_input_sha256=manifest.get('fixed_input_sha256'),
                  speed_scale=manifest['speed_scale'], trials=rows, groups=groups)
    repeat_stats = None
    if repeatability:
        measured = [r for r in rows if r['status'] == 'completed' and r['report_status'] == 'native_flying_lap']
        repeat_stats = dict(attempted_runs=len(rows), completed_runs=sum(r['status'] == 'completed' for r in rows),
            measured_flying_lap_reports=len(measured),
            runs_with_collision=sum((r['collision_change'] or 0) > 0 for r in rows),
            runs_with_clock_jumps=sum(r['clock_discontinuities'] > 0 for r in rows),
            runs_with_recorded_host_wall_steps=sum((r['host_wall_clock_steps'] or 0) > 0 for r in rows),
            clean_comparison_runs=sum(r['comparable'] for r in rows),
            measured_population='Completed native flying-lap reports, including flagged clock issues; failures remain in run counts.',
            metrics={key: spread([r[key] for r in measured]) for key in
                     ('flying_lap_mean_s', 'tracking_rms_m', 'tracking_p95_m', 'front_p95_peak_ratio',
                      'front_maximum_peak_ratio', 'rear_lateral_p95_peak_ratio', 'rear_longitudinal_p95_peak_ratio',
                      'front_above_peak_distance_pct', 'front_brief_events', 'front_sustained_events')})
        result['repeatability'] = repeat_stats
        assessment = []
        if len(measured) < 2:
            assessment.append('Report repeatability is not established: fewer than two measured flying-lap reports are available.')
        if repeat_stats['completed_runs'] < len(rows):
            assessment.append(f"The current configuration did not reliably complete this protocol: {repeat_stats['completed_runs']}/{len(rows)} runs completed.")
        if repeat_stats['runs_with_clock_jumps']:
            assessment.append('Clock discontinuities were observed; these runs describe current behavior but cannot isolate controller accuracy from timing effects.')
        repeat_stats['assessment'] = assessment
        if supplementary:
            pooled = list(supplementary)+[dict(flying_lap_s=r['flying_lap_mean_s'],
                tracking_rms_m=r['tracking_rms_m'], front_p95_peak_ratio=r['front_p95_peak_ratio'],
                front_maximum_peak_ratio=r['front_maximum_peak_ratio']) for r in measured]
            result['supplementary_completed_laps'] = dict(earlier_sources=list(earlier),
                earlier_laps=supplementary, population='Completed flying laps only, including earlier trials that failed later; not a run-success statistic.',
                pooled_metrics={key: spread([r[key] for r in pooled]) for key in
                                ('flying_lap_s', 'tracking_rms_m', 'front_p95_peak_ratio', 'front_maximum_peak_ratio')})
    (directory/'comparison.json').write_text(json.dumps(result, indent=2)+'\n')
    if rows:
        with open(directory/'comparison.csv', 'w', newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    fmt=lambda value: 'unavailable' if value is None else f'{value:.3f}'
    title = 'Current MAP configuration — repeatability' if repeatability else 'Fixed-speed-profile MAP steering-rate experiment'
    text = ['# '+title, '',
        f"Status: {result['status']}. Speed scale: {manifest['speed_scale']}. Order: {manifest['rates_radps']} rad/s.", '',
        'Identical controller, raceline, speed scale, speed cap, lookahead, steering-rate limit and speed-loop configuration.' if repeatability else 'Same raceline, speed scale, speed cap, lookahead and speed-loop configuration. Only steering-rate limit varies.', '',
        'Native flying-lap times exclude warm-up. New reports select the window between the first and second native lap-counter crossings.',
        'Observed repeatability retains timing-flagged completed laps. Failures remain visible; clean-comparison eligibility is stricter and excludes clock jumps or fallback windows.' if repeatability else 'Incomplete trials, clock-discontinuous trials, and fallback report selections are excluded from grouped comparisons. Two repeats are exploratory, not statistical proof.', '',
        '| Trial | Rate (rad/s) | Status | Report window | ROS clock jumps | Host wall steps | Eligible | Flying lap mean (s) | Tracking RMS (m) | Tracking P95 (m) | Front brief / sustained |',
        '|---|---:|---|---|---:|---:|---|---:|---:|---:|---|']
    table='<table><tr><th>Trial</th><th>Rate (rad/s)</th><th>Status</th><th>Report window</th><th>ROS clock jumps</th><th>Host wall steps</th><th>Eligible</th><th>Flying lap mean (s)</th><th>Tracking RMS (m)</th><th>Tracking P95 (m)</th><th>Front brief / sustained</th></tr>'
    for row in rows:
        window = 'native flying lap' if row['report_status'] == 'native_flying_lap' else 'fallback / not flying-lap comparable'
        cells=[str(row['trial']),str(row['rate_radps']),row['status'],window,str(row['clock_discontinuities']),
               str(row['host_wall_clock_steps']) if row['host_wall_clock_steps'] is not None else 'unrecorded',
               'yes' if row['comparable'] else 'no',fmt(row['flying_lap_mean_s']),
               fmt(row['tracking_rms_m']),fmt(row['tracking_p95_m']),f"{row['front_brief_events']} / {row['front_sustained_events']}"]
        text.append('| '+' | '.join(cells)+' |')
        table+='<tr><td><a href="'+html.escape(row['report_path'])+'">'+cells[0]+'</a></td>'+''.join('<td>'+html.escape(c)+'</td>' for c in cells[1:])+'</tr>'
    table+='</table>'
    repeat_html = ''
    if repeat_stats is not None:
        status = (f"{repeat_stats['completed_runs']}/{repeat_stats['attempted_runs']} runs completed; "
                  f"{repeat_stats['measured_flying_lap_reports']} measured flying-lap reports; "
                  f"{repeat_stats['runs_with_collision']} collision runs; "
                  f"{repeat_stats['runs_with_clock_jumps']} runs with clock jumps.")
        repeat_html = '<h2>Observed repeatability</h2><p>'+html.escape(status)+'</p>'
        repeat_html += ''.join('<p><strong>'+html.escape(t)+'</strong></p>' for t in repeat_stats['assessment'])
        repeat_html += '<p>Statistics below include completed laps with timing flags. They describe observed variability, not absolute accuracy or clean gain comparisons.</p>'
        repeat_html += '<table><tr><th>Metric</th><th>N</th><th>Mean</th><th>Min</th><th>Max</th><th>Sample standard deviation</th></tr>'
        text += ['', '## Observed repeatability', '', status, '',
                 *repeat_stats['assessment'], '',
                 '| Metric | N | Mean | Min | Max | Sample standard deviation |', '|---|---:|---:|---:|---:|---:|']
        for key, value in repeat_stats['metrics'].items():
            cells=[key, str(value['n'])]+[fmt(value[k]) for k in ('mean', 'minimum', 'maximum', 'sample_stddev')]
            text.append('| '+' | '.join(cells)+' |')
            repeat_html += '<tr>'+''.join('<td>'+html.escape(c)+'</td>' for c in cells)+'</tr>'
        repeat_html += '</table>'
    if supplementary:
        extra = result['supplementary_completed_laps']
        note = 'Supplementary completed flying laps from earlier identical-configuration attempts. Those trials failed later; the four-run completion counts above are unchanged. This is conditional on completing a lap, not proof of reliability.'
        repeat_html += '<h2>Supplementary same-configuration flying laps</h2><p>'+html.escape(note)+'</p>'
        repeat_html += '<table><tr><th>Metric</th><th>N completed laps</th><th>Minimum</th><th>Maximum</th><th>Sample standard deviation</th></tr>'
        text += ['', '## Supplementary same-configuration completed laps', '', note, '',
                 '| Metric | N | Minimum | Maximum | Sample standard deviation |', '|---|---:|---:|---:|---:|']
        for key, value in extra['pooled_metrics'].items():
            cells=[key,str(value['n'])]+[fmt(value[k]) for k in ('minimum','maximum','sample_stddev')]
            text.append('| '+' | '.join(cells)+' |')
            repeat_html += '<tr>'+''.join('<td>'+html.escape(c)+'</td>' for c in cells)+'</tr>'
        repeat_html += '</table>'
    if not rows:
        text += ['', 'No trials have completed. The simulator must stream fresh ground truth before the experiment can run.']
        table+='<p>No trial results yet. Simulator sensor streaming is required.</p>'
    images=''
    if rows:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(3,1,figsize=(10,10),sharex=True)
        for rate in sorted({r['rate_radps'] for r in rows}, reverse=True):
            rr=[r for r in rows if r['rate_radps']==rate and r['comparable']]
            for ax,key,label in zip(axes,['flying_lap_mean_s','tracking_rms_m','front_above_peak_distance_pct'],
                                   ['Native flying lap mean (s)','Tracking RMS (m)','Front distance at/above peak (%)']):
                available=[r for r in rr if r[key] is not None]
                ax.plot([r['trial'] for r in available],[r[key] for r in available],'o',label=f'{rate} rad/s')
                ax.set_ylabel(label);ax.grid(alpha=.25)
        for ax, key in zip(axes, ['flying_lap_mean_s', 'tracking_rms_m', 'front_above_peak_distance_pct']):
            excluded=[r for r in rows if not r['comparable'] and r[key] is not None]
            ax.scatter([r['trial'] for r in excluded], [r[key] for r in excluded],
                       marker='x', color='.45', label='Excluded: timing/incomplete/lap window')
        axes[-1].set_xlabel('Trial order');axes[0].legend()
        fig.tight_layout();fig.savefig(directory/'comparison.png',dpi=150);plt.close(fig)
        images='<img style="max-width:100%" src="data:image/png;base64,'+base64.b64encode((directory/'comparison.png').read_bytes()).decode()+'">'
        text += ['', '![Comparison](comparison.png)']
        if repeatability:
            import numpy as np
            fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
            plotted = 0
            for trial in [*manifest['trials'], *supplementary_overlays]:
                if not trial.get('supplementary_native_lap') and (trial['status'] != 'completed' or trial.get('report_status') != 'native_flying_lap'):
                    continue
                samples = Path(trial.get('report_directory', str(Path(trial['trial_directory'])/'report')))/'samples.csv'
                if not samples.exists():
                    continue
                with samples.open() as stream:
                    rr = list(csv.DictReader(stream))
                rr = [r for r in rr if r.get('progress_m')]
                if not rr:
                    continue
                length = trial['report_summary']['track_length_m']
                progress = np.unwrap(np.array([float(r['progress_m']) for r in rr])*2*np.pi/length)*length/(2*np.pi)
                progress = 100*(progress-progress[0])/length
                for ax, key, label in zip(axes,
                    ['signed_tracking_error_m', 'front_lateral_peak_ratio', 'vx_mps', 'steering_rad'],
                    ['Signed tracking error (m)', 'Front |slip| / peak', 'Reported speed (m/s)', 'Steering feedback (deg)']):
                    values = np.array([float(r[key]) if r.get(key) else np.nan for r in rr])
                    if key == 'steering_rad': values = np.degrees(values)
                    ax.plot(progress, values, linewidth=1, alpha=.8, label=f"Run {trial['index']}")
                    ax.set_ylabel(label); ax.grid(alpha=.25)
                plotted += 1
            if plotted:
                axes[1].axhline(1, color='red', linestyle='--', label='Documented peak coordinate')
                axes[0].axhline(0, color='.5', linewidth=.7)
                axes[0].legend(); axes[-1].set_xlabel('Mapped circuit progress since native lap start (%)')
                fig.suptitle(f'{plotted} identical-configuration completed flying laps; timing flags retained')
                fig.tight_layout(rect=(0,0,1,.97)); fig.savefig(directory/'repeatability_profiles.png', dpi=150)
                images += '<h2>Flying-lap overlays</h2><img style="max-width:100%" src="data:image/png;base64,'+base64.b64encode((directory/'repeatability_profiles.png').read_bytes()).decode()+'">'
                text += ['', '![Flying-lap overlays](repeatability_profiles.png)']
            plt.close(fig)
    text += ['', 'Native lap times are simulator telemetry, not wall durations. Tire peak thresholds remain unverified model-relative coordinates.',
             'Inspect per-trial frame timing, actual speed, localization logs, and tracking before attributing changes to steering tuning.']
    (directory/'comparison.md').write_text('\n'.join(text)+'\n')
    (directory/'comparison.html').write_text('<!doctype html><meta charset="utf-8"><title>MAP experiment</title>'
        '<style>body{font-family:system-ui;max-width:1200px;margin:30px auto;padding:15px}td,th{padding:10px;border-bottom:1px solid #ddd}table{border-collapse:collapse}</style>'
        '<h1>'+html.escape(title)+'</h1>'+
        ''.join('<p>'+html.escape(t)+'</p>' for t in text[2:9] if t)+table+repeat_html+images+
        '<p>Exploratory comparison: examine timing quality and actual speed, not only threshold crossings.</p>')
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory')
    parser.add_argument('--earlier', action='append', default=[], help='Earlier identical-configuration experiment with extracted native_flying_report')
    args=parser.parse_args()
    compare(args.directory, args.earlier)
