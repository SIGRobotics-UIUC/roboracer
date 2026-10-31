import json

from compare_map_experiment import compare


def test_comparison_excludes_aborted_trials_from_lap_time_group(tmp_path):
    report = dict(tracking_error=dict(distance_weighted_rms_m=.15, p95_abs_m=.3, p95_abs_heading_deg=5.),
                  metrics=dict(front_lateral_peak_ratio=dict(beyond_peak_distance_pct=2.)),
                  crossing_counts=dict(front_lateral_peak_ratio=dict(brief=3, sustained=0)),
                  long_frame_intervals=1)
    trials = [dict(index=i, steering_rate_limit_radps=3.2, status=status,
                   mean_flying_lap_s=lap_time, report_status='second_geometric_lap',
                   report_summary=report, trial_directory=str(tmp_path/f'trial_{i}'))
              for i, status, lap_time in [(1, 'completed', 10.), (2, 'aborted', 8.)]]
    trials.append(dict(trials[0], index=3, mean_flying_lap_s=7., clock_discontinuities=[{'ros_interval_s': 2.4, 'monotonic_interval_s': .07}]))
    manifest = dict(status='stopped_after_incomplete_trial', speed_scale=.5,
                    rates_radps=[3.2, 3.2], trials=trials)
    (tmp_path/'manifest.json').write_text(json.dumps(manifest))
    compare(tmp_path)
    result = json.loads((tmp_path/'comparison.json').read_text())
    assert result['groups'][0]['comparable_trials'] == 1
    assert result['groups'][0]['mean_trial_lap_time_s'] == 10.
    assert result['trials'][1]['comparable'] is False
    assert result['trials'][2]['comparable'] is False
    assert 'trial_1/report/report.html' in (tmp_path/'comparison.html').read_text()


def test_repeatability_retains_clock_flagged_completed_laps_and_counts_failures(tmp_path):
    report = dict(tracking_error=dict(distance_weighted_rms_m=.1, p95_abs_m=.2, p95_abs_heading_deg=3.),
                  metrics=dict(front_lateral_peak_ratio=dict(beyond_peak_distance_pct=2., p95=.8, maximum=2.)),
                  crossing_counts=dict(front_lateral_peak_ratio=dict(brief=3, sustained=0)), long_frame_intervals=1)
    trials = [dict(index=i, steering_rate_limit_radps=3.2, status=status,
                   mean_flying_lap_s=lap, report_status='native_flying_lap', report_summary=report,
                   trial_directory=str(tmp_path/f'trial_{i}'), collision_change=int(status=='aborted'),
                   clock_discontinuities=[dict(ros_interval_s=2., monotonic_interval_s=.07)] if i==2 else [])
              for i, status, lap in [(1, 'completed', 10.), (2, 'completed', 12.), (3, 'aborted', None)]]
    (tmp_path/'manifest.json').write_text(json.dumps(dict(status='completed_with_failures', mode='repeatability',
        speed_scale=.5, rates_radps=[3.2]*3, trials=trials)))
    compare(tmp_path)
    result = json.loads((tmp_path/'comparison.json').read_text())
    reliability = result['repeatability']
    assert reliability['attempted_runs'] == 3
    assert reliability['completed_runs'] == 2
    assert reliability['runs_with_collision'] == 1
    assert reliability['clean_comparison_runs'] == 1
    assert reliability['metrics']['flying_lap_mean_s']['n'] == 2
    assert reliability['metrics']['flying_lap_mean_s']['mean'] == 11.
    assert result['groups'][0]['mean_trial_lap_time_s'] == 10.
