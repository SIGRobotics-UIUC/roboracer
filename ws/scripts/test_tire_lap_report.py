"""Synthetic checks of slip geometry, missing-data handling, and lap selection."""
import numpy as np
import pytest

from tire_lap_report import (Settings, build_rows, longitudinal_slips, select_lap,
                             summarize, track_progress, wheel_lateral_slips)


def straight_data():
    stamps = [i*50_000_000 for i in range(8)]
    gt = {t: (t+1_000_000, i*.1, 0., 0., 2., 0., 0.) for i, t in enumerate(stamps)}
    enc = {side: {t: i*.1/.0585 for i, t in enumerate(stamps)} for side in ('left', 'right')}
    steering = [(t+500_000, 0.) for t in stamps]
    return gt, enc, steering, [(0, 2.)]


def test_ackermann_turn_has_zero_kinematic_slip_at_all_wheels():
    cfg = Settings(velocity_reference_x=0.)
    vx, rate = 2., 1.
    steer = np.arctan(cfg.wheelbase*rate/vx)
    slips = wheel_lateral_slips(vx, 0., rate, steer, cfg)
    for alpha, slip, forward in slips.values():
        assert alpha == pytest.approx(0., abs=1e-12)
        assert slip == pytest.approx(0., abs=1e-12)
        assert forward > 0
    ds, turn = .1, .05
    angles = np.array([ds-cfg.track_width/2*turn, ds+cfg.track_width/2*turn])/cfg.wheel_radius
    assert longitudinal_slips(ds, turn, angles, cfg) == pytest.approx([0., 0.], abs=1e-12)


def test_front_rear_lateral_slips_differ_and_reference_offset_is_respected():
    cfg = Settings(velocity_reference_x=.08)
    slips = wheel_lateral_slips(2., .08, 1., 0., cfg)
    assert slips['rear_left'][1] == pytest.approx(0.)
    assert slips['front_left'][1] > .1


def test_distance_slip_independent_of_wall_time_and_peak_not_force():
    cfg = Settings()
    for factor in (1, 2):
        gt, enc, steer, commands = straight_data()
        gt = {t*factor: (v[0]*factor, *v[1:]) for t, v in gt.items()}
        enc = {side: {t*factor: a*1.15 for t, a in values.items()} for side, values in enc.items()}
        steer = [(t*factor, a) for t, a in steer]
        rows = build_rows(gt, enc, steer, commands, cfg)
        assert all(r['rear_longitudinal_peak_ratio'] == pytest.approx(1.) for r in rows)
        assert all(r['front_lateral_peak_ratio'] == pytest.approx(0.) for r in rows)


def test_missing_sensors_are_unavailable_not_zero():
    gt, enc, steer, commands = straight_data()
    rows = build_rows(gt, enc, [], commands, Settings())
    assert all(r['front_lateral_peak_ratio'] is None for r in rows)
    assert all(r['longitudinal_valid'] for r in rows)
    rows = build_rows(gt, {'left': {}, 'right': {}}, steer, commands, Settings())
    assert all(r['rear_longitudinal_peak_ratio'] is None for r in rows)
    assert all(r['lateral_valid'] for r in rows)
    assert summarize(rows)['rear_longitudinal_peak_ratio']['median'] is None


def test_frame_gap_breaks_segments_and_invalidates_estimates():
    gt, enc, steer, commands = straight_data()
    t, v = list(gt.items())[-1]
    gt[t+1_000_000_000] = gt.pop(t)
    rows = build_rows(gt, enc, steer, commands, Settings())
    assert rows[-1]['segment'] != rows[-2]['segment']
    assert rows[-1]['front_lateral_peak_ratio'] is None
    assert rows[-1]['rear_longitudinal_peak_ratio'] is None


def test_complete_lap_selection_and_no_cross_discontinuity_lap():
    rows = [dict(segment=0, moving=True, unwrapped_progress_m=float(s)) for s in range(5, 32)]
    selected, info = select_lap(rows, 10., 'auto')
    assert info['full_lap']
    assert selected[0]['unwrapped_progress_m'] == 5
    assert selected[-1]['unwrapped_progress_m'] == 15
    split = [dict(r, segment=int(r['unwrapped_progress_m']//10)) for r in rows]
    _, info = select_lap(split, 10., 'auto')
    assert not info['full_lap']
    with pytest.raises(ValueError):
        select_lap(split, 10., '1')


def test_track_progress_jump_starts_persistent_new_segment():
    path = np.array([[0., 0.], [10., 0.], [10., 10.], [0., 10.]])
    rows = [dict(segment=0, x_m=x, y_m=0., yaw_rad=0., distance_step_m=.1, flags='') for x in (1., 1.1, 8., 8.1)]
    track_progress(rows, path, dict(x_m=0., y_m=0., yaw_rad=0.))
    assert rows[0]['segment'] == rows[1]['segment']
    assert rows[2]['segment'] == rows[3]['segment']
    assert rows[1]['segment'] != rows[2]['segment']


def test_turn_report_transforms_center_of_mass_velocity_to_wheel_locations():
    cfg = Settings()
    gt, enc, steering = {}, {'left': {}, 'right': {}}, []
    steer = np.arctan(cfg.wheelbase/2)
    for i in range(20):
        t, yaw = i*50_000_000, i*.05
        gt[t] = (t+1_000_000, 2*np.sin(yaw), 2*(1-np.cos(yaw)), yaw,
                 2., cfg.velocity_reference_x, 1.)
        steering.append((t+1_000_000, steer))
        for side, y in [('left', cfg.track_width/2), ('right', -cfg.track_width/2)]:
            enc[side][t] = (2-y)*yaw/cfg.wheel_radius
    rows = build_rows(gt, enc, steering, [(0, 2.)], cfg)
    for row in rows:
        assert row['velocity_reference_x_inferred_m'] == pytest.approx(cfg.velocity_reference_x)
        for metric in ('front_lateral_peak_ratio', 'rear_lateral_peak_ratio', 'rear_longitudinal_peak_ratio'):
            assert row[metric] == pytest.approx(0., abs=1e-10)
    wrong_reference_rows = build_rows(gt, enc, steering, [(0, 2.)], Settings(velocity_reference_x=0.))
    assert all(r['front_lateral_peak_ratio'] is None for r in wrong_reference_rows)
    assert all('velocity_reference_inconsistent' in r['flags'] for r in wrong_reference_rows)


def test_crossing_persistence_requires_duration_and_consecutive_valid_samples():
    from tire_lap_report import METRICS, crossing_events
    def row(i, value, dt=.1, segment=0):
        return dict(stamp_ns=round((i+1)*.1*1e9), dt_wall_s=dt, segment=segment,
                    steering_rate_wall_dps=50., **{metric: value for metric in METRICS})
    rows = [row(0, .5), row(1, 2.), row(2, 2.), row(3, .5),
            row(4, 1.1), row(5, 1.1), row(6, 1.1), row(7, None),
            row(8, 1.1, dt=.4), row(9, 1.1, segment=1)]
    events = crossing_events(rows, Settings())
    front = [e for e in events if e['metric'] == METRICS[0]]
    assert [e['classification'] for e in front] == ['brief', 'sustained', 'timing_uncertain', 'brief']
    assert front[0]['duration_wall_s'] == pytest.approx(.2)
    assert front[1]['intervals'] == 3
    assert front[1]['max_abs_steering_rate_wall_dps'] == 50.
    assert rows[7]['front_lateral_crossing'] == 'unavailable'
    assert rows[0]['front_lateral_crossing'] == 'below_peak'
    assert rows[4]['front_lateral_crossing'] == 'sustained'
    # Raising the minimum sample count must demote the three-sample run.
    events = crossing_events(rows, Settings(sustained_crossing_min_intervals=4))
    assert not any(e['classification'] == 'sustained' for e in events)


def test_tracking_error_sign_heading_and_distance_weighted_rms():
    from tire_lap_report import tracking_summary
    path = np.array([[0., 0.], [10., 0.], [10., 10.], [0., 10.]])
    rows = [dict(segment=0, x_m=x, y_m=y, yaw_rad=.1, distance_step_m=w, flags='')
            for x, y, w in [(2., .2, 1.), (3., -.4, 3.)]]
    track_progress(rows, path, dict(x_m=0., y_m=0., yaw_rad=0.))
    assert rows[0]['signed_tracking_error_m'] == pytest.approx(.2)
    assert rows[1]['signed_tracking_error_m'] == pytest.approx(-.4)
    summary = tracking_summary(rows)
    assert summary['distance_weighted_rms_m'] == pytest.approx(np.sqrt((.2**2+3*.4**2)/4))
    assert summary['mean_signed_m'] == pytest.approx(-.25)
    assert summary['p95_abs_heading_deg'] == pytest.approx(np.degrees(.1))


def test_native_flying_lap_selection_excludes_warmup_and_preserves_gap_warning():
    from tire_lap_report import select_native_lap
    rows = [dict(received_ns=t, segment=int(t >= 60)) for t in range(0, 120, 10)]
    selected, info = select_native_lap(rows, [(0, 0), (20, 1), (50, 1), (100, 2)], 2)
    assert [r['received_ns'] for r in selected] == list(range(20, 100, 10))
    assert info['full_lap'] and not info['continuous_geometry']
    with pytest.raises(ValueError):
        select_native_lap(rows, [(0, 0), (20, 2), (100, 3)], 2)
    with pytest.raises(ValueError):
        select_native_lap(rows, [(0, 0), (20, 1)], 2)
