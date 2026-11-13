import pytest
from wheel_slip_comparison import attach, summary, WHEELS


def test_exact_interval_pairing_and_airborne_missing():
    rows = [dict(stamp_ns=20, interval_start_stamp_ns=10, dt_wall_s=1e-8,
                 front_left_lateral_slip=.3)]
    sample = lambda grounded,side: {w: dict(grounded=grounded,forward=.1,sideways=side) for w in WHEELS}
    attach(rows,{10:sample(True,.2),20:sample(True,.4)})
    assert rows[0]['front_left_native_mean_lateral_slip'] == pytest.approx(.3)
    assert summary(rows)['metrics']['front_left_lateral']['signed_rmse'] == pytest.approx(0)
    attach(rows,{10:sample(False,.2),20:sample(True,.4)})
    assert rows[0]['front_left_native_mean_lateral_slip'] is None
    attach(rows,{19:sample(True,.4)})
    assert rows[0]['front_left_native_grounded'] is None


def test_primary_uses_instantaneous_native_not_interval_mean_or_kinematic():
    from tire_lap_report import Settings, crossing_events
    from wheel_slip_comparison import select_primary
    row = dict(stamp_ns=20, interval_start_stamp_ns=10, dt_wall_s=.05, segment=0,
               front_lateral_peak_ratio=99., rear_lateral_peak_ratio=99., rear_longitudinal_peak_ratio=99.)
    sample = lambda side: {w: dict(grounded=True,forward=.3,sideways=side) for w in WHEELS}
    attach([row], {10:sample(0.),20:sample(.012)})
    assert select_primary([row], Settings()) == 'unity'
    assert row['front_lateral_peak_ratio'] == pytest.approx(1.2)
    assert row['kinematic_front_lateral_peak_ratio'] == 99.
    assert row['front_longitudinal_peak_ratio'] == pytest.approx(2.)
    assert crossing_events([row], Settings())[0]['peak_ratio'] == pytest.approx(1.2)


def test_native_missing_or_airborne_does_not_use_estimate():
    from tire_lap_report import Settings
    from wheel_slip_comparison import select_primary
    row = dict(stamp_ns=20, dt_wall_s=.05, front_lateral_peak_ratio=99.)
    samples={20:{w:dict(grounded=w!='front_left',forward=0.,sideways=.012) for w in WHEELS}}
    attach([row],samples)
    assert select_primary([row], Settings()) == 'unity'
    assert row['front_lateral_peak_ratio'] is None
    with pytest.raises(ValueError,match='no native telemetry'):
        select_primary([{}],Settings(),'unity')
    assert select_primary([dict(front_lateral_peak_ratio=99.)],Settings()) == 'kinematic'
