"""Opponent motion and sensor models, checked against closed forms.

None of this needs ROS, which is the point of keeping the models out of the node.
"""
import numpy as np
import pytest

from planning.frenet import FrenetConverter
from planning.opponent_models import (VAR_FLOOR, ConstantSpeed, Replay, SensorModel,
                                      StaticObstacle, Weaving, motion_from_params,
                                      observe, signed_gap)

LAP = 100.0


@pytest.fixture
def rng():
    return np.random.default_rng(0)


@pytest.fixture
def ideal():
    """A perfect oracle: everything in range is reported exactly."""
    return SensorModel(max_range=LAP, rear_range=LAP)


def test_signed_gap_matches_the_converter():
    """signed_gap duplicates FrenetConverter.ds, so it has to agree with it."""
    th = np.linspace(0, 2 * np.pi, 721)[:-1]
    r = LAP / (2 * np.pi)
    conv = FrenetConverter(np.column_stack((r * np.cos(th), r * np.sin(th))))
    for a, b in [(0.0, 1.0), (99.0, 1.0), (1.0, 99.0), (50.0, 50.0), (75.0, 25.0)]:
        assert signed_gap(a, b, conv.length) == pytest.approx(conv.ds(a, b), abs=1e-6)


# ---------------------------------------------------------------- motion

def test_static_never_moves():
    m = StaticObstacle(12.0, -0.3)
    for t in (0.0, 5.0, 1000.0):
        assert (m.state_at(t).s, m.state_at(t).d) == (12.0, -0.3)
        assert (m.state_at(t).vs, m.state_at(t).vd) == (0.0, 0.0)
    assert m.is_static


def test_constant_speed_advances_and_wraps():
    m = ConstantSpeed(LAP, speed=4.0, s0=98.0)
    assert m.state_at(0.0).s == pytest.approx(98.0)
    assert m.state_at(1.0).s == pytest.approx(2.0)      # crossed the line
    assert m.state_at(0.5).vs == pytest.approx(4.0)
    assert not m.is_static


def test_weaving_is_continuous_across_the_start_line():
    """An opponent whose line jumps at s=0 is a discontinuity no driver produces,
    and the GP would be asked to learn it."""
    m = Weaving(LAP, speed=3.0, amplitude=0.4, wavelength=12.0)
    assert LAP / m.wavelength == pytest.approx(round(LAP / m.wavelength))
    eps = 1e-4
    before = m.amplitude * np.sin(2 * np.pi * (LAP - eps) / m.wavelength)
    after = m.amplitude * np.sin(2 * np.pi * eps / m.wavelength)
    assert before == pytest.approx(after, abs=1e-3)


def test_weaving_lateral_velocity_matches_the_derivative():
    m = Weaving(LAP, speed=3.0, amplitude=0.4, wavelength=12.5)
    h = 1e-5
    for t in (0.3, 1.7, 4.2):
        numeric = (m.state_at(t + h).d - m.state_at(t - h).d) / (2 * h)
        assert m.state_at(t).vd == pytest.approx(numeric, rel=1e-3, abs=1e-6)


def test_weaving_line_repeats_lap_to_lap():
    m = Weaving(LAP, speed=5.0, amplitude=0.4, wavelength=10.0)
    lap_time = LAP / 5.0
    for t in (0.0, 3.0, 9.0):
        assert m.state_at(t).d == pytest.approx(m.state_at(t + lap_time).d, abs=1e-6)


def test_replay_follows_its_samples():
    s = np.array([0.0, 25.0, 50.0, 75.0])
    d = np.array([0.0, 0.5, 0.0, -0.5])
    m = Replay(LAP, s, d, speed=1.0)
    assert m.state_at(25.0).d == pytest.approx(0.5, abs=1e-6)
    assert m.state_at(75.0).d == pytest.approx(-0.5, abs=1e-6)
    assert m.state_at(0.0).d == pytest.approx(0.0, abs=1e-6)


def test_replay_rejects_bad_samples():
    with pytest.raises(ValueError):
        Replay(LAP, [0.0], [0.0], speed=1.0)
    with pytest.raises(ValueError):
        Replay(LAP, [0.0, 1.0], [0.0], speed=1.0)


def test_factory_builds_each_mode():
    for mode in ('static', 'constant_speed', 'weaving'):
        assert motion_from_params(mode, LAP).state_at(0.0) is not None
    with pytest.raises(ValueError):
        motion_from_params('teleport', LAP)
    with pytest.raises(ValueError):
        motion_from_params('replay', LAP)       # missing samples


# ---------------------------------------------------------------- sensor

def test_nothing_is_seen_beyond_range(rng):
    m = StaticObstacle(50.0)
    sensor = SensorModel(max_range=10.0)
    assert observe(m, 0.0, 45.0, LAP, sensor, rng) is not None    # 5 m ahead
    assert observe(m, 0.0, 30.0, LAP, sensor, rng) is None        # 20 m ahead


def test_blind_astern_by_default(rng):
    m = StaticObstacle(50.0)
    assert observe(m, 0.0, 55.0, LAP, SensorModel(), rng) is None
    seen = observe(m, 0.0, 55.0, LAP, SensorModel(rear_range=8.0), rng)
    assert seen is not None and seen.s_center == pytest.approx(50.0)


def test_range_gate_uses_lap_distance_not_subtraction(rng):
    """An opponent 2 m past the line is 2 m ahead of a car 2 m before it."""
    m = StaticObstacle(2.0)
    assert observe(m, 0.0, 98.0, LAP, SensorModel(max_range=10.0), rng) is not None


def test_dropout_rate_is_honoured(rng):
    m = StaticObstacle(50.0)
    sensor = SensorModel(max_range=10.0, detection_prob=0.5)
    seen = sum(observe(m, 0.0, 45.0, LAP, sensor, rng) is not None for _ in range(4000))
    assert 0.46 < seen / 4000 < 0.54


def test_noiseless_observation_still_reports_positive_variance(rng, ideal):
    """Consumers divide by these, so zero is not an allowed answer."""
    o = observe(StaticObstacle(50.0), 0.0, 45.0, LAP, ideal, rng)
    assert o.s_var == VAR_FLOOR and o.d_var == VAR_FLOOR
    assert o.vs_var == VAR_FLOOR and o.vd_var == VAR_FLOOR


def test_reported_variance_matches_the_noise_applied(rng):
    sensor = SensorModel(max_range=20.0, sigma_s=0.05, sigma_d=0.02)
    o = observe(StaticObstacle(50.0), 0.0, 45.0, LAP, sensor, rng)
    assert o.s_var == pytest.approx(0.05 ** 2)
    assert o.d_var == pytest.approx(0.02 ** 2)


def test_noise_is_actually_applied(rng):
    sensor = SensorModel(max_range=20.0, sigma_d=0.1)
    m = StaticObstacle(50.0, 0.0)
    ds = [observe(m, 0.0, 45.0, LAP, sensor, rng).d_center for _ in range(2000)]
    assert np.std(ds) == pytest.approx(0.1, rel=0.15)
    assert np.mean(ds) == pytest.approx(0.0, abs=0.02)


def test_footprint_brackets_the_centre(rng, ideal):
    o = observe(StaticObstacle(50.0, 0.2), 0.0, 45.0, LAP, ideal, rng)
    half_l = ideal.length / 2 + ideal.inflation
    half_w = ideal.width / 2 + ideal.inflation
    assert o.s_start == pytest.approx(50.0 - half_l)
    assert o.s_end == pytest.approx(50.0 + half_l)
    assert o.d_right == pytest.approx(0.2 - half_w)
    assert o.d_left == pytest.approx(0.2 + half_w)


def test_footprint_stays_ordered_across_the_start_line(rng, ideal):
    """s_end runs past the lap length rather than wrapping, because consumers
    sort and take min/max over these as plain numbers."""
    o = observe(StaticObstacle(0.05), 0.0, 0.0, LAP, ideal, rng)
    assert o.s_start < o.s_end
    assert o.s_start > LAP - 1.0
    assert o.s_end > LAP
    assert 0.0 <= o.s_center < LAP


def test_latency_reports_the_past(rng):
    m = ConstantSpeed(LAP, speed=4.0, s0=0.0)
    sensor = SensorModel(max_range=50.0, latency=0.5)
    o = observe(m, 10.0, 38.0, LAP, sensor, rng)
    assert o.s_center == pytest.approx(4.0 * 9.5)     # where it was, not where it is


def test_static_flag_survives_observation(rng, ideal):
    assert observe(StaticObstacle(50.0), 0.0, 45.0, LAP, ideal, rng).is_static
    assert not observe(ConstantSpeed(LAP, 2.0, 50.0), 0.0, 45.0, LAP, ideal, rng).is_static


def test_same_seed_gives_the_same_run():
    """Two planners must face identical noise or the comparison measures the RNG."""
    sensor = SensorModel(max_range=20.0, sigma_d=0.1, detection_prob=0.8)
    m = ConstantSpeed(LAP, speed=3.0)

    def run(seed):
        r = np.random.default_rng(seed)
        return [observe(m, t, 0.0, LAP, sensor, r) for t in np.linspace(0, 2, 50)]

    a, b = run(7), run(7)
    assert [x is None for x in a] == [x is None for x in b]
    assert all(x.d_center == y.d_center for x, y in zip(a, b) if x is not None)
    assert run(7) != run(8)


def test_observation_matches_the_message_definition(rng, ideal):
    """The dataclass exists to be copied field for field into f1t_msgs/Obstacle,
    which is itself copied from ForzaETH. If anyone adds a field on one side
    only, the port silently stops matching upstream, so check it here."""
    import dataclasses
    import pathlib
    import re

    msg = (pathlib.Path(__file__).resolve().parents[2]
           / 'f1t_msgs' / 'msg' / 'Obstacle.msg')
    if not msg.exists():
        pytest.skip('f1t_msgs not alongside planning')
    declared = re.findall(r'^(?:int32|float64|bool)\s+(\w+)', msg.read_text(), re.M)
    o = observe(StaticObstacle(50.0), 0.0, 45.0, LAP, ideal, rng)
    assert [f.name for f in dataclasses.fields(o)] == declared
