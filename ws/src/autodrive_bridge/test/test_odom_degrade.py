import math

import numpy as np
import pytest

from autodrive_bridge.odom_degrade import OdomDegrader


def test_off_by_default_is_identity():
    d = OdomDegrader()
    assert not d.active
    assert d.step(0.1, 0.02, 0.02) == (0.1, 0.0, 0.02)


def test_scale_bias_scales_distance_and_yaw():
    d = OdomDegrader(scale_bias=0.05, yaw_bias=-0.1)
    ds, side, dyaw = d.step(0.1, 0.02, 0.02)
    assert ds == pytest.approx(0.105)
    assert dyaw == pytest.approx(0.018)
    assert side == 0.0


def test_lateral_slip_moves_odometry_inward():
    # left turn (dyaw > 0) at 5 m/s: the car slides right, so the odometry lands left of it (+y)
    d = OdomDegrader(lateral=0.02)
    _, side, _ = d.step(0.1, 0.04, 0.02)
    assert side == pytest.approx(0.02 * 5.0 * 0.04)
    _, side, _ = d.step(0.1, -0.04, 0.02)
    assert side < 0


def test_drifting_error_has_requested_std_and_correlation():
    d = OdomDegrader(scale_std=0.05, tau=0.5, seed=3)
    dt = 0.02
    e = np.array([d.step(1.0 * dt, 0.0, dt)[0] / dt - 1.0 for _ in range(50000)])
    assert e.std() == pytest.approx(0.05, rel=0.1)
    # slowly varying: consecutive samples nearly equal (an episode of slip, not white noise)
    lag1 = np.corrcoef(e[:-1], e[1:])[0, 1]
    assert lag1 == pytest.approx(math.exp(-dt / 0.5), abs=0.02)


def test_same_seed_same_noise():
    a, b = OdomDegrader(scale_std=0.1, seed=7), OdomDegrader(scale_std=0.1, seed=7)
    assert [a.step(0.1, 0.0, 0.02) for _ in range(20)] == [b.step(0.1, 0.0, 0.02) for _ in range(20)]
    c = OdomDegrader(scale_std=0.1, seed=8)
    assert c.step(0.1, 0.0, 0.02) != OdomDegrader(scale_std=0.1, seed=7).step(0.1, 0.0, 0.02)


def test_no_time_step_passes_through():
    assert OdomDegrader(scale_bias=0.1).step(0.1, 0.0, 0.0) == (0.1, 0.0, 0.0)
