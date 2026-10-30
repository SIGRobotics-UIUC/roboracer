"""Unit tests for control/lookahead.py (no ROS needed): python3 -m pytest ws/src/control/test/test_lookahead.py"""
import numpy as np
import pytest

from control.lookahead import curvature_cap, lookahead, path_curvature

TIME, LMIN, LMAX = 0.4, 0.6, 3.0
CUT, SLOPE = 0.1, 0.5


def circle(radius, step=0.1):
    t = np.linspace(0, 2 * np.pi, int(round(2 * np.pi * radius / step)), endpoint=False)
    return np.column_stack([radius * np.cos(t), radius * np.sin(t)])


def stadium(length, radius, step=0.1):
    """Closed loop: straight along y = 0, half circle of `radius`, straight back, half circle."""
    x = np.arange(0.0, length, step)
    t = np.arange(-np.pi / 2, np.pi / 2, step / radius)
    return np.concatenate([np.column_stack([x, np.zeros_like(x)]),
                           np.column_stack([length + radius * np.cos(t), radius + radius * np.sin(t)]),
                           np.column_stack([length - x, np.full_like(x, 2 * radius)]),
                           np.column_stack([-radius * np.cos(t), radius - radius * np.sin(t)])])


def cap_of(xy, **kw):
    args = dict(cut=CUT, slope=SLOPE, lookahead_min=LMIN, lookahead_distance=LMAX, smooth=0.3)
    args.update(kw)
    return curvature_cap(xy, **args)


def test_off_is_the_speed_term():
    for v in (0.0, 1.0, 2.5, 4.0, 6.0, 20.0):
        assert lookahead(v, TIME, LMIN, LMAX) == pytest.approx(np.clip(TIME * v, LMIN, LMAX))
        assert lookahead(v, TIME, LMIN, LMAX, np.inf, 0.25) == pytest.approx(np.clip(TIME * v, LMIN, LMAX))


def test_curvature_of_a_circle():
    assert path_curvature(circle(2.0)) == pytest.approx(np.full(len(circle(2.0)), 0.5), rel=1e-3)


def test_straight_unchanged():
    cap = cap_of(stadium(40.0, 1.0))
    mid = 200   # 20 m into the 40 m straight: curves > lookahead_distance / slope = 6 m away
    assert np.isinf(cap[mid])
    assert lookahead(6.0, TIME, LMIN, LMAX, cap[mid], 0.25) == pytest.approx(2.4)


def test_curve_capped():
    cap = cap_of(circle(5.0))  # sqrt(2 x 0.1 x 5) = 1.0 m
    assert cap == pytest.approx(np.full(len(cap), 1.0), rel=1e-3)
    assert lookahead(5.0, TIME, LMIN, LMAX, cap[0]) == pytest.approx(1.0, rel=1e-3)   # 2.0 m uncapped
    assert lookahead(5.0, TIME, LMIN, LMAX, cap[0], 0.25) == pytest.approx(1.25)     # min preview 0.25 s


def test_tight_corner_capped():
    r = 1.0
    cap = cap_of(stadium(20.0, r))
    reach = np.sqrt(2 * CUT * r)  # 0.45 m: below lookahead_min
    in_corner = 200 + 10   # 1 m into the first half circle
    assert cap[in_corner] == pytest.approx(LMIN)
    assert lookahead(3.0, TIME, LMIN, LMAX, cap[in_corner]) == pytest.approx(LMIN)
    # 1 m before it: about reach + slope * 1 m (a little more: the smoothed curvature ramps up into it)
    before = 200 - 10
    assert reach + SLOPE * 1.0 <= cap[before] <= reach + SLOPE * 1.4
    assert lookahead(6.0, TIME, LMIN, LMAX, cap[before]) < 2.4
    # no steps where it matters (below lookahead_distance): slope m per m driven at most
    assert np.all(np.abs(np.diff(np.minimum(cap, LMAX))) <= SLOPE * 0.1 + 1e-6)


def test_cut_scales_reach():
    xy = circle(2.0)
    assert cap_of(xy, cut=0.25) == pytest.approx(np.full(len(xy), 1.0), rel=1e-3)   # sqrt(2 x 0.25 x 2)
    assert cap_of(circle(0.4)) == pytest.approx(np.full(len(circle(0.4)), LMIN))     # floor


def test_slope_zero_is_a_window_max():
    cap = cap_of(stadium(20.0, 1.0), slope=0.0)
    assert cap[200 - 25] == pytest.approx(LMIN)   # corner 2.5 m ahead, within lookahead_distance
    assert np.isinf(cap[200 - 40])                # 4 m ahead, beyond it
    assert np.isinf(cap[231 + 40])                # 4 m after it


def test_corner_across_the_start():
    xy = stadium(20.0, 1.0)
    cap = cap_of(xy)
    shift = 215   # index 0 in the middle of the first half circle
    rolled = cap_of(np.roll(xy, -shift, axis=0))
    # same caps below lookahead_distance (above it they do not matter, and window-edge ties may differ)
    assert np.minimum(rolled, LMAX) == pytest.approx(np.roll(np.minimum(cap, LMAX), -shift))
    assert rolled[0] == pytest.approx(LMIN)


def test_repeated_points_are_skipped():
    xy = stadium(40.0, 1.0)
    dup = np.insert(xy, 200, xy[200], axis=0)   # a repeated point mid-straight
    assert np.max(path_curvature(dup)[195:206]) < 1e-9
    cap = cap_of(dup)
    assert np.all(np.isinf(cap[150:250]))
    assert np.delete(cap, 200) == pytest.approx(cap_of(xy))


def test_non_uniform_spacing():
    # the last 3 m of the straight sampled every 0.01 m, the rest every 0.1 m: curves are found by arc length
    r, length = 1.0, 20.0
    fine = stadium(length, r)
    x = np.concatenate([np.arange(0.0, length - 3.0, 0.1), np.arange(length - 3.0, length, 0.01)])
    t = np.arange(-np.pi / 2, np.pi / 2, 0.1 / r)
    xy = np.concatenate([np.column_stack([x, np.zeros_like(x)]),
                         np.column_stack([length + r * np.cos(t), r + r * np.sin(t)]),
                         fine[len(np.arange(0.0, length, 0.1)) + len(t):]])
    cap = cap_of(xy)
    i = np.argmin(np.abs(x - (length - 3.0)))   # 3 m before the corner
    j = 200 - 30                               # the same place on the uniform line
    assert cap[i] == pytest.approx(cap_of(fine)[j], abs=0.03)
    assert np.sqrt(2 * CUT * r) + SLOPE * 3.0 <= cap[i] <= np.sqrt(2 * CUT * r) + SLOPE * 3.0 + 0.2


def test_smoothing_longer_than_the_lap():
    xy = circle(0.32)   # a 2 m loop
    assert path_curvature(xy, smooth=5.0) == pytest.approx(np.full(len(xy), 1 / 0.32), rel=1e-2)  # chords
    assert np.all(cap_of(xy, smooth=5.0) == pytest.approx(LMIN))


def test_tiny_slope_is_bounded():
    xy = stadium(20.0, 1.0)
    cap = cap_of(xy, slope=1e-9)   # window clamped to half a lap: no huge arrays
    assert np.all(cap == pytest.approx(LMIN))
