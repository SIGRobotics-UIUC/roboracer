"""Frenet conversion against an analytic circle and a generated raceline."""
import os

import numpy as np
import pytest

from planning.frenet import FrenetConverter

R = 10.0


@pytest.fixture(scope='module')
def circle():
    """Anticlockwise circle of radius R: every quantity here is known in closed form."""
    th = np.linspace(0, 2 * np.pi, 721)[:-1]
    return FrenetConverter(np.column_stack((R * np.cos(th), R * np.sin(th))))


def test_lap_length(circle):
    assert circle.length == pytest.approx(2 * np.pi * R, rel=1e-4)


def test_points_on_the_line_have_zero_offset(circle):
    th = np.linspace(0, 2 * np.pi, 37)[:-1]
    s, d = circle.to_frenet(R * np.cos(th), R * np.sin(th))
    assert np.allclose(d, 0, atol=1e-3)
    assert np.allclose(s, R * th, atol=1e-2)


def test_offset_sign_is_left_of_travel(circle):
    """Travelling anticlockwise, the inside of the circle is on the left, so d > 0 there."""
    _, d_inside = circle.to_frenet(R - 2.0, 0.0)
    _, d_outside = circle.to_frenet(R + 2.0, 0.0)
    assert d_inside == pytest.approx(2.0, abs=1e-2)
    assert d_outside == pytest.approx(-2.0, abs=1e-2)


def test_roundtrip_through_cartesian(circle):
    """s returns with exactly the discretisation error the class docstring
    predicts, |d| * spacing / (2R), because a point offset toward the inside of
    a curve is genuinely nearer the neighbouring chord. Asserting the law rather
    than a round number means a real regression cannot hide under a loose
    tolerance. The lateral coordinate has no such term and round-trips tighter
    by two orders of magnitude."""
    s = np.linspace(0, circle.length, 50, endpoint=False)
    d = np.linspace(-3, 3, 50)
    x, y = circle.to_cartesian(s, d)
    s2, d2 = circle.to_frenet(x, y)
    spacing = circle.length / len(circle.xy)
    assert np.all(np.abs(circle.ds(s, s2)) <= np.abs(d) * spacing / (2 * R) + 1e-3)
    assert np.allclose(d2, d, atol=1e-3)


def test_s_wraps(circle):
    x0, y0 = circle.to_cartesian(0.5)
    x1, y1 = circle.to_cartesian(0.5 + circle.length)
    assert (x0, y0) == pytest.approx((x1, y1), abs=1e-6)


def test_signed_lap_distance_crosses_the_line(circle):
    """A car 1 m past the start is ahead of one 1 m before it, not a lap behind."""
    behind, ahead = circle.length - 1.0, 1.0
    assert circle.ds(behind, ahead) == pytest.approx(2.0, abs=1e-6)
    assert circle.ds(ahead, behind) == pytest.approx(-2.0, abs=1e-6)
    assert circle.is_ahead(behind, ahead)
    assert not circle.is_ahead(ahead, behind)


def test_heading_is_tangent(circle):
    """On an anticlockwise circle the heading leads the polar angle by 90 degrees."""
    for th in (0.0, 1.0, 3.0, 6.0):
        psi = circle.heading_at(R * th)
        expect = th + np.pi / 2
        assert np.cos(psi - expect) == pytest.approx(1.0, abs=1e-3)


def test_scalar_in_scalar_out(circle):
    s, d = circle.to_frenet(R + 1.0, 0.0)
    assert np.isscalar(s) or s.shape == ()
    assert float(d) == pytest.approx(-1.0, abs=1e-2)


def test_rejects_degenerate_reference():
    with pytest.raises(ValueError):
        FrenetConverter([[0, 0], [1, 1]])
    with pytest.raises(ValueError):
        FrenetConverter([[0, 0], [0, 0], [1, 0], [1, 1]])


R_T, W_LEFT, W_RIGHT = 10.0, 1.0, 1.5


@pytest.fixture(scope='module')
def track(tmp_path_factory):
    """A circular track written out as raceline.csv + centerline.csv, as build_raceline does."""
    d = tmp_path_factory.mktemp('track')
    th = np.linspace(0, 2 * np.pi, 600, endpoint=False)
    xy = np.column_stack((R_T * np.cos(th), R_T * np.sin(th)))
    np.savetxt(d / 'centerline.csv',
               np.column_stack((xy, np.full(len(th), W_RIGHT), np.full(len(th), W_LEFT))),
               delimiter=',', header='x_m,y_m,w_tr_right_m,w_tr_left_m')
    s_col = np.arange(len(th)) * (2 * np.pi * R_T / len(th))
    np.savetxt(d / 'raceline.csv',
               np.column_stack((s_col, xy[:, 0], xy[:, 1], th + np.pi / 2,
                                np.full(len(th), 1 / R_T), np.full(len(th), 5.0),
                                np.zeros(len(th)))),
               delimiter=';', header='s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2')
    return FrenetConverter.from_track(str(d))


def test_track_bounds_recover_the_widths(track):
    """Travelling anticlockwise the inside is on the left, so the left edge sits at +w_left."""
    hi, lo = track.width_at(np.linspace(0, track.length, 40, endpoint=False))
    assert hi == pytest.approx(W_LEFT, abs=2e-2)
    assert lo == pytest.approx(-W_RIGHT, abs=2e-2)


def test_is_on_track(track):
    s = np.linspace(0, track.length, 20, endpoint=False)
    assert np.all(track.is_on_track(s, 0.5))
    assert not np.any(track.is_on_track(s, W_LEFT + 0.4))
    assert not np.any(track.is_on_track(s, -W_RIGHT - 0.4))
    assert not track.is_on_track(5.0, W_LEFT - 0.1, margin=0.3)


def test_speed_at(track):
    assert track.speed_at(12.3) == pytest.approx(5.0)


def test_velocity_tangential_and_lateral(track):
    psi = track.heading_at(10.0)
    s_dot, d_dot = track.velocity_to_frenet(10.0, 0.0, 4 * np.cos(psi), 4 * np.sin(psi))
    assert (s_dot, d_dot) == pytest.approx((4.0, 0.0), abs=1e-6)
    s_dot, d_dot = track.velocity_to_frenet(10.0, 0.0, -2 * np.sin(psi), 2 * np.cos(psi))
    assert (s_dot, d_dot) == pytest.approx((0.0, 2.0), abs=1e-6)


def test_progress_rate_is_corrected_for_curvature(track):
    """Running wide covers more ground per unit of s, so s_dot falls; the inside gains."""
    psi = track.heading_at(10.0)
    v = (4 * np.cos(psi), 4 * np.sin(psi))
    outside, _ = track.velocity_to_frenet(10.0, -1.0, *v)
    inside, _ = track.velocity_to_frenet(10.0, 1.0, *v)
    assert outside < 4.0 < inside
    assert outside == pytest.approx(4.0 / (1 + 1 / R_T))


def test_width_without_bounds_raises(circle):
    with pytest.raises(ValueError):
        circle.width_at(1.0)


FIX = os.path.join(os.path.dirname(__file__), 'fixtures')


@pytest.mark.skipif(not os.path.exists(os.path.join(FIX, 'raceline.csv')),
                    reason='no raceline fixture')
def test_from_raceline_matches_its_own_s_column():
    conv = FrenetConverter.from_raceline(os.path.join(FIX, 'raceline.csv'))
    s, d = conv.to_frenet(conv.xy[:, 0], conv.xy[:, 1])
    assert np.allclose(d, 0, atol=5e-3)
    assert np.allclose(conv.ds(conv.s, s), 0, atol=5e-3)
    assert conv.curvature_at(conv.s) == pytest.approx(conv.kappa, abs=1e-6)
