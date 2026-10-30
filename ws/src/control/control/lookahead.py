"""Pure pursuit lookahead: speed-scaled, optionally capped by the curvature of the raceline ahead.

Speed term (always):
    L = clip(lookahead_time * speed, lookahead_min, lookahead_distance)

Curvature cap (adaptive_lookahead): pure pursuit steers on the chord to its target, so on a curve of
curvature kappa it settles inside the line, roughly L^2 * kappa / 2 deep (on the car at most about 0.6 of
that), and at a corner entry the target is already deep in the turn: a long lookahead cuts corners. So
L is capped where that cut would exceed `cut` m. At every raceline point i, precomputed once:

    cap_i = max(lookahead_min, min over points j of (sqrt(2 * cut / |kappa_j|) + slope * |d_ij|))

kappa_j: the line's curvature, from its geometry (heading change per metre), averaged over `smooth` m
so single-point noise (and short wiggles, whose signed turns cancel) does not count. d_ij: arc length
from i to j (ahead or behind). On a curve of radius R the lookahead is at most sqrt(2 * cut * R);
slope > 0 lets it shrink gradually as the curve comes closer (at most slope m per m driven) and grow
back the same way after it, so it never steps or chatters (j is searched by arc
length up to lookahead_distance / slope m either way, at most half a lap, beyond which the bound exceeds
lookahead_distance anyway);
slope = 0 is a plain max of |kappa| over the next and last lookahead_distance m. On a straight
(kappa = 0) the cap is infinite and L is the speed term unchanged.

The cap never goes below min_time * speed (a minimum preview, as a too short lookahead at speed makes
pure pursuit oscillate):

    L = min(clip(lookahead_time * speed, lookahead_min, lookahead_distance), max(cap_i, min_time * speed))
"""
import numpy as np


def _distinct(xy):
    """Drop repeated points (a zero-length segment has no heading): the kept points and, for every
    original point, the index of the kept point it maps to (a repeat maps to the next distinct one)."""
    xy = np.asarray(xy, dtype=float)
    n = len(xy)
    seg_len = np.hypot(*(np.roll(xy, -1, axis=0) - xy).T) if n else np.zeros(0)
    kept = np.flatnonzero(seg_len > 1e-6)
    if not kept.size:
        return xy[:0], np.zeros(n, dtype=int)
    return xy[kept], np.searchsorted(kept, np.arange(n)) % kept.size


def _geometry(xy):
    """Arc length of each point and of the lap, and the turn at each point, for distinct points."""
    seg = np.roll(xy, -1, axis=0) - xy
    seg_len = np.hypot(seg[:, 0], seg[:, 1])
    heading = np.arctan2(seg[:, 1], seg[:, 0])
    dpsi = np.angle(np.exp(1j * (heading - np.roll(heading, 1))))   # turn at each point, wrapped
    s = np.concatenate([[0.0], np.cumsum(seg_len)[:-1]])
    return s, seg_len.sum(), dpsi, seg_len


def _curvature(xy, smooth):
    s, lap, dpsi, seg_len = _geometry(xy)
    half = min(0.5 * smooth, 0.5 * lap)
    if half <= 0:
        return np.abs(dpsi) / np.maximum(0.5 * (seg_len + np.roll(seg_len, 1)), 1e-9)
    # mean curvature over [s - half, s + half] = turn over it / its length, from the cumulative turn
    # (piecewise linear in arc length, over three laps so the window can wrap around the start)
    s3 = np.concatenate([s - lap, s, s + lap])
    turn3 = np.cumsum(np.tile(dpsi, 3))
    return np.abs(np.interp(s + half, s3, turn3) - np.interp(s - half, s3, turn3)) / (2 * half)


def path_curvature(xy, smooth=0.5):
    """Unsigned curvature [1/m] of a closed polyline at each point, averaged over `smooth` m of arc
    (at most one lap; signed turns are averaged, so wiggles cancel). Repeated points are skipped."""
    uxy, to_kept = _distinct(xy)
    if len(uxy) < 3:
        return np.zeros(len(to_kept))
    return _curvature(uxy, smooth)[to_kept]


def curvature_cap(xy, cut, slope, lookahead_min, lookahead_distance, smooth=0.5):
    """Per-point lookahead cap [m] (see the module docstring) for a closed raceline xy (n x 2).
    Curves are searched by arc length, up to lookahead_distance / slope m either way (at most half a lap)."""
    uxy, to_kept = _distinct(xy)
    if len(uxy) < 3:
        return np.full(len(to_kept), np.inf)
    s, lap, _, _ = _geometry(uxy)
    kappa = _curvature(uxy, smooth)
    reach = np.where(kappa > 1e-9, np.sqrt(2.0 * cut / np.maximum(kappa, 1e-9)), np.inf)
    window = min(lookahead_distance / slope if slope > 0 else lookahead_distance, 0.5 * lap)
    s3, reach3 = np.concatenate([s - lap, s, s + lap]), np.tile(reach, 3)
    lo = np.searchsorted(s3, s - window, 'left')
    hi = np.searchsorted(s3, s + window, 'right')
    cap = np.array([np.min(reach3[a:b] + slope * np.abs(s3[a:b] - si)) for a, b, si in zip(lo, hi, s)])
    return np.maximum(lookahead_min, cap)[to_kept]


def lookahead(speed, lookahead_time, lookahead_min, lookahead_distance, cap=np.inf, min_time=0.0):
    """Speed-scaled lookahead [m], then capped, but not below min_time * speed (cap = inf: the plain speed term)."""
    return float(min(np.clip(lookahead_time * speed, lookahead_min, lookahead_distance),
                     max(cap, min_time * speed)))
