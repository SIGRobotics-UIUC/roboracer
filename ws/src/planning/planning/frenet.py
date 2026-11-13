"""Frenet frame conversion against a reference line (centerline or raceline).

The Frenet frame describes a point by how far it is *along* a reference line (arc length s)
and how far it sits *beside* it (lateral offset d), instead of by (x, y) in the map frame.
Local planning is far easier in these coordinates: track limits become constant bounds on d,
"ahead" and "behind" become comparisons on s, and an overtake becomes a curve in (s, d) that
starts and ends at d = 0.

    s   metres along the reference line from its first point, wrapping at the lap length
    d   metres to the left of the reference line (right is negative), matching the sign
        convention of centerline.normals_left

The reference is treated as a closed loop: s wraps at `length`, and the segment from the last
point back to the first is a real segment. Raceline CSVs from the TUM optimizer repeat the
first point as the last row; `from_raceline` drops that duplicate.

Nearest-point lookup is a KD-tree query followed by an exact projection onto the two segments
either side of the hit, so the result is exact rather than snapped to the nearest sample.
"""

import numpy as np
from scipy.spatial import cKDTree

# raceline.csv columns: s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2
_S, _X, _Y, _PSI, _KAPPA, _VX = 0, 1, 2, 3, 4, 5


def _shape(flat, shape):
    """Return a float for scalar input, an array of the broadcast shape otherwise."""
    return float(flat[0]) if shape == () else flat.reshape(shape)


class FrenetConverter:
    """Convert between map-frame (x, y) and Frenet (s, d) along a closed reference line."""

    def __init__(self, xy, s=None, psi=None, kappa=None, vx=None):
        """
        xy      (N, 2) reference points in the map frame, in driving direction, not closed
        s       (N,) arc length per point; computed from xy when omitted
        psi     (N,) heading per point in radians; computed from xy when omitted
        kappa   (N,) curvature per point; optional, needed by curvature_at and velocities
        vx      (N,) reference speed per point; optional, needed by speed_at
        """
        xy = np.asarray(xy, dtype=float)
        if xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 3:
            raise ValueError(f'need an (N, 2) reference with N >= 3, got {xy.shape}')
        if np.allclose(xy[0], xy[-1]):
            xy = xy[:-1]

        self.xy = xy
        self.n = len(xy)

        # segment i runs from point i to point i+1, the last wrapping back to point 0
        self.seg = np.roll(xy, -1, axis=0) - xy
        self.seg_len = np.linalg.norm(self.seg, axis=1)
        if np.any(self.seg_len < 1e-9):
            raise ValueError('reference line contains duplicate consecutive points')

        if s is None:
            s = np.concatenate(([0.0], np.cumsum(self.seg_len)[:-1]))
        self.s = np.asarray(s, dtype=float)[:self.n]
        self.length = float(self.s[-1] + self.seg_len[-1])

        if psi is None:
            # central difference, so the heading at a point reflects the line through it
            t = np.roll(xy, -1, axis=0) - np.roll(xy, 1, axis=0)
            psi = np.arctan2(t[:, 1], t[:, 0])
        self.psi = np.asarray(psi, dtype=float)[:self.n]

        self.kappa = None if kappa is None else np.asarray(kappa, dtype=float)[:self.n]
        self.vx = None if vx is None else np.asarray(vx, dtype=float)[:self.n]
        self.d_left = None     # filled by load_bounds: max d still inside the track
        self.d_right = None    # filled by load_bounds: min d still inside the track (negative)
        self._tree = cKDTree(xy)

    # ------------------------------------------------------------------ loading

    @classmethod
    def from_raceline(cls, path):
        """Build from a raceline.csv written by build_raceline (TUM output format)."""
        t = np.loadtxt(path, comments='#', delimiter=';', ndmin=2)
        return cls(t[:, [_X, _Y]], s=t[:, _S], psi=t[:, _PSI], kappa=t[:, _KAPPA], vx=t[:, _VX])

    @classmethod
    def from_centerline(cls, path):
        """Build from a centerline.csv written by extract_centerline (TUM input format)."""
        t = np.loadtxt(path, comments='#', delimiter=',', ndmin=2)
        return cls(t[:, :2])

    # ------------------------------------------------------------------ forward

    def to_frenet(self, x, y):
        """Map-frame (x, y) -> (s, d). Accepts scalars or arrays; returns arrays of that shape.

        d is positive to the left of the direction of travel.
        """
        shape = np.broadcast_shapes(np.shape(x), np.shape(y))
        p = np.column_stack((np.broadcast_to(np.asarray(x, float), shape).ravel(),
                             np.broadcast_to(np.asarray(y, float), shape).ravel()))

        _, nearest = self._tree.query(p)
        # the true foot of the perpendicular lies on the segment before or after the nearest
        # sample, so test both and keep whichever is closer
        cand = np.stack(((nearest - 1) % self.n, nearest % self.n), axis=1)

        best_d2 = np.full(len(p), np.inf)
        best_s = np.zeros(len(p))
        best_d = np.zeros(len(p))

        for k in range(cand.shape[1]):
            i = cand[:, k]
            a, seg, seg_len = self.xy[i], self.seg[i], self.seg_len[i]
            rel = p - a
            t = np.clip(np.einsum('ij,ij->i', rel, seg) / seg_len ** 2, 0.0, 1.0)
            foot = a + t[:, None] * seg
            d2 = np.einsum('ij,ij->i', p - foot, p - foot)

            # signed offset: cross(segment, point - a) > 0 means the point is left of travel
            cross = seg[:, 0] * rel[:, 1] - seg[:, 1] * rel[:, 0]

            take = d2 < best_d2
            best_d2[take] = d2[take]
            best_s[take] = (self.s[i] + t * seg_len)[take]
            best_d[take] = (np.sign(cross) * np.sqrt(d2))[take]

        return _shape(best_s % self.length, shape), _shape(best_d, shape)

    # ------------------------------------------------------------------ inverse

    def to_cartesian(self, s, d=0.0):
        """Frenet (s, d) -> map-frame (x, y). s wraps; d is metres to the left.

        Inverse to to_frenet up to the discretisation of the reference line. Measured
        round-trip error follows |d| * spacing / (2 * R), where spacing is the distance
        between reference samples and R the local radius of curvature:

            R = 10 m, 0.1 m spacing, d = 1 m    ->  5 mm
            R = 10 m, 0.5 m spacing, d = 1 m    ->  25 mm
            R =  1 m, 0.1 m spacing, d = 0.5 m  ->  25 mm

        Halving the sample spacing halves the error, so resample the reference finer if you
        need more precision in hairpins. Separately, the mapping stops being one-to-one once
        |d| approaches the local radius of curvature. That is a property of Frenet
        coordinates rather than of this implementation, so keep planned offsets well inside
        the track and treat large d on tight corners with suspicion.
        """
        shape = np.broadcast_shapes(np.shape(s), np.shape(d))
        sv = (np.broadcast_to(np.asarray(s, float), shape) % self.length).ravel()
        dv = np.broadcast_to(np.asarray(d, float), shape).ravel()

        i, t = self._locate(sv)
        base = self.xy[i] + t[:, None] * self.seg[i]
        psi = self._psi_at(i, t)
        return (_shape(base[:, 0] - dv * np.sin(psi), shape),
                _shape(base[:, 1] + dv * np.cos(psi), shape))

    def heading_at(self, s):
        """Reference-line heading in radians at arc length s.

        This is the smoothed heading interpolated between vertices, which is what a
        controller wants as a reference. It differs by up to half the per-segment heading
        change from the piecewise-constant segment direction that to_cartesian offsets
        along, so do not use it to reconstruct points by hand; call to_cartesian.
        """
        shape = np.shape(s)
        i, t = self._locate((np.asarray(s, float) % self.length).ravel())
        return _shape(self._psi_at(i, t), shape)

    def curvature_at(self, s):
        """Reference-line curvature at arc length s. Needs a reference built with kappa."""
        if self.kappa is None:
            raise ValueError('this converter was built without curvature; use from_raceline')
        shape = np.shape(s)
        i, t = self._locate((np.asarray(s, float) % self.length).ravel())
        k = self.kappa[i] + t * (self.kappa[(i + 1) % self.n] - self.kappa[i])
        return _shape(k, shape)

    @classmethod
    def from_track(cls, track_dir):
        """Build from a tracks/<name>/ directory: raceline geometry plus track bounds.

        This is the constructor a planner wants. The reference line is the raceline, so d = 0
        is the racing line and an evasion is a deviation from it, and the track edges are
        available through width_at.
        """
        import os
        conv = cls.from_raceline(os.path.join(track_dir, 'raceline.csv'))
        conv.load_bounds(os.path.join(track_dir, 'centerline.csv'))
        return conv

    def load_bounds(self, centerline_csv):
        """Attach track edges, expressed as d limits in *this* reference's frame.

        centerline.csv carries widths measured from the centerline, but a planner working off
        the raceline needs them relative to the raceline. Both edges are built in the map
        frame, converted into this frame, and resampled onto the reference's own s grid.
        """
        c = np.loadtxt(centerline_csv, comments='#', delimiter=',', ndmin=2)
        xy, w_right, w_left = c[:, :2], c[:, 2], c[:, 3]

        t = np.roll(xy, -1, axis=0) - np.roll(xy, 1, axis=0)
        t /= np.linalg.norm(t, axis=1, keepdims=True)
        nl = np.column_stack((-t[:, 1], t[:, 0]))          # unit normal, left of travel

        left = xy + nl * w_left[:, None]
        right = xy - nl * w_right[:, None]

        self.d_left = self._edge_to_d(left)
        self.d_right = self._edge_to_d(right)
        if np.any(self.d_left <= self.d_right):
            raise ValueError('track bounds cross: left edge is not left of the right edge')
        return self

    def _edge_to_d(self, edge_xy):
        """Lateral offset of a boundary polyline, resampled onto this reference's s grid."""
        s, d = self.to_frenet(edge_xy[:, 0], edge_xy[:, 1])
        order = np.argsort(s)
        s, d = s[order], d[order]
        # wrap one sample either side so np.interp is continuous across the start line
        s_w = np.concatenate((s[-1:] - self.length, s, s[:1] + self.length))
        d_w = np.concatenate((d[-1:], d, d[:1]))
        return np.interp(self.s, s_w, d_w)

    def width_at(self, s):
        """(d_left, d_right): the open interval of d that stays on track at arc length s.

        d_left is positive, d_right negative. Needs load_bounds or from_track.
        """
        if self.d_left is None:
            raise ValueError('no track bounds loaded; use from_track or load_bounds')
        shape = np.shape(s)
        i, t = self._locate((np.asarray(s, float) % self.length).ravel())
        j = (i + 1) % self.n
        lo = self.d_right[i] + t * (self.d_right[j] - self.d_right[i])
        hi = self.d_left[i] + t * (self.d_left[j] - self.d_left[i])
        return _shape(hi, shape), _shape(lo, shape)

    def is_on_track(self, s, d, margin=0.0):
        """True where (s, d) is inside the track edges, shrunk by `margin` on each side."""
        hi, lo = self.width_at(s)
        return (np.asarray(d) < hi - margin) & (np.asarray(d) > lo + margin)

    def speed_at(self, s):
        """Reference speed at arc length s. Needs a reference built from a raceline."""
        if self.vx is None:
            raise ValueError('this converter was built without speeds; use from_raceline')
        shape = np.shape(s)
        i, t = self._locate((np.asarray(s, float) % self.length).ravel())
        v = self.vx[i] + t * (self.vx[(i + 1) % self.n] - self.vx[i])
        return _shape(v, shape)

    def velocity_to_frenet(self, s, d, vx, vy):
        """Map-frame velocity -> (s_dot, d_dot) at the Frenet position (s, d).

        Progress along the track is not simply the tangential speed: on a corner, a car
        running wide covers more ground per unit of s than one on the inside. The 1 - kappa*d
        factor is that correction, and it is why opponent progress has to be computed here
        rather than by differencing s.
        """
        shape = np.broadcast_shapes(np.shape(s), np.shape(d), np.shape(vx), np.shape(vy))
        sv = np.broadcast_to(np.asarray(s, float), shape).ravel() % self.length
        dv = np.broadcast_to(np.asarray(d, float), shape).ravel()
        vxv = np.broadcast_to(np.asarray(vx, float), shape).ravel()
        vyv = np.broadcast_to(np.asarray(vy, float), shape).ravel()

        psi = self.heading_at(sv)
        tangential = vxv * np.cos(psi) + vyv * np.sin(psi)
        lateral = -vxv * np.sin(psi) + vyv * np.cos(psi)

        scale = 1.0 - (self.curvature_at(sv) * dv if self.kappa is not None else 0.0)
        scale = np.where(np.abs(scale) < 1e-6, 1e-6, scale)
        return _shape(tangential / scale, shape), _shape(lateral, shape)

    # ------------------------------------------------------------------ helpers

    def ds(self, s_from, s_to):
        """Signed lap distance from s_from to s_to, in (-length/2, length/2].

        Use this instead of subtracting, so a car just past the start line still reads as
        ahead of one just before it.
        """
        return (np.asarray(s_to, dtype=float) - np.asarray(s_from, dtype=float)
                + self.length / 2) % self.length - self.length / 2

    def is_ahead(self, s_self, s_other):
        """True where s_other is ahead of s_self on the lap."""
        return self.ds(s_self, s_other) > 0

    def _locate(self, s):
        """Segment index and fraction along it for each arc length."""
        i = np.clip(np.searchsorted(self.s, s, side='right') - 1, 0, self.n - 1)
        t = np.clip((s - self.s[i]) / self.seg_len[i], 0.0, 1.0)
        return i, t

    def _psi_at(self, i, t):
        """Heading interpolated across a segment, taking the short way around the circle."""
        a = self.psi[i]
        b = self.psi[(i + 1) % self.n]
        return a + t * ((b - a + np.pi) % (2 * np.pi) - np.pi)

    def __repr__(self):
        bounds = '' if self.d_left is None else ', bounds loaded'
        return f'FrenetConverter({self.n} points, {self.length:.2f} m lap{bounds})'