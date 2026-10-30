"""Monte Carlo localization (particle filter) with a likelihood-field sensor model, in numpy.

No ROS imports: mcl_node.py wires this to topics, test_mcl.py tests it on synthetic tracks.

Why a likelihood field and not ray casting: the weight of a particle comes from where the lidar
endpoints would land if the car were at that pose, looked up in a distance-to-nearest-wall grid.
That needs no range_libc (no Cython/CUDA build), runs on amd64 and arm64, and is a few vectorized
numpy operations per scan.

The filter tracks a pose; ScanMatcher finds one. Its global search scores every pose in the drivable area
(coarse to fine) against one scan, so the car can find itself standing still: at the start, after a crash, or
when the filter is lost. It reports a second, distinct pose that fits almost as well (a track with
similar-looking corridors), so the caller can refuse to guess. Its local search re-anchors a filter that the
odometry has thrown off (wheel slip, a stalled sensor stream) faster than the cloud can diffuse back.

Poses are (x, y, yaw) of base_link (the rear axle) in the map frame. The map is the map.pgm/map.yaml
that save_map writes (ROS map_server format, trinary).
"""
import math
import os

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage
from scipy.spatial import cKDTree


def wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def compose(a, b):
    """SE(2) a * b: pose b, given in the frame of pose a, expressed in a's parent frame."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], float(wrap(a[2] + b[2])))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), s * a[0] - c * a[1], float(wrap(-a[2])))


def relative(a, b):
    """Pose b expressed in the frame of pose a (a^-1 * b)."""
    return compose(inverse(a), b)


class LikelihoodMap:
    """Occupancy grid plus its distance-to-nearest-occupied-cell field (metres, capped at max_dist)."""

    def __init__(self, occupied, free, resolution, origin_xy, max_dist=2.0):
        """occupied, free: bool arrays in image order (row 0 = max y); origin = lower-left corner."""
        if not occupied.any():
            raise ValueError('map has no occupied cells')
        self.h, self.w = occupied.shape
        self.res = float(resolution)
        self.ox, self.oy = float(origin_xy[0]), float(origin_xy[1])
        self.max_dist = float(max_dist)
        self.occupied = occupied
        self.free = free
        self.dist = np.minimum(ndimage.distance_transform_edt(~occupied) * self.res,
                               self.max_dist).astype(np.float32)

    @classmethod
    def from_yaml(cls, path, max_dist=2.0):
        with open(path) as f:
            meta = yaml.safe_load(f)
        image = meta['image']
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(os.path.abspath(path)), image)
        origin = meta['origin']
        if abs(origin[2]) > 1e-6:
            raise ValueError(f'{path}: a rotated map origin (yaw {origin[2]}) is not supported')
        pix = np.asarray(Image.open(image).convert('L'), dtype=np.float64)
        p = pix / 255.0 if meta.get('negate', 0) else (255.0 - pix) / 255.0   # occupancy probability
        occupied = p > float(meta.get('occupied_thresh', 0.65))
        free = p < float(meta.get('free_thresh', 0.196))
        return cls(occupied, free, meta['resolution'], origin[:2], max_dist)

    def occupancy_grid(self):
        """int8 values for nav_msgs/OccupancyGrid.data (row 0 = min y): 100 occupied, 0 free, -1 unknown."""
        grid = np.full((self.h, self.w), -1, dtype=np.int8)
        grid[self.free] = 0
        grid[self.occupied] = 100
        return np.flipud(grid).ravel()

    def distance(self, x, y):
        """Distance to the nearest wall at map points (any shape); max_dist outside the map."""
        col = np.floor((x - self.ox) / self.res).astype(np.int64)
        row = self.h - 1 - np.floor((y - self.oy) / self.res).astype(np.int64)
        inside = (col >= 0) & (col < self.w) & (row >= 0) & (row < self.h)
        d = self.dist[np.clip(row, 0, self.h - 1), np.clip(col, 0, self.w - 1)]
        return np.where(inside, d, self.max_dist)

    def cell(self, x, y):
        """Per map point: 0 free, 1 unknown, 2 occupied or outside the map."""
        col = np.floor((np.asarray(x) - self.ox) / self.res).astype(np.int64)
        row = self.h - 1 - np.floor((np.asarray(y) - self.oy) / self.res).astype(np.int64)
        inside = (col >= 0) & (col < self.w) & (row >= 0) & (row < self.h)
        r, c = np.clip(row, 0, self.h - 1), np.clip(col, 0, self.w - 1)
        return np.where(~inside | self.occupied[r, c], 2, np.where(self.free[r, c], 0, 1))


def select_beams(ranges, angle_min, angle_increment, n_beams, range_min, range_max):
    """About n_beams evenly spaced beams of a scan, without invalid ones and max-range (no-hit) returns."""
    ranges = np.asarray(ranges, dtype=np.float64)
    step = max(1, int(round(len(ranges) / max(1, n_beams))))
    idx = np.arange(0, len(ranges), step)
    r = ranges[idx]
    ok = np.isfinite(r) & (r > range_min) & (r < range_max * 0.999)
    return r[ok], angle_min + idx[ok] * angle_increment


class ParticleFilter:
    def __init__(self, n=1000, seed=None, sigma_hit=0.15, z_hit=0.8, z_rand=0.2, likelihood_scale=0.4,
                 noise_x=(0.10, 0.02), noise_y=(0.05, 0.05), noise_yaw=(0.05, 0.10), resample_ratio=0.5,
                 wall_penalty=1e-3, unknown_penalty=1.0):
        """noise_*: standard deviation of the motion per metre travelled and per radian turned."""
        self.n = int(n)
        self.rng = np.random.default_rng(None if seed is None or seed < 0 else seed)
        self.sigma_hit, self.z_hit, self.z_rand = sigma_hit, z_hit, z_rand
        self.likelihood_scale = likelihood_scale   # <1 tempers the beams' correlation: keeps the cloud diverse
        self.noise_x, self.noise_y, self.noise_yaw = noise_x, noise_y, noise_yaw
        self.resample_ratio = resample_ratio
        self.wall_penalty = wall_penalty         # x weight of a particle inside a wall or outside the map
        self.unknown_penalty = unknown_penalty   # x weight of one on a cell the map never saw: off by default,
        #                                          unknown stripes on the track cost iros2026_v2 tracking accuracy
        self.p = np.zeros((self.n, 3))
        self.w = np.full(self.n, 1.0 / self.n)

    # ---------------- initialization ----------------
    def init_gaussian(self, pose, std_xy, std_yaw):
        self.p[:, 0] = pose[0] + self.rng.normal(0.0, std_xy, self.n)
        self.p[:, 1] = pose[1] + self.rng.normal(0.0, std_xy, self.n)
        self.p[:, 2] = wrap(pose[2] + self.rng.normal(0.0, std_yaw, self.n))
        self.w[:] = 1.0 / self.n

    # ---------------- motion ----------------
    def predict(self, delta):
        """Move every particle by the odometry delta (x, y, yaw in the previous base_link frame) plus noise
        that grows with the distance travelled and the angle turned (no motion, no noise)."""
        dx, dy, dyaw = delta
        trans, rot = math.hypot(dx, dy), abs(dyaw)
        if trans < 1e-9 and rot < 1e-9:
            return
        n = self.n
        ndx = dx + self.rng.normal(0.0, self.noise_x[0] * trans + self.noise_x[1] * rot, n)
        ndy = dy + self.rng.normal(0.0, self.noise_y[0] * trans + self.noise_y[1] * rot, n)
        ndyaw = dyaw + self.rng.normal(0.0, self.noise_yaw[0] * trans + self.noise_yaw[1] * rot, n)
        c, s = np.cos(self.p[:, 2]), np.sin(self.p[:, 2])
        self.p[:, 0] += c * ndx - s * ndy
        self.p[:, 1] += s * ndx + c * ndy
        self.p[:, 2] = wrap(self.p[:, 2] + ndyaw)

    # ---------------- sensor ----------------
    def update_sensor(self, ranges, angles, sensor_pose, lmap):
        """Reweight by how well each particle's lidar endpoints sit on walls. sensor_pose = laser in base_link."""
        if not len(ranges):
            return
        lx, ly, lyaw = sensor_pose
        x, y, th = self.p[:, 0], self.p[:, 1], self.p[:, 2]
        c, s = np.cos(th), np.sin(th)
        sx, sy, sth = x + c * lx - s * ly, y + s * lx + c * ly, th + lyaw
        a = sth[:, None] + np.asarray(angles)[None, :]
        ex = sx[:, None] + np.asarray(ranges)[None, :] * np.cos(a)
        ey = sy[:, None] + np.asarray(ranges)[None, :] * np.sin(a)
        d = lmap.distance(ex, ey)
        pz = self.z_hit * np.exp(-0.5 * (d / self.sigma_hit) ** 2) + self.z_rand
        loglik = self.likelihood_scale * np.log(pz).sum(axis=1)
        w = self.w * np.exp(loglik - loglik.max())
        # the car itself cannot be inside a wall or off the map (a cloud there sees nothing to contradict it:
        # beams ending outside the map cost no more than unknown space), and is rarely on unseen ground
        cell = lmap.cell(x, y)
        w[cell == 2] *= self.wall_penalty
        if self.unknown_penalty != 1.0:
            w[cell == 1] *= self.unknown_penalty
        total = w.sum()
        self.w = w / total if total > 0.0 and np.isfinite(total) else np.full(self.n, 1.0 / self.n)

    def neff(self):
        return 1.0 / float(np.sum(self.w ** 2))

    def resample(self, force=False):
        """Low-variance resampling when the effective sample size drops below resample_ratio * n."""
        if not force and self.neff() >= self.resample_ratio * self.n:
            return False
        positions = (self.rng.random() + np.arange(self.n)) / self.n
        cumulative = np.cumsum(self.w)
        cumulative[-1] = 1.0
        self.p = self.p[np.searchsorted(cumulative, positions)]
        self.w[:] = 1.0 / self.n
        return True

    # ---------------- estimate ----------------
    def estimate(self):
        w, p = self.w, self.p
        return (float(w @ p[:, 0]), float(w @ p[:, 1]),
                float(math.atan2(w @ np.sin(p[:, 2]), w @ np.cos(p[:, 2]))))

    def spread(self):
        """(position std [m], yaw circular std [rad]) of the weighted cloud."""
        mx, my, _ = self.estimate()
        pos = math.sqrt(float(self.w @ ((self.p[:, 0] - mx) ** 2 + (self.p[:, 1] - my) ** 2)))
        r = min(1.0, math.hypot(float(self.w @ np.sin(self.p[:, 2])), float(self.w @ np.cos(self.p[:, 2]))))
        return pos, math.sqrt(-2.0 * math.log(max(r, 1e-12)))


CENTRE_PULL = 0.005   # fit units per m (or rad) from the search centre: breaks ties toward it


class ScanMatcher:
    """Correlative scan matching on the likelihood field: the fit of a pose is the mean over the beams of
    exp(-d^2 / 2 sigma^2), d = distance from the beam's endpoint to the nearest wall (1 = every endpoint on a
    wall, 0 = none near one). Brute force over a pose grid, vectorized."""
    PASS_BACK = 0.3   # m before a beam's endpoint that must be free space

    def __init__(self, lmap, sigma=0.10, coarse_step=0.2, coarse_sigma=0.3, yaw_step=math.radians(5.0),
                 clearance=0.1):
        """coarse_step: m between the global search's candidate positions, drivable cells only (free and at
        least clearance from a wall); coarse_sigma: forgiving fit for that coarse grid."""
        self.lmap = lmap
        self.fine = np.exp(-0.5 * (lmap.dist / sigma) ** 2).astype(np.float32)
        self.coarse = np.exp(-0.5 * (lmap.dist / coarse_sigma) ** 2).astype(np.float32)
        self.passable = lmap.free.astype(np.float32)
        self.coarse_step = coarse_step
        self.yaw_step = yaw_step
        k = max(1, int(round(coarse_step / lmap.res)))
        rows, cols = np.nonzero(lmap.free & (lmap.dist >= clearance))
        keep = (rows % k == 0) & (cols % k == 0)
        rows, cols = rows[keep], cols[keep]
        self.candidates = np.c_[lmap.ox + (cols + 0.5) * lmap.res, lmap.oy + (lmap.h - rows - 0.5) * lmap.res]

    def _lookup(self, grid, x, y):
        m = self.lmap
        col = np.floor((x - m.ox) / m.res).astype(np.int64)
        row = m.h - 1 - np.floor((y - m.oy) / m.res).astype(np.int64)
        inside = (col >= 0) & (col < m.w) & (row >= 0) & (row < m.h)
        return np.where(inside, grid[np.clip(row, 0, m.h - 1), np.clip(col, 0, m.w - 1)], 0.0)

    def _scores(self, grid, xy, yaws, ranges, angles, sensor_pose):
        """Fit of every (position, yaw) pair: array (len(xy), len(yaws)). A beam counts only if the space it
        crossed is free in the map (sampled halfway and PASS_BACK before its endpoint): an endpoint that lands
        inside a wall block, or a beam that passes through a wall, does not fit."""
        lx, ly, lyaw = sensor_pose
        ranges, angles = np.asarray(ranges), np.asarray(angles)
        c, s = np.cos(yaws), np.sin(yaws)
        dirx, diry = np.cos(lyaw + angles), np.sin(lyaw + angles)

        def offsets(r):   # points at range r along each beam, relative to base_link's position: (yaws, beams)
            bx, by = lx + r * dirx, ly + r * diry
            return c[:, None] * bx[None, :] - s[:, None] * by[None, :], s[:, None] * bx[None, :] + c[:, None] * by[None, :]

        ends = offsets(ranges)
        passes = [offsets(0.5 * ranges), offsets(np.maximum(ranges - self.PASS_BACK, 0.0))]
        out = np.empty((len(xy), len(yaws)), dtype=np.float32)
        chunk = max(1, 600_000 // max(1, ends[0].size))
        for i in range(0, len(xy), chunk):
            px, py = xy[i:i + chunk, 0, None, None], xy[i:i + chunk, 1, None, None]
            v = self._lookup(grid, px + ends[0][None], py + ends[1][None])
            for ox, oy in passes:
                v = v * self._lookup(self.passable, px + ox[None], py + oy[None])
            out[i:i + chunk] = v.mean(axis=2)
        return out

    def fit(self, pose, ranges, angles, sensor_pose, coarse=False):
        """Fit of one pose (fine sigma; coarse: the forgiving one, for poses carried by odometry alone)."""
        if not len(ranges):
            return 0.0
        return float(self._scores(self.coarse if coarse else self.fine, np.array([pose[:2]]),
                                  np.array([pose[2]]), ranges, angles, sensor_pose)[0, 0])

    def search_local(self, pose, ranges, angles, sensor_pose, radius=0.8, yaw_range=math.radians(15.0),
                     step=None, yaw_step=math.radians(1.5)):
        """Best (pose, fit) within radius / yaw_range of pose, on a grid of step (default: map resolution).
        Among (nearly) equal fits the one closest to pose wins: along a featureless corridor the scan cannot
        place the car, so the search does not move it."""
        step = step or self.lmap.res
        d = np.arange(-radius, radius + 1e-9, step)
        gx, gy = np.meshgrid(pose[0] + d, pose[1] + d)
        xy = np.c_[gx.ravel(), gy.ravel()]
        dyaw = np.arange(-yaw_range, yaw_range + 1e-9, yaw_step)
        sc = self._scores(self.fine, xy, pose[2] + dyaw, ranges, angles, sensor_pose)
        pull = CENTRE_PULL * (np.hypot(xy[:, 0] - pose[0], xy[:, 1] - pose[1])[:, None] + np.abs(dyaw)[None, :])
        i, j = np.unravel_index(int(np.argmax(sc - pull)), sc.shape)
        return (float(xy[i, 0]), float(xy[i, 1]), float(wrap(pose[2] + dyaw[j]))), float(sc[i, j])

    def set_track(self, line, radius=0.6, yaw_window=math.radians(25.0)):
        """Enable search_global(track_only=True): line = (x, y) points the car drives along, in order (the
        raceline). Its candidates are the search positions within radius of the line, each tried only at
        headings within yaw_window of the line's direction there, forward and backward (a car facing
        backwards, after a crash, can look exactly like one driving forward elsewhere on a symmetric track:
        with both directions that is a tie to settle, not a confident wrong pose). A car turned across the
        track fits poorly here, and the caller falls back to the full search."""
        line = np.asarray(line, dtype=np.float64)
        heading = np.arctan2(np.roll(line[:, 1], -1) - line[:, 1], np.roll(line[:, 0], -1) - line[:, 0])
        # nearest line point per candidate: a k-d tree, not the candidates x line matrix (iros2026_v2: 10898 x
        # 1937 doubles per array, the node's memory went from 69 to 540 MB at startup)
        dist, idx = cKDTree(line).query(self.candidates)
        near = dist <= radius
        nearest = heading[idx]
        window = np.arange(-yaw_window, yaw_window + 1e-9, self.yaw_step)
        offsets = np.concatenate([window, window + math.pi])
        bins = np.round(wrap(nearest) / self.yaw_step).astype(int)   # group by heading: one yaw set per group
        self.track_groups = []
        for b in np.unique(bins[near]):
            sel = near & (bins == b)
            self.track_groups.append((self.candidates[sel], b * self.yaw_step + offsets))
        self.track_poses = int(near.sum()) * len(offsets)

    def search_global(self, ranges, angles, sensor_pose, seeds=40, separation=1.0,
                      separation_yaw=math.radians(30.0), track_only=False):
        """Distinct pose hypotheses [(pose, fit), ...], best first: every candidate position at every yaw_step
        on the coarse field (track_only: only near the line set_track gave, heading along it), then the best
        seeds refined on the fine field. Two hypotheses are distinct when more than separation m or
        separation_yaw rad apart."""
        if track_only:
            groups = getattr(self, 'track_groups', [])
        else:
            groups = [(self.candidates, np.arange(-math.pi, math.pi, self.yaw_step))]
        if not len(ranges) or not groups or not len(self.candidates):
            return []
        score, px, py, pyaw = [], [], [], []   # every coarse pose of every group, flattened
        for xy, yaws in groups:
            sc = self._scores(self.coarse, xy, yaws, ranges, angles, sensor_pose)
            score.append(sc.ravel())
            px.append(np.repeat(xy[:, 0], len(yaws)))
            py.append(np.repeat(xy[:, 1], len(yaws)))
            pyaw.append(np.tile(yaws, len(xy)))
        score, px, py, pyaw = (np.concatenate(a) for a in (score, px, py, pyaw))
        picked = []
        for f in np.argsort(score)[::-1]:   # the best seeds, at most one per coarse neighbourhood
            x, y, yaw = px[f], py[f], float(wrap(pyaw[f]))
            if all(math.hypot(x - q[0], y - q[1]) > self.coarse_step * 1.5
                   or abs(float(wrap(yaw - q[2]))) > self.yaw_step * 1.5 for q in picked):
                picked.append((x, y, yaw))
                if len(picked) >= seeds:
                    break
        refined = [self.search_local(q, ranges, angles, sensor_pose, radius=self.coarse_step,
                                     yaw_range=self.yaw_step, yaw_step=math.radians(1.0)) for q in picked]
        refined.sort(key=lambda r: -r[1])
        out = []
        for pose, score in refined:
            if all(math.hypot(pose[0] - q[0][0], pose[1] - q[0][1]) > separation
                   or abs(float(wrap(pose[2] - q[0][2]))) > separation_yaw for q in out):
                out.append((pose, score))
        return out


def pose_distance(a, b, yaw_scale=1.0):
    """Distance between two poses: metres, plus yaw_scale metres per radian of heading difference (the same
    spot facing the other way is far)."""
    return math.hypot(a[0] - b[0], a[1] - b[1]) + yaw_scale * abs(float(wrap(a[2] - b[2])))


def choose_pose(hyps, priors, ambiguity=0.97, prior_radius=3.0, yaw_scale=1.0, force=False, refine=None):
    """Pick the car's pose from search_global's hypotheses [(pose, fit), ...].

    One hypothesis fitting clearly best: that one. Several tied (fit >= ambiguity x the best): the one
    one at a prior pose (where the car was), within prior_radius: refine(prior) -> (pose, fit), a local
    search from the prior that the scan moves only as far as it can tell (search_global's hypotheses sit ~1 m
    apart at arbitrary places along a featureless corridor), if it fits as well as the tie; without refine, or
    if it fits worse, the hypothesis clearly closest (pose_distance) to a prior. Otherwise None (the scan
    cannot tell), unless force: then the closest to a prior within prior_radius, else the best fit.
    Returns (pose, alternatives, how): alternatives are the other tied poses, to rule out once the car moves.
    """
    if not hyps:
        return None
    tied = [h for h in hyps if h[1] >= ambiguity * hyps[0][1]]
    if len(tied) == 1:
        return tied[0][0], [], f'fit {tied[0][1]:.2f}'

    def dist(pose):
        return min((pose_distance(pose, q, yaw_scale) for q in priors), default=math.inf)

    ranked = sorted(tied, key=lambda h: dist(h[0]))
    d0, d1 = dist(ranked[0][0]), dist(ranked[1][0])
    pick = None
    if d0 <= prior_radius and refine is not None:
        prior = min(priors, key=lambda q: pose_distance(ranked[0][0], q, yaw_scale))
        pose, fit = refine(prior)
        if fit >= ambiguity * hyps[0][1]:
            pick, how = pose, f'{len(tied)} places fit, refined from the last pose (fit {fit:.2f})'
    if pick is None and d0 <= prior_radius and d1 > 2.0 * d0 + 0.5:
        pick, how = ranked[0][0], f'{len(tied)} places fit, the one {d0:.1f} from the last pose'
    if pick is None:
        if not force:
            return None
        pick = ranked[0][0] if d0 <= prior_radius else tied[0][0]
        how = f'{len(tied)} places fit, guessed the {"closest" if d0 <= prior_radius else "best"}; checking the others'
    alternatives = [h[0] for h in tied if pose_distance(h[0], pick, yaw_scale) > 1.0]
    return pick, alternatives, how


DECAY = 0.99   # per update, of the alternatives' evidence: noise cannot random-walk it to a decision over a lap


class Alternatives:
    """Poses the car might also be at after an ambiguous pick, carried along by the odometry and kept on the scan
    by a tiny local search (0.1 m), like the filter keeps the pick. Each update scores the best fit near each
    of them and near the pick, and sums the differences: evidence. An alternative is ruled out only when the
    evidence says it fits clearly worse (the pick out-fits it by `evidence` fit x updates in total: passing a
    feature the look-alike does not have); the car switches to one that out-fits the pick by as much. Where the
    two look the same (a featureless stretch) nothing is decided: an unresolved alternative is kept until the
    car has driven `distance` m (about a lap), not dropped as confirmed after a few metres."""

    def __init__(self, poses, odom_pose, evidence=1.5, min_updates=5, distance=200.0):
        self.poses = list(poses)[:self.MAX]   # each as it would be at odom pose self.odom0
        self.odom0 = odom_pose
        self.last_odom = odom_pose
        self.evidence, self.min_updates, self.distance = evidence, min_updates, distance
        self.score = np.zeros(len(self.poses))   # sum over updates of fit(pick) - fit(alternative)
        self.n = 0
        self.travelled = 0.0
        self.ruled_out = 0

    MAX = 12   # places checked at once (each costs a tiny local search per update)

    def add(self, poses, odom_pose, min_separation=1.0):
        """More places to check (poses now, at odom_pose): those not already among them, up to MAX."""
        moved = relative(self.odom0, odom_pose)
        now = [compose(p, moved) for p in self.poses]
        for q in poses:
            if len(self.poses) < self.MAX and all(math.hypot(q[0] - n[0], q[1] - n[1]) > min_separation
                                                  or abs(float(wrap(q[2] - n[2]))) > 0.5 for n in now):
                self.poses.append(compose(q, inverse(moved)))
                self.score = np.append(self.score, 0.0)
                now.append(q)

    @property
    def done(self):
        """Nothing left to check: every alternative ruled out, or `distance` m driven without telling them apart
        (self.poses then holds the unresolved ones)."""
        return not self.poses or self.travelled >= self.distance

    @staticmethod
    def _best(matcher, pose, ranges, angles, sensor_pose):
        return matcher.search_local(pose, ranges, angles, sensor_pose, radius=0.1, yaw_range=math.radians(1.0),
                                    step=0.05, yaw_step=math.radians(1.0))

    def update(self, matcher, pick, odom_pose, ranges, angles, sensor_pose):
        """pick: the filter's pose now. Returns (alternative pose now, its evidence, 0.0) to switch to, or None."""
        step = relative(self.last_odom, odom_pose)
        self.travelled += math.hypot(step[0], step[1])
        self.last_odom = odom_pose
        if not self.poses:
            return None
        moved = relative(self.odom0, odom_pose)
        _, fit_pick = self._best(matcher, pick, ranges, angles, sensor_pose)
        for j, p in enumerate(self.poses):
            best, fit = self._best(matcher, compose(p, moved), ranges, angles, sensor_pose)
            self.poses[j] = compose(best, inverse(moved))   # kept on the scan, as the filter keeps the pick
            self.score[j] = DECAY * self.score[j] + fit_pick - fit
        self.n += 1
        if self.n < self.min_updates:
            return None
        j = int(np.argmin(self.score))
        if -self.score[j] >= self.evidence:
            return compose(self.poses[j], moved), float(-self.score[j]), 0.0
        keep = self.score < self.evidence
        self.ruled_out += int((~keep).sum())
        self.poses = [p for p, k in zip(self.poses, keep) if k]
        self.score = self.score[keep]
        return None
