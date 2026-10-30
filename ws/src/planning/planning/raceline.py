"""TUM minimum-time raceline: runs tum_mintime.py in the TUM venv and reads back its trajectory.

The optimizer lives in its own venv (/opt/tum_venv, see docker/install_deps.sh) so its numpy /
casadi / scipy pins never meet ROS's Python; we only exchange CSV files with it.

Raceline columns (TUM's export, map frame, closed: the last row repeats the first at s = lap length):
  s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2
psi is TUM's heading convention: 0 = +y (north), counter-clockwise positive, in [-pi, pi),
i.e. psi = yaw - pi/2 with yaw the usual ROS angle from +x.

The second header line records how the line was made (build_raceline), e.g.
  # raceline: {"method": "mintime", "clean": true, "notes": [], ...}
pure_pursuit_node logs it and warns about anything but a clean mintime line.
"""
import json
import os
import re
import subprocess
import sys
import tempfile
import time

import numpy as np
from scipy import ndimage

HEADER = 's_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2'
TUM_PYTHON = os.environ.get('TUM_PYTHON', '/opt/tum_venv/bin/python')
TUM_DIR = os.environ.get('TUM_DIR', '/opt/tum_optimizer')
DRIVER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tum_mintime.py')
PROVENANCE = 'raceline: '


class OptimizationError(RuntimeError):
    """summary: one line for reports (default: the message's first line); ipopt_exit: IPOPT's EXIT message."""

    def __init__(self, msg, summary=None, ipopt_exit=None):
        super().__init__(msg)
        self.summary = summary or msg.splitlines()[0]
        self.ipopt_exit = ipopt_exit


def run_mintime(centerline_csv, ini_path, python=TUM_PYTHON):
    """Optimize; returns (trajectory Nx7, runtime s, IPOPT's EXIT message). Raises OptimizationError on failure."""
    if not os.path.isfile(python):
        raise OptimizationError(f'TUM venv python not found at {python} (built by the Dockerfile, '
                                'see ws/src/planning/docker/install_deps.sh)')
    with tempfile.TemporaryDirectory(prefix='mintime_') as work:
        out = os.path.join(work, 'traj_race_cl.csv')
        # -I: ignore ROS's PYTHONPATH so only the venv's packages are used
        cmd = [python, '-I', DRIVER, centerline_csv, ini_path, out, work]
        env = dict(os.environ, MPLBACKEND='Agg', TUM_DIR=TUM_DIR)
        t0 = time.monotonic()
        proc = subprocess.run(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        dt = time.monotonic() - t0
        log = proc.stdout
        ipopt_exit = re.search(r'^EXIT: (.*)$', log, re.M)
        ipopt_exit = ipopt_exit.group(1).strip() if ipopt_exit else None
        # IPOPT's iteration table is long; keep the summary lines
        for line in log.splitlines():
            line = re.sub(r'\x1b\[[0-9;]*m', '', line)   # TUM colours its ERROR / WARNING lines
            if line.startswith(('INFO', 'WARNING', 'EXIT', 'Spline', 'Number of Iterations',
                                'Total seconds', 'ERROR')):
                print('  [tum] ' + line)
        if proc.returncode != 0 or not os.path.isfile(out):
            tail = '\n'.join(log.splitlines()[-25:])
            if 'EXIT: Optimal Solution Found' not in log and 'EXIT:' in log:
                why = 'IPOPT did not converge'
            else:
                why = f'optimizer failed (exit code {proc.returncode})'
            raise OptimizationError(f'mintime {why}. Last output:\n{tail}', summary=f'mintime {why}',
                                    ipopt_exit=ipopt_exit)
        try:
            traj = np.loadtxt(out, comments='#', delimiter=';', ndmin=2)
        except ValueError as e:
            raise OptimizationError(f'mintime output is malformed: {e}')
    if traj.shape[1] != 7:
        raise OptimizationError(f'mintime output has {traj.shape[1]} columns, expected 7 ({HEADER})')
    return traj, dt, ipopt_exit


def write_raceline(path, traj, provenance=None):
    header = HEADER if provenance is None else HEADER + '\n' + PROVENANCE + json.dumps(provenance)
    np.savetxt(path, traj, delimiter='; ', fmt='%.7f', header=header)


def read_raceline(path):
    return np.loadtxt(path, comments='#', delimiter=';', ndmin=2)


def read_provenance(path):
    """The provenance dict build_raceline wrote into the header, None if there is none (older or hand-made file)."""
    with open(path) as f:
        for line in f:
            if not line.startswith('#'):
                break
            if line[1:].strip().startswith(PROVENANCE):
                return json.loads(line[1:].strip()[len(PROVENANCE):])
    return None


def path_traj(xy):
    """Closed raceline rows (vx, ax = 0) along a closed polyline xy (no repeated last point), e.g. the centerline.
    Curvature from the circle through the points 0.4 m behind and ahead, as TUM's d_preview/review_curv."""
    seg = np.hypot(*(np.roll(xy, -1, axis=0) - xy).T)
    k = max(1, int(round(0.4 / seg.mean())))
    a, b, c = np.roll(xy, k, axis=0), xy, np.roll(xy, -k, axis=0)
    ab, bc, ac = b - a, c - b, c - a
    kappa = 2 * np.cross(ab, bc) / (np.linalg.norm(ab, axis=1) * np.linalg.norm(bc, axis=1)
                                    * np.linalg.norm(ac, axis=1))
    t = np.roll(xy, -1, axis=0) - np.roll(xy, 1, axis=0)
    psi = np.mod(np.arctan2(t[:, 1], t[:, 0]) - np.pi / 2 + np.pi, 2 * np.pi) - np.pi   # TUM: 0 = +y
    s = np.concatenate(([0.0], np.cumsum(seg)))
    traj = np.column_stack((s[:-1], xy, psi, kappa, np.zeros((len(xy), 2))))
    traj = np.vstack((traj, traj[0]))
    traj[-1, 0] = s[-1]
    return traj


def speed_profile(traj, v_max, ax_pos, ax_neg, ay_max, v_cap=None):
    """Fills vx, ax of a closed raceline (last row = first): as fast as the acceleration ellipse
    (ax / ax_pos|neg)^2 + (ay / ay_max)^2 <= 1 allows at both ends of every segment, at most v_max
    (and v_cap per point, if given). The forward pass accelerates out of each slow point, the backward
    pass brakes into it; both run twice around the loop so the speed is continuous at the start."""
    kappa, ds = np.abs(traj[:-1, 4]), np.diff(traj[:, 0])
    v = np.minimum(v_max, np.sqrt(ay_max / np.maximum(kappa, 1e-6)))
    if v_cap is not None:
        v = np.minimum(v, v_cap[:-1])
    n = len(v)
    kappa_seg = np.maximum(kappa, np.roll(kappa, -1))     # segment i -> i + 1: the tighter end

    def reach(v0, i, a):
        """Fastest speed at the other end of segment i from v0 with acceleration budget a. The budget is
        taken at the faster end (first guess from v0), so the ellipse holds at both ends."""
        left = lambda vi: a * np.sqrt(max(0.0, 1.0 - (vi ** 2 * kappa_seg[i] / ay_max) ** 2))  # noqa: E731
        v1 = np.sqrt(v0 ** 2 + 2 * left(v0) * ds[i])
        return np.sqrt(v0 ** 2 + 2 * left(v1) * ds[i])

    for _ in range(2):
        for j in range(n):
            i, i1 = j % n, (j + 1) % n
            v[i1] = min(v[i1], reach(v[i], i, ax_pos))
    for _ in range(2):
        for j in range(n, 0, -1):
            i, i0 = j % n, j - 1
            v[i0] = min(v[i0], reach(v[i], i0, ax_neg))
    traj = traj.copy()
    traj[:-1, 5] = v
    traj[-1, 5] = v[0]
    traj[:-1, 6] = (np.roll(v, -1) ** 2 - v ** 2) / (2 * ds)
    traj[-1, 6] = traj[0, 6]
    return traj


def lap_time(traj):
    ds = np.diff(traj[:, 0])
    v = 0.5 * (traj[:-1, 5] + traj[1:, 5])
    return float(np.sum(ds / np.maximum(v, 1e-3)))


def check_inside(traj, m, mask, half_width):
    """Smallest wall clearance of the raceline (centre to wall, m). Fails if the centre leaves the track."""
    dist = ndimage.distance_transform_edt(mask) * m.res - 0.5 * m.res
    rc = m.to_pixel(traj[:, 1:3])
    d = ndimage.map_coordinates(dist, rc.T, order=1, mode='constant', cval=-1.0)
    if d.min() < 0:
        raise OptimizationError('raceline leaves the free space (min clearance %.2f m)' % d.min())
    if d.min() < half_width:
        print('WARNING: raceline gets within %.2f m of a wall (car half width %.2f m)'
              % (d.min(), half_width), file=sys.stderr)
    return float(d.min())
