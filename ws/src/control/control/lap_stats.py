"""Lap timing and per-lap statistics from the car's pose along the raceline (no ROS; lap_logger_node uses it).

A lap is timed from one crossing of the raceline's start (s = 0) to the next, interpolated between the two
poses either side of the line. It only counts if the car passed the halfway point (s in [0.4, 0.6] of the
lap) since the last crossing, so pose noise at the line or reversing over it never counts a lap.

The first lap of a run is timed from when the car first moves if it begins just after the line (s < 25 %
of the lap), otherwise from the first crossing. It is a standing start if the car began within 25 % of the
lap of the line (either side), else flying; later laps are flying.

A pose jump is a step between two poses faster than JUMP_SPEED (and over JUMP_MIN): the localizer
corrected its pose (Cartographer does by up to ~1 m on Porto) or the car was moved. A gap in the pose
stream is not one: the car may well have driven that far meanwhile. Jumps up to max_jump are counted in
the lap (pose_jumps, max_jump_m) and left out of its distance and speed; a longer one (the car was put
back at the start, a respawn) aborts the lap, as does abort() (lap_logger_node: the sim reported a reset).
The tracker then starts over as at the beginning of a run.

Statistics per lap: time, distance driven, mean and max distance of the car from the raceline (as the
localizer sees it: pure pursuit's tracking error, not the true one), and max speed. Distance and speed are
measured over SPEED_WINDOW steps, not pose to pose (localizer jitter).
"""
import math
from dataclasses import dataclass, field

import numpy as np

START_WINDOW = 0.25    # fraction of the lap after the line where a run may begin as a standing start
HALF = (0.4, 0.6)      # fraction of the lap that counts as the halfway point
MOVED = 0.1            # m from the first pose: the car has started moving
# s: speed is measured over at least this long (localizer poses arrive unevenly, and a 1 cm correction
# 1 ms after the last pose would read as 10 m/s)
SPEED_WINDOW = 0.1
JUMP_SPEED = 15.0      # m/s between two poses: no RoboRacer drives that fast, so the pose jumped
JUMP_MIN = 0.3         # m: smaller steps are never jumps (a pose 1 ms after the last, with noise)


@dataclass
class Lap:
    number: int                 # 1, 2, ... in the run, aborted laps included
    status: str                 # 'ok' or 'aborted'
    start: str                  # 'standing' or 'flying'
    time_s: float               # lap time; for an aborted lap, the time until the abort
    distance_m: float
    mean_offset_m: float
    max_offset_m: float
    max_speed_mps: float
    pose_jumps: int = 0         # localizer corrections (see the module docstring)
    max_jump_m: float = 0.0
    note: str = ''              # why it was aborted

    @property
    def mean_speed_mps(self):
        return self.distance_m / self.time_s if self.time_s > 0 else 0.0


@dataclass
class _Acc:
    t0: float
    start: str
    distance: float = 0.0
    offsets: list = field(default_factory=list)
    max_speed: float = 0.0
    jumps: list = field(default_factory=list)


class LapTracker:
    def __init__(self, xy, max_jump=3.0):
        """xy: raceline points (N x 2, closed loop, the last point not repeating the first)."""
        xy = np.asarray(xy, dtype=float)
        if len(xy) > 1 and np.allclose(xy[0], xy[-1]):
            xy = xy[:-1]
        if len(xy) < 3:
            raise ValueError('a raceline needs at least 3 points')
        self.a = xy
        self.d = np.roll(xy, -1, axis=0) - xy
        self.seg_len = np.hypot(self.d[:, 0], self.d[:, 1])
        self.seg_len2 = np.maximum(self.seg_len ** 2, 1e-12)
        self.s0 = np.concatenate(([0.0], np.cumsum(self.seg_len)[:-1]))
        self.length = float(self.seg_len.sum())
        self.max_jump = max_jump
        self.laps = []           # every Lap of the run, in order
        self._reset()

    def _reset(self):
        self.first = None        # (x, y) of the first pose since the start or the last abort
        self.prev = None         # (t, x, y, s) of the last pose
        self.acc = None          # the lap being timed, or None
        self.armed = False       # standing start: time from the first movement
        self.near_line = False   # the run began near the line: its first lap is a standing start
        self.half = False
        self.speed_ref = None    # (t, x, y): start of the current speed window

    def project(self, x, y):
        """(s, offset): arc length of the closest raceline point and the car's distance from it."""
        p = np.array([x, y])
        t = np.clip(((p - self.a) * self.d).sum(axis=1) / self.seg_len2, 0.0, 1.0)
        q = self.a + t[:, None] * self.d
        dist = np.hypot(q[:, 0] - x, q[:, 1] - y)
        i = int(np.argmin(dist))
        return float(self.s0[i] + t[i] * self.seg_len[i]), float(dist[i])

    def update(self, t, x, y):
        """Feed one pose (time in s, map-frame position). Returns the Lap it finished or aborted, else None."""
        s, offset = self.project(x, y)
        L = self.length
        if self.prev is None:
            self.first = (x, y)
            self.prev = (t, x, y, s)
            self.armed = s < START_WINDOW * L
            self.near_line = self.armed or s > (1 - START_WINDOW) * L
            return None
        t1, x1, y1, s1 = self.prev
        self.prev = (t, x, y, s)
        step = math.hypot(x - x1, y - y1)
        jump = step > JUMP_MIN and step > JUMP_SPEED * (t - t1)
        if jump and step > self.max_jump:
            return self.abort(f'pose jumped {step:.1f} m')

        if self.speed_ref is None:
            self.speed_ref = (t1, x1, y1)
        if jump:            # close the window before it, restart it after: the jump is not driving
            self._window(t1, x1, y1)
            self.speed_ref = (t, x, y)
            if self.acc is not None:
                self.acc.jumps.append(step)
        if self.armed and math.hypot(x - self.first[0], y - self.first[1]) > MOVED:
            self.armed = False
            self.acc = _Acc(t1, 'standing')   # from the last pose before the car moved
            self.speed_ref = (t1, x1, y1)

        finished = None
        crossed = s1 > (1 - START_WINDOW) * L and s < START_WINDOW * L
        if crossed:
            ahead = L - s1 + s
            t_cross = t1 + (t - t1) * ((L - s1) / ahead if ahead > 0 else 1.0)
            if self.acc is not None and self.half:
                self._window(t, x, y)            # the lap's last stretch, up to this pose
                finished = self._close(t_cross, 'ok')
            if self.acc is None or finished is not None:
                # first crossing of the run, or the next lap starts where this one ended
                self.acc = _Acc(t_cross, 'standing' if finished is None and self.near_line else 'flying')
                self.armed = self.near_line = False
                self.speed_ref = (t, x, y)
            self.half = False
        elif HALF[0] * L <= s <= HALF[1] * L:
            self.half = True

        if t - self.speed_ref[0] >= SPEED_WINDOW:
            self._window(t, x, y)
        if self.acc is not None:
            self.acc.offsets.append(offset)
        return finished

    def _window(self, t, x, y):
        """Close the distance/speed window at this pose: its straight-line length is added to the lap's
        distance (summing every pose-to-pose step would add up the localizer's jitter, ~4 % on Porto), and
        it is a speed sample if it is at least SPEED_WINDOW long."""
        t0, x0, y0 = self.speed_ref
        d = math.hypot(x - x0, y - y0)
        if self.acc is not None:
            self.acc.distance += d
            if t - t0 >= SPEED_WINDOW:
                self.acc.max_speed = max(self.acc.max_speed, d / (t - t0))
        self.speed_ref = (t, x, y)

    def abort(self, reason):
        """End the lap being timed as aborted (returned, None if none) and start over."""
        lap = None
        if self.acc is not None:
            lap = self._close(self.prev[0], 'aborted', reason)
        self._reset()
        return lap

    def _close(self, t, status, note=''):
        a = self.acc
        offs = a.offsets or [0.0]
        lap = Lap(len(self.laps) + 1, status, a.start, t - a.t0, a.distance,
                  float(np.mean(offs)), float(np.max(offs)), a.max_speed, len(a.jumps), max(a.jumps, default=0.0),
                  note)
        self.laps.append(lap)
        self.acc = None
        return lap


def summary(laps):
    """One line: laps finished and aborted, best and mean time of the finished ones."""
    ok = [lap.time_s for lap in laps if lap.status == 'ok']
    flying = [lap.time_s for lap in laps if lap.status == 'ok' and lap.start == 'flying']
    aborted = sum(lap.status == 'aborted' for lap in laps)
    if not ok:
        return f'no finished laps, {aborted} aborted'
    text = f'{len(ok)} laps, best {min(ok):.2f} s'
    if flying:
        text += f', flying mean {sum(flying) / len(flying):.2f} s'
    return text + f', {aborted} aborted'
