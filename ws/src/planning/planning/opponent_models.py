"""Ground-truth opponent motion, and a sensor model that degrades it into
something perception could plausibly have reported.

The two halves are deliberately separate. A motion model says where the
opponent really is; a sensor model says what we would have seen. Publishing
ground truth directly is the tempting shortcut and the wrong one, because the
Gaussian process inside both overtaking planners exists specifically to turn
noisy, intermittent sightings into a predicted trajectory. Given clean data it
has nothing to do, and a comparison between two such planners would measure
nothing. The degradation is the experiment, not decoration on it.

Nothing here imports rclpy, so it runs under plain pytest without a ROS
environment. The node in fake_opponent_node.py is a thin wrapper over this.
"""

from dataclasses import dataclass

import numpy as np

# Variance consumers divide by, so it must never be exactly zero even when the
# publisher is noiseless.
VAR_FLOOR = 1e-6


def signed_gap(s_from, s_to, lap_length):
    """Signed lap distance from s_from to s_to, positive when s_to is ahead.

    Mirrors FrenetConverter.ds. Duplicated rather than imported so this module
    stays free of the converter and its file-loading machinery; test_opponent_models
    asserts the two agree.
    """
    half = lap_length / 2.0
    return (s_to - s_from + half) % lap_length - half


@dataclass(frozen=True)
class OpponentState:
    """Where the opponent truly is. Not what anyone observed."""
    s: float
    d: float
    vs: float
    vd: float


# --------------------------------------------------------------------------
# Motion models
# --------------------------------------------------------------------------

class MotionModel:
    """Ground truth as a function of time, in seconds since the node started."""

    is_static = False

    def state_at(self, t):
        raise NotImplementedError


class StaticObstacle(MotionModel):
    """A cone, a parked car, debris. The case Ewan's planner branch needs."""

    is_static = True

    def __init__(self, s, d=0.0):
        self.s = float(s)
        self.d = float(d)

    def state_at(self, t):
        return OpponentState(self.s, self.d, 0.0, 0.0)


class ConstantSpeed(MotionModel):
    """Holds a fixed lateral offset at a fixed speed. The simplest moving case."""

    def __init__(self, lap_length, speed, s0=0.0, d=0.0):
        self.lap_length = float(lap_length)
        self.speed = float(speed)
        self.s0 = float(s0)
        self.d = float(d)

    def state_at(self, t):
        s = (self.s0 + self.speed * t) % self.lap_length
        return OpponentState(s, self.d, self.speed, 0.0)


class Weaving(MotionModel):
    """Lateral offset is a sine in s: d = amplitude * sin(2*pi*s / wavelength).

    Making d a function of arc length rather than of time is the whole point.
    It gives the opponent a line that repeats lap after lap, which is the premise
    both papers rest on when they learn an opponent trajectory from past laps.
    The wavelength is snapped so a whole number of cycles fits the lap, otherwise
    d jumps at the start/finish line and the GP is being asked to learn a
    discontinuity that no real driver would produce.
    """

    def __init__(self, lap_length, speed, amplitude, wavelength, s0=0.0):
        self.lap_length = float(lap_length)
        self.speed = float(speed)
        self.amplitude = float(amplitude)
        cycles = max(1, int(round(self.lap_length / float(wavelength))))
        self.wavelength = self.lap_length / cycles
        self.cycles = cycles
        self.s0 = float(s0)

    def state_at(self, t):
        s = (self.s0 + self.speed * t) % self.lap_length
        k = 2.0 * np.pi / self.wavelength
        d = self.amplitude * np.sin(k * s)
        # chain rule: vd = (dd/ds) * (ds/dt)
        vd = self.amplitude * k * np.cos(k * s) * self.speed
        return OpponentState(s, float(d), self.speed, float(vd))


class Replay(MotionModel):
    """Drive a recorded line, given as samples of d against s."""

    def __init__(self, lap_length, s_samples, d_samples, speed, s0=0.0):
        self.lap_length = float(lap_length)
        self.speed = float(speed)
        self.s0 = float(s0)
        s_arr = np.asarray(s_samples, dtype=float)
        d_arr = np.asarray(d_samples, dtype=float)
        if s_arr.size < 2 or s_arr.size != d_arr.size:
            raise ValueError('replay needs at least two matching (s, d) samples')
        order = np.argsort(s_arr)
        s_arr, d_arr = s_arr[order], d_arr[order]
        # close the loop so interpolation across the start line is continuous
        self._s = np.concatenate((s_arr, [s_arr[0] + self.lap_length]))
        self._d = np.concatenate((d_arr, [d_arr[0]]))

    def _d_at(self, s):
        return float(np.interp(s % self.lap_length, self._s, self._d))

    def state_at(self, t):
        s = (self.s0 + self.speed * t) % self.lap_length
        eps = 1e-3
        dd_ds = (self._d_at(s + eps) - self._d_at(s - eps)) / (2.0 * eps)
        return OpponentState(s, self._d_at(s), self.speed, dd_ds * self.speed)


def motion_from_params(mode, lap_length, speed=2.0, s0=0.0, d=0.0,
                       amplitude=0.4, wavelength=12.0,
                       replay_s=None, replay_d=None):
    """Build a motion model from flat node parameters."""
    mode = str(mode).lower()
    if mode == 'static':
        return StaticObstacle(s0, d)
    if mode == 'constant_speed':
        return ConstantSpeed(lap_length, speed, s0, d)
    if mode == 'weaving':
        return Weaving(lap_length, speed, amplitude, wavelength, s0)
    if mode == 'replay':
        if replay_s is None or replay_d is None:
            raise ValueError("mode 'replay' needs replay_s and replay_d")
        return Replay(lap_length, replay_s, replay_d, speed, s0)
    raise ValueError(f'unknown opponent mode {mode!r}')


# --------------------------------------------------------------------------
# Sensor model
# --------------------------------------------------------------------------

@dataclass
class SensorModel:
    """What our perception stack would have made of the truth.

    Every field defaults to the ideal, so a fresh instance is a perfect oracle
    and difficulty gets dialled in one parameter at a time. Turn the noise on
    before believing any planner result.
    """

    max_range: float = 10.0      # how far ahead a detection is possible
    rear_range: float = 0.0      # how far behind; 0 means blind astern
    detection_prob: float = 1.0  # per-cycle probability of reporting at all
    sigma_s: float = 0.0
    sigma_d: float = 0.0
    sigma_vs: float = 0.0
    sigma_vd: float = 0.0
    latency: float = 0.0         # seconds; the report describes the past
    length: float = 0.50         # opponent footprint, metres
    width: float = 0.30
    inflation: float = 0.05      # covers the Frenet box being wrong in corners


@dataclass(frozen=True)
class ObstacleObservation:
    """One obstacle as perception would report it. Field for field onto
    f1t_msgs/Obstacle, in the same order, so the node copies rather than
    computes. There is no is_opponent: see the note in Obstacle.msg."""
    id: int
    s_start: float
    s_end: float
    d_right: float
    d_left: float
    is_actually_a_gap: bool
    s_center: float
    d_center: float
    size: float
    vs: float
    vd: float
    s_var: float
    d_var: float
    vs_var: float
    vd_var: float
    is_static: bool
    is_visible: bool


def in_range(truth_s, ego_s, lap_length, sensor):
    gap = signed_gap(ego_s, truth_s, lap_length)
    return -sensor.rear_range <= gap <= sensor.max_range


def observe(motion, t, ego_s, lap_length, sensor, rng, obstacle_id=0):
    """Return what perception reports this cycle, or None for nothing seen.

    The range gate is applied to the opponent's delayed position against the
    ego's *current* position. Strictly the gate belongs at the moment of
    detection, but ego's position then is not available here and the difference
    is well under a centimetre at the latencies worth simulating.
    """
    state = motion.state_at(t - sensor.latency)

    if not in_range(state.s, ego_s, lap_length, sensor):
        return None
    if sensor.detection_prob < 1.0 and rng.random() >= sensor.detection_prob:
        return None

    s_center = (state.s + rng.normal(0.0, sensor.sigma_s)) % lap_length
    d_center = state.d + rng.normal(0.0, sensor.sigma_d)
    vs = state.vs + rng.normal(0.0, sensor.sigma_vs)
    vd = state.vd + rng.normal(0.0, sensor.sigma_vd)

    half_l = sensor.length / 2.0 + sensor.inflation
    half_w = sensor.width / 2.0 + sensor.inflation

    s_start = s_center - half_l
    s_end = s_center + half_l
    # Keep s_start < s_end across the start line by letting s_end run past the
    # lap length, rather than wrapping it back to near zero. Downstream code
    # sorts and takes min/max over these as plain numbers.
    if s_start < 0.0:
        s_start += lap_length
        s_end += lap_length

    return ObstacleObservation(
        id=obstacle_id,
        s_start=s_start,
        s_end=s_end,
        d_right=d_center - half_w,
        d_left=d_center + half_w,
        is_actually_a_gap=False,
        s_center=s_center,
        d_center=d_center,
        size=float(np.hypot(2.0 * half_l, 2.0 * half_w)),
        vs=vs,
        vd=vd,
        s_var=max(sensor.sigma_s ** 2, VAR_FLOOR),
        d_var=max(sensor.sigma_d ** 2, VAR_FLOOR),
        vs_var=max(sensor.sigma_vs ** 2, VAR_FLOOR),
        vd_var=max(sensor.sigma_vd ** 2, VAR_FLOOR),
        is_static=motion.is_static,
        is_visible=True,
    )
