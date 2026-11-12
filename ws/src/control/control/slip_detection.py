"""Slip-detection math independent of ROS.

The detector compares rate-limited commanded motion with motion measured from
successive localization poses.  It intentionally contains no tire or vehicle
dynamics model; the only geometric parameter is wheelbase.
"""
from dataclasses import dataclass
import math


def angle_delta(a, b):
    """Shortest signed angle from b to a."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def move_towards(current, target, max_change):
    """Move current toward target by at most a nonnegative max_change."""
    return current + max(-max_change, min(max_change, target - current))


@dataclass(frozen=True)
class Motion:
    speed: float
    yaw_rate: float
    dt: float


@dataclass(frozen=True)
class SlipResult:
    expected_speed: float
    expected_yaw_rate: float
    linear_error: float
    angular_error: float
    linear_slip: bool
    angular_slip: bool
    slipping: bool


class PoseMotionEstimator:
    """Estimate body-forward speed and yaw rate from timestamped map poses."""

    def __init__(self, smoothing=0.35, min_dt=0.005, max_dt=0.25,
                 max_speed=20.0, max_yaw_rate=12.0):
        values = (smoothing, min_dt, max_dt, max_speed, max_yaw_rate)
        if (not all(math.isfinite(v) for v in values) or
                not 0 < smoothing <= 1 or not 0 < min_dt < max_dt or
                max_speed <= 0 or max_yaw_rate <= 0):
            raise ValueError('invalid pose-motion estimator configuration')
        self.smoothing = smoothing
        self.min_dt = min_dt
        self.max_dt = max_dt
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.reset()

    def reset(self):
        self.previous = None
        self.filtered = None

    def update(self, stamp, x, y, yaw):
        sample = (stamp, x, y, yaw)
        if not all(math.isfinite(v) for v in sample):
            self.reset()
            return None
        if self.previous is None:
            self.previous = sample
            return None

        old_stamp, old_x, old_y, old_yaw = self.previous
        self.previous = sample
        dt = stamp - old_stamp
        if not self.min_dt <= dt <= self.max_dt:
            self.filtered = None
            return None

        dyaw = angle_delta(yaw, old_yaw)
        mid_yaw = old_yaw + 0.5 * dyaw
        dx, dy = x - old_x, y - old_y
        speed = (dx * math.cos(mid_yaw) + dy * math.sin(mid_yaw)) / dt
        yaw_rate = dyaw / dt
        if abs(speed) > self.max_speed or abs(yaw_rate) > self.max_yaw_rate:
            self.filtered = None
            return None

        if self.filtered is None:
            filtered_speed, filtered_yaw_rate = speed, yaw_rate
        else:
            a = self.smoothing
            filtered_speed = a * speed + (1 - a) * self.filtered[0]
            filtered_yaw_rate = a * yaw_rate + (1 - a) * self.filtered[1]
        self.filtered = (filtered_speed, filtered_yaw_rate)
        return Motion(filtered_speed, filtered_yaw_rate, dt)


class CommandMotion:
    """Reproduce the actuator speed-reference ramp before comparison."""

    def __init__(self, max_acceleration=2.5, max_deceleration=6.9):
        if (not all(math.isfinite(v) and v > 0 for v in
                    (max_acceleration, max_deceleration))):
            raise ValueError(
                'command acceleration limits must be finite and positive')
        self.max_acceleration = max_acceleration
        self.max_deceleration = max_deceleration
        self.expected_speed = None

    def reset(self):
        self.expected_speed = None

    def update(self, target_speed, measured_speed, dt):
        values = (target_speed, measured_speed, dt)
        if not all(math.isfinite(v) for v in values) or dt <= 0:
            self.reset()
            return None
        target_speed = max(0.0, target_speed)
        if self.expected_speed is None:
            # Starting from measured motion prevents a normal mid-run monitor
            # startup from being classified as slip.
            self.expected_speed = max(0.0, measured_speed)
        rate = (self.max_acceleration if target_speed > self.expected_speed
                else self.max_deceleration)
        self.expected_speed = move_towards(
            self.expected_speed, target_speed, rate * dt)
        return self.expected_speed


class SlipDetector:
    """Threshold the paper's linear and angular command-tracking residuals."""

    def __init__(self, wheelbase, linear_threshold, angular_threshold,
                 residual_smoothing=0.35, trigger_samples=2, clear_samples=3):
        floats = (
            wheelbase, linear_threshold, angular_threshold, residual_smoothing)
        ints = (trigger_samples, clear_samples)
        if (not all(math.isfinite(v) and v > 0 for v in floats[:3]) or
                not math.isfinite(residual_smoothing) or
                not 0 < residual_smoothing <= 1 or
                not all(isinstance(v, int) and v >= 1 for v in ints)):
            raise ValueError('invalid slip-detector configuration')
        self.wheelbase = wheelbase
        self.linear_threshold = linear_threshold
        self.angular_threshold = angular_threshold
        self.residual_smoothing = residual_smoothing
        self.trigger_samples = trigger_samples
        self.clear_samples = clear_samples
        self.reset()

    def reset(self):
        self.filtered_errors = None
        self.over_count = 0
        self.clear_count = 0
        self.slipping = False

    def update(self, expected_speed, steering, measured_speed,
               measured_yaw_rate):
        values = (
            expected_speed, steering, measured_speed, measured_yaw_rate)
        if not all(math.isfinite(v) for v in values):
            self.reset()
            return None

        expected_yaw_rate = (
            expected_speed * math.tan(steering) / self.wheelbase)
        linear_error = abs(expected_speed - measured_speed)
        angular_error = abs(expected_yaw_rate - measured_yaw_rate)
        if self.filtered_errors is None:
            filtered_linear, filtered_angular = linear_error, angular_error
        else:
            a = self.residual_smoothing
            filtered_linear = (
                a * linear_error + (1 - a) * self.filtered_errors[0])
            filtered_angular = (
                a * angular_error + (1 - a) * self.filtered_errors[1])
        self.filtered_errors = (filtered_linear, filtered_angular)

        linear_slip = filtered_linear >= self.linear_threshold
        angular_slip = filtered_angular >= self.angular_threshold
        over = linear_slip or angular_slip
        if over:
            self.over_count += 1
            self.clear_count = 0
            if self.over_count >= self.trigger_samples:
                self.slipping = True
        else:
            self.over_count = 0
            self.clear_count += 1
            if self.clear_count >= self.clear_samples:
                self.slipping = False

        return SlipResult(
            expected_speed, expected_yaw_rate, filtered_linear,
            filtered_angular, linear_slip, angular_slip, self.slipping)


class FrictionEstimator:
    """Maximum planar acceleration / gravity during valid no-slip motion."""

    def __init__(self, gravity=9.80665):
        if not math.isfinite(gravity) or gravity <= 0:
            raise ValueError('gravity must be finite and positive')
        self.gravity = gravity
        self.reset()

    def reset(self):
        self.estimate = None

    def update(self, ax, ay, valid=True, slipping=False):
        if (not valid or slipping or
                not all(math.isfinite(v) for v in (ax, ay))):
            return self.estimate
        sample = math.hypot(ax, ay) / self.gravity
        self.estimate = (sample if self.estimate is None
                         else max(self.estimate, sample))
        return self.estimate
