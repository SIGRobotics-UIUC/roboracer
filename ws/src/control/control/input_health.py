"""Clock/heading consistency and recovery latch, independent of ROS.

Heading is propagated with fresh measured yaw rate on monotonic time while input
poses are rejected. Stable but wrong poses cannot re-arm just by being repeated.
"""
import math


def angle_delta(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


class InputHealth:
    def __init__(self, recovery_seconds=.25, recovery_samples=3,
                 heading_tolerance=.25, clock_tolerance=.25):
        if (not all(math.isfinite(v) and v > 0 for v in
                    (recovery_seconds, recovery_samples, heading_tolerance, clock_tolerance)) or
                int(recovery_samples) != recovery_samples):
            raise ValueError('invalid input recovery configuration')
        self.recovery_seconds = recovery_seconds
        self.recovery_samples = recovery_samples
        self.heading_tolerance = heading_tolerance
        self.clock_tolerance = clock_tolerance
        self.reset()

    def reset(self):
        self.last_clock = None
        self.expected_yaw = None
        self.recovering = True
        self.valid_since = None
        self.valid_count = 0
        self.last_pair = None

    def fault(self):
        self.recovering = True
        self.valid_since = None
        self.valid_count = 0
        self.last_pair = None

    def advance(self, ros_now, monotonic_now, yaw_rate=0., gyro_fresh=False):
        reason = None
        if self.last_clock is not None:
            ros_dt = ros_now-self.last_clock[0]
            steady_dt = monotonic_now-self.last_clock[1]
            if self.expected_yaw is not None and gyro_fresh and math.isfinite(yaw_rate):
                self.expected_yaw += yaw_rate*max(0., min(steady_dt, .1))
            if self.last_clock[0] > 0 and abs(ros_dt-steady_dt) > self.clock_tolerance:
                self.fault()
                reason = 'clock discontinuity'
        self.last_clock = (ros_now, monotonic_now)
        return reason

    def accept(self, yaw, pair, monotonic_now):
        if self.expected_yaw is None:
            self.expected_yaw = yaw
        if self.expected_yaw is not None and abs(angle_delta(yaw,self.expected_yaw)) > self.heading_tolerance:
            self.fault()
            return 'heading inconsistent with yaw-rate propagation'
        if self.recovering:
            if self.valid_since is None:
                self.valid_since = monotonic_now
            if self.last_pair is None or (pair[0] > self.last_pair[0] and pair[1] > self.last_pair[1]):
                self.valid_count += 1
                self.last_pair = pair
            if (monotonic_now-self.valid_since < self.recovery_seconds or
                    self.valid_count < self.recovery_samples):
                return 'waiting for consistent recovery samples'
            self.recovering = False
        self.expected_yaw = yaw
        return None
