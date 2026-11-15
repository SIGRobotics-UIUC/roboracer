"""Testable monotonic lifecycle policies, independent of ROS and disk I/O."""
import math


class Health:
    def __init__(self, essential, loss_s=1.0, overrides=None):
        self.essential = set(essential)
        self.loss_s = loss_s
        self.overrides = overrides or {}
        self.seen = {}
        self.gaps = []

    def observe(self, signal, now):
        previous = self.seen.get(signal)
        if previous is not None and now - previous > 0.1:
            self.gaps.append({'signal': signal, 'duration_s': now - previous, 'monotonic_s': now})
        self.seen[signal] = now

    def missing(self, now):
        return sorted(key for key in self.essential
                      if now - self.seen.get(key, -math.inf) > self.overrides.get(key, self.loss_s))


class GatePolicy:
    """A fault latches closed; repeated enable heartbeats cannot rearm it."""
    def __init__(self, timeout=1.0, command_timeout=None):
        self.timeout = timeout
        self.command_timeout = timeout if command_timeout is None else command_timeout
        self.enabled = False
        self.fault = None
        self.heartbeat = None
        self.command = None

    def control(self, enable, now):
        self.heartbeat = now
        if not enable:
            if self.enabled:
                self.fault = self.fault or 'lifecycle stop'
            self.enabled = False
        elif not self.fault:
            self.enabled = True

    def check(self, now):
        if self.enabled and (self.heartbeat is None or now - self.heartbeat > self.timeout):
            self.fault = 'manager heartbeat lost'
        if self.enabled and not self.fault and (self.command is None or now - self.command > self.command_timeout):
            self.fault = 'controller command lost'
        if self.fault:
            self.enabled = False
        return self.enabled


class RestConfirmation:
    def __init__(self, speed=0.05, dwell=0.5, hold=1.5, timeout=5.0):
        self.speed, self.dwell, self.hold, self.timeout = speed, dwell, hold, timeout
        self.rest_since = None

    def update(self, elapsed, now, velocity, received, freshness=1.0):
        fresh = received is not None and now - received <= freshness
        rest = fresh and velocity is not None and math.isfinite(velocity) and abs(velocity) < self.speed
        if not rest:
            self.rest_since = None
        elif self.rest_since is None:
            self.rest_since = now
        confirmed = (elapsed >= self.hold and self.rest_since is not None
                     and now - self.rest_since >= self.dwell)
        return 'confirmed' if confirmed else 'unconfirmed' if elapsed >= self.timeout else 'waiting'
