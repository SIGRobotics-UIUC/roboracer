"""Benchmark-only wheel-slip stand-in for the simulator's wheel odometry.

The simulator's encoders and gyro are close to perfect, so localizers are never tested on the odometry
of a car whose tyres slip. OdomDegrader corrupts each odometry step the way slip does:

  distance   ds * (1 + scale_bias + e_s)     wheelspin / lock-up: the wheels turn more (or less) than the
                                             car moves; e_s drifts slowly (Ornstein-Uhlenbeck, std
                                             scale_std, time constant tau)
  yaw        dyaw * (1 + yaw_bias + e_y)     the yaw source over- or under-reads in corners (a bicycle model
                                             when the car understeers); e_y like e_s, std yaw_std
  sideways   lateral * speed * dyaw          the car slides out of a corner, which wheel odometry never
                                             sees: the odometry moves inward by this much relative to the car

All off (zero) by default: then step() returns its input unchanged. Only the localization benchmark
(scripts/localization_benchmark.py --slip) turns it on, through wheel_odom's slip_* parameters.
"""
import math

import numpy as np

PARAMS = ('scale_bias', 'scale_std', 'yaw_bias', 'yaw_std', 'lateral', 'tau')


class OdomDegrader:
    def __init__(self, scale_bias=0.0, scale_std=0.0, yaw_bias=0.0, yaw_std=0.0, lateral=0.0, tau=0.5, seed=0):
        self.scale_bias, self.scale_std = float(scale_bias), float(scale_std)
        self.yaw_bias, self.yaw_std = float(yaw_bias), float(yaw_std)
        self.lateral, self.tau = float(lateral), max(1e-3, float(tau))
        self.rng = np.random.default_rng(int(seed))
        # start each drifting error from its stationary distribution
        self.e_s = self.rng.normal(0.0, self.scale_std) if self.scale_std > 0 else 0.0
        self.e_y = self.rng.normal(0.0, self.yaw_std) if self.yaw_std > 0 else 0.0

    @property
    def active(self):
        return any((self.scale_bias, self.scale_std, self.yaw_bias, self.yaw_std, self.lateral))

    def _drift(self, e, std, dt):
        """One Ornstein-Uhlenbeck step: decays towards 0 with time constant tau, stationary std `std`."""
        if std <= 0:
            return 0.0
        a = math.exp(-dt / self.tau)
        return a * e + std * math.sqrt(1.0 - a * a) * self.rng.normal()

    def step(self, ds, dyaw, dt):
        """Odometry step (ds forward [m], dyaw [rad] over dt [s]) -> (forward, sideways [m], yaw [rad])."""
        if not self.active or dt <= 0:
            return ds, 0.0, dyaw
        self.e_s = self._drift(self.e_s, self.scale_std, dt)
        self.e_y = self._drift(self.e_y, self.yaw_std, dt)
        speed = ds / dt
        return (ds * (1.0 + self.scale_bias + self.e_s),
                self.lateral * speed * dyaw,
                dyaw * (1.0 + self.yaw_bias + self.e_y))
