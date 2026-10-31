"""Epoch-shaped simulation time driven solely by elapsed monotonic time.

System time is sampled once at construction for readable timestamps. Subsequent
host clock corrections cannot turn one received frame into seconds of motion.
This is a transport clock, not a claim to expose simulator physics time.
"""
import time


class SteadyEpoch:
    def __init__(self, wall_ns=None, monotonic_ns=None):
        self.epoch_ns = time.time_ns() if wall_ns is None else wall_ns
        self.start_ns = time.monotonic_ns() if monotonic_ns is None else monotonic_ns

    def now_ns(self, monotonic_ns=None):
        current = time.monotonic_ns() if monotonic_ns is None else monotonic_ns
        return self.epoch_ns + max(0, current-self.start_ns)
