import subprocess
import sys
import time

import pytest
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState

from autodrive_bridge.wheel_odom import WheelOdom

R = 0.0585   # wheel_radius default


class Recorder:
    def __init__(self):
        self.data = []

    def publish(self, msg):
        self.data.append(msg)


def wait_for(cond, node, timeout=5.0):
    """Spin node until cond() (graph discovery takes a moment, even in one process)."""
    end = time.monotonic() + timeout
    while not cond():
        assert time.monotonic() < end, 'timed out'
        rclpy.spin_once(node, timeout_sec=0.05)


def frames(node, t0, n):
    """n sim frames, 1 cm apart, each with its IMU."""
    for i in range(n):
        t = t0 + 0.01 * i
        stamp = Time(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))
        for cb in (node.left_cb, node.right_cb):
            js = JointState()
            js.header.stamp = stamp
            js.position = [0.01 * i / R]
            cb(js)
        imu = Imu()
        imu.header.stamp = stamp
        node.imu_cb(imu)


@pytest.fixture
def wheel_odom():
    n = WheelOdom()
    n.guard.delay = 0.0
    yield n
    n.destroy_node()


def other_odom():
    other = rclpy.create_node('bag_player')
    other.create_publisher(Odometry, '/odom', 10)
    return other


def test_alone_publishes(wheel_odom):
    n = wheel_odom
    n.odom_pub = Recorder()
    frames(n, 1.0, 3)
    assert n.odom_pub.data == []   # before the first check
    n.guard.check()
    assert n.guard.ok
    frames(n, 2.0, 3)
    assert len(n.odom_pub.data) == 3


def test_second_wheel_odom_holds_until_the_first_stops(wheel_odom):
    n = wheel_odom
    other = other_odom()
    try:
        wait_for(lambda: len(n.get_publishers_info_by_topic('/odom')) == 2, n)
        n.guard.check()
        assert not n.guard.ok
        n.odom_pub = Recorder()
        frames(n, 1.0, 10)
        assert n.odom_pub.data == []
        assert n.x == pytest.approx(0.09)   # still integrating
    finally:
        other.destroy_node()
    wait_for(lambda: len(n.get_publishers_info_by_topic('/odom')) == 1, n)
    n.guard.check()
    assert n.guard.ok
    frames(n, 2.0, 3)
    assert n.odom_pub.data


def test_running_wheel_odom_keeps_publishing(wheel_odom):
    n = wheel_odom
    n.guard.check()
    assert n.guard.ok
    other = other_odom()
    try:
        wait_for(lambda: len(n.get_publishers_info_by_topic('/odom')) == 2, n)
        n.guard.check()   # logs the other one
        assert n.guard.ok
    finally:
        other.destroy_node()


def test_guard_waits_for_discovery():
    n = WheelOdom()   # alone, but not ok before the discovery delay
    try:
        n.guard.check()
        assert not n.guard.ok
    finally:
        n.destroy_node()


@pytest.mark.skipif(not sys.platform.startswith('linux'), reason='PR_SET_PDEATHSIG is Linux only')
def test_exit_with_parent():
    # parent -> child; the child asks for SIGTERM when the parent dies, then sleeps
    child = ('from autodrive_bridge.single_instance import exit_with_parent; import time; '
             'assert exit_with_parent(); print("ready", flush=True); time.sleep(30)')
    parent = ('import subprocess, sys; '
              f'c = subprocess.Popen([sys.executable, "-c", {child!r}], stdout=subprocess.PIPE, text=True); '
              'c.stdout.readline(); print(c.pid, flush=True); c.wait()')
    p = subprocess.Popen([sys.executable, '-c', parent], stdout=subprocess.PIPE, text=True)
    pid = int(p.stdout.readline())
    p.kill()   # SIGKILL: the parent gets no chance to stop the child
    p.wait()
    assert wait_gone([pid], 5.0), 'child outlived its parent'


def alive(pid):
    try:
        with open(f'/proc/{pid}/stat') as f:
            return f.read().rsplit(')', 1)[1].split()[0] != 'Z'   # a zombie has exited
    except FileNotFoundError:
        return False


def wait_gone(pids, timeout):
    end = time.monotonic() + timeout
    while any(alive(p) for p in pids):
        if time.monotonic() > end:
            return False
        time.sleep(0.05)
    return True


def test_exit_with_parent_no_op_off_linux(monkeypatch):
    from autodrive_bridge import single_instance
    monkeypatch.setattr(single_instance.sys, 'platform', 'darwin')
    assert single_instance.exit_with_parent() is False
