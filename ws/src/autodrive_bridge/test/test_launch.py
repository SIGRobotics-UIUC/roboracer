"""autodrive_launch.py as a whole: how the launch and its nodes stop.

Runs `ros2 launch` on bridge_nodes_launch.py (fast_bridge, adapter, wheel_odom) in its own session,
ROS domain 77 and on a free port, so it cannot reach a running bridge or sim.
"""
import os
import signal
import socket
import subprocess
import sys
import time

import pytest

pytestmark = pytest.mark.skipif(not sys.platform.startswith('linux'), reason='reads /proc')

HERE = os.path.dirname(os.path.abspath(__file__))
NODES = {'fast_bridge', 'adapter', 'wheel_odom'}


def free_port():
    with socket.socket() as s:
        s.bind(('', 0))
        return s.getsockname()[1]


def children(pid):
    """{pid: executable name} of pid's child processes (the script, not python3)."""
    out = {}
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        try:
            with open(f'/proc/{d}/stat') as f:
                fields = f.read().rsplit(')', 1)[1].split()
            if int(fields[1]) == pid and fields[0] != 'Z':
                with open(f'/proc/{d}/cmdline', 'rb') as f:
                    names = [os.path.basename(a.decode()) for a in f.read().split(b'\0')[:2]]
                out[int(d)] = next((n for n in names if n in NODES), names[0])
        except (FileNotFoundError, ProcessLookupError, IndexError):
            pass
    return out


def alive(pid):
    try:
        with open(f'/proc/{pid}/stat') as f:
            return f.read().rsplit(')', 1)[1].split()[0] != 'Z'
    except FileNotFoundError:
        return False


def wait_until(cond, timeout):
    end = time.monotonic() + timeout
    while not cond():
        if time.monotonic() > end:
            return False
        time.sleep(0.1)
    return True


def launch(port):
    env = dict(os.environ, ROS_DOMAIN_ID='77', PYTHONUNBUFFERED='1')
    return subprocess.Popen(['ros2', 'launch', os.path.join(HERE, 'bridge_nodes_launch.py'), f'port:={port}'],
                            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            start_new_session=True)


def started(p, timeout=20.0):
    """The launch's node processes, once all of NODES run and fast_bridge listens."""
    nodes = {}

    def up():
        nodes.update(children(p.pid))
        return NODES <= set(nodes.values())
    assert wait_until(up, timeout), f'nodes did not start: {nodes}'
    time.sleep(3.0)   # imports, rclpy init, the bind
    return nodes


def finish(p):
    """Its output; kills what is left."""
    try:
        os.killpg(p.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    out, _ = p.communicate(timeout=10)
    return out


def test_fast_bridge_ends_the_launch():
    port = free_port()
    with socket.socket() as busy:   # the old bridge still holds the port
        busy.bind(('', port))
        busy.listen()
        p = launch(port)
        try:
            nodes = {}
            wait_until(lambda: nodes.update(children(p.pid)) or NODES <= set(nodes.values()), 20.0)
            assert p.wait(timeout=30) is not None
            assert wait_until(lambda: not any(alive(c) for c in nodes), 5.0), 'nodes outlived the launch'
        finally:
            out = finish(p)
    assert 'fast_bridge-1] was required: shutting down' in out, out
    assert {'adapter', 'wheel_odom'} <= set(nodes.values()), out


def test_nodes_exit_when_the_launch_is_killed():
    p = launch(free_port())
    try:
        nodes = started(p)
        os.kill(p.pid, signal.SIGKILL)   # the launch alone: nothing forwards a signal to the nodes
        p.wait(timeout=5)
        assert wait_until(lambda: not any(alive(c) for c in nodes), 5.0), \
            f'orphaned: {[n for c, n in nodes.items() if alive(c)]}'
    finally:
        finish(p)


def test_ctrl_c_stops_cleanly():
    # Ctrl-C in the terminal: SIGINT to the whole process group. fast_bridge's on_exit Shutdown must
    # not change this: every node stops on its own SIGINT, none needs the SIGTERM escalation.
    p = launch(free_port())
    try:
        nodes = started(p)
        os.killpg(p.pid, signal.SIGINT)
        assert p.wait(timeout=15) == 0
        assert not any(alive(c) for c in nodes)
    finally:
        out = finish(p)
    assert 'escalating' not in out, out
    for name in NODES:
        assert any(f'[{name}-' in line and 'finished cleanly' in line for line in out.splitlines()), out
