import os
from pathlib import Path
import signal
import sys
import time

from benchmark.collector.lifecycle import GatePolicy, Health, RestConfirmation
from benchmark.collector.process import Process
from benchmark.collector.topics import valid, NS


def test_native_unset_lap_time_is_stream_presence_not_a_finite_measurement():
    from types import SimpleNamespace
    assert valid(NS+'last_lap_time', SimpleNamespace(data=float('inf')))
    assert not valid(NS+'last_lap_time', SimpleNamespace(data=float('nan')))
    assert not valid(NS+'throttle_command', SimpleNamespace(data=float('inf')))


def test_jitter_tolerated_persistent_loss_not_fabricated():
    health = Health({'pose', 'odom'}, 1., {'odom': 0.5})
    health.observe('pose', 0.)
    health.observe('odom', 0.)
    assert health.missing(0.2) == []
    health.observe('pose', 0.3)
    assert health.gaps[0]['duration_s'] == 0.3
    assert health.missing(0.51) == ['odom']
    assert health.missing(1.31) == ['odom', 'pose']


def test_gate_readiness_stop_abort_and_frozen_ros_time():
    gate = GatePolicy(1.)
    assert not gate.check(0.)
    gate.command = 0.
    gate.control(True, 0.)
    assert gate.check(0.2)
    # This clock is monotonic and independent of frozen simulation/ROS time.
    assert not gate.check(1.01)
    assert gate.fault == 'manager heartbeat lost'
    gate.command = 1.02
    gate.control(True, 1.02)
    assert not gate.check(1.03)
    gate = GatePolicy(1.)
    gate.command = 0.
    gate.control(True, 0.)
    gate.control(False, 0.1)
    gate.control(True, 0.2)
    assert not gate.check(0.2)


def test_stop_confirmation_requires_fresh_state_hold_and_dwell():
    confirmation = RestConfirmation()
    assert confirmation.update(0., 0., 0., 0.) == 'waiting'
    assert confirmation.update(0.6, 0.6, 0., 0.6) == 'waiting'
    assert confirmation.update(1.5, 1.5, 0., 1.5) == 'confirmed'
    assert confirmation.update(1.6, 1.6, 0., 0.) == 'waiting'
    assert confirmation.update(5., 5., None, None) == 'unconfirmed'


def test_owned_process_group_cleanup_including_child(tmp_path):
    child_pid = tmp_path/'child.pid'
    code = ('import subprocess,time,pathlib; '
            'p=subprocess.Popen(["sleep","60"]); '
            f'pathlib.Path({str(child_pid)!r}).write_text(str(p.pid)); time.sleep(60)')
    proc = Process([sys.executable, '-c', code], tmp_path/'process.log')
    deadline = time.monotonic()+3
    while not child_pid.exists() and time.monotonic() < deadline:
        time.sleep(.02)
    assert child_pid.exists()
    result = proc.close()
    assert not result['group_alive']
