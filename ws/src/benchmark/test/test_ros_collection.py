"""Synthetic ROS collection only: no AutoDRIVE connection or default-domain commands."""
import json
import os
from pathlib import Path
import subprocess
import time

import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('rosbag2_py')
pytest.importorskip('ackermann_msgs')
from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float32, Int32, String
from rclpy.clock import Clock, ClockType
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from benchmark.collector.artifacts import verify
from benchmark.analysis.runner import analyze

NS = '/autodrive/roboracer_1/'


@pytest.fixture
def simulator(tmp_path):
    """A deterministic interface fixture, not a model of simulator vehicle dynamics."""
    rclpy.init(domain_id=68)
    node = rclpy.create_node('fast_bridge', parameter_overrides=[
        Parameter('publish_clock', value=True), Parameter('use_sim_time', value=True),
        Parameter('publish_restricted_topics', value=True)])
    node.declare_parameter('publish_clock', True)
    node.declare_parameter('publish_restricted_topics', True)
    kinds = {'/clock': ClockMessage, '/localized_pose': PoseStamped, '/odom': Odometry,
             '/scan': LaserScan, '/sensors/imu/raw': Imu,
             '/benchmark/requested_drive': AckermannDriveStamped,
             NS+'lap_count': Int32, NS+'collision_count': Int32, NS+'last_lap_time': Float32,
             NS+'sim_reset': String, NS+'throttle_command': Float32, NS+'steering_command': Float32}
    pubs = {name: node.create_publisher(kind, name, QoSProfile(depth=10, reliability=
            ReliabilityPolicy.BEST_EFFORT if name == '/clock' else ReliabilityPolicy.RELIABLE)) for name, kind in kinds.items()}
    state = {'lap': 0, 'collision': 0, 'speed': 0., 'enabled': False, 'last_lap': None,
             'loss': False, 'drive_values': [], 'status': None, 'reset_seen': False}
    def drive(msg):
        state['speed'] = msg.drive.speed
        state['last_drive'] = time.monotonic()
        state['drive_values'].append((time.monotonic(), msg.drive.speed))
        pubs[NS+'throttle_command'].publish(Float32(data=msg.drive.speed*.04))
        pubs[NS+'steering_command'].publish(Float32(data=0.))
    node.create_subscription(AckermannDriveStamped, '/drive', drive, 10)
    def reset(msg):
        if msg.data:
            state['lap'] = 0
            state['reset_seen'] = True
            pubs[NS+'sim_reset'].publish(String(data='reset command'))
    node.create_subscription(Bool, '/autodrive/reset_command', reset, 10)
    def status(msg):
        state['status'] = json.loads(msg.data)
    node.create_subscription(String, '/benchmark/status', status, 10)
    epoch, start = time.time_ns(), time.monotonic_ns()
    def tick():
        now = epoch+time.monotonic_ns()-start
        sec, ns = divmod(now, 10**9)
        stamp = Time(sec=sec, nanosec=ns)
        if not state.get('clock_loss'):
            pubs['/clock'].publish(ClockMessage(clock=stamp))
        if time.monotonic()-state.get('last_drive', time.monotonic()) > .5:
            # Model only the bridge's existing monotonic command timeout, not physics.
            state['speed'] = 0.
        pose = PoseStamped(); pose.header.stamp = stamp; pose.header.frame_id = 'map'
        pose.pose.position.x = 1.; pose.pose.position.y = 1.; pose.pose.orientation.w = 1.
        if state.get('managed') and (state.get('status') or {}).get('status') == 'starting_algorithms' and '/localized_pose' not in pubs:
            pubs['/localized_pose'] = node.create_publisher(PoseStamped, '/localized_pose', 10)
        if not state['loss'] and '/localized_pose' in pubs:
            pubs['/localized_pose'].publish(pose)
        odom = Odometry(); odom.header.stamp = stamp; odom.header.frame_id = 'odom'; odom.child_frame_id = 'base_link'
        odom.pose.pose = pose.pose; odom.twist.twist.linear.x = state['speed']
        pubs['/odom'].publish(odom)
        scan = LaserScan(); scan.header.stamp = stamp; scan.header.frame_id = 'laser'
        scan.angle_increment = .01; scan.ranges = [2.]*10
        pubs['/scan'].publish(scan)
        imu = Imu(); imu.header.stamp = stamp; imu.header.frame_id = 'imu'; imu.orientation.w = 1.
        imu.linear_acceleration.z = 9.80665
        pubs['/sensors/imu/raw'].publish(imu)
        request = AckermannDriveStamped(); request.header.stamp = stamp; request.header.frame_id = 'base_link'
        request.drive.speed = 1.
        if '/benchmark/requested_drive' in pubs:
            pubs['/benchmark/requested_drive'].publish(request)
        if state['speed'] > 0:
            state['last_lap'] = state['last_lap'] or time.monotonic()
            if time.monotonic()-state['last_lap'] > .5:
                state['lap'] += 1
                state['last_lap'] = time.monotonic()
        pubs[NS+'lap_count'].publish(Int32(data=state['lap']))
        pubs[NS+'collision_count'].publish(Int32(data=state['collision']))
        pubs[NS+'last_lap_time'].publish(Float32(data=.5 if state['lap'] else float('inf')))
    node.create_timer(.02, tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
    track = tmp_path/'tracks'/'synthetic'
    track.mkdir(parents=True)
    (track/'map.pgm').write_bytes(b'P5\n20 20\n255\n'+bytes([254])*400)
    (track/'map.yaml').write_text('image: map.pgm\nresolution: 1.0\norigin: [0, 0, 0]\n')
    (track/'raceline.csv').write_text('0;0;0;0;0;2;0\n10;10;0;0;0;2;0\n20;10;10;0;0;2;0\n30;0;10;0;0;2;0\n')
    processes = []
    def start_run(name, level='basic', laps=1, managed=False, reserve=1.):
        config = {'experiment': {'map': 'synthetic', 'mode': 'manual', 'laps': laps, 'warmup_laps': 0, 'timeout_s': 15.},
                  'components': {'controller': {'type': 'fixture', 'managed': False},
                                 'localization': {'type': 'fixture', 'managed': False}},
                  'logging': {'level': level}, 'watchdogs': {'startup_s': 12., 'disk_reserve_mb': reserve}}
        entry = ['ros2', 'run', 'benchmark', 'benchmark']
        if managed:
            state['managed'] = True
            node.destroy_publisher(pubs.pop('/localized_pose'))
            node.destroy_publisher(pubs.pop('/benchmark/requested_drive'))
            config['experiment']['mode'] = 'managed'
            config['components'] = {'controller': {'type': 'pure_pursuit', 'params': {'speed_scale': .5}},
                                    'localization': {'type': 'mcl'}}
            # Exercise real controller startup/remapping/cleanup. Only the
            # localizer process is stubbed: localization itself has its own suite.
            code = ('import sys; from benchmark.collector import manager; '
                    'from benchmark.collector.process import Process; '
                    'manager.Process=lambda cmd,log,env=None: Process('
                    '[sys.executable,"-c","import time; time.sleep(120)"] '
                    'if cmd[:3]==["ros2","launch","perception"] else cmd,log,env); '
                    'from benchmark.cli import main; sys.exit(main())')
            entry = ['python3', '-c', code]
        path = tmp_path/(name+'.yaml'); path.write_text(yaml.safe_dump(config))
        stream = (tmp_path/(name+'.log')).open('w')
        proc = subprocess.Popen([*entry, '--log-root', str(tmp_path/'log'),
               '--tracks-dir', str(tmp_path/'tracks'), 'run', '--config', str(path), '--run-id', name],
               stdout=stream, stderr=subprocess.STDOUT, env={**os.environ, 'ROS_DOMAIN_ID': '68'}, start_new_session=True)
        processes.append((proc, stream))
        return proc
    def spin_until(predicate, timeout=25.):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
            if predicate():
                return
        logs = '\n'.join(p.read_text() for p in tmp_path.glob('*.log'))
        pytest.fail('Synthetic collection timed out\n'+logs)
    yield node, state, start_run, spin_until
    import signal
    for proc, stream in processes:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            deadline = time.monotonic()+10
            while proc.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.01)
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=3)
        stream.close()
    node.destroy_node(); rclpy.try_shutdown()


def test_complete_collection_offline_reports_and_logging_off(simulator, tmp_path, monkeypatch):
    node, state, start, spin = simulator
    proc = start('complete')
    spin(lambda: proc.poll() is not None)
    assert proc.returncode == 0, (tmp_path/'complete.log').read_text()
    run = tmp_path/'log'/'runs'/'complete'
    meta = verify(run)
    assert meta['status'] == 'completed'
    assert meta['recorder']['closed']
    assert meta['recorder_integrity']['consistent']
    assert meta['stop_confirmation'] == 'wheel_odom_rest_and_gate_inhibited'
    assert any(v > 0 for t, v in state['drive_values'])
    assert state['drive_values'][-1][1] == 0
    # Analysis must never create a ROS node, including when a graph happens to exist.
    with monkeypatch.context() as patch:
        def forbidden(*args, **kwargs):
            raise AssertionError('Offline analysis attempted to create a node')
        patch.setattr(rclpy, 'create_node', forbidden)
        output = analyze(tmp_path/'log', 'complete', 'first')
        other = analyze(tmp_path/'log', 'complete', 'second')
    assert (output/'report.html').is_file() and (other/'metrics.json').is_file()
    summary = json.loads((output/'metrics.json').read_text())
    assert summary['modules'][0]['metrics']['lap_time']['value']['mean'] == pytest.approx(.5)
    off = start('off', 'off')
    spin(lambda: off.poll() is not None)
    assert off.returncode == 0, (tmp_path/'off.log').read_text()
    assert not (tmp_path/'log'/'runs'/'off'/'bag').exists()
    assert verify(tmp_path/'log'/'runs'/'off')['status'] == 'completed'


def test_data_loss_aborts_and_stops_without_controller_override(simulator, tmp_path):
    node, state, start, spin = simulator
    proc = start('lost-pose', laps=10)
    spin(lambda: state['speed'] > 0)
    state['loss'] = True
    spin(lambda: proc.poll() is not None)
    meta = verify(tmp_path/'log'/'runs'/'lost-pose')
    assert meta['status'] == 'aborted'
    assert 'loss' in meta['failure'] or 'required input' in meta['failure']
    assert meta['stop_confirmation'] == 'wheel_odom_rest_and_gate_inhibited'
    assert state['drive_values'][-1][1] == 0


def test_explicit_stop_lifecycle(simulator, tmp_path):
    node, state, start, spin = simulator
    proc = start('stopped', laps=10)
    spin(lambda: state['speed'] > 0)
    client = node.create_client(Trigger, '/benchmark/stop')
    assert client.wait_for_service(timeout_sec=2.)
    future = client.call_async(Trigger.Request())
    spin(lambda: future.done())
    assert future.result().success
    spin(lambda: proc.poll() is not None)
    assert verify(tmp_path/'log'/'runs'/'stopped')['status'] == 'stopped'
    assert state['drive_values'][-1][1] == 0


def test_transient_loss_continues_and_recorder_failure_aborts(simulator, tmp_path):
    import signal
    node, state, start, spin = simulator
    proc = start('recorder-failure', laps=10)
    spin(lambda: state['speed'] > 0)
    state['loss'] = True
    began = time.monotonic()
    spin(lambda: time.monotonic()-began > .2)
    state['loss'] = False
    began = time.monotonic()
    spin(lambda: time.monotonic()-began > .3)
    assert proc.poll() is None and state['speed'] > 0
    # Terminate only the recorder group owned by this temporary synthetic run.
    rows = subprocess.check_output(['ps', '-eo', 'pid,args'], text=True).splitlines()
    matches = [int(row.split(None, 1)[0]) for row in rows if 'ros2 run benchmark benchmark_recorder' in row
               and str(tmp_path/'log'/'runs'/'recorder-failure'/'recorder.yaml') in row]
    assert len(matches) == 1
    os.killpg(matches[0], signal.SIGINT)
    spin(lambda: proc.poll() is not None)
    meta = verify(tmp_path/'log'/'runs'/'recorder-failure')
    assert meta['status'] == 'aborted'
    assert 'recorder' in meta['failure']
    assert meta['stop_confirmation'] == 'wheel_odom_rest_and_gate_inhibited'
    assert any(gap['signal'] == '/localized_pose' for gap in meta['gaps'])


def test_managed_controller_startup_and_cleanup(simulator, tmp_path):
    node, state, start, spin = simulator
    proc = start('managed', managed=True)
    spin(lambda: proc.poll() is not None)
    assert proc.returncode == 0, (tmp_path/'managed.log').read_text()
    meta = verify(tmp_path/'log'/'runs'/'managed')
    assert meta['status'] == 'completed'
    assert 'controller' in meta['process_results'] and 'localization' in meta['process_results']
    assert not meta['process_results']['controller']['group_alive']
    assert not meta['process_results']['localization']['group_alive']
    assert meta['stop_confirmation'] == 'wheel_odom_rest_and_gate_inhibited'


def test_disk_reserve_failure_prevents_motion(simulator, tmp_path):
    node, state, start, spin = simulator
    proc = start('disk-reserve', reserve=1e15)
    spin(lambda: proc.poll() is not None)
    meta = verify(tmp_path/'log'/'runs'/'disk-reserve')
    assert meta['status'] == 'aborted' and meta['failure'] == 'disk_reserve_exhausted'
    assert not any(speed > 0 for stamp, speed in state['drive_values'])


def test_preflight_rejection_does_not_send_commands_to_an_existing_stack(simulator, tmp_path):
    node, state, start, spin = simulator
    other_drive = node.create_publisher(AckermannDriveStamped, '/drive', 10)
    controls = []
    listener = node.create_subscription(Bool, '/benchmark/enable', lambda msg: controls.append(msg.data), 10)
    proc = start('preflight-rejected')
    spin(lambda: proc.poll() is not None)
    assert proc.returncode != 0
    assert 'preflight_existing_drive_publisher' in (tmp_path/'preflight-rejected.log').read_text()
    assert controls == []
    assert not (tmp_path/'log'/'runs'/'preflight-rejected').exists()


def test_frozen_clock_uses_bridge_timeout_and_does_not_claim_confirmed_rest(simulator, tmp_path):
    node, state, start, spin = simulator
    proc = start('frozen-clock', laps=10)
    spin(lambda: state['speed'] > 0)
    state['clock_loss'] = True
    spin(lambda: proc.poll() is not None)
    meta = verify(tmp_path/'log'/'runs'/'frozen-clock')
    assert meta['status'] == 'aborted'
    assert meta['stop_confirmation'] == 'unconfirmed'
    assert meta['stop_mode'] == 'bridge_timeout_fallback'
    assert state['speed'] == 0.
    # Requested commands continue, but the inhibited gate must fall silent.
    assert time.monotonic()-state['last_drive'] > .5
