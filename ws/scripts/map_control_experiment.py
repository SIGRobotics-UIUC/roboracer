#!/usr/bin/env python3
"""Balanced fixed-speed-profile MAP steering-rate experiment on mapped Porto.

Run inside sourced ROS Docker. Uses normal localization and wheel-speed feedback;
ground truth bounds the diagnostic run and supplies offline evaluation only.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def stop_group(proc):
    if proc is not None and proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=5)


def main():
    import numpy as np
    import yaml
    import rclpy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, Float32, Int32
    from ackermann_msgs.msg import AckermannDriveStamped
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--track', default='porto_odom_trial')
    ap.add_argument('--scale', type=float, default=.5)
    ap.add_argument('--output', required=True)
    ap.add_argument('--timeout', type=float, default=100.)
    ap.add_argument('--laps', type=int)
    ap.add_argument('--repeat-current', type=int, default=0, help='Repeat the unchanged 3.2 rad/s configuration N times; record clock issues without stopping on them')
    ap.add_argument('--resume', action='store_true', help='Continue the remaining repeats in an existing manifest without overwriting prior trials')
    ap.add_argument('--wall-time', action='store_true', help='Use legacy wall time instead of the bridge /clock')
    args = ap.parse_args()
    args.laps = args.laps if args.laps is not None else (2 if args.repeat_current else 3)
    if args.laps < 2 or args.repeat_current < 0 or not 0 < args.scale <= .5:
        ap.error('Use at least two lap increments, nonnegative repeat count, and speed scale at most 0.5')
    output = Path(args.output)
    prior = None
    if output.exists():
        if not args.resume: ap.error('Output already exists; choose a new experiment directory or use --resume')
        prior = json.loads((output/'manifest.json').read_text())
    else:
        if args.resume: ap.error('No existing experiment to resume')
        output.mkdir(parents=True)
    track = Path('tracks')/args.track
    path = np.loadtxt(track/'raceline.csv', delimiter=';', comments='#')[:, 1:3]
    origin = yaml.safe_load((track/'map_to_world.yaml').read_text())
    raceline_hash = hashlib.sha256((track/'raceline.csv').read_bytes()).hexdigest()
    rclpy.init()
    from rclpy.parameter import Parameter
    node = rclpy.create_node('map_control_experiment', parameter_overrides=[Parameter('use_sim_time', value=not args.wall_time)])
    latest = {}
    clock_events = []
    host_clock_events = []
    def gt(msg):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        arrival = time.monotonic()
        wall_now = time.time()
        ros_now = node.get_clock().now().nanoseconds*1e-9
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if 'clock_sample' in latest:
            previous_stamp, previous_arrival, previous_wall, previous_ros = latest['clock_sample']
            source_dt, monotonic_dt = stamp-previous_stamp, arrival-previous_arrival
            wall_dt = wall_now-previous_wall
            if abs(wall_dt-monotonic_dt) > .5:
                host_clock_events.append(dict(source_stamp=stamp, wall_interval_s=wall_dt, monotonic_interval_s=monotonic_dt))
            ros_dt = ros_now-previous_ros
            if previous_ros > 0 and ros_now > 0 and abs(ros_dt-monotonic_dt) > .5:
                event = dict(ros_stamp=stamp, ros_interval_s=ros_dt, source_interval_s=source_dt,
                             monotonic_interval_s=monotonic_dt, native_lap=latest.get('lap_count'))
                clock_events.append(event)
                print(json.dumps(dict(event='clock_discontinuity', **event)), flush=True)
        latest['clock_sample'] = (stamp, arrival, wall_now, ros_now)
        latest['gt'] = (arrival, p.x, p.y, v.x)
    subs = [node.create_subscription(Odometry, '/ground_truth/odom', gt, 10)]
    for name, kind in [('lap_count', Int32), ('collision_count', Int32), ('last_lap_time', Float32)]:
        subs.append(node.create_subscription(kind, '/autodrive/roboracer_1/'+name,
                    lambda m, key=name: latest.update({key: m.data}), 10))
    drive = node.create_publisher(AckermannDriveStamped, '/drive', 10)
    reset = node.create_publisher(Bool, '/autodrive/reset_command', 10)
    def spin(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
    def hold_stop(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            msg = AckermannDriveStamped(); msg.header.stamp = node.get_clock().now().to_msg()
            drive.publish(msg); spin(.025)
    def path_distance():
        _, x, y, _ = latest['gt']
        dx, dy = x-origin['x_m'], y-origin['y_m']
        c, s = math.cos(origin['yaw_rad']), math.sin(origin['yaw_rad'])
        point = np.array([c*dx+s*dy, -s*dx+c*dy])
        return float(np.min(np.linalg.norm(path-point, axis=1)))
    frozen_inputs = [track/'raceline.csv', Path('src/control/config/vehicle.yaml'),
                     Path('src/autodrive_bridge/config/adapter.yaml'),
                     Path('src/control/control/map_node.py'), Path('src/control/config/autodrive_map_lut.npz')]
    frozen_inputs += [Path('src/control/control/input_health.py'),
                      Path('src/autodrive_bridge/autodrive_bridge/steady_time.py'),
                      Path('src/autodrive_bridge/autodrive_bridge/fast_bridge.py')]
    input_hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in frozen_inputs}
    manifest = dict(status='awaiting_simulator', track=args.track, speed_scale=args.scale, max_speed=0.,
        fixed_input_sha256=input_hashes,
        lookahead_settings=dict(slope=.4, offset=0., minimum=.6, maximum=3.),
        mode='repeatability' if args.repeat_current else 'gain_comparison',
        clock_policy='record_only' if args.repeat_current else 'abort_on_discontinuity',
        use_sim_time=not args.wall_time,
        requested_lap_increments=args.laps,
        raceline_sha256=raceline_hash, rates_radps=[3.2]*args.repeat_current if args.repeat_current else [3.2, 1.6, .8, .8, 1.6, 3.2],
        method='Identical current configuration; native measured flying lap after one warm-up lap.' if args.repeat_current else 'Balanced A B C C B A; unchanged raceline/lookahead/speed scale.', trials=[])
    if prior is not None:
        for key in ('fixed_input_sha256', 'rates_radps', 'speed_scale', 'requested_lap_increments', 'mode', 'use_sim_time'):
            if prior.get(key) != manifest[key]:
                raise ValueError(f'Resume configuration mismatch: {key}')
        manifest['trials'] = prior['trials']
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    try:
        deadline = time.monotonic()+12
        while ('gt' not in latest or time.monotonic()-latest['gt'][0] > .5) and time.monotonic() < deadline:
            spin(.1)
        if 'gt' not in latest or time.monotonic()-latest['gt'][0] > .5:
            raise RuntimeError('Simulator is not streaming fresh ground truth')
        if node.count_publishers('/drive') > 1:
            raise RuntimeError('Another /drive publisher is present; stop it before the experiment')
        manifest['status'] = 'running'
        for index, rate in enumerate(manifest['rates_radps'], 1):
            if index <= len(manifest['trials']):
                continue
            trial = output/f'{index:02d}_rate_{str(rate).replace(".", "p")}'
            trial.mkdir()
            bag = controller = localizer = None
            crash0 = None
            handles = []
            result = dict(index=index, steering_rate_limit_radps=rate, status='starting', native_laps=[])
            clock_events.clear()
            host_clock_events.clear()
            def launch(command, name):
                handle = open(trial/(name+'.log'), 'w'); handles.append(handle)
                return subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
            print(json.dumps(dict(event='trial_start', index=index, rate=rate)), flush=True)
            try:
                if any(hashlib.sha256(Path(p).read_bytes()).hexdigest() != digest for p, digest in input_hashes.items()):
                    raise RuntimeError('Fixed controller/configuration inputs changed during experiment')
                hold_stop(.5)
                for _ in range(3):
                    reset.publish(Bool(data=True)); spin(.1)
                hold_stop(2)
                if time.monotonic()-latest['gt'][0] > .5 or path_distance() > .8:
                    raise RuntimeError('Reset pose does not match the mapped track or telemetry is stale')
                lap0, crash0 = latest['lap_count'], latest['collision_count']
                topics = ['/ground_truth/odom', '/odom', '/localized_pose', '/drive', '/sensors/imu/raw']
                topics += ['/autodrive/roboracer_1/'+name for name in
                           ('left_encoder', 'right_encoder', 'steering', 'throttle',
                            'throttle_command', 'lap_count', 'lap_time', 'last_lap_time', 'collision_count')]
                bag = launch(['ros2', 'bag', 'record', *([] if args.wall_time else ['--use-sim-time']),
                              '-o', str(trial/'bag'), *topics, '/clock'], 'bag')
                spin(1.5)
                if bag.poll() is not None: raise RuntimeError('Bag recorder failed to start')
                localizer = launch(['ros2', 'launch', 'perception', 'localization.launch.py', 'track:='+args.track,
                                    'use_sim_time:='+str(not args.wall_time).lower()], 'localization')
                command = ['ros2', 'run', 'control', 'map_node', '--ros-args', '--params-file',
                           'install/control/share/control/config/vehicle.yaml', '-p', 'track:='+args.track,
                           '-p', 'pose_topic:=/localized_pose', '-p', 'speed_scale:='+str(args.scale),
                           '-p', 'max_speed:=0.0', '-p', 'steering_rate_limit:='+str(rate)]
                command += ['-p', 'use_sim_time:='+str(not args.wall_time).lower()]
                result['controller_command'] = command
                controller = launch(command, 'controller')
                started = time.monotonic(); seen_lap = lap0; pending = None
                while time.monotonic()-started < args.timeout:
                    spin(.04)
                    if clock_events and not args.repeat_current:
                        raise RuntimeError('ROS versus monotonic clock discontinuity; stopped timing-contaminated trial')
                    if latest['collision_count'] != crash0:
                        raise RuntimeError('Collision count changed; stopped trial')
                    if time.monotonic()-latest['gt'][0] > 1.:
                        raise RuntimeError('Ground-truth stream gap exceeded 1 s; stopped trial')
                    if abs(latest['gt'][3]) > 4.5 or path_distance() > 1.:
                        raise RuntimeError('Vehicle speed or track-distance safety bound exceeded')
                    if controller.poll() is not None or localizer.poll() is not None or bag.poll() is not None:
                        raise RuntimeError('Controller/localizer/recorder exited early')
                    if latest['lap_count'] > seen_lap:
                        count = latest['lap_count']
                        if count != seen_lap+1: raise RuntimeError('Lap counter skipped; timing result ambiguous')
                        pending = (count, time.monotonic()); seen_lap = count
                    if pending and time.monotonic()-pending[1] > .25:
                        lap = dict(increment=pending[0]-lap0, native_count=pending[0],
                                   lap_time_s=latest.get('last_lap_time'))
                        result['native_laps'].append(lap)
                        print(json.dumps(dict(event='lap', index=index, **lap)), flush=True)
                        pending = None
                    if seen_lap >= lap0+args.laps and pending is None:
                        result['status'] = 'completed'; break
                else:
                    result['status'] = 'timeout'
                result['collision_change'] = latest['collision_count']-crash0
                result['wall_duration_s'] = time.monotonic()-started
            except KeyboardInterrupt:
                result.update(status='interrupted', error='User interrupted experiment')
            except Exception as exc:
                result.update(status='aborted', error=str(exc))
            finally:
                if crash0 is not None:
                    result['collision_change'] = latest.get('collision_count', crash0)-crash0
                stop_group(controller)
                hold_stop(1)
                stop_group(localizer)
                spin(.3)
                stop_group(bag)
                for handle in handles: handle.close()
            flying = [lap['lap_time_s'] for lap in result['native_laps'] if lap['increment'] >= 2 and lap['lap_time_s'] and lap['lap_time_s'] > 0]
            result['mean_flying_lap_s'] = sum(flying)/len(flying) if flying else None
            result['clock_discontinuities'] = list(clock_events)
            result['host_wall_clock_steps'] = list(host_clock_events)
            result['trial_directory'] = str(trial)
            (trial/'result.json').write_text(json.dumps(result, indent=2)+'\n')
            if (trial/'bag'/'metadata.yaml').exists():
                report_command = ['python3', 'scripts/tire_lap_report.py', str(trial/'bag'),
                                  '--track-dir', str(track), '--output', str(trial/'report'), '--lap', 'native:2']
                with open(trial/'report.log', 'w') as log:
                    proc = subprocess.run(report_command, stdout=log, stderr=subprocess.STDOUT)
                result['report_status'] = 'native_flying_lap' if proc.returncode == 0 else 'unavailable'
                if proc.returncode != 0:
                    report_command[-1] = 'auto'
                    with open(trial/'report_fallback.log', 'w') as log:
                        fallback = subprocess.run(report_command, stdout=log, stderr=subprocess.STDOUT)
                    if fallback.returncode == 0: result['report_status'] = 'auto_fallback_not_comparable_flying_lap'
                summary = trial/'report'/'summary.json'
                if summary.exists():
                    result['report_summary'] = json.loads(summary.read_text())
            (trial/'result.json').write_text(json.dumps(result, indent=2)+'\n')
            manifest['trials'].append(result)
            (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
            print(json.dumps(dict(event='trial_end', index=index, status=result['status'],
                                  mean_flying_lap_s=result['mean_flying_lap_s'], report_status=result.get('report_status'))), flush=True)
            if result['status'] != 'completed':
                manifest['status'] = 'stopped_after_incomplete_trial'
                # Report generation blocks this executor; refresh the snapshot
                # before judging whether the simulator is still streaming.
                spin(.5)
                if not args.repeat_current or result['status'] == 'interrupted' or time.monotonic()-latest['gt'][0] > 1. or 'Fixed controller' in result.get('error', ''):
                    print('Stopping sequence after incomplete trial or unavailable simulator.', flush=True)
                    break
                print('Failure retained; resetting for the next identical repeat.', flush=True)
            spin(2)
        else:
            manifest['status'] = 'completed' if all(r['status'] == 'completed' for r in manifest['trials']) else 'completed_with_failures'
    except Exception as exc:
        manifest.update(status='preflight_failed', error=str(exc))
        print(json.dumps(dict(event='preflight_failed', error=str(exc))), flush=True)
    finally:
        hold_stop(1)
        (output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
        node.destroy_node(); rclpy.shutdown()
    print(str(output/'manifest.json'), flush=True)


if __name__ == '__main__':
    main()
