#!/usr/bin/env python3
"""Record a simulator MAP lap and compare actual cornering with the LUT.

Run inside the connected ROS container after sourcing install/setup.bash:
  python3 scripts/characterize_map.py record --track map_trial --scale 0.7
  python3 scripts/characterize_map.py analyze log/map_characterization_*.csv

Ground-truth topics are simulator-only. They are recorded for offline calibration,
never fed to the controller. The recorder stops at the first lap or collision.
"""
import argparse
import csv
import json
import math
import os
import signal
import subprocess
import time

FIELDS = ['time', 'gt_vx', 'gt_vy', 'gt_yaw_rate', 'wheel_vx', 'steer_sensor',
          'steer_command', 'speed_command', 'imu_ay', 'lap_count', 'collision_count',
          'gt_age', 'wheel_age', 'steer_age', 'command_age', 'imu_age']


def analyze(path, model_path, lut_path, recorded_steering_scale=1.):
    import numpy as np
    from control.map_lut import Lookup
    with open(model_path) as stream:
        model = json.load(stream)
    lut = Lookup(lut_path, model)
    with open(path, newline='') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError('empty recording')
    data = {key: np.array([float(row[key]) for row in rows]) for key in FIELDS}
    data['steer_sensor'] /= recorded_steering_scale
    t = data['time']
    good = (np.isfinite(t) & np.isfinite(data['gt_vx']) & np.isfinite(data['gt_vy']) &
            np.isfinite(data['gt_yaw_rate']) & np.isfinite(data['wheel_vx']) &
            np.isfinite(data['steer_sensor']) & (data['wheel_age'] < .12) &
            (data['steer_age'] < .12) & (data['command_age'] < .12) &
            (data['gt_vx'] >= lut.speeds[0]) & (data['gt_vx'] <= lut.speeds[-1]))
    # The LUT is a steady-state map; omit startup, braking and steering transients.
    dt = np.diff(t, prepend=t[0])
    ax = np.divide(np.diff(data['gt_vx'], prepend=data['gt_vx'][0]), dt,
                   out=np.full(len(t), np.nan), where=dt > 1e-4)
    steer_rate = np.divide(np.diff(data['steer_sensor'], prepend=data['steer_sensor'][0]), dt,
                           out=np.full(len(t), np.nan), where=dt > 1e-4)
    vy_rate = np.divide(np.diff(data['gt_vy'], prepend=data['gt_vy'][0]), dt,
                        out=np.full(len(t), np.nan), where=dt > 1e-4)
    stable = good & (np.abs(ax) < 1.5) & (np.abs(steer_rate) < .35) & (np.abs(vy_rate) < 1.) & (np.abs(data['steer_sensor']) >= .025)
    observed = data['gt_vx'] * data['gt_yaw_rate'] + vy_rate
    predicted = np.full(len(t), np.nan)
    for i in np.flatnonzero(stable):
        j = int(np.argmin(abs(lut.speeds - data['gt_vx'][i])))
        acceleration, steering = lut.columns[j]
        angle = abs(data['steer_sensor'][i])
        if angle <= steering[-1]:
            predicted[i] = math.copysign(float(np.interp(angle, steering, acceleration)), data['steer_sensor'][i])
    valid = stable & np.isfinite(predicted)
    speed_error = data['wheel_vx'][good] - data['gt_vx'][good]
    steady_speed_error = data['wheel_vx'][valid] - data['gt_vx'][valid]
    tracking = good & (np.abs(data['steer_command']) > .05) & (np.abs(steer_rate) < .35)
    steering_ratio = data['steer_sensor'][tracking] / data['steer_command'][tracking]
    lap_values = data['lap_count'][np.isfinite(data['lap_count'])]
    collision_values = data['collision_count'][np.isfinite(data['collision_count'])]
    result = {'samples':len(t), 'paired_samples':int(good.sum()), 'steady_samples':int(valid.sum()),
              'lap_count_change':int(lap_values[-1]-lap_values[0]) if len(lap_values) else None,
              'collision_count_change':int(collision_values[-1]-collision_values[0]) if len(collision_values) else None,
              'wheel_minus_ground_truth_median_mps':float(np.median(speed_error)) if len(speed_error) else None,
              'wheel_minus_ground_truth_p95_abs_mps':float(np.percentile(abs(speed_error),95)) if len(speed_error) else None,
              'steady_wheel_minus_ground_truth_p95_abs_mps':float(np.percentile(abs(steady_speed_error),95)) if len(steady_speed_error) else None,
              'steering_sensor_to_command_median_ratio':float(np.median(steering_ratio)) if len(steering_ratio) else None,
              'peak_wheel_mps':float(np.nanmax(data['wheel_vx'])),
              'peak_ground_truth_mps':float(np.nanmax(data['gt_vx']))}
    if valid.any():
        residual = observed[valid] - predicted[valid]
        result.update(lateral_acceleration_median_abs_error_mps2=float(np.median(abs(residual))),
                      lateral_acceleration_bias_mps2=float(np.median(residual)),
                      observed_peak_abs_ay_mps2=float(np.max(abs(observed[valid]))),
                      model_peak_abs_ay_mps2=float(np.max(abs(predicted[valid]))))
    print(json.dumps(result, indent=2))
    return result


def record(args):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, Float32, Int32
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from ackermann_msgs.msg import AckermannDriveStamped
    rclpy.init()
    node = rclpy.create_node('map_characterization')
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    latest = {}
    rows = []

    def sample(key, values):
        latest[key] = (time.monotonic(), values)

    def gt_callback(msg):
        now = time.monotonic()
        sample('gt', (msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                      msg.twist.twist.angular.z))
        def get(key, count):
            if key not in latest:
                return (float('nan'),) * count, float('inf')
            stamp, values = latest[key]
            return values, now - stamp
        gt, gt_age = get('gt', 3)
        wheel, wheel_age = get('wheel', 1)
        steer, steer_age = get('steer', 1)
        command, command_age = get('command', 2)
        imu, imu_age = get('imu', 1)
        lap, _ = get('lap', 1)
        collision, _ = get('collision', 1)
        rows.append([now, *gt, *wheel, *steer, *command, *imu, *lap, *collision,
                     gt_age, wheel_age, steer_age, command_age, imu_age])

    subs = [
        node.create_subscription(Odometry, '/ground_truth/odom', gt_callback, qos),
        node.create_subscription(Odometry, '/odom', lambda m: sample('wheel', (m.twist.twist.linear.x,)), qos),
        node.create_subscription(Float32, '/autodrive/roboracer_1/steering',
                                 lambda m: sample('steer', (m.data,)), qos),
        node.create_subscription(AckermannDriveStamped, '/drive',
                                 lambda m: sample('command', (m.drive.steering_angle, m.drive.speed)), qos),
        node.create_subscription(Imu, '/sensors/imu/raw',
                                 lambda m: sample('imu', (m.linear_acceleration.y,)), qos),
        node.create_subscription(Int32, '/autodrive/roboracer_1/lap_count',
                                 lambda m: sample('lap', (m.data,)), qos),
        node.create_subscription(Int32, '/autodrive/roboracer_1/collision_count',
                                 lambda m: sample('collision', (m.data,)), qos),
    ]
    reset = node.create_publisher(Bool, '/autodrive/reset_command', 10)
    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
    # DDS discovery can take several seconds in a newly started process.
    discovery_deadline = time.monotonic() + 12
    while 'gt' not in latest and time.monotonic() < discovery_deadline:
        spin(.25)
    if 'gt' not in latest:
        raise RuntimeError('no simulator ground truth; connect AutoDRIVE and enable restricted topics')
    for _ in range(3):
        reset.publish(Bool(data=True))
        spin(.1)
    spin(2)
    lap0 = latest.get('lap', (0, (0,)))[1][0]
    crash0 = latest.get('collision', (0, (0,)))[1][0]
    rows.clear()  # discard discovery and reset telemetry from this trial
    output = args.output or 'log/map_characterization_{}.csv'.format(int(time.time()))
    launch_log = os.path.splitext(output)[0] + '.launch.log'
    with open(launch_log, 'w') as log:
        proc = subprocess.Popen(['ros2', 'launch', 'control', 'race.launch.py',
                                 'track:=' + args.track, 'controller:=map',
                                 'speed_scale:=' + str(args.scale)], stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        start = time.monotonic()
        status = 'timeout'
        try:
            while time.monotonic() - start < args.timeout:
                spin(.2)
                if latest.get('collision', (0, (crash0,)))[1][0] > crash0:
                    status = 'collision'
                    break
                if latest.get('lap', (0, (lap0,)))[1][0] >= lap0 + args.laps:
                    status = 'lap'
                    break
                if proc.poll() is not None:
                    status = 'launch_exited'
                    break
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=5)
            spin(.5)
    with open(output, 'w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(FIELDS)
        writer.writerows(rows)
    node.destroy_node()
    rclpy.shutdown()
    print(json.dumps({'status':status, 'samples':len(rows), 'csv':output,
                      'launch_log':launch_log}), flush=True)
    if status == 'collision':
        raise SystemExit(2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    run = sub.add_parser('record')
    run.add_argument('--track', default='map_trial')
    run.add_argument('--scale', type=float, default=.7)
    run.add_argument('--timeout', type=float, default=90.)
    run.add_argument('--laps', type=int, default=2,
                     help='lap-count increments after reset; 2 includes a complete flying lap')
    run.add_argument('--output')
    report = sub.add_parser('analyze')
    report.add_argument('csv')
    report.add_argument('--model', default='src/control/config/map_model.json')
    report.add_argument('--lut', default='src/control/config/autodrive_map_lut.npz')
    report.add_argument('--recorded-steering-scale', type=float, default=1.,
                        help='use 0.524 for recordings made before the steering unit correction')
    args = parser.parse_args()
    if args.action == 'record':
        record(args)
    else:
        analyze(args.csv, args.model, args.lut, args.recorded_steering_scale)


if __name__ == '__main__':
    main()
