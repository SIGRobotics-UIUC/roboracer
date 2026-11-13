"""AutoDRIVE MAP speed sweep on map_trial; stops each trial at first lap or collision."""
import json, os, signal, subprocess, sys, time
import rclpy
from std_msgs.msg import Bool, Int32, Float32
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

rclpy.init()
n = rclpy.create_node('map_speed_sweep')
v = {}
for topic, typ in [('lap_count', Int32), ('collision_count', Int32), ('last_lap_time', Float32)]:
    n.create_subscription(typ, '/autodrive/roboracer_1/' + topic,
                          lambda msg, key=topic: v.__setitem__(key, msg.data), 10)
n.create_subscription(Odometry, '/odom', lambda msg: v.__setitem__('speed', msg.twist.twist.linear.x), 10)
n.create_subscription(AckermannDriveStamped, '/drive', lambda msg: v.__setitem__('command', msg.drive.speed), 10)
reset = n.create_publisher(Bool, '/autodrive/reset_command', 10)

def spin(seconds):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        rclpy.spin_once(n, timeout_sec=.1)

spin(2)
print('initial ' + json.dumps(v), flush=True)
for scale in [float(arg) for arg in sys.argv[1:]]:
    for _ in range(4):
        reset.publish(Bool(data=True))
        spin(.1)
    spin(2)
    lap = v.get('lap_count', 0)
    collision = v.get('collision_count', 0)
    log = '/root/ws/log/map_sweep_{:.2f}.log'.format(scale)
    with open(log, 'w') as output:
        proc = subprocess.Popen(['ros2', 'launch', 'control', 'race.launch.py', 'track:=map_trial',
                                 'controller:=map', 'speed_scale:={}'.format(scale)],
                                stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        start = time.monotonic()
        maximum = 0.
        command = 0.
        status = 'timeout'
        try:
            while time.monotonic() - start < 120:
                spin(.2)
                maximum = max(maximum, v.get('speed', 0))
                command = max(command, v.get('command', 0))
                if v.get('collision_count', collision) > collision:
                    status = 'collision'
                    break
                if v.get('lap_count', lap) > lap:
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
            spin(1)
    result = dict(scale=scale, status=status, seconds=round(time.monotonic()-start, 1),
                  lap_count=v.get('lap_count'), collision_count=v.get('collision_count'),
                  lap_time=v.get('last_lap_time'), max_measured_speed=round(maximum, 3),
                  max_command_speed=round(command, 3), log=log)
    print(json.dumps(result), flush=True)
n.destroy_node()
rclpy.shutdown()
