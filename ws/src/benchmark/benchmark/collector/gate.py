"""Benchmark-only command authority. Its timers use a monotonic ROS clock."""
import json
import fcntl
from pathlib import Path
import signal
import time

from .lifecycle import GatePolicy
from .topics import REQUESTED, valid, stamp_ns, subscription_qos


def main():
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from ackermann_msgs.msg import AckermannDriveStamped
    from std_msgs.msg import Bool, String
    from rosidl_runtime_py.utilities import get_message
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    class Gate(Node):
        def __init__(self):
            super().__init__('benchmark_gate')
            run_path = self.declare_parameter('run_path', '').value
            self.collection_lock = (Path(run_path)/'collection.lock').open('r') if run_path else None
            if self.collection_lock:
                fcntl.flock(self.collection_lock, fcntl.LOCK_SH | fcntl.LOCK_NB)
            timeout = self.declare_parameter('heartbeat_s', 1.0).value
            self.loss = self.declare_parameter('loss_s', 1.0).value
            self.limits = json.loads(self.declare_parameter('signal_limits', '{}').value)
            self.steer_limit = self.declare_parameter('steer_limit', 0.524).value
            self.policy = GatePolicy(timeout, self.limits.get(REQUESTED, self.loss))
            self.seen, self.stamps = {}, {}
            self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
            self.status = self.create_publisher(String, '/benchmark/gate_status', 10)
            self.create_subscription(Bool, '/benchmark/enable', self.control, 10)
            self.create_subscription(AckermannDriveStamped, REQUESTED, self.command, 10)
            essentials = {'/localized_pose': 'geometry_msgs/msg/PoseStamped', '/odom': 'nav_msgs/msg/Odometry',
                          '/scan': 'sensor_msgs/msg/LaserScan', '/sensors/imu/raw': 'sensor_msgs/msg/Imu',
                          '/clock': 'rosgraph_msgs/msg/Clock'}
            self.subs = [self.create_subscription(get_message(kind), topic,
                         lambda msg, name=topic: self.observe(name, msg), subscription_qos(self, topic))
                         for topic, kind in essentials.items()]
            self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def observe(self, topic, message):
            stamp = stamp_ns(message)
            if valid(topic, message) and stamp is not None and stamp > self.stamps.get(topic, 0):
                now_ns = self.get_clock().now().nanoseconds
                if topic == '/clock' or -100_000_000 <= now_ns-stamp <= self.limits.get(topic, self.loss)*1e9:
                    self.seen[topic] = time.monotonic()
                    self.stamps[topic] = stamp

        def control(self, message):
            self.policy.control(message.data, time.monotonic())

        def command(self, message):
            now = time.monotonic()
            if not valid(REQUESTED, message) or abs(message.drive.steering_angle) > self.steer_limit+1e-6:
                self.policy.fault = 'invalid controller command'
            else:
                age = self.get_clock().now().nanoseconds-stamp_ns(message)
                if not -100_000_000 <= age <= self.limits.get(REQUESTED, self.loss)*1e9:
                    self.policy.fault = 'stale or future controller command'
                else:
                    self.policy.command = now
                    if self.policy.check(now):
                        self.pub.publish(message)

        def tick(self):
            now = time.monotonic()
            clock_fresh = now-self.seen.get('/clock', -float('inf')) <= self.limits.get('/clock', self.loss)
            if self.policy.enabled:
                missing = [topic for topic in ('/localized_pose', '/odom', '/scan', '/sensors/imu/raw', '/clock')
                           if now-self.seen.get(topic, -float('inf')) > self.limits.get(topic, self.loss)]
                if missing:
                    self.policy.fault = 'required input lost: '+','.join(missing)
            if not self.policy.check(now) and clock_fresh:
                message = AckermannDriveStamped()
                message.header.frame_id = 'base_link'
                message.header.stamp = self.get_clock().now().to_msg()
                self.pub.publish(message)
            # The adapter's speed ramp uses ROS time. With a frozen clock,
            # repeated /drive zeros could keep its last positive throttle alive.
            # Silence lets fast_bridge's independent monotonic timeout zero it.
            self.status.publish(String(data=json.dumps({'enabled': self.policy.enabled,
                'fault': self.policy.fault,
                'stop_mode': 'repeated_zero' if clock_fresh else 'bridge_timeout_fallback'})))

    node = Gate()
    stopping = False
    def interrupted(signum, frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        while not stopping and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.policy.control(False, time.monotonic())
        for _ in range(10):
            node.tick()
            time.sleep(0.05)
        if node.collection_lock:
            node.collection_lock.close()
        node.destroy_node()
        rclpy.try_shutdown()
