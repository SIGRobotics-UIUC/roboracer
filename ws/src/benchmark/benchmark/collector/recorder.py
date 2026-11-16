"""Separate rosbag2 writer process with explicit accepted-write health telemetry."""
import json
import hashlib
import fcntl
from pathlib import Path
import shutil
import signal
import time
import yaml

from .topics import subscription_qos
from .artifacts import write_json


def main():
    import rclpy
    import rosbag2_py
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from rosidl_runtime_py.utilities import get_message
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    class Recorder(Node):
        def __init__(self):
            super().__init__('benchmark_recorder')
            self.run = Path(self.declare_parameter('run_path', '').value)
            self.collection_lock = (self.run/'collection.lock').open('r')
            fcntl.flock(self.collection_lock, fcntl.LOCK_SH | fcntl.LOCK_NB)
            self.topics = json.loads(self.declare_parameter('topics', '{}').value)
            self.reserve = self.declare_parameter('disk_reserve_mb', 256.0).value*1024**2
            requested = self.declare_parameter('storage', 'auto').value
            plugins = rosbag2_py.get_registered_writers()
            storage = ('mcap' if 'mcap' in plugins else 'sqlite3') if requested == 'auto' else requested
            if storage not in plugins:
                raise RuntimeError(f'Bag storage plugin unavailable: {storage}')
            self.health = {'counts': {}, 'error': None, 'storage': storage, 'qos': {},
                           'clock_domain': 'transport', 'write_ack': 'rosbag2 writer acceptance; not per-message fsync',
                           'warning': 'MCAP unavailable; using SQLite3' if requested == 'auto' and storage != 'mcap' else None}
            self.writer = rosbag2_py.SequentialWriter()
            self.writer.open(rosbag2_py.StorageOptions(uri=str(self.run/'bag'), storage_id=storage),
                             rosbag2_py.ConverterOptions('', ''))
            self.subs = {}
            self.index = (self.run/'capture_index.jsonl').open('x')
            self.sequence = 0
            self.status = self.create_publisher(String, '/benchmark/recorder_status', 10)
            self.create_timer(0.2, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def receive(self, topic, serialized):
            if self.health['error']:
                return
            try:
                received_monotonic_ns = time.monotonic_ns()
                stamp = self.get_clock().now().nanoseconds
                if stamp <= 0:
                    return
                self.writer.write(topic, bytes(serialized), stamp)
                self.index.write(json.dumps({'sequence': self.sequence, 'topic': topic,
                    'recorded_ns': stamp, 'received_monotonic_ns': received_monotonic_ns,
                    'sha256': hashlib.sha256(serialized).hexdigest()})+'\n')
                self.sequence += 1
                self.health['counts'][topic] = self.health['counts'].get(topic, 0)+1
            except Exception as exc:
                self.health['error'] = 'write failure: '+str(exc)

        def tick(self):
            try:
                free = shutil.disk_usage(self.run).free
                self.health['free_bytes'] = free
                if free < self.reserve:
                    self.health['error'] = 'disk reserve exhausted'
                graph = dict(self.get_topic_names_and_types())
                for topic, kind in self.topics.items():
                    if topic in self.subs or topic not in graph:
                        continue
                    if graph[topic] != [kind]:
                        raise RuntimeError(f'Topic type mismatch: {topic}')
                    qos = subscription_qos(self, topic)
                    offered = yaml.safe_dump([{'reliability': int(qos.reliability), 'durability': int(qos.durability),
                                               'history': int(qos.history), 'depth': qos.depth}])
                    info = rosbag2_py.TopicMetadata(name=topic, type=kind, serialization_format='cdr',
                                                   offered_qos_profiles=offered)
                    self.writer.create_topic(info)
                    self.subs[topic] = self.create_subscription(get_message(kind), topic,
                        lambda raw, name=topic: self.receive(name, raw), qos, raw=True)
                    self.health['qos'][topic] = {'reliability': str(qos.reliability), 'durability': str(qos.durability)}
                self.health['monotonic_s'] = time.monotonic()
                self.status.publish(String(data=json.dumps(self.health)))
            except Exception as exc:
                self.health['error'] = 'recorder failure: '+str(exc)
                self.status.publish(String(data=json.dumps(self.health)))

    stopping = False
    def interrupted(signum, frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    node = Recorder()
    try:
        while not stopping and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        # Humble closes/flushed the writer on destruction (no close method).
        del node.writer
        node.index.close()
        node.health['closed'] = (node.run/'bag'/'metadata.yaml').exists()
        write_json(node.run/'recorder_final.json', node.health)
        node.collection_lock.close()
        node.destroy_node()
        rclpy.try_shutdown()
