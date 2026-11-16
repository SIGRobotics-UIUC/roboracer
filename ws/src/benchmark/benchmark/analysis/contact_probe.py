"""Read-only live contact capture. Never creates drive/reset publishers.

Run inside the bridge's container/domain; outputs use an exclusive new directory.
These diagnostic captures are not managed/sealed benchmark race collections.
"""
import argparse
import json
import math
from pathlib import Path
import shutil
import time

from .core.wheel_contact import decode_contact
from ..collector.artifacts import unique_directory, write_json, utc, git_snapshot

CONTACT_TOPIC = '/autodrive/roboracer_1/wheel_contact'


def contact_record(raw, received_monotonic_ns):
    record = {'received_monotonic_ns': received_monotonic_ns, 'raw': raw}
    try:
        record['contact'] = decode_contact(raw)
        record['valid_envelope'] = True
    except (ValueError, TypeError, RecursionError) as exc:
        record.update(valid_envelope=False, error=str(exc))
    return record


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30.)
    parser.add_argument('--discovery-timeout', type=float, default=10.)
    parser.add_argument('--output-root', type=Path, default=Path.cwd()/'log/wheel_contact_checks')
    args = parser.parse_args(argv)
    if not all(math.isfinite(x) and x > 0 for x in (args.seconds, args.discovery_timeout)):
        parser.error('durations must be finite and positive')
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String, Float32
    from ackermann_msgs.msg import AckermannDriveStamped

    output = unique_directory(args.output_root)
    print('Capture directory: '+str(output.resolve()), flush=True)
    import os
    summary = {'started_utc': utc(), 'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
               'requested_seconds': args.seconds, 'contact_messages': 0, 'valid_envelopes': 0,
               'invalid_envelopes': 0, 'status': 'capturing',
               'limitations': ['Read-only observation; no reset, steering or speed commands are sent.',
                               'One continuous capture is not three independent controlled experiments.',
                               'Stationarity, ground flatness and absence of non-wheel support are not established by capture alone.']}
    write_json(output/'capture.json', summary)
    rclpy.init()
    node = rclpy.create_node('wheel_contact_readonly_capture')
    first_contact = None
    start = time.monotonic()
    reason = 'complete'
    with (output/'contacts.jsonl').open('x') as contacts, (output/'commands.jsonl').open('x') as commands:
        def receive(message):
            nonlocal first_contact
            now = time.monotonic_ns()
            record = contact_record(message.data, now)
            contacts.write(json.dumps(record, allow_nan=False)+'\n')
            summary['contact_messages'] += 1
            summary['valid_envelopes' if record['valid_envelope'] else 'invalid_envelopes'] += 1
            if first_contact is None and record['valid_envelope']:
                first_contact = time.monotonic()
                print('Valid contact stream discovered; recording.', flush=True)
        def command(topic, value):
            commands.write(json.dumps({'topic': topic, 'received_monotonic_ns': time.monotonic_ns(),
                                       'value': value}, allow_nan=False)+'\n')
        def drive(message):
            d = message.drive
            command('/drive', {'speed_mps': repr(d.speed), 'steering_rad': repr(d.steering_angle)})
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(String, CONTACT_TOPIC, receive, qos)
        node.create_subscription(AckermannDriveStamped, '/drive', drive, qos)
        for suffix in ('throttle_command', 'steering_command'):
            topic = '/autodrive/roboracer_1/'+suffix
            node.create_subscription(Float32, topic, lambda m, t=topic: command(t, repr(m.data)), qos)
        try:
            next_disk_check = start
            while rclpy.ok():
                now = time.monotonic()
                if first_contact is None and now-start >= args.discovery_timeout:
                    reason = 'no_valid_contact_stream'; break
                if first_contact is not None and now-first_contact >= args.seconds:
                    break
                if now >= next_disk_check:
                    if shutil.disk_usage(output).free < 64*1024*1024:
                        reason = 'disk_reserve'; break
                    next_disk_check = now+1
                rclpy.spin_once(node, timeout_sec=.1)
        except KeyboardInterrupt:
            reason = 'interrupted'
        finally:
            summary['nodes'] = node.get_node_names()
            summary['contact_publishers'] = [e.node_name for e in node.get_publishers_info_by_topic(CONTACT_TOPIC)]
            summary['drive_publishers'] = [e.node_name for e in node.get_publishers_info_by_topic('/drive')]
            summary['elapsed_seconds'] = time.monotonic()-start
            summary['recording_seconds'] = None if first_contact is None else time.monotonic()-first_contact
            summary['status'] = reason
            summary['finished_utc'] = utc()
            node.destroy_node()
            rclpy.try_shutdown()
            write_json(output/'capture.json', summary)
    # Capture exact working diffs/revisions for both source checkouts when present.
    repo = Path(os.environ.get('BENCHMARK_REPO_ROOT', '/repository'))
    summary['roboracer_source'] = git_snapshot(repo, output/'source')
    if (repo/'autodrive_plugin/.git').exists():
        summary['plugin_source'] = git_snapshot(repo/'autodrive_plugin', output/'plugin_source')
    write_json(output/'capture.json', summary)
    print(json.dumps({k: summary[k] for k in ('status', 'contact_messages', 'valid_envelopes', 'invalid_envelopes')}, indent=2))
    print('Capture directory: '+str(output.resolve()), flush=True)
    return 0 if reason == 'complete' and summary['valid_envelopes'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
