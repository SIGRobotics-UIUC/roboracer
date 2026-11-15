"""Offline rosbag2 loading. No nodes, graph discovery, or simulator connection."""
import json
import hashlib
from collections import defaultdict, deque
from pathlib import Path
import numpy as np
import yaml

from .signals import Dataset, Sample, SIGNAL_MAP, pose_value
from ...collector.artifacts import verify


def load_run(run):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    run = Path(run)
    metadata = verify(run)
    dataset = Dataset(metadata=metadata)
    for name in ('raceline.csv', 'map_to_world.yaml', 'map.yaml', 'map.pgm', 'vehicle.yaml'):
        path = run / 'assets' / name
        if path.exists():
            dataset.assets[name] = path
    if metadata.get('config', {}).get('logging', {}).get('level') == 'off':
        dataset.warnings.append('Logging was off; telemetry-dependent metrics are unavailable')
        return dataset
    bag = run / 'bag'
    if not (bag / 'metadata.yaml').exists():
        dataset.warnings.append('No finalized readable bag; collection failed')
        return dataset
    reader = rosbag2_py.SequentialReader()
    bag_info = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    storage = bag_info['storage_identifier']
    dataset.metadata['recording_duration_s'] = bag_info.get('duration', {}).get('nanoseconds', 0)/1e9
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    mapping = {v['topic']: k for k, v in SIGNAL_MAP.items()}
    mapping[metadata.get('requested_topic', '/benchmark/requested_drive')] = 'requested_drive'
    segment, order, previous_record = 0, 0, None
    highs = {}
    raw = {}
    # Storage readers may sort by recording time, hiding clock resets. Restore
    # writer acceptance order from the immutable index, including duplicates.
    index_path = run/'capture_index.jsonl'
    index = defaultdict(deque)
    if index_path.exists():
        for line in index_path.read_text().splitlines():
            record = json.loads(line)
            index[(record['topic'], record['recorded_ns'], record['sha256'])].append(record)
    else:
        dataset.warnings.append('Capture-order index unavailable; telemetry alignment is unavailable because acquisition order is unverified')
        return dataset
    records = []
    while reader.has_next():
        topic, serialized, received = reader.read_next()
        if topic not in mapping and topic != '/clock':
            continue
        key = (topic, received, hashlib.sha256(serialized).hexdigest())
        if not index.get(key):
            dataset.warnings.append('Unindexed message omitted from analysis; raw bag retained')
            continue
        entry = index[key].popleft()
        records.append((entry['sequence'], topic, serialized, received, entry.get('received_monotonic_ns')))
    if index_path.exists():
        records.sort(key=lambda record: record[0])
    for sequence, topic, serialized, received, received_monotonic_ns in records:
        message = deserialize_message(serialized, get_message(types[topic]))
        name = mapping.get(topic, 'clock')
        header = getattr(message, 'header', None)
        stamp = header.stamp.sec*10**9+header.stamp.nanosec if header else None
        if topic == '/clock':
            stamp = message.clock.sec*10**9+message.clock.nanosec
        boundary = name == 'reset' or (previous_record is not None and received < previous_record)
        if stamp is not None and name in highs and stamp < highs[name]-100_000_000:
            boundary = True
            dataset.warnings.append('Ambiguous source regression: global alignment segment split')
        if boundary:
            segment += 1
            highs.clear()
        if stamp is not None:
            if name in highs and stamp < highs[name]:
                dataset.warnings.append(f'{name}: delayed/out-of-order delivery')
            highs[name] = max(stamp, highs.get(name, stamp))
        previous_record = received
        frame = header.frame_id if header else ''
        value, valid = decode(name, message)
        if header and (stamp is None or stamp <= 0):
            valid = False
        expected = SIGNAL_MAP.get(name, {}).get('frame')
        if name in ('pose', 'tracked_pose', 'truth', 'drive', 'requested_drive'):
            valid = valid and frame == expected
        if name == 'odom':
            valid = valid and frame == 'odom' and message.child_frame_id == 'base_link'
        if name == 'truth':
            valid = valid and message.child_frame_id == SIGNAL_MAP['truth']['child_frame']
        sample = Sample(stamp, received, value, frame, sequence, segment,
                        metadata.get('clock_domain', 'unknown'), valid, received_monotonic_ns)
        raw.setdefault(name, []).append(sample)
        order += 1
    for name, samples in raw.items():
        dataset.shared.setdefault('raw_order', {})[name] = samples
        dataset.signals[name] = sorted(samples, key=lambda s: (s.segment, s.source_ns if s.source_ns is not None else s.recorded_ns, s.order))
        keys = [(s.segment, s.source_ns) for s in samples if s.source_ns is not None]
        if len(set(keys)) != len(keys):
            dataset.warnings.append(f'{name}: duplicate source stamps retained')
        invalid = sum(not sample.valid for sample in samples)
        if invalid:
            dataset.warnings.append(f'{name}: {invalid}/{len(samples)} samples invalid (frame, stamp or numeric checks)')
    dataset.warnings = sorted(set(dataset.warnings))
    return dataset


def decode(name, message):
    if name in ('pose', 'tracked_pose'):
        value = pose_value(message.pose)
        return value, value is not None
    if name in ('odom', 'truth'):
        pose = pose_value(message.pose.pose)
        v = message.twist.twist
        value = (pose or [np.nan]*3)+[v.linear.x, v.linear.y, v.angular.z]
        return value, pose is not None and bool(np.isfinite(value).all())
    if name in ('drive', 'requested_drive'):
        value = [message.drive.speed, message.drive.steering_angle]
        return value, bool(np.isfinite(value).all()) and value[0] >= 0
    if name == 'clock':
        return message.clock.sec*10**9+message.clock.nanosec, True
    if name == 'event':
        try:
            return json.loads(message.data), True
        except ValueError:
            return message.data, False
    value = message.data
    return value, not isinstance(value, float) or np.isfinite(value)
