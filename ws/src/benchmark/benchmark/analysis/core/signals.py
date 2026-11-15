"""Versioned native-message signal map and common clock/frame alignment."""
from dataclasses import dataclass, field
import math
import numpy as np

NS = '/autodrive/roboracer_1/'
SIGNAL_MAP_VERSION = 2
SIGNAL_MAP = {
    'pose': {'topic': '/localized_pose', 'field': 'pose', 'units': 'm,rad', 'frame': 'map'},
    'tracked_pose': {'topic': '/tracked_pose', 'field': 'pose', 'units': 'm,rad', 'frame': 'map'},
    'odom': {'topic': '/odom', 'field': 'pose.pose,twist.twist', 'units': 'm,rad,m/s,rad/s', 'frame': 'odom', 'child_frame': 'base_link'},
    'truth': {'topic': '/ground_truth/odom', 'field': 'pose.pose,twist.twist', 'units': 'm,rad,m/s,rad/s', 'frame': 'world', 'child_frame': 'ground_truth'},
    'drive': {'topic': '/drive', 'field': 'drive.speed,drive.steering_angle', 'units': 'm/s,rad', 'frame': 'base_link'},
    'requested_drive': {'topic': '/benchmark/requested_drive', 'field': 'drive.speed,drive.steering_angle', 'units': 'm/s,rad', 'frame': 'base_link'},
    'lap_count': {'topic': NS+'lap_count', 'field': 'data', 'units': 'count', 'frame': None},
    'collision_count': {'topic': NS+'collision_count', 'field': 'data', 'units': 'count', 'frame': None},
    'last_lap_time': {'topic': NS+'last_lap_time', 'field': 'data', 'units': 's (native simulator)', 'frame': None},
    'reset': {'topic': NS+'sim_reset', 'field': 'data', 'units': None, 'frame': None},
    'event': {'topic': '/benchmark/events', 'field': 'data', 'units': None, 'frame': None},
}
for _name, _spec in SIGNAL_MAP.items():
    _spec['timestamp'] = {'source': None if _name in ('lap_count', 'collision_count', 'last_lap_time', 'reset', 'event')
                         else 'header.stamp', 'recorded': 'rosbag timestamp', 'domain': 'run.clock_domain',
                         'provenance': 'Bridge stamps represent transport arrival; localizer/command stamps retain node semantics'}
    _spec['sign_conventions'] = 'x forward, y left; positive yaw/steering counterclockwise'
SIGNAL_MAP['odom']['covariance'] = 'pose.covariance, twist.covariance retained raw; zeros do not imply measured certainty'
SIGNAL_MAP['truth']['evaluation_only'] = True


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


@dataclass
class Sample:
    source_ns: int | None
    recorded_ns: int
    value: object
    frame: str = ''
    order: int = 0
    segment: int = 0
    domain: str = 'transport'
    valid: bool = True
    received_monotonic_ns: int | None = None
    availability: str = 'available'
    rejection_reasons: tuple = ()


@dataclass
class Dataset:
    signals: dict = field(default_factory=dict)
    metadata: dict = field(default_factory=dict)
    assets: dict = field(default_factory=dict)
    warnings: list = field(default_factory=list)
    shared: dict = field(default_factory=dict)


def measurement_samples(data, name):
    """Shared phase selection; receipt monotonic time survives a frozen ROS clock."""
    bounds = data.metadata.get('measurement', {})
    if bounds.get('started') is False:
        return []
    start, end = bounds.get('start_recorded_ns'), bounds.get('end_recorded_ns')
    mono_start, mono_end = bounds.get('start_monotonic_ns'), bounds.get('end_monotonic_ns')
    samples = []
    for sample in data.signals.get(name, []):
        if not sample.valid:
            continue
        if mono_start is not None or mono_end is not None:
            if (sample.received_monotonic_ns is None
                    or data.metadata.get('receipt_monotonic_domain') != 'collector_container_monotonic'):
                warning = 'Measurement receipt-time alignment unavailable; samples without verified monotonic receipt times excluded'
                if warning not in data.warnings:
                    data.warnings.append(warning)
                continue
            stamp, first, last = sample.received_monotonic_ns, mono_start, mono_end
        else:
            stamp, first, last = sample.recorded_ns, start, end
        if (first is None or stamp >= first) and (last is None or stamp <= last):
            samples.append(sample)
    return samples


def segment_and_sort(samples, reset_records=(), reorder_ns=100_000_000):
    """Preserve raw samples; sort copies inside defensible continuous segments.

    Small regressions are delayed delivery. Larger uncorroborated regressions
    conservatively break the segment (an ambiguity, not a claimed clock reset).
    Recorder-clock regression and explicit reset evidence also break segments.
    """
    from dataclasses import replace
    output, warnings = [], []
    segment, high, previous_record = 0, None, None
    reset_records = sorted(reset_records)
    reset_index = 0
    for raw in samples:
        stamp = raw.source_ns if raw.source_ns is not None else raw.recorded_ns
        explicit = False
        while reset_index < len(reset_records) and reset_records[reset_index] <= raw.recorded_ns:
            explicit = True
            reset_index += 1
        regressed = previous_record is not None and raw.recorded_ns < previous_record
        ambiguous = high is not None and stamp < high - reorder_ns
        if explicit or regressed or ambiguous:
            segment += 1
            high = None
            warnings.append('reset boundary' if explicit else 'recorder clock regression' if regressed
                            else 'ambiguous source discontinuity; alignment split')
        elif high is not None and stamp < high:
            warnings.append('out-of-order delivery')
        output.append(replace(raw, segment=segment))
        high = stamp if high is None else max(stamp, high)
        previous_record = raw.recorded_ns
    output.sort(key=lambda s: (s.segment, s.source_ns if s.source_ns is not None else s.recorded_ns, s.order))
    seen = set()
    for sample in output:
        key = sample.segment, sample.source_ns
        if sample.source_ns is not None and key in seen:
            warnings.append('duplicate source timestamp')
        seen.add(key)
    return output, sorted(set(warnings))


def pose_value(pose):
    p, q = pose.position, pose.orientation
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    norm = math.sqrt(sum(v*v for v in values[3:]))
    if not all(math.isfinite(v) for v in values) or abs(norm-1) > 0.01:
        return None
    return [p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]


def transform_world(values, origin):
    values = np.array(values, dtype=float, copy=True)
    c, s = math.cos(origin['yaw_rad']), math.sin(origin['yaw_rad'])
    dx, dy = values[:, 0]-origin['x_m'], values[:, 1]-origin['y_m']
    values[:, 0], values[:, 1] = c*dx+s*dy, -s*dx+c*dy
    values[:, 2] = wrap(values[:, 2]-origin['yaw_rad'])
    return values


def aligned_pose(source, targets, max_gap_s=0.1):
    """Interpolate within segment/domain, no extrapolation, duplicates or long gaps."""
    result = np.full((len(targets), 3), np.nan)
    groups = {}
    for sample in source:
        if sample.valid and sample.source_ns is not None:
            groups.setdefault((sample.segment, sample.domain), []).append(sample)
    prepared = {}
    for key, group in groups.items():
        stamps = np.array([s.source_ns for s in group], dtype=np.int64)
        duplicates = set(stamps[:-1][stamps[:-1] == stamps[1:]])
        prepared[key] = (group, stamps, duplicates)
    for i, target in enumerate(targets):
        entry = prepared.get((target.segment, target.domain))
        if not target.valid or target.source_ns is None or entry is None or target.domain in ('unknown', 'unverified'):
            continue
        group, stamps, duplicates = entry
        j = int(np.searchsorted(stamps, target.source_ns))
        if j < len(group) and stamps[j] == target.source_ns:
            if (j+1 == len(group) or stamps[j+1] != stamps[j]) and (j == 0 or stamps[j-1] != stamps[j]):
                result[i] = group[j].value[:3]
        elif (0 < j < len(group) and stamps[j] not in duplicates and stamps[j-1] not in duplicates
              and 0 < stamps[j]-stamps[j-1] <= max_gap_s*1e9):
            a, b = np.array(group[j-1].value[:3]), np.array(group[j].value[:3])
            f = (target.source_ns-stamps[j-1])/(stamps[j]-stamps[j-1])
            result[i] = a+f*(b-a)
            result[i, 2] = wrap(a[2]+f*wrap(b[2]-a[2]))
    return result


def project_reference(values, rows):
    """Closest polyline projection; signed left offset and interpolated TUM reference.

    Ambiguous equal-distance segments with different progress are invalidated.
    """
    rows = np.asarray(rows)
    if np.linalg.norm(rows[0, 1:3]-rows[-1, 1:3]) < 1e-6:
        rows = rows[:-1]
    a = rows[:, 1:3]
    delta = np.roll(a, -1, axis=0)-a
    lengths = np.linalg.norm(delta, axis=1)
    if np.any(lengths < 1e-8):
        raise ValueError('Reference contains zero-length segments')
    s0 = np.r_[0., np.cumsum(lengths)[:-1]]
    output = []
    for x, y, yaw in values:
        t = np.clip(np.sum(([x, y]-a)*delta, axis=1)/lengths**2, 0., 1.)
        offset = [x, y]-(a+t[:, None]*delta)
        distance = np.linalg.norm(offset, axis=1)
        j = int(np.argmin(distance))
        candidates = np.flatnonzero(distance < distance[j]+1e-7)
        ambiguous = any(min((int(k)-j) % len(a), (j-int(k)) % len(a)) > 1 for k in candidates)
        lateral = (delta[j, 0]*offset[j, 1]-delta[j, 1]*offset[j, 0])/lengths[j]
        heading = rows[j, 3]+np.pi/2
        heading += t[j]*wrap(rows[(j+1) % len(rows), 3]-rows[j, 3])
        speed = rows[j, 5]+t[j]*(rows[(j+1) % len(rows), 5]-rows[j, 5])
        output.append([s0[j]+t[j]*lengths[j], lateral, wrap(yaw-heading), speed, not ambiguous])
    return np.asarray(output).reshape((-1, 5)), float(lengths.sum())
