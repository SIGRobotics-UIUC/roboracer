"""Offline unique slip observations and conservative transport-time association."""
from bisect import bisect_left
from collections import Counter
from dataclasses import replace
import numpy as np

from .signals import measurement_samples, aligned_pose, project_reference
from .wheel_contact import WHEEL_NAMES, finite, curve_valid


def unique_contacts(samples):
    """Earliest acquisition wins; bridge envelope stamps are not source payload."""
    seen, unique = {}, []
    duplicates = conflicts = 0
    for sample in sorted(samples, key=lambda s: s.order):
        v = sample.value
        identity = (v['session_id'], v['reset_generation'], v['sample_sequence'])
        payload = {k: x for k, x in v.items() if k != 'stamp_ns'}
        if identity in seen:
            duplicates += 1
            conflicts += payload != seen[identity]
        else:
            seen[identity] = payload
            unique.append(sample)
    return unique, duplicates, conflicts


def wheel_values(value):
    result = {}
    for wheel in value['wheels']:
        item = {'grounded': wheel['grounded'], 'valid': wheel['valid']}
        hit = wheel.get('wheel_hit') or {}
        for direction in ('forward', 'sideways'):
            raw = hit.get(direction+'Slip')
            raw = raw if wheel['grounded'] and wheel['valid'] and finite(raw) else None
            curve = wheel.get(direction+'Friction')
            peak = curve.get('extremumSlip') if isinstance(curve, dict) else None
            ratio = abs(raw)/peak if raw is not None and curve_valid(curve) else None
            item[direction] = {'raw': raw, 'peak': peak if finite(peak) else None,
                               'ratio': ratio if finite(ratio) else None}
        result[wheel['name']] = item
    return result


def prepare(data, config):
    contacts, duplicates, conflicts = unique_contacts(measurement_samples(data, 'wheel_contact'))
    poses = measurement_samples(data, 'pose')
    # Segment IDs alone are insufficient: establish shared acquisition epochs.
    # A transition on either signal, a reset, or a plugin generation change splits
    # both sides. Thus independently segmented signals fail conservatively.
    events = [(s, kind) for kind, source in [('contact', contacts), ('pose', poses),
              ('reset', measurement_samples(data, 'reset')), ('lap_count', measurement_samples(data, 'lap_count'))] for s in source]
    epochs, epoch, previous_segment, generation = {}, 0, None, None
    for s, kind in sorted(events, key=lambda entry: entry[0].order):
        g = (s.value['session_id'], s.value['reset_generation']) if kind == 'contact' else generation
        if (kind == 'reset' or previous_segment is not None and s.segment != previous_segment
                or generation is not None and g != generation):
            epoch += 1
        epochs[id(s)] = epoch
        previous_segment, generation = s.segment, g
    groups = {}
    for s in poses:
        if s.source_ns is not None and s.frame == 'map' and all(finite(v) for v in s.value[:3]):
            groups.setdefault((epochs[id(s)], s.domain), []).append(s)
    prepared = {}
    for key, group in groups.items():
        group.sort(key=lambda s: (s.source_ns, s.order))
        stamps = [s.source_ns for s in group]
        prepared[key] = (group, stamps, Counter(stamps))
    gap = config.get('max_gap_s', .1)
    rows = None
    warnings = []
    if 'raceline.csv' in data.assets:
        try:
            rows = np.loadtxt(data.assets['raceline.csv'], delimiter=';', comments='#', ndmin=2)
            _, length = project_reference([], rows)
        except (ValueError, OSError, IndexError) as exc:
            rows = None
            warnings.append('Preserved raceline unavailable: '+str(exc))
    length = length if rows is not None else None
    use_receipt = data.metadata.get('measurement', {}).get('start_monotonic_ns') is not None
    def boundary(s):
        return s.received_monotonic_ns if use_receipt else s.recorded_ns
    counters = sorted(measurement_samples(data, 'lap_count'), key=lambda s: s.order)
    boundaries = [(b.order, boundary(b)) for a, b in zip(counters, counters[1:])
                  if a.segment == b.segment and b.value == a.value+1]
    recorded_laps = data.metadata.get('measurement', {}).get('laps', [])
    samples = []
    previous = None
    for s in contacts:
        evidence = {'method': 'unavailable', 'reason': 'No compatible bracketed localized pose',
                    'bracketing_stamps_ns': [], 'bracket_span_s': None}
        group, stamps, counts = prepared.get((epochs[id(s)], s.domain), ([], [], {}))
        xy = pose = None
        if s.source_ns is not None and s.domain == 'transport':
            j = bisect_left(stamps, s.source_ns)
            if j < len(group) and stamps[j] == s.source_ns:
                bracket = group[j:j+1]
            else:
                bracket = group[j-1:j+1] if 0 < j < len(group) else []
            if bracket:
                evidence['bracketing_stamps_ns'] = [str(p.source_ns) for p in bracket]
                evidence['bracket_span_s'] = (bracket[-1].source_ns-bracket[0].source_ns)/1e9
                if any(counts[p.source_ns] > 1 for p in bracket):
                    evidence['reason'] = 'Duplicate / ambiguous pose timestamp'
                else:
                    # Reuse bounded interpolation and wrapped yaw policy, with the
                    # common epoch replacing local segment numbers only here.
                    aligned = aligned_pose([replace(p, segment=0) for p in bracket], [replace(s, segment=0)], gap)[0]
                    if np.isfinite(aligned).all():
                        pose = aligned.tolist()
                        xy = pose[:2]
                        evidence.update(method='exact' if len(bracket) == 1 else 'interpolated', reason=None)
                    else:
                        evidence['reason'] = 'Pose bracket exceeds max gap or incompatible time domain'
        d = None
        if pose is not None and rows is not None:
            projection, _ = project_reference([pose], rows)
            if projection[0, 4]:
                d = float(projection[0, 0])
        native_lap = sum(s.order >= order for order, _ in boundaries)
        # Split interrupted laps on common acquisition epochs, while preserving
        # native counter convention and acquisition order.
        lap_key = (native_lap, epochs[id(s)])
        continuous = (previous is not None and previous['_lap_key'] == lap_key
                      and s.domain == previous['domain'] and s.source_ns is not None
                      and previous['stamp_ns'] is not None
                      and 0 < s.source_ns-int(previous['stamp_ns']) <= gap*1e9)
        if continuous and d is not None and previous['d'] is not None and abs(d-previous['d']) > length/2:
            continuous = False
        item = {'source_identity': [s.value[k] for k in ('session_id', 'reset_generation', 'sample_sequence')],
                'stamp_ns': str(s.source_ns) if s.source_ns is not None else None,
                'recorded_ns': str(s.recorded_ns), 'boundary_ns': str(boundary(s)), 'order': s.order,
                'segment': s.segment, 'domain': s.domain, 'xy': xy, 'pose': pose, 'd': d,
                'progress_percent': d/length*100 if d is not None else None,
                'alignment': evidence, 'wheels': wheel_values(s.value), 'continuous': bool(continuous),
                '_lap_key': lap_key}
        samples.append(item)
        previous = item
    laps = []
    for index, item in enumerate(samples):
        key = item.pop('_lap_key')
        if not laps or key != laps[-1]['key']:
            native, _ = key
            info = recorded_laps[native] if native < len(recorded_laps) else {}
            laps.append({'key': key, 'complete': False, 'native_lap': native+1,
                         'start_kind': info.get('start_kind', 'partial'), 'indices': []})
        item['lap'] = len(laps)-1
        laps[-1]['indices'].append(index)
    # A native boundary completes a lap only if it was not split by a discontinuity.
    for lap in laps:
        native = lap['native_lap']-1
        split = sum(x['native_lap'] == native+1 for x in laps) > 1
        end_counter = next((b for b in counters if native < len(boundaries) and b.order == boundaries[native][0]), None)
        end_compatible = end_counter is not None and epochs[id(end_counter)] == lap['key'][1]
        lap['complete'] = native < len(boundaries) and not split and end_compatible
        if not lap['complete']:
            lap['start_kind'] = 'interrupted / partial'
        lap.pop('key')
    return {'samples': samples, 'laps': laps, 'length': length,
            'reference': rows[:, 1:3].tolist() if rows is not None else [],
            'duplicates': duplicates, 'conflicting_duplicates': conflicts, 'warnings': warnings}


def summaries(samples):
    """Full-data sample-weighted summaries, independent of spatial coverage."""
    result = {}
    for direction in ('forward', 'sideways'):
        result[direction] = {}
        for name in WHEEL_NAMES:
            v = [s['wheels'][name][direction] for s in samples]
            raw = [abs(x['raw']) for x in v if x['raw'] is not None]
            ratios = [x['ratio'] for x in v if x['ratio'] is not None]
            def stats(a):
                return {'count': len(a), 'mean': float(np.mean(a)) if a else None,
                        'p95': float(np.percentile(a, 95)) if a else None, 'max': max(a) if a else None}
            result[direction][name] = {'absolute_slip': stats(raw), 'ratio': stats(ratios),
                'at_or_above_peak_count': sum(x >= 1 for x in ratios),
                'at_or_above_peak_fraction': sum(x >= 1 for x in ratios)/len(ratios) if ratios else None}
    return result
