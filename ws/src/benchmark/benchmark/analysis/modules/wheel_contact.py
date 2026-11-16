"""Raw wheel-contact diagnostics and slip-coordinate proxies, evaluation only."""
from collections import defaultdict
import json
import math

from ..core.results import Module, distribution, metric
from ..core.signals import measurement_samples
from ..core.wheel_contact import WHEEL_NAMES, finite
from ..core.wheel_slip import prepare, summaries
from .wheel_contact_view import render_report, extension


def analyze(data, output, config):
    samples = measurement_samples(data, 'wheel_contact')
    metrics = {}
    viewer = prepare(data, config)
    summary = summaries(viewer['samples'])
    from .race_view import preserved_payload
    try:
        race_viewer = preserved_payload(data, config)
    except (ValueError, OSError, KeyError, ImportError) as exc:
        race_viewer = {'laps': [], 'reason': 'Shared spatial viewer unavailable: '+str(exc)}
        viewer['warnings'].append(race_viewer['reason'])
    with (output/'slip_samples.jsonl').open('w') as stream:
        for observation in viewer['samples']:
            stream.write(json.dumps(observation, allow_nan=False)+'\n')
    (output/'slip_viewer.json').write_text(json.dumps(viewer, allow_nan=False))
    if not samples:
        return {'status': 'partial', 'metrics': {'contact_samples': metric(
            units='count', reason='No valid contact envelopes in measurement window; old bags remain unavailable')},
            'warnings': [], 'figures': [], 'viewer': viewer, 'race_viewer': race_viewer, 'slip_summary': summary}
    values = defaultdict(list)
    seen, revisions, sessions = set(), set(), set()
    repeated, invalid, ungrounded, skipped, phase_unverified = 0, 0, 0, 0, 0
    invalid_snapshots, missing_hashes = 0, 0
    previous = {}
    source_gaps, source_regressions = 0, 0
    # Raw copies, including invalid-wheel tags and complete snapshots, are report
    # artifacts only. Never modify a sealed collection or use these for control.
    with (output/'contacts.jsonl').open('w') as stream:
        for sample in sorted(samples, key=lambda s: s.order):
            value = sample.value
            stream.write(json.dumps({'recorded_ns': sample.recorded_ns, 'segment': sample.segment,
                                     'contact': value}, allow_nan=False)+'\n')
            identity = (value['session_id'], value['reset_generation'], value['sample_sequence'])
            if identity in seen:
                repeated += 1
                continue
            seen.add(identity)
            sessions.add(value['session_id'])
            revisions.add((value['session_id'], value['configuration_revision']))
            invalid_snapshots += value.get('runtime_snapshot_valid') is not True
            build = value.get('build')
            missing_hashes += not (isinstance(build, dict) and build.get('plugin_sha256') and build.get('native_game_assembly_sha256'))
            source = value['source']
            phase_unverified += source.get('phase_verified') is not True
            key = (sample.segment, value['session_id'], value['reset_generation'])
            seq = value['sample_sequence']
            if key in previous:
                delta = seq-previous[key]
                source_gaps += max(0, delta-1)
                source_regressions += delta <= 0
            previous[key] = seq
            gap = source.get('skipped_fixed_callbacks')
            if type(gap) is int and gap >= 0:
                skipped += gap
            forces = []
            for wheel in value['wheels']:
                prefix = wheel['name']+'_'
                values[prefix+'grounded'].append(int(wheel['grounded']))
                if not wheel['grounded']:
                    ungrounded += 1
                    continue
                if not wheel['valid'] or not isinstance(wheel.get('wheel_hit'), dict):
                    invalid += 1
                    continue
                hit = wheel['wheel_hit']
                for field in ('force',):
                    if finite(hit.get(field)):
                        values[prefix+field].append(hit[field])
                if finite(hit.get('force')):
                    forces.append(hit['force'])  # Negative values retained, never clipped.
                normal = hit.get('normal')
                if isinstance(normal, dict) and all(finite(normal.get(k)) for k in ('x', 'y', 'z')):
                    values[prefix+'normal_magnitude'].append(math.hypot(*(normal[k] for k in ('x', 'y', 'z'))))
            if len(forces) == 4:
                values['four_grounded_raw_force_sum'].append(sum(forces))
    # All slip distributions share the preparation path used by exports/viewer.
    for observation in viewer['samples']:
        for name, wheel in observation['wheels'].items():
            for direction in ('forward', 'sideways'):
                value = wheel[direction]
                if value['raw'] is not None:
                    values[name+'_'+direction+'Slip'].append(value['raw'])
                if value['ratio'] is not None:
                    values[name+'_'+direction+'_slip_to_configured_peak'].append(value['ratio'])
    metrics.update(contact_samples=metric(len(samples), 'transport observations'),
                   unique_source_samples=metric(len(seen), 'source observations'),
                   repeated_source_samples=metric(repeated, 'transport observations'),
                   source_sequence_gaps=metric(source_gaps, 'source observations missing'),
                   source_sequence_regressions=metric(source_regressions, 'count'),
                   skipped_fixed_callbacks=metric(skipped, 'observed callbacks; not verified solver steps'),
                   runtime_configuration_revisions=metric(len(revisions), 'count'),
                   unverified_or_invalid_runtime_snapshots=metric(invalid_snapshots, 'source observations'),
                   samples_missing_plugin_or_native_build_hash=metric(missing_hashes, 'source observations'),
                   source_sessions=metric(len(sessions), 'count'),
                   ungrounded_wheel_samples=metric(ungrounded, 'wheel observations'),
                   invalid_grounded_wheel_samples=metric(invalid, 'wheel observations'),
                   unverified_sampling_phase_samples=metric(phase_unverified, 'source observations'))
    metrics['grounded_wheel_samples'] = metric(sum(w['grounded'] for s in viewer['samples'] for w in s['wheels'].values()), 'wheel observations')
    metrics['conflicting_duplicate_samples'] = metric(viewer['conflicting_duplicates'], 'transport observations')
    for method in ('exact', 'interpolated', 'unavailable'):
        metrics['spatial_'+method+'_samples'] = metric(sum(s['alignment']['method'] == method for s in viewer['samples']), 'unique source observations')
    metrics['spatial_match_fraction'] = metric(sum(s['xy'] is not None for s in viewer['samples'])/len(viewer['samples']), 'fraction of unique source observations')
    metrics['complete_laps'] = metric(sum(l['complete'] for l in viewer['laps']), 'count')
    metrics['partial_laps'] = metric(sum(not l['complete'] for l in viewer['laps']), 'count')
    for name in WHEEL_NAMES:
        prefix = name+'_'
        for field, units in [('grounded', 'fraction of observed samples'), ('force', 'raw WheelHit.force scalar'),
                             ('forwardSlip', 'Unity slip coordinate'), ('sidewaysSlip', 'Unity slip coordinate'),
                             ('normal_magnitude', 'dimensionless')]:
            metrics[prefix+field] = distribution(values[prefix+field], units)
        for direction in ('forward', 'sideways'):
            metrics[prefix+direction+'_absolute_slip'] = distribution([abs(v) for v in values[prefix+direction+'Slip']], 'Unity slip coordinate magnitude')
            peaks = [s['wheels'][name][direction]['peak'] for s in viewer['samples']]
            metrics[prefix+direction+'_peak_denominator_changes'] = metric(sum(a != b for a, b in zip(peaks, peaks[1:])), 'observed changes; revisions may include numerical noise')
            key = prefix+direction+'_slip_to_configured_peak'
            ratios = values[key]
            metrics[key] = distribution(ratios, 'abs(raw slip) / runtime extremumSlip')
            metrics[key+'_at_or_above_one_sample_fraction'] = metric(
                sum(x >= 1 for x in ratios)/len(ratios) if ratios else None,
                'fraction of valid unique samples', reason='No valid grounded slip/curve samples')
    metrics['four_grounded_raw_force_sum'] = distribution(values['four_grounded_raw_force_sum'], 'sum of raw WheelHit.force scalars')
    return {'status': 'ok', 'metrics': metrics, 'figures': [], 'viewer': viewer, 'race_viewer': race_viewer, 'slip_summary': summary,
            'contamination': {'measurement': data.metadata.get('measurement', {}), 'resets': len(measurement_samples(data, 'reset')), 'collisions': [s.value for s in measurement_samples(data, 'collision_count')]},
            'warnings': viewer['warnings']+[
        'Evaluation only. Raw WheelHit.force is not validated vertical load or per-axis tire force.',
        'Slip-to-configured-peak ratios are slip coordinates, not grip percentages, utilization or saturation flags.',
        'No clock synchronization or solved-physics sampling phase is established. Repeated source samples are excluded from distributions.',
        'LateUpdate sampling may omit intermediate physics states; callback gaps are not a complete local physics trace.',
        'contacts.jsonl retains runtime snapshots/build hashes. Missing hashes and external-force inventories remain unverified.',
        'Fractions describe sampled observations, not elapsed time above a threshold.']}


MODULE = Module('wheel_contact', '1.1', (), ('wheel_contact', 'pose', 'lap_count', 'reset'), analyze, render_report, report_extensions=extension)
