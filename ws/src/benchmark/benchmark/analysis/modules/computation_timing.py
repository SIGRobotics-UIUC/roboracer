"""Only compute timing quantities evidenced by available clock endpoints."""
import numpy as np
from ..core.results import Module, distribution, metric
from ..core.signals import measurement_samples

V2 = {
    'localization_processing': 'Per-observation acquisition/start/end stamps, correlation ID and monotonic duration',
    'controller_callback_execution': 'Unthrottled per-callback monotonic start/end duration telemetry',
    'planning_solver_time': 'Per-plan/per-solve monotonic durations and observation IDs',
    'solver_status': 'Structured per-solve status telemetry',
    'solver_iterations': 'Structured per-solve iteration counts',
    'state_age_at_controller': 'Controller input-consumption stamp and associated estimate stamp/ID',
    'communication_latency': 'Matched send/receive endpoints with verified clock relationship',
    'command_application_latency': 'Command ID and simulator application endpoint stamp with verified clocks',
    'callback_deadline_misses': 'Scheduled/start/completion callback stamps and configured deadlines',
    'physics_time_signal_loss': 'Continuous native simulator physics clock; current /clock is transport time',
    'bridge_processing': 'Existing optional frame trace, complete and finalized, with dropped/error status',
}


def analyze(data, output, config):
    metrics = {name: metric(units='count' if name == 'solver_iterations' else None if name == 'solver_status' else 's',
                            reason='Required telemetry was not recorded', v2_requirement=need)
               for name, need in V2.items()}
    requested = [s for s in measurement_samples(data, 'requested_drive') if s.source_ns is not None
                 and s.domain == data.metadata.get('clock_domain') == 'transport']
    intervals = [(b.source_ns-a.source_ns)*1e-9 for a, b in zip(requested, requested[1:])
                 if a.segment == b.segment and a.domain == b.domain and b.source_ns > a.source_ns]
    metrics['command_publication_interval'] = distribution(intervals, 's (transport)')
    if intervals:
        metrics['command_publication_frequency'] = metric(1/float(np.mean(intervals)), 'Hz (transport)')
    else:
        metrics['command_publication_frequency'] = metric(units='Hz', reason='No continuous command source stamps')
    params = data.metadata.get('config', {}).get('components', {}).get('controller', {})
    period = (1/params.get('params', {}).get('update_rate', 40.)
              if params.get('type') == 'map' and params.get('managed', True) else None)
    metrics['command_publication_jitter'] = distribution(np.array(intervals)-period, 's') if period else metric(
        units='s', reason='No fixed publication period: pure pursuit is pose-callback driven or controller is unsupported')
    domain_ok = data.metadata.get('recorder_clock_domain') == data.metadata.get('clock_domain') == 'transport'
    for name in ('pose', 'requested_drive'):
        negatives = sum(s.recorded_ns < s.source_ns for s in measurement_samples(data, name)
                        if s.valid and s.source_ns is not None and s.domain == 'transport') if domain_ok else 0
        ages = [(s.recorded_ns-s.source_ns)*1e-9 for s in measurement_samples(data, name)
                if s.valid and s.source_ns is not None and s.domain == 'transport'
                and s.recorded_ns >= s.source_ns] if domain_ok else []
        metrics[name+'_source_stamp_age_at_recorder'] = distribution(ages, 's') if domain_ok else metric(
            units='s', reason='Recorder/source clock relationship unverified')
        metrics[name+'_source_stamp_age_at_recorder']['excluded_negative_age_samples'] = negatives
    for name, result in metrics.items():
        result['required_evidence'] = [V2[name]] if name in V2 else (
            ['valid source and recording stamps in a verified common clock'] if 'age_at_recorder' in name
            else ['continuous controller command source stamps', 'configured period for jitter'])
    return {'status': 'ok', 'metrics': metrics, 'warnings': [
        'Publication intervals are not execution durations or callback deadline misses.',
        'Observed publication intervals may include capture gaps and transport-clock quantization.',
        'Negative source-stamp ages are excluded and counted; clock delivery can differ between nodes.',
        'Estimate source-stamp age does not establish sensor age or age at controller consumption.'], 'figures': []}


MODULE = Module('computation_timing', '1.0', (), ('requested_drive', 'pose'), analyze)
