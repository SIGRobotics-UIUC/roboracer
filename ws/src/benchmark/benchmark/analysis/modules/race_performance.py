"""Race metrics distinguish measured events, estimated tracking and evaluation truth."""
from dataclasses import replace
import json
import numpy as np
import yaml

from ..core.results import Module, distribution, metric
from ..core.signals import aligned_pose, project_reference, transform_world, wrap, measurement_samples

EVIDENCE = {
    'lap_count': ('lap_count:continuous native counter',),
    'collision_count': ('collision_count:continuous native counter or lifecycle baselines',),
    'lap_completion_fraction': ('lifecycle completed/requested lap counts',),
    'lap_time': ('associated native lap counter/time events',),
    'flying_lap_time': ('associated native lap events after an established crossing',),
    'failure_classification': ('collection lifecycle metadata',),
    'estimated_lateral_error': ('valid map-frame pose', 'preserved raceline'),
    'estimated_heading_error': ('valid map-frame pose', 'preserved raceline'),
    'estimated_absolute_lateral_error': ('valid map-frame pose', 'preserved raceline'),
    'estimated_absolute_heading_error': ('valid map-frame pose', 'preserved raceline'),
    'final_reference_progress': ('valid map-frame pose', 'unambiguous preserved raceline projection'),
    'localization_position_error': ('pose', 'truth', 'map/world transform', 'admissible clock alignment'),
    'localization_heading_error': ('pose', 'truth', 'map/world transform', 'admissible clock alignment'),
    'true_lateral_error': ('truth', 'map/world transform', 'preserved raceline'),
    'true_heading_error': ('truth', 'map/world transform', 'preserved raceline'),
    'wheel_speed_minus_spatial_reference': ('odom', 'pose', 'raceline', 'verified effective speed scaling/caps'),
    'wheel_forward_speed': ('valid body-frame odom linear.x',),
    'spatial_reference_speed': ('odom', 'pose', 'raceline', 'verified effective speed scaling/caps'),
    'controller_internal_reference': ('published selected controller target/heading/speed',),
    'exact_collision_timestamp': ('native simulator-stamped collision event',),
    'map_center_clearance': ('preserved occupancy map', 'map-frame pose'),
    'minimum_map_center_clearance': ('preserved occupancy map', 'map-frame pose'),
    'track_boundary_body_clearance': ('physical boundary geometry', 'complete vehicle footprint', 'aligned pose'),
    'native_track_completion': ('native simulator track-completion signal',),
}


def counter_total(samples):
    if not samples:
        return None
    total = 0
    previous = None
    for sample in sorted(samples, key=lambda s: s.order):
        if not sample.valid:
            continue
        if previous is not None:
            if sample.segment != previous.segment or sample.value < previous.value:
                return None
            total += sample.value-previous.value
        previous = sample
    return total


def analyze(data, output, config):
    metrics, warnings, figures = {}, [], []
    meta = data.metadata
    measured = meta.get('measurement', {})
    def selected(name):
        return measurement_samples(data, name)
    for name in ('lap_count', 'collision_count'):
        total = counter_total(selected(name))
        metrics[name] = metric(total, 'count', None if total is not None else 'No continuous counter evidence')
    completed = measured.get('completed_laps')
    if completed is not None:
        metrics['lap_count'] = metric(completed, 'count', evidence_source='native lifecycle counter observations')
    requested = meta.get('config', {}).get('experiment', {}).get('laps')
    metrics['lap_completion_fraction'] = metric(completed/requested if completed is not None and requested else None,
                                               'fraction', 'Requires lifecycle lap baseline and requested laps')
    laps = measured.get('laps', [])
    valid_times = [lap['time_s'] for lap in laps if lap.get('valid') and (lap.get('time_s') or 0) > 0]
    metrics['lap_time'] = distribution(valid_times, 's (native simulator)')
    metrics['flying_lap_time'] = distribution([lap['time_s'] for lap in laps if lap.get('valid')
        and lap.get('start_kind') == 'flying' and (lap.get('time_s') or 0) > 0], 's (native simulator)')
    if 'collision_baseline' in measured and 'end_collision_count' in measured:
        delta = measured['end_collision_count']-measured['collision_baseline']
        metrics['collision_count'] = metric(delta if delta >= 0 else None, 'count', 'Counter reset invalidates collision delta')
    metrics['failure_classification'] = metric(meta.get('failure') or meta.get('status'), 'category')
    pose = selected('pose')
    rows = None
    projections = None
    if 'raceline.csv' in data.assets and pose:
        rows = np.loadtxt(data.assets['raceline.csv'], delimiter=';', comments='#', ndmin=2)
        projections, length = project_reference([s.value[:3] for s in pose], rows)
        good = projections[:, 4].astype(bool)
        metrics['estimated_lateral_error'] = distribution(projections[good, 1], 'm (left positive)', magnitude_percentile=True)
        metrics['estimated_heading_error'] = distribution(projections[good, 2], 'rad (estimate minus reference)')
        metrics['estimated_absolute_lateral_error'] = distribution(abs(projections[good, 1]), 'm')
        metrics['estimated_absolute_heading_error'] = distribution(abs(projections[good, 2]), 'rad')
        metrics['final_reference_progress'] = metric(float(projections[-1, 0]/length*100) if good[-1] else None,
                                                      '% of reference lap', 'Ambiguous reference projection')
        data.shared['reference_projection'] = projections
    else:
        for name, units in [('estimated_lateral_error', 'm'), ('estimated_heading_error', 'rad'),
                            ('final_reference_progress', '%')]:
            metrics[name] = metric(units=units, reason='Requires valid map-frame poses and preserved raceline')
    truth = selected('truth')
    truth_map = []
    if truth and 'map_to_world.yaml' in data.assets:
        origin = yaml.safe_load(data.assets['map_to_world.yaml'].read_text())
        values = transform_world([s.value[:3] for s in truth], origin)
        truth_map = [replace(s, value=list(v)+s.value[3:], frame='map') for s, v in zip(truth, values)]
    if truth_map and pose:
        warnings.append('Ground truth uses bridge-arrival stamps, not simulator acquisition stamps; '
                        'localization comparison includes unmeasured simulator/transport delay')
        aligned = aligned_pose(truth_map, pose, config.get('max_gap_s', 0.1))
        errors = np.array([s.value[:3] for s in pose])-aligned
        metrics['localization_position_error'] = distribution(np.linalg.norm(errors[:, :2], axis=1), 'm')
        metrics['localization_heading_error'] = distribution(wrap(errors[:, 2]), 'rad (estimate minus truth)')
    else:
        for name, units in [('localization_position_error', 'm'), ('localization_heading_error', 'rad')]:
            metrics[name] = metric(units=units, reason='Requires ground truth, map/world alignment and matching clocks')
    if truth_map and rows is not None:
        projected, _ = project_reference([s.value[:3] for s in truth_map], rows)
        valid = projected[:, 4].astype(bool)
        metrics['true_lateral_error'] = distribution(projected[valid, 1], 'm (left positive)', magnitude_percentile=True)
        metrics['true_heading_error'] = distribution(projected[valid, 2], 'rad')
    else:
        metrics['true_lateral_error'] = metric(units='m', reason='Requires aligned ground truth and raceline')
        metrics['true_heading_error'] = metric(units='rad', reason='Requires aligned ground truth and raceline')
    odom = selected('odom')
    metrics['wheel_forward_speed'] = distribution([s.value[3] for s in odom], 'm/s')
    if rows is not None and pose and odom:
        # Align position to each velocity observation centrally; no extrapolation.
        positions = aligned_pose(pose, odom, config.get('max_gap_s', 0.1))
        ok = np.isfinite(positions).all(axis=1)
        projected, _ = project_reference(positions[ok], rows)
        component = meta.get('config', {}).get('components', {}).get('controller', {})
        params = component.get('params', {}) if component.get('managed', True) else {}
        reference = projected[:, 3]*params.get('speed_scale', 0.5)
        if params.get('max_speed', 0) > 0:
            reference = np.minimum(reference, params['max_speed'])
        valid = projected[:, 4].astype(bool)
        speed = np.array([s.value[3] for s in odom])[ok]
        metrics['wheel_speed_minus_spatial_reference'] = distribution((speed-reference)[valid], 'm/s')
        metrics['spatial_reference_speed'] = distribution(reference[valid], 'm/s')
        if component.get('managed') is False:
            metrics['wheel_speed_minus_spatial_reference'] = metric(units='m/s', reason=
                'Manual metadata-only controller parameters are not verified applied speed settings')
            metrics['spatial_reference_speed'] = metric(units='m/s', reason=
                'Effective scaling/caps are unverified for metadata-only controllers')
    else:
        metrics['wheel_speed_minus_spatial_reference'] = metric(units='m/s', reason='Requires odometry, poses and reference')
    metrics['controller_internal_reference'] = metric(reason='Selected target/heading/speed are not fully published; '
                                                     'offline spatial reference is not the controller internal target')
    metrics['exact_collision_timestamp'] = metric(units='s', reason='Native collision counter has no simulator event stamp')
    if pose and 'map.yaml' in data.assets:
        from planning.centerline import OccupancyMap
        from scipy.ndimage import distance_transform_edt
        grid = OccupancyMap(str(data.assets['map.yaml']))
        distance = distance_transform_edt(np.pad(grid.free, 1, constant_values=False))[1:-1, 1:-1]*grid.res-0.5*grid.res
        pixels = np.rint(grid.to_pixel([s.value[:2] for s in pose])).astype(int)
        inside = (pixels[:, 0] >= 0) & (pixels[:, 0] < grid.h) & (pixels[:, 1] >= 0) & (pixels[:, 1] < grid.w)
        values = distance[pixels[inside, 0], pixels[inside, 1]]
        metrics['map_center_clearance'] = distribution(values, 'm', lower_tail=True)
        metrics['minimum_map_center_clearance'] = metric(float(values.min()) if len(values) else None, 'm',
                                                         'No valid in-map pose samples')
        warnings.append('Map clearance is center-to-nonfree raster distance, not physical body clearance; '
                        f'{int((~inside).sum())} poses outside map are excluded')
    else:
        metrics['map_center_clearance'] = metric(units='m', reason='Requires preserved occupancy geometry and valid poses')
    for name, evidence in EVIDENCE.items():
        units = ('rad' if 'heading' in name else 'm' if 'lateral' in name or 'clearance' in name
                 else 'm/s' if 'speed' in name else '%' if name == 'native_track_completion' else None)
        metrics.setdefault(name, metric(units=units, reason='Required evidence unavailable: '+', '.join(evidence)))
        metrics[name]['required_evidence'] = list(evidence)
    from .race_view import payload
    return {'status': 'ok', 'metrics': metrics, 'warnings': warnings, 'figures': figures,
            'viewer': payload(data, rows, truth_map, config)}


from .race_view import render_report


MODULE = Module('race_performance', '1.0', (), ('pose', 'odom', 'truth', 'lap_count', 'collision_count'), analyze, render_report)
