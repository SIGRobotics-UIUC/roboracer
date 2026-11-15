from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import pytest

from benchmark.analysis.core.signals import (Dataset, Sample, aligned_pose, project_reference,
                                             segment_and_sort, transform_world, measurement_samples)
from benchmark.analysis.core.results import Module, REGISTRY, metric, run_modules
from benchmark.analysis.modules.race_performance import analyze as race
from benchmark.analysis.modules.computation_timing import analyze as timing, V2
from benchmark.reports.render import render


def sample(t, value, **kwargs):
    return Sample(round(t*1e9), round(t*1e9)+1_000_000, value, **kwargs)


def test_order_duplicate_delayed_delivery_and_reset():
    raw = [sample(t, [0, 0, 0], order=i) for i, t in enumerate([1., 1.05, 1.03, 1.05, .2, .25])]
    for i, item in enumerate(raw):
        item.recorded_ns = 2_000_000_000+i*10_000_000
    sorted_values, warnings = segment_and_sort(raw)
    assert raw[2].source_ns == 1_030_000_000
    assert [s.source_ns for s in sorted_values[:4]] == [1_000_000_000, 1_030_000_000, 1_050_000_000, 1_050_000_000]
    assert sorted_values[-1].segment == 1
    assert 'duplicate source timestamp' in warnings
    assert 'out-of-order delivery' in warnings
    reset, _ = segment_and_sort(raw[:2], reset_records=[raw[1].recorded_ns])
    assert reset[0].segment != reset[1].segment


def test_alignment_wraps_angles_and_excludes_gaps_resets_and_domains():
    source = [sample(1., [0., 0., np.deg2rad(179)]), sample(1.1, [2., 0., np.deg2rad(-179)])]
    target = sample(1.05, [0, 0, 0])
    value = aligned_pose(source, [target], .11)[0]
    assert value[0] == pytest.approx(1.)
    assert abs(value[2]) == pytest.approx(np.pi)
    assert np.isnan(aligned_pose(source, [target], .05)).all()
    target.segment = 1
    assert np.isnan(aligned_pose(source, [target], .11)).all()
    target.segment = 0; target.domain = 'windows'
    assert np.isnan(aligned_pose(source, [target], .11)).all()
    duplicate = source+[source[0]]
    duplicate.sort(key=lambda s: s.source_ns)
    # Duplicates cannot silently choose an arbitrary endpoint.
    assert np.isnan(aligned_pose(duplicate, [source[0]], .11)).all()


def test_frozen_ros_time_does_not_mix_stopping_into_measurement():
    samples = [Sample(1, 10, [0, 0, 0], received_monotonic_ns=t) for t in (100, 200, 300)]
    data = Dataset(signals={'pose': samples}, metadata={
        'receipt_monotonic_domain': 'collector_container_monotonic',
        'measurement': {'started': True, 'start_monotonic_ns': 150, 'end_monotonic_ns': 250,
                        'start_recorded_ns': 10, 'end_recorded_ns': 10}})
    assert measurement_samples(data, 'pose') == [samples[1]]
    data.metadata['measurement']['started'] = False
    assert measurement_samples(data, 'pose') == []


def reference(tmp_path):
    rows = np.array([[0, 0, 0, -np.pi/2, 0, 2, 0], [10, 10, 0, 0, 0, 2, 0],
                     [20, 10, 10, np.pi/2, 0, 2, 0], [30, 0, 10, -np.pi, 0, 2, 0]])
    np.savetxt(tmp_path/'raceline.csv', rows, delimiter=';')
    return rows


def test_known_projection_sign_tum_heading_and_frame_transform(tmp_path):
    rows = reference(tmp_path)
    # Constant heading on this segment isolates TUM-to-ROS conversion.
    rows[1, 3] = -np.pi/2
    projected, length = project_reference([[2., .3, .1]], rows)
    assert length == 40
    assert projected[0, :4] == pytest.approx([2., .3, .1, 2.])
    world = transform_world([[10., 21., np.pi/2]], {'x_m': 10., 'y_m': 20., 'yaw_rad': np.pi/2})
    assert world[0] == pytest.approx([1., 0., 0.], abs=1e-10)


def test_known_race_metrics_and_missing_inputs(tmp_path):
    rows = reference(tmp_path)
    rows[1, 3] = -np.pi/2
    np.savetxt(tmp_path/'raceline.csv', rows, delimiter=';')
    (tmp_path/'map_to_world.yaml').write_text('x_m: 0\ny_m: 0\nyaw_rad: 0\n')
    data = Dataset(signals={
        'pose': [sample(1., [1., .3, .1]), sample(1.1, [1.2, .3, .1])],
        'truth': [sample(1., [1., 0., 0., 1., 0., 0.]), sample(1.1, [1.2, 0., 0., 1., 0., 0.])],
        'odom': [sample(1., [1., 0., 0., 1., 0., 0.]), sample(1.1, [1.2, 0., 0., 1., 0., 0.])],
    }, assets={'raceline.csv': tmp_path/'raceline.csv', 'map_to_world.yaml': tmp_path/'map_to_world.yaml'},
        metadata={'config': {'experiment': {'laps': 2}, 'components': {'controller': {'params': {'speed_scale': .5}}}},
                  'measurement': {'completed_laps': 1, 'laps': [{'time_s': 12., 'valid': True}]}})
    metrics = race(data, tmp_path, {'max_gap_s': .11})['metrics']
    assert metrics['localization_position_error']['value']['mean'] == pytest.approx(.3)
    assert metrics['localization_heading_error']['value']['mean'] == pytest.approx(.1)
    assert metrics['estimated_lateral_error']['value']['mean'] == pytest.approx(.3)
    assert metrics['wheel_speed_minus_spatial_reference']['value']['mean'] == pytest.approx(0.)
    assert metrics['lap_completion_fraction']['value'] == .5
    assert metrics['lap_time']['value']['mean'] == 12.
    assert metrics['map_center_clearance']['value'] is None
    missing = race(Dataset(), tmp_path, {})['metrics']
    assert missing['localization_position_error']['value'] is None
    assert missing['lap_time']['value'] is None


def test_timing_has_known_intervals_and_explicit_v2_requirements(tmp_path):
    data = Dataset(signals={'requested_drive': [sample(t, [1, 0]) for t in (1., 1.025, 1.05)]},
                   metadata={'clock_domain': 'transport', 'recorder_clock_domain': 'transport',
                             'config': {'components': {'controller': {'type': 'map'}}}})
    metrics = timing(data, tmp_path, {})['metrics']
    assert metrics['command_publication_interval']['value']['mean'] == pytest.approx(.025)
    assert metrics['command_publication_frequency']['value'] == pytest.approx(40.)
    assert metrics['command_publication_jitter']['value']['max'] == pytest.approx(0.)
    assert metrics['requested_drive_source_stamp_age_at_recorder']['value']['mean'] == pytest.approx(.001)
    for name, requirement in V2.items():
        assert metrics[name]['value'] is None
        assert metrics[name]['reason'] and metrics[name]['v2_requirement'] == requirement


def test_module_isolation_skips_and_generic_custom_figures(tmp_path, monkeypatch):
    from benchmark.analysis.core import results
    def broken(*args):
        raise RuntimeError('deliberate failure')
    def good(*args):
        (args[1]/'custom.png').write_bytes(b'fixture')
        return {'status': 'ok', 'metrics': {'example': metric(2., 'm')},
                'figures': [{'path': 'custom.png', 'caption': '<custom>'}]}
    def invalid_json(*args):
        return {'status': 'ok', 'metrics': {'invalid': metric(float('nan'), 'm')}}
    monkeypatch.setattr(results, 'REGISTRY', {m.name: m for m in (
        Module('broken', '1', (), (), broken), Module('good', '1', (), (), good),
        Module('missing', '1', ('unrecorded',), (), good), Module('invalid_json', '1', (), (), invalid_json))})
    modules = run_modules(Dataset(), ['broken', 'good', 'missing', 'invalid_json'], tmp_path, {})
    assert [m['status'] for m in modules] == ['failed', 'ok', 'skipped', 'failed']
    render(tmp_path, {'modules': modules, 'metadata': {'text': '<unsafe>'}})
    document = (tmp_path/'report.html').read_text()
    assert 'good/custom.png' in document and '&lt;custom&gt;' in document and '&lt;unsafe&gt;' in document
    json.dumps(modules, allow_nan=False)
