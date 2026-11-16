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


def test_core_report_without_modules(tmp_path):
    render(tmp_path, {'run_id': '<run>', 'analysis_id': 'analysis', 'modules': [],
                     'experiment': {'status': 'aborted', 'failure': 'recorder_failure',
                         'git': {'commit': 'abc123'},
                         'config': {'experiment': {'map': 'Porto', 'laps': 5},
                                    'components': {'online_planner': {'enabled': False}}},
                         'measurement': {'completed_laps': 2}}})
    document = (tmp_path/'report.html').read_text()
    for expected in ('Benchmark Report', 'Experiment Config', 'Collected Data',
                     '&lt;run&gt;', 'abc123', 'recorder_failure', 'Requested laps',
                     'Completed laps', 'Online planner</dt><dd>None', '/scan',
                     'benchmarkReport', 'requestSelection'):
        assert expected in document
    assert 'Race Performance' not in document


def test_module_report_entry_point(tmp_path, monkeypatch):
    from benchmark.analysis.core import results
    module = Module('custom', '1', (), (), lambda *args: {},
                    render_report=lambda result: '<section id="custom">Owned view</section>')
    monkeypatch.setattr(results, 'REGISTRY', {'custom': module})
    render(tmp_path, {'modules': [{'name': 'custom', 'status': 'ok'}]})
    assert 'Owned view' in (tmp_path/'report.html').read_text()


def test_race_view_payload_alignment_partial_lap_and_strict_json(tmp_path):
    from benchmark.analysis.modules.race_view import payload, render_report
    rows = np.array([[0,0,0,-np.pi/2,0,2], [2,2,0,0,0,2],
                     [4,2,2,np.pi/2,0,2], [6,0,2,np.pi,0,2]])
    pose = [sample(t, [x,0,0], frame='map', order=i) for i,(t,x) in
            enumerate([(1,.1),(1.05,.2),(1.1,.3),(2,.4)])]
    data = Dataset(signals={'pose':pose, 'odom':[sample(1., [0,0,0,1,0,0])]}, metadata={
        'config':{'components':{'controller':{'managed':False}}}})
    view = payload(data, rows, [], {'max_gap_s':.1})
    assert len(view['laps']) == 1 and not view['laps'][0]['complete']
    assert view['signals']['Wheel odometry speed|m/s|Motion and commands|Speed'][0] == 1
    assert view['signals']['Wheel odometry speed|m/s|Motion and commands|Speed'][1] is None
    assert not view['samples'][-1]['continuous']
    assert not any('Effective spatial' in k for k in view['signals'])
    json.dumps(view, allow_nan=False)
    document = render_report({'viewer':view, 'metrics':{}})
    assert 'Plotly.newPlot' in document and 'plotly_click' in document
    assert 'requestSelection' in document and 'drawMap' in document


def test_error_magnitude_percentile_and_clearance_lower_tail():
    from benchmark.analysis.core.results import distribution
    lateral = distribution([-10, -.2, -.1, .1], 'm', magnitude_percentile=True)['value']
    assert lateral['p95_absolute'] == pytest.approx(np.percentile([10,.2,.1,.1],95))
    assert lateral['mean'] < 0 and 'p95' not in lateral
    clearance = distribution([.01,.1,1,2], 'm', lower_tail=True)['value']
    assert clearance['min'] == .01 and clearance['p5'] < .1
    assert 'max' not in clearance and 'p95' not in clearance


def test_collected_data_does_not_flag_logging_exclusions(tmp_path):
    from benchmark.collector.topics import BASE, EXTRA
    render(tmp_path, {'modules': [], 'experiment': {
        'config': {'logging': {'level':'basic'}, 'components': {'controller': {'params': {'speed_scale': .5}}}},
        'git': {'branch': 'benchmark-v1', 'commit': 'abcdef123456789'},
        'recorder': {'closed':True, 'counts':dict.fromkeys([*BASE, *EXTRA], 10)},
        'recording_duration_s':10}})
    doc=(tmp_path/'report.html').read_text()
    assert 'Recording available' in doc
    assert 'Missing required inputs' not in doc
    assert 'Not collected at this logging level' in doc
    assert 'benchmark-v1' in doc and 'abcdef1</code>' in doc
    assert 'data-commit="abcdef123456789"' in doc
    assert 'speed scale' in doc
    assert '<details><summary>Full recording inventory' in doc
    assert '<details><summary>Provenance and saved evidence' in doc


def test_occupancy_map_payload_is_raster_with_origin(tmp_path):
    from PIL import Image
    from benchmark.analysis.modules.race_view import payload
    Image.new('L', (3, 2), 255).save(tmp_path/'map.png')
    (tmp_path/'map.yaml').write_text('image: map.png\nresolution: 0.1\norigin: [1, 2, 0.5]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    rows=np.array([[0,0,0,-np.pi/2,0,2],[2,2,0,0,0,2],
                   [4,2,2,np.pi/2,0,2],[6,0,2,np.pi,0,2]])
    view=payload(Dataset(signals={'pose':[sample(1,[1,2,0])]},
                         assets={'map.yaml':tmp_path/'map.yaml'}),rows,[],{})
    assert view['map']['image'].startswith('data:image/png;base64,')
    assert view['map']['origin'] == [1,2,.5]
    assert view['map']['width'] == 3 and view['map']['height'] == 2


@pytest.mark.parametrize('outcome,label,severity', [
    ('completed', 'Completed — requested lap count reached', None),
    ('stopped', 'Interrupted — operator stopped collection', 'warning'),
    ('aborted', 'Failed — collection or finalization aborted', 'error'),
    ('recovered_incomplete', 'Incomplete — recovered artifacts', 'warning'),
])
def test_collection_outcome_independent_of_closed_recorder(tmp_path, outcome, label, severity):
    from benchmark.collector.topics import BASE, EXTRA
    render(tmp_path, {'modules': [], 'experiment': {
        'status': outcome, 'config': {'logging': {'level': 'basic'}},
        'recorder': {'closed': True, 'counts': dict.fromkeys([*BASE, *EXTRA], 10)}}})
    doc = (tmp_path/'report.html').read_text()
    assert label in doc
    assert 'Recorder finished; bag metadata written' in doc
    if severity:
        assert '<details class="collection-notice '+severity+'">' in doc
        assert doc.index('Collection outcome') < doc.index('<details class="collection-notice')
    else:
        assert '<details class="collection-notice' not in doc


def test_invalid_samples_are_collapsed_and_expected_sentinel_is_not_warning(tmp_path):
    from benchmark.collector.topics import BASE, EXTRA
    summary = {'modules': [], 'experiment': {
        'status': 'completed', 'config': {'logging': {'level': 'basic'}},
        'recorder': {'closed': True, 'counts': dict.fromkeys([*BASE, *EXTRA], 10)}},
        'signal_validation': {'last_lap_time': {'uninitialized': 10, 'invalid': 0}}}
    render(tmp_path, summary)
    assert '<details class="collection-notice' not in (tmp_path/'report.html').read_text()
    warning = 'last_lap_time: 2/10 samples invalid; rejection counts: numeric'
    summary['warnings'] = [warning]
    render(tmp_path, summary)
    doc = (tmp_path/'report.html').read_text()
    notice = doc.split('<details class="collection-notice warning">')[1].split('</details>')[0]
    assert '⚠ Warning: Collection evidence needs attention' in notice
    assert warning in notice and ' open' not in notice


def test_recorder_error_is_error_even_when_closed(tmp_path):
    render(tmp_path, {'modules': [], 'experiment': {
        'config': {'logging': {'level': 'off'}},
        'recorder': {'closed': True, 'error': '<write failure>'}}})
    doc = (tmp_path/'report.html').read_text()
    assert '<details class="collection-notice error">' in doc
    assert '&lt;write failure&gt;' in doc


def test_reference_progress_origin_and_wrap_are_geometric():
    rows = np.array([[0,0,0,-np.pi/2,0,2], [2,2,0,0,0,2],
                     [4,2,2,np.pi/2,0,2], [6,0,2,np.pi,0,2]])
    projected, length = project_reference([[0,0,0], [0,.01,0], [.01,0,0]], rows)
    assert length == 8
    assert projected[:,0] == pytest.approx([0,7.99,.01])
    assert (projected[:,0] < length).all()


def test_true_absolute_tracking_errors_match_signed_values():
    from benchmark.analysis.modules.race_view import payload
    rows = np.array([[0,0,0,-np.pi/2,0,2], [2,2,0,0,0,2],
                     [4,2,2,np.pi/2,0,2], [6,0,2,np.pi,0,2]])
    pose = [sample(1., [.5,-.1,-.2])]
    truth = [sample(1., [.5,-.2,-.3,0,0,0])]
    view = payload(Dataset(signals={'pose':pose}), rows, truth, {})
    signals = view['signals']
    for component,unit in [('lateral','m'),('heading','rad')]:
        signed = signals[f'True {component}|{unit}|Tracking|Signed {component} error'][0]
        absolute = signals[f'True absolute {component}|{unit}|Tracking|Absolute {component} error'][0]
        assert absolute == abs(signed)
        assert absolute > 0


def test_viewer_uses_exact_recorded_values_while_analysis_can_interpolate():
    from benchmark.analysis.modules.race_view import payload
    rows = np.array([[0,0,0,-np.pi/2,0,2], [2,2,0,0,0,2],
                     [4,2,2,np.pi/2,0,2], [6,0,2,np.pi,0,2]])
    poses = [sample(1., [.4,.1,3.1]), sample(1.05, [.5,.1,-3.1])]
    odom = [sample(.98, [0,0,0,1,0,0]), sample(1.02, [0,0,0,3,0,0]),
            sample(1.05, [0,0,0,4,0,0])]
    truth = [sample(.98, [.3,0,0,1,0,0]), sample(1.02, [.5,0,0,3,0,0])]
    view = payload(Dataset(signals={'pose': poses, 'odom': odom}), rows, truth, {})
    speeds = view['signals']['Wheel odometry speed|m/s|Motion and commands|Speed']
    assert speeds == [None, 4.]
    assert view['samples'][0]['truth'] == [None, None]
    assert view['signals']['Localized pose heading|rad|Motion and commands|Heading'] == [3.1, -3.1]
    assert aligned_pose(truth, poses)[0, 0] == pytest.approx(.4)
    assert np.isnan(aligned_pose(truth, poses, interpolate=False)).all()
    assert view['value_alignment'] == 'exact_recorded_timestamp'


def test_exact_alignment_still_rejects_duplicates_and_other_segments():
    source = [sample(1., [1,2,3]), sample(1., [4,5,6])]
    assert np.isnan(aligned_pose(source, [sample(1., [0,0,0])], interpolate=False)).all()
    target = sample(1., [0,0,0], segment=1)
    assert np.isnan(aligned_pose(source[:1], [target], interpolate=False)).all()
