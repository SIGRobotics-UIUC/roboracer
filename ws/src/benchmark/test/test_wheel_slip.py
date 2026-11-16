"""Slip preparation, conservative alignment, exports, and offline rendering."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import numpy as np
import pytest
from benchmark.analysis.core.signals import Dataset, Sample
from benchmark.analysis.core.wheel_slip import prepare, summaries
from benchmark.analysis.modules.wheel_contact import analyze, MODULE
from benchmark.analysis.modules.wheel_contact_view import render_report


@pytest.fixture
def contact():
    return json.loads((Path(__file__).resolve().parents[4]/'docs/fixtures/wheel_contact_v1.json').read_text())


def c(value, stamp=1_050_000_000, order=2):
    value = deepcopy(value)
    value['stamp_ns'] = stamp
    return Sample(stamp, stamp, value, order=order)


def pose(stamp, order, yaw=0, segment=0, domain='transport'):
    return Sample(stamp, stamp, [float(order), 0., yaw], frame='map', order=order, segment=segment, domain=domain)


def dataset(contact):
    return Dataset(signals={'wheel_contact': [c(contact)], 'pose': [pose(1_000_000_000, 1, 3.1), pose(1_100_000_000, 3, -3.1)]})


def test_signs_peaks_separate_denominators_and_duplicates(contact, tmp_path):
    a = c(contact)
    a.value['wheels'][0]['wheel_hit']['forwardSlip'] = -.4
    b = deepcopy(a); b.order = 3; b.value['stamp_ns'] += 1
    conflict = deepcopy(a); conflict.order = 4; conflict.value['wheels'][0]['wheel_hit']['forwardSlip'] = .8
    d = deepcopy(a); d.order = 5; d.value['sample_sequence'] += 1
    d.value['wheels'][0]['wheel_hit']['forwardSlip'] = .4
    d.value['wheels'][0]['forwardFriction']['extremumSlip'] = .1
    d.value['wheels'][1]['forwardFriction']['extremumSlip'] = 0
    d.value['wheels'][2]['grounded'] = False
    d.value['wheels'][3]['valid'] = False
    data = Dataset(signals={'wheel_contact': [d, b, conflict, a]})
    result = analyze(data, tmp_path, {})
    assert MODULE.version == '1.1'
    assert result['viewer']['duplicates'] == 2
    assert result['viewer']['conflicting_duplicates'] == 1
    s = result['slip_summary']['forward']
    assert s['front_left']['absolute_slip']['mean'] == .4
    assert s['front_left']['ratio']['max'] == 4
    assert s['front_right']['absolute_slip']['count'] == 2
    assert s['front_right']['ratio']['count'] == 1
    assert s['rear_left']['absolute_slip']['count'] == 1
    assert s['rear_right']['absolute_slip']['count'] == 1
    assert len((tmp_path/'contacts.jsonl').read_text().splitlines()) == 4
    assert len((tmp_path/'slip_samples.jsonl').read_text().splitlines()) == 2
    json.dumps(result, allow_nan=False)


def test_interpolation_and_yaw_wrap(contact):
    s = prepare(dataset(contact), {})['samples'][0]
    assert s['alignment']['method'] == 'interpolated'
    assert abs(abs(s['pose'][2])-np.pi) < 1e-8
    assert s['alignment']['bracket_span_s'] == .1
    assert s['xy'] == [2., 0.]


@pytest.mark.parametrize('case', ['stale', 'duplicate', 'domain', 'segment', 'reset', 'independent_segments', 'missing', 'frame'])
def test_rejected_associations(contact, case):
    data = dataset(contact)
    p = data.signals['pose']
    if case == 'stale': p[-1].source_ns += 1
    if case == 'duplicate': p.append(deepcopy(p[0]))
    if case == 'domain': p[-1].domain = 'unverified'
    if case == 'segment': p[-1].segment = 1
    if case == 'reset': data.signals['reset'] = [Sample(None, 1_040_000_000, 'reset', order=2)]
    if case == 'independent_segments': p[0].segment = p[1].segment = 1
    if case == 'missing': data.signals['pose'] = []
    if case == 'frame': p[0].frame = p[1].frame = 'world'
    s = prepare(data, {})['samples'][0]
    assert s['alignment']['method'] == 'unavailable'
    assert s['xy'] is None
    assert s['wheels']['front_left']['forward']['raw'] is not None


def test_exact_laps_and_progress_without_shared(contact, tmp_path):
    raceline = tmp_path/'raceline.csv'
    np.savetxt(raceline, [[0,0,0,-np.pi/2,0,2],[2,2,0,0,0,2],[4,2,2,np.pi/2,0,2],[6,0,2,np.pi,0,2]], delimiter=';')
    data = dataset(contact)
    a = data.signals['wheel_contact'][0]; a.source_ns = 1_000_000_000
    data.signals['lap_count'] = [Sample(None, 1, 0, order=0), Sample(None, 2_000_000_000, 1, order=10)]
    b = deepcopy(a); b.order = 11; b.value['sample_sequence'] += 1
    data.signals['wheel_contact'].append(b)
    data.assets['raceline.csv'] = raceline
    out = prepare(data, {})
    assert out['samples'][0]['alignment']['method'] == 'exact'
    assert out['samples'][0]['d'] == pytest.approx(1)
    assert [x['complete'] for x in out['laps']] == [True, False]
    assert not out['samples'][1]['continuous']
    assert not data.shared


def test_no_spatial_assets_and_safe_html(contact, tmp_path):
    contact['session_id'] = '</script><img src=x onerror=alert(1)>'
    raw = tmp_path/'raw.json'; raw.write_text(json.dumps(contact))
    before = hashlib.sha256(raw.read_bytes()).hexdigest()
    result = analyze(Dataset(signals={'wheel_contact': [c(contact)]}), tmp_path, {})
    from benchmark.analysis.modules.race_view import render_report as race_report
    from benchmark.analysis.modules.wheel_contact_view import extension
    text = race_report({'viewer': {'laps': []}, 'extensions': [extension(result)['viewer']]}) + render_report(result)
    assert text.index('race-track') < text.index('Overall Slip Summary')
    assert 'Entire measured window' in text
    assert '</script><img' not in text
    assert '\\u003c/script' in text
    assert hashlib.sha256(raw.read_bytes()).hexdigest() == before
    assert result['slip_summary']['forward']['front_left']['ratio']['count'] == 1
    assert 'No valid contact' not in text


def test_projection_ambiguity_retains_xy_and_wrap_breaks(contact, tmp_path):
    raceline = tmp_path/'raceline.csv'
    # Crossing reference: center belongs to nonadjacent segments.
    np.savetxt(raceline, [[0,-1,-1,0,0,1],[2,1,1,0,0,1],[4,-1,1,0,0,1],[6,1,-1,0,0,1]], delimiter=';')
    a = c(contact, 1_000_000_000, 1)
    p = pose(a.source_ns, 0); p.value = [0,0,0]
    data = Dataset(signals={'wheel_contact':[a], 'pose':[p]}, assets={'raceline.csv':raceline})
    s = prepare(data, {})['samples'][0]
    assert s['xy'] == [0,0] and s['d'] is None
    np.savetxt(raceline, [[0,0,0,0,0,1],[2,2,0,0,0,1],[4,2,2,0,0,1],[6,0,2,0,0,1]], delimiter=';')
    b = c(contact, a.source_ns+10_000_000, 3); b.value['sample_sequence'] += 1
    p.value = [0,.01,0]
    r = pose(b.source_ns, 2); r.value = [.01,0,0]
    data.signals = {'wheel_contact':[a,b], 'pose':[p,r]}
    v = prepare(data, {})
    assert v['samples'][0]['d'] > 7
    assert v['samples'][1]['d'] < 1
    assert not v['samples'][1]['continuous']


def test_reset_before_lap_boundary_marks_interrupted(contact):
    data = dataset(contact)
    data.signals['reset'] = [Sample(None, 1_200_000_000, 'reset', order=5)]
    data.signals['lap_count'] = [Sample(None, 1, 0, order=0), Sample(None, 2_000_000_000, 1, order=10)]
    assert not prepare(data, {})['laps'][0]['complete']


def test_single_slip_overlay_takes_maximum_valid_axis_and_wheel(contact):
    from benchmark.analysis.modules.wheel_contact_view import extension
    data = Dataset(signals={'wheel_contact':[c(contact)]})
    prepared = prepare(data, {})
    sample = prepared['samples'][0]
    for wheel in sample['wheels'].values():
        for direction in ('forward', 'sideways'):
            wheel[direction]['ratio'] = None
    sample['wheels']['front_left']['forward']['ratio'] = .25
    sample['wheels']['rear_right']['sideways']['ratio'] = 2.5
    view = extension({'viewer':prepared})['viewer']
    assert len(view['overlays']) == 1
    assert view['overlays'][0]['title'] == 'Slip'
    assert view['overlays'][0]['values'] == [2.5]
    assert 'RR sideways' in view['samples'][0]['details'][0][1]
    sample['wheels']['rear_right']['sideways']['ratio'] = None
    sample['wheels']['front_left']['forward']['ratio'] = None
    assert extension({'viewer':prepared})['viewer']['overlays'][0]['values'] == [None]
