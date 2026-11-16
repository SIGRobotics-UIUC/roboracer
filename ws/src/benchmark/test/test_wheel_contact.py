"""Evaluation-only contact diagnostics, source identity and old-bag availability."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace
import pytest

from benchmark.analysis.core.bags import decode, validate_message
from benchmark.analysis.core.signals import Dataset, Sample, SIGNAL_MAP
from benchmark.analysis.core.wheel_contact import decode_contact
from benchmark.analysis.modules.wheel_contact import analyze, MODULE
from benchmark.analysis.core.results import run_modules
from benchmark.collector.topics import DETAILED, NS


@pytest.fixture
def payload():
    value = json.loads((Path(__file__).resolve().parents[4]/'docs/fixtures/wheel_contact_v1.json').read_text())
    value['stamp_ns'] = 1_000_000_000
    return value


def sample(value, order=0, segment=0):
    return Sample(value['stamp_ns'], value['stamp_ns']+100, deepcopy(value), order=order, segment=segment)


def test_contact_map_and_decoder_preserve_raw_tags(payload):
    assert NS+'wheel_contact' in DETAILED
    assert SIGNAL_MAP['wheel_contact']['evaluation_only']
    payload['wheels'][0]['wheel_hit']['force'] = {'nonfinite': 'NaN'}
    payload['wheels'][0]['valid'] = False
    message = SimpleNamespace(data=json.dumps(payload))
    result, valid, status, reasons = validate_message('wheel_contact', message, payload['stamp_ns'])
    assert result == payload and valid and status == 'available' and not reasons
    assert decode('wheel_contact', SimpleNamespace(data='{"x": NaN}'))[1] is False
    assert validate_message('wheel_contact', SimpleNamespace(data='null'), None)[3] == ['payload']


@pytest.mark.parametrize('stamp', [None, True, -1, 0, 1.5])
def test_contact_bad_timestamp_rejected(payload, stamp):
    payload['stamp_ns'] = stamp
    with pytest.raises(ValueError):
        decode_contact(json.dumps(payload))


def test_raw_force_proxy_dedup_and_configuration_evidence(payload, tmp_path):
    a = sample(payload)
    duplicate = sample(payload, order=1)
    payload['sample_sequence'] = 3
    payload['configuration_revision'] = 2
    payload['stamp_ns'] += 10_000_000
    payload['source']['skipped_fixed_callbacks'] = 2
    payload['wheels'][0]['wheel_hit']['force'] = -1.
    payload['wheels'][0]['forwardFriction']['extremumSlip'] = .3
    payload['wheels'][0]['forwardFriction']['asymptoteSlip'] = .5
    b = sample(payload, order=2)
    result = analyze(Dataset(signals={'wheel_contact': [a, duplicate, b]}), tmp_path, {})
    m = result['metrics']
    assert m['unique_source_samples']['value'] == 2
    assert m['repeated_source_samples']['value'] == 1
    assert m['source_sequence_gaps']['value'] == 1
    assert m['skipped_fixed_callbacks']['value'] == 2
    assert m['runtime_configuration_revisions']['value'] == 2
    assert m['front_left_force']['value']['mean'] == pytest.approx(4.25)
    assert m['front_left_forward_slip_to_configured_peak']['value']['mean'] == pytest.approx(.375)
    assert m['front_left_sideways_slip_to_configured_peak']['value']['mean'] == 2
    assert len((tmp_path/'contacts.jsonl').read_text().splitlines()) == 3
    assert not any('utilization' in key or 'saturation' in key for key in m)
    json.dumps(result, allow_nan=False)


def test_invalid_ungrounded_bad_curve_and_measurement_window(payload, tmp_path):
    payload['wheels'][0].update(grounded=False, wheel_hit=None)
    payload['wheels'][1]['valid'] = False
    payload['wheels'][1]['wheel_hit']['force'] = {'nonfinite': '+Infinity'}
    payload['wheels'][2]['forwardFriction']['asymptoteSlip'] = .01
    data = Dataset(signals={'wheel_contact': [sample(payload)]})
    m = analyze(data, tmp_path, {})['metrics']
    assert m['ungrounded_wheel_samples']['value'] == 1
    assert m['invalid_grounded_wheel_samples']['value'] == 1
    assert m['front_left_force']['value'] is None
    assert m['front_right_force']['value'] is None
    assert m['rear_left_forward_slip_to_configured_peak']['value'] is None
    assert m['four_grounded_raw_force_sum']['value'] is None
    data.metadata['measurement'] = {'started': False}
    assert analyze(data, tmp_path, {})['metrics']['contact_samples']['value'] is None


def test_absent_old_bag_reports_unavailable_and_module_contract(tmp_path, monkeypatch):
    from benchmark.analysis.core import results
    monkeypatch.setattr(results, 'REGISTRY', {'wheel_contact': MODULE})
    result = run_modules(Dataset(), ['wheel_contact'], tmp_path, {})[0]
    assert result['status'] == 'partial'
    assert result['metrics']['contact_samples']['value'] is None


def test_source_restart_and_reset_never_bridge_sequence_gaps(payload, tmp_path):
    a = sample(payload)
    payload['sample_sequence'] = 10
    payload['reset_generation'] += 1
    b = sample(payload, order=1)
    payload['session_id'] = 'restarted-plugin'
    payload['sample_sequence'] = 1
    c = sample(payload, order=2)
    m = analyze(Dataset(signals={'wheel_contact': [a, b, c]}), tmp_path, {})['metrics']
    assert m['unique_source_samples']['value'] == 3
    assert m['source_sequence_gaps']['value'] == 0
    assert m['source_sequence_regressions']['value'] == 0
    assert m['source_sessions']['value'] == 2


def test_native_bag_contact_arrival_stamp_and_invalid_payload(payload, tmp_path):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String
    from benchmark.collector.artifacts import finalize
    from benchmark.analysis.core.bags import load_run
    import hashlib
    run = tmp_path/'run'; run.mkdir()
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(run/'bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    topic = NS+'wheel_contact'
    writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type='std_msgs/msg/String', serialization_format='cdr'))
    with (run/'capture_index.jsonl').open('w') as index:
        for i, text in enumerate((json.dumps(payload), 'null')):
            raw = serialize_message(String(data=text))
            stamp = 2_000_000_000+i
            writer.write(topic, raw, stamp)
            index.write(json.dumps(dict(sequence=i, topic=topic, recorded_ns=stamp,
                                        sha256=hashlib.sha256(raw).hexdigest()))+'\n')
    del writer
    finalize(run, {'clock_domain': 'transport', 'config': {'logging': {'level': 'detailed'}}})
    data = load_run(run)
    samples = data.signals['wheel_contact']
    assert samples[0].source_ns == payload['stamp_ns']
    assert samples[0].recorded_ns == 2_000_000_000
    assert samples[0].value['source']['observed_fixed_time'] == .02
    assert samples[0].valid and not samples[1].valid
    assert samples[1].rejection_reasons == ('payload',)


def test_tagged_nonfinite_cannot_be_marked_valid(payload):
    payload['wheels'][0]['wheel_hit']['forwardSlip'] = {'nonfinite': 'NaN'}
    with pytest.raises(ValueError, match='must be invalid'):
        decode_contact(json.dumps(payload))
