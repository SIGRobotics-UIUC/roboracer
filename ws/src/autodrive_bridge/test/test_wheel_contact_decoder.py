"""Contract tests use synthetic JSON, never load a simulator plugin DLL."""
from copy import deepcopy
import json
from pathlib import Path
import pytest

from autodrive_bridge.wheel_contact import decode_contact


@pytest.fixture
def payload():
    return json.loads((Path(__file__).resolve().parents[4]/'docs/fixtures/wheel_contact_v1.json').read_text())


def test_contact_roundtrip_preserves_raw_negative_force_tags_and_ungrounded(payload):
    payload['wheels'][0]['wheel_hit']['force'] = -2.
    payload['wheels'][1].update(grounded=False, wheel_hit=None)
    payload['wheels'][2]['wheel_hit']['force'] = {'nonfinite': '+Infinity'}
    payload['wheels'][2].update(valid=False, sampling_error='nonfinite API value')
    assert decode_contact(json.dumps(payload)) == payload


@pytest.mark.parametrize('raw', ['null', '[]', '{}', 'NaN', '"text"', '{"x":1e999}', '['*40+']'*40, 'x'*262145])
def test_contact_rejects_bad_envelopes(raw):
    with pytest.raises((ValueError, RecursionError)):
        decode_contact(raw)


@pytest.mark.parametrize('change', ['schema', 'bool_sequence', 'order', 'ungrounded_hit', 'grounded_hit', 'bool_flag'])
def test_contact_rejects_bad_contract(payload, change):
    if change == 'schema': payload['schema_version'] = True
    if change == 'bool_sequence': payload['sample_sequence'] = True
    if change == 'order': payload['wheels'].reverse()
    if change == 'ungrounded_hit': payload['wheels'][0]['grounded'] = False
    if change == 'grounded_hit': payload['wheels'][0]['wheel_hit'] = None
    if change == 'bool_flag': payload['wheels'][0]['valid'] = 1
    with pytest.raises(ValueError):
        decode_contact(json.dumps(payload))


def test_tagged_nonfinite_cannot_be_marked_valid(payload):
    payload['wheels'][0]['wheel_hit']['forwardSlip'] = {'nonfinite': 'NaN'}
    with pytest.raises(ValueError, match='must be invalid'):
        decode_contact(json.dumps(payload))


def test_base64_wire_payload_preserves_quotes_backslashes_and_unicode(payload):
    import base64
    payload['wheels'][0]['wheel_hit']['collider']['name'] = 'ground "quoted" \\ path\n雪'
    raw = json.dumps(payload, ensure_ascii=False)
    wire = 'base64:'+base64.b64encode(raw.encode()).decode('ascii')
    # Even a Unity-style wrapper with no string escaping can safely carry this.
    outer = '["Bridge",{"V1 Wheel Contact":"'+wire+'"}]'
    assert decode_contact(json.loads(outer)[1]['V1 Wheel Contact']) == payload


@pytest.mark.parametrize('wire', ['base64:@@', 'base64:/w==', 'base64:a', 'base64:'+'A'*349532])
def test_bad_base64_wire_rejected(wire):
    with pytest.raises(ValueError):
        decode_contact(wire)
