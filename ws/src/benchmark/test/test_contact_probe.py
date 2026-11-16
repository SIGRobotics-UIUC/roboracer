import json
from pathlib import Path
import pytest
from benchmark.analysis.contact_probe import contact_record, main


def test_contact_capture_retains_valid_tags_and_raw_invalid_payload():
    value = json.loads((Path(__file__).resolve().parents[4]/'docs/fixtures/wheel_contact_v1.json').read_text())
    value['stamp_ns'] = 1
    value['wheels'][0]['valid'] = False
    value['wheels'][0]['wheel_hit']['force'] = {'nonfinite': 'NaN'}
    raw = json.dumps(value)
    result = contact_record(raw, 123)
    assert result['contact'] == value and result['raw'] == raw
    assert result['received_monotonic_ns'] == 123 and result['valid_envelope']
    invalid = contact_record('broken', 124)
    assert not invalid['valid_envelope'] and invalid['raw'] == 'broken'


@pytest.mark.parametrize('seconds', ['nan', 'inf', '0', '-1'])
def test_capture_rejects_bad_duration_before_ros_or_outputs(seconds):
    with pytest.raises(SystemExit):
        main(['--seconds', seconds])
