import json

from autodrive_bridge.frame_trace import FrameTrace, compact_payload


def test_trace_preserves_timing_and_omits_large_payloads(tmp_path):
    path = tmp_path/'frames.jsonl'
    writer = FrameTrace(str(path))
    payload = {'Simulation Time': '1.5', 'V1 Position': '0 0 0',
               'V1 Front Camera Image': 'large', 'V1 LIDAR Range Array': 'large'}
    writer.add(dict(arrival_monotonic_ns=123, payload=compact_payload(payload)))
    assert writer.close() == dict(dropped=0, error=None, writer_alive=False)
    row = json.loads(path.read_text())
    assert row['arrival_monotonic_ns'] == 123
    assert row['payload'] == {'Simulation Time': '1.5', 'V1 Position': '0 0 0'}
