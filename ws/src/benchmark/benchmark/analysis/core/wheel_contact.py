"""Decode evaluation-only contact envelopes without interpreting force as load."""
import json
import math

WHEEL_NAMES = ('front_left', 'front_right', 'rear_left', 'rear_right')


def finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def decode_contact(raw):
    if not isinstance(raw, str) or len(raw.encode('utf-8')) > 262144:
        raise ValueError('contact payload exceeds schema bounds')
    def invalid(value):
        raise ValueError('nonfinite JSON number')
    value = json.loads(raw, parse_constant=invalid)
    def check(item, depth=0):
        if depth > 32 or isinstance(item, float) and not math.isfinite(item):
            raise ValueError('invalid JSON number/nesting')
        if isinstance(item, dict):
            for child in item.values():
                check(child, depth+1)
        elif isinstance(item, list):
            for child in item:
                check(child, depth+1)
    check(value)
    if (not isinstance(value, dict) or type(value.get('schema_version')) is not int
            or value['schema_version'] != 1 or value.get('evaluation_only') is not True):
        raise ValueError('unsupported contact schema/provenance')
    if not isinstance(value.get('session_id'), str) or not value['session_id']:
        raise ValueError('missing contact session')
    for key in ('sample_sequence', 'reset_generation', 'configuration_revision', 'stamp_ns'):
        if type(value.get(key)) is not int or value[key] < (1 if key == 'stamp_ns' else 0):
            raise ValueError('invalid '+key)
    if not isinstance(value.get('source'), dict) or not isinstance(value.get('runtime_snapshot'), dict):
        raise ValueError('missing source/runtime evidence')
    wheels = value.get('wheels')
    if (not isinstance(wheels, list) or len(wheels) != 4
            or any(not isinstance(w, dict) for w in wheels)
            or [w.get('name') for w in wheels] != list(WHEEL_NAMES)):
        raise ValueError('invalid wheel identities')
    def tagged(item):
        if isinstance(item, dict):
            return 'nonfinite' in item or any(tagged(v) for v in item.values())
        return isinstance(item, list) and any(tagged(v) for v in item)
    for wheel in wheels:
        if wheel.get('valid') is True and tagged(wheel):
            raise ValueError('tagged nonfinite wheel must be invalid')
        if type(wheel.get('grounded')) is not bool or type(wheel.get('valid')) is not bool:
            raise ValueError('invalid grounded/valid flag')
        if not wheel['grounded'] and wheel.get('wheel_hit') is not None:
            raise ValueError('ungrounded hit must be null')
        if wheel['grounded'] and wheel['valid'] and not isinstance(wheel.get('wheel_hit'), dict):
            raise ValueError('missing valid grounded hit')
    return value


def curve_valid(curve):
    keys = ('extremumSlip', 'extremumValue', 'asymptoteSlip', 'asymptoteValue', 'stiffness')
    return (isinstance(curve, dict) and all(finite(curve.get(k)) for k in keys)
            and 0 < curve['extremumSlip'] < curve['asymptoteSlip']
            and curve['extremumValue'] >= 0 and curve['asymptoteValue'] >= 0 and curve['stiffness'] > 0)
