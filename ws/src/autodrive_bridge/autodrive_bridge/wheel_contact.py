"""Bounded, additive evaluation telemetry; never supplies driving inputs."""
import base64
import binascii
import json
import math

WHEEL_NAMES = ('front_left', 'front_right', 'rear_left', 'rear_right')
MAX_PAYLOAD_BYTES = 262144


def decode_contact(raw):
    """Preserve raw API fields/tags; reject malformed envelopes and JSON numbers."""
    if isinstance(raw, str) and raw.startswith('base64:'):
        if len(raw) > 7+4*((MAX_PAYLOAD_BYTES+2)//3):
            raise ValueError('encoded contact payload exceeds 256 KiB decoded limit')
        try:
            raw = base64.b64decode(raw[7:], validate=True).decode('utf-8')
        except (binascii.Error, UnicodeError) as exc:
            raise ValueError('invalid contact base64/UTF-8') from exc
    if not isinstance(raw, str) or len(raw.encode('utf-8')) > MAX_PAYLOAD_BYTES:

        raise ValueError('contact payload must be a JSON string of at most 256 KiB')
    value = json.loads(raw, parse_constant=lambda value: (_ for _ in ()).throw(
        ValueError('nonfinite JSON number: '+value)))
    # Includes overflow literals such as 1e999, which parse_constant does not see.
    def check(item, depth=0):
        if depth > 32:
            raise ValueError('contact payload nesting exceeds 32')
        if isinstance(item, float) and not math.isfinite(item):
            raise ValueError('nonfinite JSON number')
        if isinstance(item, dict):
            for child in item.values():
                check(child, depth+1)
        elif isinstance(item, list):
            for child in item:
                check(child, depth+1)
    check(value)
    if not isinstance(value, dict) or type(value.get('schema_version')) is not int or value['schema_version'] != 1:
        raise ValueError('unsupported contact schema')
    if value.get('evaluation_only') is not True or not isinstance(value.get('session_id'), str) or not value['session_id']:
        raise ValueError('missing evaluation provenance/session')
    for key in ('sample_sequence', 'reset_generation', 'configuration_revision'):
        if type(value.get(key)) is not int or value[key] < 0:
            raise ValueError('invalid '+key)
    if not isinstance(value.get('source'), dict) or not isinstance(value.get('runtime_snapshot'), dict):
        raise ValueError('missing source evidence/runtime snapshot')
    wheels = value.get('wheels')
    if not isinstance(wheels, list) or len(wheels) != 4:
        raise ValueError('expected four contact wheels')
    if any(not isinstance(wheel, dict) for wheel in wheels) or [w.get('name') for w in wheels] != list(WHEEL_NAMES):
        raise ValueError('invalid wheel identity/order')
    def tagged(item):
        if isinstance(item, dict):
            return 'nonfinite' in item or any(tagged(v) for v in item.values())
        return isinstance(item, list) and any(tagged(v) for v in item)
    for wheel in wheels:
        if wheel.get('valid') is True and tagged(wheel):
            raise ValueError('tagged nonfinite wheel must be invalid')
        if type(wheel.get('grounded')) is not bool or type(wheel.get('valid')) is not bool:
            raise ValueError('grounded/valid must be booleans')
        if not wheel['grounded'] and wheel.get('wheel_hit') is not None:
            raise ValueError('ungrounded contact fields must be null')
        if wheel['grounded'] and wheel['valid'] and not isinstance(wheel.get('wheel_hit'), dict):
            raise ValueError('valid grounded wheel requires wheel_hit')
    return value
