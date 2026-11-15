"""Strict framework configuration; component parameters require explicit adapters."""
from copy import deepcopy
import math
import json
from pathlib import Path
import re
import yaml

IDENTIFIER = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\Z')
DEFAULTS = {
    'experiment': {'laps': 5, 'warmup_laps': 1, 'map': '', 'mode': 'managed',
                   'initial_pose': None, 'timeout_s': 300.0, 'collision_policy': 'abort'},
    'components': {}, 'logging': {'level': 'basic', 'storage': 'auto', 'require_ground_truth': False},
    'watchdogs': {'loss_s': 1.0, 'signals': {}, 'startup_s': 30.0,
                  'heartbeat_s': 1.0, 'stop_timeout_s': 5.0,
                  'stop_hold_s': 1.5, 'rest_dwell_s': 0.5, 'rest_speed_mps': 0.05,
                  'disk_reserve_mb': 256.0},
    'clock': {'domain': 'transport', 'use_sim_time': True},
    'metadata': {'simulator_version': None, 'physics_step_s': None,
                 'vehicle_configuration': None, 'seeds': {}},
}
COMMON = {'speed_scale': float, 'max_speed': float}
CONTROLLERS = {
    'map': {**COMMON, 'lookahead_slope': float, 'lookahead_offset': float,
            'lookahead_min': float, 'lookahead_max': float, 'steering_rate_limit': float,
            'update_rate': float},
    'pure_pursuit': {**COMMON, 'lookahead_distance': float, 'lookahead_min': float,
                     'lookahead_time': float, 'adaptive_lookahead': bool,
                     'adaptive_lookahead_cut': float, 'adaptive_lookahead_slope': float,
                     'adaptive_lookahead_min_time': float, 'adaptive_lookahead_smooth': float},
}
LOCALIZERS = {'cartographer': {}, 'mcl': {'particles': int, 'seed': int}}


def identifier(value):
    if not isinstance(value, str) or not IDENTIFIER.fullmatch(value) or value in ('.', '..'):
        raise ValueError('IDs must be simple names of at most 128 characters')
    return value


def merge(base, supplied, path=''):
    if not isinstance(supplied, dict):
        raise ValueError(f'{path or "configuration"} must be a mapping')
    for key, value in supplied.items():
        name = f'{path}.{key}'.strip('.')
        if key not in base:
            raise ValueError(f'Unsupported framework field: {name}')
        if isinstance(base[key], dict) and name not in ('components', 'watchdogs.signals', 'metadata.seeds'):
            merge(base[key], value, name)
        else:
            base[key] = deepcopy(value)


def load(path, overrides=()):
    requested = yaml.safe_load(Path(path).read_text())
    return resolve(requested, overrides)


def resolve(requested, overrides=()):
    config = deepcopy(DEFAULTS)
    merge(config, requested)
    for expression in overrides:
        key, sep, raw = expression.partition('=')
        if not sep:
            raise ValueError('--set requires path=value')
        parts = key.split('.')
        parent = config
        for part in parts[:-1]:
            if not isinstance(parent, dict) or part not in parent:
                raise ValueError(f'Invalid override path: {key}')
            parent = parent[part]
        component_param = len(parts) == 4 and parts[0] == 'components' and parts[2] == 'params'
        if not isinstance(parent, dict) or (parts[-1] not in parent and not component_param):
            raise ValueError(f'Invalid override path: {key}')
        parent[parts[-1]] = yaml.safe_load(raw)
    exp = config['experiment']
    identifier(exp['map'])
    if exp['mode'] not in ('managed', 'manual') or exp['collision_policy'] not in ('abort', 'continue'):
        raise ValueError('Unsupported experiment mode or collision policy')
    for key in ('laps', 'warmup_laps'):
        if type(exp[key]) is not int or exp[key] < (1 if key == 'laps' else 0):
            raise ValueError(f'{key} must be a valid integer')
    if exp['initial_pose'] is not None:
        raise ValueError('Arbitrary initial_pose is unsupported by the native reset interface; '
                         'record an operator declaration in metadata.vehicle_configuration instead')
    if config['clock']['domain'] != 'transport' or config['clock']['use_sim_time'] is not True:
        raise ValueError('v1 requires the bridge monotonic transport /clock; physics time is unavailable')
    if config['logging']['level'] not in ('off', 'basic', 'detailed'):
        raise ValueError('logging.level must be off, basic or detailed')
    if config['logging']['storage'] not in ('auto', 'mcap', 'sqlite3'):
        raise ValueError('Unsupported bag storage')
    if type(config['logging']['require_ground_truth']) is not bool:
        raise ValueError('require_ground_truth must be boolean')
    for key, value in config['watchdogs'].items():
        values = value.values() if key == 'signals' and isinstance(value, dict) else [value]
        for number in values:
            positive(number, f'watchdogs.{key}')
    positive(exp['timeout_s'], 'experiment.timeout_s')
    components = config['components']
    if not isinstance(components, dict):
        raise ValueError('components must be a mapping')
    for name, component in components.items():
        identifier(name)
        if not isinstance(component, dict) or set(component) - {'type', 'managed', 'params'}:
            raise ValueError(f'Invalid component: {name}')
        if not isinstance(component.get('type'), str) or not component['type']:
            raise ValueError(f'{name}.type is required')
        component.setdefault('managed', exp['mode'] == 'managed' and name in ('controller', 'localization'))
        component.setdefault('params', {})
        if type(component['managed']) is not bool or not isinstance(component['params'], dict):
            raise ValueError(f'Invalid managed/params for {name}')
        if not component['managed']:
            continue
        adapters = CONTROLLERS if name == 'controller' else LOCALIZERS if name == 'localization' else {}
        if component['type'] not in adapters:
            raise ValueError(f'No managed adapter for {name}:{component["type"]}; use metadata-only/manual mode')
        allowed = adapters[component['type']]
        for key, value in component['params'].items():
            if key not in allowed:
                raise ValueError(f'Unmapped managed parameter: {name}.{key}')
            kind = allowed[key]
            if kind is bool or kind is int:
                if type(value) is not kind:
                    raise ValueError(f'{name}.{key} must be {kind.__name__}')
            elif type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f'{name}.{key} must be finite numeric')
            if kind is not bool and value <= 0 and key not in ('max_speed', 'lookahead_offset', 'seed'):
                raise ValueError(f'{name}.{key} must be positive')
            if key == 'max_speed' and value < 0:
                raise ValueError('max_speed cannot be negative')
            if key == 'seed' and value < -1:
                raise ValueError('seed must be -1 (random) or nonnegative')
    if exp['mode'] == 'managed':
        if any(not components.get(name, {}).get('managed') for name in ('controller', 'localization')):
            raise ValueError('Managed mode needs managed controller and localization components')
    elif any(c['managed'] for c in components.values()):
        raise ValueError('Manual mode components must be metadata-only')
    try:
        json.dumps(config, allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise ValueError('Configuration must contain finite JSON-compatible values') from exc
    return config


def positive(value, name):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
