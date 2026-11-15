"""Small analysis registry and isolated, strict-JSON module results."""
from dataclasses import dataclass
import traceback
import json
import re
import numpy as np


def metric(value=None, units=None, reason=None, **extra):
    return {'value': value, 'units': units, 'status': 'available' if value is not None else 'unavailable',
            **({'reason': reason} if reason and value is None else {}), **extra}


def distribution(values, units):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return metric(units=units, reason='No valid samples')
    return metric({'count': int(len(a)), 'mean': float(a.mean()), 'median': float(np.median(a)),
                   'p95': float(np.percentile(a, 95)), 'max': float(a.max())}, units)


@dataclass
class Module:
    name: str
    version: str
    required: tuple
    optional: tuple
    analyze: object


REGISTRY = {}


def register(module):
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', module.name):
        raise ValueError('Module names must be simple identifiers')
    if module.name in REGISTRY:
        raise ValueError(f'Duplicate module: {module.name}')
    REGISTRY[module.name] = module


def run_modules(dataset, names, output, config):
    results = []
    for name in names:
        if name not in REGISTRY:
            raise ValueError(f'Unknown analysis module: {name}')
        module = REGISTRY[name]
        base = {'name': name, 'version': module.version, 'metrics': {}, 'figures': [], 'warnings': [],
                'required_signals': list(module.required), 'optional_signals': list(module.optional)}
        missing = [key for key in module.required if not any(s.valid for s in dataset.signals.get(key, []))]
        if missing:
            base.update(status='skipped', warnings=['Missing required signals: '+', '.join(missing)])
        else:
            try:
                directory = output/name
                directory.mkdir()
                base.update(module.analyze(dataset, directory, config))
                base.setdefault('status', 'ok')
                for result in base['metrics'].values():
                    if not isinstance(result, dict) or not {'value', 'units', 'status'} <= set(result):
                        raise ValueError('Module metrics must use the standardized metric contract')
                for figure in base['figures']:
                    from pathlib import Path
                    path = Path(figure['path'])
                    if path.is_absolute() or not (directory/path).resolve().is_relative_to(directory.resolve()):
                        raise ValueError('Figure artifacts must stay inside their module directory')
                    figure['path'] = str(directory.relative_to(output)/figure['path'])
                if base['status'] == 'ok' and any(m['value'] is None for m in base['metrics'].values()):
                    base['status'] = 'partial'
                json.dumps(base, allow_nan=False)
            except Exception as exc:
                base.update(status='failed', metrics={}, figures=[], warnings=[str(exc)], error=traceback.format_exc())
        results.append(base)
    return results
