"""Regenerate independent reports from immutable collections."""
from pathlib import Path
import subprocess
import os

from .. import __version__
from ..collector.artifacts import unique_directory, write_json, utc, verify, git_snapshot
from .core.bags import load_run
from .core.results import REGISTRY, register, run_modules
from .core.signals import SIGNAL_MAP, SIGNAL_MAP_VERSION
from .modules import race_performance, computation_timing
from ..reports.render import render


def analyze(log_root, run_id, analysis_id=None, modules='all', config=None):
    from ..collector.config import identifier
    run = Path(log_root) / 'runs' / identifier(run_id)
    verify(run)
    for module in (race_performance.MODULE, computation_timing.MODULE):
        if module.name not in REGISTRY:
            register(module)
    names = list(REGISTRY) if modules == 'all' else modules.split(',')
    if not names or len(set(names)) != len(names) or any(name not in REGISTRY for name in names):
        raise ValueError('Unknown analysis module selection')
    config = config or {'max_gap_s': 0.1}
    output = unique_directory(Path(log_root) / 'reports' / run_id, analysis_id)
    try:
        data = load_run(run)
        results = run_modules(data, names, output, config)
        source = git_snapshot(Path(os.environ.get('BENCHMARK_REPO_ROOT', Path.cwd())), output/'source')
        if source.get('available') is False:
            data.warnings.append('Analysis source snapshot unavailable: '+source['reason'])
        try:
            revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], stderr=subprocess.DEVNULL).decode().strip()
            dirty = bool(subprocess.check_output(['git', 'status', '--porcelain'], stderr=subprocess.DEVNULL))
        except (OSError, subprocess.CalledProcessError):
            revision, dirty = None, None
        summary = {'schema_version': 1, 'run_id': run_id, 'analysis_id': output.name,
                   'analysis_utc': utc(), 'analysis_version': __version__, 'analysis_commit': source.get('commit', revision),
                   'analysis_dirty': source.get('dirty', dirty), 'analysis_configuration': config,
                   'analysis_source': source,
                   'signal_map_version': SIGNAL_MAP_VERSION, 'signal_map': SIGNAL_MAP,
                   'sign_conventions': 'x forward, y left, yaw/steer positive left; angles radians',
                   'experiment': data.metadata, 'warnings': data.warnings, 'modules': results,
                   'status': 'failed' if any(m['status'] == 'failed' for m in results) else 'complete'}
        write_json(output / 'metrics.json', summary)
        render(output, summary)
    except Exception as exc:
        write_json(output / 'analysis_failure.json', {'status': 'failed', 'reason': str(exc)})
        raise
    return output
