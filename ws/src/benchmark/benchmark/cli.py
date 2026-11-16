"""The benchmark command works offline for analysis and status inspection."""
import argparse
import json
from pathlib import Path
import shutil
import shlex

from .collector.artifacts import finalize, unique_directory, utc, write_json
from .collector.config import identifier, load


def resolve_config(value):
    """Prefer explicit files; resolve bare names in workspace then installed config."""
    path = Path(value)
    if path.is_file():
        return path
    if path.parent != Path('.') or path.suffix:
        raise ValueError(f'Configuration file does not exist: {path}')
    candidates = [Path.cwd()/'src/benchmark/config'/f'{path.name}.yaml',
                  Path.cwd()/'ws/src/benchmark/config'/f'{path.name}.yaml']
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory('benchmark'))/'config'/f'{path.name}.yaml')
    except (ImportError, LookupError):
        pass
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise ValueError('Configuration not found; searched: '+', '.join(map(str, candidates)))


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--log-root', type=Path, default=Path.cwd()/'benchmark_runs',
                   help='Benchmark artifact root (default: ./benchmark_runs)')
    p.add_argument('--tracks-dir', type=Path, default=Path.cwd()/'tracks')
    sub = p.add_subparsers(dest='command', required=True)
    run = sub.add_parser('run')
    run.add_argument('--config', required=True, type=Path, help='YAML path or config name, e.g. porto')
    run.add_argument('--run-id', type=identifier,
                     help='Run ID prefix; a UTC timestamp is appended automatically')
    run.add_argument('--set', action='append', default=[])
    run.add_argument('--analyze', action='store_true',
                     help='Generate a report after the collection is finalized, including aborted runs')
    sub.add_parser('status')
    sub.add_parser('stop')
    analysis = sub.add_parser('analyze')
    analysis.add_argument('run_id', type=identifier)
    analysis.add_argument('--analysis-id')
    analysis.add_argument('--modules', default='all')
    analysis.add_argument('--max-gap-s', type=float, default=0.1)
    recovery = sub.add_parser('recover', help='Copy an incomplete run into a new finalized failure artifact')
    recovery.add_argument('run_id', type=identifier)
    recovery.add_argument('--run-id', dest='new_id', required=True, type=identifier)
    return p


def generate_report(log_root, run_id, analysis_id=None, modules='all', max_gap_s=0.1):
    """Use the existing analyzer and label its output artifacts for terminal users."""
    from .analysis.runner import analyze
    from .collector.progress import say
    say(f'Analyzing run {run_id}; generating report…')
    output = Path(analyze(log_root, run_id, analysis_id, modules,
                          {'max_gap_s': max_gap_s})).resolve()
    say(f'Report generated in: {output}')
    say(f'View HTML report at: {output / "report.html"}')
    say(f'Metrics JSON: {output / "metrics.json"}')
    return output


def collection_finished(log_root, run_id, analyze=False):
    """Called only after sealing and cleaning up the collection."""
    if analyze:
        generate_report(log_root, run_id)
    else:
        from .collector.progress import say
        command = shlex.join(['benchmark', 'analyze', run_id])
        say('Generate a report with:\n  '+command)
        if Path(log_root).resolve() != (Path.cwd()/'benchmark_runs').resolve():
            say('Custom artifact root: pass --log-root '+shlex.quote(str(Path(log_root).resolve()))+
                ' before analyze.')
        say('Tip: add --analyze to benchmark run to generate the report automatically.')


def main(argv=None):
    args = parser().parse_args(argv)
    try:
        if args.command == 'analyze':
            import math
            if not math.isfinite(args.max_gap_s) or args.max_gap_s <= 0:
                raise ValueError('--max-gap-s must be finite and positive')
            generate_report(args.log_root, args.run_id, args.analysis_id, args.modules, args.max_gap_s)
        elif args.command == 'run':
            args.config = resolve_config(args.config)
            from .collector.progress import say
            say(f'Configuration: {args.config.resolve()}')
            config = load(args.config, args.set)
            from .collector.manager import run
            return run(config, args, on_finalized=lambda run_id:
                       collection_finished(args.log_root, run_id, args.analyze))
        elif args.command == 'recover':
            recover(args)
        else:
            from .collector.manager import remote
            return remote(args.command)
    except (ValueError, OSError, RuntimeError) as exc:
        print(json.dumps({'status': 'error', 'reason': str(exc)}))
        return 1
    return 0


def recover(args):
    import fcntl
    source = args.log_root/'runs'/args.run_id
    if not source.exists():
        source = args.log_root/'attempts'/args.run_id
    if (source/'complete.json').exists():
        raise ValueError('Finalized runs cannot be recovered or modified')
    if not source.is_dir():
        raise ValueError('Incomplete run does not exist')
    lock = source/'collection.lock'
    # Opening an existing lock does not change raw data; preserve the original.
    with lock.open('r') as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError('Cannot recover an active collection') from None
        if any(path.is_symlink() for path in source.rglob('*')):
            raise ValueError('Recovery refuses symbolic links')
        target = unique_directory(args.log_root/'runs', args.new_id)
        for child in source.iterdir():
            if child.is_symlink():
                raise ValueError('Recovery refuses symbolic links')
            if child.is_dir():
                shutil.copytree(child, target/child.name)
            else:
                shutil.copyfile(child, target/child.name)
        metadata_path = target/'metadata.json'
        metadata = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
        metadata.update(status='recovered_incomplete', failure='crash_or_incomplete_finalization',
                        run_id=target.name, recovery_source=args.run_id, recovery_utc=utc(), stop_confirmation='unknown')
        # No bag repair is performed implicitly. Valid existing metadata may permit partial analysis.
        finalize(target, metadata)
        print(target)


if __name__ == '__main__':
    raise SystemExit(main())
