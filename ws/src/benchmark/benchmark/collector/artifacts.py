"""Exclusive artifact creation, reproducibility snapshots and finalization."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

from .config import identifier


def utc():
    return datetime.now(timezone.utc).isoformat()


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('w') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def unique_directory(root, explicit=None, *, timestamp_prefix=None):
    if explicit is not None and timestamp_prefix is not None:
        raise ValueError('Choose an exact identifier or a timestamp prefix, not both')
    if timestamp_prefix is not None:
        identifier(timestamp_prefix)
        # Reserve room for the timestamp and largest collision suffix.
        if len(timestamp_prefix) > 99:
            raise ValueError('Run ID prefix must be at most 99 characters')
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    stem = identifier(explicit) if explicit else datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    if timestamp_prefix is not None:
        stem = timestamp_prefix+'-'+stem
    for suffix in range(10000):
        path = root / (stem if suffix == 0 else f'{stem}-{suffix}')
        try:
            path.mkdir()
            return path
        except FileExistsError:
            if explicit:
                raise ValueError(f'Identifier already exists: {explicit}')
    raise RuntimeError('Could not allocate unique directory')


def git_snapshot(workspace, output):
    output = Path(output)
    output.mkdir()
    def git(*args):
        return subprocess.check_output(['git', '-c', f'safe.directory={workspace}', '-C', str(workspace), *args], stderr=subprocess.PIPE)
    try:
        commit = git('rev-parse', 'HEAD').decode().strip()
        branch = git('rev-parse', '--abbrev-ref', 'HEAD').decode().strip()
        status = git('status', '--porcelain=v1', '-z')
        (output / 'status.bin').write_bytes(status)
        (output / 'tracked.patch').write_bytes(git('diff', '--binary', 'HEAD'))
        untracked = git('ls-files', '--others', '--exclude-standard', '-z').split(b'\0')
        root = Path(git('rev-parse', '--show-toplevel').decode().strip())
        names = []
        for raw in filter(None, untracked):
            name = os.fsdecode(raw)
            source = root / name
            if source.is_symlink():
                raise ValueError(f'Untracked symlink cannot be snapshotted: {name}')
            target = output / 'untracked' / name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
            names.append({'path': name, 'sha256': digest(target)})
        return {'commit': commit, 'branch': branch, 'dirty': bool(status), 'untracked': names,
                'tracked_patch_sha256': digest(output / 'tracked.patch')}
    except (subprocess.CalledProcessError, OSError, ValueError) as exc:
        return {'available': False, 'reason': str(exc)}


def finalize(run, metadata):
    run = Path(run)
    if (run / 'complete.json').exists():
        raise ValueError('Run already finalized')
    metadata['finalized_utc'] = utc()
    write_json(run / 'metadata.json', metadata)
    hashes = {str(p.relative_to(run)): digest(p) for p in sorted(run.rglob('*')) if p.is_file()}
    write_json(run / 'complete.json', {'schema_version': 1, 'sha256': hashes})
    for path in run.rglob('*'):
        path.chmod(0o555 if path.is_dir() else 0o444)
    run.chmod(0o555)


def verify(run):
    run = Path(run)
    marker = run / 'complete.json'
    if not marker.is_file():
        raise ValueError('Run is incomplete; explicit recovery is required')
    manifest = json.loads(marker.read_text())['sha256']
    actual = {str(p.relative_to(run)) for p in run.rglob('*') if p.is_file() and p != marker}
    if actual != set(manifest):
        raise ValueError('Run artifact set changed after finalization')
    for name, expected in manifest.items():
        path = (run / name).resolve()
        if not path.is_relative_to(run.resolve()) or digest(path) != expected:
            raise ValueError(f'Run integrity failure: {name}')
    return json.loads((run / 'metadata.json').read_text())
