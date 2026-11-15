from copy import deepcopy
import json
import os
from pathlib import Path
import pytest

from benchmark.collector.config import resolve
from benchmark.collector.artifacts import unique_directory, finalize, verify, write_json
from benchmark.collector.adapters import controller_parameters, managed_commands
from benchmark.cli import recover


def configuration():
    return {'experiment': {'map': 'synthetic'}, 'components': {
        'localization': {'type': 'mcl'}, 'controller': {'type': 'map', 'params': {'speed_scale': 0.5}},
        'future_planner': {'type': 'unimplemented', 'managed': False, 'params': {'anything': [1, 2]}}}}


def test_typed_overrides_and_metadata_only_components():
    config = resolve(configuration(), ['components.controller.params.update_rate=20'])
    assert config['components']['controller']['params']['update_rate'] == 20
    assert config['components']['future_planner']['params']['anything'] == [1, 2]
    for override in ('components.controller.params.horizon=20', 'experiment.typo=1',
                     'components.controller.params.speed_scale=true', 'clock.domain=physics'):
        with pytest.raises(ValueError):
            resolve(configuration(), [override])


@pytest.mark.parametrize('change', [
    {'experiment': {'initial_pose': [0, 0, 0]}}, {'logging': {'level': 'wrong'}},
    {'experiment': {'laps': True}}, {'watchdogs': {'loss_s': float('nan')}},
    {'components': {'controller': {'type': 'mpc', 'managed': True}}},
    {'components': {'localization': {'type': 'mcl', 'params': {'particles': 0}}, 'controller': {'type': 'map'}}},
])
def test_reject_unsupported_configuration(change):
    supplied = configuration()
    for key, value in change.items():
        supplied[key] = {**supplied.get(key, {}), **value}
    with pytest.raises(ValueError):
        resolve(supplied)


def test_exclusive_ids_finalization_and_tamper_detection(tmp_path):
    run = unique_directory(tmp_path, 'run')
    with pytest.raises(ValueError):
        unique_directory(tmp_path, 'run')
    with pytest.raises(ValueError):
        unique_directory(tmp_path, '../escape')
    write_json(run/'metadata.json', {'status': 'collecting'})
    with pytest.raises(ValueError, match='incomplete'):
        verify(run)
    (run/'raw.bin').write_bytes(b'raw')
    finalize(run, {'status': 'completed'})
    assert verify(run)['status'] == 'completed'
    assert (run/'raw.bin').stat().st_mode & 0o222 == 0
    with pytest.raises(ValueError, match='already finalized'):
        finalize(run, {})
    # Explicitly simulate owner tampering; read-only flags alone are not a security boundary.
    (run/'raw.bin').chmod(0o644)
    (run/'raw.bin').write_bytes(b'changed')
    with pytest.raises(ValueError, match='integrity'):
        verify(run)
    run.chmod(0o755)


def test_independent_analyses_never_reuse_id(tmp_path):
    a = unique_directory(tmp_path/'reports'/'run', 'analysis-a')
    b = unique_directory(tmp_path/'reports'/'run', 'analysis-b')
    (a/'metrics.json').write_text('{}')
    assert a != b and (a/'metrics.json').read_text() == '{}'
    with pytest.raises(ValueError):
        unique_directory(tmp_path/'reports'/'run', 'analysis-a')


@pytest.mark.parametrize('controller', ['map', 'pure_pursuit'])
def test_managed_startup_maps_parameters_and_remaps_command_owner(controller):
    supplied = configuration()
    supplied['components']['controller']['type'] = controller
    supplied['components']['localization']['params'] = {'particles': 500, 'seed': 0}
    config = resolve(supplied)
    params = controller_parameters(config, {'wheelbase': .33, 'max_steering_angle': .524}, '/assets/raceline.csv')
    assert params['pose_topic'] == '/localized_pose' and params['speed_scale'] == .5
    assert params['raceline_path'] == '/assets/raceline.csv'
    local, control = managed_commands(config, '/run/controller.yaml')
    assert local[:4] == ['ros2', 'launch', 'perception', 'mcl.launch.py']
    assert 'particles:=500' in local
    assert 'seed:=0' in local
    assert control[3] == ('map_node' if controller == 'map' else 'pure_pursuit_node')
    assert control[-2:] == ['-r', '/drive:=/benchmark/requested_drive']


def test_recovery_is_explicit_exclusive_and_never_changes_original(tmp_path):
    import fcntl
    from argparse import Namespace
    source = unique_directory(tmp_path/'runs', 'crashed')
    (source/'metadata.json').write_text('{"status":"collecting"}')
    lock = (source/'collection.lock').open('x')
    fcntl.flock(lock, fcntl.LOCK_SH)
    args = Namespace(log_root=tmp_path, run_id='crashed', new_id='recovered')
    with pytest.raises(ValueError, match='active'):
        recover(args)
    lock.close()
    original = (source/'metadata.json').read_bytes()
    recover(args)
    assert (source/'metadata.json').read_bytes() == original
    assert not (source/'complete.json').exists()
    recovered = verify(tmp_path/'runs'/'recovered')
    assert recovered['status'] == 'recovered_incomplete' and recovered['run_id'] == 'recovered'
    assert recovered['stop_confirmation'] == 'unknown'
    with pytest.raises(ValueError):
        recover(args)


def test_timestamp_run_prefix_collision_and_exact_ids(tmp_path, monkeypatch):
    from datetime import datetime, timezone
    from benchmark.collector import artifacts

    class FrozenTime:
        @staticmethod
        def now(tz):
            return datetime(2026, 11, 15, 4, 56, 26, 159100, tzinfo=timezone.utc)

    monkeypatch.setattr(artifacts, 'datetime', FrozenTime)
    first = unique_directory(tmp_path, timestamp_prefix='porto')
    second = unique_directory(tmp_path, timestamp_prefix='porto')
    assert first.name == 'porto-20261115T045626.159100Z'
    assert second.name == first.name+'-1'
    assert first.is_dir() and second.is_dir()
    assert unique_directory(tmp_path).name == '20261115T045626.159100Z'
    assert unique_directory(tmp_path, 'exact').name == 'exact'
    with pytest.raises(ValueError, match='already exists'):
        unique_directory(tmp_path, 'exact')
    with pytest.raises(ValueError):
        unique_directory(tmp_path, timestamp_prefix='../escape')
    with pytest.raises(ValueError, match='99 characters'):
        unique_directory(tmp_path, timestamp_prefix='x'*100)
    assert len(unique_directory(tmp_path, timestamp_prefix='x'*99).name) <= 128
