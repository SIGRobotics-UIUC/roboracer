"""Metadata provenance and native lap-time validation without a live ROS graph."""
import math
import subprocess
from types import SimpleNamespace as Message

import pytest

from benchmark.collector.artifacts import git_snapshot
from benchmark.collector.metadata import git_metadata, initial_pose_metadata
from benchmark.analysis.core.bags import validate_message


def test_git_snapshot_attached_detached_and_unavailable(tmp_path):
    repo = tmp_path/'repo'
    repo.mkdir()
    def git(*args):
        return subprocess.check_output(['git', '-C', str(repo), *args]).decode().strip()
    git('init', '-b', 'recorded-branch')
    git('-c', 'user.name=Test', '-c', 'user.email=test@example.com', 'commit', '--allow-empty', '-m', 'fixture')
    attached = git_snapshot(repo, tmp_path/'attached')
    assert attached['branch'] == 'recorded-branch' and attached['branch_state'] == 'attached'
    assert attached['dirty'] is False
    (repo/'dirty.txt').write_text('fixture')
    assert git_snapshot(repo, tmp_path/'dirty')['dirty'] is True
    git('checkout', '--detach')
    detached = git_snapshot(repo, tmp_path/'detached')
    assert detached['branch'] is None and detached['branch_state'] == 'detached'
    assert detached['commit'] == attached['commit']
    missing = git_snapshot(tmp_path, tmp_path/'unavailable')
    assert missing['branch_state'] == 'unavailable' and missing['dirty'] is None


def test_legacy_metadata_never_borrows_current_git_or_observed_pose():
    old = git_metadata({'commit':'old', 'dirty': True})
    assert old['branch'] is None and old['branch_state'] == 'unrecorded'
    assert old['dirty'] is True
    assert git_metadata({'branch':'HEAD', 'commit':'old'})['branch_state'] == 'detached'
    for config in ({}, {'experiment': {'initial_pose':None}}):
        pose = initial_pose_metadata(config)
        assert pose['explicitly_supplied'] is False
        assert pose['resolution'] == 'unresolved' and pose['resolved_pose'] is None
    supplied = initial_pose_metadata({'experiment': {'initial_pose':[1,2,3]}})
    assert supplied['requested_pose'] == [1,2,3]
    assert supplied['resolved_pose'] is None  # requested does not mean applied


@pytest.mark.parametrize('value,counter,state', [
    (math.inf,0,'uninitialized'), (math.inf,1,'invalid'), (math.inf,None,'invalid'),
    (-math.inf,0,'invalid'), (math.nan,0,'invalid'), (0.,0,'invalid'),
    (-1.,0,'invalid'), (13.58,1,'available')])
def test_last_lap_time_states_preserve_values(value, counter, state):
    decoded, valid, availability, reasons = validate_message('last_lap_time', Message(data=value), None, counter)
    assert decoded is value
    assert availability == state and valid == (state == 'available')
    assert reasons == (['numeric'] if state == 'invalid' else [])


def test_rejection_reasons_are_independent_and_headerless_time_is_normal():
    msg = Message(header=Message(frame_id='world'), drive=Message(speed=math.nan, steering_angle=0.))
    _, valid, state, reasons = validate_message('drive', msg, 0)
    assert not valid and state == 'invalid'
    assert set(reasons) == {'frame','timestamp','numeric'}
    _, valid, _, reasons = validate_message('last_lap_time', Message(data=12.), None, 1)
    assert valid and not reasons


def test_snapshot_failure_retains_available_git_identity(tmp_path):
    repo = tmp_path/'repo'; repo.mkdir()
    def git(*args):
        subprocess.check_call(['git', '-C', str(repo), *args], stdout=subprocess.DEVNULL)
    git('init','-b','branch-with-snapshot-error')
    git('-c','user.name=Test','-c','user.email=test@example.com','commit','--allow-empty','-m','fixture')
    (repo/'unsupported-link').symlink_to('missing-target')
    source = git_snapshot(repo, tmp_path/'snapshot')
    assert source['available'] is False and source['commit']
    assert source['branch'] == 'branch-with-snapshot-error'
    assert source['dirty'] is True
