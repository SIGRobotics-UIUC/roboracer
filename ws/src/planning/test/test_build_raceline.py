"""End to end on the fixture map: centerline + TUM mintime + preview. Needs the TUM venv (Docker image)."""
import os
import shutil

import numpy as np
import pytest
from PIL import Image

from planning import build_raceline
from planning.raceline import (HEADER, TUM_DIR, TUM_PYTHON, OptimizationError, lap_time, path_traj,
                               read_provenance, read_raceline, run_mintime, speed_profile)

FIX = os.path.join(os.path.dirname(__file__), 'fixtures')
CONFIG = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'racecar.ini')


def fixture_track(tmp_path):
    d = tmp_path / 'fixture'
    d.mkdir()
    shutil.copy(os.path.join(FIX, 'track_map_v3.pgm'), d / 'map.pgm')
    yaml = open(os.path.join(FIX, 'track_map_v3.yaml')).read().replace('track_map_v3.pgm', 'map.pgm')
    (d / 'map.yaml').write_text(yaml)
    return d


@pytest.mark.skipif(not os.path.isfile(TUM_PYTHON),
                    reason='TUM venv not installed: run the tests in the Docker container '
                           '(docker compose exec ros, then python3 -m pytest src/planning/test)')
def test_build_raceline(tmp_path):
    d = fixture_track(tmp_path)
    build_raceline.main(['fixture', '--tracks-dir', str(tmp_path), '--config', CONFIG])   # returns: exit 0

    for f in ('centerline.csv', 'raceline.csv', 'preview.png'):
        assert (d / f).is_file()
    assert (d / 'raceline.csv').read_text().splitlines()[0] == '# ' + HEADER
    rl = read_raceline(d / 'raceline.csv')
    assert rl.shape[1] == 7
    assert np.allclose(rl[0, 1:], rl[-1, 1:])               # closed: last row repeats the first
    assert np.all(np.diff(rl[:, 0]) > 0)
    assert np.all(rl[:, 5] > 0.5) and rl[:, 5].max() < 15.0
    # same direction as the centerline (counter-clockwise here)
    x, y = rl[:-1, 1], rl[:-1, 2]
    assert np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y) > 0
    prov = read_provenance(d / 'raceline.csv')
    assert prov['method'] == 'mintime' and prov['clean'] and prov['notes'] == [] and prov['changed'] == {}
    assert prov['solver'] == 'IPOPT EXIT: Optimal Solution Found.'


def tum_patched():
    """TUM_DIR has docker/tum_mintime.patch's safe_traj ellipse as one constraint row."""
    src = os.path.join(TUM_DIR, 'opt_mintime_traj', 'src', 'opt_mintime.py')
    return os.path.isfile(src) and ('ax_pos_safe"]) ** 2\n                     + (ca.fmin(axk, 0)'
                                    in open(src).read())


@pytest.mark.skipif(not (os.path.isfile(TUM_PYTHON) and tum_patched()),
                    reason='SLOW, and needs the TUM optimizer with the one-row safe_traj ellipse of '
                           'docker/tum_mintime.patch: rebuild the Docker image')
def test_slow_iros2026_clean_mintime(tmp_path):
    # SLOW (minutes): regression for the iros2026 maps, where mintime failed (IPOPT restoration failed /
    # max iterations) while TUM had the safe_traj ellipse as two rows. Must be a clean mintime line, exit 0.
    d = tmp_path / 'iros2026'
    d.mkdir()
    shutil.copy(os.path.join(FIX, 'iros2026.png'), d / 'iros2026.png')
    shutil.copy(os.path.join(FIX, 'iros2026.yaml'), d / 'map.yaml')
    shutil.copy(os.path.join(FIX, 'iros2026_trajectory.csv'), d / 'trajectory.csv')
    build_raceline.main(['iros2026', '--tracks-dir', str(tmp_path), '--config', CONFIG, '--carve-trajectory'])
    prov = read_provenance(d / 'raceline.csv')
    assert prov['method'] == 'mintime' and prov['clean'], prov


def test_fallback_reported(tmp_path, capsys):
    # every TUM run fails as IPOPT would (stand-in venv python): the centerline fallback, loudly, exit 2
    fake = tmp_path / 'python'
    fake.write_text('#!/bin/sh\necho "EXIT: Maximum Number of Iterations Exceeded."\nexit 1\n')
    fake.chmod(0o755)
    d = fixture_track(tmp_path)
    with pytest.raises(SystemExit) as e:
        build_raceline.main(['fixture', '--tracks-dir', str(tmp_path), '--config', CONFIG, '--tum-python', str(fake)])
    assert e.value.code == build_raceline.EXIT_NOT_CLEAN == 2
    err = capsys.readouterr().err
    assert 'is NOT a clean TUM mintime raceline: method centerline' in err
    assert '- mintime failed: mintime IPOPT did not converge (Maximum Number of Iterations Exceeded.)' in err
    lines = (d / 'raceline.csv').read_text().splitlines()
    assert lines[0] == '# ' + HEADER and lines[1].startswith('# raceline: ')
    prov = read_provenance(d / 'raceline.csv')
    assert prov['method'] == 'centerline' and not prov['clean']
    assert [n.split(':')[0] for n in prov['notes']] == ['mintime failed', 'mintime-retry failed']
    assert prov['solver'] is None
    rl = read_raceline(d / 'raceline.csv')              # still loads as pure_pursuit_node loads it
    assert rl.shape[1] == 7 and np.allclose(rl[0, 1:], rl[-1, 1:]) and np.all(rl[:, 5] > 0.5)
    assert (d / 'preview.png').is_file()


def test_unknown_provenance(tmp_path):
    # a hand-written stand-in (as tracks/iros2026_v2 had) or a raceline from before provenance was recorded
    p = tmp_path / 'raceline.csv'
    p.write_text('# ' + HEADER + '  (STAND-IN: centerline)\n0; 0; 0; 0; 0; 1; 0\n1; 1; 0; 0; 0; 1; 0\n')
    assert read_provenance(p) is None


def test_speed_profile():
    t = np.linspace(0, 2 * np.pi, 400, endpoint=False)
    circle = path_traj(np.column_stack((2.0 * np.cos(t), 2.0 * np.sin(t))))
    assert np.allclose(circle[:-1, 4], 0.5, rtol=1e-3)                  # counter-clockwise: kappa > 0
    v = speed_profile(circle, v_max=10.0, ax_pos=2.5, ax_neg=3.5, ay_max=7.0)[:, 5]
    assert np.allclose(v, np.sqrt(7.0 / 0.5), rtol=1e-3)               # the lateral limit all the way round
    # stadium: 20 m straights, r = 2 m half circles; no faster than the circle, and +-ax within the limits
    s = np.arange(0, 20, 0.1)
    a = np.linspace(-np.pi / 2, np.pi / 2, 63, endpoint=False)
    xy = np.vstack((np.column_stack((s, np.full_like(s, -2))), np.column_stack((20 + 2 * np.cos(a), 2 * np.sin(a))),
                    np.column_stack((20 - s, np.full_like(s, 2))), np.column_stack((-2 * np.cos(a), -2 * np.sin(a)))))
    rl = speed_profile(path_traj(xy), v_max=10.0, ax_pos=2.5, ax_neg=3.5, ay_max=7.0)
    assert rl[:, 5].max() < 10.0 and rl[:, 5].min() == pytest.approx(np.sqrt(14.0), rel=0.02)
    assert rl[:, 6].max() <= 2.5 + 1e-9 and rl[:, 6].min() >= -3.5 - 1e-9
    # the ellipse holds at both ends of every segment
    ax = rl[:-1, 6]
    for end in (0, 1):
        ay = rl[end:len(rl) - 1 + end, 5] ** 2 * np.abs(rl[end:len(rl) - 1 + end, 4])
        assert np.max(np.where(ax > 0, ax / 2.5, ax / 3.5) ** 2 + (ay / 7.0) ** 2) <= 1 + 1e-6
    assert lap_time(rl) > 0


def test_failed_optimizer_reported(tmp_path, capsys):
    # stand-in for the venv python: TUM's coloured ERROR line, then a malformed output file ($5)
    fake = tmp_path / 'python'
    fake.write_text('#!/bin/sh\nprintf \'\\033[91mERROR: Optimization did not succeed!\\033[0m\\n\'\n'
                    'echo garbage > "$5"\n')
    fake.chmod(0o755)
    with pytest.raises(OptimizationError, match='malformed'):
        run_mintime(str(tmp_path / 'centerline.csv'), CONFIG, python=str(fake))
    assert '[tum] ERROR: Optimization did not succeed!' in capsys.readouterr().out


def test_stale_preview_removed(tmp_path):
    d = tmp_path / 'blank'
    d.mkdir()
    Image.fromarray(np.full((20, 20), 205, dtype=np.uint8)).save(d / 'map.pgm')   # all unknown
    (d / 'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n'
                                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    (d / 'preview.png').write_bytes(b'old')
    with pytest.raises(SystemExit):
        build_raceline.main(['blank', '--tracks-dir', str(tmp_path), '--config', CONFIG])
    assert not (d / 'preview.png').exists()
