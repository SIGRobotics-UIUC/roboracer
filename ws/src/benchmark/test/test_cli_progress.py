"""Configuration shortcuts and human-readable collection progress."""
from pathlib import Path

import pytest

from benchmark.cli import resolve_config
from benchmark.collector.progress import lap_message, result_message


def test_workspace_config_shortcut_and_explicit_precedence(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    config = tmp_path/'src/benchmark/config/porto.yaml'
    config.parent.mkdir(parents=True)
    config.write_text('experiment: {}')
    assert resolve_config('porto') == config
    explicit = tmp_path/'porto'
    explicit.write_text('explicit')
    assert resolve_config('porto') == Path('porto')
    assert resolve_config(config) == config
    with pytest.raises(ValueError, match='does not exist'):
        resolve_config('missing/path.yaml')


def test_repo_root_config_shortcut(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    config = tmp_path/'ws/src/benchmark/config/porto.yaml'
    config.parent.mkdir(parents=True)
    config.touch()
    assert resolve_config('porto') == config


def test_lap_progress_distinguishes_warmup_measurement_and_native_timing():
    assert 'Warm-up lap 1/1' in lap_message(1, 1, 5, 10.25, True)
    assert 'Measurement lap 2/5' in lap_message(3, 1, 5)
    assert '10.250 s (native simulator)' in lap_message(1, 1, 5, 10.25, True)
    assert 'unavailable' in lap_message(2, 1, 5, 10.25, False)
    message = result_message('aborted', 0, 5, 'snapshot failed', None)
    assert '0/5 measured laps' in message and 'snapshot failed' in message


def test_analyze_labels_output_paths(tmp_path, monkeypatch, capsys):
    from benchmark.cli import main
    from benchmark.analysis import runner
    calls = []
    def analyze(*args):
        calls.append(args)
        return tmp_path/'reports'/'run-123'/'analysis-1'
    monkeypatch.setattr(runner, 'analyze', analyze)
    assert main(['analyze', 'run-123', '--modules', 'none']) == 0
    output = capsys.readouterr().out
    assert 'Report generated in:' in output
    assert str(tmp_path/'reports/run-123/analysis-1/report.html') in output
    assert 'Metrics JSON:' in output
    assert calls[0][1:4] == ('run-123', None, 'none')


def test_manual_analysis_hint_is_shell_quoted(tmp_path, capsys):
    import shlex
    from benchmark.cli import collection_finished
    root = tmp_path/'logs with spaces'
    collection_finished(root, 'porto-20261115T045626.159100Z')
    output = capsys.readouterr().out
    command = output.split('\n  ', 1)[1].splitlines()[0]
    assert shlex.split(command) == ['benchmark', 'analyze', 'porto-20261115T045626.159100Z']
    assert 'Custom artifact root: pass --log-root' in output
    assert 'add --analyze' in output


@pytest.mark.parametrize('automatic', [False, True])
@pytest.mark.parametrize('status', [0, 1])
def test_run_analysis_chain_preserves_collection_exit_status(tmp_path, monkeypatch, automatic, status):
    from benchmark import cli
    from benchmark.collector import manager
    config = tmp_path/'porto.yaml'
    config.write_text('fixture')
    monkeypatch.setattr(cli, 'load', lambda *args: {})
    calls = []
    def run(config, args, *, on_finalized):
        on_finalized('porto-generated-timestamp')
        return status
    monkeypatch.setattr(manager, 'run', run)
    monkeypatch.setattr(cli, 'collection_finished', lambda *args: calls.append(args))
    args = ['--log-root', str(tmp_path), 'run', '--config', str(config)]
    if automatic:
        args.append('--analyze')
    assert cli.main(args) == status
    assert calls == [(tmp_path, 'porto-generated-timestamp', automatic)]


def test_unfinalized_run_does_not_offer_analysis(tmp_path, monkeypatch):
    from benchmark import cli
    from benchmark.collector import manager
    config = tmp_path/'porto.yaml'
    config.touch()
    monkeypatch.setattr(cli, 'load', lambda *args: {})
    monkeypatch.setattr(manager, 'run', lambda *args, **kwargs: 1)
    calls = []
    monkeypatch.setattr(cli, 'collection_finished', lambda *args: calls.append(args))
    assert cli.main(['run', '--config', str(config), '--analyze']) == 1
    assert not calls


def test_default_artifact_root_and_legacy_override(tmp_path, monkeypatch):
    from benchmark.cli import parser
    monkeypatch.chdir(tmp_path)
    for command in (['run', '--config', 'porto'], ['analyze', 'run-id'],
                    ['status'], ['recover', 'old-run', '--run-id', 'recovered']):
        assert parser().parse_args(command).log_root == tmp_path/'benchmark_runs'
    assert parser().parse_args(['--log-root', str(tmp_path/'log'),
                               'analyze', 'old-run']).log_root == tmp_path/'log'
