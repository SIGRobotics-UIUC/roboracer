"""SLAM Toolbox mapping: interfaces/config, transactional saves and planner contract."""
import importlib.util
import math
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from slam_toolbox.srv import SerializePoseGraph

from perception.save_slam_map import (SaveSlamMap, destination, save_bundle, validate_artifacts,
                                      write_grid, write_trajectory)
from perception.slam_mapping_pose import SlamMappingPose, valid_pose

PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
SCRIPT = WORKSPACE / 'scripts/map_track.sh'


def grid():
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.width, msg.info.height, msg.info.resolution = 3, 2, .05
    msg.info.origin.orientation.w = 1.
    msg.info.origin.position.x, msg.info.origin.position.y = -1., 2.
    msg.data = [0, -1, 100, 100, -1, 0]
    return msg


def trajectory():
    msg = RosPath()
    msg.header.frame_id = 'map'
    for i in range(12):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp.sec = i + 1
        p.pose.position.x = i * .1
        p.pose.orientation.w = 1.
        msg.poses.append(p)
    return msg


def serializer(prefix):
    Path(prefix + '.posegraph').write_bytes(b'synthetic graph')
    Path(prefix + '.data').write_bytes(b'synthetic scan data')


def fake_extract(prefix, directory):
    directory.mkdir()
    write_grid(directory, grid())
    write_trajectory(directory, trajectory())
    (directory / 'odometry.csv').write_bytes((directory / 'trajectory.csv').read_bytes())
    (directory / 'graph_metadata.json').write_text('{}')


@pytest.fixture(autouse=True)
def mock_native_export_for_unit_tests(monkeypatch):
    monkeypatch.setattr('perception.save_slam_map.export_snapshot', fake_extract)


def test_backend_selection_executes_actual_shell_parser():
    parser = SCRIPT.read_text().split('source "$(dirname "$0")/common.sh"')[0]
    for flags, expected in [([], 'cartographer|mapping.launch.py|save_map|'),
                            (['--slam-toolbox', '--wall-follow', '--laps', '2'],
                             'slam_toolbox|slam_mapping.launch.py|save_slam_map|wall_follow.launch.py')]:
        command = parser + '\nprintf "%s|%s|%s|%s" "$backend" "$mapping_launch" "$save_command" "$follow_launch"'
        result = subprocess.run(['bash', '-c', command, 'map_track.sh', 'new_track', *flags],
                                capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
        assert result.stdout == expected
    for flags in (['--unknown'], ['--laps', '-1'], ['--wall-follow', '--gap-follow']):
        assert subprocess.run(['bash', '-c', parser, 'map_track.sh', 'new', *flags], capture_output=True).returncode != 0


def test_map_track_reports_slow_scan_rates(tmp_path):
    # Regression: maps mapped with scans arriving at ~5 Hz (a slow simulator) came out with the track drawn twice;
    # map_track.sh warns from the rates Cartographer logs.
    parser = SCRIPT.read_text().split('source "$(dirname "$0")/common.sh"')[0]
    log = tmp_path / 'map_track.log'
    line = '[cartographer_node-1] [INFO] [1.0] [cartographer logger]: I collated_trajectory_builder.cc:81] {} rate: {} Hz'

    def summary(rates, env=None):
        log.write_text(''.join(line.format(kind, rate) + '\n' for rate in rates for kind in ('imu', 'odom', 'scan')))
        command = parser + '\nscan_rate_summary "$LOG" "$min_scan_hz"'
        return subprocess.run(['bash', '-c', command, 'map_track.sh', 'new'], capture_output=True, text=True,
                              check=True, env=dict(os.environ, LOG=str(log), **(env or {}))).stdout.strip()

    assert summary([19.41, 22.42, 9.25, 5.15, 5.78]) == '5 3 5.2'      # readings, below 10 Hz, lowest
    assert summary([19.69, 25.48, 16.44]) == '3 0 16.4'                # a healthy run
    assert summary([19.69, 25.48, 16.44], {'MIN_SCAN_HZ': '20'}) == '3 2 16.4'
    assert summary([]) == ''                                           # no reading (SLAM Toolbox, a short run)
    command = parser + '\ntrue'
    assert subprocess.run(['bash', '-c', command, 'map_track.sh', 'new'], capture_output=True,
                          env=dict(os.environ, MIN_SCAN_HZ='fast')).returncode != 0


def test_slam_only_conservative_speed_and_user_override_order():
    parser = SCRIPT.read_text().split('source "$(dirname "$0")/common.sh"')[0]
    command = parser + '\nprintf "%s\\n" "${follow_args[@]}"'
    for flags, expected in [(['--wall-follow'], ''),
                            (['--slam-toolbox','--wall-follow'], 'speed:=0.5'),
                            (['--slam-toolbox','--wall-follow','speed:=0.4'], 'speed:=0.5\nspeed:=0.4')]:
        result = subprocess.run(['bash','-c',command,'map_track.sh','new',*flags],capture_output=True,text=True,check=True)
        assert result.stdout.strip()==expected


def test_launch_and_config_use_installed_async_interface():
    spec = importlib.util.spec_from_file_location('slam_mapping_launch', PACKAGE / 'launch/slam_mapping.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    from launch_ros.actions import Node
    nodes = [a for a in description.entities if isinstance(a, Node)]
    assert len(nodes) == 2
    text = (PACKAGE / 'launch/slam_mapping.launch.py').read_text()
    assert "executable='async_slam_toolbox_node'" in text
    assert "('pose', '/mapping/slam_pose')" in text
    assert 'ground_truth' not in text and 'map_origin' not in text.replace('no map_origin', '')
    config = yaml.safe_load((PACKAGE / 'config/slam_mapping.yaml').read_text())['slam_toolbox']['ros__parameters']
    assert config['mode'] == 'mapping' and config['use_sim_time'] is False
    assert (config['map_frame'], config['odom_frame'], config['base_frame']) == ('map', 'odom', 'base_link')
    assert config['scan_topic'] == '/scan' and config['resolution'] == .05
    assert config['transform_publish_period'] > 0
    assert SerializePoseGraph.Request(filename='/tmp/prefix').filename == '/tmp/prefix'
    assert SerializePoseGraph.Response.RESULT_SUCCESS == 0


def test_grid_encoding_matches_planner(tmp_path):
    write_grid(tmp_path, grid())
    meta = yaml.safe_load((tmp_path / 'map.yaml').read_text())
    assert meta['resolution'] == .05 and meta['origin'] == [-1., 2., 0.]
    assert list((tmp_path / 'map.pgm').read_bytes()[-6:]) == [0, 205, 254, 254, 205, 0]
    from planning.centerline import OccupancyMap
    loaded = OccupancyMap(str(tmp_path / 'map.yaml'))
    assert loaded.free.sum() == 2 and loaded.occupied.sum() == 2


def test_saved_map_and_clockwise_trajectory_feed_centerline(tmp_path):
    from planning.centerline import OccupancyMap, extract_centerline, signed_area
    fixture = WORKSPACE / 'src/planning/test/fixtures/track_map_v3.yaml'
    source = OccupancyMap(str(fixture))
    centerline, _, _ = extract_centerline(str(fixture))
    msg = grid()
    msg.info.width, msg.info.height = source.w, source.h
    msg.info.origin.position.x = float(source.origin[0])
    msg.info.origin.position.y = float(source.origin[1])
    occ = np.full((source.h, source.w), -1, dtype=np.int8)
    occ[source.free], occ[source.occupied] = 0, 100
    msg.data = occ[::-1].ravel().tolist()
    write_grid(tmp_path, msg)
    path = trajectory()
    path.poses.clear()
    for i, (x, y) in enumerate(centerline[::-1, :2]):
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp.sec = 'map', i + 1
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.w = 1.
        path.poses.append(pose)
    write_trajectory(tmp_path, path)
    planned, _, _ = extract_centerline(str(tmp_path / 'map.yaml'), str(tmp_path / 'trajectory.csv'))
    assert signed_area(planned[:, :2]) < 0


@pytest.mark.parametrize('change', ['frame', 'resolution', 'dimensions', 'unknown', 'rotation', 'nan'])
def test_invalid_grid_refused(tmp_path, change):
    msg = grid()
    if change == 'frame': msg.header.frame_id = 'world'
    if change == 'resolution': msg.info.resolution = .1
    if change == 'dimensions': msg.info.width = 5
    if change == 'unknown': msg.data = [-1] * 6
    if change == 'rotation': msg.info.origin.orientation.z = .1
    if change == 'nan': msg.info.origin.position.x = float('nan')
    with pytest.raises(RuntimeError, match='map'):
        write_grid(tmp_path, msg)


def test_bundle_published_only_after_graph_pair_validated(tmp_path):
    out = tmp_path / 'new'
    save_bundle(out, grid(), trajectory(), serializer)
    validate_artifacts(out)
    assert not (out / 'map.pbstream').exists()
    assert not (out / 'map_to_world.yaml').exists()
    assert len((out / 'trajectory.csv').read_text().splitlines()) == 13
    original = (out / 'map.pgm').read_bytes()
    with pytest.raises(FileExistsError):
        save_bundle(out, grid(), trajectory(), serializer)
    assert (out / 'map.pgm').read_bytes() == original


def test_unrelated_final_graph_is_not_published(tmp_path):
    def wrong_graph(prefix, directory):
        fake_extract(prefix, directory)
        values=np.loadtxt(directory/'trajectory.csv',delimiter=',')
        values[:,0]+=100
        np.savetxt(directory/'trajectory.csv',values,delimiter=',')
    with pytest.raises(RuntimeError, match='does not cover this mapping trajectory'):
        save_bundle(tmp_path/'new',grid(),trajectory(),serializer,extract=wrong_graph)
    assert not (tmp_path/'new').exists()


@pytest.mark.parametrize('mode', ['missing_data', 'empty_graph', 'service_error'])
def test_save_failure_preserves_stage_and_allows_retry(tmp_path, mode):
    out = tmp_path / 'new'
    def fail(prefix):
        if mode == 'service_error':
            raise RuntimeError('serialize service unavailable')
        serializer(prefix)
        if mode == 'missing_data': Path(prefix + '.data').unlink()
        if mode == 'empty_graph': Path(prefix + '.posegraph').write_bytes(b'')
    with pytest.raises(RuntimeError, match='staging artifacts retained'):
        save_bundle(out, grid(), trajectory(), fail)
    assert not out.exists()
    assert list(tmp_path.glob('.new-saving-*'))
    save_bundle(out, grid(), trajectory(), serializer)
    validate_artifacts(out)


def test_destination_rejects_traversal_and_existing_symlink(tmp_path, monkeypatch):
    monkeypatch.setenv('TRACKS_DIR', str(tmp_path))
    with pytest.raises(ValueError): destination('../unsafe')
    out = destination('new')
    out.symlink_to(tmp_path / 'absent')
    with pytest.raises(FileExistsError): save_bundle(out, grid(), trajectory(), serializer)


def test_trajectory_rejects_bad_direction_inputs(tmp_path):
    msg = trajectory()
    msg.poses[2].header.stamp.sec = 1
    with pytest.raises(RuntimeError): write_trajectory(tmp_path, msg)
    msg = trajectory()
    msg.header.frame_id = 'world'
    with pytest.raises(RuntimeError): write_trajectory(tmp_path, msg)


def test_mapping_pose_relay_has_no_privileged_subscriptions():
    rclpy.init()
    node = SlamMappingPose()
    try:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id, msg.header.stamp.sec = 'map', 1
        msg.pose.pose.orientation.w = 1.
        assert valid_pose(msg)
        node.pose_cb(msg)
        node.pose_cb(msg)  # same stamp is not another trajectory sample
        assert len(node.path.poses) == 1
        msg.header.frame_id = 'world'
        assert not valid_pose(msg)
        node.pose_cb(msg)
        assert len(node.path.poses) == 1
        inputs = [topic for topic, _ in node.get_subscriber_names_and_types_by_node(node.get_name(), '/')]
        assert '/mapping/slam_pose' in inputs
        assert not any('ground_truth' in topic or 'autodrive' in topic for topic in inputs)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize('mode', ['unavailable', 'timeout', 'failure'])
def test_serialization_errors_are_not_success(monkeypatch, mode):
    response = SerializePoseGraph.Response(result=255)
    future = SimpleNamespace(done=lambda: mode != 'timeout', result=lambda: response)
    client = SimpleNamespace(wait_for_service=lambda **kw: mode != 'unavailable', call_async=lambda req: future)
    destroyed = []
    fake = SimpleNamespace(timeout=.01, create_client=lambda *args: client, destroy_client=destroyed.append)
    monkeypatch.setattr(rclpy, 'spin_until_future_complete', lambda *args, **kwargs: None)
    with pytest.raises(RuntimeError): SaveSlamMap.serialize(fake, '/tmp/prefix')
    assert destroyed == [client]
