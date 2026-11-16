"""Native bag fixtures exercise storage sorting, clock resets and frame validity offline."""
import hashlib
import json
import os
import pytest

rosbag2_py = pytest.importorskip('rosbag2_py')
pytestmark = pytest.mark.skipif(os.environ.get('ROS_DISTRO') != 'humble', reason='Native bag fixtures target ROS 2 Humble')
pytest.importorskip('geometry_msgs')
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import serialize_message

from benchmark.collector.artifacts import finalize
from benchmark.analysis.core.bags import load_run


@pytest.mark.parametrize('storage', ['sqlite3', 'mcap'])
def test_native_bag_order_index_and_clock_reset(tmp_path, storage):
    if storage not in rosbag2_py.get_registered_writers():
        pytest.skip(f'{storage} storage plugin unavailable; rebuild image for MCAP validation')
    run = tmp_path/'run'; run.mkdir()
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(run/'bag'), storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))
    writer.create_topic(rosbag2_py.TopicMetadata(name='/localized_pose', type='geometry_msgs/msg/PoseStamped',
                                                serialization_format='cdr'))
    source = [1., 1.05, 1.03, 1.05, .2, .25]
    received = [2., 2.05, 2.06, 2.07, .3, .35]
    with (run/'capture_index.jsonl').open('w') as index:
        for sequence, (stamp, record) in enumerate(zip(source, received)):
            message = PoseStamped()
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(round(stamp*1e9), 10**9)
            message.header.frame_id = 'world' if sequence == 5 else 'map'
            message.pose.orientation.w = 1.
            message.pose.position.x = float(sequence)
            raw = serialize_message(message)
            recorded_ns = round(record*1e9)
            writer.write('/localized_pose', raw, recorded_ns)
            index.write(json.dumps({'sequence': sequence, 'topic': '/localized_pose', 'recorded_ns': recorded_ns,
                                    'sha256': hashlib.sha256(raw).hexdigest()})+'\n')
    del writer
    finalize(run, {'clock_domain': 'transport', 'recorder_clock_domain': 'transport',
                   'config': {'logging': {'level': 'basic'}}})
    data = load_run(run)
    raw_order = data.shared['raw_order']['pose']
    assert [s.order for s in raw_order] == list(range(6))
    assert [s.value[0] for s in raw_order] == list(range(6))
    assert raw_order[4].segment != raw_order[3].segment
    assert not raw_order[5].valid
    sorted_first = [s.source_ns for s in data.signals['pose'] if s.segment == 0]
    assert sorted_first == [1_000_000_000, 1_030_000_000, 1_050_000_000, 1_050_000_000]
    assert any('duplicate' in w for w in data.warnings)


def test_ground_truth_parent_and_child_frames_are_separate(tmp_path):
    run = tmp_path/'truth'; run.mkdir()
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(run/'bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    topic = '/ground_truth/odom'
    writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type='nav_msgs/msg/Odometry', serialization_format='cdr'))
    with (run/'capture_index.jsonl').open('w') as index:
        for sequence, child in enumerate(('ground_truth', 'wrong_child')):
            message = Odometry()
            message.header.stamp.sec = sequence+1
            message.header.frame_id = 'world'
            message.child_frame_id = child
            message.pose.pose.orientation.w = 1.
            raw = serialize_message(message)
            stamp = (sequence+1)*10**9
            writer.write(topic, raw, stamp)
            index.write(json.dumps({'sequence': sequence, 'topic': topic, 'recorded_ns': stamp,
                                    'sha256': hashlib.sha256(raw).hexdigest()})+'\n')
    del writer
    finalize(run, {'clock_domain': 'transport', 'config': {'logging': {'level': 'basic'}}})
    samples = load_run(run).signals['truth']
    assert samples[0].valid and samples[0].frame == 'world'
    assert not samples[1].valid


def test_native_lap_time_sentinel_and_numeric_rejections(tmp_path):
    from std_msgs.msg import Int32, Float32
    import math
    run = tmp_path/'lap-times'; run.mkdir()
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(run/'bag'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    ns='/autodrive/roboracer_1/'
    for name, kind in [('lap_count','Int32'), ('last_lap_time','Float32')]:
        writer.create_topic(rosbag2_py.TopicMetadata(name=ns+name, type='std_msgs/msg/'+kind,
                                                   serialization_format='cdr'))
    messages = [('lap_count',Int32(data=0)),('last_lap_time',Float32(data=math.inf)),
                ('lap_count',Int32(data=1)),('last_lap_time',Float32(data=12.5)),
                ('last_lap_time',Float32(data=math.inf)),('last_lap_time',Float32(data=math.nan)),
                ('last_lap_time',Float32(data=-1.))]
    with (run/'capture_index.jsonl').open('w') as index:
        for seq,(name,message) in enumerate(messages):
            raw=serialize_message(message); stamp=(seq+1)*10**9
            writer.write(ns+name,raw,stamp)
            index.write(json.dumps({'sequence':seq,'topic':ns+name,'recorded_ns':stamp,
                                    'sha256':hashlib.sha256(raw).hexdigest()})+'\n')
    del writer
    finalize(run, {'clock_domain':'transport','config':{'logging':{'level':'basic'}},
                   'git':{'commit':'old-commit','dirty':True}})
    before=(run/'metadata.json').read_bytes()
    data=load_run(run)
    diagnostic=data.shared['signal_validation']['last_lap_time']
    assert diagnostic['total']==5 and diagnostic['uninitialized']==1
    assert diagnostic['available']==1 and diagnostic['invalid']==3
    assert diagnostic['rejected_by_reason']=={'frame':0,'timestamp':0,'numeric':3,'payload':0}
    samples=data.shared['raw_order']['last_lap_time']
    assert len(samples)==5 and samples[0].value==math.inf
    assert samples[0].availability=='uninitialized' and not samples[0].valid
    assert math.isnan(samples[3].value)
    assert data.metadata['git']['branch_state']=='unrecorded'
    assert data.metadata['initial_pose_metadata']['resolution']=='unresolved'
    assert (run/'metadata.json').read_bytes()==before
