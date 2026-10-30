import base64
import gzip
import math

import pytest
from std_msgs.msg import Bool, Float32

from autodrive_bridge.fast_bridge import FastBridge, is_jump

# every key on_frame reads with publish_camera and publish_restricted_topics false (conftest.py)
FRAME = {
    'V1 Throttle': '0.0', 'V1 Steering': '0.0',
    'V1 Orientation Quaternion': '0 0 0 1', 'V1 Angular Velocity': '0 0 0',
    'V1 Linear Acceleration': '0 0 0', 'V1 Encoder Angles': '0 0', 'V1 LIDAR Scan Rate': '40',
    'V1 LIDAR Range Array': base64.b64encode(gzip.compress(('1.0\n' * 1081).encode())).decode(),
}
RESTRICTED = ('ips', 'odom', 'lap_count', 'collision_count', 'lap_time', 'last_lap_time', 'best_lap_time')


def frame(collisions=0, x=0.0, speed=0.0):
    """FRAME plus the restricted fields on_frame reads with publish_restricted_topics true."""
    return dict(FRAME, **{'V1 Position': f'{x} 0 0', 'V1 Linear Velocity': f'{speed} 0 0',
                          'V1 Lap Count': '0', 'V1 Collisions': str(collisions), 'V1 Lap Time': '0',
                          'V1 Last Lap Time': '0', 'V1 Best Lap Time': '0'})


class Count:
    def __init__(self):
        self.n = 0

    def publish(self, msg):
        self.n += 1


class Record:
    def __init__(self):
        self.data = []

    def publish(self, msg):
        self.data.append(msg.data)


@pytest.fixture
def bridge():
    b = FastBridge()
    b.reset_pub = Record()
    yield b
    b.node.destroy_node()


@pytest.fixture
def restricted(bridge):
    """publish_restricted_topics on (conftest turns it off), its publishers stubbed."""
    bridge.publish_restricted_topics = True
    bridge.pub.update({k: Count() for k in RESTRICTED})
    return bridge


def test_stop_replies_zeros(bridge):
    bridge._throttle_cb(Float32(data=0.5))
    assert bridge._reply()['V1 Throttle'] == '0.5'
    bridge.stop()
    bridge._throttle_cb(Float32(data=0.5))   # a command after stop still is not sent
    assert bridge._reply()['V1 Throttle'] == '0.0'


def test_imu_held_after_reset(bridge):
    bridge.pub['imu'] = imu = Count()
    bridge._reset_cb(Bool(data=True))
    held = bridge.reset_frames + bridge.imu_reset_hold_frames
    resets = []
    for i in range(held + 1):
        resets.append(bridge.on_frame(dict(FRAME))['V1 Reset'])
        assert imu.n == (1 if i == held else 0)
    assert resets.count('True') == bridge.reset_frames


def test_reset_command_event_from_the_next_frame(bridge):
    bridge._throttle_cb(Float32(data=0.5))
    bridge._reset_cb(Bool(data=True))
    assert bridge.reset_pub.data == []   # the executor thread does not publish
    throttles = []
    for _ in range(bridge.reset_frames + 1):
        bridge._throttle_cb(Float32(data=0.5))
        throttles.append(bridge.on_frame(dict(FRAME))['V1 Throttle'])
    assert throttles == ['0.0'] * bridge.reset_frames + ['0.5']
    assert bridge.reset_pub.data == ['reset command']   # also without restricted topics


def test_respawn_zeroes_exactly_reset_frames_replies(restricted):
    b = restricted
    b._throttle_cb(Float32(data=0.5))
    assert b.on_frame(frame(collisions=0))['V1 Throttle'] == '0.5'   # first frame only initializes
    replies = []
    for _ in range(b.reset_frames + 2):
        b._throttle_cb(Float32(data=0.5))   # sent before the gate closed, or after the hold
        replies.append(b.on_frame(frame(collisions=1)))
    assert [r['V1 Throttle'] for r in replies] == ['0.0'] * b.reset_frames + ['0.5'] * 2
    assert all(r['V1 Reset'] == 'False' for r in replies)   # a respawn must not reset the car
    assert b.reset_pub.data == ['collision count 0 -> 1']


def test_respawn_hold_leaves_the_command_zeroed(restricted):
    b = restricted
    b.on_frame(frame(collisions=0))
    b._throttle_cb(Float32(data=0.5))
    replies = [b.on_frame(frame(collisions=1))['V1 Throttle'] for _ in range(b.reset_frames + 2)]
    assert replies == ['0.0'] * (b.reset_frames + 2)   # no new command: zero, cmd_timeout not reached


def test_event_before_the_frames_sensors(restricted):
    """The gate closes its relay on the event: it must come before the respawn frame's sensor messages."""
    b = restricted
    b.on_frame(frame(collisions=0))
    order = []

    class Shared:
        def __init__(self, name):
            self.name = name

        def publish(self, msg):
            order.append(self.name)
    b.reset_pub = Shared('sim_reset')
    b.pub = {k: Shared(k) for k in b.pub}
    b.on_frame(frame(collisions=1))
    assert order[0] == 'sim_reset' and order.count('sim_reset') == 1 and len(order) > 1


def test_event_topic_name(bridge):
    names = [pub.topic_name for pub in bridge.node.publishers]
    assert '/autodrive/roboracer_1/sim_reset' in names   # localization_gate subscribes to this name


def test_teleport_without_count_change(restricted):
    """A track switch or Scene Reset moves the car without a respawn (count unchanged at 0)."""
    b = restricted
    for x in (0.0, 0.05, 0.10, 0.15):
        b.on_frame(frame(x=x, speed=4.0))
    assert b.reset_pub.data == []
    b.on_frame(frame(x=6.95, speed=0.0))
    assert b.reset_pub.data == ['position jumped 6.8 m']


def test_no_respawn_event_without_restricted_topics(bridge):
    bridge._throttle_cb(Float32(data=0.5))
    for c in (0, 1):
        assert bridge.on_frame(frame(collisions=c))['V1 Throttle'] == '0.5'
    assert bridge.reset_pub.data == []


def test_jump_scaled_by_stamp_gap():
    prev = (0.0, 0.0, 0.0, 10.0)
    assert not is_jump(prev, (0.02, 0.2, 0.0, 10.0))     # one frame at 10 m/s
    assert not is_jump(prev, (0.3, 3.0, 0.0, 10.0))      # a sim hitch at speed
    assert is_jump(prev, (0.02, 1.3, 0.0, 0.0))          # short respawn at speed (speed zeroed)
    assert is_jump(prev, (0.02, 3.0, 0.0, 0.0))          # teleport
    assert is_jump((0.0, 0.0, 0.0, 0.0), (0.02, 1.5, 0.0, 0.0))


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_non_finite_commands_sent_as_zero(bridge, bad):
    # str(nan) would go to the sim as 'nan'
    bridge._throttle_cb(Float32(data=0.5))
    bridge._steering_cb(Float32(data=0.5))
    bridge._throttle_cb(Float32(data=bad))
    bridge._steering_cb(Float32(data=bad))
    reply = bridge._reply()
    assert reply['V1 Throttle'] == '0.0' and reply['V1 Steering'] == '0.0'
