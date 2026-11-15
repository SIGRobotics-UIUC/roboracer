"""Capture profiles and lightweight validity checks (no evaluation truth feedback)."""
import math
from ..analysis.core.signals import NS, pose_value

REQUESTED = '/benchmark/requested_drive'
BASE = {'/localized_pose': 'geometry_msgs/msg/PoseStamped', '/odom': 'nav_msgs/msg/Odometry',
        '/scan': 'sensor_msgs/msg/LaserScan', '/sensors/imu/raw': 'sensor_msgs/msg/Imu',
        '/clock': 'rosgraph_msgs/msg/Clock', REQUESTED: 'ackermann_msgs/msg/AckermannDriveStamped',
        '/drive': 'ackermann_msgs/msg/AckermannDriveStamped',
        NS+'lap_count': 'std_msgs/msg/Int32', NS+'collision_count': 'std_msgs/msg/Int32',
        NS+'last_lap_time': 'std_msgs/msg/Float32'}
EXTRA = {'/tf': 'tf2_msgs/msg/TFMessage', '/tf_static': 'tf2_msgs/msg/TFMessage',
         '/tracked_pose': 'geometry_msgs/msg/PoseStamped', '/ground_truth/odom': 'nav_msgs/msg/Odometry',
         '/raceline/provenance': 'std_msgs/msg/String', '/benchmark/events': 'std_msgs/msg/String',
         '/benchmark/status': 'std_msgs/msg/String', NS+'sim_reset': 'std_msgs/msg/String',
         '/benchmark/gate_status': 'std_msgs/msg/String',
         NS+'throttle_command': 'std_msgs/msg/Float32', NS+'steering_command': 'std_msgs/msg/Float32',
         NS+'throttle': 'std_msgs/msg/Float32', NS+'steering': 'std_msgs/msg/Float32'}
DETAILED = {NS+'left_encoder': 'sensor_msgs/msg/JointState', NS+'right_encoder': 'sensor_msgs/msg/JointState',
            '/map': 'nav_msgs/msg/OccupancyGrid', '/mcl/particles': 'geometry_msgs/msg/PoseArray',
            NS+'lap_time': 'std_msgs/msg/Float32', NS+'best_lap_time': 'std_msgs/msg/Float32',
            NS+'wheel_slip': 'std_msgs/msg/String'}
LATCHED = {'/tf_static', '/raceline/provenance', '/map'}


def profile(config):
    topics = {**BASE, **EXTRA}
    if config['logging']['level'] == 'detailed':
        topics.update(DETAILED)
    if config['logging']['require_ground_truth']:
        topics['/ground_truth/odom'] = 'nav_msgs/msg/Odometry'
    essential = set(BASE) | {NS+'throttle_command', NS+'steering_command'}
    if config['logging']['require_ground_truth']:
        essential.add('/ground_truth/odom')
    return topics, essential


def stamp_ns(message):
    stamp = message.header.stamp if hasattr(message, 'header') else message.clock if hasattr(message, 'clock') else None
    return stamp.sec*10**9+stamp.nanosec if stamp else None


def valid(topic, message):
    # Native AutoDRIVE sentinel before the first completed lap: stream is
    # present, but a numerical lap duration is not available yet.
    if topic == NS+'last_lap_time' and message.data == math.inf:
        return True
    if topic in ('/localized_pose', '/tracked_pose'):
        return message.header.frame_id == 'map' and pose_value(message.pose) is not None
    if topic in ('/odom', '/ground_truth/odom'):
        expected = 'odom' if topic == '/odom' else 'world'
        v = message.twist.twist
        return (message.header.frame_id == expected and pose_value(message.pose.pose) is not None
                and all(math.isfinite(x) for x in (v.linear.x, v.linear.y, v.angular.z))
                and (topic != '/odom' or message.child_frame_id == 'base_link'))
    if topic in ('/drive', REQUESTED):
        drive = message.drive
        return (message.header.frame_id in ('', 'base_link') and math.isfinite(drive.speed)
                and drive.speed >= 0 and math.isfinite(drive.steering_angle))
    if topic == '/sensors/imu/raw':
        return (message.header.frame_id == 'imu' and all(math.isfinite(x) for x in
                (message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z,
                 message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z)))
    if topic == '/scan':
        return (message.header.frame_id == 'laser' and bool(message.ranges) and message.angle_increment > 0
                and any(not math.isnan(value) and value > 0 for value in message.ranges))
    if hasattr(message, 'data'):
        return not isinstance(message.data, float) or math.isfinite(message.data)
    return True


def subscription_qos(node, topic):
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    endpoints = node.get_publishers_info_by_topic(topic)
    reliability = ReliabilityPolicy.RELIABLE
    if topic == '/clock' or any(e.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for e in endpoints):
        reliability = ReliabilityPolicy.BEST_EFFORT
    durability = DurabilityPolicy.TRANSIENT_LOCAL if topic in LATCHED else DurabilityPolicy.VOLATILE
    return QoSProfile(depth=100, reliability=reliability, durability=durability)
