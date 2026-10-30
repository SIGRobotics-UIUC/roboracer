"""Race a mapped track with localization and pure pursuit or MAP.

    ros2 launch control race.launch.py track:=<name> [speed_scale:=0.5] [max_speed:=0.0] [pose_topic:=/localized_pose]
        [controller:=pure_pursuit|map] [avoid:=false] [localizer:=cartographer|mcl]
        [adaptive_lookahead:=true] [adaptive_lookahead_cut:=0.1]
        [adaptive_lookahead_min_time:=0.25]

Needs tracks/<name>/map.pbstream (map.yaml for localizer:=mcl) and raceline.csv (ws/scripts/map_track.sh
<name>), and the car at the pose mapping started from (ws/scripts/race.sh resets the sim car there).
/localized_pose is published once the localizer has localized (Cartographer: localization_gate; mcl: the
particle cloud has converged), so the car waits for that. localizer:=mcl is the particle filter
(perception mcl.launch.py); the default is Cartographer pure localization.
avoid:=true also starts planning's obstacle_spliner_node on the track's raceline and map, and pure
pursuit follows its /local_path around obstacles the map did not have (stopping when there is none).
adaptive_lookahead:=true (the default) caps pure pursuit's lookahead by the curvature of the raceline ahead (see
pure_pursuit_node), for tight corners at full speed.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from perception.tracks import track_dir


def obstacle_spliner(context):
    if LaunchConfiguration('avoid').perform(context).lower() not in ('true', '1'):
        return []
    d = track_dir(LaunchConfiguration('track').perform(context))
    return [Node(package='planning', executable='obstacle_spliner_node', output='screen', parameters=[{
        'raceline_path': os.path.join(d, 'raceline.csv'),
        'map_yaml': os.path.join(d, 'map.yaml'),
        'pose_topic': LaunchConfiguration('pose_topic'),
    }])]


def controller_launch(context, include):
    controller = LaunchConfiguration('controller').perform(context)
    if controller == 'pure_pursuit':
        return [include('control', 'pure_pursuit.launch.py', [
            'track', 'pose_topic', 'speed_scale', 'max_speed', 'avoid',
            'adaptive_lookahead', 'adaptive_lookahead_cut', 'adaptive_lookahead_min_time'])]
    if controller == 'map':
        if LaunchConfiguration('avoid').perform(context).lower() in ('true', '1'):
            raise ValueError('MAP does not support avoid:=true')
        return [include('control', 'map.launch.py', ['track', 'pose_topic', 'speed_scale', 'max_speed'])]
    raise ValueError('controller must be pure_pursuit or map')


def generate_launch_description():
    def include(package, launch_file, args, condition=None):
        return IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file]),
            launch_arguments={a: LaunchConfiguration(a) for a in args}.items(), condition=condition)

    use_mcl = PythonExpression(["'", LaunchConfiguration('localizer'), "' == 'mcl'"])

    return LaunchDescription([
        DeclareLaunchArgument('track', description='track name under ws/tracks/'),
        DeclareLaunchArgument('pose_topic', default_value='/localized_pose',
                              description="a localizer's PoseStamped in the map frame"),
        DeclareLaunchArgument('speed_scale', default_value='0.5', description='drive speed = raceline vx * speed_scale'),
        DeclareLaunchArgument('max_speed', default_value='0.0', description='cap on the drive speed [m/s], 0 = no cap'),
        DeclareLaunchArgument('controller', default_value='pure_pursuit'),
        DeclareLaunchArgument('avoid', default_value='false',
                              description='avoid obstacles the map did not have (obstacle_spliner_node)'),
        DeclareLaunchArgument('localizer', default_value='cartographer',
                              description='cartographer: pure localization on map.pbstream; mcl: particle filter on map.yaml'),
        DeclareLaunchArgument('adaptive_lookahead', default_value='true',
                              description='cap the lookahead by the curvature of the raceline'),
        DeclareLaunchArgument('adaptive_lookahead_cut', default_value='0.1',
                              description='with adaptive_lookahead: corner cut [m], lookahead <= sqrt(2 cut / kappa)'),
        DeclareLaunchArgument('adaptive_lookahead_min_time', default_value='0.25',
                              description='with adaptive_lookahead: but lookahead >= min_time x speed [s]'),
        include('perception', 'localization.launch.py', ['track'], UnlessCondition(use_mcl)),
        include('perception', 'mcl.launch.py', ['track'], IfCondition(use_mcl)),
        OpaqueFunction(function=lambda context: controller_launch(context, include)),
        OpaqueFunction(function=obstacle_spliner),
    ])
