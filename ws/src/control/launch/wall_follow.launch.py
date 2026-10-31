"""Centring wall follower, for driving the mapping lap(s):

    ros2 launch control wall_follow.launch.py [speed:=1.0] [laps:=0] [pose_topic:=/tracked_pose]
        [track_width:=1.2] [debug:=false]

Needs only /scan. With laps > 0 it counts laps on pose_topic, stops the car after that many and exits
(the launch shuts down with it). scripts/map_track.sh --wall-follow runs this during mapping.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        SetParameter(name='use_sim_time', value=ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)),
        DeclareLaunchArgument('speed', default_value='1.0', description='speed on straights [m/s]'),
        DeclareLaunchArgument('laps', default_value='0', description='stop after this many laps, 0 = never'),
        DeclareLaunchArgument('pose_topic', default_value='/tracked_pose',
                              description='geometry_msgs/PoseStamped for lap counting'),
        DeclareLaunchArgument('track_width', default_value='1.2',
                              description='track width [m], used when only one wall is visible'),
        DeclareLaunchArgument('debug', default_value='false', description='log wall distances and steering'),
        Node(
            package='control',
            executable='wall_follow_node',
            output='screen',
            emulate_tty=True,
            on_exit=Shutdown(),
            parameters=[PathJoinSubstitution([FindPackageShare('control'), 'config', 'vehicle.yaml']), {
                'speed': ParameterValue(LaunchConfiguration('speed'), value_type=float),
                'laps': ParameterValue(LaunchConfiguration('laps'), value_type=int),
                'pose_topic': LaunchConfiguration('pose_topic'),
                'track_width': ParameterValue(LaunchConfiguration('track_width'), value_type=float),
                'debug': ParameterValue(LaunchConfiguration('debug'), value_type=bool),
            }],
        ),
    ])
