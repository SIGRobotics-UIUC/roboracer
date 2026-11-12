"""Diagnostic slip/friction monitor (does not publish /drive).

ros2 launch control slip_monitor.launch.py [pose_topic:=/localized_pose]
    [slip_linear_threshold:=0.75] [slip_angular_threshold:=0.75]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        SetParameter(name='use_sim_time', value=ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)),
        DeclareLaunchArgument('pose_topic', default_value='/localized_pose'),
        DeclareLaunchArgument('slip_linear_threshold', default_value='0.75'),
        DeclareLaunchArgument('slip_angular_threshold', default_value='0.75'),
        Node(package='control', executable='slip_monitor_node', name='slip_monitor', output='screen',
             parameters=[
                 PathJoinSubstitution([FindPackageShare('control'), 'config', 'vehicle.yaml']),
                 PathJoinSubstitution([FindPackageShare('control'), 'config', 'slip_monitor.yaml']),
                 {
                     'pose_topic': LaunchConfiguration('pose_topic'),
                     'linear_threshold': ParameterValue(
                         LaunchConfiguration('slip_linear_threshold'), value_type=float),
                     'angular_threshold': ParameterValue(
                         LaunchConfiguration('slip_angular_threshold'), value_type=float),
                 },
             ]),
    ])
