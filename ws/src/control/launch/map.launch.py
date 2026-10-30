"""Standalone MAP controller on an existing raceline and localization."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    args=[DeclareLaunchArgument('track'),DeclareLaunchArgument('pose_topic',default_value='/tracked_pose'),DeclareLaunchArgument('speed_scale',default_value='0.5'),DeclareLaunchArgument('max_speed',default_value='0.0')]
    args.append(Node(package='control',executable='map_node',output='screen',parameters=[PathJoinSubstitution([FindPackageShare('control'),'config','vehicle.yaml']),{'track':ParameterValue(LaunchConfiguration('track'),value_type=str),'pose_topic':LaunchConfiguration('pose_topic'),'speed_scale':ParameterValue(LaunchConfiguration('speed_scale'),value_type=float),'max_speed':ParameterValue(LaunchConfiguration('max_speed'),value_type=float)}]))
    return LaunchDescription(args)
