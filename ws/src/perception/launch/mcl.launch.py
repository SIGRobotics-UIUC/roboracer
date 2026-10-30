"""ros2 launch perception mcl.launch.py track:=<name>

Monte Carlo localization (particle filter) on tracks/<track>/map.yaml, instead of Cartographer:
TF map -> odom, /localized_pose (once the particle cloud has converged), /mcl/particles, /map.
Same outputs as localization.launch.py, so control race.launch.py works with either (localizer:=mcl).
Never run it together with localization.launch.py: both publish map -> odom.
Parameters: config/mcl.yaml; override one with  ros2 launch perception mcl.launch.py track:=x particles:=2000
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('track', description='track name, loads tracks/<track>/map.yaml'),
        DeclareLaunchArgument('particles', default_value='1000', description='number of particles'),
        Node(package='perception', executable='mcl_node', name='mcl', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('perception'), 'config', 'mcl.yaml']), {
                 'track': ParameterValue(LaunchConfiguration('track'), value_type=str),
                 'particles': ParameterValue(LaunchConfiguration('particles'), value_type=int),
             }]),
    ])
