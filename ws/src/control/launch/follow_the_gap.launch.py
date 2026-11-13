"""Follow the gap, the no-map fallback (reactive /scan -> /drive, no localization or raceline):

    ros2 launch control follow_the_gap.launch.py [speed:=1.0]

Needs only the bridge (or the car's drivers). Stop it with Ctrl-C before running race.sh: both publish /drive.
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
        DeclareLaunchArgument('speed', default_value='1.0', description='drive speed [m/s]'),
        Node(
            package='control',
            executable='follow_the_gap_node',
            output='screen',
            parameters=[PathJoinSubstitution([FindPackageShare('control'), 'config', 'vehicle.yaml']), {
                'speed': ParameterValue(LaunchConfiguration('speed'), value_type=float),
            }],
        ),
    ])
