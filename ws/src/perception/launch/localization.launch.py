"""ros2 launch perception localization.launch.py track:=<name>

Cartographer pure localization against tracks/<name>/map.pbstream (frozen): TF map -> odom,
/tracked_pose, and /map for visualization. localization_gate starts the localization trajectory
(at the map origin: the car starts from the pose mapping started from, the sim reset pose),
restarts it after a sim reset, and relays /tracked_pose on /localized_pose once localized.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from perception.tracks import track_dir


def nodes(context):
    track = LaunchConfiguration('track').perform(context)
    pbstream = os.path.join(track_dir(track), 'map.pbstream')
    if not os.path.isfile(pbstream):
        raise RuntimeError(f'{pbstream} not found; map the track first (save_map {track})')

    cartographer = Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
                        output='screen',
                        arguments=['-configuration_directory',
                                   PathJoinSubstitution([FindPackageShare('perception'), 'config']),
                                   '-configuration_basename', 'localization.lua',
                                   '-load_state_filename', pbstream,
                                   '-load_frozen_state', 'true',
                                   # gflags bools take no separate value; localization_gate starts it
                                   '-start_trajectory_with_default_topics=false'],
                        remappings=[('imu', '/sensors/imu/raw')])

    grid = Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
                name='cartographer_occupancy_grid_node', output='screen',
                arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'])
    gate = Node(package='perception', executable='localization_gate', name='localization_gate',
                output='screen', parameters=[{'track': track}])
    return [cartographer, grid, gate]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        SetParameter(name='use_sim_time', value=ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)),
        DeclareLaunchArgument('track', description='track name, loads tracks/<track>/map.pbstream'),
        OpaqueFunction(function=nodes),
    ])
