"""ros2 launch perception mapping.launch.py track:=<name>

Cartographer mapping on /scan + /odom: TF map -> odom, /tracked_pose, /map (occupancy grid).
Drive the lap(s) with teleop_keyboard, then save with: ros2 run perception save_map <name>
map_origin records the map origin in the sim world for map_to_world.yaml (does nothing on the car).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_dir = PathJoinSubstitution([FindPackageShare('perception'), 'config'])

    cartographer = Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
                        output='screen',
                        arguments=['-configuration_directory', config_dir,
                                   '-configuration_basename', 'mapping.lua'],
                        remappings=[('imu', '/sensors/imu/raw')])

    grid = Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
                name='cartographer_occupancy_grid_node', output='screen',
                arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'])

    map_origin = Node(package='perception', executable='map_origin', name='map_origin', output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        SetParameter(name='use_sim_time', value=ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)),
        DeclareLaunchArgument('track', description='track name; save with: ros2 run perception save_map <track>'),
        LogInfo(msg=['Mapping track "', LaunchConfiguration('track'),
                     '". When done: ros2 run perception save_map ', LaunchConfiguration('track')]),
        cartographer,
        grid,
        map_origin,
    ])
