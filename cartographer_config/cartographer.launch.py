import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', '/home/autodrive_devkit/cartographer_config',
                '-configuration_basename', 'autodrive_cartographer.lua'
            ],
            remappings=[
                ('scan', '/autodrive/roboracer_1/lidar')
            ]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser_broadcaster',
            arguments=['0.1', '0', '0.1', '0', '0', '0', 'roboracer_1', 'lidar']
        )
    ])
