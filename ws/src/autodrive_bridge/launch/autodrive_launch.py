"""ros2 launch autodrive_bridge autodrive_launch.py

fast_bridge: AutoDRIVE app <-> ROS 2 over Socket.IO on :4567 (/autodrive/roboracer_1/*)
adapter:     sim sensors -> /scan, /car_state/odom, TF map -> base_link;  /drive -> throttle/steering
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(package='autodrive_bridge', executable='fast_bridge', name='fast_bridge', output='screen'),
        Node(package='autodrive_bridge', executable='adapter', name='adapter', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('autodrive_bridge'), 'config', 'adapter.yaml'])]),
        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge'),
    ])
