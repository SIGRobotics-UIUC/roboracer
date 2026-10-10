"""ros2 launch autodrive_bridge autodrive_launch.py

fast_bridge: AutoDRIVE app <-> ROS 2 over Socket.IO on :4567 (/autodrive/roboracer_1/*)
adapter:     sim sensors -> /scan, /car_state/odom, TF map -> base_link;  /drive -> throttle/steering
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    publish_restricted_topics = LaunchConfiguration('publish_restricted_topics')
    publish_camera = LaunchConfiguration('publish_camera')

    bridge = Node(package='autodrive_bridge', executable='fast_bridge', name='fast_bridge', output='screen', 
                  parameters=[{'publish_restricted_topics': publish_restricted_topics,'publish_camera': publish_camera,}])
    
    adapter = Node(package='autodrive_bridge', executable='adapter', name='adapter', output='screen',
                  parameters=[PathJoinSubstitution([FindPackageShare('autodrive_bridge'), 'config', 'adapter.yaml'])])
    
    foxglove = Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge')

    return LaunchDescription([
        DeclareLaunchArgument('publish_restricted_topics',default_value='true'),
        DeclareLaunchArgument('publish_camera',default_value='true'),
        bridge,
        adapter,
        foxglove,        
    ])
