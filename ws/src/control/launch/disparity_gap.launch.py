"""Follow the gap with disparity extension, for driving the mapping lap(s):

    ros2 launch control disparity_gap.launch.py [speed:=1.0] [laps:=0] [pose_topic:=/tracked_pose]

Needs only /scan. speed is a cap: the node slows below it when the road ahead is short or the turn is
tight. With laps > 0 it counts laps on pose_topic, stops the car after that many and exits (the launch
shuts down with it). scripts/map_track.sh --gap-follow runs this during mapping.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('speed', default_value='1.0', description='speed cap [m/s]'),
        DeclareLaunchArgument('laps', default_value='0', description='stop after this many laps, 0 = never'),
        DeclareLaunchArgument('pose_topic', default_value='/tracked_pose',
                              description='geometry_msgs/PoseStamped for lap counting'),
        Node(
            package='control',
            executable='disparity_gap_node',
            output='screen',
            emulate_tty=True,
            on_exit=Shutdown(),
            parameters=[PathJoinSubstitution([FindPackageShare('control'), 'config', 'vehicle.yaml']), {
                'max_speed': ParameterValue(LaunchConfiguration('speed'), value_type=float),
                'laps': ParameterValue(LaunchConfiguration('laps'), value_type=int),
                'pose_topic': LaunchConfiguration('pose_topic'),
            }],
        ),
    ])
