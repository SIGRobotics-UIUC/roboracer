"""Pure pursuit on a generated raceline:

    ros2 launch control pure_pursuit.launch.py track:=<name> [pose_topic:=/localized_pose] [speed_scale:=0.5]
        [max_speed:=0.0] [avoid:=false] [adaptive_lookahead:=true] [adaptive_lookahead_cut:=0.1]
        [adaptive_lookahead_min_time:=0.25]

Follows $TRACKS_DIR/<name>/raceline.csv (default ~/ws/tracks). Needs localization publishing pose_topic
(/localized_pose: perception's localization_gate or mcl_node, so the car waits until it is localized).
avoid:=true follows /local_path instead (planning's obstacle_spliner_node; race.launch.py starts it).
adaptive_lookahead:=true (the default) caps the lookahead by the curvature of the raceline near the car (so pure pursuit
cuts curves by at most about adaptive_lookahead_cut m; not below adaptive_lookahead_min_time x speed).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('track', description='track name under ws/tracks/'),
        DeclareLaunchArgument('pose_topic', default_value='/localized_pose',
                              description='geometry_msgs/PoseStamped in the map frame, once localized'),
        DeclareLaunchArgument('speed_scale', default_value='0.5',
                              description='drive speed = raceline vx * speed_scale'),
        DeclareLaunchArgument('max_speed', default_value='0.0',
                              description='cap on the drive speed [m/s], 0 = no cap'),
        DeclareLaunchArgument('avoid', default_value='false',
                              description='follow /local_path (obstacle avoidance) instead of the raceline'),
        DeclareLaunchArgument('adaptive_lookahead', default_value='true',
                              description='cap the lookahead by the curvature of the raceline'),
        DeclareLaunchArgument('adaptive_lookahead_cut', default_value='0.1',
                              description='with adaptive_lookahead: corner cut [m], lookahead <= sqrt(2 cut / kappa)'),
        DeclareLaunchArgument('adaptive_lookahead_min_time', default_value='0.25',
                              description='with adaptive_lookahead: but lookahead >= min_time x speed [s]'),
        Node(
            package='control',
            executable='pure_pursuit_node',
            output='screen',
            parameters=[PathJoinSubstitution([FindPackageShare('control'), 'config', 'vehicle.yaml']), {
                'track': ParameterValue(LaunchConfiguration('track'), value_type=str),
                'pose_topic': LaunchConfiguration('pose_topic'),
                'speed_scale': ParameterValue(LaunchConfiguration('speed_scale'), value_type=float),
                'max_speed': ParameterValue(LaunchConfiguration('max_speed'), value_type=float),
                'avoid': ParameterValue(LaunchConfiguration('avoid'), value_type=bool),
                'adaptive_lookahead': ParameterValue(LaunchConfiguration('adaptive_lookahead'), value_type=bool),
                'adaptive_lookahead_cut': ParameterValue(
                    LaunchConfiguration('adaptive_lookahead_cut'), value_type=float),
                'adaptive_lookahead_min_time': ParameterValue(
                    LaunchConfiguration('adaptive_lookahead_min_time'), value_type=float),
            }],
        ),
    ])
