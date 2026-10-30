"""Race a mapped track: Cartographer localization + pure pursuit on its raceline.

    ros2 launch control race.launch.py track:=<name> [speed_scale:=0.5] [max_speed:=0.0] [pose_topic:=/localized_pose]

Needs tracks/<name>/map.pbstream and raceline.csv (ws/scripts/map_track.sh <name>), and the car at
the pose mapping started from (ws/scripts/race.sh resets the sim car there). /localized_pose is
/tracked_pose once Cartographer has localized (localization_gate), so the car waits for that.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    def include(package, launch_file, args):
        return IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file]),
            launch_arguments={a: LaunchConfiguration(a) for a in args}.items())

    return LaunchDescription([
        DeclareLaunchArgument('track', description='track name under ws/tracks/'),
        DeclareLaunchArgument('pose_topic', default_value='/localized_pose',
                              description="a localizer's PoseStamped in the map frame"),
        DeclareLaunchArgument('speed_scale', default_value='0.5', description='drive speed = raceline vx * speed_scale'),
        DeclareLaunchArgument('max_speed', default_value='0.0', description='cap on the drive speed [m/s], 0 = no cap'),
        include('perception', 'localization.launch.py', ['track']),
        DeclareLaunchArgument('controller', default_value='pure_pursuit'),
        OpaqueFunction(function=lambda context: [include('control', {'pure_pursuit':'pure_pursuit.launch.py','map':'map.launch.py'}[LaunchConfiguration('controller').perform(context)], ['track', 'pose_topic', 'speed_scale', 'max_speed'])]),
    ])
