"""ros2 launch autodrive_bridge autodrive_launch.py [publish_restricted_topics:=true] [publish_camera:=false] [port:=4567]

fast_bridge: AutoDRIVE app <-> ROS 2 over Socket.IO on :4567 (/autodrive/roboracer_1/*)
adapter:     sim sensors -> /scan, /sensors/imu/raw, /ground_truth/odom;  /drive -> throttle/steering
wheel_odom:  rear-wheel encoders -> /odom + TF odom -> base_link (sim stand-in for vesc_to_odom)
foxglove_bridge on :8765

Teleop runs in its own terminal: ros2 run autodrive_bridge teleop_keyboard

Stop it with Ctrl-C in its terminal (or kill -INT -- -<its process group>): a SIGINT to the launch
process alone reaches no node, they get SIGTERM only 5 s later. When fast_bridge exits (it could not
bind the port: the old bridge still runs) the whole launch ends, rather than running a second
adapter and wheel_odom beside the old ones.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vehicle = PathJoinSubstitution([FindPackageShare('control'), 'config', 'vehicle.yaml'])
    config = PathJoinSubstitution([FindPackageShare('autodrive_bridge'), 'config', 'adapter.yaml'])

    return LaunchDescription([
        # restricted = sim ground truth and lap stats, off in competition
        DeclareLaunchArgument('publish_restricted_topics', default_value='true'),
        # nothing in the stack uses the camera, and decoding it every frame slows the sim loop
        DeclareLaunchArgument('publish_camera', default_value='false'),
        DeclareLaunchArgument('port', default_value='4567'),   # the sim's Connect port
        Node(package='autodrive_bridge', executable='fast_bridge', name='fast_bridge', output='screen',
             parameters=[{'publish_restricted_topics': LaunchConfiguration('publish_restricted_topics'),
                          'publish_camera': LaunchConfiguration('publish_camera'),
                          'port': LaunchConfiguration('port')}],
             on_exit=Shutdown(reason='fast_bridge exited')),
        Node(package='autodrive_bridge', executable='adapter', name='adapter', output='screen',
             parameters=[vehicle, config]),
        Node(package='autodrive_bridge', executable='wheel_odom', name='wheel_odom', output='screen',
             parameters=[vehicle, config]),
        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge'),
    ])
