"""MAP ROS input/output and race-launch selection, isolated from the live stack."""
import math
import os
import runpy
import tempfile
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from launch import LaunchContext

from control.map_node import MAP


def circle(path):
    angle = np.linspace(0, 2 * np.pi, 100, endpoint=False)
    xy = np.column_stack((2 * np.cos(angle), 2 * np.sin(angle)))
    rows = np.column_stack((np.arange(100), xy, angle + np.pi / 2,
                            np.zeros(100), np.full(100, 2.), np.zeros(100)))
    np.savetxt(path, rows, delimiter=';')


def test_map_ros_commands_stops_and_recovers():
    with tempfile.TemporaryDirectory() as directory:
        path = os.path.join(directory, 'raceline.csv')
        circle(path)
        rclpy.init(domain_id=42, args=['--ros-args', '-p', 'raceline_path:=' + path,
                                       '-p', 'pose_topic:=/map_test_pose',
                                       '-p', 'odom_topic:=/map_test_odom',
                                       '-p', 'wheelbase:=0.33',
                                       '-p', 'max_steering_angle:=0.524'])
        map_node = MAP()
        tester = rclpy.create_node('map_ros_test')
        executor = SingleThreadedExecutor()
        executor.add_node(map_node)
        executor.add_node(tester)
        poses = tester.create_publisher(PoseStamped, '/map_test_pose', 10)
        odoms = tester.create_publisher(Odometry, '/map_test_odom', 10)
        drives = []
        tester.create_subscription(AckermannDriveStamped, '/drive', drives.append, 10)

        def spin(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.01)

        def publish(frame='map', age=0., x=2., speed=1.):
            stamp = tester.get_clock().now().nanoseconds - int(age * 1e9)
            pose = PoseStamped()
            pose.header.frame_id = frame
            pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(stamp, 10**9)
            pose.pose.position.x = x
            pose.pose.orientation.z = math.sin(math.pi / 4)
            pose.pose.orientation.w = math.cos(math.pi / 4)
            odom = Odometry()
            odom.header.stamp = tester.get_clock().now().to_msg()
            odom.twist.twist.linear.x = speed
            poses.publish(pose)
            odoms.publish(odom)
            spin(.12)

        try:
            end = time.monotonic() + 5
            while (poses.get_subscription_count() == 0 or odoms.get_subscription_count() == 0) and time.monotonic() < end:
                spin(.05)
            assert poses.get_subscription_count() and odoms.get_subscription_count()
            spin(.1)
            assert drives and drives[-1].drive.speed == 0  # missing state
            publish()
            assert drives[-1].drive.speed == pytest.approx(1.)  # target 2 m/s at default 0.5 scale
            assert 0 < drives[-1].drive.steering_angle <= .524  # CCW target, rad
            spin(.65)
            assert drives[-1].drive.speed == 0 and drives[-1].drive.steering_angle == 0  # timeout
            publish(frame='world')
            assert drives[-1].drive.speed == 0  # invalid frame
            publish()
            assert drives[-1].drive.speed > 0  # fresh state recovers
            publish(age=.2)
            assert drives[-1].drive.speed == 0  # timestamp went backwards
            publish()
            assert drives[-1].drive.speed > 0  # progress reset and reacquired
            publish(x=10.)
            assert drives[-1].drive.speed == 0  # implausible pose jump
            publish()
            assert drives[-1].drive.speed > 0
        finally:
            executor.remove_node(tester)
            executor.remove_node(map_node)
            tester.destroy_node()
            map_node.destroy_node()
            executor.shutdown()
            rclpy.try_shutdown()


def test_race_launch_selects_one_controller_and_rejects_unsupported_modes():
    path = os.path.join(os.path.dirname(__file__), '..', 'launch', 'race.launch.py')
    choose = runpy.run_path(path)['controller_launch']
    context = LaunchContext()
    context.launch_configurations['avoid'] = 'false'
    include = lambda package, launch_file, args: launch_file
    context.launch_configurations['controller'] = 'map'
    assert choose(context, include) == ['map.launch.py']
    context.launch_configurations['controller'] = 'pure_pursuit'
    assert choose(context, include) == ['pure_pursuit.launch.py']
    context.launch_configurations['controller'] = 'map'
    context.launch_configurations['avoid'] = 'true'
    with pytest.raises(ValueError, match='avoid'):
        choose(context, include)
    context.launch_configurations['controller'] = 'unknown'
    with pytest.raises(ValueError, match='controller'):
        choose(context, include)
