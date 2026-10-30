"""autodrive_launch.py without foxglove_bridge, for test_launch.py: foxglove_bridge would bind :8765.
Pass port:= a free port."""
import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    path = os.path.join(get_package_share_directory('autodrive_bridge'), 'launch', 'autodrive_launch.py')
    spec = importlib.util.spec_from_file_location('autodrive_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    keep = [e for e in module.generate_launch_description().entities
            if not (isinstance(e, Node) and e.node_executable == 'foxglove_bridge')]
    return LaunchDescription(keep)
