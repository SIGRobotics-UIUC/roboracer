import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'autodrive_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SIGRobotics',
    maintainer_email='todo@todo.todo',
    description='AutoDRIVE RoboRacer simulator bridge: Socket.IO to ROS 2, plus an adapter to the stack topics.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'fast_bridge = autodrive_bridge.fast_bridge:main',
            'adapter = autodrive_bridge.adapter:main',
        ],
    },
)
