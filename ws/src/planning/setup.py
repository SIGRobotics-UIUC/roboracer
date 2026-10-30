import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.ini')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SIGRobotics',
    maintainer_email='todo@todo.todo',
    description='Unseen-track planning: centerline from the Cartographer map + TUM minimum-time raceline.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'build_raceline = planning.build_raceline:main',
            'obstacle_spliner_node = planning.obstacle_spliner_node:main',
        ],
    },
)
