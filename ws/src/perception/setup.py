import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua') + glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SIGRobotics',
    maintainer_email='todo@todo.todo',
    description='Cartographer mapping and localization, particle filter (MCL) localization, and saving maps to tracks/<name>/.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'save_map = perception.save_map:main',
            'map_origin = perception.map_origin:main',
            'corrector = perception.corrector:main',
            'localization_gate = perception.localization_gate:main',
            'mcl_node = perception.mcl_node:main',
        ],
    },
)
