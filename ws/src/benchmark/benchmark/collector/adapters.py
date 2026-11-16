"""Explicit managed mappings to existing nodes and localization launch files."""
from .topics import REQUESTED


def controller_parameters(config, vehicle, raceline):
    component = config['components']['controller']
    return {**vehicle, **component['params'], 'track': config['experiment']['map'],
            'raceline_path': str(raceline), 'pose_topic': '/localized_pose', 'use_sim_time': True}


def managed_commands(config, controller_file):
    localizer = config['components']['localization']
    local = ['ros2', 'launch', 'perception',
             'mcl.launch.py' if localizer['type'] == 'mcl' else 'localization.launch.py',
             'track:='+config['experiment']['map'], 'use_sim_time:=true']
    local += [f'{key}:={value}' for key, value in localizer['params'].items()]
    controller = ['ros2', 'run', 'control',
                  'map_node' if config['components']['controller']['type'] == 'map' else 'pure_pursuit_node',
                  '--ros-args', '--params-file', str(controller_file), '-r', '/drive:='+REQUESTED]
    return local, controller
