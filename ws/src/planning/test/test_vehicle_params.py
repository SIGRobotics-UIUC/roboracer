"""racecar.ini (read by TUM's optimizer, which cannot load ROS parameters) must match the car
geometry in control/config/vehicle.yaml, the one place it is set."""
import configparser
import json
import math
import os

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
VEHICLE = os.path.join(HERE, '..', '..', 'control', 'config', 'vehicle.yaml')
RACECAR = os.path.join(HERE, '..', 'config', 'racecar.ini')


def test_racecar_ini_matches_vehicle_yaml():
    with open(VEHICLE) as f:
        car = yaml.safe_load(f)['/**']['ros__parameters']
    ini = configparser.ConfigParser()
    ini.read(RACECAR)
    veh = json.loads(ini.get('GENERAL_OPTIONS', 'veh_params'))
    mintime = json.loads(ini.get('OPTIMIZATION_OPTIONS', 'vehicle_params_mintime'))
    assert mintime['delta_max'] == pytest.approx(car['max_steering_angle'])
    assert mintime['wheelbase_front'] + mintime['wheelbase_rear'] == pytest.approx(car['wheelbase'])
    assert veh['curvlim'] == pytest.approx(math.tan(car['max_steering_angle']) / car['wheelbase'], abs=0.01)


def test_width_opt_fits_the_car():
    ini = configparser.ConfigParser()
    ini.read(RACECAR)
    width = json.loads(ini.get('GENERAL_OPTIONS', 'veh_params'))['width']
    width_opt = json.loads(ini.get('OPTIMIZATION_OPTIONS', 'optim_opts_mintime'))['width_opt']
    assert width_opt >= width   # the car fits inside its own margin


def test_map_model_matches_planning():
    with open(os.path.join(HERE, '..', '..', 'control', 'config', 'map_model.json')) as f:
        model = json.load(f)
    ini = configparser.ConfigParser()
    ini.read(RACECAR)
    veh = json.loads(ini.get('GENERAL_OPTIONS', 'veh_params'))
    opt = json.loads(ini.get('OPTIMIZATION_OPTIONS', 'vehicle_params_mintime'))
    tire = json.loads(ini.get('OPTIMIZATION_OPTIONS', 'tire_params_mintime'))
    assert model['mass'] == pytest.approx(veh['mass'])
    for key, source in [('lf','wheelbase_front'), ('lr','wheelbase_rear'), ('inertia','I_z'), ('h_cg','cog_z')]:
        assert model[key] == pytest.approx(opt[source])
    for axle in ('front', 'rear'):
        for key in ('b','c','e'):
            assert model[f'{key}_{axle}'] == pytest.approx(tire[f'{key.upper()}_{axle}'])
