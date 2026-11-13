#!/usr/bin/env python3
"""Check documented lateral slip coordinates against steady-turn simulator telemetry.

Inputs are characterize_map.py CSV recordings. This estimates wheel-centre slip and
body lateral acceleration; neither is a direct WheelCollider slip or tire-force sample.
"""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from control.map_lut import Lookup
from tire_lap_report import Settings, wheel_lateral_slips


def summarize(path, lut, cfg):
    with open(path, newline='') as stream:
        rows = list(csv.DictReader(stream))
    points = []
    previous = None
    for row in rows:
        if previous is None:
            previous = row
            continue
        try:
            t, v, vy, yaw, steer = (float(row[k]) for k in
                ('time', 'gt_vx', 'gt_vy', 'gt_yaw_rate', 'steer_sensor'))
            t0, v0, vy0, steer0 = (float(previous[k]) for k in
                ('time', 'gt_vx', 'gt_vy', 'steer_sensor'))
            steer_age = float(row['steer_age'])
        except (KeyError, ValueError):
            previous = row
            continue
        previous = row
        dt = t - t0
        if not np.isfinite((dt, v, vy, yaw, steer, v0, vy0, steer0, steer_age)).all():
            continue
        if not (.015 < dt < .12 and lut.speeds[0] <= v <= lut.speeds[-1]
                and steer_age <= .12 and abs((v-v0)/dt) < 1.5
                and abs((vy-vy0)/dt) < 1. and abs((steer-steer0)/dt) < .35
                and abs(steer) >= .025):
            continue
        slips = wheel_lateral_slips(v, vy, yaw, steer, cfg)
        front_slip = max(abs(slips[f'front_{side}'][1]) for side in ('left', 'right'))
        rear_slip = max(abs(slips[f'rear_{side}'][1]) for side in ('left', 'right'))
        observed = (vy-vy0)/dt + v*yaw
        column = int(np.argmin(abs(lut.speeds-v)))
        accelerations, steering = lut.columns[column]
        if abs(steer) > steering[-1]:
            continue
        predicted = math.copysign(float(np.interp(abs(steer), steering, accelerations)), steer)
        kinematic = v*v*math.tan(steer)/cfg.wheelbase
        points.append((front_slip, rear_slip, abs(observed),
                       abs(observed-predicted), abs(observed-kinematic)))
    array = np.asarray(points)
    if not len(array):
        raise ValueError(f'No accepted samples in {path}')
    bands = []
    for low, high in ((0, .002), (.002, .004), (.004, .006), (.006, .008),
                      (.008, .01), (.01, .015), (.015, .03), (.03, float('inf'))):
        selected = array[(array[:, 0] >= low) & (array[:, 0] < high)]
        bands.append(dict(low=low, high=high if math.isfinite(high) else None,
                          samples=len(selected),
                          median_abs_lateral_acceleration_mps2=(float(np.median(selected[:, 2]))
                                                               if len(selected) else None)))
    return dict(source=str(path), accepted_samples=len(array),
                front_slip_p95=float(np.percentile(array[:, 0], 95)),
                front_slip_max=float(np.max(array[:, 0])),
                front_samples_at_or_above_guide_peak=int(np.sum(array[:, 0] >= .01)),
                observed_abs_lateral_acceleration_max_mps2=float(np.max(array[:, 2])),
                lut_median_abs_error_mps2=float(np.median(array[:, 3])),
                kinematic_median_abs_error_mps2=float(np.median(array[:, 4])),
                bands=bands)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recordings', nargs='+', type=Path)
    parser.add_argument('--model', type=Path, default=Path('src/control/config/map_model.json'))
    parser.add_argument('--lut', type=Path, default=Path('src/control/config/autodrive_map_lut.npz'))
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    model = json.loads(args.model.read_text())
    lut = Lookup(args.lut, model)
    cfg = Settings()
    result = dict(method='Steady-turn screen: 15-120 ms intervals; speed within LUT; '
                         '|ax|<1.5, |dsteer/dt|<0.35, |dvy/dt|<1; steering age <=0.12 s. '
                         'Slip is wheel-centre kinematic estimate, not internal simulator tire slip.',
                  guide_lateral_peak_coordinate=.01,
                  recordings=[summarize(path, lut, cfg) for path in args.recordings])
    output = json.dumps(result, indent=2) + '\n'
    if args.output:
        args.output.write_text(output)
    print(output, end='')


if __name__ == '__main__':
    main()
