"""Repeatable PF benchmark, using test_mcl's synthetic map, trajectory and noisy lidar.

From /root/ws after sourcing ROS:
PYTHONPATH=src/perception:$PYTHONPATH python3 src/perception/test/benchmark_mcl.py
Optional: --force-resample; --seed 1. Outputs JSON, milliseconds per update.
Sensor generation is excluded. Local scan-matcher fit is timed separately and
never fed to the filter. This is a tracking benchmark, not a recovery benchmark.
"""
import argparse
import json
import time

import numpy as np

from perception.mcl import ParticleFilter, ScanMatcher, relative, select_beams
from test_mcl import (ANGLE_INC, ANGLE_MIN, LASER, RANGE_MAX, lap, make_track,
                      pose_error, scan_from)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--force-resample', action='store_true')
    parser.add_argument('--seeds', type=int, nargs='+', help='run several paired seeds')
    parser.add_argument('--scenario', choices=['baseline', 'uncertain', 'all'], default='baseline')
    args = parser.parse_args()
    results = []
    for scenario in (['baseline', 'uncertain'] if args.scenario == 'all' else [args.scenario]):
        for seed in (args.seeds or [args.seed]):
            results.extend(run(seed, scenario, args.force_resample))
    print(json.dumps(results, indent=2))


def run(seed, scenario, force_resample):
    lmap, poses = make_track(), lap()
    rng = np.random.default_rng(seed)
    inputs = []
    for i in range(3, len(poses), 3):
        d = relative(poses[i - 3], poses[i])
        delta = (d[0] * 1.03 + rng.normal(0, .003), d[1] + rng.normal(0, .003),
                 d[2] + rng.normal(0, .003))
        r, a = select_beams(scan_from(lmap, poses[i], rng), ANGLE_MIN, ANGLE_INC, 60, .15, RANGE_MAX)
        inputs.append((poses[i], delta, r, a))
    results = []
    for adaptive in ((False, True) if seed % 2 else (True, False)):
        pf = ParticleFilter(seed=seed, adaptive_particles=adaptive,
            **(dict(noise_x=(.5, .1), noise_y=(.5, .1), noise_yaw=(.2, .3))
               if scenario == 'uncertain' else {}))
        pf.init_gaussian(poses[0], * ((.8, .5) if scenario == 'uncertain' else (.15, .1)))
        matcher = ScanMatcher(lmap)
        # Warm NumPy/map operations without modifying the measured filter or its RNG.
        warm = ParticleFilter(seed=seed, adaptive_particles=adaptive)
        warm.init_gaussian(poses[0], .15, .1)
        for _, delta, r, a in inputs[:5]:
            warm.predict(delta)
            warm.update_sensor(r, a, LASER, lmap)
            warm.resample()
        counts, errors, times, fit_times = [], [], [], []
        resamples = 0
        increases = 0
        previous_n = pf.n
        for true, delta, r, a in inputs:
            start = time.perf_counter()
            pf.predict(delta)
            pf.update_sensor(r, a, LASER, lmap)
            resamples += int(pf.resample(force=force_resample))
            estimate = pf.estimate()
            pf.spread()
            times.append(1000 * (time.perf_counter() - start))
            start = time.perf_counter()
            matcher.fit(estimate, r, a, LASER)
            fit_times.append(1000 * (time.perf_counter() - start))
            increases += int(pf.n > previous_n)
            previous_n = pf.n
            counts.append(pf.n)
            errors.append(pose_error(true, estimate))
        errors = np.asarray(errors)
        results.append(dict(mode='adaptive' if adaptive else 'fixed', seed=seed,
            scenario=scenario, force_resample=force_resample, updates=len(inputs), resamples=resamples, population_increases=increases,
            particles_min=min(counts), particles_mean=float(np.mean(counts)), particles_max=max(counts),
            position_mean_m=float(errors[:, 0].mean()), position_max_m=float(errors[:, 0].max()),
            yaw_mean_rad=float(errors[:, 1].mean()), pf_mean_ms=float(np.mean(times)),
            pf_p95_ms=float(np.percentile(times, 95)), pf_p99_ms=float(np.percentile(times, 99)), fit_mean_ms=float(np.mean(fit_times))))
    return results


if __name__ == '__main__':
    main()
