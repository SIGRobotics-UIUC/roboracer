"""GPL-3.0-or-later adaptation of ETH-PBL MAP-Controller vehicle_dynamics_stown.py.
Original dynamics: Matthias Althoff, TUM Cyber-Physical Systems Group (2020a).
Reference revision: 5cce3319a063e7a70c25de5d1bad3b9b8d278ee6.
"""
import math
import numpy as np


def derivative(x, steer, speed, p):
    # State [X,Y,yaw,vx,vy,yaw_rate]; fixed forward speed in LUT simulation.
    X, Y, yaw, vx, vy, rate = x
    lf, lr, mass, inertia = (p[k] for k in ('lf', 'lr', 'mass', 'inertia'))
    af = steer - math.atan2(vy + rate * lf, vx)
    ar = -math.atan2(vy - rate * lr, vx)
    fzf = mass * 9.81 * lr / (lf + lr)
    fzr = mass * 9.81 * lf / (lf + lr)
    def force(alpha, load, axle):
        if p['model'] == 'linear':
            return p['mu'] * load * p['c_' + axle + '_linear'] * alpha
        b, c, d, e = (p[f'{key}_{axle}'] for key in ('b', 'c', 'd', 'e'))
        z = b * alpha
        return p['mu'] * load * d * math.sin(c * math.atan(z - e * (z - math.atan(z))))
    ff, fr = force(af, fzf, 'front'), force(ar, fzr, 'rear')
    return np.array([vx * math.cos(yaw) - vy * math.sin(yaw),
                     vx * math.sin(yaw) + vy * math.cos(yaw), rate, 0.,
                     (ff + fr) / mass - vx * rate, (lf * ff - lr * fr) / inertia])


def steady_acceleration(steer, speed, p, dt=0.01, duration=2.0):
    x = np.array([0., 0., 0., speed, 0., 0.])
    rates = []
    for _ in range(int(duration / dt)):
        # RK4 substitutes for upstream odeint; same fixed steering/speed and convergence criterion.
        k1 = derivative(x, steer, speed, p)
        k2 = derivative(x + dt*k1/2, steer, speed, p)
        k3 = derivative(x + dt*k2/2, steer, speed, p)
        k4 = derivative(x + dt*k3, steer, speed, p)
        x += dt*(k1+2*k2+2*k3+k4)/6
        rates.append(x[5])
        if not np.isfinite(x).all():
            return float('nan')
    return speed * x[5] if abs(rates[-1]-rates[-10]) <= 0.05 else float('nan')
