"""GPL-3.0-or-later adaptation of ETH-PBL MAP steering lookup and generator.
Reference revision 5cce3319a063e7a70c25de5d1bad3b9b8d278ee6.
"""
import json
import numpy as np
from .map_dynamics import steady_acceleration

REVISION = '5cce3319a063e7a70c25de5d1bad3b9b8d278ee6'


def generate(path, parameters, speeds=None, steers=None, dt=0.01, duration=2.0):
    speeds = np.asarray(np.linspace(.9, 7., 65) if speeds is None else speeds, dtype=float)
    steers = np.asarray(np.r_[np.linspace(0, .1, 30), np.linspace(.1, .524, 30)[1:]] if steers is None else steers, dtype=float)
    values = np.full((len(steers), len(speeds)), np.nan)
    for j, speed in enumerate(speeds):
        for i, steer in enumerate(steers):
            value = steady_acceleration(steer, speed, parameters, dt, duration)
            if not np.isfinite(value):
                break
            if i and value <= values[i-1, j] + 1e-8:
                break
            values[i, j] = value
    meta = dict(schema=1, units={'speed':'m/s','steer':'rad','acceleration':'m/s^2'},
                dimensions=['steer','speed'], parameters=parameters, dt=dt,
                duration=duration, reference=REVISION)
    np.savez_compressed(path, speeds=speeds, steers=steers, acceleration=values,
                        metadata=np.array(json.dumps(meta)))


class Lookup:
    def __init__(self, path, parameters):
        with np.load(path, allow_pickle=False) as data:
            self.speeds = data['speeds']
            self.steers = data['steers']
            self.values = data['acceleration']
            meta = json.loads(str(data['metadata']))
        if (meta.get('schema') != 1 or meta.get('reference') != REVISION or
            meta.get('dimensions') != ['steer','speed'] or meta.get('parameters') != parameters or
            meta.get('units') != {'speed':'m/s','steer':'rad','acceleration':'m/s^2'}):
            raise ValueError('LUT metadata/model mismatch')
        if (self.values.shape != (len(self.steers),len(self.speeds)) or len(self.speeds)<2 or
            len(self.steers)<2 or not np.isfinite(self.speeds).all() or
            not np.isfinite(self.steers).all() or np.any(np.diff(self.speeds)<=0) or
            np.any(np.diff(self.steers)<=0) or self.steers[0]!=0 or
            np.any(np.isinf(self.values))):
            raise ValueError('malformed LUT axes or values')
        self.columns=[]
        for j in range(len(self.speeds)):
            col=self.values[:,j]
            valid=np.flatnonzero(np.isfinite(col))
            if len(valid)<2 or not np.array_equal(valid,np.arange(len(valid))):
                raise ValueError('insufficient or disjoint valid LUT region')
            a=col[valid]
            if abs(a[0])>1e-5 or np.any(np.diff(a)<=0):
                raise ValueError('nonmonotonic acceleration region')
            self.columns.append((a,self.steers[valid]))

    def query(self, acceleration, speed):
        if not np.isfinite([acceleration,speed]).all() or speed>self.speeds[-1]:
            raise ValueError('speed outside LUT or nonfinite query')
        j=int(np.argmin(abs(self.speeds-speed)))
        a,s=self.columns[j]
        requested=abs(acceleration)
        clamped=float(np.clip(requested,a[0],a[-1]))
        return float(np.sign(acceleration)*np.interp(clamped,a,s)), requested!=clamped, j
