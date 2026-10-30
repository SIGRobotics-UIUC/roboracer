"""Raceline acquisition and MAP L1 guidance, independent of ROS."""
import numpy as np

class Trajectory:
    def __init__(self, rows, max_segment=1.0, search_window=2., reacquire_distance=1.):
        rows=np.asarray(rows,dtype=float)
        if rows.ndim!=2 or rows.shape[1]!=7 or len(rows)<3 or not np.isfinite(rows).all() or np.any(rows[:,5]<0):
            raise ValueError('invalid seven-column raceline')
        if np.linalg.norm(rows[0,1:3]-rows[-1,1:3])<1e-5:
            rows=rows[:-1]
        self.rows=rows
        self.xy=rows[:,1:3]
        seg=np.roll(self.xy,-1,axis=0)-self.xy
        lengths=np.linalg.norm(seg,axis=1)
        if np.any(lengths<1e-6) or np.any(lengths>max_segment):
            raise ValueError('discontinuous raceline')
        self.heading=np.arctan2(seg[:,1],seg[:,0])
        self.window=max(1,int(np.ceil(search_window/np.median(lengths))))
        self.reacquire_distance=reacquire_distance
        self.closest=None

    def reset(self): self.closest=None

    def target(self,x,y,yaw,lookahead,max_offset):
        d=np.linalg.norm(self.xy-[x,y],axis=1)
        n=len(d)
        closest=None
        if self.closest is not None:
            near=(self.closest+np.arange(-self.window,self.window+1))%n
            i=near[np.argmin(d[near])]
            if d[i]<=self.reacquire_distance: closest=int(i)
        if closest is None:
            aligned=np.flatnonzero(np.cos(self.heading-yaw)>0)
            if not len(aligned): raise ValueError('no heading-aligned trajectory')
            closest=int(aligned[np.argmin(d[aligned])])
        if d[closest]>max_offset: raise ValueError('pose far from trajectory')
        self.closest=closest
        ahead=(closest+np.arange(n))%n
        far=np.flatnonzero(d[ahead]>=lookahead+d[closest])
        target=int(ahead[far[0]] if len(far) else ahead[np.argmax(d[ahead])])
        dx,dy=self.xy[target]-[x,y]
        eta=np.arctan2(-dx*np.sin(yaw)+dy*np.cos(yaw),dx*np.cos(yaw)+dy*np.sin(yaw))
        return target,float(np.hypot(dx,dy)),float(eta),float(d[closest])


def l1_acceleration(speed,eta,distance):
    if distance<=1e-6: raise ValueError('zero target distance')
    return 2*speed*speed*np.sin(eta)/distance
