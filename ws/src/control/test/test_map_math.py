import json
import math
import os
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0,os.path.abspath(os.path.join(os.path.dirname(__file__),'..')))
from control.map_dynamics import derivative,steady_acceleration
from control.map_guidance import Trajectory,l1_acceleration
from control.map_lut import Lookup,generate

CONFIG=os.path.join(os.path.dirname(__file__),'..','config')
class MAPMath(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(CONFIG,'map_model.json')) as f: self.model=json.load(f)
    def test_l1_symmetry(self):
        self.assertAlmostEqual(l1_acceleration(2,.3,1),-l1_acceleration(2,-.3,1))
        self.assertAlmostEqual(l1_acceleration(2,0,1),0)
    def test_lut(self):
        lut=Lookup(os.path.join(CONFIG,'autodrive_map_lut.npz'),self.model)
        a,s=lut.columns[10]; speed=lut.speeds[10]
        self.assertAlmostEqual(lut.query(a[2],speed)[0],s[2])
        self.assertAlmostEqual(lut.query(-a[2],speed)[0],-s[2])
        self.assertTrue(lut.query(1e6,speed)[1])
        with self.assertRaises(ValueError): lut.query(0,lut.speeds[-1]+.1)
        with self.assertRaises(ValueError): Lookup(os.path.join(CONFIG,'autodrive_map_lut.npz'),dict(self.model,mass=4))
    def test_dynamics_symmetry(self):
        x=np.array([0.,0.,0.,2.,0.,0.])
        self.assertAlmostEqual(derivative(x,.1,2,self.model)[5],-derivative(x,-.1,2,self.model)[5])
        self.assertGreater(steady_acceleration(.1,2,self.model),0)
    def test_trajectory(self):
        t=np.linspace(0,2*np.pi,100,endpoint=False)
        xy=np.column_stack([2*np.cos(t),2*np.sin(t)])
        rows=np.column_stack([np.arange(100),xy,t,np.zeros(100),np.ones(100),np.zeros(100)])
        line=Trajectory(rows,max_segment=.2)
        target,d,eta,error=line.target(2,0,np.pi/2,.6,1)
        self.assertGreater(target,0); self.assertGreater(eta,0)
        rotated=Trajectory(np.column_stack([rows[:,0],-xy[:,1],xy[:,0],rows[:,3:]]),max_segment=.2)
        self.assertAlmostEqual(rotated.target(0,2,np.pi,.6,1)[2],eta)
        bad=rows.copy();bad[10,1]=100
        with self.assertRaises(ValueError): Trajectory(bad,max_segment=.2)
if __name__=='__main__': unittest.main()
