"""Independent MuJoCo path validation and interpolation for the saved OMPL fixture."""
import math
import numpy as np
import mujoco
PROTOCOL = {"planning_collision_tolerance_m": 1e-6}

def robot_name(n):return n.startswith(('dexmate_','jaw_pad_'))

class Validity:
 def __init__(self,env):
  self.env=env;self.model=env.model;self.scratch=mujoco.MjData(env.model)
  self.snapshot=env.data.qpos.copy();self.indices=env.arms['right']['qad']
  self.limits=env.arms['right']['limits'].copy();self.calls=0
 def __call__(self,q):
  self.calls+=1;q=np.asarray(q)
  if q.shape!=(7,) or not np.isfinite(q).all() or np.any(q<self.limits[:,0]) or np.any(q>self.limits[:,1]):return False
  self.scratch.qpos[:]=self.snapshot;self.scratch.qpos[self.indices]=q
  mujoco.mj_forward(self.model,self.scratch)
  return not any(c.dist < -PROTOCOL['planning_collision_tolerance_m'] for _,c,_ in pairs(self.model,self.scratch))

def dense(path,max_step=.005):
 path=np.asarray(path,dtype=float)
 if path.ndim!=2 or path.shape[1]!=7 or len(path)<2 or not np.isfinite(path).all():raise ValueError('Planner must return a finite Nx7 path with >=2 points')
 yield path[0]
 for a,b in zip(path[:-1],path[1:]):
  n=max(1,math.ceil(np.max(abs(b-a))/max_step))
  for i in range(1,n+1):yield a+(b-a)*i/n


def pairs(m,d):
 for i,c in enumerate(d.contact):
  names=[m.geom(int(g)).name or '' for g in (c.geom1,c.geom2)]
  if any(robot_name(n) for n in names):yield i,c,names
