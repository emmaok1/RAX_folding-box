import sys,json
from pathlib import Path
sys.path.insert(0,str(Path.cwd()))
from environment import FoldingBoxEnv
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
E=FoldingBoxEnv(placement='flat');q=Rotation.from_matrix([[0,0,1],[0,-1,0],[1,0,0]]).as_quat()[[3,0,1,2]]
for z in [1.1,1.,.922]:
 for x in [-.62,-.56,-.49]:
  p=np.r_[x,-.07,z,q];j,err=E.inverse_kinematics('right',p,iterations=150)
  print(p,j,err,flush=True)
print('hinge',E.data.joint('crease_body_front').qpos, 'q',q)
for i in range(E.model.nbody):
 n=E.model.body(i).name
 if 'front' in n:print(n,E.data.body(i).xpos)
E.close()
