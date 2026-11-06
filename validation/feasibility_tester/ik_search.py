import sys,json
from pathlib import Path
sys.path.insert(0,str(Path.cwd()))
from environment import FoldingBoxEnv
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
E=FoldingBoxEnv(placement='flat');a=E.arms['right'];rng=np.random.default_rng(5);found=[]
for roll in [0,180]:
 r=Rotation.from_euler('y',90,degrees=True)*Rotation.from_euler('z',roll,degrees=True)
 p=np.r_[-.49,-.10,.9222,r.as_quat()[[3,0,1,2]]]
 for k in range(40):
  seed=a['qcmd'] if k==0 else rng.uniform(a['limits'][:,0],a['limits'][:,1])
  q,err=E.inverse_kinematics('right',p,iterations=120,seed=seed)
  if err['position_m']>.002 or err['orientation_rad']>.02:continue
  d=mujoco.MjData(E.model);d.qpos[:]=E.data.qpos;d.qpos[a['qad']]=q;mujoco.mj_forward(E.model,d)
  cs=[]
  for c in d.contact:
   names=[E.model.geom(int(g)).name or '' for g in [c.geom1,c.geom2]]
   if any('dexmate_R_' in n or 'jaw_pad_R' in n for n in names) and c.dist<-.0001:cs.append([names,float(c.dist)])
  if not cs:
   found.append({'pose':p.tolist(),'q':q.tolist(),'err':err,'roll':roll});print('FOUND',found[-1],flush=True);break
  if k<3:print('collided',roll,k,cs[:8],flush=True)
Path('validation/feasibility_tester/ik_candidates.json').write_text(json.dumps(found,indent=2))
