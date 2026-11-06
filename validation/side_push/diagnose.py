from pathlib import Path
import sys,json
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
R=Path(__file__).resolve().parents[2];sys.path[:0]=[str(R),str(R/'validation/planner_comparison')]
from environment import FoldingBoxEnv
from common import Validity,PROTOCOL
E=FoldingBoxEnv(placement='flat');m=E.model;d=E.data;a=E.arms['right'];hand=[i for i in range(m.ngeom) if (m.geom(i).name or '').startswith('jaw_pad_R_') or ('dexmate_R_gripper_' in (m.geom(i).name or '') and '_collision_' in (m.geom(i).name or ''))]
def vertices(g,sd):
 mi=m.geom_dataid[g];v=m.mesh_vert[m.mesh_vertadr[mi]:m.mesh_vertadr[mi]+m.mesh_vertnum[mi]];return v@sd.geom_xmat[g].reshape(3,3).T+sd.geom_xpos[g]
results=[]
for yaw in [30,45,60]:
 theta=np.deg2rad(yaw);rot=Rotation.from_matrix([[0,np.cos(theta),np.sin(theta)],[1,0,0],[0,np.sin(theta),-np.cos(theta)]])
 for y in [-.25,-.265,-.28]:
  target=np.r_[-.205,y,.95,rot.as_quat()[[3,0,1,2]]];q,err=E.inverse_kinematics('right',target,iterations=250,seed=np.array(PROTOCOL['approach_q_right']))
  if err['position_m']>.001:
   rng=np.random.default_rng(42)
   for trial in range(8):
    seed=rng.uniform(a['limits'][:,0]+.01,a['limits'][:,1]-.01);q2,err2=E.inverse_kinematics('right',target,iterations=180,seed=seed)
    if err2['position_m']+err2['orientation_rad']*.2<err['position_m']+err['orientation_rad']*.2:q,err=q2,err2
    if err['position_m']<.001 and err['orientation_rad']<.01:break
  if err['position_m']>.001:
   print('IKBLOCK',yaw,y,err,flush=True);results.append({'yaw':yaw,'y':y,'target':target.tolist(),'ik':err});continue
  sd=mujoco.MjData(m);sd.qpos[:]=d.qpos;sd.qpos[a['qad']]=q;mujoco.mj_forward(m,sd)
  minz=min(vertices(g,sd)[:,2].min() for g in hand);target[2]+=.922-minz
  q,err=E.inverse_kinematics('right',target,iterations=250,seed=q);sd.qpos[a['qad']]=q;mujoco.mj_forward(m,sd)
  cs=[]
  for c in sd.contact:
   names=[m.geom(int(g)).name or '' for g in [c.geom1,c.geom2]]
   if any(n.startswith(('dexmate_','jaw_pad_')) for n in names) and c.dist<-.00001:cs.append([names,float(c.dist)])
  low=[]
  for g in hand:
   v=vertices(g,sd);v=v[v[:,2]<.9243]
   if len(v):low.append({'geom':m.geom(g).name,'min':v.min(0).tolist(),'max':v.max(0).tolist()})
  row={'yaw':yaw,'y':y,'target':target.tolist(),'q':q.tolist(),'ik':err,'collisions':cs,'low_vertices':low};results.append(row)
  print('yaw',yaw,'y',y,'z',target[2],'ik',err,'collisions',cs,'low',low,flush=True)
Path('validation/side_push/tilted_diagnostics.json').write_text(json.dumps(results,indent=2));E.close()
