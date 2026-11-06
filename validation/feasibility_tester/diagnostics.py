"""Read-only model and scratch-pose collision diagnostics; never executes scratch states."""
import sys,json
from pathlib import Path
sys.path.insert(0,str(Path.cwd()))
from environment import FoldingBoxEnv
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
E=FoldingBoxEnv(placement='flat');m=E.model;d=E.data;a=E.arms['right'];g=m.geom('table_collision').id
result={'table_center_world_m':d.geom_xpos[g].tolist(),'table_half_extents_world_m':(abs(d.geom_xmat[g].reshape(3,3))@m.geom_size[g]).tolist(),'panels':{},'scratch_candidates':[]}
for name in ['base','body_front','side_right','side_left','body_back']:
 verts=[]
 for i in range(m.ngeom):
  if not (m.geom(i).name or '').startswith('card_'+name+'_'):continue
  mi=m.geom_dataid[i];v=m.mesh_vert[m.mesh_vertadr[mi]:m.mesh_vertadr[mi]+m.mesh_vertnum[mi]]
  verts.extend(v@d.geom_xmat[i].reshape(3,3).T+d.geom_xpos[i])
 verts=np.array(verts);result['panels'][name]={'bounds_min_world_m':verts.min(0).tolist(),'bounds_max_world_m':verts.max(0).tolist()}
for tilt in [0,20,40,60]:
 r=Rotation.from_euler('y',90+tilt,degrees=True);p=np.r_[-.49,-.10,.9222,r.as_quat()[[3,0,1,2]]]
 q,err=E.inverse_kinematics('right',p,iterations=200)
 sd=mujoco.MjData(m);sd.qpos[:]=d.qpos;sd.qpos[a['qad']]=q;mujoco.mj_forward(m,sd);cs=[]
 for c in sd.contact:
  names=[m.geom(int(gi)).name or '' for gi in [c.geom1,c.geom2]]
  if any('dexmate_R_' in n or 'jaw_pad_R' in n for n in names) and c.dist<-.0001:cs.append({'geoms':names,'penetration_m':-float(c.dist)})
 result['scratch_candidates'].append({'tilt_deg':tilt,'pose':p.tolist(),'ik_error':err,'penetrating_contacts':cs})
result['note']='Scratch states only; collision results are not physical penetration measurements. World geometry incorporates table body offset. Flat wall front is fully supported: outer x=−.52 vs table edge x=−.55.'
Path('validation/feasibility_tester/diagnostics.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='scratch_candidates'},indent=2))
for c in result['scratch_candidates']:print('tilt',c['tilt_deg'],'IK',c['ik_error'],'table_pen',max([z['penetration_m'] for z in c['penetrating_contacts'] if 'table_collision' in z['geoms']]+[0]),'contacts',len(c['penetrating_contacts']))
