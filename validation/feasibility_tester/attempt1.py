"""Bounded physical grasp checkpoint. Run: MUJOCO_GL=egl .venv/bin/python validation/feasibility_tester/reproduce.py"""
import sys,json,time,hashlib
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation,Slerp
from PIL import Image
sys.path.insert(0,str(Path.cwd()))
from environment import FoldingBoxEnv
out=Path('validation/feasibility_tester');out.mkdir(exist_ok=True)
e=FoldingBoxEnv(placement='flat');rows=[];phase='settle';start_wall=time.time()
rot0=Rotation.from_matrix([[0,0,1],[0,-1,0],[1,0,0]])
def pose(x,y,z,rot=rot0):return np.r_[x,y,z,rot.as_quat()[[3,0,1,2]]]
def snap(name):Image.fromarray(e.render_camera('back')).save(out/(name+'.png'))
def step(action=None):
 e.step(action);cs=[];pen=0.
 for i,c in enumerate(e.data.contact):
  names=[e.model.geom(int(g)).name or '' for g in (c.geom1,c.geom2)]
  if not any(n.startswith('card_') for n in names):continue
  pen=max(pen,-float(c.dist));f=np.zeros(6);mujoco.mj_contactForce(e.model,e.data,i,f)
  if any('gripper' in n or 'jaw_pad' in n for n in names):cs.append({'geoms':names,'force_N':f[:3].tolist(),'distance_m':float(c.dist),'position':c.pos.tolist()})
 rows.append({'time_s':float(e.data.time),'phase':phase,'angle_deg':float(np.rad2deg(e.data.joint('crease_body_front').qpos[0])),'tcp':e.tcp_pose('right').tolist(),'gripper_qpos':e.data.qpos[e.grippers['right']['qad']].tolist(),'box_pose':e.data.joint('box_free').qpos.tolist(),'contacts':cs,'max_box_penetration_m':pen,'warning_counts':e.data.warning.number.tolist(),'ik':dict(e.ik_errors['right'])})
 if e.data.warning.number.any() or not np.isfinite(e.data.qpos).all():raise RuntimeError('physics warning/nonfinite')
 if pen>.003:raise RuntimeError('box penetration exceeds 3mm safety stop')
def move(name,end,seconds,opening=.2):
 global phase
 phase=name;start=e.targets['right'].copy();end=np.asarray(end);n=round(seconds/e.control_dt)
 rot=Slerp([0,1],Rotation.from_quat(np.array([start[3:7],end[3:7]])[:,[1,2,3,0]]))
 for k in range(1,n+1):
  u=k/n;action={s:np.r_[e.targets[s],e.grippers[s]['command']] for s in e.arms};action['right']=np.r_[start[:3]*(1-u)+end[:3]*u,rot([u]).as_quat()[0][[3,0,1,2]],opening];step(action)
 print(name,'sim',round(e.data.time,2),'wall',round(time.time()-start_wall,1),'angle',round(rows[-1]['angle_deg'],2),'tcp',np.round(e.tcp_pose('right')[:3],4),'contacts',[(c['geoms'],round(c['force_N'][0],3)) for c in rows[-1]['contacts']],'pen',rows[-1]['max_box_penetration_m'],flush=True)
 snap(name)
error=None
try:
 for _ in range(25):step()
 snap('before')
 move('approach',pose(-.62,-.07,1.08),4)
 move('lower',pose(-.62,-.07,.9222),2)
 move('insert',pose(-.475,-.07,.9222),2)
 move('close',e.targets['right'].copy(),1,0)
 pivot=np.array([-.43,-.07,.9242]);p0=e.targets['right'][:3].copy();theta=np.deg2rad(25)
 for k in range(1,11):
  r=Rotation.from_euler('y',theta*k/10);p=pivot+r.apply(p0-pivot);move('lift_%02d'%k,pose(*p,rot=r*rot0),.15,0)
 move('hold',e.targets['right'].copy(),1.2,0)
 move('release',e.targets['right'].copy(),.8,.3)
 p=e.targets['right'].copy();p[0]-=.1;p[2]+=.08;move('retract',p,1.5,.3)
 move('settle_after',e.targets['right'].copy(),1,.3)
except Exception as exc:
 error=repr(exc);print('STOP',error,flush=True);snap('stopped')
finally:
 def opposing(row):
  jaws=set()
  for c in row['contacts']:
   if c['force_N'][0]<.01 or not any(n.startswith('card_body_front') for n in c['geoms']):continue
   for n in c['geoms']:
    if n=='jaw_pad_R_1' or '_R_gripper_l1_collision_' in n:jaws.add(1)
    if n=='jaw_pad_R_2' or '_R_gripper_l2_collision_' in n:jaws.add(2)
  return jaws=={1,2}
 above=maxrun=0
 for row in rows:
  above=above+e.control_dt if row['angle_deg']>=15 and opposing(row) else 0;maxrun=max(maxrun,above)
 result={'error':error,'max_angle_deg':max(r['angle_deg'] for r in rows),'max_continuous_seconds_above15_with_opposing_contacts':maxrun,'opposing_contact_frames':sum(opposing(r) for r in rows),'max_box_penetration_m_at50Hz':max(r['max_box_penetration_m'] for r in rows),'warnings':e.data.warning.number.tolist(),'final_angle_deg':rows[-1]['angle_deg'],'final_contacts':rows[-1]['contacts'],'simulation_seconds':e.data.time,'wall_seconds':time.time()-start_wall,'scene_sha256':hashlib.sha256(Path('models/scene.xml').read_bytes()).hexdigest(),'method':'Execution exclusively FoldingBoxEnv.step motor references. IK scratch uses existing implementation. No executed state, geometry, physics, collision, force, equality changes. World table edge is x=-0.55m and front wall edge is x=-0.52m; there is no overhang. Initial planning assumption was incorrect. Trace sampled at50Hz.'}
 (out/'trace.json').write_text(json.dumps(rows)+'\n');(out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2),flush=True);e.close()
