"""Shared CPU-planner smoke protocol. No core/physics edits; motor-only execution."""
from pathlib import Path
import sys, json, time, hashlib, math
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from environment import FoldingBoxEnv

PROTOCOL={
 'task':'Right-hand 5mm upward move, then reset and approach above base, gentle contact, retract.',
 'approach_tcp_world_wxyz':[-.32,-.065,.98,0,1,0,0],
 'approach_q_right':[.7064171109511613,-.2193711844649172,-2.174936533189925,-1.302777334538206,.9195548792273602,-.7994354424219224,1.373],
 'placement':'flat','settle_seconds':.5,'planning_time_limit_seconds':10,
 'path_validation_max_joint_increment_rad':.005,'playback_max_joint_speed_rad_s':.4,
 'final_settle_seconds':1.,'final_tcp_error_limit_m':.002,
 'contact_descent_speed_m_s':.005,'contact_descent_limit_m':.06,
 'contact_detection_normal_force_N':.01,'contact_abort_force_N':5.,
 'penetration_abort_m':.001,'forbidden_contact_penetration_tolerance_m':.0001,
 'planning_collision_tolerance_m':1e-6,
 'limits':'One shared deterministic start and goal per stage; this is an integration smoke, not a planner benchmark or folding proof.'}

def robot_name(n):return n.startswith(('dexmate_','jaw_pad_'))
def grasp_geom(n):return n.startswith('jaw_pad_R_') or ('dexmate_R_gripper_' in n and '_collision_' in n)
def pairs(m,d):
 for i,c in enumerate(d.contact):
  names=[m.geom(int(g)).name or '' for g in (c.geom1,c.geom2)]
  if any(robot_name(n) for n in names):yield i,c,names

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

class Smoke:
 def __init__(self,out):
  self.out=Path(out);self.out.mkdir(parents=True,exist_ok=True)
  self.env=FoldingBoxEnv(placement='flat');self.trace=[];self.stage='init';self.contact_mode=False
  self.orig_step=mujoco.mj_step;self.substeps=0;self.max_pen=0.;self.max_force=0.;self.contact_events=[]
 def __enter__(self):
  def observed_step(model,data,*args,**kwargs):
   self.orig_step(model,data,*args,**kwargs);self.substeps+=1
   for i,c,names in pairs(model,data):
    force=np.zeros(6);mujoco.mj_contactForce(model,data,i,force)
    intended=any(grasp_geom(n) for n in names) and any(n.startswith('card_base_') for n in names)
    if intended:
     self.max_pen=max(self.max_pen,-float(c.dist));self.max_force=max(self.max_force,float(force[0]))
     if force[0]>.01 and len(self.contact_events)<100:
      self.contact_events.append({'stage':self.stage,'time_s':float(data.time),'geoms':names,'normal_force_N':float(force[0]),'distance_m':float(c.dist)})
    if (not self.contact_mode or not intended) and (c.dist < -.0001 or force[0]>.05):
     raise RuntimeError('Forbidden physical contact: '+str((names,float(c.dist),float(force[0]))))
    if intended and (force[0]>5 or c.dist<-.001):raise RuntimeError('Contact force/penetration limit exceeded')
   if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or data.warning.number.any():raise RuntimeError('Nonfinite dynamics / MuJoCo warning')
  mujoco.mj_step=observed_step;return self
 def __exit__(self,*args):mujoco.mj_step=self.orig_step;self.env.close()
 def snap(self,name):
  from PIL import Image
  Image.fromarray(self.env.render_camera('back')).save(self.out/(name+'.png'))
 def tick(self,joints=None,action=None):
  e=self.env
  if joints is not None:e.step_joint_targets({'right':joints})
  else:e.step(action)
  self.trace.append({'stage':self.stage,'time_s':float(e.data.time),'right_q':e.data.qpos[e.arms['right']['qad']].tolist(),'tcp':e.tcp_pose('right').tolist(),'joint_target':e.arms['right']['qcmd'].tolist(),'box_pose':e.data.joint('box_free').qpos.tolist()})
 def settle(self,seconds):
  for _ in range(round(seconds/self.env.control_dt)):self.tick(joints=self.env.arms['right']['qcmd'].copy())
 def reset(self):
  self.env.reset();self.contact_mode=False;self.stage='reset_settle';self.settle(.5)
 def plan_execute(self,planner,goal,label):
  e=self.env;start=e.data.qpos[e.arms['right']['qad']].copy();valid=Validity(e)
  self.stage=label+'_plan'
  if not valid(start) or not valid(goal):raise RuntimeError('Shared start/goal collides')
  t=time.perf_counter();path,meta=planner(start.copy(),np.asarray(goal).copy(),valid,label);elapsed=time.perf_counter()-t
  result={'planner_metadata':meta,'callback_wall_seconds':elapsed,'validity_calls':valid.calls}
  if path is None:result['success']=False;return result
  path=np.asarray(path,dtype=float)
  if np.max(abs(path[0]-start))>1e-4 or np.max(abs(path[-1]-goal))>1e-4:raise RuntimeError('Returned path endpoints mismatch')
  t=time.perf_counter();count=0
  for q in dense(path):
   count+=1
   if not valid(q):raise RuntimeError('Independent MuJoCo dense validation failed')
  result.update({'path_waypoints':len(path),'dense_validation_samples':count,'dense_validation_seconds':time.perf_counter()-t,'path_joint_length_rad':float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum())})
  (self.out/(label+'_path.json')).write_text(json.dumps(path.tolist())+'\n')
  self.stage=label+'_execute';t=time.perf_counter()
  for q in dense(path,.4*e.control_dt):self.tick(joints=q)
  self.settle(1.)
  result.update({'success':True,'execution_wall_seconds':time.perf_counter()-t,'actual_final_q':e.data.qpos[e.arms['right']['qad']].tolist(),'actual_final_tcp':e.tcp_pose('right').tolist(),'final_max_joint_error_rad':float(np.max(abs(e.data.qpos[e.arms['right']['qad']]-goal)))})
  self.snap(label+'_after');return result
 def contact(self):
  e=self.env;self.contact_mode=True;self.stage='contact_descent';start=e.targets['right'].copy()
  # step_joint_targets maintained TCP references; now use existing Cartesian controller.
  found=False
  for k in range(1,601):
   target=start.copy();target[2]-=min(k*.005*e.control_dt,.06)
   action={s:np.r_[e.targets[s],e.grippers[s]['command']] for s in e.arms};action['right']=np.r_[target,.3]
   self.tick(action=action)
   if self.contact_events:found=True;break
  self.snap('contact')
  result={'confirmed':found,'first_contact':self.contact_events[0] if found else None,'tcp_at_stop':e.tcp_pose('right').tolist()}
  self.stage='retract';target=e.targets['right'].copy();end=target.copy();end[2]=max(1.02,target[2]+.05)
  for i in range(1,101):
   action={s:np.r_[e.targets[s],e.grippers[s]['command']] for s in e.arms};action['right']=np.r_[target+(end-target)*i/100,.3];self.tick(action=action)
  self.settle(.5);self.snap('retracted')
  remaining=[]
  for i,c,names in pairs(e.model,e.data):
   if any(n.startswith('card_') for n in names):
    f=np.zeros(6);mujoco.mj_contactForce(e.model,e.data,i,f)
    if f[0]>.01:remaining.append(names)
  result['released']=not remaining;return result

def run(planner,outdir,identity=None):
 report={'protocol':PROTOCOL,'identity':identity or {},'stages':{},'result':'not_completed'}
 with Smoke(outdir) as s:
  try:
   s.reset();s.snap('before');e=s.env;pose=e.tcp_pose('right').copy();pose[2]+=.005
   q,ik=e.inverse_kinematics('right',pose,iterations=300)
   before=e.tcp_pose('right').copy();r=s.plan_execute(planner,q,'upward');r['ik']=ik
   r['tcp_error_m']=float(np.linalg.norm(e.tcp_pose('right')[:3]-pose[:3]));r['measured_displacement_m']=(e.tcp_pose('right')[:3]-before[:3]).tolist();report['stages']['upward']=r
   if not r['success'] or r['tcp_error_m']>.002:raise RuntimeError('Upward planning/tracking failed')
   s.reset();r=s.plan_execute(planner,PROTOCOL['approach_q_right'],'approach');report['stages']['approach']=r
   r['tcp_error_m']=float(np.linalg.norm(e.tcp_pose('right')[:3]-np.array(PROTOCOL['approach_tcp_world_wxyz'][:3])))
   if not r['success'] or r['tcp_error_m']>.002:raise RuntimeError('Approach planning/tracking failed')
   c=s.contact();report['stages']['contact']=c
   report['result']='pass' if c['confirmed'] and c['released'] else 'fail'
  except Exception as exc:
   report['result']='fail';report['error']=repr(exc)
   try:s.snap('stopped')
   except Exception:pass
  finally:
   report.update({'physics_substeps_observed':s.substeps,'max_robot_base_penetration_m':s.max_pen,'peak_robot_base_normal_force_N':s.max_force,'warnings':s.env.data.warning.number.tolist(),'source_sha256':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in ['environment.py','models/scene.xml','physics.json','robot_config.json','video_rebuild/config.json']}})
   (s.out/'trace.json').write_text(json.dumps(s.trace)+'\n');(s.out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(report,indent=2),flush=True);return report

if __name__=='__main__':
 (Path(__file__).parent/'protocol.json').write_text(json.dumps(PROTOCOL,indent=2)+'\n')
