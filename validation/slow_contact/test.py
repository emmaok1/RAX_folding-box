"""Matched speed-only contact test. Run from repo root with MUJOCO_GL=egl .venv/bin/python validation/slow_contact/test.py."""
from pathlib import Path
import sys, json, time, hashlib, csv, gzip, math
import numpy as np
import mujoco
from PIL import Image
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'validation/planner_comparison')]
from environment import FoldingBoxEnv
from common import dense, Validity, robot_name
OUT=Path(__file__).resolve().parent
SOURCES=['environment.py','models/scene.xml','physics.json','robot_config.json','video_rebuild/config.json']
PATH_FILE=ROOT/'validation/planner_comparison/ompl/run/approach_path.json'

class StopTest(RuntimeError):pass

def trial(speed):
 out=OUT/('speed_'+str(speed).replace('.','p')+'_mm_s');out.mkdir(exist_ok=True)
 e=FoldingBoxEnv(placement='flat');orig=mujoco.mj_step;phase='setup';substeps=0
 peaks={};first=None;observations=[];motor_trace=[];hard_stop=None
 hand_ids=[i for i in range(e.model.ngeom) if (e.model.geom(i).name or '').startswith('jaw_pad_R_') or ('dexmate_R_gripper_' in (e.model.geom(i).name or '') and '_collision_' in (e.model.geom(i).name or ''))]
 base_id=e.model.geom('card_base_0').id
 # Near this horizontal base, use pad mesh vertex heights in the base frame.
 # Vendor collision meshes occasionally return spurious zero geomDistance;
 # this analytic plane clearance avoids that diagnostic query artifact.
 pad_ids=[g for g in hand_ids if e.model.geom(g).name.startswith('jaw_pad_R_')]
 def clearance():
  base=e.data.body('box_base');rot=base.xmat.reshape(3,3);values=[]
  for g in pad_ids:
   mesh=int(e.model.geom_dataid[g]);a=int(e.model.mesh_vertadr[mesh]);n=int(e.model.mesh_vertnum[mesh])
   world=e.model.mesh_vert[a:a+n]@e.data.geom_xmat[g].reshape(3,3).T+e.data.geom_xpos[g]
   local=(world-base.xpos)@rot
   assert np.all(np.abs(local[:,0])<.15) and np.all(np.abs(local[:,1])<.11), 'Pad footprint outside base'
   values.append(float(local[:,2].min())-e.physical_params['thickness_m']/2)
  return min(values)
 def observe(model,data,*args,**kwargs):
  nonlocal first,substeps
  orig(model,data,*args,**kwargs);substeps+=1;total=peak=penetration=0.;min_dist=None;pair=None
  for i,c in enumerate(data.contact):
   ids=[int(c.geom1),int(c.geom2)];names=[model.geom(g).name or '' for g in ids]
   if not any(robot_name(n) for n in names):continue
   f=np.zeros(6);mujoco.mj_contactForce(model,data,i,f)
   intended=base_id in ids and any(g in hand_ids for g in ids)
   if not intended and (c.dist<-.0001 or f[0]>.05):raise StopTest('Unexpected robot collision '+str((names,c.dist,f[0])))
   if intended:
    total+=max(0.,float(f[0]));penetration=max(penetration,-float(c.dist));min_dist=float(c.dist) if min_dist is None else min(min_dist,float(c.dist))
    if f[0]>peak:peak=float(f[0]);pair=names
  p=peaks.setdefault(phase,{'per_contact_normal_N':0.,'sum_normal_N':0.,'penetration_m':0.})
  p['per_contact_normal_N']=max(p['per_contact_normal_N'],peak);p['sum_normal_N']=max(p['sum_normal_N'],total);p['penetration_m']=max(p['penetration_m'],penetration)
  if phase=='trial_descent' and peak>.01 and first is None:first={'time_s':float(data.time),'normal_force_N':peak,'sum_normal_N':total,'distance_m':min_dist,'geoms':pair}
  if phase in ('trial_descent','retract','release_settle'):observations.append([float(data.time),phase,peak,total,penetration,min_dist])
  if peak>5.:raise StopTest('5N per-contact force limit reached')
  if penetration>.001:raise StopTest('1mm hand/base penetration limit reached')
  if data.warning.number.any() or not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():raise StopTest('MuJoCo warning or nonfinite dynamics')
 def tick(action=None,joints=None):
  if joints is None:e.step(action)
  else:e.step_joint_targets({'right':joints})
  if phase in ('trial_descent','retract','release_settle'):motor_trace.append({'time_s':float(e.data.time),'phase':phase,'right_tcp':e.tcp_pose('right').tolist(),'target':e.targets['right'].tolist(),'qpos':e.data.qpos[e.arms['right']['qad']].tolist()})
 def command(target):
  action={s:np.r_[e.targets[s],e.grippers[s]['command']] for s in e.arms};action['right']=np.r_[target,.3];tick(action=action)
 result={'speed_mm_s':speed,'increment_mm_per_20ms':speed*.02,'source_sha256':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in SOURCES}}
 mujoco.mj_step=observe;started=time.monotonic()
 try:
  for _ in range(25):tick()
  path=json.loads(PATH_FILE.read_text());valid=Validity(e)
  assert all(valid(q) for q in dense(path)), 'Stored OMPL approach no longer valid'
  phase='ompl_path_replay'
  for q in dense(path,.4*e.control_dt):tick(joints=q)
  for _ in range(50):tick(joints=e.arms['right']['qcmd'].copy())
  phase='common_prestage';target=e.targets['right'].copy()
  for _ in range(500):
   if clearance()<=.002:break
   target[2]-=.005*e.control_dt;command(target.copy())
  else:raise StopTest('Unable to reach common near-surface start')
  phase='common_start_settle'
  for _ in range(50):command(target.copy())
  gap=clearance();assert .0005<gap<.004,('Unsafe or inconsistent initial clearance',gap)
  assert not any(p['per_contact_normal_N']>.01 for p in peaks.values()),'Prestage touched box'
  result.update({'start_clearance_m':gap,'clearance_method':'Minimum actual jaw-pad mesh vertex height above base top plane; footprint verified inside base','start_tcp':e.tcp_pose('right').tolist(),'start_time_s':float(e.data.time),'start_executed_state_sha256':hashlib.sha256(e.data.qpos.tobytes()+e.data.qvel.tobytes()+e.data.ctrl.tobytes()).hexdigest(),'start_target':target.tolist()})
  Image.fromarray(e.render_camera('back')).save(out/'before.png')
  phase='trial_descent';z0=target[2];start_time=float(e.data.time)
  for k in range(1,math.ceil(.006/(speed*.001*e.control_dt))+1):
   target[2]=z0-k*speed*.001*e.control_dt;command(target.copy())
   if first is not None:break
  if first is None:raise StopTest('No contact within6mm descent')
  result.update({'first_contact':first,'reaction_time_after_first_contact_s':float(e.data.time)-first['time_s'],'descent_sim_seconds':float(e.data.time)-start_time,'stop_tcp':e.tcp_pose('right').tolist()})
  Image.fromarray(e.render_camera('back')).save(out/'contact.png')
  phase='retract';start=target.copy();end=start.copy();end[2]+=.01
  for k in range(1,51):command(start+(end-start)*k/50)
  phase='release_settle'
  for _ in range(25):command(end.copy())
  released=not any(r[2]>.01 for r in observations[-80:])
  result.update({'contact_confirmed':True,'released':released,'result':'pass' if released else 'fail'})
  Image.fromarray(e.render_camera('back')).save(out/'retracted.png')
 except Exception as exc:
  hard_stop=repr(exc);result.update({'result':'fail','error':hard_stop,'contact_confirmed':first is not None,'first_contact':first})
 finally:
  mujoco.mj_step=orig
  result.update({'peaks_by_phase':peaks,'peak_contact_force_N':max((p['per_contact_normal_N'] for p in peaks.values()),default=0.),'peak_sum_normal_N':max((p['sum_normal_N'] for p in peaks.values()),default=0.),'max_hand_base_penetration_m':max((p['penetration_m'] for p in peaks.values()),default=0.),'physics_substeps_observed':substeps,'warnings':e.data.warning.number.tolist(),'wall_seconds':time.monotonic()-started,'simulation_time_s':float(e.data.time)})
  (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
  (out/'motor_trace.json').write_text(json.dumps(motor_trace)+'\n')
  with gzip.open(out/'force_trace.csv.gz','wt',newline='') as f:
   w=csv.writer(f);w.writerow(['time_s','phase','max_contact_normal_N','sum_contact_normal_N','penetration_m','min_contact_distance_m']);w.writerows(observations)
  e.close()
 print(json.dumps(result,indent=2),flush=True)
 return result

if __name__=='__main__':
 results=[trial(speed) for speed in [5.,1.,.5]]
 summary={'method':'Same motor-only OMPL-path replay, common near-surface prestage and settled executed start. Only final descent speed/increment varies. Same20ms command updates/first-touch detection rule and retreat; forces monitored every0.25ms physics substep. No gains, physics, geometry, planner library, or force-response logic changed. Single deterministic trial at each speed.','baseline_force_limit_N':5.,'results':results,'start_states_identical':len({r.get('start_executed_state_sha256') for r in results})==1 and all('start_executed_state_sha256' in r for r in results),'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
 (OUT/'comparison.json').write_text(json.dumps(summary,indent=2)+'\n')
 print('SUMMARY',json.dumps({str(r['speed_mm_s']):r['peak_contact_force_N'] for r in results}),flush=True)
