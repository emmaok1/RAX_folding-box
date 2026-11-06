from pathlib import Path
import sys,json,time,csv,gzip,hashlib
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from PIL import Image
R=Path(__file__).resolve().parents[2];sys.path[:0]=[str(R),str(R/'validation/planner_comparison'),str(R/'validation/planner_comparison/ompl')]
from environment import FoldingBoxEnv
from common import Validity,dense
from planner import OMPLPlanner
O=Path(__file__).resolve().parent/'attempt1';O.mkdir(exist_ok=True)
e=FoldingBoxEnv(placement='flat');m=e.model;d=e.data;orig=mujoco.mj_step;phase='settle';steps=0;rows=[];peaks={};events=[];last=0.;error=None;startwall=time.monotonic();first=None
names=[m.geom(g).name or '' for g in range(m.ngeom)];hand={g for g,n in enumerate(names) if n.startswith('jaw_pad_R_') or ('dexmate_R_gripper_' in n and '_collision_' in n)};card={g for g,n in enumerate(names) if n.startswith('card_')};table=m.geom('table_collision').id
f=gzip.open(O/'substeps.csv.gz','wt',newline='');w=csv.writer(f);w.writerow(['time_s','phase','hand_normal_sum_N','hand_normal_peak_N','card_penetration_m','robot_penetration_m','table_support_N','box_force_x_N','box_force_y_N','box_force_z_N'])
def state():return {'time_s':float(d.time),'phase':phase,'tcp':e.tcp_pose('right').tolist(),'target':e.targets['right'].tolist(),'box_pose':d.joint('box_free').qpos.tolist(),'base_euler_deg':Rotation.from_matrix(d.body('box_base').xmat.reshape(3,3)).as_euler('xyz',degrees=True).tolist(),'hinges_deg':{n:float(np.rad2deg(d.joint('crease_'+n).qpos[0])) for n in e.panels if n!='base'},'normal_N':last}
def observe(mm,dd,*a,**kw):
 global steps,last,first
 orig(mm,dd,*a,**kw);steps+=1;total=peak=cp=rp=support=0.;force=np.zeros(3);forbid=[];contacts=[]
 for i,c in enumerate(dd.contact):
  ids=[int(c.geom1),int(c.geom2)];ns=[names[g] for g in ids];hascard=any(g in card for g in ids);robot=any(n.startswith(('dexmate_','jaw_pad_')) for n in ns)
  if not hascard and not robot:continue
  cf=np.zeros(6);mujoco.mj_contactForce(mm,dd,i,cf);fn=max(0.,float(cf[0]));pen=max(0.,-float(c.dist))
  if hascard:cp=max(cp,pen)
  if hascard and table in ids:support+=fn
  if robot:
   rp=max(rp,pen);intended=any(g in hand for g in ids) and any(n.startswith('card_side_right_') for n in ns)
   if intended:
    total+=fn;peak=max(peak,fn);world=c.frame.reshape(3,3).T@cf[:3];force+=world if ids[1] in card else -world
    if fn>.02:contacts.append({'geoms':ns,'normal_N':fn,'distance_m':float(c.dist),'position_world_m':c.pos.tolist()})
   elif fn>.1 or pen>.0001:forbid.append({'geoms':ns,'force_N':fn,'penetration_m':pen})
 p=peaks.setdefault(phase,{'sum_normal_N':0.,'per_contact_N':0.,'card_penetration_m':0.,'robot_penetration_m':0.})
 for k,v in zip(p,[total,peak,cp,rp]):p[k]=max(p[k],v)
 if first is None and contacts:first={'time_s':dd.time,'phase':phase,'contacts':contacts}
 if contacts and (len(events)<20 or total>=p['sum_normal_N']):events.append({'time_s':dd.time,'phase':phase,'contacts':contacts})
 last=total;w.writerow([dd.time,phase,total,peak,cp,rp,support,*force])
 if forbid:raise RuntimeError('Unexpected robot contact '+str(forbid))
 if total>5 or cp>.0005:raise RuntimeError('5N force/0.5mm penetration trip '+str((total,cp)))
 if dd.warning.number.any() or not np.isfinite(dd.qpos).all() or not np.isfinite(dd.qvel).all():raise RuntimeError('Warning/nonfinite')
def tick(target=None,q=None):
 if q is not None:e.step_joint_targets({'right':q})
 elif target is not None:
  action={s:np.r_[e.targets[s],e.grippers[s]['command']] for s in e.arms};action['right']=np.r_[target,.3];e.step(action)
 else:e.step()
 rows.append(state())
def snap(n):Image.fromarray(e.render_camera('back')).save(O/(n+'.png'))
def stage(n):
 global phase
 phase=n;print(n,'sim',round(d.time,3),'wall',round(time.monotonic()-startwall,1),'tcp',e.tcp_pose('right')[:3],'box',d.body('box_base').xpos,'F',last,flush=True)
result={'strategy':'45-degree tilted downward inward jaw approaches exposed side_right +X outer edge, then moves world-X at0.5mm/s for at most10mm after touch. No downward loading, grasp or fold.','edge_world_segment_m':[[-.2115,-.232,.922],[-.2115,-.2095,.922]],'source_sha256':{n:hashlib.sha256((R/n).read_bytes()).hexdigest() for n in ['environment.py','models/scene.xml','physics.json','robot_config.json','video_rebuild/config.json']},'trip_thresholds':{'hand_sum_normal_N':5,'card_penetration_m':.0005,'note':'Observation after each0.25ms step; exceeding sample can overshoot threshold, not force cap.'}}
mujoco.mj_step=observe
try:
 for _ in range(25):tick()
 result['initial']=state();snap('before');candidate=next(x for x in json.loads((O.parent/'tilted_diagnostics.json').read_text()) if x.get('yaw')==45 and x.get('y')==-.265);q=np.array(candidate['q']);result['scratch_candidate']=candidate
 stage('ompl_approach');valid=Validity(e);assert valid(q);planner=OMPLPlanner(e.arms['right']['limits'],O,solve_seconds=5);path,meta=planner(d.qpos[e.arms['right']['qad']].copy(),q,valid,'approach');result['planner']=meta
 if path is None:raise RuntimeError('No OMPL path')
 checks=list(dense(path));assert all(valid(j) for j in checks);result['independent_dense_validation_samples']=len(checks)
 for j in dense(path,.3*e.control_dt):tick(q=j)
 for _ in range(50):tick(q=q)
 result['precontact']=state();stage('slow_edge_touch');target=np.array(candidate['target']);x0=target[0]
 for k in range(1,1201):
  target[0]=x0-k*.0005*e.control_dt;tick(target)
  if first is not None:break
 else:raise RuntimeError('No side edge contact within12mm')
 snap('contact');result['touch']=state();result['first_contact']=first;stage('push10mm');x0=target[0]
 for k in range(1,1001):
  target[0]=x0-k*.0005*e.control_dt;tick(target)
  if k%200==0:print('push',k,'base_dx_mm',1000*(d.body('box_base').xpos[0]-result['initial']['box_pose'][0]),'F',last,flush=True)
  if d.body('box_base').xpos[0]-result['initial']['box_pose'][0]<=-.010:break
 result['after_push']=state();snap('after_push');stage('retreat');start=target.copy();end=start.copy();end[0]+=.005;end[2]+=.015
 for k in range(1,151):tick(start+(end-start)*k/150)
 stage('released_settle')
 for _ in range(100):tick(end)
 result['final']=state();snap('after_release');result['completed']=True
except Exception as exc:error=repr(exc);print('STOP',error,flush=True);result['stopped']=state();snap('stopped')
finally:
 mujoco.mj_step=orig;f.close();result.update({'error':error,'substeps':steps,'phase_peaks':peaks,'contact_events':events,'first_contact':first,'warnings':d.warning.number.tolist(),'final_hand_force_N':last,'simulation_s':d.time,'wall_s':time.monotonic()-startwall});np.savez_compressed(O/'final_state.npz',qpos=d.qpos.copy(),qvel=d.qvel.copy(),ctrl=d.ctrl.copy(),time=np.array(d.time));(O/'result.json').write_text(json.dumps(result,indent=2)+'\n')
 with gzip.open(O/'state_trace.json.gz','wt') as ff:json.dump(rows,ff,separators=(',',':'))
 e.close();print('DONE',error,'substeps',steps,flush=True)
