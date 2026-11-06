"""Motor-only top-contact sliding; ends after retreat and clearance diagnostics."""
from pathlib import Path
import sys,json,time,gzip,csv,argparse,hashlib,math
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from PIL import Image
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'validation/planner_comparison'),str(ROOT/'validation/planner_comparison/ompl')]
from environment import FoldingBoxEnv
from common import Validity,dense,PROTOCOL
from planner import OMPLPlanner
ap=argparse.ArgumentParser();ap.add_argument('--load',type=float,default=4.);ap.add_argument('--distance',type=float,default=.075);ap.add_argument('--tag',default='trial2');args=ap.parse_args()
out=Path(__file__).resolve().parent/args.tag;out.mkdir(exist_ok=True)
e=FoldingBoxEnv(placement='flat');m=e.model;d=e.data;phase='settle';orig=mujoco.mj_step;started=time.monotonic();rows=[];peaks={};steps=0;last_sum=0.;subload=[];error=None;events=[]
name=[m.geom(i).name or '' for i in range(m.ngeom)];hand={i for i,n in enumerate(name) if n.startswith('jaw_pad_R_') or ('dexmate_R_gripper_' in n and '_collision_' in n)};card={i for i,n in enumerate(name) if n.startswith('card_')};table=m.geom('table_collision').id
fout=gzip.open(out/'substep_forces.csv.gz','wt',newline='');writer=csv.writer(fout);writer.writerow(['t','phase','hand_sum_normal_N','hand_peak_normal_N','hand_sum_tangent_N','card_penetration_m','robot_penetration_m','table_support_N'])
def vertices(g,dd=d):
 mi=m.geom_dataid[g];v=m.mesh_vert[m.mesh_vertadr[mi]:m.mesh_vertadr[mi]+m.mesh_vertnum[mi]];return v@dd.geom_xmat[g].reshape(3,3).T+dd.geom_xpos[g]
def snap(n):Image.fromarray(e.render_camera('back')).save(out/(n+'.png'))
def observed(model,data,*a,**kw):
 global steps,last_sum
 orig(model,data,*a,**kw);steps+=1;total=peak=tang=cp=rp=support=0.;forbidden=[]
 for i,c in enumerate(data.contact):
  ids=[int(c.geom1),int(c.geom2)];ns=[name[g] for g in ids];hascard=any(g in card for g in ids);robot=any(n.startswith(('dexmate_','jaw_pad_')) for n in ns)
  if not hascard and not robot:continue
  f=np.zeros(6);mujoco.mj_contactForce(model,data,i,f);fn=max(0,float(f[0]));pen=max(0.,-float(c.dist))
  if hascard:cp=max(cp,pen)
  if hascard and table in ids:support+=fn
  if robot:
   rp=max(rp,pen);intended=hascard and any(g in hand for g in ids)
   if intended:
    total+=fn;peak=max(peak,fn);tang+=float(np.linalg.norm(f[1:3]))
   elif fn>.1 or pen>.0001:forbidden.append({'geoms':ns,'normal_N':fn,'penetration_m':pen})
 p=peaks.setdefault(phase,{'hand_sum_normal_N':0.,'hand_peak_normal_N':0.,'hand_sum_tangent_N':0.,'card_penetration_m':0.,'robot_penetration_m':0.})
 for key,val in zip(p,[total,peak,tang,cp,rp]):p[key]=max(p[key],val)
 writer.writerow([data.time,phase,total,peak,tang,cp,rp,support]);last_sum=total;subload.append(total)
 if forbidden:events.extend(forbidden);raise RuntimeError('Forbidden robot collision '+str(forbidden[:2]))
 if total>5 or peak>5 or cp>.0005:raise RuntimeError('Force/penetration bound exceeded '+str((total,peak,cp)))
 if data.warning.number.any() or not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():raise RuntimeError('Warning/nonfinite')
def state():
 b=d.body('box_base');return {'time_s':float(d.time),'phase':phase,'tcp':e.tcp_pose('right').tolist(),'target':e.targets['right'].tolist(),'box_pose':d.joint('box_free').qpos.tolist(),'base_euler_xyz_deg':Rotation.from_matrix(b.xmat.reshape(3,3)).as_euler('xyz',degrees=True).tolist(),'hinges_deg':{n:float(np.rad2deg(d.joint('crease_'+n).qpos[0])) for n in e.panels if n!='base'},'hand_sum_normal_N':last_sum}
def tick(target=None,q=None):
 subload.clear()
 if q is not None:e.step_joint_targets({'right':q})
 elif target is not None:
  act={s:np.r_[e.targets[s],e.grippers[s]['command']] for s in e.arms};act['right']=np.r_[target,.3];e.step(act)
 else:e.step()
 rows.append(state());return float(np.mean(subload))
def stage(n):
 global phase
 phase=n;print(n,'sim',round(d.time,2),'wall',round(time.monotonic()-started,1),'base',np.round(d.body('box_base').xpos,5),'tcp',np.round(e.tcp_pose('right')[:3],5),'load',round(last_sum,3),flush=True)
def padgap():
 b=d.body('box_base');return min(float(((vertices(g)-b.xpos)@b.xmat.reshape(3,3))[:,2].min())-.002 for g in hand if name[g].startswith('jaw_pad'))
result={'requested_load_N':args.load,'requested_drag_m':args.distance,'source_sha256':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in ['environment.py','models/scene.xml','physics.json','robot_config.json','video_rebuild/config.json']},'method':'Real OMPL RRTConnect free-space approach; execution only existing env.step/step_joint_targets. Observational mj_step wrapper monitors every physical substep without modifying state, physics or contact. Motor-reference z adjustment uses measured contact sum to maintain top load. No grasp or fold commanded.'}
mujoco.mj_step=observed
try:
 for _ in range(25):tick()
 initial=state();result['initial']=initial;snap('before')
 stage('ompl_approach');goalpose=np.array([-.32,0,.98,0,1,0,0]);goal,ik=e.inverse_kinematics('right',goalpose,iterations=300,seed=np.array(PROTOCOL['approach_q_right']));valid=Validity(e)
 if not valid(goal):raise RuntimeError('Approach goal invalid')
 planner=OMPLPlanner(e.arms['right']['limits'],output_dir=out,solve_seconds=5);path,meta=planner(d.qpos[e.arms['right']['qad']].copy(),goal,valid,'approach');result['planner']=meta
 if path is None or not all(valid(q) for q in dense(path)):raise RuntimeError('OMPL path absent/invalid')
 for q in dense(path,.4*e.control_dt):tick(q=q)
 for _ in range(50):tick(q=goal)
 stage('near_surface');target=e.targets['right'].copy()
 for _ in range(500):
  if padgap()<.002:break
  target[2]-=.005*e.control_dt;tick(target)
 else:raise RuntimeError('No near-surface start')
 stage('slow_touch')
 for _ in range(600):
  target[2]-=.0005*e.control_dt;mean=tick(target)
  if last_sum>.05:break
 else:raise RuntimeError('No touch')
 result['first_touch']=state();contact_z=target[2];stage('load_settle')
 for _ in range(750):
  target[2]+=float(np.clip((mean-args.load)*5e-4*e.control_dt,-.00001,.00001));mean=tick(target)
  if abs(target[2]-contact_z)>.012:raise RuntimeError('Excessive load-control displacement')
 result['pre_drag']=state();result['actual_pre_drag_mean_normal_N']=mean;snap('loaded');stage('drag');x0=target[0];dragstart=state()
 for k in range(1,math.ceil(args.distance/(.005*e.control_dt))+1):
  target[0]=x0-min(args.distance,k*.005*e.control_dt);target[2]+=float(np.clip((mean-args.load)*5e-4*e.control_dt,-.00001,.00001));mean=tick(target)
  if abs(target[2]-contact_z)>.012:raise RuntimeError('Lost load/contact')
  if k%100==0:print('drag',k,'base_dx',d.body('box_base').xpos[0]-initial['box_pose'][0],'load',mean,'angle',rows[-1]['base_euler_xyz_deg'],flush=True)
 result['post_drag']=state();snap('after_drag');stage('unload');start=target.copy();end=start.copy();end[2]+=.025
 for k in range(1,151):tick(start+(end-start)*k/150)
 stage('release_settle')
 for _ in range(100):tick(end)
 result['final']=state();snap('after_release');result['completed_slide_protocol']=True
except Exception as exc:
 error=repr(exc);print('STOP',error,flush=True);result['stopped_state']=state();snap('stopped')
finally:
 mujoco.mj_step=orig;fout.close();np.savez_compressed(out/'final_state.npz',qpos=d.qpos.copy(),qvel=d.qvel.copy(),ctrl=d.ctrl.copy(),time=np.array(d.time));result.update({'error':error,'physics_substeps_observed':steps,'phase_peaks':peaks,'warnings':d.warning.number.tolist(),'forbidden_contact_events':events,'simulation_seconds':d.time,'wall_seconds':time.monotonic()-started})
 # Read-only postexecution geometry and candidate grasp collision checks.
 b=d.body('box_base');front=np.vstack([vertices(g) for g,n in enumerate(name) if n.startswith('card_body_front_')]);baseverts=vertices(m.geom('card_base_0').id)
 result['final_geometry']={'front_bounds_min_m':front.min(0).tolist(),'front_bounds_max_m':front.max(0).tolist(),'front_min_x_overhang_m':max(0.,-.55-float(front[:,0].min())),'base_bounds_min_m':baseverts.min(0).tolist(),'base_bounds_max_m':baseverts.max(0).tolist(),'base_vertices_inside_table_xy':bool(np.all((baseverts[:,0]>=-.55)&(baseverts[:,0]<=.25)&(abs(baseverts[:,1])<=.8))),'table_bounds_world_m':[[-.55,-.8,.86],[.25,.8,.92]]}
 support=[]
 for i,c in enumerate(d.contact):
  ids=[int(c.geom1),int(c.geom2)]
  if table in ids and any(g in card for g in ids):
   f=np.zeros(6);mujoco.mj_contactForce(m,d,i,f);support.append({'geoms':[name[g] for g in ids],'normal_N':float(f[0]),'distance_m':float(c.dist)})
 result['final_table_support']=support;result['final_hand_sum_normal_N']=last_sum
 result['scratch_grasp_candidates']=[]
 # Front-edge point is local [0,-.2,0]. Candidate TCP 30mm inward along local +y.
 for roll in [0,180]:
  world_edge=b.xpos+b.xmat.reshape(3,3)@np.array([0.,-.2,0.]);pos=world_edge+b.xmat.reshape(3,3)@np.array([0,.03,0.]);r=Rotation.from_euler('y',90,degrees=True)*Rotation.from_euler('z',roll,degrees=True);pose=np.r_[pos,r.as_quat()[[3,0,1,2]]]
  q,err=e.inverse_kinematics('right',pose,iterations=300,seed=np.array(PROTOCOL['approach_q_right']));sd=mujoco.MjData(m);sd.qpos[:]=d.qpos;sd.qpos[e.arms['right']['qad']]=q;mujoco.mj_forward(m,sd)
  # All vendor gripper collision meshes and custom pads, not only a single point.
  low=[]
  for g in hand:
   vv=vertices(g,sd);inside=(vv[:,0]>=-.55)&(vv[:,0]<=.25)&(abs(vv[:,1])<=.8)
   if inside.any():low.append({'geom':name[g],'min_vertex_height_above_table_m':float(vv[inside,2].min()-.92)})
  cs=[]
  for c in sd.contact:
   ids=[int(c.geom1),int(c.geom2)]
   if any(name[g].startswith(('dexmate_R_','jaw_pad_R')) for g in ids) and c.dist<-.00001:cs.append({'geoms':[name[g] for g in ids],'penetration_m':-float(c.dist)})
  result['scratch_grasp_candidates'].append({'roll_deg':roll,'target_pose':pose.tolist(),'ik_residual':err,'all_hand_geoms_vertices_over_table':low,'penetrating_contacts':cs,'note':'Scratch only. Vertex-plane tests complement physical collision detection; neither certifies a feasible full approach.'})
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');(out/'state_trace.json').write_text(json.dumps(rows)+'\n');e.close();print('RESULT',json.dumps({'error':error,'final_geometry':result['final_geometry'],'substeps':steps,'wall_seconds':result['wall_seconds']}),flush=True)
