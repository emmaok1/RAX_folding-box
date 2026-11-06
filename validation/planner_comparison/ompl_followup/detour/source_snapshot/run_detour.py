"""Fixed real-scene detour test. Bad direct interpolation is scratch-only."""
from pathlib import Path
import os,sys,time,json,hashlib
os.environ.setdefault('MUJOCO_GL','egl')
HERE=Path(__file__).resolve().parent; COMMON=HERE.parents[1]
sys.path[:0]=[str(COMMON),str(COMMON/'ompl')]
import numpy as np
import mujoco
from PIL import Image
from common import Smoke,Validity,dense,pairs,ROOT
from planner import OMPLPlanner
GOAL=np.array([-0.9106779543613794,0.1545691378980807,-1.5337635369644556,-0.009810463668761038,-1.0650163192264723,0.18247461324925096,-0.6942817177811355])

def main():
 out=HERE/'run';out.mkdir(exist_ok=True)
 report={'test':'existing-table obstacle detour','result':'fail','goal_right_rad':GOAL.tolist(),'baseline_executed':False,'planner':'OMPL2.0.1 RRTConnect','rng_seed':2026}
 with Smoke(out) as s:
  # Augment shared safety monitor with evidence counters without changing physics.
  real_step=s.orig_step
  metrics={'robot_contact_observations':0,'maximum_robot_penetration_m':0.,'maximum_robot_contact_normal_force_N':0.,'forbidden_contacts_over_shared_limits':0}
  def audited_step(m,d,*args,**kwargs):
   real_step(m,d,*args,**kwargs)
   for i,c,names in pairs(m,d):
    f=np.zeros(6);mujoco.mj_contactForce(m,d,i,f)
    metrics['robot_contact_observations']+=1
    metrics['maximum_robot_penetration_m']=max(metrics['maximum_robot_penetration_m'],-float(c.dist))
    metrics['maximum_robot_contact_normal_force_N']=max(metrics['maximum_robot_contact_normal_force_N'],float(f[0]))
    if c.dist<-.0001 or f[0]>.05:metrics['forbidden_contacts_over_shared_limits']+=1
  s.orig_step=audited_step
  try:
   s.reset();s.snap('before');e=s.env;valid=Validity(e)
   start=e.data.qpos[e.arms['right']['qad']].copy();report['start_right_rad']=start.tolist()
   assert valid(start) and valid(GOAL),'Endpoint invalid'
   goal_tcp=e.tcp_pose('right',valid.scratch).copy();report['goal_tcp_world_wxyz']=goal_tcp.tolist()
   baseline=[];first=None;deepest=None; t=time.perf_counter()
   samples=list(dense([start,GOAL],.0025))
   for index,q in enumerate(samples):
    okay=valid(q)
    if not okay:
     contacts=[{'geoms':names,'distance_m':float(c.dist)} for _,c,names in pairs(e.model,valid.scratch) if c.dist < -1e-6]
     entry={'index':index,'fraction':index/(len(samples)-1),'q':q.tolist(),'contacts':contacts}
     baseline.append(entry)
     if first is None:first=entry
     penetration=min(x['distance_m'] for x in contacts)
     if deepest is None or penetration < deepest['minimum_distance_m']:deepest=dict(entry,minimum_distance_m=penetration)
   assert baseline,'Direct path unexpectedly collision-free'
   report['baseline']={'samples':len(samples),'max_joint_spacing_rad':.0025,'collision_samples':len(baseline),'validation_s':time.perf_counter()-t,'first_collision':first,'deepest_collision':deepest}
   (out/'direct_path_collisions.json').write_text(json.dumps(baseline,indent=2)+'\n')
   valid(np.asarray(deepest['q']))
   renderer=mujoco.Renderer(e.model,480,640);renderer.update_scene(valid.scratch,camera='back');Image.fromarray(renderer.render()).save(out/'scratch_direct_collision_NOT_EXECUTED.png');renderer.close()
   adapter=OMPLPlanner(valid.limits,out,10.)
   def planner(start,goal,v,label):
    path,meta=adapter(start,goal,v,label)
    if path is not None:
     t=time.perf_counter();num=0
     for q in dense(path,.0025):
      num+=1
      if not v(q):raise RuntimeError('Fine independent validation failed before execution')
     meta['extra_independent_validation']={'samples':num,'max_joint_spacing_rad':.0025,'seconds':time.perf_counter()-t}
    return path,meta
   result=s.plan_execute(planner,GOAL,'detour');report['execution']=result
   if not result['success']:raise RuntimeError('OMPL did not find exact path')
   result['actual_tcp_error_m']=float(np.linalg.norm(e.tcp_pose('right')[:3]-goal_tcp[:3]))
   if result['actual_tcp_error_m']>.002:raise RuntimeError('Actual endpoint tracking error exceeds2mm')
   report['result']='pass'
  except Exception as exc:
   report['error']=repr(exc)
   try:s.snap('stopped')
   except Exception:pass
  finally:
   report.update({'metrics':metrics,'physics_substeps_observed':s.substeps,'warnings':s.env.data.warning.number.tolist(),'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [HERE/'run_detour.py',COMMON/'common.py',COMMON/'ompl/planner.py',ROOT/'environment.py',ROOT/'models/scene.xml',ROOT/'physics.json',ROOT/'robot_config.json']}})
   (out/'trace.json').write_text(json.dumps(s.trace)+'\n');(HERE/'result.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(report,indent=2),flush=True)
 return report
if __name__=='__main__':sys.exit(0 if main()['result']=='pass' else 1)
