"""Real OMPL rejection of reachable Cartesian target inside unchanged table."""
from pathlib import Path
import sys,os,json,time,hashlib
os.environ.setdefault('MUJOCO_GL','egl')
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3];BASE=ROOT/'validation/planner_comparison'
sys.path[:0]=[str(ROOT),str(BASE),str(BASE/'ompl')]
import numpy as np,mujoco
from PIL import Image
from common import Validity,PROTOCOL,pairs
from environment import FoldingBoxEnv
from planner import OMPLPlanner

def snapshot(e):
 return {'qpos':e.data.qpos.copy(),'qvel':e.data.qvel.copy(),'ctrl':e.data.ctrl.copy(),'time':float(e.data.time),'qcmd':{s:a['qcmd'].copy() for s,a in e.arms.items()},'targets':{s:v.copy() for s,v in e.targets.items()},'grippers':{s:{k:(v.copy() if isinstance(v,np.ndarray) else v) for k,v in g.items()} for s,g in e.grippers.items()},'env_step':e.env_step}
def serial(v):
 if isinstance(v,np.ndarray):return v.tolist()
 if isinstance(v,dict):return {k:serial(x) for k,x in v.items()}
 if isinstance(v,np.generic):return v.item()
 return v

def main():
 e=FoldingBoxEnv(placement='flat')
 for _ in range(25):e.step_joint_targets({'right':e.arms['right']['qcmd'].copy()})
 before=snapshot(e);Image.fromarray(e.render_camera('back')).save(HERE/'before.png')
 count={'step':0,'step_joint_targets':0}
 for name in count:
  original=getattr(e,name)
  def counted(*args,_name=name,_orig=original,**kw):count[_name]+=1;return _orig(*args,**kw)
  setattr(e,name,counted)
 target=np.array([-.32,-.065,.88,0.,1.,0.,0.]);seed=np.array(PROTOCOL['approach_q_right'])
 q,ik=e.inverse_kinematics('right',target,iterations=1000,seed=seed)
 valid=Validity(e);start=e.data.qpos[e.arms['right']['qad']].copy();assert valid(start),'Baseline collides'
 is_valid=valid(q);achieved=e.tcp_pose('right',valid.scratch).copy()
 contacts=[]
 for i,c,names in pairs(e.model,valid.scratch):
  if c.dist<0:contacts.append({'geoms':names,'penetration_m':-float(c.dist),'world_contact_position_m':c.pos.tolist(),'geom_ids':[int(c.geom1),int(c.geom2)]})
 assert ik['position_m']<.0002 and ik['orientation_rad']<.002,(target,ik)
 assert not is_valid and any('table_collision' in c['geoms'] for c in contacts),contacts
 adapter=OMPLPlanner(valid.limits,HERE,solve_seconds=2.)
 t=time.perf_counter();path,meta=adapter(start,q,valid,'inside_table');wall=time.perf_counter()-t
 ss=adapter._objects[-1][2]
 solution_any=bool(ss.haveSolutionPath());approx=bool(ss.haveApproximateSolutionPath()) if hasattr(ss,'haveApproximateSolutionPath') else None
 assert path is None and not meta['exact_solution'] and not solution_any,(meta,solution_any)
 assert 'Invalid goal' in meta['status'] or 'Invalid Goal' in meta['status'],meta['status']
 after=snapshot(e);unchanged={k:serial(before[k])==serial(after[k]) for k in before};assert all(unchanged.values()),unchanged
 assert sum(count.values())==0,count
 Image.fromarray(e.render_camera('back')).save(HERE/'rejected.png')
 report={'result':'pass','test':'Reachable Cartesian target inside table rejected by real OMPL; zero physical execution','requested_tcp_world_wxyz':target.tolist(),'achieved_scratch_tcp_world_wxyz':achieved.tolist(),'ik_residual':ik,'goal_q_right':q.tolist(),'goal_valid':is_valid,'blocking_contacts':contacts,'planner':meta,'planning_adapter_wall_seconds':wall,'have_any_solution_path':solution_any,'have_approximate_solution_path':approx,'returned_path_is_none':path is None,'executed_state_exactly_unchanged':unchanged,'motor_interface_calls_after_snapshot':count,'executed_time_before_s':before['time'],'executed_time_after_s':after['time'],'screenshots':'Both before.png and rejected.png show unchanged safe executed baseline, never scratch collision state.','mujoco_warnings':e.data.warning.number.tolist(),'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'environment.py',ROOT/'models/scene.xml',ROOT/'physics.json',ROOT/'robot_config.json',BASE/'common.py',BASE/'ompl/planner.py',HERE/'test.py']}}
 (HERE/'before_state.json').write_text(json.dumps(serial(before),indent=2));(HERE/'after_state.json').write_text(json.dumps(serial(after),indent=2));(HERE/'result.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True);e.close()
if __name__=='__main__':main()
