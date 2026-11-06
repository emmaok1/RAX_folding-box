"""Bounded deterministic scratch-only candidate selection; never executes bad path."""
from pathlib import Path
import os,sys,time,json
os.environ.setdefault('MUJOCO_GL','egl')
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]))
from common import Smoke,Validity,dense,pairs
import numpy as np
started=time.perf_counter(); rng=np.random.default_rng(20261103); found=[];stats={'tried':0,'valid_endpoints':0}
with Smoke(HERE/'search') as s:
 s.reset();e=s.env;v=Validity(e);start=e.data.qpos[e.arms['right']['qad']].copy()
 for k in range(300):
  if time.perf_counter()-started>180:break
  # Full bounded joint-space endpoint sample; scratch data only.
  goal=rng.uniform(v.limits[:,0]+.08,v.limits[:,1]-.08)
  if np.linalg.norm(goal-start)>5:continue
  stats['tried']+=1
  if not v(goal):continue
  stats['valid_endpoints']+=1
  blockers=[]
  for f in np.linspace(0,1,61):
   q=start+f*(goal-start);v(q)
   for _,c,names in pairs(e.model,v.scratch):
    if c.dist<-.005:
     blockers.append({'fraction':float(f),'geoms':names,'distance_m':float(c.dist),'q':q.tolist()})
  if blockers:
   strongest=min(blockers,key=lambda x:x['distance_m'])
   found.append({'candidate_index':k,'start':start.tolist(),'goal':goal.tolist(),'joint_distance_rad':float(np.linalg.norm(goal-start)),'blocker':strongest})
   print(json.dumps(found[-1]),flush=True)
   if len(found)>=5:break
 stats.update({'wall_s':time.perf_counter()-started,'validity_calls':v.calls,'found':found})
 (HERE/'candidate_search.json').write_text(json.dumps(stats,indent=2)+'\n')
print(json.dumps(stats),flush=True)
