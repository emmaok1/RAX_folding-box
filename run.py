"""Current Dexmate task entry points; viewer uses the same environment controller."""
import argparse,json,time
import numpy as np,mujoco
from environment import FoldingBoxEnv
p=argparse.ArgumentParser();p.add_argument('mode',choices=['viewer','debug-open','debug-closed','reset']);p.add_argument('--placement',choices=['centered','flat'],default='centered');p.add_argument('--paused',action='store_true');a=p.parse_args()
if a.mode=='reset':
 e=FoldingBoxEnv(seed=11,placement=a.placement);before={n:getattr(e.data,n).copy() for n in ['qpos','qvel','ctrl']};e.step();e.reset(seed=11);result={n:bool(np.array_equal(v,getattr(e.data,n))) for n,v in before.items()};print(json.dumps(result,indent=2));assert all(result.values());e.close()
else:
 import mujoco.viewer
 e=FoldingBoxEnv(placement=a.placement);keys=[];paused=a.paused or a.mode.startswith('debug');selected='right'
 if a.mode.startswith('debug'):e.debug_pose(closed=a.mode=='debug-closed')
 print('Dexmate current task. R reset | Space pause | 1 left / 2 right | arrows XY, W/S Z (5 mm) | Z/X jaw close/open | O/C DEBUG open/closed. Debug never earns success.',flush=True)
 with mujoco.viewer.launch_passive(e.model,e.data,key_callback=lambda k:keys.append(k),show_left_ui=False,show_right_ui=False) as viewer:
  viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_ISLAND]=False
  viewer.opt.geomgroup[:]=[1,1,1,0,0,0]
  viewer.cam.lookat[:]=[-.4,0,1.05];viewer.cam.distance=2.65;viewer.cam.azimuth=145;viewer.cam.elevation=-24
  while viewer.is_running():
   tick=time.monotonic()
   while keys:
    k=keys.pop(0)
    if k==82:e.reset();paused=False
    elif k==32:paused=not paused
    elif k==49:selected='left'
    elif k==50:selected='right'
    elif k in [79,67]:e.reset();e.debug_pose(closed=k==67);paused=True
    elif k in [262,263,264,265,87,83,90,88]:
     action={s:np.r_[e.targets[s],e.grippers[s]['command']] for s in e.arms};dx={262:[0,-.005,0],263:[0,.005,0],264:[-.005,0,0],265:[.005,0,0],87:[0,0,.005],83:[0,0,-.005]}
     if k in dx:action[selected][:3]+=dx[k]
     else:action[selected][7]=np.clip(action[selected][7]+(.05 if k==88 else -.05),0,1)
     e._set_action(action)
   if not paused:e.step()
   viewer.sync();time.sleep(max(0,e.control_dt-(time.monotonic()-tick)))
 e.close()
