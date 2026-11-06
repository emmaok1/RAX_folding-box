import json, sys, hashlib
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation, Slerp
from PIL import Image
sys.path.insert(0, str(Path.cwd()))
from environment import FoldingBoxEnv

out = Path('validation/robot_contact'); out.mkdir(parents=True, exist_ok=True)
e = FoldingBoxEnv(placement='flat')
rows=[]

def contacts():
    result=[]
    for i,c in enumerate(e.data.contact):
        names=[e.model.geom(int(g)).name or '' for g in [c.geom1,c.geom2]]
        if not any(n.startswith('card_') for n in names): continue
        other=next((n for n in names if not n.startswith('card_')), '')
        if not (other.startswith('jaw_pad_') or 'gripper' in other):continue
        f=np.zeros(6);mujoco.mj_contactForce(e.model,e.data,i,f)
        result.append({'geoms':names,'normal_force_N':float(f[0]),'distance_m':float(c.dist)})
    return result

def step(action=None):
    e.step(action)
    assert not e.data.warning.number.any(), 'MuJoCo warning'
    assert np.isfinite(e.data.qpos).all() and np.isfinite(e.data.qvel).all()
    cs=contacts()
    rows.append({'time_s':float(e.data.time),'right_tcp':e.tcp_pose('right').tolist(), 'left_tcp':e.tcp_pose('left').tolist(),'contacts':cs,'box_pose':e.data.joint('box_free').qpos.tolist()})
    return cs

def image(name):
    Image.fromarray(e.render_camera('back')).save(out/(name+'.png'))

def move(target, seconds, opening=.3, stop_contact=False):
    start=e.targets['right'].copy();end=np.array(target)
    rot=Slerp([0,1],Rotation.from_quat(np.array([start[3:7],end[3:7]])[:,[1,2,3,0]]))
    for k in range(1,round(seconds/e.control_dt)+1):
        u=k/round(seconds/e.control_dt);q=rot([u]).as_quat()[0][[3,0,1,2]]
        action={s:np.r_[e.targets[s],e.grippers[s]['command']] for s in e.arms}
        action['right']=np.r_[(1-u)*start[:3]+u*end[:3],q,opening]
        cs=step(action)
        if stop_contact and any(c['normal_force_N']>.01 for c in cs):return cs
    print('segment',e.tcp_pose('right').tolist(),'IK',e.ik_errors['right'],flush=True)
    return []

try:
    for _ in range(25):step()
    image('before')
    start={s:e.tcp_pose(s).copy() for s in e.arms}
    action={s:np.r_[start[s],.3] for s in e.arms}
    for s in action:action[s][2]+=.005
    for _ in range(50):step(action)
    movement={s:{'requested_delta_m':[0,0,.005],'measured_delta_m':(e.tcp_pose(s)[:3]-start[s][:3]).tolist(),'target_error_m':float(np.linalg.norm(e.tcp_pose(s)[:3]-action[s][:3]))} for s in e.arms}
    assert all(x['target_error_m']<.001 for x in movement.values()),movement
    print('movement',movement,flush=True)
    image('movement')
    # Move through free space, then lower slowly toward the middle of the base.
    move([-.32,-.065,1.08,0,1,0,0],3.)
    move([-.32,-.065,.95,0,1,0,0],2.)
    cs=move([-.32,-.065,.913,0,1,0,0],2.,stop_contact=True)
    image('contact')
    print('CONTACT',cs,flush=True)
    contact_time=float(e.data.time);contact_tcp=e.tcp_pose('right').tolist()
    retract=e.targets['right'].copy();retract[2]=1.02
    move(retract,1.5)
    for _ in range(25):step()
    image('retracted')
    released=not contacts()
    result={'movement':movement,'contact_confirmed':bool(cs),'contact_time_s':contact_time,'contact_tcp':contact_tcp,'contacts':cs,'released_after_retract':released,'warnings':e.data.warning.number.tolist(),'simulation_time_s':float(e.data.time),'method':'Existing Cartesian motor-control interface only; no writes to executed qpos/qvel or box state, no external forces, no physics/config changes. Direct scripted commissioning test; no model agent.','scene_sha256':hashlib.sha256(Path('models/scene.xml').read_bytes()).hexdigest()}
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    (out/'trace.json').write_text(json.dumps(rows)+'\n')
    print(json.dumps(result,indent=2),flush=True)
    assert cs,'No hand/cardboard contact'
    assert released,'Contact did not release'
finally:e.close()
