import sys, os, numpy as np, mujoco
root, out = sys.argv[1], sys.argv[2]
os.chdir(root); sys.path.insert(0, root)
from environment import FoldingBoxEnv
env = FoldingBoxEnv(placement='flat'); env.reset()   # reset = mj_resetData + mj_forward only
m, d = env.model, env.data
assert d.time == 0.0
m.vis.global_.offwidth = max(m.vis.global_.offwidth, 1920); m.vis.global_.offheight = max(m.vis.global_.offheight, 1200)
r = mujoco.Renderer(m, 1080, 1620)
from PIL import Image
def shot(name, lookat, dist, az, el):
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = lookat; cam.distance = dist; cam.azimuth = az; cam.elevation = el
    r.update_scene(d, cam); Image.fromarray(r.render()).save(f'{out}/{name}.png'); print('saved', name)
b = lambda n: d.body(n).xpos.copy()
print({n: b(n).round(3).tolist() for n in ['box_base','dexmate_R_gripper_base','dexmate_R_arm_l4','dexmate_base_link','table']})
box = b('box_base'); rg = b('dexmate_R_gripper_base'); arm = b('dexmate_R_arm_l4')
shot('scene_full', (box+b('dexmate_base_link'))/2 + [0,0,0.35], 2.6, 150, -22)
shot('scene_full_side', (box+b('dexmate_base_link'))/2 + [0,0,0.3], 2.4, 90, -15)
shot('arm_right', (arm+rg)/2, 0.75, 230, -20)
shot('gripper_right', rg, 0.28, 220, -15)
bc=np.mean([d.body(i).xpos for i in range(m.nbody) if (m.body(i).name or '').startswith('box_')],axis=0); print('box center',bc)
shot('box_top', bc, 1.05, 90, -89.9)
shot('box_oblique', bc, 1.0, 140, -40)
for name in ['back','head','handcam_rgb_left','handcam_rgb_right']:
    rr = mujoco.Renderer(m, 480, 640); rr.update_scene(d, camera=name); Image.fromarray(rr.render()).save(f'{out}/cam_{name}.png')
print('time after', d.time)
