"""Native viewer of states published by the actual RAX backend; no simulation steps."""
import argparse,time,json,os,subprocess
from pathlib import Path
import numpy as np,mujoco,mujoco.viewer

def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();state=a.root/'live_state.npz';policy=json.loads((a.root/'fold_policy.json').read_text());kind='OFFLINE SCRIPTED FIXTURE' if policy.get('offline_fixture') else 'LIVE CLOUD MODEL / RAX';deadline=time.monotonic()+120
    while not state.exists():
        if time.monotonic()>deadline:raise SystemExit('No RAX episode started within 120 seconds')
        time.sleep(.1)
    with np.load(state) as x:model_path=str(x['model_path'])
    m=mujoco.MjModel.from_binary_path(model_path);d=mujoco.MjData(m)
    with np.load(state) as x:d.qpos[:]=x['qpos'];d.qvel[:]=x['qvel'];d.time=float(x['time'])
    mujoco.mj_forward(m,d)
    from rax_integration.motion_planning.operator_stop import request_stop
    with mujoco.viewer.launch_passive(m,d,key_callback=lambda key:request_stop(a.root,'viewer S key') if key==83 else None,show_left_ui=False,show_right_ui=False) as v:
        v.cam.lookat[:]=[-.4,0,1.05];v.cam.distance=2.65;v.cam.azimuth=145;v.cam.elevation=-24;v.opt.geomgroup[:]=[1,1,1,0,0,0];v.opt.flags[mujoco.mjtVisFlag.mjVIS_ISLAND]=False
        for line in subprocess.check_output(['wmctrl','-lp'],text=True).splitlines():
            fields=line.split(None,4)
            if len(fields)>2 and fields[2]==str(os.getpid()):
                subprocess.run(['wmctrl','-ir',fields[0],'-T','MuJoCo | LIVE CLOUD + RAX + OMPL | FULL SCENE | simulator-assisted | S = STOP'])
                subprocess.run(['wmctrl','-ia',fields[0]])
        while v.is_running():
            with np.load(state) as x:
                with v.lock():d.qpos[:]=x['qpos'];d.qvel[:]=x['qvel'];d.time=float(x['time'])
                label=f"{kind} BACKEND STATE | step {int(x['step'])} | sim {d.time:.2f}s\n{str(x['phase'])}"
                age=time.time()-float(x['wall'])
            label+='\nSimulator-assisted collision planning | S stops this attempt'
            if (a.root/'motion_planning/STOP').exists():label+='\nOPERATOR STOP REQUESTED'
            if (a.root/'attempt_result.json').exists():
                result=json.loads((a.root/'attempt_result.json').read_text());label+=f"\nSTOPPED: {result['termination_reason']}"
            elif age>5:label+='\nNo new robot state while model is deciding'
            v.set_texts((mujoco.mjtFontScale.mjFONTSCALE_100,mujoco.mjtGridPos.mjGRID_TOPLEFT,label,'Rendering actual recorded backend states; no local controller'))
            # State-only synchronization recomputes the viewer's own derived data.
            v.sync(state_only=True);time.sleep(.03)
    if not (a.root/'attempt_result.json').exists():request_stop(a.root,'live window closed')
if __name__=='__main__':main()
