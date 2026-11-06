"""Visible, simulator-assisted free-space proof in the unchanged full task scene."""
from pathlib import Path
import argparse,json,os,signal,subprocess,time,threading
import numpy as np
import mujoco,mujoco.viewer
from environment import FoldingBoxEnv
from rax_integration.motion_planning.model import ROOT
from rax_integration.motion_planning.service import MotionService

def cancelled(stopping,service=None):
    return stopping.is_set() or (service is not None and service.stop_file.exists())

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--task-id',required=True)
    parser.add_argument('--run-dir',type=Path)
    parser.add_argument('--preview-seconds',type=float,default=8)
    parser.add_argument('--display-delay',type=float,default=.08)
    args=parser.parse_args()
    out=args.run_dir or ROOT/'rax_integration/planning_validation/evidence'/('full_visible_'+time.strftime('%Y%m%dT%H%M%S',time.gmtime()))
    out.mkdir(parents=True,exist_ok=True)
    env=FoldingBoxEnv(placement='flat');stopping=threading.Event();service=None;window=None;states=[]
    def stop(*unused):
        stopping.set()
        if service is not None:service.stop('user/operator stop')
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    def find_window():
        try:
            for line in subprocess.check_output(['wmctrl','-lp'],text=True,stderr=subprocess.DEVNULL).splitlines():
                fields=line.split(None,4)
                if len(fields)>2 and fields[2]==str(os.getpid()):return fields[0]
        except (OSError,subprocess.CalledProcessError):pass
        return None
    def phase(name,**details):
        nonlocal window
        packet={'phase':name,'task_id':args.task_id,'wall_unix_s':time.time(),'step_id':env.env_step,'sim_time_s':float(env.data.time),**details}
        print(json.dumps(packet),flush=True)
        (out/'phase.json').write_text(json.dumps(packet,indent=2))
        window=window or find_window()
        if window:
            subprocess.run(['wmctrl','-ir',window,'-T',f'MuJoCo | FULL robot+desk+box | {name} | local OMPL / NO CLOUD'],stderr=subprocess.DEVNULL)
    try:
        with mujoco.viewer.launch_passive(env.model,env.data,key_callback=lambda key:stop() if key==83 else None,show_left_ui=False,show_right_ui=False) as viewer:
            viewer.cam.lookat[:]=[-.4,0,1.05];viewer.cam.distance=2.65;viewer.cam.azimuth=145;viewer.cam.elevation=-24
            viewer.opt.geomgroup[:]=[1,1,1,0,0,0]
            def show(seconds):
                until=time.monotonic()+seconds
                while viewer.is_running() and not cancelled(stopping,service) and time.monotonic()<until:
                    viewer.sync(state_only=True);time.sleep(.03)
            def screenshot(name):
                if window:
                    try:subprocess.run(['import','-window',window,str(out/name)],timeout=8,check=False,stderr=subprocess.DEVNULL)
                    except (OSError,subprocess.TimeoutExpired):pass
            def on_step(entry,obs,info):
                states.append((env.data.qpos.copy(),env.data.qvel.copy(),float(env.data.time)))
                if not viewer.is_running():stop()
                phase('EXECUTING',row=entry['step_id'],tracking_error_rad=entry['max_tracking_error_rad'])
                viewer.sync(state_only=True)
                if args.display_delay>0:time.sleep(args.display_delay)
            phase('INITIAL PAUSED',information_policy='simulator-state-assisted local collision planning; not vision-only',paper_present=True)
            show(2)
            if cancelled(stopping) or not viewer.is_running():
                phase('STOPPED BEFORE PLANNING');return
            service=MotionService(env,args.task_id,out,world_policy='simulator_snapshot',on_step=on_step,can_step=lambda:viewer.is_running() and not cancelled(stopping,service))
            if cancelled(stopping,service):
                phase('STOPPED BEFORE PLANNING');return
            start=env.tcp_pose('right').copy();q=env.data.qpos[env.arms['right']['qad']].copy();q[1]+=.06
            service.model.set_arm('right',q);position,rotation=service.model.pose('right')
            target={'frame':'world','tcp':'tcp_R','position_m':position.tolist(),'quaternion_wxyz':rotation.as_quat()[[3,0,1,2]].tolist()}
            phase('PLANNING',target=target)
            plan=service.plan({'episode_id':args.task_id,'expected_step_id':env.env_step,'arm':'right','target':target})
            (out/'plan.json').write_text(json.dumps(plan,indent=2))
            if plan['status']=='planned':
                preview=service.preview(plan['plan_id']);(out/'preview.json').write_text(json.dumps(preview,indent=2))
                if preview['status']=='preview':
                    points=[start[:3]]
                    for row in preview['joint_positions_rad'][::max(1,len(preview['joint_positions_rad'])//12)]:
                        service.model.set_arm('right',np.array(row));points.append(service.model.pose('right')[0])
                    service.model.load(service.snapshot())
                    with viewer.lock():
                        viewer.user_scn.ngeom=0
                        for point in points:
                            geom=viewer.user_scn.geoms[viewer.user_scn.ngeom]
                            mujoco.mjv_initGeom(geom,type=mujoco.mjtGeom.mjGEOM_SPHERE,size=np.full(3,.004),pos=point,mat=np.eye(3).ravel(),rgba=np.array([0,.8,1,.8]))
                            viewer.user_scn.ngeom+=1
                        geom=viewer.user_scn.geoms[viewer.user_scn.ngeom]
                        mujoco.mjv_initGeom(geom,type=mujoco.mjtGeom.mjGEOM_SPHERE,size=np.full(3,.01),pos=np.array(target['position_m']),mat=np.eye(3).ravel(),rgba=np.array([0,1,0,.85]));viewer.user_scn.ngeom+=1
                    phase('PREVIEW - NO MOTION',rows=len(preview['joint_positions_rad']),physics_advanced=False)
                    viewer.sync(state_only=True);screenshot('preview_full_scene.png');show(args.preview_seconds)
                    if args.execute and viewer.is_running() and not cancelled(stopping,service):
                        result=service.execute(plan['plan_id']);end=env.tcp_pose('right').copy()
                        report={'task_id':args.task_id,'scene':'FULL ORIGINAL robot,table,cardboard','world_policy':'simulator_snapshot','cloud_requests':0,'start_tcp':start.tolist(),'target':target,'end_tcp':end.tolist(),'measured_translation_m':float(np.linalg.norm(end[:3]-start[:3])),'display_delay_per_20ms_row_s':args.display_delay,'execution':result}
                        (out/'result.json').write_text(json.dumps(report,indent=2))
                        if states:np.savez_compressed(out/'measured_states.npz',qpos=np.array([s[0] for s in states]),qvel=np.array([s[1] for s in states]),time=np.array([s[2] for s in states]))
                        phase('FINAL PAUSED',result=result)
                    else:phase('PREVIEW ONLY PAUSED')
                else:phase('PREVIEW REFUSED',result=preview)
            else:phase('PLANNING REFUSED',result=plan)
            if viewer.is_running() and not cancelled(stopping,service):
                viewer.sync(state_only=True);screenshot('final_full_scene.png')
            while viewer.is_running() and not cancelled(stopping,service):viewer.sync(state_only=True);time.sleep(.03)
    finally:
        if service is not None and stopping.is_set():service.stop('operator stopped visible demo')
        env.close()

if __name__=='__main__':main()
