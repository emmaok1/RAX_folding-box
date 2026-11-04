"""MP-FULL-20261103-072531: one full-scene, simulator-assisted free-space demo.

Draft only until monitor READY. No cloud, grasping, folding or live qpos writes.
The native viewer and original joint controller share the SAME model/data.
"""
from pathlib import Path
import argparse,json,os,signal,subprocess,time
import numpy as np
import mujoco,mujoco.viewer
from scipy.spatial.transform import Rotation
from environment import FoldingBoxEnv
from rax_integration.motion_planning.service import MotionService
from rax_integration.motion_planning.operator_stop import request_stop

TASK='MP-FULL-20261103-072531'

def robot_fk_goal(env,side):
    """A small joint-neighbour FK goal using measured ROBOT state only.

    Scratch FK never copies box/free/crease qpos and never touches live state.
    No collision query is used to choose the goal. Planning must validate it.
    """
    scratch=mujoco.MjData(env.model)
    for name in ('right','left'):
        for ids in (env.arms[name]['qad'],env.grippers[name]['qad']):
            scratch.qpos[ids]=env.data.qpos[ids]
    ids=env.arms[side]['qad'];scratch.qpos[ids[1]]+=.018 if side=='right' else -.018
    mujoco.mj_kinematics(env.model,scratch)
    site=env.model.site('tcp_R' if side=='right' else 'tcp_L').id
    q=Rotation.from_matrix(scratch.site_xmat[site].reshape(3,3)).as_quat()[[3,0,1,2]]
    return {'frame':'world','tcp':'tcp_R' if side=='right' else 'tcp_L',
            'position_m':scratch.site_xpos[site].tolist(),'quaternion_wxyz':q.tolist()}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path)
    parser.add_argument('--arm',choices=['right','left'],default='right')
    parser.add_argument('--preview-seconds',type=float,default=10.)
    parser.add_argument('--row-display-delay',type=float,default=.15)
    args=parser.parse_args()
    if not 0<=args.preview_seconds<=60 or not 0<=args.row_display_delay<=.5:
        parser.error('Presentation delays must be bounded: preview <=60s, per-row <=0.5s')
    root=(args.root or Path(__file__).parent/'evidence'/('full_native_'+time.strftime('%Y%m%dT%H%M%S'))).resolve()
    root.mkdir(parents=True,exist_ok=False)
    (root/'viewer.pid').write_text(str(os.getpid()))
    print(json.dumps({'task':TASK,'pid':os.getpid(),'root':str(root),'phase':'starting FULL ORIGINAL scene'}),flush=True)
    def stop(reason):
        request_stop(root,reason)
    old_handlers={s:signal.signal(s,lambda signum,frame:stop('operator signal '+str(signum))) for s in (signal.SIGTERM,signal.SIGINT)}
    env=None;service=None
    try:
        env=FoldingBoxEnv(placement='flat',seed=0,image_obs=False)
        service=MotionService(env,'full_native_free_space',root,world_policy='simulator_snapshot')
        target=robot_fk_goal(env,args.arm);start=env.tcp_pose(args.arm).copy();start_time=float(env.data.time)
        def key(code):
            if code in (81,113,256):stop('operator viewer stop key')
        with mujoco.viewer.launch_passive(env.model,env.data,key_callback=key,show_left_ui=False,show_right_ui=False) as view:
            view.cam.type=mujoco.mjtCamera.mjCAMERA_FIXED
            view.cam.fixedcamid=env.model.camera('back').id
            def phase(name,detail=''):
                view.set_texts((int(mujoco.mjtFontScale.mjFONTSCALE_100),int(mujoco.mjtGridPos.mjGRID_TOPLEFT),
                    'FULL ORIGINAL SCENE / LOCAL OMPL / NO CLOUD',
                    'SIMULATOR-ASSISTED collision world (not vision-only)\n'+name+'\n'+detail+'\nFree-space only; NO grasp, fold or locking success. Q / Esc: STOP.'))
                view.sync(state_only=True)
                print('PHASE',name,detail,flush=True)
            def cancelled():
                if not view.is_running() and not service.stop_file.exists():stop('native window closed')
                return service.stop_file.exists()
            service.can_step=lambda:not cancelled()
            def wait(seconds):
                until=time.monotonic()+seconds
                while view.is_running() and not cancelled() and time.monotonic()<until:
                    view.sync(state_only=True);time.sleep(.03)
            def screenshot(name):
                # Capture the ACTUAL new native window, never another viewer.
                try:
                    windows=subprocess.check_output(['wmctrl','-lpG'],text=True)
                    for row in windows.splitlines():
                        fields=row.split(None,8)
                        if len(fields)>=9 and fields[2]==str(os.getpid()):
                            wid=fields[0]
                            subprocess.run(['wmctrl','-ir',wid,'-N','MuJoCo | FULL SCENE | SIMULATOR-ASSISTED FREE-SPACE'],check=True)
                            subprocess.run(['wmctrl','-ia',wid],check=True)
                            subprocess.run(['xwd','-silent','-id',wid,'-out',str(root/(name+'.xwd'))],check=True,timeout=5)
                            import shutil
                            convert=shutil.which('convert')
                            if convert:subprocess.run([convert,str(root/(name+'.xwd')),str(root/(name+'.png'))],check=True,timeout=5)
                            (root/'window.json').write_text(json.dumps({'task':TASK,'pid':os.getpid(),'window_id':wid,'geometry':fields[3:7]},indent=2))
                            return
                    print('SCREENSHOT unavailable: no native window matching this PID',flush=True)
                except (OSError,subprocess.SubprocessError) as exc:
                    print('SCREENSHOT error',type(exc).__name__,str(exc),flush=True)
            phase('PLANNING - live physics paused','Robot + table + cardboard retained. Test-selected short robot FK goal.')
            screenshot('planning_native')
            if cancelled():return
            plan=service.plan({'episode_id':service.episode,'expected_step_id':env.env_step,'arm':args.arm,'target':target})
            (root/'plan.json').write_text(json.dumps(plan,indent=2));print('PLAN',json.dumps(plan),flush=True)
            if plan.get('status')!='planned':
                phase('PLANNING REFUSED - no execution',str(plan.get('reason')))
                screenshot('refusal_native')
                while view.is_running() and not cancelled():wait(.1)
                return
            preview=service.preview(plan['plan_id']);(root/'preview.json').write_text(json.dumps(preview,indent=2))
            if preview.get('status')!='preview':
                phase('PREVIEW REFUSED',str(preview.get('reason')));return
            rows=preview['joint_positions_rad']
            if not 1<=len(rows)<=50:raise RuntimeError('Planner output violates existing 50-row ceiling')
            # Render-only path markers from robot FK. No live joint/object writes.
            fk=mujoco.MjData(env.model)
            for side in ('right','left'):
                for ids in (env.arms[side]['qad'],env.grippers[side]['qad']):fk.qpos[ids]=env.data.qpos[ids]
            site=env.model.site('tcp_R' if args.arm=='right' else 'tcp_L').id
            with view.lock():
                view.user_scn.ngeom=0
                for row in rows:
                    fk.qpos[env.arms[args.arm]['qad']]=row;mujoco.mj_kinematics(env.model,fk)
                    index=view.user_scn.ngeom
                    mujoco.mjv_initGeom(view.user_scn.geoms[index],mujoco.mjtGeom.mjGEOM_SPHERE,np.array([.002]*3),fk.site_xpos[site],np.eye(3).ravel(),np.array([1.,.75,.1,1.]))
                    view.user_scn.ngeom+=1
                index=view.user_scn.ngeom
                mujoco.mjv_initGeom(view.user_scn.geoms[index],mujoco.mjtGeom.mjGEOM_SPHERE,np.array([.004]*3),np.array(target['position_m']),np.eye(3).ravel(),np.array([.2,1.,.3,1.]))
                view.user_scn.ngeom+=1
            phase('PREVIEW ONLY - physics paused',f'{len(rows)} cached 20ms rows. Yellow=planned TCP path; green=goal (render-only).')
            screenshot('preview_native');wait(args.preview_seconds)
            def on_step(entry,obs,info):
                phase('MEASURED EXECUTION - original joint controller',f"row {entry['step_id']} sim={env.data.time:.3f}s tracking={entry['max_tracking_error_rad']:.5f}rad")
                wait(args.row_display_delay)  # Display delay only, no extra physics.
            service.on_step=on_step
            result=service.execute(plan['plan_id']) if not cancelled() else {'status':'cancelled_before_execution','executed':False}
            end=env.tcp_pose(args.arm)
            report={'task':TASK,'world_policy':'simulator_snapshot','source':'current FoldingBoxEnv flat; same viewer/controller model and data',
                'inference_requests':0,'task_success_claimed':False,'target_selection':'measured robot FK only',
                'start_tcp':start.tolist(),'target':target,'end_tcp':end.tolist(),'measured_translation_m':float(np.linalg.norm(end[:3]-start[:3])),
                'sim_seconds_advanced':float(env.data.time)-start_time,'display_delay_s_per_row':args.row_display_delay,'execution':result}
            (root/'result.json').write_text(json.dumps(report,indent=2));print('RESULT',json.dumps(report),flush=True)
            if not cancelled():
                phase('FINAL MEASURED STATE - physics paused',f"completed={result.get('command_completed',False)}, reached={result.get('goal_reached',False)}, error={result.get('measured_goal_error')}")
                screenshot('final_native')
            while view.is_running() and not cancelled():wait(.1)
    finally:
        if env is not None:env.close()
        for sig,handler in old_handlers.items():signal.signal(sig,handler)

if __name__=='__main__':main()
