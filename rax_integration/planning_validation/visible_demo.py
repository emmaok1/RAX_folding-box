"""One native-viewer static-fixture plan and measured motor execution, no cloud."""
from pathlib import Path
import os,json,time,subprocess
import numpy as np,mujoco,mujoco.viewer
from PIL import ImageGrab
from rax_integration.planning_validation.fixture import StaticRobotFixture,nearby_target
from rax_integration.motion_planning.service import MotionService

def main():
    root=Path(__file__).parent/'evidence'/('native_'+time.strftime('%Y%m%dT%H%M%S'));root.mkdir()
    (root/'viewer.pid').write_text(str(os.getpid()));print('PID',os.getpid(),'ROOT',root,flush=True)
    env=StaticRobotFixture();service=MotionService(env,'native_static_diagnostic',root,scene_text=env.fixture_xml)
    target=nearby_target(env,'right');start=env.tcp_pose('right').copy()
    plan=service.plan({'episode_id':service.episode,'expected_step_id':0,'arm':'right','target':target})
    (root/'plan.json').write_text(json.dumps(plan,indent=2));print('PLAN',json.dumps(plan),flush=True)
    if plan['status']!='planned':env.close();return
    preview=service.preview(plan['plan_id']);(root/'preview.json').write_text(json.dumps(preview,indent=2))
    with mujoco.viewer.launch_passive(env.model,env.data,show_left_ui=False,show_right_ui=False) as view:
        view.cam.type=mujoco.mjtCamera.mjCAMERA_FIXED;view.cam.fixedcamid=env.model.camera('back').id
        def text(phase,detail):
            view.set_texts((int(mujoco.mjtFontScale.mjFONTSCALE_150),int(mujoco.mjtGridPos.mjGRID_TOPLEFT),'STATIC DIAGNOSTIC / FREE-SPACE',phase+'\n'+detail+'\nCardboard omitted. NO cardboard avoidance / folding claim.'))
            view.sync(state_only=True)
        def screenshot(name):
            windows=subprocess.check_output(['wmctrl','-lpG'],text=True)
            for row in windows.splitlines():
                parts=row.split(None,8)
                if len(parts)>=9 and parts[2]==str(os.getpid()):
                    wid=parts[0];subprocess.run(['wmctrl','-ir',wid,'-N','MuJoCo | STATIC DIAGNOSTIC FREE-SPACE | NO FOLDING'],check=True)
                    subprocess.run(['wmctrl','-ia',wid],check=True);time.sleep(.2)
                    x,y,w,h=map(int,parts[3:7]);ImageGrab.grab(bbox=(x,y,x+w,y+h),xdisplay=os.environ['DISPLAY']).save(root/name)
                    (root/'window.json').write_text(json.dumps({'pid':os.getpid(),'window_id':wid,'geometry':[x,y,w,h]},indent=2));break
        text('PREVIEW ONLY - physics paused','Valid cached plan: '+str(len(preview['joint_positions_rad']))+' rows at 20ms')
        screenshot('preview_native.png');print('WINDOW_READY preview paused 10 wall seconds',flush=True)
        until=time.monotonic()+10
        while view.is_running() and time.monotonic()<until and not service.stop_file.exists():view.sync(state_only=True);time.sleep(.05)
        def row(entry,obs,info):
            text('MEASURED EXECUTION - original motors / joint refs',f"row {entry['step_id']} | sim {env.data.time:.2f}s | tracking {entry['max_tracking_error_rad']:.5f}rad")
            # Presentation delay only; no interpolated states or additional physics.
            time.sleep(.15)
        service.on_step=row
        if view.is_running() and not service.stop_file.exists():result=service.execute(plan['plan_id'])
        else:result={'status':'cancelled_before_execution','executed':False}
        measured=env.tcp_pose('right');report={'scope':'STATIC DIAGNOSTIC; cardboard omitted; not folding','pid':os.getpid(),'target':target,'start_tcp':start.tolist(),'end_tcp':measured.tolist(),'measured_translation_m':float(np.linalg.norm(measured[:3]-start[:3])),'display_delay_s_per_20ms_row':.15,'execution':result}
        (root/'result.json').write_text(json.dumps(report,indent=2));print('RESULT',json.dumps(report),flush=True)
        text('FINAL MEASURED STATE - physics paused',f"completed={result.get('command_completed',False)} reached={result.get('goal_reached',False)} | {np.linalg.norm(measured[:3]-start[:3])*1000:.2f}mm displacement")
        screenshot('final_native.png')
        while view.is_running() and not service.stop_file.exists():view.sync(state_only=True);time.sleep(.05)
    env.close()

if __name__=='__main__':main()
