"""Three bounded repeats per arm in the explicit static-only diagnostic fixture."""
from pathlib import Path
import json,time,hashlib,subprocess
import numpy as np,mujoco
from PIL import Image,ImageDraw
from environment import GymRenderingSpec
from rax_integration.planning_validation.fixture import StaticRobotFixture,nearby_target
from rax_integration.motion_planning.service import MotionService
from rax_integration.planning_validation.run_checks import hashes

HERE=Path(__file__).parent;OUT=HERE/'evidence'/('repeat_'+time.strftime('%Y%m%dT%H%M%S'));OUT.mkdir(parents=True,exist_ok=True)
def run():
    before=hashes();results=[]
    for side in ['right','left']:
        for trial in range(3):
            e=StaticRobotFixture();e.render_spec=GymRenderingSpec();root=OUT/f'{side}_{trial}';s=MotionService(e,'static_fixture',root,scene_text=e.fixture_xml);r=s.plan({'episode_id':'static_fixture','expected_step_id':0,'arm':side,'target':nearby_target(e,side)});assert r['status']=='planned',r
            renderer=encoder=None
            if side=='right' and trial==0:
                renderer=mujoco.Renderer(e.model,480,640);encoder=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s','640x480','-r','50','-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p',str(OUT/'static_free_space.mp4')],stdin=subprocess.PIPE)
                def frame(entry,obs,info):
                    renderer.update_scene(e.data,camera='back');im=Image.fromarray(renderer.render());dr=ImageDraw.Draw(im);dr.rectangle((0,0,640,44),fill='white');dr.text((5,5),'STATIC DIAGNOSTIC FIXTURE - CARDBOARD OMITTED\nReal motor execution; NOT box-task or cardboard-avoidance success',fill='black');encoder.stdin.write(np.asarray(im).tobytes())
                s.on_step=frame;mujoco.mj_saveModel(e.model,str(OUT/'static_fixture_model.mjb'),None)
            try:r=s.execute(r['plan_id']);assert r['command_completed'] and r['goal_reached'],r;results.append({'side':side,'repeat':trial,**r})
            finally:
                if encoder:encoder.stdin.close();encoder.wait();renderer.close()
                e.close()
    after=hashes();assert before==after
    (OUT/'results.json').write_text(json.dumps({'results':results,'protected_hashes_before':before,'protected_hashes_after':after,'protected_unchanged':before==after,'inference_requests':0,'scope':'6 short single-arm static robot/table fixture runs. Dynamic cardboard omitted by construction; no cardboard avoidance/contact/folding claim.'},indent=2));print(OUT);print(json.dumps(results,indent=2))
if __name__=='__main__':run()
