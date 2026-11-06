"""Independent zero-step checks, runnable while execution-runtime repair is blocked.

No planner/service construction, mj_step, agent, frontend, viewer or provider.
This does NOT validate the incomplete continuation/budget implementation.
"""
from pathlib import Path
import sys,json,hashlib,copy,datetime
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'rax_integration/upstream'))
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from environment import FoldingBoxEnv
from rax_integration.motion_planning.service import MotionService
from rax_integration.motion_planning.telemetry import SubstepMotorSampler
from rax_integration.motion_planning.limits import plan_rows,execute_rows
from services.dexmate_box.tools import validate_request


def main():
    out=Path(__file__).parent
    order=json.loads((ROOT/'rax_integration/monitor_orders/MP-TRACK-CONTINUE-20261104.json').read_text())
    findings={};env=FoldingBoxEnv(placement='flat')
    initial=(env.data.qpos.copy(),env.data.qvel.copy(),env.data.ctrl.copy(),env.env_step,float(env.data.time))
    try:
        frames={}
        for side,arm in env.arms.items():
            site=arm['site'];bid=int(env.model.site_bodyid[site]);body=env.data.body(bid)
            tcp=env.tcp_pose(side);local=env.model.site_pos[site]
            error=float(np.linalg.norm(tcp[:3]-(body.xpos+body.xmat.reshape(3,3)@local)))
            active=Rotation.from_quat(tcp[3:][[1,2,3,0]]).as_matrix()
            orientation_error=float(np.max(abs(active-env.data.site_xmat[site].reshape(3,3))))
            q=tcp[3:][[1,2,3,0]]
            sign_error=float((Rotation.from_quat(q)*Rotation.from_quat(-q).inv()).magnitude())
            names=[env.model.joint(int(j)).name for j in arm['joint_ids']]
            expected=[arm['prefix']+'_arm_j'+str(i) for i in range(1,8)]
            passed=bool(names==expected and len(set(arm['qad']))==7 and len(set(arm['vid']))==7
                        and np.allclose(local,[0,0,.16],atol=1e-12) and error<1e-12
                        and orientation_error<1e-12 and sign_error<1e-12)
            frames[side]={'passed':passed,'joint_names':names,'qpos_indices':arm['qad'].tolist(),
                          'qvel_indices':arm['vid'].tolist(),'site_body':env.model.body(bid).name,
                          'tcp_local_m':local.tolist(),'position_error_m':error,
                          'rotation_matrix_error':orientation_error,'quaternion_sign_error_rad':sign_error}
        findings['M01']={'status':'pass' if all(v['passed'] for v in frames.values()) else 'fail',
                         'kind':'current loaded full-scene kinematics; zero control rows',
                         'arms':frames,'limitation':'Virtual TCP transform only; no fingertip/grasp-access proof.'}
        pose=env.tcp_pose('right').tolist()
        good={'episode_id':'zero_step','expected_step_id':0,'arm':'right',
              'target':{'frame':'world','tcp':'tcp_R','position_m':pose[:3],'quaternion_wxyz':pose[3:]}}
        validate_request('dexmate_motion_plan',good)
        invalid=[]
        for label,key,value in [('frame','frame','table'),('tcp','tcp','tcp_L'),('zero_quaternion','quaternion_wxyz',[0,0,0,0]),('nonfinite_position','position_m',[float('nan'),0,1])]:
            args=copy.deepcopy(good);args['target'][key]=value
            try:validate_request('dexmate_motion_plan',args)
            except Exception as exc:invalid.append({'case':label,'refused':True,'error_type':type(exc).__name__,'reason':str(exc)})
            else:invalid.append({'case':label,'refused':False})
        args=copy.deepcopy(good);args['arm']='middle'
        try:validate_request('dexmate_motion_plan',args)
        except Exception as exc:invalid.append({'case':'invalid_arm','refused':True,'reason':str(exc)})
        else:invalid.append({'case':'invalid_arm','refused':False})
        findings['M03']={'status':'not-run','subchecks':{'schema_refusals':invalid},
                         'subcheck_passed':all(x['refused'] for x in invalid),
                         'remaining':'Real orientation-only, unreachable and timed-out planner queries not run on current repair.'}
        service=object.__new__(MotionService);service.plans={'one':{'cursor':50}}
        before=copy.deepcopy(service.plans);cursors=[]
        for expected in (0,49,51,None,True,-1,50):
            result=service.cursor_refusal('one',expected)
            cursors.append({'expected_cursor':expected,'result':result})
        cursor_ok=(all(x['result'] is not None for x in cursors[:-1]) and cursors[-1]['result'] is None and service.plans==before)
        findings['M06']={'status':'not-run','subcheck_passed':cursor_ok,'subchecks':cursors,
                         'remaining':'Pure admission predicate only; no mutex/backend/budget/physical execution or stale-other-plan proof here.'}
        cfg=json.loads((ROOT/'rax_integration/motion_planning/config.json').read_text())
        bounds={'max_plan_rows':plan_rows(cfg),'max_execute_rows':execute_rows(cfg)}
        for label,fn,patch in [('plan501',plan_rows,{'max_plan_rows':501}),('execute51',execute_rows,{'max_execute_rows':51})]:
            try:fn({**cfg,**patch})
            except ValueError as exc:bounds[label]={'refused':True,'reason':str(exc)}
            else:bounds[label]={'refused':False}
        findings['M10']={'status':'not-run','subchecks':bounds,
                         'remaining':'Pure limit validation only; action-call/episode/actual-row budget integration not run on current repair.'}
        original=mujoco.mj_step
        try:
            with SubstepMotorSampler(env):raise RuntimeError('injected sampler exception; no mj_step')
        except RuntimeError:pass
        findings['M09']={'status':'not-run','sampler_exception_restored':mujoco.mj_step is original,
                         'remaining':'No search/chunk/between-chunk STOP or execution error tests in this zero-step check.'}
        final=(env.data.qpos.copy(),env.data.qvel.copy(),env.data.ctrl.copy(),env.env_step,float(env.data.time))
        unchanged=all(np.array_equal(a,b) for a,b in zip(initial,final))
        sources=['environment.py','robot_config.json','physics.json','models/scene.xml',
                 'rax_integration/motion_planning/service.py','rax_integration/motion_planning/config.json',
                 'rax_integration/motion_planning/limits.py','rax_integration/motion_planning/telemetry.py',
                 'rax_integration/upstream/services/dexmate_box/tools.py']
        hashes={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in sources}
        cases=[]
        for spec in order['validation_program']['cases']:
            case={'id':spec['id'],'area':spec['area'],'status':'not-run',
                  'reason':'Current execution-runtime refinement blocked by source protection; prior passing runs have different source identities.'}
            case.update(findings.get(spec['id'],{}));cases.append(case)
        result={'task_id':order['task_id'],'order_revision':order['revision'],
                'observed_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
                'scope':'Independent current static kinematics and pure predicates only; incomplete motion runtime NOT validated.',
                'physics_control_rows':env.env_step,'sim_time_s':float(env.data.time),'provider_requests':0,
                'viewer_launched':False,'state_unchanged_after_reset':unchanged,'source_hashes':hashes,'cases':cases,
                'historical_evidence_separate':['planning_validation/evidence/mp_track_continue_20261104/implementation.json'],
                'remaining_blocker':'Execution sources are read-only444; no chmod/replace/bypass performed.'}
        (out/'suite_index.json').write_text(json.dumps(result,indent=2,allow_nan=False))
        print(json.dumps({'index':str(out/'suite_index.json'),'M01':findings['M01'],
                          'schema_refusals':findings['M03']['subcheck_passed'],'cursor_predicate':cursor_ok,
                          'sampler_restored':mujoco.mj_step is original,'unchanged':unchanged,'control_rows':env.env_step},indent=2))
        assert unchanged and env.env_step==0 and float(env.data.time)==0
        assert findings['M01']['status']=='pass' and findings['M03']['subcheck_passed'] and cursor_ok and mujoco.mj_step is original
    finally:env.close()


if __name__=='__main__':main()
