"""Independent small commissioning cases; full unchanged scene, no cloud/viewer/contact."""
from pathlib import Path
import json, tempfile, unittest, time, sys
import numpy as np
from scipy.spatial.transform import Rotation
from environment import FoldingBoxEnv
from rax_integration.motion_planning.service import MotionService
from rax_integration.planning_validation.commission_tracking import FIRST, FAILED, frozen_path, slow_rows, admit_diagnostic
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'upstream'))
from services.dexmate_box.tools import validate_request

MEASUREMENTS=[]
RAW_TRACES=[]
class MicroChecks(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.env=None;self.serial=0
    def tearDown(self):
        if self.env is not None:self.env.close()
    def fresh(self):
        if self.env is not None:self.env.close()
        self.env=FoldingBoxEnv(placement='flat',image_obs=False);self.serial+=1
        self.trace=[]
        self.s=MotionService(self.env,'micro',Path(self.tmp.name)/str(self.serial),world_policy='simulator_snapshot',
                             on_step=lambda row,obs,info:self.trace.append({**row,'progress':info['progress']}))
        return self.env,self.s
    def perform(self,side,pos,quat,label):
        s,e=self.s,self.env
        trace_start=len(self.trace)
        before=[e.data.qpos.copy(),e.data.qvel.copy(),e.data.ctrl.copy(),e.data.time,e.env_step]
        target={'frame':'world','tcp':'tcp_R' if side=='right' else 'tcp_L','position_m':list(pos),'quaternion_wxyz':list(quat)}
        plan=s.plan({'episode_id':'micro','expected_step_id':e.env_step,'arm':side,'target':target})
        self.assertTrue(all(np.array_equal(a,b) for a,b in zip(before,[e.data.qpos,e.data.qvel,e.data.ctrl,e.data.time,e.env_step])))
        record={'case':label,'arm':side,'kind':'actual_full_scene_motor_execution','plan_status':plan['status'],'target':target}
        MEASUREMENTS.append(record)
        self.assertEqual(plan['status'],'planned',plan)
        cursor=0;chunks=[]
        for _ in range(11):
            result=s.execute(plan['plan_id'],expected_cursor=cursor);chunks.append(result)
            if result['status']!='paused':break
            cursor=result['cursor']
        record.update(status='pass' if result.get('goal_reached') else 'fail',chunks=len(chunks),rows=sum(x['rows_executed'] for x in chunks),
                      goal_error=result['measured_goal_error'],max_tracking_error_rad=max(x['max_tracking_error_rad'] for x in self.trace),
                      sampled_robot_panel_contacts=max(x['progress']['robot_panel_contacts'] for x in self.trace))
        RAW_TRACES.append({'case':label,'arm':side,'rows':self.trace[trace_start:]})
        self.assertTrue(result['goal_reached'],result)
        self.assertLessEqual(record['max_tracking_error_rad'],.02)
        self.assertEqual(record['sampled_robot_panel_contacts'],0)
        self.assertFalse(any(e.data.warning.number))
    def test_named_frames_tcp_and_fk_both_arms(self):
        e,s=self.fresh()
        for side,prefix in [('right','R'),('left','L')]:
            arm=e.arms[side];site=arm['site'];body=int(e.model.site_bodyid[site])
            self.assertEqual([e.model.joint(int(j)).name for j in arm['joint_ids']],[f'{prefix}_arm_j{i}' for i in range(1,8)])
            np.testing.assert_allclose(e.model.site_pos[site],[0,0,.16],atol=1e-12)
            expected=e.data.xpos[body]+e.data.xmat[body].reshape(3,3)@np.array([0,0,.16])
            np.testing.assert_allclose(e.tcp_pose(side)[:3],expected,atol=1e-12)
            pos,rotation=s.model.pose(side)
            np.testing.assert_allclose(pos,e.tcp_pose(side)[:3],atol=1e-12)
            np.testing.assert_allclose(rotation.as_matrix(),e.data.site_xmat[site].reshape(3,3),atol=1e-12)
        MEASUREMENTS.append({'case':'frames','kind':'read_only_full_scene_FK','status':'pass','both_arms':True})
    def test_small_signed_world_axes_both_arms(self):
        for side in ['right','left']:
            for axis in range(3):
                for sign in [-1,1]:
                    with self.subTest(side=side,axis=axis,sign=sign):
                        self.fresh();pose=self.env.tcp_pose(side).copy();position=pose[:3].copy();position[axis]+=sign*.005
                        self.perform(side,position,pose[3:],f'axis_{axis}_{sign}')
    def test_small_world_orientation_both_arms(self):
        for side in ['right','left']:
            for axis in range(3):
                with self.subTest(side=side,axis=axis):
                    self.fresh();pose=self.env.tcp_pose(side).copy();rotvec=np.zeros(3);rotvec[axis]=np.deg2rad(1)
                    orientation=Rotation.from_rotvec(rotvec)*Rotation.from_quat(pose[[4,5,6,3]])
                    self.perform(side,pose[:3],orientation.as_quat()[[3,0,1,2]],f'orientation_axis_{axis}')
    def test_left_multi_waypoint_standoff(self):
        for repeat in range(3):
            self.fresh();orientation=self.env.tcp_pose('left')[3:].copy()
            for index,position in enumerate([[.02,.39,1.115],[-.02,.36,1.13],[-.07,.34,1.13],[-.11,.32,1.125],[-.15,.30,1.12]]):
                self.perform('left',position,orientation,f'left_standoff_r{repeat}_segment_{index}')

    def test_chunk_length_boundaries_49_50_51(self):
        for length in [49,50,51]:
            with self.subTest(length=length):
                e,s=self.fresh();q,r=frozen_path(FIRST);p=admit_diagnostic(s,q,r,r['rows']);self.assertTrue(s.execute(p['plan_id'])['goal_reached'])
                q,r=frozen_path(FAILED);rows=slow_rows(q['snapshot']['robot']['right']['qcmd'],r['rows'],2)[:length]
                p=admit_diagnostic(s,q,r,rows);self.assertEqual(p['status'],'planned')
                chunks=[];cursor=0
                for _ in range(2):
                    result=s.execute(p['plan_id'],expected_cursor=cursor);chunks.append(result)
                    if result['status']!='paused':break
                    cursor=result['cursor']
                self.assertTrue(result['command_completed'],result)
                self.assertEqual(sum(x['rows_executed'] for x in chunks),length)
                self.assertEqual([x['rows_executed'] for x in chunks],[length] if length<=50 else [50,1])
                MEASUREMENTS.append({'case':f'length_{length}','kind':'actual_full_scene_prefix_scheduler_test_not_goal_test','status':'pass','chunk_rows':[x['rows_executed'] for x in chunks]})
    def test_total_500_admission_and_501_refusal(self):
        e,s=self.fresh();q,r=frozen_path(FIRST)
        rows=r['rows']+[r['rows'][-1]]*(500-len(r['rows']))
        before=e.env_step;p=admit_diagnostic(s,q,r,rows)
        self.assertEqual(p['status'],'planned');self.assertEqual(s.preview(p['plan_id'])['total_rows'],500)
        rejected=admit_diagnostic(s,q,r,rows+[rows[-1]])
        self.assertEqual(rejected['reason'],'DIAGNOSTIC_ROW_LIMIT');self.assertEqual(e.env_step,before)
        MEASUREMENTS.append({'case':'total_length_limit','kind':'nonexecuting_admission_test','status':'pass','accepted':500,'refused':501})
    def test_bad_targets_and_unreachable_request_do_not_move(self):
        e,s=self.fresh();pose=e.tcp_pose('right').copy()
        request={'episode_id':'micro','expected_step_id':0,'arm':'right','target':{'frame':'world','tcp':'tcp_R','position_m':pose[:3].tolist(),'quaternion_wxyz':pose[3:].tolist()}}
        import copy
        invalid=[]
        for key,value in [('frame','camera'),('tcp','tcp_L'),('quaternion_wxyz',[0,0,0,0]),('position_m',[float('nan'),0,0])]:
            bad=copy.deepcopy(request);bad['target'][key]=value;invalid.append(bad)
        for bad in invalid:
            with self.assertRaises(Exception):validate_request('dexmate_motion_plan',bad)
        request['target']['position_m']=[100,100,100];before=e.env_step
        result=s.plan(request);self.assertEqual(result['status'],'refused',result);self.assertEqual(e.env_step,before)
        MEASUREMENTS.append({'case':'invalid_unreachable','kind':'schema_and_real_planning_refusal_no_motion','status':'pass','reason':result['reason']})

if __name__=='__main__':
    out=Path('rax_integration/planning_validation/evidence')/('micro_checks_'+time.strftime('%Y%m%dT%H%M%S',time.gmtime()));out.mkdir(parents=True)
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(MicroChecks))
    report={'passed':result.wasSuccessful(),'tests_run':result.testsRun,'cases':MEASUREMENTS,'failures':[(str(t),msg) for t,msg in result.failures+result.errors],
            'cloud_requests':0,'scope':'Full unchanged scene; explicit local reset per independent test. No viewer/contact/folding/cloud. Test-selected targets, not autonomous teacher.'}
    from rax_integration.motion_planning.model import source_hash
    report['motion_source_sha256']=source_hash()
    with (out/'control_rows.jsonl').open('w') as stream:
        for trace in RAW_TRACES:
            for row in trace['rows']:stream.write(json.dumps({'case':trace['case'],'arm':trace['arm'],**row},separators=(',',':'))+'\n')
    report['control_trace_file']='control_rows.jsonl'
    (out/'results.json').write_text(json.dumps(report,indent=2));print('EVIDENCE',out,flush=True)
    raise SystemExit(0 if result.wasSuccessful() else 1)
