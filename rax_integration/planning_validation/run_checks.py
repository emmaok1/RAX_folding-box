"""Focused stage-1 tests. No cloud, viewer, grasping or task-physics changes."""
from pathlib import Path
import sys,json,time,unittest,threading,hashlib
from unittest import mock
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'rax_integration/upstream'))
import numpy as np,mujoco
from environment import FoldingBoxEnv
from rax_integration.planning_validation.fixture import StaticRobotFixture,nearby_target
from rax_integration.motion_planning.model import robot_snapshot,source_hash,StaticRobotModel
from rax_integration.motion_planning.collision import CollisionChecker
from rax_integration.motion_planning.service import MotionService
from rax_integration.motion_planning.operator_stop import request_stop
from services.dexmate_box import tools as T

OUT=ROOT/'rax_integration/planning_validation/evidence';RUN=OUT/('checks_'+time.strftime('%Y%m%dT%H%M%S'));RUN.mkdir(parents=True,exist_ok=True);MEASUREMENTS=[]
PROTECTED=['environment.py','robot_model.py','robot_config.json','physics.json','models/scene.xml','models/arena.xml','box_geometry.py','geometry/panels.json']
def hashes():return {f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in PROTECTED}
def frozen(e):return [getattr(e.data,k).copy() for k in ['qpos','qvel','ctrl']]+[e.data.time,e.env_step]
def same(a,b):return all(np.array_equal(x,y) for x,y in zip(a,b))

class PlannerChecks(unittest.TestCase):
    def setUp(self):
        self.e=StaticRobotFixture();self.root=RUN/self._testMethodName;self.s=MotionService(self.e,'diagnostic_static',self.root,scene_text=self.e.fixture_xml)
    def tearDown(self):self.e.close()
    def plan(self,side='right'):
        r=self.s.plan({'episode_id':'diagnostic_static','expected_step_id':self.e.env_step,'arm':side,'target':nearby_target(self.e,side)});self.assertEqual(r['status'],'planned',r);return r['plan_id']
    def test_01_plan_preview_and_tracked_execution_both_arms(self):
        for side in ['right','left']:
            if side=='left':
                self.e.close();self.e=StaticRobotFixture();self.s=MotionService(self.e,'diagnostic_static',self.root/'left',scene_text=self.e.fixture_xml)
            before=frozen(self.e);pid=self.plan(side);self.assertTrue(same(before,frozen(self.e)));p=self.s.preview(pid);self.assertEqual(p['status'],'preview');self.assertTrue(same(before,frozen(self.e)));r=self.s.execute(pid);self.assertTrue(r['command_completed'],r);self.assertTrue(r['goal_reached'],r);self.assertLessEqual(r['rows_executed'],50);self.assertAlmostEqual(self.e.data.time,r['rows_executed']*.02,places=10);self.assertEqual(self.s.execute(pid)['reason'],'PLAN_ALREADY_USED');MEASUREMENTS.append({'fixture':'static robot/table, cardboard omitted','side':side,**r})
    def test_02_stale_step_refuses_without_motion(self):
        p=self.plan();self.e.env_step+=1;a=frozen(self.e);self.assertEqual(self.s.execute(p)['reason'],'STALE_STEP');self.assertTrue(same(a,frozen(self.e)))
    def test_03_measured_start_change_refuses(self):
        p=self.plan();idx=self.e.arms['left']['qad'][0];self.e.data.qpos[idx]+=.01;mujoco.mj_forward(self.e.model,self.e.data);a=frozen(self.e);self.assertEqual(self.s.execute(p)['reason'],'START_CHANGED');self.assertTrue(same(a,frozen(self.e)))
    def test_04_source_world_change_refuses(self):
        p=self.plan();self.s.static_hash='changed-world';a=frozen(self.e);self.assertEqual(self.s.execute(p)['reason'],'SOURCE_OR_WORLD_CHANGED');self.assertTrue(same(a,frozen(self.e)))
    def test_05_tracking_guard_sensor_fault(self):
        p=self.plan();real=robot_snapshot
        def faulty(e):
            s=real(e)
            if e.env_step:s['robot']['right']['q'][0]+=.1
            return s
        with mock.patch('rax_integration.motion_planning.service.robot_snapshot',faulty):r=self.s.execute(p)
        self.assertEqual(r['reason'],'TRACKING_ERROR');self.assertEqual(r['rows_executed'],1);MEASUREMENTS.append({'diagnostic_sensor_fault_only':True,**r})
    def test_06_stop_during_execution(self):
        p=self.plan();self.s.on_step=lambda *a:request_stop(self.root,'diagnostic row-boundary cancellation');r=self.s.execute(p);self.assertEqual(r['reason'],'STOP_REQUESTED');self.assertEqual(r['rows_executed'],1);MEASUREMENTS.append(r)
    def test_07_cancel_planning_out_of_band(self):
        result=[];before=frozen(self.e)
        def plan():result.append(self.s.plan({'episode_id':'diagnostic_static','expected_step_id':0,'arm':'right','target':nearby_target(self.e,'right')}))
        t=threading.Thread(target=plan);t.start();deadline=time.monotonic()+5
        while not list((self.root/'motion_planning').glob('*.query.json')) and time.monotonic()<deadline:time.sleep(.01)
        request_stop(self.root,'diagnostic planning cancellation');t.join(timeout=15);self.assertFalse(t.is_alive());self.assertEqual(result[0]['reason'],'STOP_REQUESTED');self.assertTrue(same(before,frozen(self.e)))
    def test_08_unsettled_reference_refuses(self):
        self.e.arms['left']['qcmd'][0]+=.1;a=frozen(self.e);r=self.s.plan({'episode_id':'diagnostic_static','expected_step_id':0,'arm':'right','target':nearby_target(self.e,'right')});self.assertEqual(r['reason'],'START_REFERENCE_MISMATCH');self.assertTrue(same(a,frozen(self.e)))
    def test_09_pair_coverage_and_collision_refusal(self):
        c=CollisionChecker(self.s.model,'right',.005);m=c.robot.m;names=[(m.geom(a).name,m.geom(b).name) for a,b in c.pairs];self.assertTrue(any('table_collision' in pair for pair in names));self.assertTrue(any(any('dexmate_L_' in n for n in pair) and any('dexmate_R_' in n for n in pair) for pair in names));self.assertTrue(any(any('jaw_pad' in n for n in pair) for pair in names));self.assertTrue(any(any('torso' in n or 'head' in n for n in pair) for pair in names))
        rng=np.random.default_rng(11);bad=None
        for _ in range(50):
            q=rng.uniform(m.jnt_range[self.s.model.index['right']['v'],0]+.01,m.jnt_range[self.s.model.index['right']['v'],1]-.01)
            if not c.check(q):bad=q;break
        self.assertIsNotNone(bad);self.e.data.qpos[self.e.arms['right']['qad']]=bad;self.e.arms['right']['qcmd']=bad.copy();mujoco.mj_forward(self.e.model,self.e.data);a=frozen(self.e);r=self.s.plan({'episode_id':'diagnostic_static','expected_step_id':0,'arm':'right','target':nearby_target(self.e,'right')});self.assertEqual(r['reason'],'START_IN_COLLISION');self.assertTrue(same(a,frozen(self.e)));MEASUREMENTS.append({'negative_fixture_initialization':True,'refusal':r})
    def test_10_schema_unknown_fields_and_zero_quaternion(self):
        args={'episode_id':'diagnostic_static','expected_step_id':0,'arm':'right','target':nearby_target(self.e,'right')};T.validate_request('dexmate_motion_plan',args)
        with self.assertRaises(Exception):T.validate_request('dexmate_motion_plan',{**args,'read_box_truth':True})
        args['target']['quaternion_wxyz']=[0,0,0,0]
        with self.assertRaises(Exception):T.validate_request('dexmate_motion_plan',args)
    def test_11_ownership_refusal(self):
        self.s.active='owned';r=self.s.plan({'episode_id':'diagnostic_static','expected_step_id':0,'arm':'left','target':nearby_target(self.e,'left')});self.assertEqual(r['reason'],'ARM_OWNERSHIP_BUSY')
    def test_14_unsettled_other_arm_refuses(self):
        self.e.data.qvel[self.e.arms['left']['vid'][0]]=.04
        before=frozen(self.e)
        r=self.s.plan({'episode_id':'diagnostic_static','expected_step_id':0,'arm':'right','target':nearby_target(self.e,'right')})
        self.assertEqual(r['reason'],'START_NOT_SETTLED');self.assertTrue(same(before,frozen(self.e)))
    def test_15_operator_stop_blocks_relay_before_transport(self):
        # In-process handler and dummy credentials: no server or provider network.
        import io
        from rax_integration.claude_relay import BoundedClaudeRelay
        calls=[]
        relay=object.__new__(BoundedClaudeRelay)
        relay.lock=threading.Lock();relay.capability='diagnostic-only';relay.root=self.root/'relay';relay.root.mkdir(parents=True)
        relay.ledger={'inference_requests':0,'token_count_requests':0,'rejections':[]};relay.settings={};relay.started=time.monotonic()
        relay.transport=lambda *args:calls.append(args)
        class Handler:
            headers={'Authorization':'Bearer diagnostic-only'}
            wfile=io.BytesIO()
            def send_response(self,status):self.status=status
            def send_header(self,*args):pass
            def end_headers(self):pass
        request_stop(self.root,'diagnostic no-network relay stop');handler=Handler();relay.handle(handler)
        self.assertEqual(handler.status,400);self.assertEqual(calls,[]);self.assertEqual(relay.ledger['inference_requests'],0);self.assertEqual(relay.ledger['token_count_requests'],0)
    def test_16_retiming_bounds(self):
        from rax_integration.motion_planning.trajectory import retime
        cfg=self.s.cfg;q=np.array(self.s.model.snapshot['robot']['right']['q']) if hasattr(self.s.model,'snapshot') else np.zeros(7)
        rows=retime([q,q+np.array([.018,0,0,0,0,0,0])],cfg,np.ones(7)*2.4)
        self.assertIsNotNone(rows);self.assertLessEqual(len(rows),50)
        v=np.diff(np.vstack([q,rows]),axis=0)/.02
        acc=np.diff(np.vstack([np.zeros(7),v,np.zeros(7)]),axis=0)/.02
        self.assertLessEqual(np.max(abs(v)),cfg['speed_limit_rad_s']+1e-10)
        self.assertLessEqual(np.max(abs(acc)),cfg['acceleration_limit_rad_s2']+1e-10)
    def test_17_policy_identity_and_stale_policy_refusal(self):
        from rax_integration.motion_planning.model import source_files
        names={str(p.relative_to(ROOT)) for p in source_files()}
        required={'rax_integration/motion_planning/service.py','rax_integration/motion_planning/config.json','rax_integration/requirements-planning.txt'}
        required|={'rax_integration/upstream/services/dexmate_box/'+n for n in ['backend.py','tools.py','registry.py']}
        self.assertTrue(required<=names);self.assertTrue(any(n.endswith('.so') for n in names));self.assertTrue(any(n.endswith('METADATA') and 'ompl-' in n for n in names))
        before=frozen(self.e)
        with mock.patch('rax_integration.motion_planning.service.source_epoch',return_value=[('diagnostic_policy_change',0,0)]):
            r=self.s.plan({'episode_id':'diagnostic_static','expected_step_id':0,'arm':'right','target':nearby_target(self.e,'right')})
        self.assertEqual(r['reason'],'SOURCE_CHANGED_RECREATE_SERVICE');self.assertTrue(same(before,frozen(self.e)))
    def test_13_ompl_rrtconnect_non_direct_obstacle(self):
        # Planner-only geometric fixture: no robot physics or task objects.
        import subprocess
        file=Path(__file__).with_name('rrt_probe.py');p=subprocess.run([sys.executable,'-I',str(file)],capture_output=True,text=True,timeout=10);self.assertEqual(p.returncode,0,p.stdout+p.stderr);self.assertIn('RRTConnect',p.stdout)

    def test_12_live_dynamic_scene_refusal_and_no_truth_reads(self):
        live=FoldingBoxEnv();before=frozen(live);allowedq=set(np.r_[*[live.arms[s]['qad'] for s in ['right','left']],*[live.grippers[s]['qad'] for s in ['right','left']]].tolist());allowedv=set(np.r_[*[live.arms[s]['vid'] for s in ['right','left']],*[live.grippers[s]['vid'] for s in ['right','left']]].tolist())
        class Array:
            def __init__(self,a,allowed):self.a,self.allowed=a,allowed
            def __getitem__(self,key):
                if not isinstance(key,(list,np.ndarray)) or not set(np.asarray(key).ravel().tolist())<=self.allowed:raise AssertionError('Hidden dynamic-state read attempted')
                return self.a[key]
        class Data:
            def __init__(self,d):self.d=d
            def __getattr__(self,k):
                if k=='qpos':return Array(self.d.qpos,allowedq)
                if k=='qvel':return Array(self.d.qvel,allowedv)
                return getattr(self.d,k)
        original=live.data
        try:
            live.data=Data(original);s=MotionService(live,'actual_box',self.root/'actual_box');r=s.plan({'episode_id':'actual_box','expected_step_id':0,'arm':'right','target':nearby_target(self.e,'right')});self.assertEqual(r['reason'],'INSUFFICIENT_DYNAMIC_WORLD_COVERAGE');MEASUREMENTS.append({'actual_task_scene':True,'no_hidden_qpos_reads':True,**r})
        finally:live.data=original
        self.assertTrue(same(before,frozen(live)));live.close()

if __name__=='__main__':
    before=hashes();suite=unittest.defaultTestLoader.loadTestsFromTestCase(PlannerChecks);result=unittest.TextTestRunner(verbosity=2).run(suite);after=hashes();report={'tests_run':result.testsRun,'passed':result.wasSuccessful(),'failures':[(str(t),s) for t,s in result.failures+result.errors],'measurements':MEASUREMENTS,'protected_hashes_before':before,'protected_hashes_after':after,'protected_files_unchanged':before==after,'inference_requests':0,'scope':'Static fixture tracked motion is not cardboard avoidance or task success. Actual box scene must refuse unknown dynamic coverage.'};(RUN/'results.json').write_text(json.dumps(report,indent=2));print('EVIDENCE',RUN,flush=True);raise SystemExit(0 if result.wasSuccessful() and before==after else 1)
