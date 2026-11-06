"""Full-scene simulator-assisted planner checks; no cloud or object actuation."""
from pathlib import Path
import copy,hashlib,json,time,unittest
from unittest import mock
import numpy as np
from environment import FoldingBoxEnv
from rax_integration.motion_planning.service import MotionService
from rax_integration.motion_planning.model import ROOT,world_snapshot
from rax_integration.motion_planning.collision import CollisionChecker
OUT=ROOT/'rax_integration/planning_validation/evidence'/('full_checks_'+time.strftime('%Y%m%dT%H%M%S',time.gmtime()))
OUT.mkdir(parents=True,exist_ok=True)
PROTECTED=['environment.py','robot_model.py','robot_config.json','physics.json','models/scene.xml','models/arena.xml','box_geometry.py','geometry/panels.json']
def hashes():return {name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in PROTECTED}
def state(e):return (e.data.qpos.copy(),e.data.qvel.copy(),e.data.ctrl.copy(),float(e.data.time),e.env_step)
def same(a,b):return all(np.array_equal(x,y) for x,y in zip(a,b))
MEASUREMENTS=[]
class FullSceneChecks(unittest.TestCase):
    def setUp(self):
        self.e=FoldingBoxEnv(placement='flat');self.out=OUT/self._testMethodName
        self.s=MotionService(self.e,'full_scene_check',self.out,world_policy='simulator_snapshot')
    def tearDown(self):self.e.close()
    def target(self):
        q=self.e.data.qpos[self.e.arms['right']['qad']].copy();q[1]+=.018
        self.s.model.set_arm('right',q);p,R=self.s.model.pose('right')
        return {'frame':'world','tcp':'tcp_R','position_m':p.tolist(),'quaternion_wxyz':R.as_quat()[[3,0,1,2]].tolist()}
    def plan(self):
        p=self.s.plan({'episode_id':'full_scene_check','expected_step_id':self.e.env_step,'arm':'right','target':self.target()})
        self.assertEqual(p['status'],'planned',p);return p['plan_id']
    def test_full_scene_immutable_plan_and_original_controller_execution(self):
        before=state(self.e);pid=self.plan();self.assertTrue(same(before,state(self.e)))
        self.assertEqual(self.s.model.m.nq,self.e.model.nq)
        c=CollisionChecker(self.s.model,'right',.005)
        paper=[(a,b) for a,b in c.pairs if any('card_' in self.s.model.m.geom(g).name for g in (a,b))]
        self.assertGreater(len(paper),0)
        self.assertEqual(self.s.preview(pid)['status'],'preview');self.assertTrue(same(before,state(self.e)))
        with mock.patch.object(self.e,'inverse_kinematics',side_effect=AssertionError('Cartesian IK must not execute joint trajectory')):
            result=self.s.execute(pid)
        self.assertTrue(result['command_completed'],result);self.assertTrue(result['goal_reached'],result)
        self.assertLessEqual(result['rows_executed'],50);self.assertAlmostEqual(self.e.data.time,result['rows_executed']*.02,places=10)
        MEASUREMENTS.append({'paper_pairs':len(paper),'full_model_nq':self.e.model.nq,**result})
    def test_changed_paper_snapshot_refuses_without_live_mutation(self):
        pid=self.plan();before=state(self.e);changed=world_snapshot(self.e)
        free=next(v for v in changed['joints'].values() if v['type']==0);free['qpos'][0]+=.01
        with mock.patch('rax_integration.motion_planning.service.world_snapshot',return_value=changed):
            result=self.s.execute(pid)
        self.assertEqual(result['reason'],'WORLD_GEOMETRY_CHANGED');self.assertTrue(same(before,state(self.e)))
    def test_policy_change_refuses_without_motion(self):
        pid=self.plan();before=state(self.e)
        (self.out/'motion_policy.json').write_text(json.dumps({'world_policy':'robot_static'}))
        result=self.s.execute(pid);self.assertEqual(result['reason'],'POLICY_CHANGED');self.assertTrue(same(before,state(self.e)))
    def test_legacy_mode_still_refuses_without_box_state_reads(self):
        before=state(self.e)
        with mock.patch('rax_integration.motion_planning.service.world_snapshot',side_effect=AssertionError('Forbidden box truth read')):
            limited=MotionService(self.e,'legacy',self.out/'legacy',world_policy='robot_static')
            result=limited.plan({'episode_id':'legacy','expected_step_id':0,'arm':'right','target':self.target()})
        self.assertEqual(result['reason'],'INSUFFICIENT_DYNAMIC_WORLD_COVERAGE');self.assertTrue(same(before,state(self.e)))
    def test_injected_collision_world_detects_paper_not_live_teleportation(self):
        before=state(self.e);snapshot=self.s.snapshot();free=next(v for v in snapshot['world']['joints'].values() if v['type']==0)
        free['qpos'][:3]=(self.e.tcp_pose('right')[:3]+np.array([0,0,-.02])).tolist()
        self.s.model.load(snapshot);checker=CollisionChecker(self.s.model,'right',.005)
        self.assertFalse(checker.check(np.array(snapshot['robot']['right']['q'])))
        self.assertTrue(any('card_' in name for name in checker.last_conflict['geoms']),checker.last_conflict)
        self.assertTrue(same(before,state(self.e)))
    def test_fast_moving_world_refuses_before_motion(self):
        before=state(self.e);moving=world_snapshot(self.e)
        free=next(v for v in moving['joints'].values() if v['type']==0);free['qvel'][0]=.5
        with mock.patch('rax_integration.motion_planning.service.world_snapshot',return_value=moving):
            result=self.s.plan({'episode_id':'full_scene_check','expected_step_id':0,'arm':'right','target':self.target()})
        self.assertEqual(result['reason'],'WORLD_MOVING');self.assertTrue(same(before,state(self.e)))
    def test_stop_after_first_actual_control_row(self):
        pid=self.plan();self.s.on_step=lambda *args:self.s.stop('test cancellation')
        result=self.s.execute(pid);self.assertEqual(result['reason'],'STOP_REQUESTED');self.assertEqual(result['rows_executed'],1)
        self.assertFalse(result['goal_reached']);MEASUREMENTS.append(result)
if __name__=='__main__':
    before=hashes();result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(FullSceneChecks));after=hashes()
    (OUT/'results.json').write_text(json.dumps({'passed':result.wasSuccessful(),'tests_run':result.testsRun,'measurements':MEASUREMENTS,'protected_before':before,'protected_after':after,'protected_unchanged':before==after,'cloud_requests':0},indent=2))
    print('EVIDENCE',OUT,flush=True);raise SystemExit(0 if result.wasSuccessful() and before==after else 1)
