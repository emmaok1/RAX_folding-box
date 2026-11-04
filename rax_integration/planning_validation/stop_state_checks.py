"""STOP state-machine checks without simulator, controller, viewer or network."""
from pathlib import Path
import json,tempfile,threading,unittest
from types import SimpleNamespace
from unittest.mock import Mock
from rax_integration.motion_planning.service import MotionService
from rax_integration.motion_planning.operator_stop import request_stop

class StopStateChecks(unittest.TestCase):
    def setUp(self):
        t=tempfile.TemporaryDirectory();self.addCleanup(t.cleanup)
        s=self.s=object.__new__(MotionService);s.root=Path(t.name);s.out=s.root/'motion_planning';s.out.mkdir()
        s.stop_file=s.out/'STOP';s.mutex=threading.Lock();s.active=None;s.owner_plan='paused';s.recovery_required=None;s.last=None
        s.env=SimpleNamespace(env_step=7,step_joint_targets=Mock(side_effect=AssertionError('STOP must not actuate')))
        s.episode='synthetic-state-only';s.created_policy='synthetic-policy';s.cfg={'max_rows':50,'max_execute_rows':50,'max_plan_rows':500}
        def plan(state,cursor):return {'state':state,'cursor':cursor,'result':{'rows':((0.,)*7,)*82},'source_hash':'synthetic','world_hash':'synthetic','trajectory_hash':'synthetic'}
        s.plans={'planned':plan('planned',0),'paused':plan('paused',7)}
    def test_paused_stop_immediately_terminalizes_and_releases_owner(self):
        reply=self.s.stop('unit STOP')
        self.assertEqual(reply['status'],'stopped');self.assertFalse(reply['pending_execution_boundary'])
        self.assertEqual(set(reply['terminalized_plans']),{'planned','paused'})
        self.assertTrue(all(p['state']=='stopped' for p in self.s.plans.values()))
        self.assertIsNone(self.s.owner_plan);self.assertEqual(self.s.recovery_required['plan_id'],'paused')
        self.assertEqual(json.loads((self.s.out/'paused.cursor.json').read_text())['state'],'stopped')
        self.assertEqual(self.s.env.env_step,7);self.s.env.step_joint_targets.assert_not_called()
    def test_stop_then_resume_refuses_without_motion(self):
        self.s.stop();reply=self.s.execute('paused',expected_cursor=7)
        self.assertEqual(reply['reason'],'STOP_REQUESTED');self.assertEqual(self.s.plans['paused']['cursor'],7)
        self.assertEqual(self.s.env.env_step,7);self.s.env.step_joint_targets.assert_not_called()
    def test_active_stop_does_not_block_or_mutate_midrow(self):
        self.s.plans['paused']['state']='executing';self.s.active='paused';self.s.mutex.acquire()
        try:
            reply=self.s.stop('callback STOP')
            self.assertTrue(reply['pending_execution_boundary']);self.assertEqual(reply['status'],'stop_requested')
            self.assertEqual(self.s.plans['paused']['state'],'executing');self.assertEqual(self.s.owner_plan,'paused')
            self.assertEqual(self.s.plans['paused']['cursor'],7);self.assertTrue(self.s.stop_file.exists())
        finally:self.s.mutex.release()
    def test_out_of_band_flag_finalizes_on_next_execute_admission(self):
        request_stop(self.s.root,'operator flag')
        reply=self.s.execute('paused',expected_cursor=7)
        self.assertEqual(reply['reason'],'STOP_REQUESTED');self.assertEqual(self.s.plans['paused']['state'],'stopped')
        self.assertIsNone(self.s.owner_plan);self.s.env.step_joint_targets.assert_not_called()

if __name__=='__main__':unittest.main(verbosity=2)
