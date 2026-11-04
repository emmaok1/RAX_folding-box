"""Adapter budget/audit tests using a fake planner; no simulation or API calls."""
import sys,time,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'upstream'))
from services.dexmate_box.backend import DexmateBoxBackend

class AdapterChecks(unittest.TestCase):
    def setUp(self):
        b=self.b=object.__new__(DexmateBoxBackend)
        b.contract=SimpleNamespace(require_robot=lambda *a:None)
        b.env=SimpleNamespace(env_step=0);b.finished=False;b.termination_reason=None
        b.wall_limit=10;b.started=time.monotonic();b.smoke_policy=None
        b.fold_policy={'max_action_tool_calls':1,'max_action_rows':2};b._action_calls=0
        b._last_control_mode='cartesian_ik';b.log=Mock();b._video_frame=Mock();b._write_result=Mock()
        b._observe=lambda **kw:(kw,[])
        self.motion=Mock();self.motion.cursor_refusal.return_value=None;self.motion.preview.return_value={'status':'preview','joint_positions_rad':[[0]*7]*2}
        self.motion.execute.return_value={'status':'completed','executed':True,'rows_executed':2}
        self.motion.terminate.side_effect=lambda plan_id,reason:{'status':'stopped','reason':reason,'executed':False}
        b._motion_service=lambda:self.motion
    def execute(self):return self.b.call('dexmate_motion_execute',{'plan_id':'diagnostic'})
    def test_call_budget_and_normal_audit(self):
        self.execute();self.assertEqual(self.b._action_calls,1);self.b.log.assert_called_once()
        reply,_=self.execute()
        self.assertEqual(reply['transition']['motion']['reason'],'ACTION_CALL_LIMIT')
        self.assertTrue(self.b.finished)
        self.assertEqual(self.b._action_calls,1)
        self.assertEqual(self.motion.execute.call_count,1)
    def test_row_budget(self):
        self.motion.preview.return_value['joint_positions_rad']*=2
        self.execute()
        self.motion.execute.assert_called_once_with('diagnostic',max_rows=2,expected_cursor=None)
        self.assertEqual(self.b.log.call_args.kwargs['requested_rows'],2)
        self.assertEqual(self.b.log.call_args.kwargs['trajectory_remaining_rows'],4)
    def test_explicit_chunk_over_fold_row_cap_refuses(self):
        with self.assertRaises(Exception):self.b.call('dexmate_motion_execute',{'plan_id':'diagnostic','max_rows':3})
        self.motion.execute.assert_not_called();self.assertEqual(self.b._action_calls,0)
    def test_duplicate_and_skipped_cursor_spend_no_action_budget(self):
        for expected in (0,51):
            self.motion.cursor_refusal.return_value={'reason':'CURSOR_MISMATCH','cursor':50,'expected_cursor':expected}
            self.motion.refuse.return_value={'status':'refused','reason':'CURSOR_MISMATCH','executed':False}
            reply,_=self.b.call('dexmate_motion_execute',{'plan_id':'diagnostic','expected_cursor':expected})
            self.assertEqual(reply['transition']['motion']['reason'],'CURSOR_MISMATCH')
            self.assertEqual(self.b._action_calls,0)
            self.motion.execute.assert_not_called()
        self.b.log.assert_not_called()

    def test_finished_episode(self):
        self.b.finished=True
        reply,_=self.execute()
        self.assertEqual(reply['transition']['motion']['reason'],'EPISODE_ENDED')
        self.motion.execute.assert_not_called();self.assertFalse(self.b._motion_can_step())
    def test_expired_wall_budget(self):
        self.b.started-=20;self.execute();self.motion.execute.assert_not_called()
        self.assertTrue(self.b.finished);self.assertEqual(self.b.termination_reason,'wall_time_limit');self.assertEqual(self.b._action_calls,0)
    def test_terminal_row_audit_video_and_episode(self):
        self.b.env.env_step=5
        row={'step_id':5,'requested':{'right':[0]*7},'applied':{'right':[0]*7},'measured':{},'terminated':False,'truncated':True}
        self.b._motion_row(row,{}, {'progress':{'succeed':False}})
        self.b._video_frame.assert_called_once();self.assertEqual(self.b.log.call_args.kwargs['requested'],row['requested'])
        self.assertEqual(self.b.log.call_args.kwargs['applied'],row['applied']);self.assertEqual(self.b.log.call_args.kwargs['measured'],row['measured'])
        self.assertTrue(self.b.finished);self.assertFalse(self.b._motion_can_step())
    def test_stop_marks_episode_finished(self):
        self.motion.execute.return_value={'reason':'STOP_REQUESTED','executed':False}
        self.execute();self.assertTrue(self.b.finished);self.assertEqual(self.b.termination_reason,'operator_stop');self.b._write_result.assert_called_once()

if __name__=='__main__':unittest.main(verbosity=2)
