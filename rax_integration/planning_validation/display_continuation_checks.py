"""Mocked display-loop checks. No simulator/viewer/agent/process is launched."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from rax_integration.planning_validation.display_execution import execute_for_display


class DisplayContinuationChecks(unittest.TestCase):
    def service(self):
        return SimpleNamespace(cfg={'max_rows':50,'max_trajectory_rows':500},execute=Mock(),stop=Mock())

    def test_same_id_continues_to_actual_full_completion(self):
        service=self.service()
        service.execute.side_effect=[{'status':'paused','cursor':50,'goal_reached':False},
                                     {'status':'paused','cursor':100,'goal_reached':False},
                                     {'status':'completed','cursor':120,'goal_reached':True}]
        result=execute_for_display(service,'one-plan',lambda:False)
        self.assertEqual(len(result['chunks']),3)
        self.assertTrue(result['last_result']['goal_reached'])
        self.assertTrue(all(call.args==('one-plan',) for call in service.execute.call_args_list))
        self.assertEqual([call.kwargs['expected_cursor'] for call in service.execute.call_args_list],[0,50,100])
        service.stop.assert_not_called()

    def test_cancel_before_execution_never_moves(self):
        service=self.service();result=execute_for_display(service,'one-plan',lambda:True)
        service.execute.assert_not_called();service.stop.assert_called_once()
        self.assertTrue(result['cancelled']);self.assertIsNone(result['last_result'])

    def test_cancel_between_chunks_never_runs_next_chunk(self):
        service=self.service();service.execute.return_value={'status':'paused','cursor':50}
        result=execute_for_display(service,'one-plan',lambda:service.execute.call_count==1)
        service.execute.assert_called_once();service.stop.assert_called_once()
        self.assertTrue(result['cancelled']);self.assertEqual(len(result['chunks']),1)

    def test_tracking_stop_is_final_without_retry(self):
        service=self.service();service.execute.return_value={'status':'stopped','reason':'TRACKING_ERROR','goal_reached':False}
        result=execute_for_display(service,'one-plan',lambda:False)
        service.execute.assert_called_once();self.assertFalse(result['last_result']['goal_reached'])

    def test_window_closed_during_execution_also_publishes_stop(self):
        service=self.service();service.execute.return_value={'status':'stopped','reason':'EPISODE_WALL_LIMIT'}
        result=execute_for_display(service,'one-plan',lambda:bool(service.execute.call_count))
        service.execute.assert_called_once();service.stop.assert_called_once()
        self.assertTrue(result['cancelled'])
        self.assertEqual(result['last_result']['reason'],'EPISODE_WALL_LIMIT')

    def test_bad_cursor_provider_cannot_loop_without_limit(self):
        service=self.service();service.execute.return_value={'status':'paused','cursor':0}
        result=execute_for_display(service,'one-plan',lambda:False)
        service.execute.assert_called_once();service.stop.assert_called_once()
        self.assertEqual(result['refusal'],'DISPLAY_INVALID_CURSOR')

    def test_explicit_plan_execute_limits_override_legacy_names(self):
        service=self.service()
        service.cfg.update(max_plan_rows=100,max_execute_rows=50)
        service.execute.side_effect=[{'status':'paused','cursor':50},{'status':'paused','cursor':100}]
        result=execute_for_display(service,'one-plan',lambda:False)
        self.assertEqual(service.execute.call_count,2)
        self.assertEqual(result['refusal'],'DISPLAY_CONTINUATION_CALL_LIMIT')


if __name__=='__main__':unittest.main(verbosity=2)
