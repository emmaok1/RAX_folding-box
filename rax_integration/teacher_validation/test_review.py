"""Review regressions: real preparation, mocked transport/processes, zero inference."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

from rax_integration.teacher.tests.test_prepare import PreparationChecks
from rax_integration.teacher.prepare import (load_prepared, publish_memory, memory_for,
                                            api_settings)
from rax_integration.teacher.run import execute, paid_preflight


class ReviewChecks(unittest.TestCase):
    setUp = PreparationChecks.setUp
    prepare = PreparationChecks.prepare

    def grant(self, cfg):
        scope = self.base/'operator'
        scope.mkdir()
        (scope/'CURRENT_SCOPE.json').write_text(json.dumps({
            'cloud_inference_allowed':True, 'attempt_consumed':False,
            'approved_budget_usd':10, 'authorization_task_id':'SYNTHETIC-NOT-A-GRANT'}))
        (cfg.root/'cloud_authorization.json').write_text(json.dumps({
            'allowed':True, 'consumed':False, 'max_cost_usd':10,
            'task_id':'SYNTHETIC-NOT-A-GRANT', 'user_authorization':'mocked local test only',
            'max_requests':40, 'max_wall_seconds':600, 'max_episode_starts':3}))
        return scope

    def test_final_deployed_instruction_tampering_refuses(self):
        cfg, deployed = self.prepare()
        with (deployed['workspace']/'AGENTS.md').open('a') as stream:stream.write('\nchanged\n')
        with self.assertRaisesRegex(ValueError, 'instructions'):load_prepared(cfg.root)

    def test_runtime_action_policy_change_refuses(self):
        cfg, _ = self.prepare()
        (cfg.root/'fold_policy.json').write_text('{}')
        with self.assertRaisesRegex(ValueError, 'Runtime policy'):load_prepared(cfg.root)

    def test_memory_publish_refuses_destination_symlink(self):
        cfg, deployed = self.prepare()
        outside = self.base/'outside'
        outside.mkdir()
        destination = deployed['workspace']/'memory/OPERATOR_PROGRESS.md'
        destination.unlink()
        destination.symlink_to(outside/'capture.txt')
        with self.assertRaises(ValueError):publish_memory(memory_for(cfg.root), deployed['workspace'])
        self.assertEqual(list(outside.iterdir()), [])

    def test_changed_environment_progress_is_explicitly_stale(self):
        from rax_integration.teacher.memory import MemoryStore
        cfg, _ = self.prepare()
        original = memory_for(cfg.root)
        original.update_progress({'next_step':'old-world-action'})
        changed = MemoryStore(original.root, {'world_environment_id':'changed'},
                              evidence_root=cfg.root/'native/episodes')
        export = self.base/'changed-export'
        changed.export_workspace(export)
        state = changed.snapshot()['state']
        self.assertEqual(state['progress']['status'], 'no_current_progress')
        self.assertEqual(state['progress']['notes'], {})
        self.assertEqual(state['progress_history'][-1]['notes']['next_step'], 'old-world-action')
        self.assertNotIn('old-world-action', (export/'memory/PROGRESS.md').read_text())
        self.assertIn('old-world-action', (export/'memory/structured.json').read_text())

    def test_model_and_nan_approval_refuse(self):
        with self.assertRaises(ValueError):api_settings('some-other-priced-model')
        cfg, _ = self.prepare(budget_usd=10)
        scope = self.grant(cfg)
        with mock.patch('rax_integration.teacher.run.HERE', scope):
            with self.assertRaises(PermissionError):paid_preflight(cfg.root, float('nan'))

    def test_relay_constructor_failure_closes_grant_and_removes_capability(self):
        cfg, _ = self.prepare(budget_usd=10)
        scope = self.grant(cfg)
        created = []
        original = tempfile.mkstemp
        def token(**kwargs):
            fd, name = original(dir=self.base, **kwargs)
            created.append(Path(name))
            return fd, name
        with mock.patch('rax_integration.teacher.run.HERE', scope), \
             mock.patch('harness.claude_cli.login.private_text', return_value='synthetic-not-a-real-key'), \
             mock.patch('rax_integration.teacher.run.tempfile.mkstemp', side_effect=token), \
             mock.patch('rax_integration.teacher.run.BoundedClaudeRelay', side_effect=RuntimeError('synthetic construction failure')):
            with self.assertRaises(RuntimeError):execute(cfg.root, approval=10, key_file=self.base/'key')
        state = json.loads((scope/'CURRENT_SCOPE.json').read_text())
        self.assertFalse(state['cloud_inference_allowed'])
        self.assertTrue(state['attempt_consumed'])
        self.assertNotIn('active_run_root', state)
        self.assertTrue(created)
        self.assertTrue(all(not p.exists() for p in created))
        self.assertEqual(json.loads((cfg.root/'teacher_result.json').read_text())['actual_model_requests'], 0)

    def test_viewer_crash_stops_mocked_agent_without_inference(self):
        cfg, _ = self.prepare(budget_usd=10)
        scope = self.grant(cfg)
        relay = mock.Mock()
        relay.url = 'http://127.0.0.1:1'
        relay.ledger = {'inference_requests':0, 'usage':[]}
        agent = mock.Mock(returncode=143, pid=123456789)
        agent.communicate.side_effect = [subprocess.TimeoutExpired('mock', .25), ('',''), ('','')]
        viewer = mock.Mock()
        viewer.poll.return_value = 139
        with mock.patch('rax_integration.teacher.run.HERE', scope), \
             mock.patch.dict(os.environ, {'DISPLAY':':synthetic'}), \
             mock.patch('harness.claude_cli.login.private_text', return_value='synthetic-not-a-real-key'), \
             mock.patch('rax_integration.teacher.run.BoundedClaudeRelay', return_value=relay), \
             mock.patch('rax_integration.launch.start_frontend', return_value=(mock.Mock(), self.base/'socket', mock.Mock())), \
             mock.patch('rax_integration.launch.stop_frontend'), \
             mock.patch('rax_integration.teacher.run.prepare_live_episode'), \
             mock.patch('harness.claude_cli.auto_research.write_home'), \
             mock.patch('harness.claude_cli.auto_research.agent_environment', return_value={}), \
             mock.patch('harness.agent_host.start', return_value=agent), \
             mock.patch('harness.agent_host.record'), \
             mock.patch('harness.agent_host.stop') as stop, \
             mock.patch('rax_integration.teacher.run.subprocess.Popen', return_value=viewer):
            result = execute(cfg.root, approval=10, key_file=self.base/'key', live=True)
        self.assertEqual(result['termination_reason'], 'viewer_exited')
        self.assertEqual(result['actual_model_requests'], 0)
        self.assertTrue(stop.called)
        self.assertFalse(json.loads((scope/'CURRENT_SCOPE.json').read_text())['cloud_inference_allowed'])


if __name__ == '__main__':
    unittest.main()
