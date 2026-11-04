"""Local wire-level teacher reasoning tests; transport never accesses a provider."""
import io
import json
from pathlib import Path
import tempfile
import unittest
import urllib.error
import urllib.request

from rax_integration.claude_relay import BoundedClaudeRelay
from rax_integration.teacher.prepare import api_settings
from rax_integration.relay_checks import run as legacy_guards


class ReasoningChecks(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)

    def exercise(self, overrides=None, *, legacy=False):
        seen = []
        def transport(path, body, headers):
            seen.append((path, json.loads(json.dumps(body))))
            response = {'input_tokens':100} if path.endswith('count_tokens') else {
                'usage':{'input_tokens':100,'output_tokens':10}, 'content':[]}
            return 200, 'application/json', io.BytesIO(json.dumps(response).encode())
        settings = {**api_settings('claude-sonnet-4-6'), 'max_cost_usd':100, **(overrides or {})}
        if legacy:settings.pop('reasoning_policy',None)
        relay = BoundedClaudeRelay(settings, 'synthetic-secret', 'synthetic-cap', self.root/'requests', transport=transport)
        relay.start()
        try:
            request = urllib.request.Request(relay.url+'/v1/messages',
                data=json.dumps({'model':settings['model'], 'thinking':{'type':'disabled'},
                    'output_config':{'effort':'low'}, 'max_tokens':99999,
                    'messages':[{'role':'user','content':'synthetic local wire check'}]}).encode(),
                headers={'x-api-key':'synthetic-cap','Content-Type':'application/json'})
            try:
                with urllib.request.urlopen(request, timeout=5) as result:status = result.status
            except urllib.error.HTTPError as exc:status = exc.code
            ledger = json.loads(json.dumps(relay.ledger))
        finally:
            relay.close()
        return status, seen, ledger

    def test_teacher_count_and_messages_enable_adaptive_with_inclusive_cap(self):
        status, seen, ledger = self.exercise({'reasoning_policy':'teacher_adaptive_high'})
        self.assertEqual(status, 200)
        self.assertEqual([p for p,_ in seen], ['/v1/messages/count_tokens','/v1/messages'])
        self.assertTrue(all(b['thinking'] == {'type':'adaptive'} for _,b in seen))
        self.assertEqual(seen[-1][1]['output_config']['effort'], 'high')
        self.assertEqual(seen[-1][1]['max_tokens'], 8192)
        self.assertEqual(ledger['output_tokens_reserved'], 8192)
        expected = (356*3*2+8192*15)/1e6
        self.assertAlmostEqual(ledger['cost_reserved_usd'], expected)

    def test_budgeted_teacher_reserves_space_for_action_output(self):
        status, seen, ledger = self.exercise()
        self.assertEqual(status,200)
        self.assertTrue(all(b['thinking']=={'type':'enabled','budget_tokens':1024} for _,b in seen))
        self.assertEqual(seen[-1][1]['max_tokens'],8192)
        self.assertEqual(ledger['output_tokens_reserved'],8192)

    def test_legacy_explicit_disabled_behavior_remains(self):
        status, seen, _ = self.exercise(legacy=True)
        self.assertEqual(status, 200)
        self.assertTrue(all(b['thinking'] == {'type':'disabled'} for _,b in seen))

    def test_teacher_reasoning_cannot_bypass_cost_rejection(self):
        status, seen, ledger = self.exercise({'max_cost_usd':0})
        self.assertEqual(status, 400)
        self.assertEqual([p for p,_ in seen], ['/v1/messages/count_tokens'])
        self.assertEqual(ledger['inference_requests'], 0)

    def test_unknown_reasoning_policy_is_rejected_before_transport(self):
        with self.assertRaises(ValueError):
            BoundedClaudeRelay({**api_settings('claude-sonnet-4-6'),'reasoning_policy':'unreviewed'},
                               'synthetic','synthetic',self.root/'requests')

    def test_existing_nine_legacy_budget_guards(self):
        settings = {**api_settings('claude-sonnet-4-6'), 'max_cost_usd':100,
                    'max_output_tokens_per_request':512, 'reasoning_policy':'disabled'}
        checks = legacy_guards(self.root/'legacy', settings)
        self.assertEqual(len(checks), 9)
        self.assertTrue(all(checks.values()))


if __name__ == '__main__':
    unittest.main()
