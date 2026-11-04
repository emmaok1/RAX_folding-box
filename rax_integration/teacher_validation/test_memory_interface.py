"""Real memory bridge/schema regressions; temporary stores, no simulator or cloud."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'upstream'))
from services.dexmate_box.tools import tool_definitions, validate_request
from rax_integration.teacher.memory_api import call_memory

class MemoryInterfaceChecks(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        (self.root/'native/episodes/fixture/teacher_states').mkdir(parents=True)
        (self.root/'native/episodes/fixture/teacher_states/0.json').write_text('{"synthetic":true}')
        (self.root/'teacher_memory_identity.json').write_text(json.dumps({'world_environment_id':'synthetic-world'}))
        self.events = []
        self.backend = SimpleNamespace(config=SimpleNamespace(root=self.root), teacher_world_id='synthetic-world',
                                       log=lambda event, **kw:self.events.append((event, kw)))

    def test_discovery_exposes_exact_outcomes_and_record_id(self):
        tool = next(t for t in tool_definitions() if t['name']=='dexmate_teacher_memory')
        fields = tool['inputSchema']['properties']['payload']['properties']
        self.assertEqual(fields['outcome']['enum'], ['pass','fail','inconclusive','unsupported'])
        self.assertIn('id',fields)
        self.assertIn('READ lookup only',fields['experiment_id']['description'])

    def test_captured_invalid_shapes_give_correction_without_write(self):
        before = call_memory(self.backend, {'operation':'read'})
        invalid = [
            ({'outcome':'success'}, 'hypothesis'),
            ({'hypothesis':'diagnostic','outcome':'success'}, 'payload.outcome'),
            ({'hypothesis':'diagnostic','diagnostic_outcome':'passed'}, 'payload.outcome'),
            ({'hypothesis':'diagnostic','outcome':'pass','experiment_id':'exp001'}, 'payload.id'),
        ]
        for payload, field in invalid:
            with self.subTest(payload=payload), self.assertRaisesRegex(ValueError, field):
                call_memory(self.backend, {'operation':'record_experiment','payload':payload})
        after = call_memory(self.backend, {'operation':'read'})
        self.assertEqual(after['revision'],before['revision'])
        self.assertEqual(after['total_experiments'],0)
        self.assertEqual(self.events,[])

    def test_correct_record_survives_reopen_and_is_idempotent(self):
        args = {'operation':'record_experiment','payload':{
            'id':'diagnostic-001','hypothesis':'synthetic bridge persistence check',
            'outcome':'pass','evidence_paths':['fixture/teacher_states/0.json']}}
        validate_request('dexmate_teacher_memory',args)
        reply = call_memory(self.backend,args)
        self.assertTrue(reply['persisted'])
        self.assertEqual(reply['record']['id'],'diagnostic-001')
        self.assertEqual(reply['skill_verification'],'operator_only')
        revision = call_memory(self.backend,{'operation':'read'})['revision']
        call_memory(self.backend,args)
        read = call_memory(self.backend,{'operation':'read'})
        self.assertEqual(read['revision'],revision)
        self.assertEqual(read['total_experiments'],1)
        self.assertEqual(read['skills'],[])
        detail = call_memory(self.backend,{'operation':'read','payload':{'experiment_id':'diagnostic-001'}})
        self.assertEqual(detail['experiment']['outcome'],'pass')
        self.assertEqual(len(detail['experiment']['evidence']),1)

    def test_state_read_provides_contract_and_example_is_executable(self):
        read = call_memory(self.backend, {'operation':'read'})
        example = read['write_contract']['record_experiment']['example']
        reply = call_memory(self.backend, {'operation':'record_experiment','payload':example})
        self.assertTrue(reply['persisted'])
        self.assertEqual(reply['record']['outcome'],'inconclusive')

    def test_mcp_validator_matches_bridge_refusal(self):
        args = {'operation':'record_experiment','payload':{'hypothesis':'synthetic','outcome':'success'}}
        for fn in [lambda:validate_request('dexmate_teacher_memory',args),lambda:call_memory(self.backend,args)]:
            with self.assertRaisesRegex(ValueError, 'pass.*fail.*inconclusive.*unsupported'):
                fn()

    def test_evidence_escape_and_model_verification_remain_denied(self):
        with self.assertRaises(ValueError):
            call_memory(self.backend, {'operation':'record_experiment','payload':{
                'id':'escape','hypothesis':'diagnostic','outcome':'pass','evidence_paths':['../outside']}})
        with self.assertRaises(Exception):
            validate_request('dexmate_teacher_memory',{'operation':'verify_skill','payload':{}})
        self.assertEqual(call_memory(self.backend,{'operation':'read'})['total_experiments'],0)
