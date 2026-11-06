"""Evidence/persistence tests use synthetic operator artifacts in temporary dirs."""
import copy,hashlib,json,tempfile,unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from unittest import mock
from rax_integration.teacher.memory import MemoryStore,IntegrityError,StaleEnvironmentError
from rax_integration.teacher.identity import capture_identity,identity_id,evidence_reference

IDENTITY={'scene':'test-scene-sha','controller':'test-controller-sha','evaluator':'test-evaluator-sha','world_policy':'test-simulator-snapshot'}
SPEC={'id':'short-transfer','description':'Short unladen free-space transfer','scope':'free_space','preconditions':'empty jaws; clear observed approach','procedure':'plan, preview, execute, measure','expected_effect':'TCP reaches measured target','success_predicates':['goal_reached','no_contact']}
SPEC['source_identity']={'kind':'manual_tool_trace','sha256':hashlib.sha256(b'{"synthetic_unit_fixture":true}').hexdigest(),'evidence_path':'trace.json'}

class MemoryChecks(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup);self.base=Path(self.temp.name)
        self.evidence=self.base/'operator';self.evidence.mkdir();(self.evidence/'trace.json').write_text('{"synthetic_unit_fixture":true}')
        self.root=self.base/'store';self.store=MemoryStore(self.root,IDENTITY,evidence_root=self.evidence)
    def spec(self):return self.store.register_skill(SPEC)
    def artifact(self,**updates):
        skill=self.spec()
        data={'skill_id':SPEC['id'],'skill_version_sha256':skill['version_sha256'],'environment_id':self.store.environment_id,
              'passed':True,'method':'robot_execution','episode_id':'SYNTHETIC-UNIT-FIXTURE','step_ids':[0,20],
              'checks':{'goal_reached':True,'no_contact':True},'level':'local_effect',
              'prohibited':{k:False for k in ['object_state_writes','artificial_attachment','box_joint_actuation','playback']},
              'evidence_paths':['trace.json'],'source_identity':skill['spec']['source_identity']}
        data.update(updates);path=self.evidence/'verification.json';path.write_text(json.dumps(data));return path.name
    def test_persist_resume_idempotency_and_progress(self):
        record={'id':'e1','hypothesis':'Small transfers are repeatable','outcome':'inconclusive','evidence_paths':['trace.json']}
        self.store.record_experiment(record);before=self.store.recover()['revision'];self.store.record_experiment(record)
        self.assertEqual(self.store.recover()['revision'],before)
        self.store.update_progress({'stage':'approach','episode_id':'live-episode-to-resume','next':'inspect current status before action'})
        resumed=MemoryStore(self.root,IDENTITY,evidence_root=self.evidence)
        self.assertEqual(resumed.snapshot()['state']['progress']['notes']['episode_id'],'live-episode-to-resume')
        self.assertEqual(len(resumed.snapshot()['state']['experiments']),1)
        with self.assertRaises(ValueError):self.store.record_experiment({**record,'outcome':'pass'})
    def test_claims_do_not_promote_skills(self):
        self.spec();self.store.record_claim(kind='hypothesis',text='Friction may be limiting')
        self.store.record_claim(kind='fact',text='Trace file exists',evidence_paths=['trace.json'])
        self.assertFalse(self.store.skills()[0]['verified'])
        with self.assertRaises(ValueError):self.store.record_claim(kind='fact',text='The task succeeded')
    def test_verified_scope_and_modified_evidence(self):
        self.store.verify_skill(SPEC['id'],self.artifact());self.assertTrue(self.store.skills()[0]['verified'])
        (self.evidence/'trace.json').write_text('changed')
        self.assertEqual(self.store.skills()[0]['status'],'evidence_changed_or_missing')
        self.assertFalse(self.store.skills()[0]['verified'])
    def test_changed_environment_preserves_history_invalidates_verification(self):
        self.store.verify_skill(SPEC['id'],self.artifact())
        changed=MemoryStore(self.root,{**IDENTITY,'evaluator':'new-rule'},evidence_root=self.evidence)
        self.assertEqual(changed.skills()[0]['status'],'stale_environment');self.assertFalse(changed.skills()[0]['verified'])
        self.assertEqual(len(changed.snapshot()['state']['identities']),2)
        with self.assertRaises(StaleEnvironmentError):self.store.update_progress({'stage':'stale write'})
    def test_environment_change_resets_active_progress_labels_old_claims(self):
        self.store.record_experiment({'id':'prior-experiment','hypothesis':'previous scene works','outcome':'pass'})
        self.store.record_claim(kind='fact',text='Prior diagnostic passed',experiment_ids=['prior-experiment'])
        self.store.update_progress({'stage':'old-environment wall milestone'})
        changed=MemoryStore(self.root,{**IDENTITY,'scene':'changed scene'},evidence_root=self.evidence)
        state=changed.snapshot()['state']
        self.assertEqual(state['progress']['notes'],{});self.assertEqual(state['progress']['status'],'no_current_progress')
        self.assertEqual(state['progress']['environment_id'],changed.environment_id)
        self.assertEqual(state['progress_history'][-1]['notes']['stage'],'old-environment wall milestone')
        self.assertEqual(next(iter(state['claims'].values()))['applicability'],'historical_environment_only')
        self.assertFalse(state['experiments']['prior-experiment']['current_environment'])
        with self.assertRaises(StaleEnvironmentError):self.store.snapshot()
        with self.assertRaises(StaleEnvironmentError):self.store.skills()
        changed.export_workspace(self.base/'changed-notes')
        exported=json.loads((self.base/'changed-notes/memory/structured.json').read_text())
        self.assertFalse(next(iter(exported['claims'].values()))['current_environment'])
        self.assertNotIn('old-environment wall milestone',(self.base/'changed-notes/memory/PROGRESS.md').read_text())
    def test_same_identity_recovery_keeps_progress_current(self):
        self.store.update_progress({'stage':'same-world prior interactive progress'})
        resumed=MemoryStore(self.root,IDENTITY,evidence_root=self.evidence)
        self.assertEqual(resumed.snapshot()['state']['progress']['notes']['stage'],'same-world prior interactive progress')
    def test_changed_skill_invalidates_verification(self):
        self.store.verify_skill(SPEC['id'],self.artifact())
        self.store.register_skill({**SPEC,'procedure':'different action'})
        self.assertEqual(self.store.skills()[0]['status'],'changed_skill')
    def test_debug_replay_attachment_and_actuation_cannot_verify(self):
        for flag in ['object_state_writes','artificial_attachment','box_joint_actuation','playback']:
            with self.subTest(flag=flag):
                p=self.artifact();data=json.loads((self.evidence/p).read_text());data['prohibited'][flag]=True;(self.evidence/p).write_text(json.dumps(data))
                with self.assertRaises(ValueError):self.store.verify_skill(SPEC['id'],p)
        for updates in [{'method':'debug_pose'},{'environment_id':'other-scene'},{'skill_version_sha256':'old-version'},{'checks':{'goal_reached':False,'no_contact':True}},{'step_ids':[20,0]}]:
            with self.subTest(updates=updates):
                with self.assertRaises((ValueError,StaleEnvironmentError)):self.store.verify_skill(SPEC['id'],self.artifact(**updates))
        self.assertFalse(self.store.skills()[0]['verified'])
    def test_task_confirmation_requires_native_evaluator(self):
        with self.assertRaises(ValueError):self.store.verify_skill(SPEC['id'],self.artifact(level='task_complete'))
        with self.assertRaises(ValueError):self.store.verify_skill(SPEC['id'],self.artifact(level='task_complete',native_evaluator={'task_complete':False}))
    def test_refutation_preserves_prior_verification(self):
        self.store.verify_skill(SPEC['id'],self.artifact())
        # Preserve the first artifact under a distinct path before adding contradiction.
        (self.evidence/'first.json').write_bytes((self.evidence/'verification.json').read_bytes())
        self.store.verify_skill(SPEC['id'],self.artifact(passed=False,checks={'goal_reached':False,'no_contact':True}))
        skill=self.store.skills()[0];self.assertEqual(len(skill['verifications']),2);self.assertEqual(skill['status'],'refuted')
    def test_crash_after_journal_commit_before_snapshot_recovers(self):
        with mock.patch('rax_integration.teacher.memory.persist',side_effect=OSError('synthetic crash')):
            with self.assertRaises(OSError):self.store.update_progress({'stage':'durable-event'})
        resumed=MemoryStore(self.root,IDENTITY,evidence_root=self.evidence)
        self.assertEqual(resumed.snapshot()['state']['progress']['notes']['stage'],'durable-event')
        self.assertTrue(Path(resumed.checkpoint()['path']).is_file())
    def test_incomplete_tail_is_preserved_and_repaired(self):
        revision=self.store.recover()['revision']
        with (self.root/'events.jsonl').open('ab') as f:f.write(b'{"incomplete":')
        resumed=MemoryStore(self.root,IDENTITY,evidence_root=self.evidence)
        self.assertEqual(resumed.recover()['revision'],revision)
        self.assertEqual(len(list((self.root/'recovery').glob('*.uncommitted-tail'))),1)
        resumed.update_progress({'stage':'still writable'})
    def test_committed_corruption_is_not_silently_repaired(self):
        p=self.root/'events.jsonl';event=json.loads(p.read_text().splitlines()[0]);event['payload']['identity']['scene']='tampered';p.write_text(json.dumps(event)+'\n')
        with self.assertRaises(IntegrityError):MemoryStore(self.root,IDENTITY,evidence_root=self.evidence)
    def test_concurrent_sessions_do_not_lose_events(self):
        stores=[MemoryStore(self.root,IDENTITY,evidence_root=self.evidence) for _ in range(4)]
        def write(i):stores[i%4].record_experiment({'id':f'e{i}','hypothesis':'Concurrent durable experiment','outcome':'inconclusive'})
        with ThreadPoolExecutor(max_workers=4) as pool:list(pool.map(write,range(24)))
        self.assertEqual(len(self.store.snapshot()['state']['experiments']),24)
        self.assertEqual(self.store.recover()['revision'],25)
    def test_evidence_traversal_and_symlinks_refused(self):
        (self.evidence/'link').symlink_to(self.evidence/'trace.json')
        for path in ['../trace.json',str(self.evidence/'trace.json'),'link']:
            with self.subTest(path=path):
                with self.assertRaises((ValueError,OSError)):evidence_reference(self.evidence,path)
    def test_identity_is_order_independent_and_source_sensitive(self):
        a=capture_identity({'a':self.evidence/'trace.json'},metadata={'policy':'simulation'})
        self.assertEqual(identity_id({'a':1,'b':2}),identity_id({'b':2,'a':1}))
        (self.evidence/'trace.json').write_text('new material')
        self.assertNotEqual(identity_id(a),identity_id(capture_identity({'a':self.evidence/'trace.json'},metadata={'policy':'simulation'})))
    def test_candidate_text_hash_does_not_verify_unknown_or_changed_execution(self):
        p=self.artifact(source_identity={**SPEC['source_identity'],'sha256':'0'*64})
        with self.assertRaises(ValueError):self.store.verify_skill(SPEC['id'],p)
        p=self.artifact();self.store.register_skill({k:v for k,v in SPEC.items() if k!='source_identity'})
        artifact=json.loads((self.evidence/p).read_text());artifact['skill_version_sha256']=self.store.skills()[0]['version_sha256'];(self.evidence/p).write_text(json.dumps(artifact))
        with self.assertRaises(ValueError):self.store.verify_skill(SPEC['id'],p)
        self.assertFalse(self.store.skills()[0]['verified'])
    def test_frozen_code_bundle_binding_and_changed_manifest(self):
        p=self.artifact();source={'kind':'rax_bundle','sha256':'a'*64,'evidence_path':'bundle-manifest.json'}
        (self.evidence/source['evidence_path']).write_text(json.dumps({'sha256':source['sha256'],'files':{'controller.py':'b'*64},'runtime_sha256':'c'*64}))
        skill=self.store.register_skill({**SPEC,'source_identity':source})
        artifact=json.loads((self.evidence/p).read_text());artifact.update(source_identity=source,skill_version_sha256=skill['version_sha256']);(self.evidence/p).write_text(json.dumps(artifact))
        self.store.verify_skill(SPEC['id'],p);self.assertTrue(self.store.skills(source_identity=source)[0]['verified'])
        other={**source,'sha256':'d'*64};self.assertFalse(self.store.skills(source_identity=other)[0]['verified'])
        (self.evidence/source['evidence_path']).write_text('changed frozen code manifest')
        self.assertFalse(self.store.skills()[0]['verified'])

if __name__=='__main__':unittest.main(verbosity=2)
