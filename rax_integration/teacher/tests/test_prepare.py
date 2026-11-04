"""Normal RAX workspace preparation with a tiny explicitly mocked world snapshot.
No simulator/backend/agent process or API request is started by these checks.
"""
import hashlib,json,os,tempfile,unittest
from pathlib import Path
from unittest import mock
from rax_integration.teacher.prepare import prepare_session,load_prepared,memory_for,checkpoint_session,publish_memory
from rax_integration.teacher.run import command_for,paid_preflight
from rax_integration.teacher.world import digest

def tiny_world(project,destination):
    destination=Path(destination);destination.mkdir();raw=b'# synthetic preparation fixture, not a simulator scene\n'
    (destination/'fixture.txt').write_bytes(raw);files={'fixture.txt':hashlib.sha256(raw).hexdigest()}
    manifest={'environment_id':digest(files),'files':files};(destination/'manifest.json').write_text(json.dumps(manifest));return manifest

class PreparationChecks(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);self.base=Path(temp.name)
        env=mock.patch.dict(os.environ,{'RAX_ARTIFACT_ROOT':str(self.base)});env.start();self.addCleanup(env.stop)
        self.patch=mock.patch('rax_integration.teacher.prepare.create_world_snapshot',side_effect=tiny_world);self.patch.start();self.addCleanup(self.patch.stop)
        files={'fixture.txt':hashlib.sha256(b'# synthetic preparation fixture, not a simulator scene\n').hexdigest()}
        source=mock.patch('rax_integration.teacher.prepare.world_sources',return_value=files);source.start();self.addCleanup(source.stop)
    def prepare(self,name='run',**kwargs):
        return prepare_session(self.base/name,**kwargs)
    def test_normal_workspace_capability_skills_and_no_template_tokens(self):
        cfg,deployed=self.prepare();self.assertTrue(cfg.teacher_privileged);self.assertFalse(cfg.agent_formal);self.assertTrue(cfg.write_handoff)
        workspace=deployed['workspace']
        for name in ['experience-memory','autonomous-control','transfer-handoff','free-space-move','object-localization','learn-from-incidents']:
            self.assertTrue((workspace/'.agents/skills'/name/'SKILL.md').is_file(),name)
        unresolved=[]
        for p in workspace.rglob('*.md'):
            if any(x in p.read_text() for x in ('{{','{%')):unresolved.append(str(p.relative_to(workspace)))
        self.assertEqual(unresolved,[])
        self.assertFalse(json.loads((cfg.root/'cloud_authorization.json').read_text())['allowed'])
        self.assertTrue((workspace/'runtime/world').is_symlink())
        self.assertFalse((cfg.root/'state.json').exists(),'No frontend or episode may start during preparation')
        loaded,_=load_prepared(cfg.root);self.assertTrue(loaded.teacher_privileged)
    def test_normal_agent_command_preserves_file_code_tools_and_bounds(self):
        cfg,deployed=self.prepare();limits=json.loads((cfg.root/'teacher_policy.json').read_text())['api_limits']
        command=command_for(cfg,deployed,limits)
        self.assertIn('--restricted',command);self.assertEqual(command[command.index('--tools')+1],'Read,Write,Edit');self.assertNotIn('--allowedTools',command)
        self.assertEqual(cfg.agent_effort,'high');self.assertIn('Bash',command)
        self.assertIn('--strict-mcp-config',command);self.assertIn('--max-turns',command);self.assertIn('--max-budget-usd',command)
        self.assertIn('mcp__auto_research__submit',command)
    def test_no_paid_grant_refuses_before_transport(self):
        cfg,_=self.prepare(budget_usd=10)
        scope=self.base/'scope';scope.mkdir();(scope/'CURRENT_SCOPE.json').write_text(json.dumps({'cloud_inference_allowed':False,'attempt_consumed':True}))
        with mock.patch('rax_integration.teacher.run.HERE',scope):
            with self.assertRaises(PermissionError):paid_preflight(cfg.root,10)
        self.assertFalse((cfg.root/'requests').exists());self.assertFalse((cfg.root/'agent.pid').exists())
    def test_policy_tampering_invalidates_preparation(self):
        cfg,_=self.prepare();p=cfg.root/'teacher_policy.json';p.chmod(0o600);p.write_text('{}')
        with self.assertRaises(ValueError):load_prepared(cfg.root)
    def test_grant_must_cover_all_caps_without_starting_anything(self):
        cfg,_=self.prepare(budget_usd=10);scope=self.base/'scope';scope.mkdir()
        (scope/'CURRENT_SCOPE.json').write_text(json.dumps({'cloud_inference_allowed':True,'attempt_consumed':False,'approved_budget_usd':10,'authorization_task_id':'SYNTHETIC-PREFLIGHT'}))
        auth={'allowed':True,'consumed':False,'max_cost_usd':10,'task_id':'SYNTHETIC-PREFLIGHT','user_authorization':'synthetic gate test, NOT a real grant','max_requests':40,'max_wall_seconds':600,'max_episode_starts':3}
        with mock.patch('rax_integration.teacher.run.HERE',scope):
            for key,value in [('max_requests',39),('max_wall_seconds',599),('max_episode_starts',2)]:
                (cfg.root/'cloud_authorization.json').write_text(json.dumps({**auth,key:value}))
                with self.assertRaises(PermissionError):paid_preflight(cfg.root,10)
            (cfg.root/'cloud_authorization.json').write_text(json.dumps(auth));self.assertEqual(paid_preflight(cfg.root,10)[2]['max_cost_usd'],10)
        self.assertFalse((cfg.root/'requests').exists());self.assertFalse((cfg.root/'agent.pid').exists())
    def test_changed_live_world_refuses_before_any_agent_request(self):
        cfg,_=self.prepare()
        with mock.patch('rax_integration.teacher.prepare.world_sources',return_value={'changed':'source'}):
            with self.assertRaises(ValueError):load_prepared(cfg.root)
        self.assertFalse((cfg.root/'requests').exists())
    def test_carry_memory_code_handoff_without_episode_replay(self):
        cfg,deployed=self.prepare();workspace=deployed['workspace'];store=memory_for(cfg.root)
        store.update_progress({'stage':'synthetic carry test','next':'query live status, do not replay'})
        code=workspace/'code/example/controller.py';code.parent.mkdir(parents=True);code.write_text('# SYNTHETIC CODE TRANSFER TEST - not executed\n')
        record=checkpoint_session(cfg.root)
        other,files=self.prepare('resumed',carry_bundle=record['bundle'],carry_manifest=record['manifest'])
        self.assertTrue((files['workspace']/'code/example/controller.py').is_file())
        self.assertEqual(memory_for(other.root).snapshot()['state']['progress']['notes']['stage'],'synthetic carry test')
        self.assertFalse((other.root/'state.json').exists());self.assertFalse((other.root/'requests').exists())

    def test_authored_notes_preserved_during_publication_and_resume(self):
        cfg,files=self.prepare();workspace=files['workspace'];notes={
            'memory/PROGRESS.md':'Authored progress unchanged\n','memory/INDEX.md':'Authored candidate index\n',
            'memory/TIPS.md':'Authored tips unchanged\n','handoff/README.md':'Authored handoff unchanged\n'}
        for name,text in notes.items():(workspace/name).write_text(text)
        store=memory_for(cfg.root);store.update_progress({'stage':'fresh operator progress'});publish_memory(store,workspace)
        for name,text in notes.items():self.assertEqual((workspace/name).read_text(),text)
        pointer=json.loads((workspace/'memory/RESTORE.json').read_text());self.assertFalse(pointer['execution_resumed'])
        self.assertIn('fresh operator progress',(workspace/'memory/OPERATOR_PROGRESS.md').read_text())
        record=checkpoint_session(cfg.root)
        _,resumed=self.prepare('resume_notes',carry_bundle=record['bundle'],carry_manifest=record['manifest'])
        for name,text in notes.items():self.assertEqual((resumed['workspace']/name).read_text(),text)
    def test_failed_publication_retains_previous_completion_pointer(self):
        cfg,files=self.prepare();workspace=files['workspace'];before=(workspace/'memory/RESTORE.json').read_bytes()
        store=memory_for(cfg.root);store.update_progress({'stage':'not yet published'})
        with mock.patch('rax_integration.teacher.prepare._copy_tree',side_effect=OSError('synthetic publication failure')):
            with self.assertRaises(OSError):publish_memory(store,workspace)
        self.assertEqual((workspace/'memory/RESTORE.json').read_bytes(),before)
        publish_memory(store,workspace);self.assertNotEqual((workspace/'memory/RESTORE.json').read_bytes(),before)
    def test_publication_rejects_destination_symlink(self):
        cfg,files=self.prepare();workspace=files['workspace'];outside=self.base/'outside';outside.mkdir()
        target=workspace/'memory/OPERATOR_PROGRESS.md';target.unlink();target.symlink_to(outside/'do-not-write')
        with self.assertRaises(ValueError):publish_memory(memory_for(cfg.root),workspace)
        self.assertFalse((outside/'do-not-write').exists())

if __name__=='__main__':unittest.main(verbosity=2)
