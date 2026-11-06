"""Actual upstream pack/verify/unpack; no evaluation driver is launched."""
import json,unittest
from pathlib import Path
from rax_integration.teacher.memory import MemoryStore
from rax_integration.teacher.bundles import restore_workspace,verify
from rax_integration.teacher.tests.test_memory import MemoryChecks,IDENTITY,SPEC

class BundleChecks(MemoryChecks):
    # Only these bundle-specific cases are selected by load_tests below.
    def test_bundle_resume_carries_notes_evidence_and_journal(self):
        self.store.verify_skill(SPEC['id'],self.artifact())
        self.store.update_progress({'stage':'verified local effect; task unverified','next':'check current episode status'})
        result=self.store.freeze_bundle(self.base/'bundle')
        manifest=result['manifest'];verify(self.base/'bundle',manifest)
        restore_workspace(self.base/'bundle',manifest,self.base/'restored')
        memory=self.base/'restored/memory'
        resumed=MemoryStore(memory/'store',IDENTITY,evidence_root=memory/'evidence')
        self.assertTrue(resumed.skills()[0]['verified'])
        self.assertTrue((memory/'INDEX.md').is_file());self.assertTrue((memory/'skills/short-transfer/SKILL.md').is_file())
        self.assertIn('check current episode status',(memory/'PROGRESS.md').read_text())
        self.assertTrue((self.base/'restored/handoff/README.md').is_file())
    def test_bundle_changed_evidence_is_rejected_before_restore(self):
        self.store.verify_skill(SPEC['id'],self.artifact());result=self.store.freeze_bundle(self.base/'bundle')
        p=self.base/'bundle/transfer/memory/INDEX.md';p.chmod(0o644);p.write_text('tampered')
        with self.assertRaises(ValueError):restore_workspace(self.base/'bundle',result['manifest'],self.base/'restored')
        self.assertFalse((self.base/'restored').exists())
    def test_incomplete_bundle_not_registered_and_restore_never_merges(self):
        with self.assertRaises(ValueError):self.store.freeze_bundle(self.base/'too-small',max_mb=0)
        self.assertFalse((self.base/'too-small.manifest.json').exists())
        result=self.store.freeze_bundle(self.base/'valid');target=self.base/'occupied';target.mkdir();(target/'existing').write_text('preserve')
        with self.assertRaises(ValueError):restore_workspace(self.base/'valid',result['manifest'],target)
        self.assertEqual((target/'existing').read_text(),'preserve')
    def test_export_selects_matching_environment_verification(self):
        self.store.verify_skill(SPEC['id'],self.artifact())
        other=MemoryStore(self.root,{**IDENTITY,'scene':'other-scene'},evidence_root=self.evidence)
        data=json.loads((self.evidence/'verification.json').read_text());data['environment_id']=other.environment_id;data['episode_id']='OTHER-ENV-TEST'
        (self.evidence/'other.json').write_text(json.dumps(data));other.verify_skill(SPEC['id'],'other.json')
        original=MemoryStore(self.root,IDENTITY,evidence_root=self.evidence)
        original.export_workspace(self.base/'notes')
        index=(self.base/'notes/memory/INDEX.md').read_text()
        self.assertIn('SYNTHETIC-UNIT-FIXTURE',index);self.assertNotIn('OTHER-ENV-TEST',index)

def load_tests(loader,tests,pattern):
    names=['test_bundle_resume_carries_notes_evidence_and_journal','test_bundle_changed_evidence_is_rejected_before_restore','test_incomplete_bundle_not_registered_and_restore_never_merges','test_export_selects_matching_environment_verification']
    return unittest.TestSuite(BundleChecks(n) for n in names)

if __name__=='__main__':unittest.main(verbosity=2)
