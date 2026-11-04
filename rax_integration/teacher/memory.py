"""Durable evidence-linked teacher memory, not a teacher policy or robot runner.

Verification artifacts must be produced by the trusted local integration, outside
the model-editable workspace. This store validates their structure and content
identity; it cannot independently establish that an artifact's measurements are
true. Unit-test fixtures never constitute physical robot validation.
"""
from __future__ import annotations
from contextlib import contextmanager
import copy,fcntl,hashlib,json,os,re,threading,time,uuid
from pathlib import Path
from .identity import canonical,identity_id,evidence_reference,reference_valid
from .bundles import persist,freeze_workspace

class IntegrityError(RuntimeError):pass
class StaleEnvironmentError(RuntimeError):pass

def _digest(value):return hashlib.sha256(canonical(value)).hexdigest()
def _slug(value):
    if not isinstance(value,str) or not re.fullmatch('[a-z][a-z0-9]*(?:-[a-z0-9]+)*',value):raise ValueError('Expected lowercase hyphenated skill slug')
    return value
def _text(value,label):
    if not isinstance(value,str) or not value.strip():raise ValueError(label+' must be nonempty text')
    return value
def _source(value):
    if not isinstance(value,dict) or set(value)!={'kind','sha256','evidence_path'}:raise ValueError('Explicit execution source kind/sha256/evidence_path required')
    if value['kind'] not in ('manual_tool_trace','rax_bundle'):raise ValueError('Unknown execution source kind')
    if not isinstance(value['sha256'],str) or not re.fullmatch('[a-f0-9]{64}',value['sha256']):raise ValueError('Execution source requires a SHA256 identity')
    _text(value['evidence_path'],'Execution-source evidence')
    return value
def _empty():
    return {'schema_version':1,'environment_id':None,'identities':{},'experiments':{},'claims':{},'progress':{},'progress_history':[],'skills':{}}

class MemoryStore:
    """Operator-owned store. All mutations are serialized and journaled.

    `identity` is supplied by integration and should include scene, physics,
    controllers, success evaluator, observation/world policy and tool interfaces.
    `evidence_root` is trusted operator evidence, never an agent-writable directory.
    """
    def __init__(self,root,identity,*,evidence_root):
        self.root=Path(root).absolute();self.evidence_root=Path(evidence_root).absolute()
        self.identity=copy.deepcopy(identity);self.environment_id=identity_id(identity)
        self.root.mkdir(parents=True,exist_ok=True,mode=0o700)
        if self.root.is_symlink():raise ValueError('Memory root must not be a symlink')
        self._mutex=threading.RLock();self.journal=self.root/'events.jsonl';self.state_file=self.root/'state.json'
        with self._locked():
            state,revision,head=self._recover()
            if state['environment_id']!=self.environment_id:
                self._append(state,revision,head,'environment',{'identity':self.identity,'environment_id':self.environment_id})

    @contextmanager
    def _locked(self):
        with self._mutex:
            fd=os.open(self.root/'.lock',os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
            try:
                fcntl.flock(fd,fcntl.LOCK_EX);yield
            finally:fcntl.flock(fd,fcntl.LOCK_UN);os.close(fd)

    def _reduce(self,state,kind,payload):
        if kind=='environment':
            previous=state['environment_id']
            if previous!=payload['environment_id']:
                if state['progress']:state['progress_history'].append(copy.deepcopy(state['progress']))
                state['progress']={'environment_id':payload['environment_id'],'notes':{},'status':'no_current_progress',
                                   'prior_environment_id':previous}
            state['environment_id']=payload['environment_id'];state['identities'][payload['environment_id']]=payload['identity']
        elif kind=='experiment':state['experiments'][payload['id']]=payload
        elif kind=='claim':state['claims'][payload['id']]=payload
        elif kind=='progress':state['progress']=payload
        elif kind=='skill':
            previous=state['skills'].get(payload['id'],{})
            state['skills'][payload['id']]={**payload,'verifications':previous.get('verifications',[])}
        elif kind=='verification':state['skills'][payload['skill_id']]['verifications'].append(payload)
        else:raise IntegrityError('Unknown journal event: '+kind)

    def _recover(self):
        state=_empty();revision=0;head='0'*64
        raw=self.journal.read_bytes() if self.journal.exists() else b''
        # Incomplete final writes were never committed. Preserve them before repair.
        if raw and not raw.endswith(b'\n'):
            end=raw.rfind(b'\n')+1;tail=raw[end:]
            recovery=self.root/'recovery';recovery.mkdir(mode=0o700,exist_ok=True)
            path=recovery/(uuid.uuid4().hex+'.uncommitted-tail')
            fd=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
            with os.fdopen(fd,'wb') as f:f.write(tail);f.flush();os.fsync(f.fileno())
            with self.journal.open('r+b') as f:f.truncate(end);f.flush();os.fsync(f.fileno())
            raw=raw[:end]
        for line in raw.splitlines():
            try:
                event=json.loads(line);body={k:event[k] for k in ('seq','previous','kind','payload','wall_unix_s')}
                if event['seq']!=revision+1 or event['previous']!=head or event['hash']!=_digest(body):raise IntegrityError('Journal hash/sequence mismatch')
                self._reduce(state,event['kind'],event['payload']);revision=event['seq'];head=event['hash']
            except (ValueError,KeyError,TypeError) as exc:raise IntegrityError('Malformed committed journal entry') from exc
        snapshot={'revision':revision,'head':head,'state':state}
        previous=None
        if self.state_file.exists():
            try:previous=json.loads(self.state_file.read_text())
            except (ValueError,OSError):pass
            if isinstance(previous,dict) and previous.get('revision',0)>revision:
                raise IntegrityError('Snapshot is ahead of durable journal; refusing data loss')
        if previous!=snapshot:persist(self.state_file,snapshot)
        return state,revision,head

    def _append(self,state,revision,head,kind,payload):
        payload=json.loads(canonical(payload))
        body={'seq':revision+1,'previous':head,'kind':kind,'payload':payload,'wall_unix_s':time.time()}
        event={**body,'hash':_digest(body)};line=canonical(event)+b'\n'
        fd=os.open(self.journal,os.O_WRONLY|os.O_APPEND|os.O_CREAT|os.O_NOFOLLOW,0o600)
        with os.fdopen(fd,'ab') as f:f.write(line);f.flush();os.fsync(f.fileno())
        self._reduce(state,kind,payload)
        persist(self.state_file,{'revision':revision+1,'head':event['hash'],'state':state})
        return copy.deepcopy(payload)

    def _change(self,kind,build):
        with self._locked():
            state,revision,head=self._recover()
            if state['environment_id']!=self.environment_id:raise StaleEnvironmentError('Another session activated a different environment')
            payload=build(state)
            return self._append(state,revision,head,kind,payload) if payload is not None else None

    def recover(self):
        with self._locked():
            state,revision,head=self._recover()
            return {'revision':revision,'head':head,'environment_id':state['environment_id']}

    def snapshot(self):
        with self._locked():
            state,revision,head=self._recover()
            if state['environment_id']!=self.environment_id:raise StaleEnvironmentError('This store instance no longer owns the active environment identity')
            return {'revision':revision,'head':head,'state':self._knowledge_view(state)}

    def _knowledge_view(self,state):
        """Projection only: never relabel historical claims as current facts."""
        view=copy.deepcopy(state)
        for collection in ('claims','experiments'):
            for item in view[collection].values():
                current=item['environment_id']==self.environment_id
                item['current_environment']=current
                item['applicability']='current_environment' if current else 'historical_environment_only'
        for item in view['progress_history']:item['applicability']='historical_progress_only'
        return view

    def _references(self,paths):
        if not isinstance(paths,(list,tuple)):raise ValueError('Evidence paths must be a list')
        return [evidence_reference(self.evidence_root,p) for p in dict.fromkeys(paths)]

    def record_experiment(self,record):
        payload=copy.deepcopy(record)
        _text(payload.get('hypothesis'),'Experiment hypothesis')
        if payload.get('outcome') not in ('pass','fail','inconclusive','unsupported'):raise ValueError('Explicit diagnostic outcome required')
        payload['evidence']=self._references(payload.pop('evidence_paths',[]))
        payload['environment_id']=self.environment_id;payload.setdefault('id',uuid.uuid4().hex)
        _text(payload['id'],'Experiment id')
        def build(state):
            old=state['experiments'].get(payload['id'])
            if old is not None:
                if old!=payload:raise ValueError('Experiment id already has different contents')
                return None
            return payload
        return self._change('experiment',build) or payload

    def record_claim(self,*,kind,text,experiment_ids=(),evidence_paths=()):
        if kind not in ('fact','hypothesis'):raise ValueError('Claim kind must be fact or hypothesis')
        _text(text,'Claim');refs=self._references(evidence_paths)
        if kind=='fact' and not experiment_ids and not refs:raise ValueError('A fact requires supporting evidence or recorded experiments')
        payload={'kind':kind,'text':text,'experiment_ids':list(experiment_ids),'evidence':refs,'environment_id':self.environment_id}
        payload['id']=_digest(payload)
        def build(state):
            if any(i not in state['experiments'] for i in experiment_ids):raise ValueError('Unknown supporting experiment')
            if kind=='fact' and any(state['experiments'][i]['environment_id']!=self.environment_id for i in experiment_ids):raise StaleEnvironmentError('A current fact cannot inherit experiments from a stale environment')
            if payload['id'] in state['claims']:return None
            return payload
        return self._change('claim',build) or payload

    def update_progress(self,progress):
        if not isinstance(progress,dict):raise ValueError('Progress must be a mapping')
        # Operator notes are never interpreted as evaluator/skill success.
        return self._change('progress',lambda state:{'environment_id':self.environment_id,'notes':progress,'status':'current'})

    def register_skill(self,spec):
        spec=copy.deepcopy(spec);slug=_slug(spec.get('id'))
        for key in ('description','preconditions','procedure','expected_effect'):_text(spec.get(key),key)
        checks=spec.get('success_predicates')
        if not isinstance(checks,list) or not checks or not all(isinstance(k,str) and k for k in checks) or len(set(checks))!=len(checks):raise ValueError('Named success predicates required')
        if spec.get('scope') not in ('free_space','grasp','fold','physical_lock','full_task'):raise ValueError('Explicit skill scope required')
        if 'source_identity' in spec:_source(spec['source_identity'])
        payload={'id':slug,'spec':spec,'version_sha256':_digest(spec),'registered_environment_id':self.environment_id}
        def build(state):
            previous=state['skills'].get(slug)
            if previous and previous['version_sha256']==payload['version_sha256']:return None
            return payload
        self._change('skill',build)
        return payload

    def verify_skill(self,skill_id,evidence_path):
        """Consume an operator-validator artifact; model assertions never suffice."""
        slug=_slug(skill_id);ref=evidence_reference(self.evidence_root,evidence_path)
        artifact=json.loads((self.evidence_root/ref['path']).read_text())
        # Recheck after reading to prevent accepting a changed artifact by accident.
        if not reference_valid(self.evidence_root,ref):raise IntegrityError('Verification artifact changed while reading')
        def build(state):
            skill=state['skills'].get(slug)
            if skill is None:raise ValueError('Unknown skill')
            if artifact.get('environment_id')!=self.environment_id:raise StaleEnvironmentError('Verification environment mismatch')
            if artifact.get('skill_id')!=slug or artifact.get('skill_version_sha256')!=skill['version_sha256']:raise ValueError('Verification skill/version mismatch')
            source=_source(artifact.get('source_identity'))
            if source!=skill['spec'].get('source_identity'):raise ValueError('Candidate and executed source/bundle identities differ or are unbound')
            source_ref=evidence_reference(self.evidence_root,source['evidence_path'])
            if source['kind']=='manual_tool_trace':
                if source_ref['sha256']!=source['sha256']:raise ValueError('Executed manual trace hash mismatch')
            else:
                manifest=json.loads((self.evidence_root/source_ref['path']).read_text())
                if manifest.get('sha256')!=source['sha256'] or not isinstance(manifest.get('files'),dict) or not manifest['files']:
                    raise ValueError('Frozen RAX bundle manifest does not match executed bundle')
                if not reference_valid(self.evidence_root,source_ref):raise IntegrityError('Bundle manifest changed while reading')
            if type(artifact.get('passed')) is not bool:raise ValueError('Measured pass/fail is required')
            if artifact.get('method')!='robot_execution':raise ValueError('Only actual robot execution may verify a skill')
            _text(artifact.get('episode_id'),'Verification episode')
            steps=artifact.get('step_ids')
            if not isinstance(steps,list) or len(steps)<2 or not all(type(s) is int and s>=0 for s in steps) or any(a>=b for a,b in zip(steps,steps[1:])):raise ValueError('Increasing actual step ids required')
            checks=artifact.get('checks');required=skill['spec']['success_predicates']
            if not isinstance(checks,dict) or not set(required)<=set(checks) or not all(type(x) is bool for x in checks.values()):raise ValueError('Measured skill predicate checks required')
            if artifact.get('level') not in ('local_effect','task_complete'):raise ValueError('Explicit confirmation level required')
            if artifact['passed']:
                flags=artifact.get('prohibited',{})
                for key in ('object_state_writes','artificial_attachment','box_joint_actuation','playback'):
                    if flags.get(key) is not False:raise ValueError('Prohibited execution method or missing audit: '+key)
                if not all(checks.values()):raise ValueError('Failed predicates cannot verify success')
                if skill['spec']['scope'] in ('physical_lock','full_task') or artifact['level']=='task_complete':
                    native=artifact.get('native_evaluator',{})
                    if native.get('task_complete') is not True:raise ValueError('Native evaluator confirmation required')
            paths=artifact.get('evidence_paths',[])
            if not paths:raise ValueError('Trace/measurement evidence required')
            refs=self._references(paths)
            if source_ref not in refs:refs.append(source_ref)
            return {'skill_id':slug,'skill_version_sha256':skill['version_sha256'],'environment_id':self.environment_id,
                    'passed':artifact['passed'],'level':artifact['level'],'episode_id':artifact['episode_id'],
                    'episode_mode':artifact.get('episode_mode','diagnostic'),'step_ids':steps,
                    'frame_indices':artifact.get('frame_indices',[]),'observed_effect':artifact.get('observed_effect'),
                    'checks':checks,'artifact':ref,'evidence':refs,'source_identity':source,'trust':'operator-validator attestation; not independently replayed'}
        return self._change('verification',build)

    def skills(self,*,verified_only=False,source_identity=None):
        entries=self._skills_from_state(self.snapshot()['state'])
        if source_identity is not None:
            _source(source_identity)
            for item in entries:
                if item['spec'].get('source_identity')!=source_identity:item['verified']=False;item['status']='stale_execution_source'
        return [item for item in entries if item['verified']] if verified_only else entries

    def _skills_from_state(self,state):
        result=[]
        for slug,skill in sorted(state['skills'].items()):
            item=copy.deepcopy(skill);item['verified']=False;item['status']='candidate';item['current_verification']=None
            same_version=[v for v in skill['verifications'] if v['skill_version_sha256']==skill['version_sha256']]
            matching=[v for v in same_version if v['environment_id']==self.environment_id]
            if matching:
                latest=matching[-1]
                item['current_verification']=copy.deepcopy(latest)
                if not skill['spec'].get('source_identity') or latest.get('source_identity')!=skill['spec']['source_identity']:item['status']='unbound_execution_source'
                elif not all(reference_valid(self.evidence_root,r) for r in [latest['artifact'],*latest['evidence']]):item['status']='evidence_changed_or_missing'
                else:item['verified']=latest['passed'];item['status']='verified' if latest['passed'] else 'refuted'
            elif same_version:item['status']='stale_environment'
            elif skill['verifications']:item['status']='changed_skill'
            result.append(item)
        return result

    def checkpoint(self):
        with self._locked():
            state,revision,head=self._recover()
            if state['environment_id']!=self.environment_id:raise StaleEnvironmentError('Cannot checkpoint a stale store instance')
            path=self.root/'checkpoints'/f'{revision:08d}-{head}.json';path.parent.mkdir(exist_ok=True,mode=0o700)
            persist(path,{'revision':revision,'head':head,'state':state})
            return {'path':str(path),'revision':revision,'head':head,'environment_id':state['environment_id']}

    def export_workspace(self,destination):
        """Fresh RAX memory/handoff tree; referenced evidence travels with notes."""
        destination=Path(destination)
        if destination.exists():raise ValueError('Export destination must be fresh')
        with self._locked():
            state,revision,head=self._recover()
            if state['environment_id']!=self.environment_id:raise StaleEnvironmentError('Environment changed before export')
            destination.mkdir(parents=True,mode=0o700);memory=destination/'memory';memory.mkdir();handoff=destination/'handoff';handoff.mkdir()
            store=memory/'store';store.mkdir();persist(store/'state.json',{'revision':revision,'head':head,'state':state})
            (store/'events.jsonl').write_bytes(self.journal.read_bytes())
            skills=self._skills_from_state(state);index=['| Skill | Tags | Confirmation | Last episode |','|---|---|---|---|'];tips=['| Task phase | Preconditions | What worked | Visible effect | Evidence |','|---|---|---|---|---|']
            def clean(x):return str(x).replace('|','/').replace('\n',' ')
            for item in skills:
                slug=item['id'];spec=item['spec'];folder=memory/'skills'/slug;folder.mkdir(parents=True)
                current=item['current_verification']
                confirmation=('task-confirmed' if current['level']=='task_complete' else 'observed') if item['verified'] else item['status']
                index.append(f"| skills/{slug}/SKILL.md | {clean(spec['scope'])} | {confirmation} | {clean(current['episode_id'] if current else 'none')} |")
                skill_text=f"---\nname: learned-{slug}\ndescription: {json.dumps(spec['description'],ensure_ascii=False)}\n---\n\n# {slug}\n\n- Confirmation: {confirmation}\n- Applies when: {spec['preconditions']}\n- Executed action: {json.dumps(spec.get('executed_action','See pinned operator trace'),ensure_ascii=False)}\n- Pose recipe: {json.dumps(spec.get('pose_recipe','Unrecorded; reacquire current observed features'),ensure_ascii=False)}\n- Actions: {spec['procedure']}\n- Expected effect: {spec['expected_effect']}\n- Scope: {spec['scope']}\n- Environment identity: {self.environment_id}\n"
                examples=[]
                for v in item['verifications']:
                    examples.append({'task':spec.get('task','dexmate_box_folding'),'episode_id':v['episode_id'],
                        'episode_mode':v.get('episode_mode','diagnostic'),
                        'confirmation':('task-confirmed' if v['level']=='task_complete' else 'observed') if v['passed'] else 'refuted',
                        'manifest_path':v['artifact']['path'],'step_ids':v.get('step_ids',[]),'frame_indices':v.get('frame_indices',[]),
                        'action_kind':spec.get('action_kind','unrecorded'),'executed_action':spec.get('executed_action'),
                        'pose_recipe':spec.get('pose_recipe'),'point_reacquisition':spec.get('point_reacquisition'),
                        'observed_effect':v.get('observed_effect') or 'Operator predicate report: '+json.dumps(v['checks'],sort_keys=True),
                        'operator_attestation':v})
                (folder/'SKILL.md').write_text(skill_text);persist(folder/'evidence.json',{'schema_version':4,'skill':'learned-'+slug,'examples':examples,'verified_in_current_environment':item['verified'],'mode_extension':'diagnostic is local validation, not exploration/rehearsal success'})
                if item['verified']:tips.append(f"| {clean(spec['scope'])} | {clean(spec['preconditions'])} | {clean(spec['procedure'])} | {clean(spec['expected_effect'])} | skills/{slug}/evidence.json |")
            (memory/'INDEX.md').write_text('\n'.join(index)+'\n');(memory/'TIPS.md').write_text('\n'.join(tips)+'\n')
            progress_status = 'current_environment' if state['progress'].get('environment_id') == self.environment_id else 'stale_or_empty_progress_revalidate_before_use'
            (memory/'PROGRESS.md').write_text('Current environment: '+self.environment_id+'\nProgress status: '+progress_status+'\n\n'+json.dumps(state['progress'],indent=2)+'\n\nFacts/hypotheses and experiment outcomes are stored separately; neither implies verified task success.\n')
            persist(memory/'structured.json',self._knowledge_view(state))
            (handoff/'README.md').write_text('# Teacher memory handoff\n\nEnvironment: '+self.environment_id+'\nMemory revision: '+str(revision)+'\n\nRead memory/PROGRESS.md and memory/INDEX.md. Use only matching verified skills; reacquire observable features and check live status before acting. Do not restart an episode after recovery. Candidate, refuted and stale entries are not success evidence. This package starts no robot, model, rehearsal or evaluation.\n\nCurrent progress (operator notes, not evaluator truth; status: '+progress_status+'):\n'+json.dumps(state['progress'],indent=2)+'\n')
            # Copy pinned evidence only, not whole recordings or arbitrary workspace files.
            references=[]
            for experiment in state['experiments'].values():references+=experiment['evidence']
            for claim in state['claims'].values():references+=claim['evidence']
            for item in state['skills'].values():
                for verification in item['verifications']:references+=[verification['artifact'],*verification['evidence']]
            current_refs={(r['path'],r['sha256']) for item in skills if item['verified'] for r in [item['current_verification']['artifact'],*item['current_verification']['evidence']]}
            missing=[]
            for ref in references:
                if not reference_valid(self.evidence_root,ref):
                    if (ref['path'],ref['sha256']) in current_refs:raise IntegrityError('Active skill evidence changed during export')
                    # Historical missing evidence is explicitly listed, never promoted.
                    missing.append(ref)
                    continue
                target=memory/'evidence'/ref['path'];target.parent.mkdir(parents=True,exist_ok=True)
                raw=(self.evidence_root/ref['path']).read_bytes()
                if hashlib.sha256(raw).hexdigest()!=ref['sha256']:raise IntegrityError('Evidence changed during export')
                target.write_bytes(raw)
            persist(memory/'evidence_status.json',{'historical_missing_or_changed':missing,'current_verified_evidence_copied':True})
            return {'workspace':str(destination),'environment_id':self.environment_id,'revision':revision,'head':head}

    def freeze_bundle(self,destination,*,max_mb=10):
        destination=Path(destination);staging=self.root/'exports'/uuid.uuid4().hex
        info=self.export_workspace(staging)
        manifest=freeze_workspace(staging,destination,environment_id=info['environment_id'],revision=info['revision'],max_mb=max_mb)
        return {'manifest':manifest,**info,'bundle':str(destination)}
