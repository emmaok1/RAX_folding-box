"""Trusted bridge from serialized teacher MCP calls to the durable operator store."""
import json
from pathlib import Path



EXPERIMENT_OUTCOMES = ('pass', 'fail', 'inconclusive', 'unsupported')
EXPERIMENT_EXAMPLE = {'id':'approach-001', 'hypothesis':'A bounded approach reaches its measured goal',
                      'outcome':'inconclusive', 'evidence_paths':[]}

def validate_memory_request(args):
    """Give actionable errors before touching durable state; never infer success."""
    payload = args.get('payload', {})
    if not isinstance(payload, dict):
        raise ValueError('payload must be an object')
    if args.get('operation', 'read') == 'record_experiment':
        if not isinstance(payload.get('hypothesis'), str) or not payload['hypothesis'].strip():
            raise ValueError('record_experiment requires payload.hypothesis: a nonempty string. Example: '+json.dumps(EXPERIMENT_EXAMPLE))
        if payload.get('outcome') not in EXPERIMENT_OUTCOMES:
            raise ValueError('record_experiment requires payload.outcome: pass | fail | inconclusive | unsupported. '
                             'Do not use success, passed, or diagnostic_outcome. Example: '+json.dumps(EXPERIMENT_EXAMPLE))
        if 'experiment_id' in payload:
            raise ValueError('record_experiment uses payload.id for an idempotent record ID; experiment_id is only for read lookup. Example: '+json.dumps(EXPERIMENT_EXAMPLE))
        if 'id' in payload and (not isinstance(payload['id'], str) or not payload['id'].strip()):
            raise ValueError('record_experiment payload.id must be a nonempty string')
        paths = payload.get('evidence_paths', [])
        if not isinstance(paths, list) or any(not isinstance(path, str) for path in paths):
            raise ValueError('record_experiment payload.evidence_paths must be a list of relative evidence_id strings; use [] only for an explicitly unsupported/diagnostic note')


def store_for_backend(backend):
    from .memory import MemoryStore
    policy = Path(backend.config.root)/'teacher_memory_identity.json'
    if not policy.exists():
        raise RuntimeError('Teacher memory identity was not prepared by the operator')
    identity = json.loads(policy.read_text())
    if identity.get('world_environment_id') != backend.teacher_world_id:
        raise RuntimeError('Teacher memory/world identity mismatch')
    return MemoryStore(Path(backend.config.root)/'teacher_memory', identity,
                       evidence_root=Path(backend.config.root)/'native/episodes')


def call_memory(backend, args):
    validate_memory_request(args)
    store = store_for_backend(backend)
    operation, payload = args.get('operation', 'read'), args.get('payload', {})
    if operation == 'read':
        snap = store.snapshot()
        state = snap['state']
        if payload:
            if set(payload) != {'experiment_id'} or payload['experiment_id'] not in state['experiments']:
                raise ValueError('Read payload accepts one existing experiment_id')
            return {'experiment': state['experiments'][payload['experiment_id']]}
        # Bound API context growth. Full records remain in the journal and can be
        # fetched individually; truncation is explicit, never a success claim.
        def brief(value, depth=0):
            if isinstance(value, str): return value if len(value)<=1200 else value[:1200]+' [truncated]'
            if depth >= 4: return '[detail omitted; query experiment_id or read stored notes]'
            if isinstance(value, dict): return {k:brief(v,depth+1) for k,v in list(value.items())[:20]}
            if isinstance(value, list): return [brief(v,depth+1) for v in value[:12]]
            return value
        result = {'revision': snap['revision'], 'environment_id': store.environment_id,
                  'memory_environment_id':store.environment_id, 'world_environment_id':backend.teacher_world_id,
                  'progress': brief(state['progress']),
                  'progress_current_environment': state['progress'].get('environment_id') == store.environment_id,
                  'historical_entries_require_environment_check': True,
                  'skills': brief(store.skills()[-12:]),
                  'recent_experiments': brief(list(state['experiments'].values())[-12:]),
                  'recent_claims': brief(list(state['claims'].values())[-12:]),
                  'summary_only': True, 'total_experiments': len(state['experiments']),
                  'detail_lookup': 'read with payload={experiment_id: id}',
                  'write_contract': {'record_experiment': {'required':['hypothesis','outcome'], 'outcome_enum':list(EXPERIMENT_OUTCOMES), 'id_field':'id (optional stable key; not experiment_id)', 'example':EXPERIMENT_EXAMPLE},
                                     'authority':'diagnostic records do not verify skills; Markdown notes are separate from this durable store'}}
        while len(json.dumps(result)) > 64000:
            key = max(('skills','recent_experiments','recent_claims'), key=lambda k:len(json.dumps(result[k])))
            if not result[key]: break
            result[key].pop(0)
        return result
    if operation == 'record_experiment':
        result = store.record_experiment(payload)
    elif operation == 'record_claim':
        result = store.record_claim(**payload)
    elif operation == 'update_progress':
        result = store.update_progress(payload)
    elif operation == 'register_skill':
        result = store.register_skill(payload)
    elif operation == 'checkpoint':
        result = store.checkpoint()
    else:
        raise ValueError('Unsupported teacher memory operation')
    backend.log('teacher_memory', operation=operation)
    # No model-callable verify_skill: operator measurements qualify a skill.
    return {'operation': operation, 'persisted': True, 'memory_environment_id':store.environment_id,
            'record': result, 'skill_verification': 'operator_only'}
