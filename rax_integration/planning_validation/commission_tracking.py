"""Stage A: current full-scene motor commissioning of frozen accepted paths.

No cloud, planner search, object-state writes or controller changes. Historical
joint references are proposals, independently collision-checked against the
current full scene before admission. This is controller commissioning, not a
new model-driven manipulation success. Slow variants use the SAME cached path
and cursor with <=50-row calls; all current guards remain enabled.
"""
from pathlib import Path
import argparse
import hashlib
import json
import time
import uuid

import numpy as np

from environment import FoldingBoxEnv
from rax_integration.motion_planning.model import ROOT, source_epoch, source_hash, digest
from rax_integration.motion_planning.collision import CollisionChecker
from rax_integration.motion_planning.service import MotionService
from rax_integration.motion_planning.telemetry import SubstepMotorSampler
from rax_integration.motion_planning.limits import plan_rows,execute_rows

HISTORY = ROOT/'rax_integration/runs/teacher_fullfold_20261104T001821'
FIRST = '0b798c3487de45d39dfb8fc6369e74cc'
FAILED = '069fa2ce20ea40b79af3ff2a9e4cc53b'


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False))


def protected_hashes():
    frozen = json.loads(next((HISTORY/'native/episodes').glob('*/source_hashes.json')).read_text())
    return {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in frozen}


def frozen_path(ident):
    directory = HISTORY/'motion_planning'
    query = json.loads((directory/(ident+'.query.json')).read_text())
    result = json.loads((directory/(ident+'.worker.json')).read_text())
    if result['status'] != 'planned':
        raise ValueError('Commission only accepted historical paths')
    return query, result


def slow_rows(start, rows, factor):
    """Time-stretch the SAME reference polyline, retaining every knot exactly."""
    if factor not in (1, 2, 4):
        raise ValueError('Only original/half/quarter speed commissioning is defined')
    result = []
    for row in rows:
        for i in range(1, factor+1):
            result.append((np.asarray(start)+(np.asarray(row)-start)*i/factor).tolist())
        start = np.asarray(row)
    return result


def admit_diagnostic(service, query, result, rows):
    """Operator-only admission, never an MCP bypass or an unvalidated cache edit."""
    snapshot = service.snapshot()
    reason = service.precondition(snapshot)
    if reason:
        return service.refuse(reason)
    if len(rows) > plan_rows(service.cfg):
        return service.refuse('DIAGNOSTIC_ROW_LIMIT')
    side = query['arm']
    checker = CollisionChecker(service.model, side, service.cfg['collision_clearance_m'])
    previous = np.asarray(snapshot['robot'][side]['q'])
    for row in rows:
        if not checker.edge(previous, np.asarray(row), service.cfg, lambda: service.stop_file.exists()):
            return service.refuse('DIAGNOSTIC_PATH_COLLISION', collision=checker.last_conflict)
        previous = np.asarray(row)
    ident = uuid.uuid4().hex
    rows=tuple(tuple(float(v) for v in row) for row in rows)
    accepted = {**result, 'rows': rows, 'duration_s': len(rows)*service.env.control_dt,
                'diagnostic_only': True, 'admission': 'frozen accepted reference proposal; current full-scene edge validation'}
    service.plans[ident] = {
        'state': 'planned', 'cursor':0, 'arm': side, 'target': query['target'], 'snapshot': snapshot,
        'epoch': source_epoch(), 'source_hash': source_hash(),
        'world_hash': service.world_version(snapshot), 'model_hash': service.static_hash,
        'trajectory_hash':digest(rows),
        'result': accepted,
    }
    service.log('diagnostic_admission', plan_id=ident, rows=len(rows),
                historical_source=str(HISTORY), collision_samples=checker.samples,
                label='reference-driven motor commissioning; not cloud planning or folding')
    return {'status': 'planned', 'plan_id': ident}


def summarize(samples):
    result = {}
    for side in ('right', 'left'):
        values = [s['arms'][side] for s in samples]
        if not values:
            continue
        raw = np.asarray([v['pd_plus_bias_unclipped_Nm'] for v in values])
        ctrl = np.asarray([v['motor_ctrl'] for v in values])
        limits = np.asarray(values[0]['ctrlrange'])
        saturated = (raw < limits[:, 0]-1e-9) | (raw > limits[:, 1]+1e-9)
        result[side] = {
            'substeps': len(values), 'peak_abs_ctrl': np.max(abs(ctrl), axis=0).tolist(),
            'peak_abs_unclipped_torque_Nm': np.max(abs(raw), axis=0).tolist(),
            'saturated_substeps_per_joint': np.sum(saturated, axis=0).tolist(),
            'ctrlrange': limits.tolist(),
        }
    return result


def trial(out, factor):
    env = FoldingBoxEnv(placement='flat')
    service = MotionService(env, 'commission_tracking', out, world_policy='simulator_snapshot')
    trace = []
    service.on_step = lambda entry, obs, info: trace.append({**entry, 'physical_progress': info['progress']})
    sampler = SubstepMotorSampler(env)
    first_query, first_result = frozen_path(FIRST)
    query, result = frozen_path(FAILED)
    outcome = {'speed_scale': 1/factor, 'source_history': str(HISTORY), 'cloud_requests': 0,
               'full_original_scene': True, 'controller_and_guards_unchanged': True}
    try:
        with sampler:
            admitted = admit_diagnostic(service, first_query, first_result, first_result['rows'])
            if admitted['status'] != 'planned':
                outcome.update(status='FAIL', initialization=admitted)
                return outcome
            initial = service.execute(admitted['plan_id'])
            outcome['initialization'] = initial
            if not initial['goal_reached']:
                outcome['status'] = 'FAIL'
                return outcome
            rows = slow_rows(query['snapshot']['robot'][query['arm']]['qcmd'], result['rows'], factor)
            outcome['full_reference_rows'] = len(rows)
            outcome['prefix_only'] = False
            admitted = admit_diagnostic(service, query, result, rows)
            if admitted['status'] != 'planned':
                outcome.update(status='FAIL', execution=admitted)
                return outcome
            chunks=[]
            for _ in range(int(np.ceil(len(rows)/execute_rows(service.cfg)))):
                executed=service.execute(admitted['plan_id'],expected_cursor=service.plans[admitted['plan_id']].get('cursor',0))
                chunks.append(executed)
                if executed['status']!='paused':break
            outcome['chunks']=chunks
            outcome['execution'] = executed
            outcome['status'] = 'PASS' if executed['goal_reached'] else 'FAIL'
            outcome['final_physical_progress'] = env.progress()
            return outcome
    finally:
        write(out/'result.json', outcome)
        write(out/'control_rows.json', trace)
        write(out/'motor_summary.json', summarize(sampler.samples))
        with (out/'substeps.jsonl').open('w') as stream:
            for sample in sampler.samples:
                stream.write(json.dumps(sample, allow_nan=False)+'\n')
        env.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    out = args.output or ROOT/'rax_integration/planning_validation/evidence'/('tracking_commission_'+time.strftime('%Y%m%dT%H%M%S', time.gmtime()))
    out.mkdir(parents=True, exist_ok=False)
    before = protected_hashes()
    frozen = json.loads(next((HISTORY/'native/episodes').glob('*/source_hashes.json')).read_text())
    if before != frozen:
        raise RuntimeError('Core source identity differs from frozen run; do not silently replay')
    results = []
    for factor in (1, 2, 4):
        folder = out/('speed_'+str(1/factor))
        folder.mkdir()
        result = trial(folder, factor)
        results.append(result)
        print(json.dumps(result), flush=True)
    after = protected_hashes()
    write(out/'results.json', {'trials': results, 'protected_unchanged': before == after,
                             'protected_hashes': before, 'inference_requests': 0,
                             'limitation': 'Controller commissioning of frozen reference paths, not model task execution. No grasp/folding/locking claim.'})
    print('EVIDENCE', out, flush=True)
    if before != after:
        raise RuntimeError('Protected source changed during commissioning')


if __name__ == '__main__':
    main()
