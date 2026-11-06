"""CPU-only baseline/community-controller smoke comparison; no core changes."""
from pathlib import Path
from contextlib import nullcontext
import sys, json, time, hashlib, csv, gzip
import numpy as np
import mujoco
from PIL import Image
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from environment import FoldingBoxEnv
from test_support import Validity, dense, robot_name
from adapter import RightArmImpedance
HERE = Path(__file__).resolve().parent
OUT = ROOT / 'validation/community_control'
SOURCES = ['environment.py', 'models/scene.xml', 'physics.json', 'robot_config.json', 'video_rebuild/config.json']


def run_case(kind, route):
    out = OUT / (kind + '_' + route)
    out.mkdir(parents=True, exist_ok=True)
    env = FoldingBoxEnv(placement='flat')
    model, data = env.model, env.data
    original_step = mujoco.mj_step
    phase = 'setup'
    active = False
    frames = []
    force_rows = []
    metrics = {}
    first_contact = None
    callbacks = None
    substeps = 0
    base_id = model.geom('card_base_0').id
    hand_ids = {i for i in range(model.ngeom) if (model.geom(i).name or '').startswith('jaw_pad_R_') or ('dexmate_R_gripper_' in (model.geom(i).name or '') and '_collision_' in (model.geom(i).name or ''))}
    pad_ids = [i for i in hand_ids if model.geom(i).name.startswith('jaw_pad_R_')]

    def clearance():
        base = data.body('box_base')
        values = []
        for geom in pad_ids:
            mesh = model.geom_dataid[geom]
            start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            world = model.mesh_vert[start:start+count] @ data.geom_xmat[geom].reshape(3,3).T + data.geom_xpos[geom]
            local = (world-base.xpos) @ base.xmat.reshape(3,3)
            assert (abs(local[:,0]) < .15).all() and (abs(local[:,1]) < .11).all()
            values.append(float(local[:,2].min())-.002)
        return min(values)

    def observed_step(m, d, *args, **kwargs):
        nonlocal first_contact, substeps
        original_step(m, d, *args, **kwargs)
        substeps += 1
        peak = total = penetration = 0.
        for index, contact in enumerate(d.contact):
            ids = [int(contact.geom1), int(contact.geom2)]
            names = [m.geom(i).name or '' for i in ids]
            if not any(robot_name(n) for n in names):
                continue
            force = np.zeros(6)
            mujoco.mj_contactForce(m, d, index, force)
            intended = base_id in ids and any(i in hand_ids for i in ids)
            if not intended and (force[0] > .05 or contact.dist < -.0001):
                raise RuntimeError('Unexpected robot contact: '+str((names, float(force[0]), float(contact.dist))))
            if intended:
                if kind != 'touch' or not active:
                    if force[0] > .01:
                        raise RuntimeError('Unexpected early box contact')
                peak = max(peak, float(force[0]))
                total += max(0., float(force[0]))
                penetration = max(penetration, -float(contact.dist))
                if phase == 'descent' and force[0] > .01 and first_contact is None:
                    first_contact = {'time_s': float(d.time), 'normal_N': float(force[0]), 'geoms': names}
        item = metrics.setdefault(phase, {'peak_normal_N':0., 'peak_sum_normal_N':0., 'max_penetration_m':0.})
        item['peak_normal_N'] = max(item['peak_normal_N'], peak)
        item['peak_sum_normal_N'] = max(item['peak_sum_normal_N'], total)
        item['max_penetration_m'] = max(item['max_penetration_m'], penetration)
        if active:
            force_rows.append([float(d.time), phase, peak, total, penetration])
        if total > 5 or penetration > .001:
            raise RuntimeError('5N/1mm trip threshold exceeded')
        arm = env.arms['right']
        q = d.qpos[arm['qad']]
        if np.any(q < arm['limits'][:,0]-.005) or np.any(q > arm['limits'][:,1]+.005):
            raise RuntimeError('Right arm joint limit violation')
        if d.warning.number.any() or not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():
            raise RuntimeError('MuJoCo warning or nonfinite dynamics')

    def tick(target=None, joints=None):
        if joints is not None:
            env.step_joint_targets({'right': joints})
        elif target is None:
            env.step()
        else:
            action = {s:np.r_[env.targets[s],env.grippers[s]['command']] for s in env.arms}
            action['right'] = np.r_[target, .3]
            env.step(action)
        if active:
            frames.append({'time_s': float(data.time), 'phase': phase, 'tcp': env.tcp_pose('right').tolist(), 'target':env.targets['right'].tolist(), 'qpos':data.qpos[env.arms['right']['qad']].tolist(), 'ctrl':data.ctrl[env.arms['right']['act']].tolist()})

    def image(name):
        Image.fromarray(env.render_camera('back')).save(out/(name+'.png'))

    report = {'kind':kind, 'route':route, 'result':'not_completed'}
    mujoco.mj_step = observed_step
    wall_start = time.monotonic()
    try:
        for _ in range(25): tick()
        if kind == 'touch':
            path_file = HERE/'approach_path.json'
            path = json.loads(path_file.read_text())
            valid = Validity(env)
            assert all(valid(q) for q in dense(path))
            report['ompl_path_sha256'] = hashlib.sha256(path_file.read_bytes()).hexdigest()
            phase = 'ompl_path_replay'
            for q in dense(path, .008): tick(joints=q)
            for _ in range(50): tick(joints=env.arms['right']['qcmd'].copy())
            phase = 'near_surface'
            target = env.targets['right'].copy()
            for _ in range(500):
                if clearance() <= .002: break
                target[2] -= .0001
                tick(target)
            else: raise RuntimeError('No near-surface start')
            for _ in range(50): tick(target)
            report['initial_pad_clearance_m'] = clearance()
            assert .0005 < clearance() < .004
        start_pose = env.tcp_pose('right').copy()
        report['initial_tcp'] = start_pose.tolist()
        report['initial_state_sha256'] = hashlib.sha256(data.qpos.tobytes()+data.qvel.tobytes()+data.ctrl.tobytes()).hexdigest()
        image('before')
        context = RightArmImpedance(env) if route == 'community' else nullcontext()
        with context as controller:
            callbacks = controller
            active = True
            if kind == 'move5mm':
                phase = 'movement'
                target = start_pose.copy()
                for k in range(1,51):
                    t = k/50
                    target[2] = start_pose[2]+.005*(10*t**3-15*t**4+6*t**5)
                    tick(target)
                phase = 'hold'
                for _ in range(100): tick(target)
                report['commanded_displacement_m'] = [0.,0.,.005]
                report['actual_displacement_m'] = (env.tcp_pose('right')[:3]-start_pose[:3]).tolist()
                report['final_tcp_error_m'] = float(np.linalg.norm(env.tcp_pose('right')[:3]-target[:3]))
                assert report['final_tcp_error_m'] < .001, '5mm motion tracking error exceeds1mm'
            else:
                phase = 'descent'
                target = env.targets['right'].copy()
                z0 = target[2]
                for k in range(1,601):
                    target[2] = z0-k*.0005*env.control_dt
                    tick(target)
                    if first_contact is not None: break
                if first_contact is None: raise RuntimeError('No contact within6mm descent')
                report['first_contact'] = first_contact
                report['reaction_delay_s'] = float(data.time)-first_contact['time_s']
                image('contact')
                phase = 'contact_hold'
                for _ in range(50): tick(target)
                report['hold_target'] = target.tolist()
                report['tcp_after_hold'] = env.tcp_pose('right').tolist()
                phase = 'retract'
                initial = target.copy()
                for k in range(1,51):
                    target = initial.copy()
                    target[2] += .01*k/50
                    tick(target)
                phase = 'released_hold'
                for _ in range(25): tick(target)
                report['released'] = not any(row[3] > .01 for row in force_rows[-80:])
                assert report['released']
            image('after')
            if controller is not None: assert controller.calls > 0, 'Community callback never executed'
            report['result'] = 'pass'
    except Exception as exc:
        report['result'] = 'fail'
        report['error'] = repr(exc)
        try: image('stopped')
        except Exception: pass
    finally:
        mujoco.mj_step = original_step
        report.update({'wall_seconds':time.monotonic()-wall_start, 'phase_metrics':metrics, 'physics_substeps':substeps, 'warnings':data.warning.number.tolist(), 'simulation_time_s':float(data.time), 'source_sha256':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in SOURCES}, 'mujoco_version':mujoco.__version__, 'numpy_version':np.__version__, 'controller':callbacks.metadata() if callbacks is not None else {'type':'Existing baseline IK/joint PD'}})
        report['peak_contact_normal_N'] = max((p['peak_normal_N'] for p in metrics.values()), default=0.)
        report['max_hand_base_penetration_m'] = max((p['max_penetration_m'] for p in metrics.values()), default=0.)
        (out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
        with gzip.open(out/'force_trace.csv.gz','wt',newline='') as f:
            w=csv.writer(f);w.writerow(['time_s','phase','max_normal_N','sum_normal_N','penetration_m']);w.writerows(force_rows)
        with gzip.open(out/'state_trace.json.gz','wt') as f: json.dump(frames,f)
        env.close()
    print(json.dumps({'kind':kind,'route':route,'result':report['result'],'error':report.get('error'),'tracking_m':report.get('final_tcp_error_m'),'peak_force_N':report['peak_contact_normal_N'],'wall_seconds':report['wall_seconds']},ensure_ascii=False),flush=True)
    return report


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((HERE/'UPSTREAM.json').read_text())
    for entry in manifest['files'].values():
        assert hashlib.sha256((HERE/entry['local']).read_bytes()).hexdigest() == entry['sha256']
    fixture = manifest['planning_fixture']
    assert hashlib.sha256((HERE/fixture['file']).read_bytes()).hexdigest() == fixture['sha256']
    reports = [run_case(kind, route) for kind in ['move5mm','touch'] for route in ['baseline','community']]
    pairs_equal = {}
    for kind in ['move5mm', 'touch']:
        values = [r.get('initial_state_sha256') for r in reports if r['kind']==kind]
        pairs_equal[kind] = len(values)==2 and None not in values and len(set(values))==1
    callback_restored = mujoco.get_mjcb_control() is None
    passed = all(r['result']=='pass' for r in reports) and all(pairs_equal.values()) and callback_restored
    report = {'result':'pass' if passed else 'fail',
              'experiment_source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [HERE/'adapter.py', HERE/'compare.py', HERE/'test_support.py', HERE/'approach_path.json']},
              'upstream':manifest, 'matched_initial_states':pairs_equal,
              'control_callback_restored':callback_restored, 'cases':reports,
              'limits':'Single deterministic smoke per controller/task. Pinned controller reused unchanged; adapter maps7right-arm DOFs and clamps motor torques. Zero desired twist/acceleration. No GPU, cuRobo, model/physics edits, or force-regulation claim.'}
    (OUT/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    print('COMPARISON', report['result'], pairs_equal, flush=True)
    if not passed:
        raise SystemExit(1)


if __name__=='__main__': main()
