"""Offline mechanical smoke checks; does not certify material fidelity or locking."""
import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from environment import FoldingBoxEnv, ROOT


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def check_state(env):
    require(all(np.isfinite(getattr(env.data, n)).all() for n in ('qpos', 'qvel', 'qacc')), 'Nonfinite dynamics')
    require(not env.data.warning.number.any(), 'MuJoCo dynamics warning')


def settle(env, seconds):
    penetration = 0.
    for _ in range(round(seconds / env.control_dt)):
        env.step()
        check_state(env)
        for contact in env.data.contact:
            names = [env.model.geom(int(i)).name or '' for i in (contact.geom1, contact.geom2)]
            if any(n.startswith('card_') for n in names):
                penetration = max(penetration, -float(contact.dist))
    return penetration


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'validation/baseline.json')
    args = parser.parse_args()
    report = {}
    env = FoldingBoxEnv(placement='flat')
    try:
        m, d = env.model, env.data
        bodies = [m.body('box_' + n).id for n in env.panels]
        joints = [m.joint('crease_' + n).id for n in env.panels if n != 'base']
        require(len(bodies) == 14 and len(joints) == 13, 'Unexpected panel/crease count')
        expected = sum(p['area_m2'] for p in env.geometry['panels']) * env.physical_params['basis_weight_g_m2'] / 1000
        require(np.isclose(sum(m.body_mass[bodies]), expected, atol=1e-10), 'Mass does not match net area')
        require((m.body_inertia[bodies] > 0).all(), 'Nonpositive panel inertia')
        require((m.body_inertia[bodies].sum(axis=1)[:, None] + 1e-12 >= 2 * m.body_inertia[bodies]).all(), 'Nonphysical principal inertia')
        require(np.allclose(m.opt.gravity, [0, 0, -9.81]), 'Unexpected gravity')
        for joint_id in m.actuator_trnid[:, 0]:
            require(int(joint_id) not in joints and int(joint_id) != m.joint('box_free').id, 'Actuator drives cardboard')
        require(m.neq == 2 and all((m.equality(i).name or '').startswith('coupling_') for i in range(m.neq)), 'Unexpected equality constraints')
        for geom_id in range(m.ngeom):
            if (m.geom(geom_id).name or '').startswith('card_'):
                require(bool(m.geom_contype[geom_id] and m.geom_conaffinity[geom_id]), 'Disabled cardboard collision')
        require(not env.geometry['grasp_features'], 'Unexpected grasp aids')
        report['structure'] = {'panels': 14, 'passive_creases': 13, 'mass_kg': float(expected), 'physics_dt_s': m.opt.timestep}

        before = {n: getattr(d, n).copy() for n in ('qpos', 'qvel', 'ctrl')}
        env.step()
        env.reset()
        require(all(np.array_equal(v, getattr(d, n)) for n, v in before.items()), 'Nondeterministic reset')
        report['reset'] = 'pass'

        peak = settle(env, 10.)
        progress = env.progress()
        require(progress['stabilized'] and progress['table_panel_contacts'] > 0, 'Blank not supported by table')
        require(peak < .001, 'Excessive resting penetration')
        require(not progress['succeed'] and not progress['physical_locking_verified'], 'Unverified success claim')
        coarse = d.qpos.copy()
        report['rest_10s'] = {'max_penetration_m': peak, 'table_contacts': progress['table_panel_contacts'], 'warnings': d.warning.number.tolist()}
        env.reset()
        m.opt.timestep /= 2
        env._n_substeps *= 2
        settle(env, 10.)
        error = float(np.max(np.abs(d.qpos - coarse)))
        require(error < .002, 'Rest state fails half-timestep sensitivity check')
        report['half_timestep_rest'] = {'max_qpos_difference_m_or_rad': error}
        m.opt.timestep *= 2
        env._n_substeps //= 2

        env.reset()
        d.joint('box_free').qpos[2] += .05  # Test initial condition only.
        mujoco.mj_forward(m, d)
        start = float(d.joint('box_free').qpos[2])
        env.step()
        velocity = float(d.joint('box_free').qvel[2])
        require(abs(velocity + 9.81 * env.control_dt) < .002, 'Free fall does not follow gravity')
        require(d.joint('box_free').qpos[2] < start, 'Dropped blank does not fall')
        peak = settle(env, 3.)
        require(env.progress()['stabilized'], 'Dropped blank does not settle')
        require(peak < .002, 'Excessive drop penetration')
        report['gravity_and_drop'] = {'velocity_after_20ms_m_s': velocity, 'expected_m_s': -9.81 * env.control_dt, 'max_penetration_m': peak}

        env.reset()
        joint = m.joint('crease_body_front')
        initial = float(d.joint('crease_body_front').qpos[0])
        d.qfrc_applied[int(joint.dofadr[0])] = .06  # Physical external torque, not a pose command.
        settle(env, .2)
        angle = float(d.joint('crease_body_front').qpos[0])
        d.qfrc_applied[:] = 0
        require(angle > initial + .05, 'Passive crease did not fold under external torque')
        peak = settle(env, 3.)
        require(peak < .002, 'Excessive penetration after crease torque')
        report['external_torque_folding'] = {'torque_Nm': .06, 'duration_s': .2, 'angle_change_rad': angle - initial, 'max_release_penetration_m': peak}
        frame = env.render_camera('back')
        require(frame.shape == (480, 640, 3) and frame.std() > 5, 'Rendering failed')
        report['render'] = {'shape': list(frame.shape), 'pixel_std': float(frame.std())}

        manifest = json.loads((ROOT / 'RAX_UPSTREAM.json').read_text())
        actual = {str(p.relative_to(ROOT / 'rax')): hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT / 'rax').rglob('*') if p.is_file() and '__pycache__' not in p.parts}
        require(actual == manifest['files_sha256'], 'RAX differs from pinned upstream')
        report['rax'] = {'commit': manifest['commit'], 'verified_files': len(actual)}
        report['source_sha256'] = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in ('models/scene.xml', 'physics.json', 'robot_config.json', 'video_rebuild/config.json', 'environment.py')}
        report['result'] = 'pass'
        report['limits'] = 'Numerical and mechanical smoke checks only; real material fidelity, full assembly and locking unverified.'
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
    finally:
        env.close()


if __name__ == '__main__':
    main()
