"""Read-only privileged state from the current owned episode, not historical exports."""
import hashlib
import json
import numpy as np
import mujoco
from services.json_io import jsonable


def _pose(data, body):
    return {'position_m': data.xpos[body].tolist(), 'quaternion_wxyz': data.xquat[body].tolist()}


def contact_snapshot(env):
    data, model = env.data, env.model
    contacts = []
    for index in range(data.ncon):
        contact = data.contact[index]
        wrench = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, wrench)
        geoms = [int(contact.geom1), int(contact.geom2)]
        contacts.append({'geoms': [model.geom(g).name for g in geoms],
                         'bodies': [model.body(int(model.geom_bodyid[g])).name for g in geoms],
                         'position_world_m': contact.pos.tolist(),
                         'signed_distance_m': float(contact.dist),
                         'contact_frame_rows_world': contact.frame.reshape(3, 3).tolist(),
                         'force_contact_frame_N': wrench[:3].tolist(),
                         'torque_contact_frame_Nm': wrench[3:].tolist()})
    return contacts


def teacher_state(backend):
    if not backend.contract.teacher_privileged:
        raise PermissionError('Teacher ground-truth capability required')
    env, data, model = backend.env, backend.env.data, backend.env.model
    robot = {}
    for side, arm in env.arms.items():
        jaw = env.grippers[side]
        robot[side] = {
            'measured': {'joint_positions_rad': data.qpos[arm['qad']].tolist(),
                         'joint_velocities_rad_s': data.qvel[arm['vid']].tolist(),
                         'tcp_world_xyz_wxyz': env.tcp_pose(side).tolist(),
                         'jaw_positions_rad': data.qpos[jaw['qad']].tolist(),
                         'jaw_velocities_rad_s': data.qvel[jaw['vid']].tolist()},
            'references': {'joint_positions_rad': arm['qcmd'].tolist(),
                           'tcp_world_xyz_wxyz': np.asarray(env.targets[side]).tolist(),
                           'jaw_position_rad': float(jaw['qcmd']),
                           'opening_fraction': float(jaw['command'])},
            'joint_limits_rad': arm['limits'].tolist(),
            'tcp_in_gripper_base_xyz_wxyz': [0., 0., .16, 1., 0., 0., 0.]}
    panels = []
    for spec in env.geometry['panels']:
        name = spec['panel']
        body = model.body('box_'+name).id
        rotation = data.xmat[body].reshape(3, 3)
        origin = np.asarray(spec['origin_flat_m'])
        def polygon(points):
            xyz = np.column_stack([np.asarray(points), np.zeros(len(points))])
            return ((xyz-origin)@rotation.T+data.xpos[body]).tolist()
        entry = {'panel': name, 'body_name': 'box_'+name,
                 'pose_world': _pose(data, body), 'com_world_m': data.xipos[body].tolist(),
                 'normal_world': rotation[:, 2].tolist(),
                 'outline_midplane_world_m': polygon(spec['polygon_flat_m']),
                 'holes_midplane_world_m': [polygon(hole) for hole in spec['holes_flat_m']],
                 'target_angle_rad': spec['target_rad']}
        if name != 'base':
            joint = model.joint('crease_'+name)
            entry['hinge'] = {'joint_name': joint.name,
                              'angle_rad': float(data.qpos[int(joint.qposadr[0])]),
                              'velocity_rad_s': float(data.qvel[int(joint.dofadr[0])]),
                              'pivot_world_m': data.xanchor[joint.id].tolist(),
                              'axis_world': data.xaxis[joint.id].tolist(),
                              'range_rad': model.jnt_range[joint.id].tolist()}
        panels.append(entry)
    contacts = contact_snapshot(env)
    cameras = {}
    for name in ('back', 'head', 'handcam_rgb_left', 'handcam_rgb_right'):
        index = model.camera(name).id
        cameras[name] = {'position_world_m': data.cam_xpos[index].tolist(),
                         'rotation_camera_to_world': data.cam_xmat[index].reshape(3, 3).tolist(),
                         'vertical_fov_deg': float(model.cam_fovy[index]),
                         'axes': 'MuJoCo camera local +x right, +y up, looks along -z',
                         'render_width_height_px': [env.render_spec.width, env.render_spec.height]}
    source = getattr(backend, 'source_hashes', {})
    source_id = hashlib.sha256(json.dumps(source, sort_keys=True).encode()).hexdigest()
    # Existing progress() maintains a release timer; a read-only observation must
    # not mutate it merely by asking for privileged state.
    release_since = env._release_since
    try:
        progress = env.progress()
    finally:
        env._release_since = release_since
    packet = {'schema': 'rax.teacher.live_state.v1', 'privilege': 'trusted_simulator_ground_truth',
              'observation_kind': 'live_owned_episode', 'episode_id': backend.episode_id,
              'step_id': backend.step_id, 'sim_time_s': float(data.time),
              'episode_ended': backend.finished, 'source_fingerprint': source_id,
              'environment_id': getattr(backend, 'teacher_world_id', None),
              'world_environment_id': getattr(backend, 'teacher_world_id', None),
              'memory_environment_id': getattr(backend, 'teacher_memory_id', None),
              'frames': {'world': 'right-handed MuJoCo world, z up', 'length': 'metres',
                         'angles': 'radians', 'quaternions': 'active local-to-world wxyz'},
              'robot': robot, 'box_free_pose_world_xyz_wxyz': data.joint('box_free').qpos.tolist(),
              'box_free_velocity_native': data.joint('box_free').qvel.tolist(),
              'box_velocity_semantics': 'MuJoCo free-joint native qvel: world translation, local angular velocity.',
              'panels': panels, 'contacts': contacts, 'cameras': cameras,
              'contact_semantics': 'Current sampled solver contacts, not a history or grasp/retention proof.',
              'physical_progress': progress, 'control_feedback': backend._control_feedback(),
              'limitations': ['Simulator truth is not calibrated hardware truth.',
                             'Current evaluator withholds physical-locking success.',
                             'Body/site poses are the latest engine-computed kinematics; no forward or integration is triggered by this read.',
                             'No force closure, stable grasp or successful folding inferred from one sample.']}
    # Strict serialization is also the finite-number check at the privilege boundary.
    packet = jsonable(packet)
    json.dumps(packet, allow_nan=False)
    return packet
