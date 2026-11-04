"""Read-only motor telemetry. This module never changes commands or physics.

Control-row samples describe the last integration substep, not substep peaks.
The commissioning-only sampler below additionally records the exact controller
inputs immediately before every call to the original mj_step.
"""
from contextlib import AbstractContextManager
import time

import mujoco
import numpy as np


def motor_snapshot(env):
    out = {}
    for side, arm in env.arms.items():
        ids = arm['act']
        out[side] = {
            'joint_names': [env.model.joint(int(j)).name for j in arm['joint_ids']],
            'q_rad': env.data.qpos[arm['qad']].tolist(),
            'velocity_rad_s': env.data.qvel[arm['vid']].tolist(),
            'reference_rad': arm['qcmd'].tolist(),
            'motor_ctrl': env.data.ctrl[ids].tolist(),
            'actuator_force_Nm': env.data.actuator_force[ids].tolist(),
            'ctrlrange': env.model.actuator_ctrlrange[ids].tolist(),
            'ctrllimited': env.model.actuator_ctrllimited[ids].tolist(),
            'forcerange': env.model.actuator_forcerange[ids].tolist(),
            'forcelimited': env.model.actuator_forcelimited[ids].tolist(),
            'joint_velocity_limit_rad_s': arm['velocity'].tolist(),
        }
    return out


class SubstepMotorSampler(AbstractContextManager):
    """Single-process diagnostic hook, restored even after an execution exception.

    The wrapper calls the same original function once with unchanged arguments.
    It filters by model/data identity and cannot be nested. No other simulation
    or worker should run concurrently in the diagnostic process.
    """
    def __init__(self, env):
        self.env = env
        self.samples = []
        self.original = None

    def __enter__(self):
        if getattr(mujoco.mj_step, '_commissioning_sampler', False):
            raise RuntimeError('A commissioning substep sampler is already active')
        self.original = mujoco.mj_step
        def step(model, data, *args, **kwargs):
            if model is self.env.model and data is self.env.data:
                entry = {'sim_time_s': float(data.time), 'wall_monotonic_s': time.monotonic(), 'arms': {}}
                kp = np.asarray(self.env.cfg['joint_kp_Nm_rad'])
                kd = np.asarray(self.env.cfg['joint_kd_Nms_rad'])
                for side, arm in self.env.arms.items():
                    raw = kp*(arm['qcmd']-data.qpos[arm['qad']])-kd*data.qvel[arm['vid']]+data.qfrc_bias[arm['vid']]
                    entry['arms'][side] = {
                        'reference_rad': arm['qcmd'].tolist(),
                        'measured_rad': data.qpos[arm['qad']].tolist(),
                        'velocity_rad_s': data.qvel[arm['vid']].tolist(),
                        'pd_plus_bias_unclipped_Nm': raw.tolist(),
                        'motor_ctrl': data.ctrl[arm['act']].tolist(),
                        'ctrlrange': model.actuator_ctrlrange[arm['act']].tolist(),
                    }
                self.samples.append(entry)
            return self.original(model, data, *args, **kwargs)
        step._commissioning_sampler = True
        self.wrapper = step
        mujoco.mj_step = step
        return self

    def __exit__(self, *exc):
        if self.original is not None:
            mujoco.mj_step = self.original
        return False
