"""Small CPU-only adapter around pinned, unmodified community impedance control.

The callback replaces only right-arm motor commands during an experiment.
The original environment continues driving the left arm and both grippers.
No simulation state, model parameters, or collision settings are written.
"""
from contextlib import AbstractContextManager
import numpy as np
import mujoco
from vendor.control import GeometricImpedanceController


class RightArmImpedance(AbstractContextManager):
    def __init__(self, env):
        self.env = env
        self.arm = env.arms['right']
        self.controller = GeometricImpedanceController(
            env.model, env.data, 'tcp_R', self.arm['vid'])
        self.controller.posture_reference = env.data.qpos[self.arm['qad']].copy()
        self.calls = 0
        self.saturated_substeps = 0
        self.peak_raw_torque_Nm = 0.
        self.peak_applied_torque_Nm = 0.
        self.previous_callback = None

    def control(self, model, data):
        # Forward kinematics on scratch data must not command execution motors.
        if data is not self.env.data:
            return
        target = self.env.targets['right']
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, target[3:7])
        zero = np.zeros(3)
        # Desired velocities/accelerations are zero at each20ms setpoint.
        # This intentionally tests pose tracking, not upstream trajectory feedforward.
        torque = self.controller.torque(target[:3], rotation.reshape(3, 3),
                                        zero, zero, zero, zero)
        if torque.shape != (7,) or not np.isfinite(torque).all():
            raise RuntimeError('Nonfinite or invalid community torque output')
        limits = model.actuator_ctrlrange[self.arm['act']]
        applied = np.clip(torque, limits[:, 0], limits[:, 1])
        self.calls += 1
        self.saturated_substeps += int(np.any(applied != torque))
        self.peak_raw_torque_Nm = max(self.peak_raw_torque_Nm, float(np.max(abs(torque))))
        self.peak_applied_torque_Nm = max(self.peak_applied_torque_Nm, float(np.max(abs(applied))))
        data.ctrl[self.arm['act']] = applied

    def __enter__(self):
        self.previous_callback = mujoco.get_mjcb_control()
        if self.previous_callback is not None:
            raise RuntimeError('Another MuJoCo control callback is active')
        mujoco.set_mjcb_control(self.control)
        return self

    def __exit__(self, *args):
        mujoco.set_mjcb_control(self.previous_callback)

    def metadata(self):
        return {
            'upstream_class': 'GeometricImpedanceController',
            'controlled_joint_names': [self.env.model.joint(int(i)).name for i in self.arm['joint_ids']],
            'controlled_dofs': self.arm['vid'].tolist(),
            'tool_site': 'tcp_R', 'torso': 'fixed in existing scene',
            'translation_gain': np.diag(self.controller.Kp).tolist(),
            'rotation_gain': np.diag(self.controller.KR).tolist(),
            'damping_gain': np.diag(self.controller.Kd).tolist(),
            'posture_gain': self.controller.posture_gain,
            'posture_damping': self.controller.posture_damping,
            'reference_policy': 'Fixed activation posture;20ms pose setpoints, zero desired twist/acceleration',
            'torque_limits_Nm': self.env.model.actuator_ctrlrange[self.arm['act']].tolist(),
            'callback_calls': self.calls,
            'saturated_substeps': self.saturated_substeps,
            'peak_raw_torque_Nm': self.peak_raw_torque_Nm,
            'peak_applied_torque_Nm': self.peak_applied_torque_Nm,
        }
