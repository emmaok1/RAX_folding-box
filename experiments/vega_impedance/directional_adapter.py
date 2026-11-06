"""Optional world-axis gain profile; pinned upstream source stays unmodified."""
import numpy as np
import mujoco
from adapter import RightArmImpedance


class DirectionalRightArmImpedance(RightArmImpedance):
    """Hold worldY/Z more strongly while preserving worldX pushing stiffness.

    The upstream elastic matrix is in the desired tool frame, and damping is
    expressed in the actual tool frame. Transform both explicitly; assigning
    diagonal tool gains would mix the physical axes at the tilted push pose.
    """
    def __init__(self, env):
        super().__init__(env)
        self.stiffness_world = np.diag([900., 2700., 1800.])
        self.damping_world = np.diag(120.*np.sqrt(np.array([1.,3.,2.])))

    def control(self, model, data):
        if data is not self.env.data:
            return
        desired = np.empty(9)
        mujoco.mju_quat2Mat(desired, self.env.targets['right'][3:7])
        desired = desired.reshape(3,3)
        actual = data.site_xmat[self.arm['site']].reshape(3,3)
        self.controller.Kp = desired.T @ self.stiffness_world @ desired
        self.controller.Kd[:3,:3] = actual.T @ self.damping_world @ actual
        super().control(model,data)

    def metadata(self):
        result = super().metadata()
        # The upstream matrices are now dense, so their diagonals alone would
        # be misleading as a description of the gains.
        result.pop('translation_gain')
        result.pop('damping_gain')
        result.update({
            'profile':'world_directional_v1',
            'stiffness_world_N_m':np.diag(self.stiffness_world).tolist(),
            'linear_damping_world_Ns_m':np.diag(self.damping_world).tolist(),
            'elastic_matrix_desired_frame':self.controller.Kp.tolist(),
            'damping_matrix_actual_body_frame':self.controller.Kd.tolist(),
            'rotation_policy':'Upstream default rotation stiffness/damping unchanged',
            'change':'Only worldY/Z translational stiffness and damping increased; worldX unchanged',
        })
        return result
