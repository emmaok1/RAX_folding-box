"""Exact MuJoCo distance probes on a private 1-DoF geometry fixture; zero mj_step.
This isolates edge/clearance behavior. It is not a Dexmate execution or continuous-collision certificate.
"""
import unittest
from types import SimpleNamespace
import mujoco
import numpy as np
from rax_integration.motion_planning.collision import CollisionChecker

class ThinEdgeChecks(unittest.TestCase):
    def checker(self):
        model=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <body name="thin_panel"><geom name="probe_thin_panel" type="box" pos="0.0013 0 0" size="0.0005 .02 .02"/></body>
          <body name="dexmate_root"><body name="dexmate_R_probe">
            <joint type="slide" axis="1 0 0" range="-.1 .1"/><geom name="probe_tip" type="sphere" size=".002" mass=".01"/>
          </body></body></worldbody></mujoco>''')
        data=mujoco.MjData(model)
        def set_arm(side,q):data.qpos[0]=q[0];mujoco.mj_forward(model,data)
        return CollisionChecker(SimpleNamespace(m=model,d=data,exclusions=set(),set_arm=set_arm),'right',.005)
    def test_thin_crossing_rejected_despite_clear_endpoints(self):
        checker=self.checker()
        self.assertTrue(checker.check(np.array([-.02])))
        self.assertTrue(checker.check(np.array([.02])))
        cfg={'edge_joint_resolution_rad':.004,'max_validation_samples':1000}
        self.assertFalse(checker.edge(np.array([-.02]),np.array([.02]),cfg,lambda:False))
        self.assertIn('probe_thin_panel',checker.last_conflict['geoms'])
        self.assertEqual(checker.robot.d.time,0)
    def test_clearance_threshold_is_preserved(self):
        checker=self.checker();boundary=.0013+.0005+.002+.005
        self.assertFalse(checker.check(np.array([boundary-1e-6])))
        self.assertTrue(checker.check(np.array([boundary+1e-6])))
        self.assertEqual(checker.robot.d.time,0)

if __name__=='__main__':unittest.main(verbosity=2)
