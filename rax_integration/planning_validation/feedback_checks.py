"""Replay actual failed observations; no cloud, simulation or viewer."""
from pathlib import Path
import json,sys,tempfile,unittest
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'rax_integration/upstream'))
from services.dexmate_box.backend import DexmateBoxBackend
RUN=ROOT/'rax_integration/runs/fold_20261103T130237_be872c'
def backend(actual,references,solver,mode='cartesian_ik'):
    b=object.__new__(DexmateBoxBackend);b._last_control_mode=mode
    b.env=SimpleNamespace(tcp_pose=lambda side:np.array(actual[side]),targets=references,ik_errors=solver)
    return b
class FeedbackChecks(unittest.TestCase):
    def replay(self,step):
        files=list(RUN.glob(f'native/episodes/*/observations/{step:06d}_*/observation.json'))
        self.assertTrue(files,f'Missing original observation step{step}')
        o=json.loads(files[0].read_text());poses=o['state']['tcp_pose'];f=o['control_feedback']
        b=backend({'right':poses[:7],'left':poses[7:14]},f['tcp_references'],f['ik_residual'])
        return b,o
    def test_real_step187_small_solver_error_is_not_arrival(self):
        b,o=self.replay(187);f=b._control_feedback()
        self.assertLess(f['ik_residual']['right']['position_m'],.0001)
        self.assertGreater(f['reference_tracking']['right']['position_m'],.14)
        self.assertFalse(f['reference_tracking']['right']['within_reference_tolerance'])
    def test_real_step227_large_physical_error_reaches_tool_reply(self):
        b,o=self.replay(227);b.step_id=227;b.episode_id='recorded_replay';b.finished=False;b.config=SimpleNamespace(task='cardboard_box_folding')
        b.env.data=SimpleNamespace(time=o['sim_time_s']);b.env.grippers={side:{'command':1.} for side in ('right','left')}
        b.env._compute_observation=lambda:{'state':o['state']};b.env.progress=lambda:{}
        b.action_layout=lambda:{};b.log=Mock();b._publish_live=Mock()
        with tempfile.TemporaryDirectory() as directory:
            b.output=Path(directory);packet,images=b._observe(cameras=[])
            f=packet['control_feedback'];self.assertGreater(f['reference_tracking']['right']['position_m'],.21)
            self.assertFalse(f['reference_tracking']['right']['within_reference_tolerance']);self.assertEqual(images,[])
            self.assertLess(f['ik_residual']['right']['position_m'],.02)
    def test_quaternion_sign_equivalence_and_orientation_error(self):
        ref={side:[0,0,1,1,0,0,0] for side in ('right','left')};actual={'right':[0,0,1,-1,0,0,0],'left':[0,0,1,0,1,0,0]}
        f=backend(actual,ref,{})._control_feedback();self.assertAlmostEqual(f['reference_tracking']['right']['orientation_rad'],0)
        self.assertTrue(f['reference_tracking']['right']['within_reference_tolerance']);self.assertAlmostEqual(f['reference_tracking']['left']['orientation_rad'],np.pi)
        self.assertFalse(f['reference_tracking']['left']['within_reference_tolerance'])
    def test_planned_reference_feedback_does_not_claim_final_goal(self):
        poses={side:[0,0,1,1,0,0,0] for side in ('right','left')}
        f=backend(poses,poses,{'right':{'position_m':0}},'planned_joint_references')._control_feedback()
        self.assertIsNone(f['ik_residual']);self.assertNotIn('goal_reached',f);self.assertIn('not the final trajectory goal',f['reference_tracking_semantics'])
    def test_invalid_pose_is_not_arrival_or_nan_json(self):
        refs={side:[0,0,1,1,0,0,0] for side in ('right','left')};poses={**refs,'right':[float('nan'),0,1,1,0,0,0]}
        f=backend(poses,refs,{})._control_feedback();self.assertFalse(f['reference_tracking']['right']['data_valid'])
        self.assertFalse(f['reference_tracking']['right']['within_reference_tolerance']);json.dumps(f,allow_nan=False)
if __name__=='__main__':unittest.main(verbosity=2)
