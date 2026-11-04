"""Teacher input/capability checks. Local full scene; no provider or viewer."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT))
sys.path.insert(0, str(PROJECT/'rax_integration/upstream'))
import numpy as np
from environment import FoldingBoxEnv
from services.mcp_contract import Contract
from services.dexmate_box.tools import tool_definitions, validate_request
from services.dexmate_box.backend import DexmateBoxBackend
from services.controller.gateway import sanitized
from services.controller.sandbox import BundleSandbox
from services.controller.config import Limits
from rax_integration.teacher.state import teacher_state
from rax_integration.teacher.world import create_world_snapshot, verify_world_snapshot


class TeacherStateChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.env = FoldingBoxEnv(placement='flat', image_obs=False)

    @classmethod
    def tearDownClass(cls):
        cls.env.close()

    def backend(self, privileged=True):
        b = SimpleNamespace(env=self.env, episode_id='local-test', step_id=self.env.env_step,
                            finished=False, source_hashes={'test': 'source'}, teacher_world_id='test-world',
                            _last_control_mode='cartesian_ik',
                            contract=Contract(benchmark='dexmate_box', observation_profile='rgb-only',
                                              external_perception=False, teacher_privileged=privileged))
        b._control_feedback = lambda: DexmateBoxBackend._control_feedback(b)
        return b

    def test_capability_discovery_and_dispatch(self):
        normal, teacher = self.backend(False).contract, self.backend().contract
        self.assertNotIn('dexmate_teacher_state', {t['name'] for t in normal.select(tool_definitions())})
        self.assertIn('dexmate_teacher_state', {t['name'] for t in teacher.select(tool_definitions())})
        with self.assertRaises(PermissionError): normal.require_robot('dexmate_teacher_state', {})
        with self.assertRaises(PermissionError): teacher_state(self.backend(False))
        self.assertTrue(Contract.from_config(SimpleNamespace(benchmark='dexmate_box', teacher_privileged=True)).teacher_privileged)
        teacher.require_robot('dexmate_teacher_state', {})
        validate_request('dexmate_teacher_state', {})
        with self.assertRaises(Exception): validate_request('dexmate_teacher_state', {'qpos': []})

    def test_current_state_and_read_immutability(self):
        env = self.env
        before = [getattr(env.data, n).copy() for n in ('qpos', 'qvel', 'ctrl', 'qacc', 'qacc_warmstart')]
        tick, step, release = env.data.time, env.env_step, env._release_since
        packet = teacher_state(self.backend())
        self.assertEqual(packet['observation_kind'], 'live_owned_episode')
        self.assertEqual(len(packet['panels']), 14)
        self.assertEqual(sum('hinge' in p for p in packet['panels']), 13)
        np.testing.assert_array_equal(packet['box_free_pose_world_xyz_wxyz'], env.data.joint('box_free').qpos)
        for panel in packet['panels']:
            np.testing.assert_array_equal(panel['pose_world']['position_m'], env.data.body(panel['body_name']).xpos)
        self.assertEqual(packet['robot']['right']['references']['jaw_position_rad'], env.grippers['right']['qcmd'])
        self.assertEqual(packet['step_id'], step)
        self.assertEqual(packet['sim_time_s'], tick)
        for name, prior in zip(('qpos', 'qvel', 'ctrl', 'qacc', 'qacc_warmstart'), before):
            np.testing.assert_array_equal(getattr(env.data, name), prior)
        self.assertEqual(env._release_since, release)
        self.assertEqual(env.env_step, step)
        json.dumps(packet, allow_nan=False)

    def test_ground_truth_survives_existing_reply_sanitization(self):
        packet = teacher_state(self.backend())
        result = self.backend().contract.observation.clean(sanitized(packet))
        self.assertEqual(result['panels'], packet['panels'])
        self.assertEqual(result['contacts'], packet['contacts'])
        self.assertEqual(result['cameras'], packet['cameras'])
        self.assertEqual(result['environment_id'], 'test-world')

    def test_world_snapshot_complete_stable_and_tamper_detection(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)/'world'
            manifest = create_world_snapshot(PROJECT, root)
            self.assertGreater(len(manifest['files']), 200)
            self.assertIn('models/scene.xml', manifest['files'])
            self.assertEqual(manifest, verify_world_snapshot(root))
            target = root/'physics.json'
            target.chmod(0o600)
            target.write_text('{}')
            with self.assertRaises(ValueError): verify_world_snapshot(root)

    def test_rehearsal_world_mount_is_read_only(self):
        limits = Limits(10, 2048, 1., 128, 128, 1024*1024)
        command = BundleSandbox(limits).command('/bundle', 'rehearsal', 'controller.py',
            workspace='/workspace', layout='/layout', writable='/writable', sdk='/sdk',
            references=[('/teacher_world', 'runtime/world')])
        index = command.index('/teacher_world')
        self.assertEqual(command[index-1], '--ro-bind')
        self.assertEqual(command[index+1], '/workspace/runtime/world')

    def test_protected_sources_unchanged(self):
        baseline = Path(__file__).parent/'baseline/protected_hashes.json'
        expected = json.loads(baseline.read_text())
        actual = {name: hashlib.sha256((PROJECT/name).read_bytes()).hexdigest() for name in expected}
        self.assertEqual(expected, actual)


if __name__ == '__main__':
    unittest.main()
