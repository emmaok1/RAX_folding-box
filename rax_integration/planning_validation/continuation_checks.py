"""Full original-scene cursor/guard tests; no cloud, contact or object actuation."""
from pathlib import Path
import copy
import json
import tempfile
import unittest
from unittest import mock

import mujoco
import numpy as np

from environment import FoldingBoxEnv
from rax_integration.motion_planning.service import MotionService
from rax_integration.motion_planning.collision import CollisionChecker
from rax_integration.motion_planning.telemetry import SubstepMotorSampler
from rax_integration.planning_validation.commission_tracking import FIRST, FAILED, frozen_path, slow_rows, admit_diagnostic


class ContinuationChecks(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.e=FoldingBoxEnv(placement='flat')
        self.s=MotionService(self.e,'cursor_check',Path(self.tmp.name),world_policy='simulator_snapshot')
        q,r=frozen_path(FIRST)
        p=admit_diagnostic(self.s,q,r,r['rows'])
        self.assertEqual(p['status'],'planned',p)
        self.assertTrue(self.execute(p['plan_id'])['goal_reached'])
        self.q,self.r=frozen_path(FAILED)

    def tearDown(self):
        self.e.close();self.tmp.cleanup()

    def execute(self,plan_id,**kwargs):
        kwargs.setdefault('expected_cursor',self.s.plans.get(plan_id,{}).get('cursor',0))
        return self.s.execute(plan_id,**kwargs)

    def path(self,factor=2):
        rows=slow_rows(self.q['snapshot']['robot']['right']['qcmd'],self.r['rows'],factor)
        p=admit_diagnostic(self.s,self.q,self.r,rows)
        self.assertEqual(p['status'],'planned',p)
        return p['plan_id']

    def paused(self):
        p=self.path();r=self.execute(p)
        self.assertEqual(r['status'],'paused',r)
        self.assertEqual(r['cursor'],50)
        return p

    def test_exact_cursor_and_full_measured_completion(self):
        p=self.paused();before=self.e.data.qpos.copy();step=self.e.env_step
        preview=self.s.preview(p)
        np.testing.assert_array_equal(before,self.e.data.qpos)
        self.assertEqual(step,self.e.env_step)
        self.assertEqual(len(preview['joint_positions_rad']),32)
        self.assertEqual(preview['cursor'],50)
        result=self.execute(p)
        self.assertEqual(result['rows_executed'],32)
        self.assertEqual(result['cursor_start'],50)
        self.assertEqual(result['cursor'],82)
        self.assertTrue(result['goal_reached'],result)
        audit=[json.loads(x) for x in (self.s.out/'audit.jsonl').read_text().splitlines()]
        indices=[x['trajectory_row'] for x in audit if x['event']=='joint_action' and x['plan_id']==p]
        self.assertEqual(indices,list(range(1,83)))
        checkpoint=json.loads((self.s.out/(p+'.cursor.json')).read_text())
        self.assertEqual(checkpoint['cursor'],82)
        self.assertEqual(checkpoint['state'],'completed')
        self.assertEqual(self.execute(p)['reason'],'PLAN_ALREADY_USED')

    def test_paused_plan_retains_both_arm_ownership(self):
        self.paused()
        result=self.s.plan({'episode_id':'cursor_check','expected_step_id':self.e.env_step})
        self.assertEqual(result['reason'],'ARM_OWNERSHIP_BUSY')

    def test_continuation_rejects_intervening_physics_step(self):
        p=self.paused();snapshot=self.s.snapshot();snapshot['step']+=1
        before=self.e.env_step
        with mock.patch.object(self.s,'snapshot',return_value=snapshot):r=self.execute(p)
        self.assertEqual(r['reason'],'CONTINUATION_STEP_CHANGED')
        self.assertEqual(self.e.env_step,before)

    def test_continuation_rejects_measured_velocity_change(self):
        p=self.paused();snapshot=copy.deepcopy(self.s.snapshot())
        snapshot['robot']['right']['v'][0]+=.001
        before=self.e.env_step
        with mock.patch.object(self.s,'snapshot',return_value=snapshot):r=self.execute(p)
        self.assertEqual(r['reason'],'CONTINUATION_STATE_CHANGED')
        self.assertEqual(self.e.env_step,before)

    def test_policy_revocation_between_chunks(self):
        p=self.paused();before=self.e.env_step
        (self.s.root/'motion_policy.json').write_text('{"world_policy":"robot_static"}')
        self.assertEqual(self.execute(p)['reason'],'POLICY_CHANGED')
        self.assertEqual(before,self.e.env_step)

    def test_source_revocation_between_chunks(self):
        p=self.paused();before=self.e.env_step
        with mock.patch('rax_integration.motion_planning.service.source_epoch',return_value=[]):
            self.assertEqual(self.execute(p)['reason'],'SOURCE_OR_WORLD_CHANGED')
        self.assertEqual(before,self.e.env_step)

    def test_world_change_between_chunks_refuses(self):
        p=self.paused();changed=copy.deepcopy(self.s.snapshot())
        next(v for v in changed['world']['joints'].values() if v['type']==0)['qpos'][0]+=.01
        before=self.e.env_step
        with mock.patch.object(self.s,'snapshot',return_value=changed):
            self.assertEqual(self.execute(p)['reason'],'WORLD_GEOMETRY_CHANGED')
        self.assertEqual(before,self.e.env_step)

    def test_stop_at_chunk_boundary_never_executes_next_row(self):
        p=self.paused();before=self.e.env_step;self.s.stop('test between chunks')
        self.assertEqual(self.execute(p)['reason'],'STOP_REQUESTED')
        self.assertEqual(before,self.e.env_step)

    def test_budget_callback_stops_at_actual_row(self):
        p=self.path();start=self.e.env_step
        self.s.can_step=lambda:self.e.env_step<start+3
        r=self.execute(p)
        self.assertEqual(r['reason'],'EPISODE_WALL_LIMIT')
        self.assertEqual(r['rows_executed'],3)
        self.assertEqual(r['cursor'],3)
        self.assertEqual(self.execute(p)['reason'],'PLAN_ALREADY_USED')

    def test_tracking_stop_is_not_resumable_or_silently_recovered(self):
        p=self.path(factor=1);r=self.execute(p)
        self.assertEqual(r['reason'],'TRACKING_ERROR')
        self.assertEqual(r['rows_executed'],19)
        refs={s:a['qcmd'].copy() for s,a in self.e.arms.items()}
        self.assertEqual(self.execute(p)['reason'],'PLAN_ALREADY_USED')
        self.assertEqual(self.s.plan({'episode_id':'cursor_check','expected_step_id':self.e.env_step})['reason'],'RECOVERY_REQUIRED')
        for side in refs:np.testing.assert_array_equal(refs[side],self.e.arms[side]['qcmd'])

    def test_invalid_chunk_size_cannot_move(self):
        p=self.path();before=self.e.env_step
        for size in (0,51,True,.5):
            self.assertEqual(self.execute(p,max_rows=size)['reason'],'INVALID_CHUNK_SIZE')
        self.assertEqual(before,self.e.env_step)

    def test_duplicate_skipped_missing_cursor_do_not_move(self):
        p=self.paused();before=self.e.env_step;cursor=self.s.plans[p]['cursor']
        for expected in (0,49,51):
            r=self.s.execute(p,expected_cursor=expected)
            self.assertEqual(r['reason'],'CURSOR_MISMATCH')
            self.assertEqual(self.e.env_step,before)
            self.assertEqual(self.s.plans[p]['cursor'],cursor)
            self.assertEqual(self.s.owner_plan,p)
        self.assertEqual(self.s.execute(p)['reason'],'EXPECTED_CURSOR_REQUIRED')
        self.assertEqual(self.e.env_step,before)
        self.assertTrue(self.s.execute(p,expected_cursor=50)['goal_reached'])

    def test_stale_other_plan_preview_cannot_release_owner(self):
        a=self.path();b=self.path()
        self.assertEqual(self.execute(a)['status'],'paused')
        self.assertEqual(self.s.owner_plan,a)
        before=self.e.env_step
        self.assertEqual(self.s.preview(b)['reason'],'STALE_STEP')
        self.assertEqual(self.s.owner_plan,a)
        self.assertEqual(self.s.plan({})['reason'],'ARM_OWNERSHIP_BUSY')
        self.assertEqual(self.e.env_step,before)
        self.assertTrue(self.execute(a)['goal_reached'])

    def test_execute_other_cached_plan_cannot_release_owner(self):
        a=self.path();b=self.path()
        self.assertEqual(self.execute(a)['status'],'paused')
        before=self.e.env_step
        self.assertEqual(self.execute(b)['reason'],'ARM_OWNERSHIP_BUSY')
        self.assertEqual(self.s.owner_plan,a)
        self.assertEqual(self.e.env_step,before)

    def test_diagnostic_sampler_restored_on_exception(self):
        original=mujoco.mj_step
        with self.assertRaisesRegex(RuntimeError,'diagnostic fault'):
            with SubstepMotorSampler(self.e):raise RuntimeError('diagnostic fault')
        self.assertIs(mujoco.mj_step,original)

    def test_batched_broadphase_matches_original_scalar_distance_checks(self):
        checker=CollisionChecker(self.s.model,'right',self.s.cfg['collision_clearance_m'])
        model=checker.robot;m,d=model.m,model.d
        def scalar(q):
            model.set_arm('right',q)
            for a,b in checker.pairs:
                if m.geom_type[a]!=mujoco.mjtGeom.mjGEOM_PLANE and m.geom_type[b]!=mujoco.mjtGeom.mjGEOM_PLANE:
                    if np.linalg.norm(d.geom_xpos[a]-d.geom_xpos[b])>m.geom_rbound[a]+m.geom_rbound[b]+checker.clearance:continue
                if mujoco.mj_geomDistance(m,d,a,b,checker.clearance,None)<checker.clearance-1e-9:return False
            return True
        rng=np.random.default_rng(42)
        home=np.asarray(self.s.snapshot()['robot']['right']['q'])
        limits=model.index['right']['limits']
        samples=[home+rng.uniform(-.04,.04,7) for _ in range(20)]
        samples += [rng.uniform(limits[:,0],limits[:,1]) for _ in range(20)]
        classifications=[]
        for q in samples:
            actual=checker.check(q);expected=scalar(q)
            self.assertEqual(actual,expected)
            classifications.append(actual)
        self.assertIn(True,classifications);self.assertIn(False,classifications)


if __name__=='__main__':unittest.main(verbosity=2)
