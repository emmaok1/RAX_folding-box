"""Cancellation behavior without native windows, model calls, or physics."""
import contextlib,itertools,json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock
import numpy as np
from scipy.spatial.transform import Rotation
from rax_integration.planning_validation import full_scene_live as demo
class DisplayStopChecks(unittest.TestCase):
    def setup_ui(self):
        env=SimpleNamespace(model=object(),data=SimpleNamespace(time=0.,qpos=np.zeros(7)),env_step=0,close=mock.Mock(),arms={'right':{'qad':np.arange(7)}},tcp_pose=lambda side:np.array([0.,0.,1.,1.,0.,0.,0.]))
        viewer=SimpleNamespace(cam=SimpleNamespace(lookat=np.zeros(3)),opt=SimpleNamespace(geomgroup=np.zeros(6)),sync=mock.Mock())
        calls=itertools.count()
        def running():
            if next(calls)>100:raise AssertionError('Display failed to leave cancellation loop')
            return True
        viewer.is_running=running
        return env,viewer
    def test_stop_before_start_does_not_create_planner(self):
        env,viewer=self.setup_ui()
        def launch(*args,**kwargs):
            kwargs['key_callback'](83)
            return contextlib.nullcontext(viewer)
        with tempfile.TemporaryDirectory() as directory, contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(demo,'FoldingBoxEnv',return_value=env))
            planner=stack.enter_context(mock.patch.object(demo,'MotionService'))
            stack.enter_context(mock.patch.object(demo.mujoco.viewer,'launch_passive',side_effect=launch))
            stack.enter_context(mock.patch.object(demo.subprocess,'check_output',return_value=''))
            stack.enter_context(mock.patch('sys.argv',['demo','--task-id','stop-test','--run-dir',directory]))
            demo.main();planner.assert_not_called();env.close.assert_called_once()
            self.assertEqual(json.loads((Path(directory)/'phase.json').read_text())['phase'],'STOPPED BEFORE PLANNING')
    def test_external_stop_after_plan_closes_final_loop(self):
        env,viewer=self.setup_ui()
        with tempfile.TemporaryDirectory() as directory, contextlib.ExitStack() as stack:
            stop_file=Path(directory)/'motion_planning/STOP';stop_file.parent.mkdir()
            model=SimpleNamespace(set_arm=mock.Mock(),pose=lambda side:(np.array([0.,0.,1.]),Rotation.identity()))
            service=SimpleNamespace(stop_file=stop_file,model=model)
            def plan(arguments):
                stop_file.write_text('external operator stop')
                return {'status':'refused','reason':'STOP_REQUESTED'}
            service.plan=mock.Mock(side_effect=plan)
            stack.enter_context(mock.patch.object(demo,'FoldingBoxEnv',return_value=env))
            stack.enter_context(mock.patch.object(demo,'MotionService',return_value=service))
            stack.enter_context(mock.patch.object(demo.mujoco.viewer,'launch_passive',return_value=contextlib.nullcontext(viewer)))
            stack.enter_context(mock.patch.object(demo.subprocess,'check_output',return_value=''))
            ticks=itertools.count();stack.enter_context(mock.patch.object(demo.time,'monotonic',side_effect=lambda:next(ticks)))
            stack.enter_context(mock.patch.object(demo.time,'sleep'))
            stack.enter_context(mock.patch('sys.argv',['demo','--task-id','external-stop-test','--run-dir',directory]))
            demo.main();service.plan.assert_called_once();env.close.assert_called_once()
if __name__=='__main__':unittest.main(verbosity=2)
