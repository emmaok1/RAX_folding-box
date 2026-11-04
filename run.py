"""Viewer and deterministic reset check for the baseline simulation."""
import argparse
import json
import time

import mujoco
import numpy as np
from environment import FoldingBoxEnv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['viewer', 'reset'])
    parser.add_argument('--placement', choices=['centered', 'flat'], default='flat')
    parser.add_argument('--paused', action='store_true')
    args = parser.parse_args()
    env = FoldingBoxEnv(seed=11, placement=args.placement)
    try:
        if args.mode == 'reset':
            before = {n: getattr(env.data, n).copy() for n in ['qpos', 'qvel', 'ctrl']}
            env.step()
            env.reset(seed=11)
            result = {n: bool(np.array_equal(v, getattr(env.data, n))) for n, v in before.items()}
            print(json.dumps(result, indent=2))
            assert all(result.values())
            return
        import mujoco.viewer
        keys = []
        paused = args.paused
        print('Box simulation. R reset | Space pause. Use MuJoCo mouse interaction to apply forces.', flush=True)
        with mujoco.viewer.launch_passive(env.model, env.data, key_callback=keys.append,
                                         show_left_ui=False, show_right_ui=False) as viewer:
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_ISLAND] = False
            viewer.opt.geomgroup[:] = [1, 1, 1, 0, 0, 0]
            viewer.cam.lookat[:] = [-.4, 0, 1.05]
            viewer.cam.distance = 2.65
            viewer.cam.azimuth = 145
            viewer.cam.elevation = -24
            while viewer.is_running():
                tick = time.monotonic()
                while keys:
                    key = keys.pop(0)
                    if key == 82:
                        env.reset()
                    elif key == 32:
                        paused = not paused
                if not paused:
                    env.step()
                viewer.sync()
                time.sleep(max(0, env.control_dt - (time.monotonic() - tick)))
    finally:
        env.close()


if __name__ == '__main__':
    main()
