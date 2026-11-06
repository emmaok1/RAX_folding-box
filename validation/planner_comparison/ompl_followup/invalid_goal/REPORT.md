# Invalid Cartesian target rejection: PASS

Real OMPL RRTConnect2.0.1 rejected a reachable target inside the unchanged table with status `Invalid goal`. No robot command or physical step was executed after the safe baseline snapshot.

Target TCP world pose was[-0.32,-0.065,0.88]m, quaternion wxyz[0,1,0,0]. Scratch-only IK reached it with1.959µm position residual and1.929e-5rad orientation residual, so this was collision rejection rather than IK failure. Original MuJoCo collision checking found both custom right jaw pads penetrating `table_collision` by43.697mm and43.697mm, and both right gripper bodies by60.889mm and60.868mm. Additional gripper/base-panel intersections were present. Full pair names, penetration depths, and world contact positions are in result.json; these are hypothetical scratch-state contacts, never physical execution.

The existing real OMPLPlanner was called directly with its full MuJoCo validity checker. It checked one valid start and one invalid goal, reported `Invalid goal` after2.0148s with a2s solve budget, returned no path, and `haveSolutionPath()` was false. Thus neither an exact nor approximate path was accepted. This wrapper lacks a direct `haveApproximateSolutionPath()` method (recorded null), but no solution of any type exists.

After planning, executed qpos, qvel, ctrl, simulation time, arm qcmd references, Cartesian targets, gripper references, and env_step were exactly equal to their preplanning values. Calls to both motor interfaces were zero. Simulation time remained0.5s after initial settling, and all MuJoCo warnings remained zero. before.png and rejected.png show the same safe executed baseline; they do not visualize the rejected scratch configuration.

Reproduce from project root:

```bash
.venv/bin/python3.11 validation/planner_comparison/ompl_followup/invalid_goal/test.py
```

Artifacts: test.py; run.log; result.json; inside_table_planner.json; before_state.json; after_state.json; before.png; rejected.png. Source hashes in result.json cover core environment/model/physics, shared checker, actual OMPL adapter, and test source. No installations, Tesseract use, shared edits, or physics changes were made.
