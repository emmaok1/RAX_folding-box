Motion commissioning (MP-TRACK-CONTINUE-20261104)
==============================================

Run from the box project root with its existing `.venv/bin/python`:

```
.venv/bin/python -m rax_integration.planning_validation.commission_tracking
.venv/bin/python -m unittest rax_integration.planning_validation.continuation_checks rax_integration.planning_validation.adapter_checks -v
.venv/bin/python -m rax_integration.planning_validation.commission_free_space --repeats 3
.venv/bin/python -m rax_integration.planning_validation.continuation_mcp_check
.venv/bin/python -m rax_integration.planning_validation.plot_tracking PATH_TO_TRACKING_EVIDENCE
```

These are local commissioning commands, not cloud-model folding attempts.
They use the full current robot/table/cardboard scene and the original motor
controller. The tracking experiment revalidates frozen accepted reference paths
against the current collision world; it does not teleport robot or object state.
Each independent diagnostic starts using the existing flat reset. Raw substep
motor samples are captured with a read-only wrapper around the same `mj_step`,
restored in a context manager even on failure. Wall runtime includes diagnostic
logging and collision checks; simulated control time remains 20 ms per row.

Viewer command for the monitor to coordinate (do not auto-launch or open a
duplicate window):

```
DISPLAY=:1 MUJOCO_GL=glfw .venv/bin/python -m rax_integration.planning_validation.full_scene_live --task-id MP-TRACK-CONTINUE-20261104 --execute --preview-seconds 5 --display-delay 0.03
```

This shows the full original robot/table/cardboard scene, a test-selected robot
FK goal and preview, then measured joint execution through the SAME model/data.
Display continuation reuses the same plan ID and does not call a cloud model.
It holds the final state visible; S, window close, signals or its out-of-band
STOP file end the display. The native window has not been launched as part of
this task. Six mocked display-loop checks cover continuation/cancellation and
the finite call bound; they are not evidence of a new native display run.
The actual local MCP execution video is independently saved with its run.

Execution contract
------------------

`max_trajectory_rows=500` is the total cached reference limit (10 simulated
seconds). `max_rows=50` remains the execution-call ceiling. The adapter can lower
that per-call ceiling using its action policy. Speed/acceleration limits are now
0.2 rad/s and 0.25 rad/s² globally, with tighter per-joint speed caps of
0.095–0.120 rad/s. Those caps are rounded down from
`0.75 * tracking_guard * kp / (kd + joint_damping)`: the unchanged controller
has zero reference-velocity feedforward, so steady lag is approximately
`(kd + joint_damping)/kp * reference_velocity`. This estimate adds margin;
measured tracking checks remain mandatory. These are commissioning planner
limits, not new manufacturer/controller/physics settings. The
tracking guard remains 0.02 rad and final tolerances remain 5 mm / 0.05 rad.

`dexmate_motion_execute({"plan_id":"...","expected_cursor":0,"max_rows":50})` returns the actual
chunk row count, total cursor and remaining rows. `status=paused` means continue
the SAME plan ID with `expected_cursor` equal to the returned cursor;
duplicates and skipped cursors refuse without motion. `command_completed=false`
and `goal_reached=false` until the
entire trajectory has finished. A valid chunk boundary does not require zero
velocity. It does require the exact saved measured state, reference state,
episode step/time, jaw command, unchanged source/policy and quasi-static world.
No physical step or other action may intervene. Both arms remain owned by the
paused path. Each resumed row still gets collision and tracking validation.
The per-row audit, video callback, action-call/row limits, wall and episode
limits, and out-of-band STOP remain active across calls.

IK/search and final resampled-edge validation have separate 4 s deadlines,
inside the existing 12 s worker cap. Final checks cannot inherit an already
exhausted search deadline. Refusal distinguishes validation timeout, sample
limit and collision. The broad phase batches the same bounding-sphere tests;
near pairs still use the same MuJoCo distance call and 5 mm threshold.
The focused regression compares classification against the prior scalar
implementation on local and colliding scratch-model poses.

The cursor is atomically saved after each actual control row. Its file is audit
evidence, not authority to resume after a process restart. A new service does not
deserialize it into an executable plan. Restart recovery requires an explicit
operator decision and fresh measured-state planning.

Guard stop and recovery
----------------------

A tracking/collision/STOP/budget/callback failure makes the path permanently
non-resumable. There is no automatic reset of joint/TCP references, no Cartesian
`hold` workaround, and no gain/tolerance change. Status describes the failure
and requires an explicitly operator-authorized episode reset followed by fresh
settled-start commissioning. This iteration provides no automatic in-place
braking/recovery controller. Reset is an initialization operation, never evidence
of successful manipulation. STOP must not be cleared to continue an old path.

Interpretation
--------------

Original-speed replay reproduces the historical tracking failure; a guard stop
is a failed motion. Slowed complete trajectories are evaluated by measured final
error, not successful command submission. Free-space checks reject any measured
robot/cardboard contact. Collision distances reported by the checker are
threshold tests/lower bounds, not exact global minimum distances or continuous
collision certificates. No result here establishes grasping, folding or locking;
the original task evaluator remains unchanged.
