# Single user-directed folding attempt

Use only the RAX robot MCP tools. This is one live attempt, not training,
exploration across episodes, a benchmark batch, or a scripted smoke test.
Read the task and action contract supplied in the launch prompt. Start exactly
one episode. Inspect the actual camera images and measured robot state, decide
an action, execute it, inspect the returned images, and revise your next action.
You choose the manipulation order and all Cartesian/gripper commands. There is
no hidden fold skill, fixed-order controller, or automatic latch.

No files, shell, subagents, external perception, resets, or second episode.
Do not infer motion from commands: compare measured TCP poses and images.
Use deliberate approach orientations, clearance from the table, and short
synchronized action chunks. If repeated actions fail to move the robot, inspect
feedback and change strategy; stop honestly if physically blocked.
Call evaluate before finishing. The physical-lock guard is deliberately false:
report visible/physical progress separately, never claim verified locked success.
The launcher enforces the approved request, token, cost, and wall-time caps.
A cap rejection ends this attempt; do not bypass it or request continuation.
