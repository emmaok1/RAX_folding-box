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

For this new whole-task attempt, use motion planning for free-space transfers.
The selected run policy explicitly enables simulator-state-assisted collision
snapshots. That is not a vision-only planning experiment. Read the returned
plan/refusal and actual measured errors. Planning/preview do not step physics;
execute only valid cached plans. The planner keeps jaws fixed and does not
support held-object or intentional-contact folding paths. Grasping, closing jaws,
and contact folding are authorized here through existing Cartesian/jaw controls;
do not mislabel those controls as collision-checked planner execution.
If an arm has a failed old reference, all-zero pose retains that failed target.
Use its measured pose as an explicit hold command and inspect settling feedback.
Stop honestly if repeated attempts produce no actual contact/grasp/fold progress.
Keep narration to at most two short sentences per tool call; prioritize actions
and measured feedback over optimistic descriptions.

Full-scene planning policy: a run may explicitly select local simulator-assisted collision planning using motion_policy.json with {"world_policy":"simulator_snapshot"}. This retains the complete robot/table/cardboard model and copies actual non-robot joint state into private collision data. It is privileged simulator-state-assisted planning, NOT a vision-only experiment. Without that explicit run policy the robot_static mode still refuses unknown dynamics. No object coordinates are added to the RGB/proprioception observation packet. Plan and preview do not step physics; changed object geometry or policy invalidates execution. A local free-space proof is not grasping, folding, or locking success.

Measured tracking feedback: control_feedback.reference_tracking compares the actual TCP with the current commanded reference. Use its position/orientation errors to judge arrival. ik_residual describes only the IK solver scratch solution and can be tiny while the physical TCP is far away. In planned-joint mode, reference_tracking concerns the current joint-reference FK; final goal error is transition.motion.measured_goal_error. Neither field proves grasp, contact or folding.
