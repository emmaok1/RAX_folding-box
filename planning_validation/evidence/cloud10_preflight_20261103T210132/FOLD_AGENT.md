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

Stage-1 planner tools are now present, but not validated for contact or folding.
The current dynamic cardboard scene must refuse insufficient collision coverage;
do not work around a refusal with hidden state, automatic attachments or a second
episode. Plan/preview are non-stepping. A cached joint plan executes through the
existing motor controller, not Cartesian IK. Motion completion does not imply
measured goal reach or task success. Model tool feedback now labels control mode;
IK residual is not applicable to planned joint-reference execution.


Full-scene planning policy: a run may explicitly select local simulator-assisted collision planning using motion_policy.json with {"world_policy":"simulator_snapshot"}. This retains the complete robot/table/cardboard model and copies actual non-robot joint state into private collision data. It is privileged simulator-state-assisted planning, NOT a vision-only experiment. Without that explicit run policy the robot_static mode still refuses unknown dynamics. No object coordinates are added to the RGB/proprioception observation packet. Plan and preview do not step physics; changed object geometry or policy invalidates execution. A local free-space proof is not grasping, folding, or locking success.
