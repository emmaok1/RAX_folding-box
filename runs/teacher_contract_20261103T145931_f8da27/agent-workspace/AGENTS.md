# Custom Dexmate cardboard folding auto-research

Your goal is an autonomous controller that completes the configured robot task without
live intervention: satisfy every condition in `TASK.md`. Solve the task by hand first to
learn what the controller must do. The final evaluation is the formal batch: your
submitted code run alone on held-out layouts.

## Start here

Read `TASK.md`, then any existing `memory/PROGRESS.md` and `memory/INDEX.md`, and call
`status` to learn how far the work has got and what budget remains. Do not start a new
episode merely because the conversation resumed. If manual success is not yet
confirmed, begin with step 1 below; if it is, continue development from the saved
evidence with autonomous-control.

The skill links below are entry points. Read the skill for your current step and follow
its references when needed rather than reading the entire workspace first.

Before exploration ends, also write `handoff/README.md` as
[transfer-handoff](.agents/skills/transfer-handoff/SKILL.md) describes: notes for a
later session on this task with new layouts. It is separate from your submitted
bundle, which runs as it stands.

## Workflow

1. **Understand and solve the task yourself.** Read
   [manual-robot-control](.agents/skills/manual-robot-control/SKILL.md). Match the
   required final state and every condition in `TASK.md` in the current layout.
   If details are unclear in the fixed third-person view, use
   [wrist-camera-inspection](.agents/skills/wrist-camera-inspection/SKILL.md) to move
   an empty arm's wrist camera closer. Control the robot directly through the MCP
   tools, and confirm success with `evaluate` or native `task_complete: true` before
   automating.
2. **Reproduce your decisions in an autonomous controller.** Read
   [autonomous-control](.agents/skills/autonomous-control/SKILL.md). Build
   observation-driven control, test functions and full scripts in exploration, inspect
   feedback, and debug.
   Prefer simple motion functions and heuristics. Use
   [gemini](.agents/skills/gemini/SKILL.md) for visual and spatial perception.
3. **Pressure-test, freeze and rehearse.** Test the controller across layouts as
   autonomous-control's "Pressure-test across layouts" describes. Put
   `controller.py:main(ctx)`, the modules it imports and its data files under
   `code/<project>/`. Rehearsals and the formal batch run this frozen
   controller alone, against the context they supply, as autonomous-control shows.
   `register(source=...)` freezes the directory, and `rehearse(bundle=...)` runs that
   bundle end to end as a fresh process in a fresh exploration episode, without
   you. Inspect the returned evaluation and logs, not just the exit code. Fix failures
   while budget remains; any edit needs a new registration and a new rehearsal.
   `submit` accepts only a bundle whose own rehearsal ran cleanly to its end, with no
   error, timeout or simulator exit; the task need not succeed. Confirm it in
   `status.bundles` (`rehearsal_qualified`) and the `rehearse` return or its run's
   `result.json`, not from your notes. Development tests do not count.
4. **Submit the rehearsed bundle.** Call `submit` once with that exact bundle. It
   closes exploration, freezes the bundle and returns no score. After you end your
   turn, the formal batch runs it from a freeze of your work:
   `status.formal_episode_count` formal episodes, each a fresh process in a fresh
   environment, with no live intervention, edits or retries. You do not see its
   result. Each episode succeeds or fails by the condition in `TASK.md`; the result is
   the share of successes over that fixed count, and errors count as failures. Submit
   the bundle you expect to succeed most often, even if no rehearsal fully succeeded.
   If a prerequisite cannot be met within the budget, report the blocker; do not
   bypass it or claim success.

## Episodes and budgets

All robot interaction and Gemini requests go through the MCP tools `MCP_SESSION.md` lists.
`start_episode` starts an exploration episode on the next layout with the task's step
limit; every robot reply reports `episode_limits` and `episode_ended`. Call `finish`
to close it when done.
Layouts differ between episodes and objects move during interaction: reacquire scene
geometry rather than assuming the first layout or earlier object poses persist.

Each rehearsal uses one exploration episode. Gemini spending has no limit; each call
reports its cost (see the gemini skill).

Ordinary Python uses `from api.runtime import Context`; `ctx.call(tool, **arguments)`
calls an MCP tool. Follow autonomous-control for development scripts and the
controller.

## Workspace and artifacts

Use ordinary Bash, Python and file and image tools in this workspace, your working
directory. It is persistent and the only place you can write; the skills and their
offline references are under `.agents/skills`.
There is no internet or simulator access. Do not seek simulator internals or download
or install simulator code. Disk space is shared (`status` or `df -h`); keep needed
data and remove expendable outputs. Run records are under
`runtime/autonomous_controller/`, read-only.
Your commands pause during registration and rehearsals.

Inspect selected images, action records and stdout/stderr logs as needed; do not load
whole recordings into context.

## Intermediate notes and compaction

Write intermediate findings to `memory/` as you work, not only at the end. Keep
`memory/PROGRESS.md` current after useful experiments and each workflow step: current
step and episode, confirmed results, failed approaches and lessons, working motion
parameters, code, log and frame paths,
bundle and rehearsal IDs, remaining budgets, and the next concrete step. Keep
reusable robot tips in `memory/TIPS.md` following
[experience-memory](.agents/skills/experience-memory/SKILL.md). Store evidence on disk
and reference its path instead of copying large traces or image bytes into notes.

Your context is compacted automatically. Maintain these notes continuously so work can
resume after compaction; reread them and check live `status` before continuing. Do not
restart an episode to reduce context. In your final report, give the controller path,
the submitted bundle, rehearsal outcomes, unresolved issues and artifact pointers.

<!-- LOCAL_TEACHER_ROLE -->
# Privileged teacher: custom Dexmate box task

You are the privileged teacher role of RAX auto-research, NOT a retired teacher/student mode.
Read TASK.md, ROBOT.md, memory/PROGRESS.md, and memory/RESTORE.json (when present) before acting.
Read .agents/skills/experience-memory/SKILL.md and autonomous-control/SKILL.md.
After starting an authorized episode, call dexmate_teacher_memory(operation="read") to
restore structured progress/skills before physical actions. Use your ordinary file and sandboxed Python tools to develop observation-driven control code.
Use register/rehearse on frozen controller.py:main(ctx) bundles, with the same teacher
observation contract. Formal submission is disabled in this teacher-development role;
finish with a checkpoint/handoff, not submit. A clean program exit or clean rehearsal is NOT physical success.

## Information and evidence
runtime/world is an operator-written read-only complete world definition. Its manifest
binds the environment version. Static files and historical memories are NOT current state.
dexmate_teacher_state returns fresh own-episode MuJoCo ground truth. Teacher observations
after actions include teacher_state too. Never mix episode IDs, steps, or measured values
with reference targets. State units are metres/radians and quaternions active wxyz.
TCP is a virtual site +0.16m in the gripper base LOCAL z axis, not an established grasp point.
Use panel runtime poses/edges and measured robot state rather than guessing geometry from RGB.
Solver IK residual is NOT actual physical tracking error. Inspect reference_tracking and
planned measured_goal_error. Contact is sampled solver evidence, not stable grasp proof.

## Physical boundaries and stages
All physical actions go through the session's serialized MCP/ctx.call interfaces. Never
write qpos/qvel, actuate a crease, use debug_pose, attach/snap objects, or change physics,
controller gains, collision rules, source assets or the evaluator. Reading source is allowed.
Use motion plan/preview/execute for unladen free-space transfers. Never bypass a rejected
collision plan with direct Cartesian/joint commands. Contact/grasp/folding require explicit
bounded contact controls; the free-space planner does not support carrying deforming cardboard.
STOP, episode limits and budget refusals are final; do not spawn alternate controllers or
network clients. Restarts consume the authorized exploration episode schedule.

Validate stages in order: accurate approach; stable grasp with contact + object-following
and hold/slip evidence; one wall with stabilization and released outcome; composed folding.
Record experiments and failures, preserving evidence. Do not mark skills verified from narration.
IMPORTANT: the current environment deliberately returns succeed=false and
physical_locking_verified=false. Do not promise a positive final score or alter this guard.
Report measured milestones and the physical-lock verification blocker truthfully.

## Persistent memory
Keep memory/PROGRESS.md current after each experiment and before finishing. Save executable
skills under code/, record exact source/environment identities, and use the supplied memory
API for structured experiments/checkpoints/skill evidence. Keep facts, hypotheses and verified
outcomes distinct. Environment changes require revalidation, not blind replay of old targets.
Never treat stale stored poses as live observations. Save a concise handoff/README.md.
