---
name: experience-memory
description: Save successful Custom Dexmate cardboard folding action tips and retrieve them for future similar actions with matching observable conditions within this task.
---

# Custom Dexmate cardboard folding experience memory

Memory under `memory/` belongs to this task and persists across exploration
episodes. Retrieve matching tips when controlling the robot or building automation.
At task start or after compaction, read `memory/INDEX.md` if present and retrieve
only matching learned skills. Open linked evidence when applicability or outcome
is uncertain; do not load the entire history. Keep learned procedures under
`memory/skills/`, separate from the provided skills.
Rehearsals and the formal batch cannot read the editable workspace: put needed
procedures into the frozen controller before registration.

## Record successful tips immediately

- As soon as an action produces a visibly successful, useful result for reuse
  in the same task, add or update a concise row in `memory/TIPS.md`.
  A visible local result is enough to save the tip; episode-level confirmation
  can be added later.
- Record the observable preconditions, task phase, tool/action or reusable pose
  relation, visible effect, and exact manifest/frame evidence. State what made
  it work so later attempts avoid repeating exploration or a superseded failure.
- Update the matching tip when the same maneuver is refined. Apply it to future
  similar actions with matching observable conditions.
- A tip records the visible local effect. Episode-level confirmation comes from
  `evaluate` or native `task_complete: true`.

## Select

- Select successful experience with future reuse value. Motion manifests
  already preserve the complete executed actions, states, and ordered frames
  at the published paths returned by MCP under `runtime/autonomous_controller/runs/`.
- Promote a tip to a learned skill when it shows a reusable motion,
  contact strategy, recovery, or completed task procedure.
- Reference the exact manifest, step IDs, and decisive frame indices; keep PNGs,
  trajectories and complete manifests at their published paths.
- Record each example's `episode_mode` (exploration or rehearsal), as the format
  reference shows.
- Label an episode-level `task_complete: true` as task-confirmed; otherwise describe
  the visible local effect.

## Encode poses

Record executed actions and reusable pose recipes as
[this robot's pose records](../robot-interface/references/pose-records.md) define.

## Distill

Create or revise `memory/skills/<slug>/SKILL.md` and `evidence.json`.
Keep procedures short; keep episode-specific values and evidence links in
`evidence.json`.

Update `memory/INDEX.md` with the skill path, task/maneuver tags,
confirmation level, distinct evidence modes, and last episode. A learned skill
may guide current control, but current perception and the live tool schemas take
precedence. Revise the skill when later evidence contradicts it; retain
provenance in evidence.

Read [references/format.md](references/format.md) only when creating or
updating a learned skill.
