# Learned memory format

Store learned skills only below `memory/skills/`, each under a lowercase
hyphenated slug.

## `TIPS.md`

One row per reusable local result; update a matching row instead of adding variants
of the same lesson:

```markdown
| Task phase | Preconditions | What worked | Visible effect | Evidence |
|---|---|---|---|---|
| <phase> | <observable conditions> | <tool and action> | <visible change> | <returned frame manifest path> frame <n> |
```

## `INDEX.md`

```markdown
| Skill | Tags | Confirmation | Modes | Last episode |
|---|---|---|---|---|
| `skills/<slug>/SKILL.md` | <tags> | observed | exploration | <episode_id> |
```

`Confirmation` is `observed` for a clear local visual effect, or
`task-confirmed` when the procedure contributed to an episode accepted with
`task_complete: true`. `Modes` lists the evidence modes: `exploration`, `rehearsal`
or both.

## Generated `SKILL.md`

```markdown
---
name: learned-<slug>
description: <maneuver and when this experience applies>
---

# <name>

- Applies when: <observable conditions>
- Avoid when: <observable mismatch>
- Executed action: <tool and exact action representation>
- Pose recipe: <anchor, recipe representation, frame, delta>
- Actions: <compact ordered procedure>
- Expected effect: <visible result>
- Recovery: <evidence-backed alternative>
```

Do not copy tool documentation or global limits into a learned skill, and do not
encode hidden state, inferred task success or an unobserved object pose.

## `evidence.json`

```json
{
  "schema_version": 4,
  "skill": "learned-<slug>",
  "examples": [
    {
      "task": "<task>",
      "episode_id": "<episode_id>",
      "episode_mode": "exploration | rehearsal",
      "confirmation": "observed | task-confirmed",
      "manifest_path": "runtime/autonomous_controller/runs/<run>/frames/<sequence>/manifest.json",
      "step_ids": [1, 2],
      "frame_indices": [0, 1],
      "action_kind": "<see pose records>",
      "executed_action": "<see pose records>",
      "pose_recipe": "<see pose records>",
      "point_reacquisition": "<see pose records>",
      "observed_effect": "<visible change>"
    }
  ]
}
```

Every example has an `episode_mode`. The robot-specific fields are defined in
[pose records](../../robot-interface/references/pose-records.md). For a
point-relative recipe, `point_reacquisition` records the visible feature, the
offset, the RGB alignment procedure and the preferred views.
