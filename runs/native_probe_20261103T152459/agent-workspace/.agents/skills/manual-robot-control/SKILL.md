---
name: manual-robot-control
description: Solve Custom Dexmate cardboard folding tasks by directly choosing robot actions from observations during exploration. Use for solving the task by hand, recovery, and physically preparing scenes for script tests.
---

# Manual robot control

First learn how to complete the task yourself through the robot tools. Read
`TASK.md` and use current observations to satisfy every condition in it. Follow the
plan and budgets in AGENTS.md; confirmed manual success leads to developing an
autonomous controller.

## First exploration

1. Read `TASK.md` and identify the required object relationships, orientation,
   placement and robot end state. Write a short completion checklist in your notes.
2. Check `status` and `MCP_SESSION.md`.
   If no episode is active and budget remains, call `start_episode`; otherwise
   inspect the active scene with `dexmate_status` and `dexmate_observe`. Check
   whether that episode has ended before attempting motion.
3. View the current camera images. Identify the relevant objects and a short
   sequence of manipulation stages.
4. Ground the next target in current observations with
   [object-localization](../object-localization/SKILL.md), choose a small, inspectable
   action chunk, and pick the motion tool for it as
   [robot-interface](../robot-interface/SKILL.md) describes.
5. Inspect feedback and new images, revise the next action, and save useful
   parameters and evidence. Work toward the completion checklist, then call
   `evaluate`.

## Observe and choose actions

Use `start_episode` to begin exploration and inspect the returned observations.

Use these skills as needed rather than loading every reference up front:

- [Robot interface](../robot-interface/SKILL.md) for this robot's motion tools,
  frames and conventions, and its related motion skills. Read it before composing
  actions.
- [Object localization](../object-localization/SKILL.md) to locate visible object
  points from the current images.
- [Wrist-camera inspection](../wrist-camera-inspection/SKILL.md) to obtain a
  clearer close-up with an empty arm when the fixed third-person view is
  insufficient; a close wrist view gives more detail than cropping `back`.
- [Free-space motion](../free-space-move/SKILL.md) for planned moves to pose
  targets; inspect goal-reaching feedback, not just planning success.
- [Experience memory](../experience-memory/SKILL.md) to record and reuse tested
  motion strategies under matching observable conditions.

Camera images are tool-result attachments, not MCP resources; you see them directly
in the tool result. Never print or stringify image bytes, call `resources/list` for
cameras, or resize inline images. View saved frames selectively with the Read tool
rather than loading many at once.
Treat previews as hypothetical robot motion, not observations of changed task objects.

## Motion and feedback

[Robot interface](../robot-interface/SKILL.md) says which motion tool fits which
objective on this robot and how its bounded steps behave. The live tool schemas
define argument formats, batch limits and step accounting. Batching reduces calls,
not the number of executed environment steps. Distinguish returned commands from
measured states: the executed prefix is useful replay data, while observations tell
you what physically happened. Read-only inspection and previews do not advance the
episode.

Observe the result of each meaningful action chunk and adapt. When a tool call
fails, read its full error and observe before the next motion. Do not bypass MCP or
blindly repeat the failed
motion; stop and report the tool failure. Do not pause an episode to ask for action
advice; decide from observations and your instructions.
If optional robot preview fails, continue without it when current evidence is
sufficient.
If localization fails or is uncertain, take a new view; do not replace missing
geometry with guessed coordinates.

## Confirm success and hand off

After strong visible evidence of completion, use `evaluate` for trusted task
confirmation. Native `task_complete: true` also confirms success. An ended episode
without confirmed success is not evidence of completion; do not infer its cause.
Partial visible progress, a successful plan or a clean tool return is insufficient.

After `evaluate` returns false, recover within the remaining episode steps when
possible. Call `evaluate` again after meaningful action and new evidence, not
repeated observation alone. When an episode ends, stop motion.
Start the next exploration episode while budget remains, keeping episodes for
rehearsals. `exploration_remaining: 0` means no new episodes or rehearsals; it does
not remove the current episode's unused steps.

Record the successful strategy and evidence in `memory/`, then read
[autonomous-control](../autonomous-control/SKILL.md) to implement it, and keep
testing in this episode's remaining steps before you close it with
`finish` (it is not a submission).
