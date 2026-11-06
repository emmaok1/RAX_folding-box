---
name: autonomous-control
description: Run and debug Python robot control in your workspace, then register, rehearse and submit a frozen controller. Use for closed-loop scripts, switching between your script and direct control, and run artifacts.
---

# Autonomous control

Follow the workflow in AGENTS.md. The environment and API are external services,
reachable only through MCP.

## Start from your successful exploration

Read the saved successful strategy and decisive frames in `memory/`. Create a
project under `code/<project>/` and turn one useful maneuver into a
Python function that receives `ctx` and explicit inputs. Test it in the current
exploration scene using the ordinary Python example below. Inspect its final
images and logs, correct it, and repeat before composing the full `main(ctx)`.

Separate reusable motion logic from scene-specific inputs: reacquire object
locations from current observations instead of embedding one layout's positions.

## Turn successful control into automated scripts

After succeeding through direct robot control, reproduce the decisions that made
it work in an automated observation–decision–action script that runs without you.
Use current observations to adapt to new layouts, rather than blindly replaying
one episode's actions.

- Hard-code reusable motion functions and decision rules using MCP calls when suitable.
- Use Gemini when simple perception rules are insufficient. Write your own
  prompts and response-processing code; see the [gemini skill](../gemini/SKILL.md).
- Practise and debug functions within the same exploration episode, using direct
  robot calls to rearrange objects into the test conditions you need rather than
  starting a new episode. Exploration outcomes are not scored, so use the remaining
  steps.

Combine these approaches as needed, and test the resulting closed-loop script in
exploration before rehearsing it.

## Pressure-test across layouts

Use all exploration episodes for development and robustness tests, not just enough
to obtain one success. Layouts vary between episodes; reacquire objects and adapt
approach poses instead of reusing layout-specific coordinates. Within an episode,
refresh observations after moving objects. Use new episodes to test different
layouts and the full controller, and keep episodes to rehearse the final version
(AGENTS.md step 3). Record each tested layout, outcome, failure and fix in memory;
agent-assisted recovery is debugging, not an autonomous success. Add
observation-driven recovery for expected perception or planning failures. Do not claim untested
generalization.

## Exploration: ordinary Python

Python runs in your workspace as your other commands do, with normal files, stdout and
stderr.

### Write the controller against the supplied context

Write reusable functions against a supplied context `ctx`. The entrypoint below is
the same in exploration, rehearsals and the formal batch:

```python
# code/reach/controller.py — registered controller code
def main(ctx):
    reply = ctx.call("dexmate_observe")
    # Your observation–decision–action loop:
    # ctx.call("dexjoco_step", actions=action_rows)  # official rows: flange xyz, rotvec, hand joints
    # ctx.call("dexjoco_hold", steps=10)
```

Only the development wrapper opens the context:

```python
# Development wrapper, outside the registered code/reach directory
import sys
from api.runtime import Context

sys.path.insert(0, "code/reach")
from controller import main

with Context() as session:
    main(session)
```

Keep this wrapper under `experiments/` and run it from the workspace.
Submitted `main(ctx)` and its helpers use the supplied context unchanged, passing
`ctx` down: no new `Context()`.
`Context()` connects to the same MCP endpoint as you do. Your function can send
chosen official action rows or holds, or repeatedly observe, choose rows and inspect
feedback using heuristics or Gemini.
Return whenever you want to inspect images and choose new actions yourself. The
scene retains the script's actions; returning or closing the connection does not
reset or finish the episode.

Robot calls run one at a time, but a script does not hold the robot: other calls can
come between its calls.
Run one controller at a time, and stop it before acting directly. Interrupting
Python does not cancel an already-dispatched action: after an interruption or an
ambiguous motion error, check `dexmate_status` and a fresh observation before
continuing. Stop when the episode ends.

You may test the entire controller in the current episode, including stepping in
yourself.

## MCP and files

`ctx.call(tool_name, **arguments)` uses the live tool schemas and returns MCP
content blocks. JSON metadata is in the first text block; images remain image
blocks. Inspect current tool schemas instead of inventing arguments.
Calls reporting an MCP error raise `RuntimeError`; inspect the call's record in the
run's `mcp.jsonl` (`artifacts.mcp_trace`) rather than blindly retrying motion.

Use normal Python for files, images and recording. Returned artifact paths are
relative to the workspace; save script replies and prints to local files for
debugging. Robot replies, per-call images and saved frames are published
automatically; ordinary shell stdout/stderr needs your own redirection if you want
a saved log.

## Rehearsal and submission

Put `main(ctx)` in `controller.py`, with all required modules and data files in its
source directory. `register(source="code/reach")` freezes that directory.
`rehearse(bundle=...)` closes the live exploration episode and runs the frozen
controller alone, as a fresh process, in a fresh episode. Wait for outstanding robot calls
before invoking it.
A rehearsal and each formal episode have the same wall limit
(`status.episode_timeout_seconds`, startup included). Leave timing margin; formal
episodes may run in parallel and slower. A timeout stops the controller, closes the
environment and returns an error with diagnostic artifact paths; it does not count as
a pass. `status.bundles` lists registered bundles and whether each has passed a
rehearsal.

Rehearsals and the formal batch pass `main` its context as the sole argument. This
channel exposes robot and Gemini calls, not `start_episode`, `finish`, `register`, `rehearse` or
`submit`. Rehearsals and the formal batch run with this workspace's path as the working
directory and the project at its own path, e.g. `code/reach`, read-only.
Use paths relative to `__file__` for bundled inputs. They receive only their own frames,
under `runtime/autonomous_controller/runs/<run-id>/frames/`, and the submitted
project, not the live workspace, previous runs or the exploration episode.

Write generated files under `output/` in both settings; it is fresh and
byte-capped in rehearsals and the formal batch. After the controller exits, safe
regular files are copied to
`runtime/autonomous_controller/runs/<run-id>/exports/`. Storage quotas
apply; links and special files are rejected. After the episode ends, stop calling
tools and finish file writes promptly (up to ten seconds within the original
deadline).

Results point to frozen code, stdout/stderr logs, `mcp.jsonl`, per-call responses,
images and full frame sequences. Read only what you need, directly with ordinary
file tools.
