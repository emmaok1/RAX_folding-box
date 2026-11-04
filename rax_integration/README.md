# Custom Dexmate cardboard folding with RAX and Claude Code

Latest handoff received: fresh video design, 14 panels / 13 passive hinges; no SM1 geometry, double walls or base handle. Robot actions remain 16-D; hinge observations are now 13-D. See validation/video_rebuild_handoff.json for peer-reported facts. Cloud API has not independently validated this revision. All execution remains blocked pending a new user instruction; prior results below remain historical.

**Latest scope: previous box-design basis cancelled.** A fresh rebuild from the local reference video is underway. The current scene is provisional. All earlier design and smoke results below are historical; integration remains offline and validation is on hold until the fresh handoff.

**Redesign in progress: awaiting a new-version handoff.** B273_final and all
previous hashes/checks below are historical. Do not use them as the current
baseline. Integration checks, fixture runs and preparation are on hold while
box-builder changes geometry/physics. Cloud access, training and exploration
remain prohibited. No core files or success criteria are changed by this adapter.

Historical scope correction: the eight-request aggregate exceeded the earlier
four-request scope Cloud API had stated. No separate explicit user authorization
is recorded for the four regression/continuation requests. The dollar cap was
met, but that does not establish request-cap compliance. See
[authorization review](validation/AUTHORIZATION_REVIEW.md). Current cloud access
remains blocked.

**Current scope: offline integration only.** Cloud connections, model inference,
training and exploration are prohibited in this phase. Historical smoke records
are not renewed authorization. `CURRENT_SCOPE.json` blocks both live smoke and
read-only model continuation before credential access or process launch. Historical
API commands below are inactive until the user explicitly changes scope and
authorizes a new budget.

Latest handoff: **B273_final is frozen**. All eight hashes in
`diagnostics/configs/frozen_parameters.json` match. A read-only metadata check
reports 0.25 ms physics, 80 substeps per 20 ms control row. No scene rebuild,
simulation, or paid inference was run for this handoff. The 46-check suite and
Claude smoke results below belong to their recorded earlier 0.5 ms revision;
they have not been relabeled as B273 validation. Box-builder's pickup report
records two deterministic whole-blank lift/10-second-hold/release successes;
full folding and locking remain unverified. See
[B273 handoff evidence](validation/B273_handoff.json) and
[pickup report](../pickup_iterations/REPORT.md). Core task files and the open
viewer are protected; integration work stays inside this directory.

This integrates the existing `FoldingBoxEnv` in place. It is a **custom task
integrated with RAX**, not an official DexJoCo benchmark. The existing
`./run.sh viewer`, `reset`, `validate`, and `contact-demo` entries remain usable.
The adapter changes no scene, cardboard, physics, robot-controller, or physical success code.
Separate box/material work updated the task during integration. The adapter now
derives timing and gripper mode from the loaded environment; each new episode
preserves source/configuration snapshots and a compiled MuJoCo model. Current
settings are 0.25 ms physics / 80 substeps per 20 ms action and a zero-opening
closing-torque mode. Historical and current evidence are identified separately.

## Setup and launch

Run from the current `折纸盒` directory:

```bash
bash rax_integration/setup.sh
MUJOCO_GL=egl ./run.sh rax-check
MUJOCO_GL=egl ./run.sh rax-fixture
MUJOCO_GL=egl ./run.sh rax-prepare
DISPLAY=:1 XDG_RUNTIME_DIR=/run/user/1000 MUJOCO_GL=glfw ./run.sh viewer
```

`rax-check` exercises the actual robot and official RAX MCP socket/stdio bridge
without any model. `rax-fixture` runs the installed Claude Code against a scripted
local Anthropic wire fixture; it sends **zero real inference requests** and spends
zero API dollars. The fixture proves image/tool/action transport, not learned
visual reasoning. `rax-prepare` creates a reviewable workspace and budget file
without inference. Each command prints its result path under `rax_integration/runs/`.

The existing `.venv` is reused. Its base packages remain pinned by
`requirements.txt`; `requirements-rax.txt` adds Jinja2 3.1.6 for RAX workspace
generation. System prerequisites are Claude Code, bubblewrap, socat, ffmpeg and
ffprobe. This machine has Claude Code 2.1.316 and bubblewrap; missing socat was
extracted task-locally from Ubuntu's `socat_1.7.4.1-3ubuntu4_amd64.deb`. The setup
script uses the system socat when available and otherwise supports apt-based
local extraction. It does not install a simulator, train, or start inference.

For an independent Python 3.11 environment, install both requirements files into
the environment used by `run.sh`; the source assets are already in this task.

## Real Claude smoke completed; bounded rerun command

The user subsequently supplied a direct Anthropic credential and explicitly
authorized this session to use it for the original work. The live smoke completed
successfully under the existing $0.50 / 120 s limits. Its credential is stored in
`~/.config/rax/dexmate_box_anthropic_api_key` (mode 600), **outside this project**.
No credential value is copied into code, commands, peer messages or run logs.
Inherited third-party provider variables were not used.

The initial smoke plus the current-metadata regression used **8 actual inference
requests**, estimated **$0.1920435 total**, with **$0.433074 reserved** under the
combined $0.50 cap. The regression's tighter three-request budget stopped after
the action; one separately bounded read-only continuation sent the updated images
to Claude and completed its report without another episode or robot movement.
The original cap stop is retained in its own result and audit ledger.

Simulation-engineer subsequently confirmed that the historical DeepSeek/RoboCasa
attempt caps do not authorize Claude API or new box-folding runs; see
[coordination notes](COORDINATION.md).

The exact bounded launch command is:

```bash
MUJOCO_GL=egl ./run.sh rax-smoke \
  --key-file "$HOME/.config/rax/dexmate_box_anthropic_api_key" \
  --approve-budget-usd 0.50
```

The explicit budget flag is required only for real inference. The reviewable
configuration is [smoke_config.json](smoke_config.json): direct
`https://api.anthropic.com`, `claude-sonnet-4-6`, at most 4 inference requests and
4 token-count requests; 24,000 input tokens per request and 60,000 total; 512 output
tokens per request and 2,048 total; $0.50 reserved cost; 120 seconds of CLI wall
time; one episode and one action call, at most 25 rows / 0.5 simulated seconds.
The action must preserve measured orientations, remain within 5 mm of each TCP,
and change gripper opening by at most 0.05. The smoke asks for +2 mm world Z on
both arms and returns the updated camera observations.

The bounded local relay uses the real key only in the operator process and sends
it only to Anthropic. Claude Code receives an ephemeral loopback capability.
Headers and credential values are excluded/redacted from records. The official
token-count endpoint admits each inference with a 256-token/1% margin. Cost
admission reserves $3/M input times 2 (covers a one-hour cache write) and $15/M
output, without refunding earlier reservations. The configured total token bounds
imply at most $0.39072 of this conservative reservation. Native Claude Code
`--max-turns` and `--max-budget-usd` provide additional limits. Request retries
also consume admission limits. The CLI is stopped at its wall deadline and its
process tree is cleaned up. Actual billing is reported separately from reserved
cost; token counting is an estimate and an unexpected usage excess stops further
admission.

Model support, pricing, and counting were checked against the official
[Sonnet pricing](https://platform.claude.com/docs/en/about-claude/pricing),
[token-counting documentation](https://platform.claude.com/docs/en/build-with-claude/token-counting),
and [Claude Code CLI reference](https://code.claude.com/docs/en/cli-reference).
Direct Anthropic authentication and the complete image/action/updated-image loop
were verified in the live run. This establishes one bounded integration smoke,
not autonomous folding competence or general visual reasoning accuracy.

## Exact upstream and reused components

Official source: [robot-agent-exploration/rax](https://github.com/robot-agent-exploration/rax),
branch `dexjoco/native`, exact commit
`4b577aa2ba0039ad01707ede236be9913274180e`. The branch was verified against the
official GitHub branch listing. Its native MuJoCo manipulation path supports dual
arms and Claude Code; no container, official benchmark asset download, training,
or large evaluation is needed here. A task-local Apache-2.0 snapshot is in
`upstream/`, including LICENSE and NOTICE. Shared `/home/jennyok/rax-fireworks/rax`
and other agents' RAX checkouts were left unchanged.

Reused infrastructure:

- `services/benchmarks.py`'s registry/interface mechanism, with one custom entry.
- `services/mcp_contract.py` and `services/controller/{gateway,frontend,supervisor,recording}.py`:
  tool capability filtering, serialized actions, reset/evaluate/finish lifecycle,
  audit trails, and image-bearing MCP responses.
- `harness/mcp_bridge.py`, `harness/agent_host.py`, and
  `harness/claude_cli/{auto_research,login}.py`: native Claude Code command,
  separate home, sandbox, clean credential environment, MCP bridge, process cleanup.
- `services/workspace/`, contract generation, and the robot-neutral workspace runtime.

Adapted components:

- `services/dexmate_box/` implements backend hooks and robot tools around the
  **existing** environment. It replaces Panda/Allegro flange/rotvec/44-D assumptions
  with actual Dexmate TCP/wxyz/16-D semantics. It uses the existing task controller
  directly in the serialized frontend process; upstream's separate DexJoCo
  simulator process, Panda controller and robot-only preview are not used.
- Two small upstream edits register the custom backend before tool inventories
  are derived and permit `sim_gpu='cpu'` for this custom in-process backend.
  EGL rendering is retained without inventing a CUDA GPU UUID.
- Custom robot/perception skills, ROBOT.md, TASK.md and AGENTS.md describe the
  actual embodiment and bounded scope. Generated instructions do not ask this
  smoke agent to train, explore at scale, or submit a benchmark evaluation.
- `launch.py`, `smoke.py` and `claude_relay.py` add local commands and explicit
  model/token/cost/action/time bounds around the unchanged official launcher.

See `UPSTREAM.json`, `upstream_file_hashes.json`, and `upstream-adaptations.patch`
for original file hashes, exact deviations, licenses and provenance.

## Verified action and observation contract

Each row is right `[x,y,z,qw,qx,qy,qz,opening]`, then left same: **16 values**.
Positions are absolute world metres; world is right-handed with z up. Quaternions
are active local-to-world, scalar first, normalized by the existing environment.
TCP sites are `tcp_R`/`tcp_L`, exactly `[0,0,0.16] m` in each gripper-base frame,
with identical axes. This is not an upstream flange pose or a hardware calibration.
Positive opening maps to 0.96 times the command in radians per coupled jaw.
The current zero-opening branch uses bounded closing torque (configured 0.2 N m)
plus damping. Both arm and gripper references are set before the loaded physical
substeps: B273_final uses eighty 0.25 ms substeps, so one row is 20 ms / 50 Hz. `repeat` repeats
a single row without sending duplicate JSON. Tools do not write cardboard state;
the existing physics handles movement through contact.

Robot tools are `dexmate_status`, `dexmate_observe`, `dexmate_step`,
`dexmate_hold`, and `dexmate_gripper`. RAX lifecycle tools provide reset
(`start_episode`), evaluation (`evaluate`) and termination (`finish`). The native
90 s / 4500-row task horizon also terminates execution; later actions are rejected.
Back, ego/head, left-wrist and right-wrist RGB images are supported. Only measured
robot proprioception is exposed to the model; task state stays in operator records.

## Evidence and outcome

The earlier 0.5 ms revision's local checks passed **46/46**, with zero model inference, plus a focused
check that both grippers close under the current zero-opening torque mode.
Both arms tracked +3 mm
world Z to approximately 2.996 mm; a 0.02 rad world-Z rotation verified active
quaternion composition. TCP offsets/axes, normalized wxyz, coupled jaw motion,
action timing, deterministic reset, malformed-chunk atomic rejection, task
registration, four images, stdio/socket MCP, physical evaluation, termination,
video decoding, source hashes, and nine API guard boundaries were checked.
The horizon test uses an explicitly labeled clock fixture; it is not task progress.

The real Claude Code offline fixture made three scripted wire requests: no image
initially, images at step 0, then updated images at step 25. RAX executed one
25-row action for 0.5 simulated seconds; the CLI exited cleanly. No Anthropic
inference occurred. Stable evidence is in [validation/](validation/); complete
records remain in the printed run directories.

The **live** Claude smoke used four `claude-sonnet-4-6` inference requests and
four token counts, completed in 39.18 s, and exited cleanly. Provider usage totals
were 30,874 input tokens including cache creation/reads and 1,040 output tokens.
Claude Code's list-price estimate was **$0.1041366**, independently reconciled
against the provider's usage and one-hour cache-write rates; the final invoice
was not queried. Conservative admission reserved $0.222852, below the $0.50 cap.

The model received all four initial images, sent one 16-D action with `repeat=25`,
and received all four updated images at native step 25. All 25 requested native
rows exactly match the model's action. Measured TCP z displacement was approximately
+2.166 mm right and +2.168 mm left after 0.5 simulated seconds. There was no
robot/cardboard contact and no folding or locking accomplishment. The model's
second response reached its 512-token cap while narrating; it then issued the
action and completed the loop within four admitted requests, without changing limits.
The recorder showed small passive cardboard settling (up to 0.00669 rad crease
change), so the model's phrase that no panels moved is not treated as physical
ground truth. See [live smoke evidence](validation/claude_live_smoke.json) and
[live video](validation/claude_live_smoke.mp4).

After a separate scene rebuild, local revalidation caught a stale adapter
description of the internal physics timestep. The 20 ms action timing and
recorded 0.5 s of motion were correct; the description was corrected to derive
the actual 0.5 ms / 40-substep timing from the loaded environment. Current
gripper closing mode is likewise reported from configuration. No physics setting
was changed by this correction.

The [current-metadata live regression](validation/claude_live_regression.json)
completed using three execution-phase inference requests followed by one read-only
continuation of the same conversation. Its first phase stopped at its request cap
after the single 25-row action; the continuation made no tool calls or additional
motion and sent the step-25 images to Claude. Combined estimated regression cost
was $0.0879069. The compiled model and source/configuration snapshots from this
run are preserved in its private episode directory. The
[API usage audit](validation/api_usage_audit.json) records all eight inference and
eight token-count requests across both runs, totaling 61,555 input tokens including
cache usage and 1,665 output tokens. Resumed CLI totals include earlier phases;
the audit uses each relay's provider usage to avoid counting them twice.

Integration success, partial task progress, and final physical completion are
separate fields. The authoritative `FoldingBoxEnv.progress()` is reused without
relaxation. Physical locking is unvalidated and final completion remains false.
The smoke does not fold the box. Pre-existing physical limitations and material
assumptions are listed separately in [ENVIRONMENT_DEFECTS.md](ENVIRONMENT_DEFECTS.md).

## Run records

Each episode records `native/episodes/<id>/operator.jsonl`, source hashes,
`observations/*/{observation.json,back,ego,wrist_left,wrist_right}.png`, and a
10 fps `episode.mp4` from the existing back camera. Operator JSONL includes each
requested and applied action, motor references/controls, joint state, IK residual,
physical progress, simulation time and termination reason. RAX's `published/runs/`
stores tool calls, errors, exact image-bearing responses and lifecycle outcomes.
New episodes also retain `source_snapshot/` and `loaded_model.mjb`; the latter
stores the exact compiled model, including assets, even if the shared scene is
rebuilt later. Source hashes identify each evidence version.

For a real smoke, `requests/*.request.json` and `*.response.raw` record model
payloads/results without auth headers; `requests/budget.json` records limits,
reservations, provider usage and rejection reasons. `claude-stream.jsonl`,
`claude-stderr.log`, `launch_command.json`, and `smoke_result.json` connect the
model, tool, action, updated observation and episode evidence. Run directories,
local extracted tools and credentials are excluded from source control.
