# RAX privileged teacher and persistent memory

This is a teacher **role in current RAX auto-research**, using its normal Claude
workspace, sandboxed file/Python tools, experience-memory and handoff skills,
controller registration, and bounded rehearsals. It does not enable the retired
`mode=teacher/student` configuration. The authoritative capability is serialized
`Configuration.teacher_privileged=True`; ordinary configurations default to false.

The teacher has explicitly privileged simulator world/state/contact observations.
The student/ordinary RGB contract stays separate. Static `runtime/world` is a
content-addressed world definition; `dexmate_teacher_state` is current owned-episode
state. Neither a saved pose nor an old trace is current perception.

Current authorization covers **coding and local checks only**. No new cloud budget,
training, full folding episode or formal evaluation is authorized or launched.
The environment still withholds physical-locking/task success. Clean program or
rehearsal completion cannot satisfy that physical-success guard.

## Offline preparation and inspection

Run from the box project:

```bash
.venv/bin/python -m rax_integration.teacher.prepare --root rax_integration/runs/teacher_example
.venv/bin/python -m rax_integration.teacher.run --root rax_integration/runs/teacher_example
```

The safe settings template is also usable directly:

```bash
.venv/bin/python -m rax_integration.teacher.prepare --root rax_integration/runs/teacher_example_from_settings --settings rax_integration/teacher/settings.example.json
```

It sets a zero-dollar ceiling. Settings are preparation parameters, not a user
authorization; changing the example cannot enable paid execution.

The root must be fresh. Preparation copies the full current world, creates the
normal Claude/RAX workspace and skills, enables teacher capability, prepares
explicit simulator-snapshot planning policy and structured memory. It does not
start a frontend, simulator episode, viewer, model request or training job.
The second command inspects prepared metadata; it does not execute anything.

Preparation defaults to an API ceiling of zero and an explicit bounded future
episode schedule. `--api-budget-usd` configures a ceiling only; it grants no spend
or episode permission. Source/config/policy tampering invalidates prepared state.

`teacher_policy.json` describes the role and preparation scope. A future paid
session requires **both** fresh global `CURRENT_SCOPE.json` authorization and a
matching per-root `cloud_authorization.json`, a matching `authorization_task_id`,
an unconsumed grant, a positive approved ceiling and explicit operator CLI approval.
The per-session grant must explicitly cover request, wall-time and episode caps;
a dollar allowance alone does not authorize more episodes or model calls.
Historical grants cannot pass these checks. Secret keys stay outside the project.
`run --execute` exists for a future authorized operator; it is not used by local
validation. There are no automatic retries, resumed model invocations, or held-out
formal runs. A wall/API/STOP limit ends that invocation.

The future execution path uses the actual native Claude CLI with normal RAX tools;
it does not remove file/code tools with the old `--tools ''` restriction and does
not replace model planning with a fixed folding sequence. The relay bounds all
model requests/tokens/reserved dollars. It is local transport code, not a new API
authorization. Known response costs and requests without recorded usage are kept
separate in `teacher_result.json`.

## Memory and transfer

See [CONTRACT.md](CONTRACT.md) for the structured operator store and verifier schema.
The model can record experiments, claims, candidate skills and progress, but cannot
call `verify_skill`. Verification requires trusted measured evidence, an exact
environment/skill identity and an intact pinned trace. Simulator consistency is
not validation against a real cardboard box. Synthetic tests are not learned robot
skills.

Verification additionally binds `source_identity`: the exact executed manual trace
or frozen RAX controller-bundle manifest, not just a skill's descriptive text.
Changing code/bundle identity cannot inherit verification from the old procedure.
On environment change, active progress resets to `no_current_progress`; earlier
progress is retained as history and old claims/experiments are explicitly labeled
historical. Same-environment notes can survive a rehearsal, but must never be
confused with fresh episode state. The selected environment/version verification,
not the last history entry from another environment, supplies exported confirmation.

The operator store lives outside model-writable memory. Exported workspace copies
are notes, never read back as authoritative during that session. Startup and
post-agent-exit publication preserves authored progress/index/tips/handoff files
verbatim, publishes OPERATOR summaries and operator_skills separately, and writes
memory/RESTORE.json atomically last as its completion pointer. Publication must
not overlap a running model/controller. Individual copies are not a multi-file
transaction; recovery uses the private journal or last verified frozen bundle.
Code and handoff travel through actual RAX
`transfer.pack` / `verify` / `unpack`, not an invented teacher lifecycle.

`checkpoint_session(root)` writes `teacher_checkpoint.json` containing the frozen
memory/code/handoff package and manifest. For a new prepared session,
`prepare_session(..., carry_bundle=..., carry_manifest=...)` verifies that package,
restores evidence and the operator store, and rebinds it to current source identity.
Changed source/physics/controller/evaluator/policy invalidates old skill
verification; it does not delete history. Recovery never teleports simulator state,
replays a trajectory, starts an episode or authorizes cloud continuation. Query
current RAX status before acting after recovery.

## Cancellation and visualization

Out-of-band `motion_planning/STOP` is checked by the bounded launcher and action
paths; it is independent of serialized MCP calls. A future `--live` viewer follows
actual backend state snapshots. It is a state mirror, not another robot controller
or an independent simulator run. Native display must be available explicitly.
S/closing that viewer posts STOP. No viewer is opened by preparation or tests.

The planner handles unladen free-space motion under explicit simulator snapshot
policy. Intentional contact/held cardboard needs separately scoped original
bounded controls. Robot joint references still execute through the original motor
controller. No material, robot, controller, collision or evaluator changes are made
by this package.

## Reproduce local checks

```bash
.venv/bin/python -m unittest rax_integration.teacher.tests.test_memory rax_integration.teacher.tests.test_bundles rax_integration.teacher.tests.test_prepare -v
```

Memory tests use synthetic temporary artifacts; workspace tests use the actual
upstream generator with a tiny explicitly mocked world and never start an episode.
They cover evidence/invalidation, crash recovery, concurrent writers, bundle tamper
and size refusal, code/memory/handoff carry, capability configuration, normal tool
availability and refusal of unpaid execution. They do not establish a successful
grasp, fold, locking mechanism, or a tested live paid teacher run.

## Explicit teacher reasoning policy

Teacher preparation selects `teacher_adaptive_high`: the bounded relay sends
`thinking={"type":"adaptive"}` and `output_config.effort="high"` for pinned
Sonnet4.6. The token-count request uses the same thinking mode. Historical callers
without this explicit policy retain disabled thinking. Teacher output is capped
at8192 tokens per request INCLUDING thinking and visible output, and the full cap
is reserved against the existing dollar/token ledger. No provider execution is
claimed by the local scripted-wire checks. Native Claude sandbox execution and
actual paid teacher behavior still need prelaunch/live validation.

## Verified execution mode on this host

An offline native-CLI Bash probe was inconclusive: listable denied directories
may be empty masks, and no credential bytes were read. That probe does not
establish a credential leak or a failed sandbox. Native command tools are not
relied on and remain disabled in this configuration. Teacher launches use
`--restricted --tools Read,Write,Edit`, explicit settings, and read-only world/run
references. Skills are read as files. Python controllers execute only through
RAX register/rehearse in the tested BundleSandbox. An actual scripted native-CLI
check verified canary-read denial, world-write denial, workspace/code writes,
Bash unavailability, and a clean frozen Python rehearsal at step0. No provider
inference or successful physical skill is implied by that check.
