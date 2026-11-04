# Teacher memory contract

`MemoryStore` is persistent data storage for the trusted RAX integration. It is
not a retired RAX teacher mode, a model client, a robot controller, or an episode
launcher. It never resets, resumes, trains or moves a robot. Restoring memory does
not authorize cloud spending or starting another episode.

The integration owns observation/world policy and the source files selected for
the environment identity. Include scene and robot assets, physics/configuration,
controller, evaluator, tool/interface and observation/world policy identities.
`capture_identity(sources, metadata=...)` hashes named files; alternatively pass an
explicit nonempty identity mapping. `store.environment_id` is its canonical digest.

```python
from rax_integration.teacher import MemoryStore, capture_identity

identity = capture_identity(sources, metadata={"world_policy": selected_policy})
store = MemoryStore(private_store_path, identity, evidence_root=operator_evidence)
store.record_experiment({
    "id": "unique-experiment-id", "hypothesis": "...",
    "outcome": "pass",  # pass/fail/inconclusive/unsupported diagnostic outcome
    "parameters": {}, "observed": {}, "evidence_paths": ["run/trace.json"]})
store.record_claim(kind="hypothesis", text="...", experiment_ids=[])
store.update_progress({"episode_id": current_episode, "next": "read live status"})
candidate = store.register_skill({
    "id": "short-transfer", "description": "...", "scope": "free_space",
    "preconditions": "...", "procedure": "...", "expected_effect": "...",
    "success_predicates": ["measured_goal_reached", "no_contact"]})
store.verify_skill("short-transfer", "run/operator-verification.json")
verified = store.skills(verified_only=True)
store.checkpoint()
```

Experiment outcomes and facts/hypotheses never promote a skill. Optional skill
fields `executed_action`, `action_kind`, `pose_recipe`, `point_reacquisition`, and
`task` carry observation-relative action records into RAX learned-memory files.
Unrecorded pose/action details remain explicitly unrecorded.

Only trusted operator validation may call `verify_skill`. Evidence root and
validator files must not be writable by the learned model. The store checks
structure, declared checks, source/skill identity and evidence hashes; it does
**not** independently reconstruct physical success from a video or verify the
truth of an arbitrary JSON assertion. The integration must generate validation
artifacts from measured execution and evaluator results, not model narration.

A verification artifact has these fields:

```json
{
  "skill_id": "short-transfer",
  "skill_version_sha256": "digest returned by register_skill",
  "source_identity": {
    "kind": "rax_bundle",
    "sha256": "exact frozen RAX controller bundle identity",
    "evidence_path": "run/executed-bundle-manifest.json"
  },
  "environment_id": "store.environment_id",
  "passed": true,
  "method": "robot_execution",
  "episode_id": "actual episode",
  "step_ids": [0, 20],
  "episode_mode": "exploration",
  "level": "local_effect",
  "checks": {"measured_goal_reached": true, "no_contact": true},
  "prohibited": {
    "object_state_writes": false, "artificial_attachment": false,
    "box_joint_actuation": false, "playback": false
  },
  "evidence_paths": ["run/measured-trace.json", "run/decisive-frame.png"]
}
```

Step IDs must strictly increase. All required predicates must be present and true
for promotion. `level=task_complete`, `scope=physical_lock`, or `scope=full_task`
also require `native_evaluator.task_complete=true` from the unchanged evaluator.
This does not manufacture task success while the current physical-lock guard is
false. Failed measured verification (`passed=false`) records a refutation without
deleting earlier history. Unit tests use synthetic operator fixtures only; their
passing results do not establish a robot skill or a real cardboard measurement.

The candidate skill must declare the SAME `source_identity` as its operator
verification. A source-less candidate can be stored but cannot be verified.
`rax_bundle` pins an operator-captured executed bundle manifest with matching
`sha256` and a nonempty file manifest. RAX's execution path remains responsible
for verifying those actual frozen code files; this memory store never executes
or independently attests them. Manual experience instead uses
`kind=manual_tool_trace` with the actual trace-file SHA256. Source evidence is
pinned alongside measured evidence and rechecked on retrieval. Call
`skills(source_identity=currently_selected_source)` to reject a skill from
different executable code. A verified frozen bundle never confers verification
on a mutable workspace file or a new bundle with unchanged descriptive text.

Evidence paths are relative to `evidence_root`; traversal, symlinks and special
files are rejected. Retrieval rechecks pinned hashes. A changed identity, skill
definition, missing/changed trace or failed later verification removes current
verified status. Old history remains accessible and explicitly labeled. A store
instance may not write after another session activates a different identity.
Activating a changed identity resets active progress to empty notes with
`status=no_current_progress`; earlier progress remains in `progress_history`.
Read/export projections mark old claims and experiments
`applicability=historical_environment_only` and `current_environment=false`.
Resuming the same identity retains progress; live episode/step must still be
checked by integration rather than treating earlier notes as current simulator state.

Mutations use process/thread serialization, a hash-chained fsynced journal and
actual upstream `services.controller.storage.persist`. Recovery replays committed
events if a crash interrupts snapshot writing, preserves/truncates only an
unterminated final write, and refuses corruption of complete committed records.
`recover()` and `snapshot()` read/recover metadata only, never simulator state.

`export_workspace(fresh_path)` creates RAX `memory/PROGRESS.md`, `INDEX.md`,
`TIPS.md`, learned `skills/*/{SKILL.md,evidence.json}`, structured data/journal,
pinned evidence, and `handoff/README.md`. Learned evidence uses RAX schema 4
fields with explicit operator-attestation extensions; `diagnostic` is a local
mode extension and is not an exploration/rehearsal result. Hypotheses, stale and
refuted skills remain labeled. Historical unavailable evidence is enumerated.

`freeze_bundle(fresh_destination, max_mb=10)` uses actual upstream
`services.transfer.pack` and `storage.verify`; size-excluded/partial packages are
not registered. It freezes memory, optional code and handoff, not an executable
controller submission. `teacher.bundles.restore_workspace(package, manifest,
fresh_destination)` verifies the bundle and calls upstream `unpack`. Reopen a
restored store at `memory/store`, with evidence root `memory/evidence`; use current
environment identity so old verification cannot silently survive a source change.

Local checks (no simulator, cloud or paid model):

```bash
.venv/bin/python -m unittest rax_integration.teacher.tests.test_memory rax_integration.teacher.tests.test_bundles -v
```
