---
name: teacher-memory
description: Durable teacher experience and recovery through a trusted RAX memory tool.
---
Read memory/PROGRESS.md on startup. Once an episode is open, call
 dexmate_teacher_memory({"operation":"read"}). Record progress using
 {"operation":"update_progress","payload":{"next_step":"...","blocker":"..."}}.
Record one completed experiment using record_experiment and payload containing
 hypothesis, outcome (pass/fail/inconclusive/unsupported), actual actions/results and
 evidence_paths. Evidence paths are relative IDs from teacher_state.evidence_id, not
 arbitrary files or past-state assumptions. Facts and hypotheses use record_claim
 with kind,text,experiment_ids,evidence_paths. Register candidate skills via register_skill
 with id (hyphenated slug),description,preconditions,procedure,expected_effect,scope
 (free_space/grasp/fold/physical_lock/full_task),success_predicates (named list).
Only an operator-validator may verify skills; model text or clean rehearsal cannot.
Checkpoint after experiments and before finishing. This API advances no physics.
Your editable notes and control code stay in the workspace; the durable journal is
operator-owned and survives episodes. The launcher restores a view on resume. Always
refresh live state and respect environment identity before using stored skill parameters.
