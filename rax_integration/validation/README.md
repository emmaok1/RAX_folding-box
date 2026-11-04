# Validation evidence

Latest frozen task: **B273_final**. [Handoff check](B273_handoff.json) verified
all eight frozen hashes and read-only 0.25 ms / 80-substep metadata. No simulator
or model was run in this handoff. The 46 checks and paid smoke evidence below
describe the earlier recorded revision, not B273_final. Box-builder's
[pickup report](../../pickup_iterations/REPORT.md) records two deterministic
whole-blank lift/10-second-hold/release successes. Full folding and locking
remain unverified.

- [Summary](SUMMARY.json): 46 current local checks passed, a focused current-mode
  gripper check, completed user-authorized live loops, exact source provenance,
  and no credential values found in integration source/run records.
- [Current live regression](claude_live_regression.json) and
  [video](claude_live_regression.mp4): correct 0.5 ms / 40-substep metadata,
  one model action repeated 25 times, followed by a read-only continuation that
  sent the updated images to Claude. The initial three-request cap stop is retained.
- [API usage audit](api_usage_audit.json): 8 total inference requests, 8 token
  counts, estimated $0.1920435, reserved $0.433074 within the $0.50 combined cap.
- [Live Claude smoke](claude_live_smoke.json) and [video](claude_live_smoke.mp4):
  four actual Sonnet 4.6 requests, 39.18 s, estimated $0.1041366, one model-requested
  action repeated for 25 native rows, followed by updated images sent to Claude.
  All native requested rows match the model tool call exactly. The video was
  decoded independently: six nonblank changing frames at 640x480, 10 fps.
- [Local checks](local_checks.json): actual 16-actuator / 16-D action embodiment,
  world-frame TCP offsets and axes, wxyz quaternions and active rotations, 50 Hz
  control / 0.5 ms substeps, synchronized motion, grippers, reset, native termination,
  atomic rejection, official RAX MCP transports and nine API guard boundaries.
- [Zero-opening gripper check](gripper_zero_mode_check.json): both jaws close
  physically under the current bounded-torque mode, without model inference.
- [Claude Code fixture](claude_offline_fixture.json): real Claude Code 2.1.316,
  official RAX launcher and MCP bridge, scripted local Anthropic SSE responses.
  Three wire requests carried images at steps 0 and 25. **No model inference.**
- [Fixture video](claude_offline_fixture.mp4): one 25-row synchronized action,
  0.5 simulated seconds, existing back camera. The +2 mm references produced about
  +2.16 mm measured TCP displacement at this transient endpoint on both sides.
- [Before](before_back.png) and [after](after_back.png); initial
  [ego](before_ego.png), [left wrist](before_wrist_left.png), and
  [right wrist](before_wrist_right.png). All four streams were visually inspected.
  Reset wrist views show the open jaws; the blank is visible in the back/head views.
- [Environment source hashes](environment_source_hashes.json).
- [Current live source hashes](live_environment_source_hashes.json) and
  [post-initial-smoke rebuild audit](post_live_source_changes.json). The initial
  44-check suite used the earlier 1 ms configuration; the 46-check suite revalidated
  the updated configuration. Historical result files remain separate.

The local control suite's settled +3 mm references produced approximately
+2.996 mm on both arms. It also checked a 0.02 rad world-Z orientation change.
The separate horizon check deliberately advances a test fixture's simulation
clock near the 90 s limit. That fixture is not evidence of 90 s task execution.

These results establish local integration, transport and a real Claude-controlled
free-space smoke. The separate API fixture remains scripted. Neither test
demonstrates autonomous folding, physical tab insertion, or locking. The existing
physical success gate stays false. In the live smoke the robot did not contact
the cardboard; small passive settling occurred, contrary to the model's literal
claim that no panels moved. Operator state, rather than model narration, decides
physical progress. See the live smoke report for measured changes and usage.
