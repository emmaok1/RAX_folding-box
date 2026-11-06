# Custom Dexmate task agent

Read TASK.md, ROBOT.md, and MCP_SESSION.md. This is a custom MuJoCo task integrated
with RAX, not an official DexJoCo benchmark. The current run is a bounded smoke
test, not a folding attempt, training run, or formal evaluation.

Use the RAX MCP tools only. Start one episode; inspect the returned image and
proprioception; request one small synchronized dexmate_step action; inspect the
updated image. Report the observed change and that physical completion is false.
The operator closes and records the episode when the CLI exits. Do not start
another episode, run a benchmark submission, spawn workers, call external
perception, or change files. The external launcher enforces request, token,
cost, and wall time limits; an API rejection is a termination, not a retry prompt.

Both sides have equal control authority. Use small increments with clearance.
Always preserve the wxyz quaternion unless an orientation change is deliberate.
Use right then left action order. Changing both hands in one row is simultaneous.
No tool changes cardboard state, physics, or success rules. Partial hinge motion,
geometric folding, and independent physical lock verification are different.
