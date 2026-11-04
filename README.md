# Dexmate box folding

Current video-inspired plain box: 14 panels, 13 passive creases, 4 mm board,
107.4308 g; no blue grasp aids. Existing robot, table, cameras and controller
are unchanged. Full folding and physical locking remain unverified.

```bash
./run.sh viewer --paused
./run.sh reset
./run.sh rax-fold --prepare
```

RAX cloud entry: `./run.sh rax-fold --live --key-file <private-key-file>
--approve-budget-usd <approved-cap>`. The scope file and explicit cost cap must
permit the run. This starts the real Claude Code / RAX observation-action loop;
it contains no fixed folding order. See `rax_integration/fold_config.json` for
request, token and time limits.
