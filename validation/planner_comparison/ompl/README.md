# OMPL + MuJoCo smoke test

This isolated route uses the published `ompl==2.0.1` CPython 3.11 wheel and its real `geometric.RRTConnect` planner. Dependencies are local to `deps/`; the root environment and simulation sources are not modified.

The adapter plans seven right-arm joints within model joint limits. It delegates validity to the shared MuJoCo callback, uses a 0.15 rad Euclidean extension range and at most 0.005 rad Euclidean interpolation spacing, and accepts only exact OMPL solutions. OMPL shortcutting uses the same validity callback. There is no alternative planner or straight-line success fallback. This is discrete collision checking, not a continuous collision guarantee.

`setup.json` records package/platform information and binary hashes. Per-call planner JSON records import, setup, actual search and simplification times separately; the path is saved as NumPy data. `adapter_setup_smoke.log` is an API-only obstacle detour smoke test, not robot execution evidence.

Official references:
- https://github.com/ompl/ompl/blob/main/doc/markdown/python.md
- https://ompl.kavrakilab.org/core/classompl_1_1geometric_1_1RRTConnect.html

Install, if needed:

```sh
.venv/bin/python -m pip install --target validation/planner_comparison/ompl/deps --no-deps ompl==2.0.1
```

Robot smoke completed: PASS. Run `MUJOCO_GL=egl .venv/bin/python validation/planner_comparison/ompl/run_smoke.py`. See `REPORT.txt`, `summary.json`, and `run/result.json` for evidence and limitations.
