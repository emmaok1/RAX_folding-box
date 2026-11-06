# Pure Tesseract + TrajOpt CPU smoke

Final corrected run: upward motion passed; approach optimization reached its10s limit and returned no path. Contact descent/retract were therefore not attempted. This is an integration smoke, not a folding experiment or broad planner ranking.

| Stage | Setup | Planning pipeline | Result |
|---|---:|---:|---|
| Upward5mm |2.032s|1.076s|11waypoints; independent MuJoCo validation passed; motor execution passed|
| Approach above base |2.033s|11.558s|TrajOptMotionPlannerTask OPT_TIME_LIMIT;0waypoints|

Upward measured displacement was4.993158mm vertically; final TCP target error20.65µm; maximum joint target error6.19e-6rad. Independent validation checked11samples with maximum joint increment0.005rad. Shared monitor observed8,880physics substeps across the run, no MuJoCo warnings or robot/base contact. No executed-state teleports, force injection, collision or physics changes were used.

The pipeline is explicitly `TrajOptPipeline`: ProcessPlanningInputTask → FormatPlanningInputTask → MinLengthTask → TrajOptMotionPlannerTask → DiscreteContactCheckTask → IterativeSplineParameterizationTask. Optimizer is real TrajOpt sequential convex optimization with the OSQP solver profile. The package bundles other planners; no OMPL planner was called, seeded, or used as fallback. The10s bound applies to solver optimization; reported planning time covers the whole pipeline and can exceed it.

Defaults retained:0m hard collision margin/coefficient10,0.005m soft margin/coefficient50, continuous longest-valid-segment collision evaluation with0.1segment length. This soft clearance preference differs from a zero-margin validity-only planner. Diagnostic101point sampling of the direct approach seed found no Tesseract penetration, but right arm links5/7 have2.850mm clearance at the endpoint. That is below the5mm soft margin and may contribute to optimization difficulty; causation was not established. No clearance tuning was performed.

`export_model.py` rebuilds the planner model from the compiled MuJoCo snapshot: all71collision geometries,69convex mesh hulls including custom jaw pads, table box, and floor. The infinite floor uses a200m×200m×2m box with the exact top plane; this finite representation covers the robot's entire workspace. Right arm's seven joints are articulated; other bodies, grippers, box panels, and left arm are frozen at the planning snapshot. All six-configuration body/geom/TCP transforms agree within6.78e-11 absolute matrix units. Original geom collision masks, body exclusions, welded-body and parent-body filtering are preserved. Accepted paths additionally use the original MuJoCo model for independent dense validation, and execution is through the shared motor-reference interface.

The first run was preserved in `run_initial/` and `run_initial.log`. It likewise passed upward and timed out approach. Review found one exporter parity bug: its parent-body exemption incorrectly included the world body. MuJoCo exempts world-parent pairs from this filter. This was corrected and exactly one rerun was made, with identical target, solver limit, and collision settings. Final evidence is in `run/`, including result.json, per-stage planner metadata, model manifests/URDF/SRDF, direct seed collision audit, motor trace, and screenshots.

Reproduce from project root: `bash validation/planner_comparison/trajopt/run.sh`. Dependencies are isolated under this directory: published `tesseract-robotics-nanobind==0.35.0.8` wheel and `loguru==0.7.3`; shared root venv was not modified. The wheel requires a newer GLIBCXX than the host default; a copy of the already-installed libstdc++.so.6.0.34 is isolated in `runtime/` and selected only for this process. `provenance.json` records source/runtime hashes and primary documentation links. No core or upstream source was edited.
