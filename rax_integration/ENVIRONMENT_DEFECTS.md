# Pre-existing environment and task limits

B273_final handoff update: box-builder reports two deterministic whole-blank
pickup, 10-second hold and release successes. The older statements below about
unverified whole-blank pickup are historical. The latest report still identifies
transient penetration up to 0.719 mm, closed-geometry interference up to 1.403 mm,
and resting hinge speed 0.150 rad/s. Full folding and physical locking remain
unverified. See ../pickup_iterations/REPORT.md. Cloud API verified frozen hashes
and read-only timing metadata; it did not rerun those physical trials.

These findings come from the current environment, DESIGN.md, and
validation_review/REPORT.md. The RAX integration preserves them; it changes no
scene XML, cardboard geometry, collision settings, material, solver, gains,
controller, or success criterion.

The separate material/box work has since substituted DigiFlute E settings
(configured 1.53 mm thickness and engineered extra scores), changed the physics
timestep to 0.5 ms, and added a zero-opening closing-torque mode. The earlier
validation_review results below describe their recorded earlier configuration;
they do not establish physical performance of this newer material variant.
Current per-episode configuration, source snapshots and compiled model are
recorded by this integration without changing them. Final physical-lock success
is still disabled in the current environment.

- Physical locking is unvalidated. `FoldingBoxEnv.progress()` deliberately sets
  `physical_locking_verified=False`, so final reward/success cannot be true.
  Preserve this until independent insertion and retention validation passes.
- Prior task evidence demonstrates supported local gripping and about 16.6 degrees
  of passive wall motion. Whole-box lifting, full 90-degree folding, robot-driven
  tab insertion, and released physically locked closure have not been demonstrated.
- The closed debug relaxation has continued lid-tip/inner-wall motion and contact
  penetrations above the 0.1 mm task threshold. A debug closed pose is not a robot
  achievement or a verified lock.
- Thin cardboard is represented by rigid panels, passive crease joints, and four
  provisional non-crease bend regions. Plastic history, material hysteresis,
  thickness, grip force, and lock retention have not been calibrated from hardware.
- MuJoCo filters directly connected parent/child collisions. Independent SAT
  checks cover those pairs, but the simulation still cannot represent all local
  cardboard bending or remote contact between such panels.
- Jaw pads, gripper limits, gains, mounting, and virtual camera optics include
  documented simulation assumptions. The TCP is the existing virtual site, not a
  measured hardware tool calibration. This adapter makes no hardware safety claim.

The successful integration checks and offline scripted CLI fixture are software
and control evidence. Neither is evidence of a complete folding solution.
