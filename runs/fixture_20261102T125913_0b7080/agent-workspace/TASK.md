# Custom cardboard_box_folding

Fold the current SM1-derived connected-lid gift-box blank using the Dexmate arms and
grippers through physical contact. Keep the existing scene, geometry, materials,
passive creases, and physics. No object teleportation, debug folding, welds,
adhesion, active box joints, or relaxed evaluation criteria.

The current separate box/material work uses a DigiFlute E material substitution
and engineered extra scores, recorded in the existing physics.json and geometry.
Use the current task as loaded; integration does not rebuild or revise it.

Existing physical criteria are authoritative in FoldingBoxEnv.progress(): four
body walls and corners near pi/2 (0.22 rad tolerance), upright body and vertical
walls, double-wall return folds, actual tab/slot capture relationships, lid and
lid flaps near pi/2, low body speed (<0.03 combined norm) and hinge speed
(<0.05 rad/s), no robot/cardboard contact, cardboard penetration <0.0001 m,
and at least 5 s released persistence. Debug poses never count.

Physical locking must additionally pass independent insertion/retention
validation. The current environment sets physical_locking_verified=False, so
final succeed/reward remain false. Preserve this gate. Geometric capture or a
released snapshot alone is insufficient. The existing physical task has not
demonstrated full folding or locking; integration success only confirms the
software/control loop. This task is not an official DexJoCo benchmark.

For the smoke test, only observe, move in clear free space by at most a few
millimetres using the measured poses, then report the updated observation.
