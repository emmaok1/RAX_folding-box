# Current plain cardboard folding task

Attempt to fold the currently loaded video-inspired connected-lid box with the
existing two Dexmate arms and parallel jaws, through actual contact. The blank
starts flat on the existing table. It has no blue grasp aids. It is an engineering
reconstruction, not a manufacturer-measured dieline or a demonstrated solution.

Public design: 300 x 220 mm base crease spans, 90 mm walls, 4 mm rigid panels,
400 g/m2 areal mass; 14 panels and 13 passive creases. Four body walls, four
front/back corner flaps, a connected lid, two lid-side flaps, lid-front flap,
and a hinged tongue with a real front-wall slot. Total mass is 0.1074308 kg.
No box joint has an actuator. Crease resistance and contact properties are
provisional engineering assumptions. Do not change them.

Work toward stabilizing the blank, forming walls and inward corners, an open
box, then folded lid flaps, lid closure and tongue insertion. You must decide
an accessible order from images, not follow a hard-coded joint sequence.
Either arm may stabilize or fold. No debug poses, object teleportation, welds,
attachments, snapping, artificial supports, or direct object-joint commands.

Observe the back, head and wrist images and robot proprioception. Hidden box
coordinates/joint angles are not supplied to the model. Robot tools return
measured poses, timing and IK residuals; a requested pose is not proof it was reached.

Existing task success requires relative wall/corner/lid geometry, whole-section
tab/slot relationships, low speed, released persistence, acceptable penetration,
and independently verified physical retention. Physical locking is unvalidated;
the existing final success guard remains false. Attempt manipulation honestly,
report actual progress and blockers, and never substitute visual closure for
locked assembly success. One episode only; no training, exploration campaign,
controller submission, or benchmark batch. The existing simulation horizon is
90 seconds; cloud/request/wall limits may end the attempt sooner.

<!-- TEACHER_TASK_OVERRIDE -->
# Authoritative teacher-specific task contract
For this privileged teacher role, the ordinary RGB-only/no-hidden-coordinates,
image-only ordering, one-episode and no-controller restrictions above are superseded.
Use fresh authorized simulator ground truth and the read-only world definition.
The current TEACHER_AUTHORIZATION.json and host gates set the exact episode/time budget.
Bounded exploration and observation-driven controller register/rehearse are allowed.
Formal submission/evaluation is forbidden. All physical prohibitions and the unchanged
physical-locking success guard above remain in force. Follow TEACHER.md execution stages.
