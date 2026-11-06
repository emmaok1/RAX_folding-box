# Existing Dexmate action contract

Dexmate Vega 1 U, two 7-joint arms and two coupled DexGripper S jaws; 16 robot
actuators. Base, table, cameras and robot placement are unchanged.

The RAX dexmate_step tool accepts 1..50 rows, each exactly 16 numbers:
right [world x,y,z, qw,qx,qy,qz, opening], then left with the same eight fields.
Positions are metres in the MuJoCo world, z up. Unit quaternions are active
local-to-world, scalar first wxyz. Optional repeat=1..50 repeats one row;
at most 50 total rows per tool call. Both arms are staged before physics.
Opening is 0 closed, 1 open. Positive commands target 0.96*opening radians;
zero uses the existing bounded closing-torque controller, not a grasp attachment.
Current loaded settings: 0.4 N m closing torque, 0.6 N m maximum gripper torque,
0.25 ms physics timestep, 80 substeps per 20 ms control row (50 Hz). The live
dexmate_status and observation action_layout are authoritative.

TCP sites are 0.16 m along each gripper_base local +z, with the same axes.
These are provisional virtual TCPs, not calibrated grasp centres. Deliberate
orientation changes must use wxyz quaternions. Preserve each arm's own pose
when holding it; all-zero seven-value pose holds its previous reference.
One repeated target may need several chunks to reach it because references
have velocity limits and motion is motor/IK/PD driven. Check actual TCP and
IK residual feedback. Do not mistake a distant unreachable reference for motion.

Tools: dexmate_status (no stepping), dexmate_observe (camera subset allowed,
no stepping), dexmate_step (Cartesian actions), dexmate_hold (hold references),
dexmate_gripper (change openings, hold arms). start_episode starts one reset;
evaluate checks unchanged criteria; finish closes the attempt.

Images: back, ego/head, wrist_left and wrist_right. Proprioception is measured
TCP poses, arm joint positions/velocities, jaw positions, and opening commands.
Hidden object states are operator records, not model observations. There is
no automatically executable fold(panel) skill in this adapter.

## Stage-1 free-space planner

New dexmate_motion_plan/preview/execute/status/stop tools are distinct from
legacy Cartesian actions. They use world metres and active wxyz TCP poses;
plan/preview do not advance physics. Execute uses cached seven-joint arm paths
through the unchanged joint-reference/PD interface, at 20 ms and <=50 rows.
Both arms are reserved during execution; the inactive arm holds its measured
start, only after references/joints/jaws are settled and mutually consistent.
Jaw changes, contact/grasp/held objects and synchronized two-arm paths are not
supported. An all-zero legacy Cartesian pose still retains the previous target,
not the measured pose.

The current cardboard state is NOT read for planning. Its dynamic coverage is
unknown, so the current box scene refuses motion-plan execution explicitly.
Static diagnostic fixtures may validate the robot/table pipeline only; they do
not demonstrate cardboard avoidance. No privileged collision oracle is enabled.

STOP: normal MCP calls serialize. For interruption during an executing call,
the operator uses `python -m rax_integration.motion_planning.operator_stop RUN_ROOT`.
The executor checks that flag before each 20 ms control row and freezes further
simulation after the current row; this is not a hardware brake or a substep
latency guarantee. Plans are single-use and invalidated by changed state/source.
The bound is one current control row in simulation time, not 20 ms wall time:
rendering, callbacks or a blocked native call can delay acknowledgement. The
planning worker polls the flag every 10 ms and is terminated on cancellation
(1 s graceful termination timeout before kill). If acknowledgement stalls,
the operator must terminate the owning frontend/agent process group; a queued
MCP stop alone cannot interrupt serialized execution. In-flight cloud requests
cannot be unbilled; no cloud run is authorized by these planning tools.
Static collision geometry provenance is the loaded scene XML and its compiled
robot/table meshes, bound to source and world fingerprints. No estimated
cardboard world is currently available; unknown dynamics are excluded globally
by refusing planning, rather than silently treating their missing geometry as
free space. Adding estimated obstacles later requires explicit provenance,
uncertainty bounds and refresh/staleness rules before coverage can be accepted.
Inspect command_completed, measured goal_reached and goal errors separately.
Collision checks use the compiled runtime convex collision representation;
structural same-body/welded, source-explicit and adjacent attachment exclusions
are documented in the audit. Edges are discretely validated, not certified CCD.


Full-scene planning policy: a run may explicitly select local simulator-assisted collision planning using motion_policy.json with {"world_policy":"simulator_snapshot"}. This retains the complete robot/table/cardboard model and copies actual non-robot joint state into private collision data. It is privileged simulator-state-assisted planning, NOT a vision-only experiment. Without that explicit run policy the robot_static mode still refuses unknown dynamics. No object coordinates are added to the RGB/proprioception observation packet. Plan and preview do not step physics; changed object geometry or policy invalidates execution. A local free-space proof is not grasping, folding, or locking success.
