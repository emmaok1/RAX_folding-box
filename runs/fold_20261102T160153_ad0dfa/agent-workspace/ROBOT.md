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
no automatically executable fold(panel) skill or planning tool in this adapter.
