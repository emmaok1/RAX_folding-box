# Actual embodiment and action contract

Dexmate Vega 1 U from dexmate-ai/dexmate-urdf commit
ffcc62500fea1e2e7600411b4ec9f3b420bb780a, wrist FT / DexGripper S variant.
The base, stand, torso, and head are fixed. Each arm has seven motor-driven
revolute joints L_arm_j1..7 / R_arm_j1..7. Each gripper has two coupled jaws;
there is one jaw motor per side, sixteen robot actuators total. No Allegro hands.

`dexmate_step.actions` is a list of 1..50 rows. Every row has exactly 16 values:
right [world x,y,z in metres, qw,qx,qy,qz, opening], then left same.
Optional `repeat=1..50` repeats one row, avoiding large duplicate action payloads.
Opening is 0 closed to 1 open. Positive opening maps to a jaw reference of
0.96 times the command in radians. With the current controller, zero opening
uses bounded closing torque (configured 0.2 N m) plus damping; it is not a
position-PD reference at zero. Live dexmate_status reports the loaded mode.
Measured jaw motion has dynamics and need not equal its reference immediately.

Positions are absolute in the existing right-handed MuJoCo world, z upward.
Rotations are active local-to-world unit quaternions, scalar first wxyz;
nonzero input quaternions are normalized. There are no Euler or rotation-vector
action fields. TCP sites tcp_R and tcp_L are 0.16 m along local +z of each
gripper_base, with the same orientation as that base. This is the existing
virtual TCP, not a newly calibrated grasp center or an upstream flange pose.
World displacement is not tool-local displacement. Use measured TCP pose as the
starting reference; preserve each arm's own orientation.

Both arm/gripper references are staged, then the unchanged existing IK + limited
joint-reference velocity + PD/gravity-compensation controller advances the
configured MuJoCo substeps. One row = 20 ms = 50 Hz; a 50-row chunk = 1 simulated second.
The historical B273_final physics step was 0.25 ms (80 substeps per row). The new
redesign is awaiting handoff; its timing has not been verified. Live dexmate_status and
every observation derive timing from the loaded environment, not a fixed constant.
The task horizon is 90 simulated seconds (4500 rows). The wrapper additionally
enforces a wall deadline. Positive-opening jaw references slew at the configured
1 rad/s; zero-opening torque mode does not impose a measured jaw velocity cap.
The existing provisional torque limit is 0.3 N m. Arm effort/velocity limits and jaw-pad
assumptions are in the unchanged robot_config.json and source URDF.

An all-zero seven-value pose holds that arm's previous TCP reference; it does
not mean a world-origin target. dexmate_hold preserves references. dexmate_gripper
changes jaw references while holding both arms. Observe/status do not step.
start_episode resets through FoldingBoxEnv.reset; evaluate calls its physical
progress criteria; finish closes the episode. Terminal episodes reject actions.

RGB cameras: back = existing fixed scene camera; ego = virtual head camera;
wrist_left/right = handcam_rgb_left/right. Virtual camera calibration is
provisional, so this integration exposes RGB and measured robot state only.
Proprioception arrays follow the existing right-then-left order: TCP 14 values,
arm position/velocity 14 each, jaw positions 4. Hidden object/joint state is
recorded for the operator and is not model observation.
