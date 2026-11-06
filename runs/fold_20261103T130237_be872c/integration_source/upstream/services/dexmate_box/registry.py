"""Import-light custom backend registration in the official RAX registry."""
TOOLS = frozenset({'dexmate_status', 'dexmate_observe', 'dexmate_step', 'dexmate_hold', 'dexmate_gripper','dexmate_motion_plan','dexmate_motion_preview','dexmate_motion_execute','dexmate_motion_status','dexmate_motion_stop'})


def entry(Benchmark):
    return Benchmark(
        name='dexmate_box', label='Custom Dexmate cardboard folding',
        adapter='services.dexmate_box.benchmark', robot_tools=TOOLS,
        geometry_tools=frozenset(), unscoped_tools=frozenset(),
        mutating=frozenset({'dexmate_step', 'dexmate_hold', 'dexmate_gripper','dexmate_motion_execute','dexmate_motion_stop'}),
        previewable=frozenset(), status_tool='dexmate_status', observe_tool='dexmate_observe',
        tool_prefixes=('dexmate_',), episode_tool_prefixes=('dexmate_',),
        simulator_modules=('services.dexmate_box.backend',), startup_seconds=60,
        robot={'benchmark': 'Custom Dexmate cardboard folding', 'status': 'dexmate_status',
               'observe': 'dexmate_observe', 'step': 'dexmate_step', 'plan': 'dexmate_motion_plan', 'pixel': '',
               'hz': 50, 'overview_camera': 'back', 'overview': 'fixed third-person',
               'cameras': ('back', 'ego', 'wrist_left', 'wrist_right'), 'arms': 'dual',
               'wrist_images': '`wrist_left` and `wrist_right`', 'tracking_skill': 'robot-interface'},
        partial_credit=False, graded_score=False, method_skills=False, episode_reviews=False,
        task_tag='custom', score_label='Custom physical completion (locking not validated)',
        layout_note='Existing task scene; seeds reproduce reset, without benchmark layout claims.',
        fixed={'expert_demonstrations': (0, 'No official expert dataset for this custom task.')})
