"""Physical and MCP integration checks, without a model or network inference."""
import copy
import json
from pathlib import Path
import subprocess
import time

import numpy as np
import mujoco
from scipy.spatial.transform import Rotation

from .launch import MCPClient, start_frontend, stop_frontend, value, config_for


def run_checks(config, deployed):
    from services.dexmate_box.backend import DexmateBoxBackend, fingerprints
    from services.benchmarks import for_task
    from harness.claude_cli import auto_research as claude
    from services.simulator.io import RejectedBeforeAction
    checks, evidence = {}, {}
    def check(name, condition, detail=None):
        checks[name] = bool(condition)
        if detail is not None: evidence[name] = detail
        if not condition: raise AssertionError(name)
    before = fingerprints()
    check('custom_task_registered', for_task(config.task).name == 'dexmate_box')
    command = claude.agent_command(config, deployed, prompt='offline command construction', json_output=True)
    check('official_claude_harness_command', '--mcp-config' in command and '--strict-mcp-config' in command and '-p' in command)
    check('claude_sandbox_enabled', json.loads((deployed['claude_home']/'settings.json').read_text())['sandbox']['enabled'])
    backend = DexmateBoxBackend(config)
    started = time.monotonic()
    try:
        packet, images = backend.start(formal=False, seed=11)
        env = backend.env
        qpos0, qvel0, ctrl0 = (getattr(env.data, key).copy() for key in ('qpos', 'qvel', 'ctrl'))
        check('actual_robot_and_dimensions', env.model.nu == 16 and all(env.action_space[s].shape == (8,) for s in ('right','left')),
              {'nq': env.model.nq, 'nv': env.model.nv, 'actuators': env.model.nu, 'action_dim': 16})
        check('four_camera_images', len(images) == 4 and packet['images'] == ['back','ego','wrist_left','wrist_right'])
        check('observe_does_not_step', backend.step_id == 0 and env.data.time == 0)
        check('physics_timing', env.control_dt == .02 and env.model.opt.timestep == env.cfg['physics_dt_s']
              and abs(env._n_substeps*env.model.opt.timestep-env.control_dt)<1e-12,
              {'control_dt_s':env.control_dt,'physics_dt_s':float(env.model.opt.timestep),
               'substeps_per_row':env._n_substeps})
        check('reported_timing_matches_loaded_model', packet['action_layout']['timing']['physics_dt_s']==env.model.opt.timestep
              and packet['action_layout']['timing']['substeps_per_row']==env._n_substeps)
        check('reported_gripper_mode_matches_current_controller',
              packet['action_layout']['gripper']['configured_closing_torque_Nm']==env.cfg.get('gripper_closing_torque_Nm',0))
        frames = {}
        for side, prefix in [('right','R'),('left','L')]:
            site = env.model.site('tcp_'+prefix)
            base = env.model.body('dexmate_'+prefix+'_gripper_base')
            p = env.tcp_pose(side)
            matrix = env.data.site_xmat[site.id].reshape(3,3)
            qmat = Rotation.from_quat(p[[4,5,6,3]]).as_matrix()
            reconstructed = env.data.xpos[base.id] + env.data.xmat[base.id].reshape(3,3)@np.array([0,0,.16])
            check(side+'_tcp_frame_and_wxyz', np.allclose(p[:3], reconstructed, atol=1e-10) and np.allclose(qmat, matrix, atol=1e-10))
            check(side+'_tool_axes', np.allclose(env.data.xmat[base.id], env.data.site_xmat[site.id], atol=1e-10))
            frames[side] = p.tolist()
        evidence['reset_tcp_poses_wxyz'] = frames
        row = backend._hold_row()
        bads = [row[:-1], row[:3]+[0]*4+row[7:], row[:7]+[1.1]+row[8:], row[:3]+[float('nan')]+row[4:]]
        for i, bad in enumerate(bads):
            try: backend.call('dexmate_step', {'actions': [row, bad]})
            except RejectedBeforeAction: pass
            else: raise AssertionError('bad chunk accepted')
            check('invalid_chunk_atomic_'+str(i), env.data.time == 0 and np.array_equal(env.data.qpos,qpos0) and np.array_equal(env.data.ctrl,ctrl0))
        target = row.copy()
        target[2] += .003
        target[10] += .003
        # Scale quaternions to independently verify scalar-first normalization in both arms.
        target[3:7] = (np.array(target[3:7])*2).tolist()
        target[11:15] = (np.array(target[11:15])*2).tolist()
        result, _ = backend.call('dexmate_step', {'actions': [target]*50})
        deltas = {s: (env.tcp_pose(s)[:3]-np.array(frames[s])[:3]).tolist() for s in ('right','left')}
        check('synchronized_bimanual_motion', all(d[2] > .001 for d in deltas.values()), deltas)
        check('metre_target_accuracy', all(np.linalg.norm(env.tcp_pose(s)[:3]-np.array(frames[s])[:3]-[0,0,.003]) < .002 for s in frames))
        check('normalized_orientation_and_time', all(abs(np.linalg.norm(env.targets[s][3:7])-1)<1e-12 for s in frames) and abs(env.data.time-1.)<1e-10)
        rotated = backend._hold_row()
        rotation_goals = {}
        for side,offset in [('right',0),('left',8)]:
            measured = env.tcp_pose(side)
            current = Rotation.from_quat(measured[[4,5,6,3]])
            goal = Rotation.from_rotvec([0,0,.02])*current
            rotation_goals[side] = goal
            rotated[offset:offset+3] = measured[:3].tolist()
            rotated[offset+3:offset+7] = goal.as_quat()[[3,0,1,2]].tolist()
        backend.call('dexmate_step', {'actions':[rotated], 'repeat':50})
        rotation_errors = {s:float((rotation_goals[s]*Rotation.from_quat(env.tcp_pose(s)[[4,5,6,3]]).inv()).magnitude()) for s in frames}
        check('world_active_rotation_and_repeat', all(v<.003 for v in rotation_errors.values()) and abs(env.data.time-2.)<1e-9, rotation_errors)
        openings_before = {s: env.data.qpos[env.grippers[s]['qad']].copy() for s in frames}
        backend.call('dexmate_gripper', {'openings': {'right': .65, 'left': .65}, 'steps':50})
        jaws = {s: env.data.qpos[env.grippers[s]['qad']].tolist() for s in frames}
        check('coupled_gripper_control', all(jaws[s][0] > openings_before[s][0]+.1 and abs(jaws[s][0]-jaws[s][1])<.005 for s in frames), jaws)
        check('gripper_units_and_mapping', all(abs(env.grippers[s]['qcmd']-.96*.65)<1e-10 for s in frames))
        verdict = backend.evaluate()
        check('physical_lock_gate_preserved', verdict['task_complete'] is False and env.progress()['physical_locking_verified'] is False)
        # Reset remains the actual environment reset; no alternate cardboard placement or geometry.
        env.reset(seed=11)
        check('deterministic_reset', all(np.array_equal(a,b) for a,b in zip((qpos0,qvel0,ctrl0),(env.data.qpos,env.data.qvel,env.data.ctrl))))
        # Isolated terminal fixture advances only the episode clock to exercise the existing 90 s horizon.
        # This is not a task attempt or success evidence.
        env.data.time = env.cfg['episode_seconds']-.02
        backend.call('dexmate_hold', {'steps':2})
        check('native_horizon_termination', backend.finished and backend.termination_reason == 'simulation_time_limit')
        frozen = env.data.qpos.copy()
        try: backend.call('dexmate_hold', {'steps':1})
        except RejectedBeforeAction: pass
        else: raise AssertionError('terminal action accepted')
        check('terminal_actions_rejected', np.array_equal(frozen,env.data.qpos))
    finally: backend.close()
    check('scene_geometry_physics_unchanged', before == fingerprints())
    probe = subprocess.run(['ffprobe','-v','error','-select_streams','v:0','-show_entries','stream=codec_name,width,height,nb_frames','-of','json',str(backend.output/'episode.mp4')],capture_output=True,text=True,check=True)
    check('episode_video_decodable', bool(json.loads(probe.stdout)['streams']), json.loads(probe.stdout))

    # Real Unix-socket frontend + official MCP bridge, no model subprocess.
    server, path, log = start_frontend(config, deployed)
    client = MCPClient(path)
    try:
        check('mcp_initialize', client.request('initialize')['serverInfo']['name'] == 'auto-research')
        names = [t['name'] for t in client.request('tools/list')['tools']]
        check('mcp_custom_tool_discovery', all(n in names for n in ['dexmate_step','dexmate_observe','dexmate_gripper','start_episode','evaluate','finish']))
        # Exercise the shipped stdio bridge against the actual frontend socket.
        from harness import agent_host
        bridge = subprocess.run([str(agent_host.agent_python()),'-I',str(agent_host.BRIDGE),str(path)],
                    input=json.dumps({'jsonrpc':'2.0','id':1,'method':'tools/list'})+'\n',capture_output=True,text=True,timeout=20,check=True)
        check('official_stdio_mcp_bridge', 'dexmate_step' in bridge.stdout)
        obs = value(client.call('start_episode'))
        tcp = obs['state']['tcp_pose']
        row = tcp[:7]+[.3]+tcp[7:14]+[.3]
        row[2] += .002
        row[10] += .002
        response = client.call('dexmate_step', {'actions':[row]*25})
        updated = value(response)
        check('mcp_observe_action_updated_image_loop', updated['step_id'] == 25 and len([b for b in response['content'] if b['type']=='image']) == 4)
        check('mcp_robot_only_observation', 'box_pose' not in updated['state'] and 'hinge_angles' not in updated['state'])
        evaluation = value(client.call('evaluate'))
        check('mcp_custom_evaluation', evaluation['evaluation']['task_complete'] is False)
        value(client.call('finish'))
        check('mcp_episode_closed', value(client.call('status'))['active_episode'] is None)
    finally:
        client.close()
        stop_frontend(server,path,log)
    check('all_scene_sources_still_unchanged', fingerprints() == before)
    from .relay_checks import run as check_relay
    settings = json.loads((Path(__file__).parent/'smoke_config.json').read_text())
    guard_checks = check_relay(config.root/'relay_checks',settings)
    for name,passed in guard_checks.items(): check('relay_'+name,passed)
    report = {'all_passed': all(checks.values()), 'checks':checks, 'evidence':evidence,
              'inference_requests':0, 'model_cost_usd':0, 'wall_seconds':time.monotonic()-started,
              'custom_task_not_official_dexjoco':True, 'complete_folding_with_physical_locking':False,
              'terminal_test_uses_clock_fixture':True,
              'physical_environment_defects':'See ../../ENVIRONMENT_DEFECTS.md; no physical completion claimed'}
    (config.root/'validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({'all_passed':report['all_passed'],'checks_passed':len(checks),
                      'inference_requests':0,'report':str(config.root/'validation.json')},indent=2))
