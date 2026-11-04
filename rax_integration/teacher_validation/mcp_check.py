"""Real RAX frontend + frozen zero-motion teacher rehearsal, no model calls."""
from dataclasses import replace, asdict
import json
from pathlib import Path
import sys

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT))
from rax_integration.launch import (new_root, config_for, start_frontend, stop_frontend,
                                    MCPClient, value)
from rax_integration.teacher.world import create_world_snapshot

CONTROLLER = '''import json
from pathlib import Path
def main(ctx):
    world = json.loads(Path("runtime/world/manifest.json").read_text())
    for filename in ["runtime/world/physics.json", "api/runtime.py"]:
        try:
            Path(filename).write_text("tamper")
        except OSError:
            pass
        else:
            raise AssertionError("protected reference was writable")
    assert not Path("/home/jennyok/.config/rax").exists()
    import socket
    for address in [("1.1.1.1",443),("127.0.0.1",80)]:
        try:
            connection = socket.create_connection(address,timeout=.2)
        except OSError:
            continue
        connection.close()
        raise AssertionError("network escaped sandbox")
    result = ctx.call("dexmate_teacher_state")
    state = json.loads(next(b["text"] for b in result["content"] if b["type"] == "text"))
    assert state["environment_id"] == world["environment_id"]
    assert state["step_id"] == 0
    assert len(state["panels"]) == 14
    mem = ctx.call("dexmate_teacher_memory")
    memory = json.loads(next(b["text"] for b in mem["content"] if b["type"] == "text"))
    assert memory["progress"]["notes"]["next_step"] == "validate-grasp"
    assert not state["physical_progress"]["succeed"]
    Path("output/teacher_contract.json").write_text(json.dumps({"environment_id":world["environment_id"],"step_id":state["step_id"],"panels":len(state["panels"]),"physical_success":False}))
'''


def main():
    root = new_root('teacher_contract')
    (root/'teacher_policy.json').write_text(json.dumps({'role': 'teacher', 'cloud_allowed': False,
        'max_exploration_episodes': 2, 'reason': 'zero-motion interface validation only'}))
    manifest = create_world_snapshot(PROJECT, root/'teacher_world')
    (root/'teacher_memory_identity.json').write_text(json.dumps({'world_environment_id': manifest['environment_id'], 'role': 'local-contract-test'}))
    (root/'motion_policy.json').write_text(json.dumps({'world_policy': 'simulator_snapshot'}))
    config = replace(config_for(root), teacher_privileged=True, write_handoff=True,
                     exploration_seeds=(11, 12), agent_formal=False)
    from harness.codex_cli.auto_research import prepare_workspace
    from harness.claude_cli.auto_research import claude_workspace
    deployed = prepare_workspace(config, 'claude-home')
    claude_workspace(deployed['workspace'])
    controller = deployed['workspace']/'code/contract'
    controller.mkdir(parents=True)
    (controller/'controller.py').write_text(CONTROLLER)
    server = client = path = log = None
    checks = {}
    try:
        server, path, log = start_frontend(config, deployed)
        client = MCPClient(path)
        tools = client.request('tools/list')['tools']
        checks['teacher_tool_discovered'] = 'dexmate_teacher_state' in {t['name'] for t in tools}
        checks['formal_submit_disabled'] = 'submit' not in {t['name'] for t in tools}
        checks['normal_bundle_workflow'] = {'register','rehearse'} <= {t['name'] for t in tools}
        obs = value(client.call('start_episode'))
        checks['observation_contains_live_truth'] = obs['teacher_state']['observation_kind'] == 'live_owned_episode'
        truth = value(client.call('dexmate_teacher_state'))
        saved = value(client.call('dexmate_teacher_memory', {'operation':'update_progress','payload':{'next_step':'validate-grasp','source':'local-contract-test'}}))
        recalled = value(client.call('dexmate_teacher_memory'))
        checks['world_and_memory_identities_consistent'] = truth['memory_environment_id'] == recalled['memory_environment_id'] and truth['world_environment_id'] == recalled['world_environment_id']
        checks['memory_roundtrip'] = recalled['progress']['notes']['next_step'] == 'validate-grasp'
        checks['immutable_evidence'] = (root/'native/episodes'/truth['evidence_id']).is_file()
        checks['same_world'] = truth['environment_id'] == manifest['environment_id']
        checks['zero_steps_before_rehearsal'] = truth['step_id'] == 0
        bundle = value(client.call('register', {'source': 'code/contract'}))
        result = value(client.call('rehearse', {'bundle': bundle['bundle']}))
        (root/'rehearsal_result.json').write_text(json.dumps(result, indent=2))
        # Do not confuse a zero-motion plumbing rehearsal with grasp/folding.
        checks['rehearsal_clean'] = result.get('reason') == 'exit' and result.get('returncode') == 0
        proofs = list(root.glob('published/runs/*/exports/teacher_contract.json'))
        checks['frozen_controller_received_world_and_truth'] = bool(proofs)
        if proofs:
            checks['zero_steps_in_frozen_controller'] = json.loads(proofs[0].read_text())['step_id'] == 0
        checks['no_provider_requests'] = not (root/'requests').exists()
        client.call('finish')
    finally:
        if client: client.close()
        if server: stop_frontend(server, path, log)
        (root/'checks.json').write_text(json.dumps(checks, indent=2))
    print(json.dumps({'root': str(root), 'checks': checks}, indent=2))
    if not checks or not all(checks.values()):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
