"""Reuse official RAX Claude Code command, sandbox, socket frontend, and recorder."""
from dataclasses import asdict
import json
import os
from pathlib import Path
import secrets
import subprocess
import tempfile
import time

from .launch import MCPClient, start_frontend, stop_frontend, value, PROJECT, require_cloud_scope
from .claude_relay import BoundedClaudeRelay, usage_accounting


def run_smoke(config, deployed, settings, key_file, *, transport=None, offline=False):
    if not offline: require_cloud_scope()
    from harness import agent_host
    from harness.claude_cli import auto_research as claude
    from harness.claude_cli.login import private_text
    key_file = key_file.expanduser().resolve()
    if key_file.is_relative_to(PROJECT): raise ValueError('Keep API credentials outside the project')
    secret = private_text(key_file).strip()
    if not secret or any(c.isspace() for c in secret): raise ValueError('Invalid API credential')
    capability = secrets.token_urlsafe(32)
    relay = BoundedClaudeRelay(settings,secret,capability,config.root/'requests',transport=transport)
    relay.start()
    server = agent = client = None
    path = log = None
    started = time.monotonic()
    termination = 'cli_exit'
    # The real credential never enters Claude Code's environment, home, workspace, or transcript.
    fd, ephemeral = tempfile.mkstemp(prefix='rax-smoke-capability-')
    os.fchmod(fd,0o600)
    with os.fdopen(fd,'w') as f: f.write(capability)
    try:
        deployed['login'].write_text(json.dumps({'agent_cli':'claude','provider':'custom',
            'model':settings['model'],'base_url':relay.url,'key_file':ephemeral,'account':None}))
        (config.root/'smoke_policy.json').write_text(json.dumps(settings,indent=2))
        (config.root/'budget-review.json').write_text(json.dumps({**settings,
                    'authorization':'offline_scripted_fixture' if offline else 'explicit_command_line_budget'},indent=2))
        server,path,log = start_frontend(config,deployed)
        prompt = ('Call tools directly without pre-action narration or calculation text. Keep your final report under 60 words. '
                  'Run exactly one smoke loop. Call start_episode once and inspect its back image and measured TCP poses. '
                  'Then call dexmate_step exactly once with actions containing one 16-D row and repeat=25. Right pose xyz with z increased by '
                  '0.002 metres and its original wxyz quaternion, opening 0.3, then left pose with the same +0.002 m '
                  'world z displacement, its own original wxyz quaternion, opening 0.3. Inspect the updated images '
                  'in the result. State what changed and that this demonstrates integration only, with folding '
                  'and physical locking incomplete. Stop. The operator will close the episode. No other tools.')
        command = claude.agent_command(config,deployed,prompt=prompt,json_output=True)
        command[command.index('--effort')+1] = 'low'
        command = command[:-2] + ['--tools','', '--allowedTools',
            'mcp__auto_research__start_episode','mcp__auto_research__dexmate_step',
            '--max-turns',str(settings['max_requests']), '--max-budget-usd',str(settings['max_cost_usd']),
            '--system-prompt','Use only the authorized robot MCP tools for the single bounded custom-task smoke loop. No files, shell, training, or evaluation batches.',
            '--',prompt]
        (config.root/'launch_command.json').write_text(json.dumps(command,indent=2))
        environment = claude.agent_environment(config,deployed,path)
        local_tools = PROJECT/'rax_integration/tools/usr/bin'
        if local_tools.is_dir(): environment['PATH'] = str(local_tools)+':'+environment['PATH']
        environment.update({'CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC':'1','DISABLE_TELEMETRY':'1',
            'ENABLE_TOOL_SEARCH':'false',
            'CLAUDE_CODE_MAX_OUTPUT_TOKENS':str(settings['max_output_tokens_per_request']),
            'MAX_THINKING_TOKENS':'0','ANTHROPIC_DEFAULT_OPUS_MODEL':settings['model'],
            'ANTHROPIC_DEFAULT_SONNET_MODEL':settings['model'],'ANTHROPIC_DEFAULT_HAIKU_MODEL':settings['model']})
        # Inherited third-party provider URLs/tokens never enter this clean native-harness environment.
        agent = agent_host.start(command,cwd=deployed['workspace'],env=environment,
                    stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        agent_host.record(agent,config.root/'agent.pid')
        try: stdout,stderr = agent.communicate(timeout=settings['max_wall_seconds'])
        except subprocess.TimeoutExpired:
            termination = 'wall_time_limit'
            agent_host.stop(agent)
            stdout,stderr = agent.communicate(timeout=10)
        for name,content in [('claude-stream.jsonl',stdout),('claude-stderr.log',stderr)]:
            (config.root/name).write_bytes(relay.safe_bytes(content.encode()))
    finally:
        if agent is not None: agent_host.stop(agent)
        (config.root/'agent.pid').unlink(missing_ok=True)
        if server is not None:
            client = MCPClient(path)
            try: client.call('finish')
            finally:
                client.close()
                stop_frontend(server,path,log)
        relay.close()
        Path(ephemeral).unlink(missing_ok=True)
    requests = []
    def observed_packets(node):
        found = []
        if isinstance(node,dict):
            if node.get('type') == 'tool_result':
                content = node.get('content',[])
                content = content if isinstance(content,list) else [{'type':'text','text':content}]
                image_count = sum(b.get('type')=='image' for b in content if isinstance(b,dict))
                for block in content:
                    if isinstance(block,dict) and block.get('type')=='text':
                        try: packet = json.loads(block.get('text',''))
                        except ValueError: continue
                        if isinstance(packet,dict) and 'state' in packet and 'step_id' in packet:
                            found.append({'step_id':packet['step_id'],'images':image_count})
            for child in node.values(): found.extend(observed_packets(child))
        elif isinstance(node,list):
            for child in node: found.extend(observed_packets(child))
        return found
    for file in sorted((config.root/'requests').glob('*.request.json')):
        request = json.loads(file.read_text())
        text = json.dumps(request)
        requests.append({'file':file.name,'contains_image':'"type": "image"' in text,
                         'contains_tool_result':'"type": "tool_result"' in text,
                         'observations_sent_to_api':observed_packets(request.get('messages',[]))})
    episodes = [json.loads(p.read_text()) for p in (config.root/'native/episodes').glob('*/result.json')]
    seen = [obs for request in requests for obs in request['observations_sent_to_api']]
    loop_complete = (any(e['step_id']>0 for e in episodes) and any(p['step_id']==0 and p['images'] for p in seen)
                     and any(p['step_id']>0 and p['images'] for p in seen))
    if termination != 'wall_time_limit':
        if relay.ledger['rejections']: termination = 'api_guard: '+relay.ledger['rejections'][-1]
        elif agent is not None and agent.returncode: termination = 'cli_error'
    cli_result = {}
    for line in (config.root/'claude-stream.jsonl').read_text().splitlines():
        try: event = json.loads(line)
        except ValueError: continue
        if event.get('type') == 'result': cli_result = event
    token_usage,phase_cost = usage_accounting(relay.ledger['usage'],settings)
    report = {'loop_complete':loop_complete,'inference_sent':not offline and relay.ledger['inference_requests']>0,
        'offline_scripted_api_fixture':offline,'model':settings['model'],'endpoint':settings['endpoint'],
        'actual_model_inference_requests':0 if offline else relay.ledger['inference_requests'],
        'actual_api_cost_usd':0 if offline else None,
        'estimated_api_cost_usd':0 if offline else phase_cost,
        'cli_estimated_cost_usd':cli_result.get('total_cost_usd'),
        'cost_estimate_source':'Provider-reported usage and configured list/cache rates; final invoice not queried',
        'token_usage':token_usage,
        'scripted_wire_requests':relay.ledger['inference_requests'] if offline else 0,
        'termination_reason':termination,'cli_returncode':agent.returncode if agent else None,
        'model_request_evidence':requests,'api_budget':relay.ledger,'episodes':episodes,
        'elapsed_seconds':time.monotonic()-started,
        'integration_success':loop_complete,'complete_folding_with_physical_locking':False,
        'task_kind':'custom task integrated with RAX, not an official DexJoCo benchmark'}
    (config.root/'smoke_result.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({'loop_complete':loop_complete,'inference_sent':report['inference_sent'],
                       'report':str(config.root/'smoke_result.json')},indent=2))
    if not loop_complete: raise RuntimeError('Smoke loop incomplete; inspect smoke_result.json and bounded request records')
    return report
