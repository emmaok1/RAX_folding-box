"""One read-only Claude continuation after an executed smoke hit its request cap.

Reuses the original RAX workspace/conversation. Its episode budget is already
spent and closed, so no additional simulator episode or action can execute.
Original capped results remain immutable; this phase has its own ledger.
"""
import argparse
import json
import os
from pathlib import Path
import secrets
import subprocess
import tempfile

from .launch import HERE, config_for, start_frontend, stop_frontend, require_cloud_scope
from .claude_relay import BoundedClaudeRelay, usage_accounting


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--key-file',type=Path,required=True)
    args=p.parse_args()
    require_cloud_scope()
    root=args.run.resolve()
    if not root.is_relative_to(HERE/'runs'): raise ValueError('Run must be task-local')
    state=json.loads((root/'state.json').read_text())
    if state.get('active') or state['exploration_started']!=1: raise ValueError('Only an already closed single smoke can continue')
    phase=root/'read_only_continuation'
    phase.mkdir(mode=0o700)
    settings=json.loads((root/'budget-review.json').read_text())
    settings.update(max_requests=1,max_token_count_requests=1,max_input_tokens_per_request=24000,
        max_input_tokens_total=24000,max_output_tokens_per_request=320,max_output_tokens_total=320,
        max_cost_usd=.10,max_wall_seconds=40,authorization='user_authorized_read_only_completion_of_executed_smoke')
    from harness import agent_host
    from harness.claude_cli import auto_research as claude
    from harness.claude_cli.login import private_text
    config=config_for(root)
    deployed=claude.redeploy(config)
    deployed['model']=settings['model']
    credential=private_text(args.key_file.expanduser().resolve()).strip()
    capability=secrets.token_urlsafe(32)
    relay=BoundedClaudeRelay(settings,credential,capability,phase/'requests')
    fd,path=tempfile.mkstemp(prefix='rax-continuation-capability-')
    with os.fdopen(fd,'w') as f:f.write(capability)
    (phase/'prior-agent-login.json').write_bytes(deployed['login'].read_bytes())
    deployed['login'].write_text(json.dumps({'agent_cli':'claude','provider':'custom','model':settings['model'],
        'base_url':relay.url,'key_file':path,'account':None}))
    relay.start()
    server=agent=None
    try:
        server,socket,log=start_frontend(config,deployed)
        prompt=('The smoke action already executed and the episode is closed. Inspect the updated step-25 '
                'images and measured TCP state already returned in this conversation. Do not call any tools '
                'or request further motion or reset. In at most 50 words report the observed TCP change and '
                'that folding and physical locking remain incomplete. This is the final read-only continuation.')
        command=claude.agent_command(config,deployed,prompt=prompt,json_output=True)
        command[command.index('--effort')+1]='low'
        command=command[:-2]+['--tools','','--max-turns','1','--max-budget-usd','0.10','--',prompt]
        environment=claude.agent_environment(config,deployed,socket)
        environment['PATH']=str(HERE/'tools/usr/bin')+':'+environment['PATH']
        environment.update(CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC='1',DISABLE_TELEMETRY='1',ENABLE_TOOL_SEARCH='false',
            CLAUDE_CODE_MAX_OUTPUT_TOKENS='320',MAX_THINKING_TOKENS='0',
            ANTHROPIC_DEFAULT_OPUS_MODEL=settings['model'],ANTHROPIC_DEFAULT_SONNET_MODEL=settings['model'],
            ANTHROPIC_DEFAULT_HAIKU_MODEL=settings['model'])
        agent=agent_host.start(command,cwd=deployed['workspace'],env=environment,stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        agent_host.record(agent,root/'agent.pid')
        try:stdout,stderr=agent.communicate(timeout=40)
        except subprocess.TimeoutExpired:
            agent_host.stop(agent);stdout,stderr=agent.communicate(timeout=10)
        (phase/'claude-stream.jsonl').write_bytes(relay.safe_bytes(stdout.encode()))
        (phase/'claude-stderr.log').write_bytes(relay.safe_bytes(stderr.encode()))
        (phase/'launch_command.json').write_text(json.dumps(command,indent=2))
    finally:
        if agent:agent_host.stop(agent)
        (root/'agent.pid').unlink(missing_ok=True)
        if server:stop_frontend(server,socket,log)
        relay.close();Path(path).unlink(missing_ok=True)
    request=json.loads(next((phase/'requests').glob('*.request.json')).read_text())
    seen=[]
    for message in request.get('messages',[]):
        for block in message.get('content',[]) if isinstance(message.get('content'),list) else []:
            if block.get('type')!='tool_result':continue
            content=block.get('content',[])
            if not isinstance(content,list):continue
            images=sum(b.get('type')=='image' for b in content if isinstance(b,dict))
            for item in content:
                if item.get('type')!='text':continue
                try:value=json.loads(item.get('text',''))
                except ValueError:continue
                if isinstance(value,dict) and value.get('step_id')==25 and 'state' in value:
                    seen.append({'step_id':25,'images':images})
    events=[json.loads(line) for line in stdout.splitlines() if line.strip()]
    final=next((e for e in reversed(events) if e.get('type')=='result'),{})
    totals,phase_cost=usage_accounting(relay.ledger['usage'],settings)
    result={'updated_images_sent_to_claude':any(s['images'] for s in seen),'observations':seen,
        'inference_requests':relay.ledger['inference_requests'],'estimated_api_cost_usd':phase_cost,
        'cli_cumulative_estimated_cost_usd':final.get('total_cost_usd'),
        'token_usage':totals,'cli_cumulative_usage':final.get('usage'),
        'cli_returncode':agent.returncode,'api_budget':relay.ledger,
        'additional_robot_actions':0,'additional_episodes':0,'model_report':final.get('result'),
        'original_phase_end_reason':'Model request limit','completion_kind':'one read-only continuation'}
    (phase/'result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps({k:result[k] for k in ['updated_images_sent_to_claude','inference_requests','estimated_api_cost_usd','cli_returncode','additional_robot_actions']},indent=2))
    if not result['updated_images_sent_to_claude'] or agent.returncode:raise RuntimeError('Read-only continuation incomplete')


if __name__=='__main__':main()
