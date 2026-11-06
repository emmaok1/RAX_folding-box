"""Bounded normal-Claude/RAX teacher launcher. Inspect is default; no paid grant exists."""
from __future__ import annotations
import argparse,fcntl,json,math,os,secrets,signal,subprocess,sys,tempfile,time
from pathlib import Path
from .prepare import HERE,PROJECT,load_prepared,checkpoint_session,configure_file_tools
from .bundles import persist
from ..claude_relay import BoundedClaudeRelay,usage_accounting

def command_for(config,deployed,limits):
    """Keep normal sandboxed Claude file/code tools and RAX register/rehearse."""
    from harness.claude_cli import auto_research as claude
    prompt=('Read the compact required startup set once: TEACHER.md, TASK.md, ROBOT.md, TEACHER_AUTHORIZATION.json and memory/RESTORE.json. Prioritize the execution-first schedule in TEACHER.md: restore structured memory once, then your first experiment is one accepted nearby unladen move from measured state, not writing a complete folding controller. On one bounded correction failing, record the exact refusal. '
            'If TEACHER_START.json exists, the operator already opened episode1 for live visualization: '
            'read it, call status/current teacher state and REUSE that active episode; do not reset it to initialize. '
            'Use the normal RAX auto-research workflow: understand measured state, perform '
            'bounded experiments within the operator episode schedule, write observation-driven '
            'code with Read/Write/Edit tools; native Bash is unavailable. Execute Python ONLY via RAX register/rehearse exact frozen controllers, and maintain evidence-linked memory '
            'and handoff. Do not start a new episode merely after recovery. Do not train models, '
            'change physics/controllers/evaluator, or start formal evaluation. A clean rehearsal '
            'is not physical success. Full locking remains unverified under the unchanged guard. '
            'This invocation has passed fresh operator authorization; TEACHER_LIMITS records preparation defaults, while TEACHER_AUTHORIZATION records this admitted invocation. Cloud/API/STOP limits are final. No alternate provider or background model client.')
    command=claude.agent_command(config,deployed,prompt=prompt,json_output=True)
    # Append to, rather than replace, upstream security/tool denials.
    if '--disallowedTools' in command:
        start=command.index('--disallowedTools')+1;end=start
        while end<len(command) and not command[end].startswith('--'):end+=1
        command[end:end]=['mcp__auto_research__submit','Bash','Monitor']
    separator=command.index('--')
    return command[:separator]+['--restricted','--tools','Read,Write,Edit','--add-dir',str(config.root/'teacher_world'),str(config.root/'published'),'--settings',str(deployed['claude_home']/'settings.json'),'--max-turns',str(limits['max_requests']),'--max-budget-usd',str(limits['max_cost_usd'])]+command[separator:]

def effective_wall_budget(limits, auth, *, now=None):
    """A continuation cannot reset or outlive its original task deadline."""
    seconds = float(limits['max_wall_seconds'])
    deadline = auth.get('original_deadline_unix_s')
    if deadline is not None:
        if isinstance(deadline, bool) or not isinstance(deadline, (int, float)) or not math.isfinite(deadline):
            raise PermissionError('Invalid original task deadline')
        seconds = min(seconds, deadline - (time.time() if now is None else now))
    if not math.isfinite(seconds) or seconds <= 0:
        raise PermissionError('Task wall deadline has expired')
    return seconds

def paid_preflight(root,approval):
    """Read-only fail-closed gate, before secrets, processes, API or simulator imports."""
    root=Path(root).resolve();config,deployed=load_prepared(root)
    policy=json.loads((root/'teacher_policy.json').read_text());limits=policy['api_limits']
    auth=json.loads((root/'cloud_authorization.json').read_text())
    scope=json.loads((HERE/'CURRENT_SCOPE.json').read_text())
    if not scope.get('cloud_inference_allowed') or scope.get('attempt_consumed') or scope.get('active_run_root'):
        raise PermissionError('No fresh unconsumed global cloud authorization')
    if auth.get('allowed') is not True or auth.get('consumed') or not auth.get('task_id') or not auth.get('user_authorization'):
        raise PermissionError('No fresh per-session user authorization')
    if scope.get('authorization_task_id')!=auth['task_id']:raise PermissionError('Global/session authorization task mismatch')
    if limits['max_cost_usd']<=0 or approval is None or not math.isfinite(approval) or approval<limits['max_cost_usd'] or auth.get('max_cost_usd',0)<limits['max_cost_usd']:
        raise PermissionError('Explicit approved budget must cover the prepared ceiling')
    if scope.get('approved_budget_usd',0)<limits['max_cost_usd']:raise PermissionError('Global approved ceiling is smaller than session ceiling')
    for key,required in [('max_requests',limits['max_requests']),('max_wall_seconds',limits['max_wall_seconds']),('max_episode_starts',len(config.exploration_seeds))]:
        if auth.get(key,0)<required:raise PermissionError('Explicit user grant does not cover prepared '+key)
    if (root/'motion_planning/STOP').exists() or (root/'STOP').exists():raise PermissionError('Operator STOP is active')
    effective_wall_budget(limits, auth)
    return config,deployed,limits,auth

def prepare_live_episode(root, socket, workspace):
    from ..launch import MCPClient, value
    client=MCPClient(socket)
    try:
        packet=value(client.call('start_episode'))
    finally:
        client.close()
    persist(Path(workspace)/'TEACHER_START.json', {
        'operator_prestarted':True,'episode_id':packet['episode_id'],'step_id':packet['step_id'],
        'initial_teacher_state':packet.get('teacher_state'),
        'note':'Initial historical snapshot. Reuse this active episode; refresh live state before actions. No robot action was sent by the operator.'})


def execute(root,*,approval,key_file,live=False):
    """Exactly one agent invocation; no auto-retry, continuation or formal batch."""
    root=Path(root).resolve()
    with (HERE/'cloud_attempt.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        config,deployed,limits,auth=paid_preflight(root,approval)
        limits = {**limits, 'max_wall_seconds':effective_wall_budget(limits, auth)}
        if live and not os.environ.get('DISPLAY'):raise RuntimeError('Native viewer requested but DISPLAY is unset')
        from harness import agent_host
        from harness.claude_cli import auto_research as claude
        from harness.claude_cli.login import private_text
        from ..launch import start_frontend,stop_frontend
        from ..motion_planning.operator_stop import request_stop
        path=Path(key_file).expanduser().resolve()
        if path.is_relative_to(PROJECT):raise ValueError('Credential must remain outside the project')
        secret=private_text(path).strip()
        if not secret or any(c.isspace() for c in secret):raise ValueError('Invalid private credential')
        # All gates have passed; reserve this grant before starting any transport.
        auth.update(consumed=True,started_unix_s=time.time());persist(root/'cloud_authorization.json',auth)
        scope=json.loads((HERE/'CURRENT_SCOPE.json').read_text());scope['active_run_root']=str(root);persist(HERE/'CURRENT_SCOPE.json',scope)
        capability = ''
        name = None
        relay = None
        agent=server=viewer=None;socket=frontend_log=None;stdout=stderr='';reason='agent_exit';started=time.monotonic();signal_stop=False
        old = {}
        def stop(signum,frame):
            nonlocal signal_stop
            signal_stop=True;request_stop(root,'operator signal '+str(signum))
        try:
            capability=secrets.token_urlsafe(32);fd,name=tempfile.mkstemp(prefix='rax-teacher-capability-');os.fchmod(fd,0o600)
            with os.fdopen(fd,'w') as stream:stream.write(capability)
            relay=BoundedClaudeRelay(limits,secret,capability,root/'requests')
            old={sig:signal.signal(sig,stop) for sig in (signal.SIGINT,signal.SIGTERM)}
            relay.start()
            persist(deployed['workspace']/'TEACHER_AUTHORIZATION.json', {
                'task_id':auth['task_id'], 'invocation_authorized':True, 'limits':limits,
                'scope':'Bounded teacher development, no automatic formal evaluation',
                'note':'Informational copy; host gates and ledgers are authoritative.'})
            persist(deployed['login'],{'agent_cli':'claude','provider':'custom','model':limits['model'],
                'base_url':relay.url,'key_file':name,'account':None})
            server,socket,frontend_log=start_frontend(config,deployed)
            claude.write_home(config,deployed,socket)
            configure_file_tools(config,deployed)
            if live:
                prepare_live_episode(root,socket,deployed['workspace'])
                display=os.environ.get('DISPLAY')
                if not display:raise RuntimeError('Native viewer requested but DISPLAY is unset')
                with (root/'viewer.log').open('wb') as output:
                    viewer=subprocess.Popen([sys.executable,'-m','rax_integration.live_view',str(root)],cwd=PROJECT,
                        env={**os.environ,'MUJOCO_GL':'glfw'},stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
                (root/'viewer.pid').write_text(str(viewer.pid))
            command=command_for(config,deployed,limits);persist(root/'teacher_launch_command.json',command)
            environment=claude.agent_environment(config,deployed,socket)
            environment.update(CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC='1',DISABLE_TELEMETRY='1',
                ENABLE_TOOL_SEARCH='false',CLAUDE_CODE_MAX_OUTPUT_TOKENS=str(limits['max_output_tokens_per_request']),
                ANTHROPIC_DEFAULT_OPUS_MODEL=limits['model'],
                ANTHROPIC_DEFAULT_SONNET_MODEL=limits['model'],ANTHROPIC_DEFAULT_HAIKU_MODEL=limits['model'])
            environment.pop('MAX_THINKING_TOKENS', None)  # Keep native adaptive reasoning under relay output caps.
            local=HERE/'tools/usr/bin'
            if local.is_dir():environment['PATH']=str(local)+':'+environment.get('PATH','')
            agent=agent_host.start(command,cwd=deployed['workspace'],env=environment,stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
            agent_host.record(agent,root/'agent.pid')
            print(json.dumps({'teacher_started':True,'root':str(root),'pid':agent.pid,'task_id':auth['task_id'],'max_cost_usd':limits['max_cost_usd']}),flush=True)
            while True:
                try:stdout,stderr=agent.communicate(timeout=.25);break
                except subprocess.TimeoutExpired:
                    if signal_stop or (root/'STOP').exists() or (root/'motion_planning/STOP').exists():reason='operator_stop'
                    elif viewer is not None and viewer.poll() is not None:reason='viewer_exited'
                    elif time.monotonic()-started>=limits['max_wall_seconds']:reason='wall_time_limit'
                    else:continue
                    request_stop(root,reason);agent_host.stop(agent);stdout,stderr=agent.communicate(timeout=10);break
        except Exception as exc:
            reason='launcher_error:'+type(exc).__name__
            persist(root/'teacher_launcher_error.json',{'error_type':type(exc).__name__})
            raise
        finally:
            cleanup_errors=[]
            def cleanup(label,fn):
                try:return fn()
                except Exception as exc:cleanup_errors.append({'operation':label,'error_type':type(exc).__name__})
            def close_grant():
                scope=json.loads((HERE/'CURRENT_SCOPE.json').read_text());scope.pop('active_run_root',None)
                scope.update(cloud_inference_allowed=False,attempt_consumed=True,completed_run_root=str(root))
                persist(HERE/'CURRENT_SCOPE.json',scope)
            # Close permission before fallible process/log/checkpoint cleanup.
            cleanup('close_cloud_grant', close_grant)
            cleanup('stop_flag',lambda:request_stop(root,'teacher session ended: '+reason))
            if agent is not None:
                cleanup('agent_stop',lambda:agent_host.stop(agent))
                if not stdout:
                    try:stdout,stderr=agent.communicate(timeout=5)
                    except (OSError,subprocess.TimeoutExpired):pass
            cleanup('agent_pid_unlink', lambda:(root/'agent.pid').unlink(missing_ok=True))
            if server is not None:cleanup('frontend_stop',lambda:stop_frontend(server,socket,frontend_log))
            if viewer is not None and viewer.poll() is None:
                cleanup('viewer_terminate',viewer.terminate)
                try:viewer.wait(timeout=5)
                except subprocess.TimeoutExpired:cleanup('viewer_kill',viewer.kill);cleanup('viewer_wait',lambda:viewer.wait(timeout=5))
            if relay is not None:cleanup('relay_close',relay.close)
            if name is not None:cleanup('capability_unlink',lambda:Path(name).unlink(missing_ok=True))
            for key,text in [('claude-stream.jsonl',stdout),('claude-stderr.log',stderr)]:
                raw=text.encode().replace(secret.encode(),b'<redacted>')
                if capability:raw=raw.replace(capability.encode(),b'<redacted>')
                cleanup(key,lambda key=key,raw=raw:(root/key).write_bytes(raw))
            ledger=relay.ledger if relay is not None else {'usage':[],'inference_requests':0}
            usage,cost=usage_accounting(ledger['usage'],limits)
            result={'kind':'normal_RAX_teacher_development','task_id':auth['task_id'],'termination_reason':reason,
                'actual_model_requests':ledger['inference_requests'],'provider_usage_responses':len(ledger['usage']),
                'requests_without_recorded_usage':ledger['inference_requests']-len(ledger['usage']),
                'known_response_estimated_cost_usd':cost,'token_usage':usage,'elapsed_seconds':time.monotonic()-started,
                'cli_returncode':agent.returncode if agent else None,'physical_success_claimed':False,'formal_evaluation_started':False,'cleanup_errors':cleanup_errors}
            cleanup('result_write',lambda:persist(root/'teacher_result.json',result))
            try:checkpoint_session(root)
            except Exception as exc:persist(root/'teacher_checkpoint_error.json',{'error_type':type(exc).__name__})
            for sig,handler in old.items():cleanup('signal_restore',lambda sig=sig,handler=handler:signal.signal(sig,handler))
        return result

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--execute',action='store_true');parser.add_argument('--approve-budget-usd',type=float)
    parser.add_argument('--key-file',type=Path);parser.add_argument('--live',action='store_true')
    args=parser.parse_args()
    if not args.execute:
        config,deployed=load_prepared(args.root);policy=json.loads((config.root/'teacher_policy.json').read_text())
        print(json.dumps({'prepared':str(config.root),'execution_started':False,'teacher_privileged':config.teacher_privileged,
            'cloud_authorization':json.loads((config.root/'cloud_authorization.json').read_text()),'limits':policy['api_limits']}));return
    if args.key_file is None:parser.error('--execute requires operator private --key-file')
    execute(args.root,approval=args.approve_budget_usd,key_file=args.key_file,live=args.live)

if __name__=='__main__':main()
