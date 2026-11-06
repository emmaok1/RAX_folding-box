"""Launch one real model-controlled attempt through the shipped RAX Claude harness.
No fold order, grasp search, or object actuation lives in this launcher.
"""
import argparse,json,os,secrets,signal,subprocess,sys,tempfile,time,fcntl
from pathlib import Path
from .launch import HERE,PROJECT,new_root,prepare,start_frontend,stop_frontend,MCPClient,require_cloud_scope
from .claude_relay import BoundedClaudeRelay,usage_accounting

TOOLS=['start_episode','dexmate_status','dexmate_observe','dexmate_step','dexmate_hold','dexmate_gripper','evaluate','finish']

def prepare_attempt(settings,live,offline=False,prepare_only=False):
    root=new_root('fold');(root/'fold_policy.json').write_text(json.dumps({**settings,'live_view':live,'offline_fixture':offline},indent=2));config,deployed=prepare(root,settings['model'])
    # The benchmark's task-timeout hook and launcher share the reviewed wall cap.
    from dataclasses import replace,asdict
    config=replace(config,development=replace(config.development,wall_seconds=settings['max_wall_seconds']),formal=replace(config.formal,wall_seconds=settings['max_wall_seconds']))
    deployed['configuration'].write_text(json.dumps(asdict(config),default=str,indent=2))
    (root/'budget-review.json').write_text(json.dumps(settings,indent=2))
    import hashlib
    names=['fold.py','live_view.py','launch.py','claude_relay.py','fold_config.json','TASK.md','ROBOT.md','FOLD_AGENT.md',
           'upstream/services/dexmate_box/backend.py','upstream/services/dexmate_box/tools.py','upstream/services/dexmate_box/benchmark.py']
    sources={}
    for name in names:
        raw=(HERE/name).read_bytes();target=root/'integration_source'/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(raw);sources[name]=hashlib.sha256(raw).hexdigest()
    mode='preparation_only' if prepare_only else 'offline_scripted_fixture' if offline else 'real_cloud_attempt'
    (root/'run_manifest.json').write_text(json.dumps({'run_kind':mode,'agent_launched':False,'live_view_requested':live,
        'core_scene_sha256':hashlib.sha256((PROJECT/'models/scene.xml').read_bytes()).hexdigest(),'integration_source_hashes':sources},indent=2))
    return root,config,deployed

class TwoActionFixture:
    """Offline plumbing only. Never counted as model-driven manipulation."""
    def __init__(self):
        from .offline_fixture import ScriptedTransport
        self.inner=ScriptedTransport();self.calls=0
    def __call__(self,path,body,headers):
        if path.endswith('/count_tokens'):return self.inner(path,body,headers)
        import io
        self.calls+=1;self.inner.stage=[0,1,1,2][min(self.calls-1,3)];status,kind,response=self.inner(path,body,headers);raw=response.read().replace(b'fixture_tool_2',f'fixture_tool_{self.calls}'.encode());return status,kind,io.BytesIO(raw)

def run(root,config,deployed,settings,key_file,live,offline):
    from harness import agent_host
    from harness.claude_cli import auto_research as claude
    if not offline:require_cloud_scope()
    if offline:secret='offline-fixture-not-a-real-key'
    else:
        from harness.claude_cli.login import private_text
        path=key_file.expanduser().resolve()
        if path.is_relative_to(PROJECT):raise ValueError('Credentials must stay outside the project')
        secret=private_text(path).strip()
        if not secret or any(c.isspace() for c in secret):raise ValueError('Invalid API credential')
    capability=secrets.token_urlsafe(32);relay=BoundedClaudeRelay(settings,secret,capability,root/'requests',transport=TwoActionFixture() if offline else None)
    fd,token_path=tempfile.mkstemp(prefix='rax-fold-capability-');os.fchmod(fd,0o600)
    with os.fdopen(fd,'w') as f:f.write(capability)
    server=agent=None;socket=log=None;started=time.monotonic();reason='agent_exit';stdout=stderr=''
    relay.start()
    try:
        deployed['login'].write_text(json.dumps({'agent_cli':'claude','provider':'custom','model':settings['model'],'base_url':relay.url,'key_file':token_path,'account':None}))
        server,socket,log=start_frontend(config,deployed)
        if live:
            env=os.environ.copy();env['MUJOCO_GL']='glfw'
            viewlog=(root/'viewer.log').open('wb')
            viewer=subprocess.Popen([sys.executable,'-m','rax_integration.live_view',str(root)],cwd=PROJECT,env=env,stdout=viewlog,stderr=subprocess.STDOUT,start_new_session=True);(root/'viewer.pid').write_text(str(viewer.pid));viewlog.close()
        prompt=('Perform the single user-requested long-horizon box-folding attempt through RAX now. '
                'Use the actual robot MCP tools directly, not a local fixed script. Start exactly one episode, '
                'inspect images/proprioception, choose and execute physical actions, inspect returned observations, '
                'and adapt. There is no predefined folding order or ready-made fold skill. '
                'You may attempt grasping and folding; do not stop after a free-space smoke move. '
                'Observe actual motion and IK residuals; break unreachable targets into reachable waypoints. '
                'Use both hands as needed. Do not repeat ineffective actions blindly. '
                'Do not call files, shell, subagents, training, resets, or a second episode. '
                'A budget rejection ends the attempt. Evaluate and finish when done or physically blocked. '
                'Keep narration concise and never claim locking that was not verified.\n\n'+
                (HERE/'TASK.md').read_text()+'\n\n'+(HERE/'ROBOT.md').read_text())
        command=claude.agent_command(config,deployed,prompt=prompt,json_output=True);command[command.index('--effort')+1]='high'
        command=command[:-2]+['--tools','', '--allowedTools',*['mcp__auto_research__'+t for t in TOOLS],
                 '--max-turns',str(settings['max_requests']),'--max-budget-usd',str(settings['max_cost_usd']),
                 '--system-prompt','Use only the authorized RAX robot MCP tools for one live box-folding attempt. No files, shell, workers, training, exploration campaign, or benchmark submissions.', '--',prompt]
        (root/'launch_command.json').write_text(json.dumps(command,indent=2));environment=claude.agent_environment(config,deployed,socket);local=HERE/'tools/usr/bin'
        if local.is_dir():environment['PATH']=str(local)+':'+environment['PATH']
        environment.update(CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC='1',DISABLE_TELEMETRY='1',ENABLE_TOOL_SEARCH='false',CLAUDE_CODE_MAX_OUTPUT_TOKENS=str(settings['max_output_tokens_per_request']),MAX_THINKING_TOKENS='0',ANTHROPIC_DEFAULT_OPUS_MODEL=settings['model'],ANTHROPIC_DEFAULT_SONNET_MODEL=settings['model'],ANTHROPIC_DEFAULT_HAIKU_MODEL=settings['model'])
        agent=agent_host.start(command,cwd=deployed['workspace'],env=environment,stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True);agent_host.record(agent,root/'agent.pid')
        manifest=json.loads((root/'run_manifest.json').read_text());manifest['agent_launched']=True;(root/'run_manifest.json').write_text(json.dumps(manifest,indent=2))
        print(json.dumps({'started':True,'offline_fixture':offline,'root':str(root),'model':settings['model'],'max_cost_usd':0 if offline else settings['max_cost_usd']}),flush=True)
        try:stdout,stderr=agent.communicate(timeout=settings['max_wall_seconds'])
        except subprocess.TimeoutExpired:reason='wall_time_limit';agent_host.stop(agent);stdout,stderr=agent.communicate(timeout=10)
    except KeyboardInterrupt:reason='operator_stop'
    except Exception:
        reason='launcher_error';raise
    finally:
        if agent is not None:
            agent_host.stop(agent)
            if not stdout:
                try:stdout,stderr=agent.communicate(timeout=5)
                except subprocess.TimeoutExpired:pass
        for name,content in [('claude-stream.jsonl',stdout),('claude-stderr.log',stderr)]:(root/name).write_bytes(relay.safe_bytes(content.encode()))
        (root/'agent.pid').unlink(missing_ok=True)
        if server is not None:
            client=None
            try:client=MCPClient(socket);client.call('finish')
            except Exception as err:(root/'cleanup_error.json').write_text(json.dumps({'kind':type(err).__name__}))
            finally:
                if client:client.close()
                stop_frontend(server,socket,log)
        relay.close();Path(token_path).unlink(missing_ok=True)
        if relay.ledger['rejections'] and reason=='agent_exit':reason='api_guard: '+relay.ledger['rejections'][-1]
        elif agent is not None and agent.returncode and reason=='agent_exit':reason='agent_error'
        episodes=[json.loads(p.read_text()) for p in (root/'native/episodes').glob('*/result.json')];usage,cost=usage_accounting(relay.ledger['usage'],settings)
        result={'offline_scripted_fixture':offline,'inference_sent':not offline and relay.ledger['inference_requests']>0,'model':settings['model'],'actual_model_requests':0 if offline else relay.ledger['inference_requests'],'scripted_fixture_requests':relay.ledger['inference_requests'] if offline else 0,'estimated_api_cost_usd':0 if offline else cost,'token_usage':usage,'termination_reason':reason,'elapsed_seconds':time.monotonic()-started,'episodes':episodes,'limits':settings,'complete_folding_with_physical_locking':any(e['complete_folding_with_physical_locking'] for e in episodes),'planner':('OFFLINE scripted transport through actual Claude Code CLI and RAX MCP; no learned-model decisions' if offline else 'Real cloud-model decisions through Claude Code and RAX MCP; no fixed-fold script')}
        (root/'attempt_result.json').write_text(json.dumps(result,indent=2));print(json.dumps({'finished':True,'result':str(root/'attempt_result.json'),'termination':reason,'inference_requests':result['actual_model_requests'],'estimated_api_cost_usd':result['estimated_api_cost_usd'],'executed_steps':[e['step_id'] for e in episodes]}),flush=True)
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=HERE/'fold_config.json');p.add_argument('--prepare',action='store_true');p.add_argument('--offline-fixture',action='store_true');p.add_argument('--live',action='store_true');p.add_argument('--key-file',type=Path);p.add_argument('--approve-budget-usd',type=float);a=p.parse_args();settings=json.loads(a.config.read_text())
    if not a.prepare and not a.offline_fixture:
        require_cloud_scope()
        if a.approve_budget_usd is None or a.approve_budget_usd<settings['max_cost_usd']:p.error('Explicit budget must cover fold_config.json')
        if a.key_file is None:p.error('Provide existing private --key-file; never put a key in project files')
    real=not a.prepare and not a.offline_fixture
    lock=None
    if real:
        lock=(HERE/'cloud_attempt.lock').open('a')
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        scope=json.loads((HERE/'CURRENT_SCOPE.json').read_text())
        if scope.get('active_run_root') or scope.get('attempt_consumed'):p.error('The single authorized attempt is active or consumed')
    root,config,deployed=prepare_attempt(settings,a.live,a.offline_fixture,a.prepare)
    if real:
        scope['active_run_root']=str(root);(HERE/'CURRENT_SCOPE.json').write_text(json.dumps(scope,indent=2))
    if a.prepare:print(json.dumps({'prepared':str(root),'inference_sent':False,'budget':settings},indent=2));return
    def stop(_sig,_frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop)
    try:run(root,config,deployed,settings,a.key_file,a.live,a.offline_fixture)
    finally:
        if real:
            scope=json.loads((HERE/'CURRENT_SCOPE.json').read_text());scope.update(cloud_inference_allowed=False,attempt_consumed=True,phase='single_cloud_attempt_stopped',completed_run_root=str(root));scope.pop('active_run_root',None);(HERE/'CURRENT_SCOPE.json').write_text(json.dumps(scope,indent=2));lock.close()
if __name__=='__main__':main()
