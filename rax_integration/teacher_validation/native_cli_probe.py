"""Actual native Claude Bash sandbox, scripted local responses, NO cloud/robot task."""
import io
import json
import os
from pathlib import Path
import secrets
import subprocess
import sys
import tempfile
import time

from rax_integration.teacher.prepare import HERE, PROJECT, prepare_session
from rax_integration.teacher.run import command_for
from rax_integration.claude_relay import BoundedClaudeRelay
from rax_integration.launch import start_frontend, stop_frontend

PROBE = r'''python - <<'PY'
from pathlib import Path
import json, socket, os
workspace=Path.cwd()
assert workspace.name=='agent-workspace'
world=Path('runtime/world')
manifest=json.loads((world/'manifest.json').read_text())
assert len(manifest['files'])>200
try:
    list(Path('/home/jennyok/.config/rax').iterdir())
except OSError:
    pass
else:
    raise AssertionError('operator credential directory readable')
try:
    os.chmod(world/'physics.json',0o644)
except OSError:
    pass
else:
    raise AssertionError('world reference writable')
for address in [('1.1.1.1',443),('127.0.0.1',80)]:
    try:
        c=socket.create_connection(address,timeout=.3)
    except OSError:
        continue
    c.close()
    raise AssertionError('network escaped native sandbox')
Path('output/native_sandbox_proof.json').write_text(json.dumps({
 'kind':'OFFLINE SCRIPTED NATIVE CLI SANDBOX PROBE',
 'world_readable':True,'world_readonly':True,'operator_credentials_unreadable':True,
 'network_blocked':True,'workspace_writable':True,'cloud_requests':0,'robot_steps':0}))
print('OFFLINE NATIVE SANDBOX PROBE PASSED; NO MODEL OR ROBOT TASK')
PY'''


class ScriptedProbe:
    def __init__(self):self.stage=0
    def __call__(self,path,body,headers):
        if path.endswith('count_tokens'):
            return 200,'application/json',io.BytesIO(b'{"input_tokens":10000}')
        self.stage+=1
        tool=self.stage==1
        block=({'type':'tool_use','id':'offline_bash_1','name':'Bash','input':{}} if tool else
               {'type':'text','text':''})
        delta=({'type':'input_json_delta','partial_json':json.dumps({'command':'python code/native_sandbox_probe.py',
                  'description':'Offline native sandbox capability check, no robot/cloud', 'timeout':20000})}
               if tool else {'type':'text_delta','text':'Offline scripted probe finished; no real inference.'})
        events=[
            {'type':'message_start','message':{'id':'offline_probe_'+str(self.stage),'type':'message',
             'role':'assistant','model':body['model'],'content':[],'stop_reason':None,
             'stop_sequence':None,'usage':{'input_tokens':10000,'output_tokens':0}}},
            {'type':'content_block_start','index':0,'content_block':block},
            {'type':'content_block_delta','index':0,'delta':delta},
            {'type':'content_block_stop','index':0},
            {'type':'message_delta','delta':{'stop_reason':'tool_use' if tool else 'end_turn',
             'stop_sequence':None},'usage':{'output_tokens':200}}, {'type':'message_stop'}]
        raw=''.join('event: '+e['type']+'\ndata: '+json.dumps(e)+'\n\n' for e in events).encode()
        return 200,'text/event-stream',io.BytesIO(raw)


def main():
    from harness import agent_host
    from harness.claude_cli import auto_research as claude
    root=HERE/'runs'/('native_probe_'+time.strftime('%Y%m%dT%H%M%S'))
    config,deployed=prepare_session(root)
    (deployed['workspace']/'code/native_sandbox_probe.py').write_text(PROBE.split("\n",1)[1].rsplit("\nPY",1)[0])
    settings=json.loads((root/'teacher_policy.json').read_text())['api_limits']
    # Synthetic reservation ceiling only: the transport never reaches the provider.
    settings={**settings,'max_cost_usd':100,'max_requests':4,'max_wall_seconds':90}
    transport=ScriptedProbe();cap=secrets.token_urlsafe(24)
    relay=BoundedClaudeRelay(settings,'offline-not-a-real-key',cap,root/'requests',transport=transport)
    fd,key=tempfile.mkstemp(prefix='rax-native-probe-');os.fchmod(fd,0o600)
    with os.fdopen(fd,'w') as stream:stream.write(cap)
    agent=server=None;socket=log=None;stdout=stderr=''
    started=time.monotonic();relay.start()
    try:
        deployed['login'].write_text(json.dumps({'agent_cli':'claude','provider':'custom',
            'model':settings['model'],'base_url':relay.url,'key_file':key,'account':None}))
        server,socket,log=start_frontend(config,deployed)
        claude.write_home(config,deployed,socket)
        command=command_for(config,deployed,settings)
        separator=command.index('--')
        command=command[:separator]+['--debug-file',str(root/'native_debug.log'),'--settings',(deployed['claude_home']/'settings.json').read_text(),'--setting-sources','user']+command[separator:]
        (root/'probe_command.json').write_text(json.dumps(command,indent=2))
        environment=claude.agent_environment(config,deployed,socket)
        environment.update(CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC='1',DISABLE_TELEMETRY='1',
            ENABLE_TOOL_SEARCH='false',CLAUDE_CODE_MAX_OUTPUT_TOKENS='8192',
            ANTHROPIC_DEFAULT_OPUS_MODEL=settings['model'],ANTHROPIC_DEFAULT_SONNET_MODEL=settings['model'],
            ANTHROPIC_DEFAULT_HAIKU_MODEL=settings['model'])
        environment.pop('MAX_THINKING_TOKENS',None)
        environment['PATH']=str(HERE/'tools/usr/bin')+':'+environment['PATH']
        agent=agent_host.start(command,cwd=deployed['workspace'],env=environment,
             stdin=subprocess.DEVNULL,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        try:stdout,stderr=agent.communicate(timeout=90)
        except subprocess.TimeoutExpired:
            agent_host.stop(agent);stdout,stderr=agent.communicate(timeout=10)
    finally:
        if agent is not None:agent_host.stop(agent)
        if server is not None:stop_frontend(server,socket,log)
        relay.close();Path(key).unlink(missing_ok=True)
        (root/'cli_stdout.jsonl').write_bytes(relay.safe_bytes(stdout.encode()))
        (root/'cli_stderr.log').write_bytes(relay.safe_bytes(stderr.encode()))
    proof=deployed['workspace']/'output/native_sandbox_proof.json'
    result={'kind':'OFFLINE SCRIPTED NATIVE CLI SANDBOX PROBE','root':str(root),
            'cli_returncode':agent.returncode if agent else None,'proof_written':proof.exists(),
            'scripted_messages':transport.stage,'actual_cloud_inference_requests':0,'robot_steps':0,
            'elapsed_seconds':time.monotonic()-started}
    if proof.exists():result['proof']=json.loads(proof.read_text())
    (root/'probe_result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    if not proof.exists() or result['cli_returncode']!=0:raise SystemExit(1)


if __name__=='__main__':main()
