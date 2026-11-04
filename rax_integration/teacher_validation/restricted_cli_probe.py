"""Real restricted Claude file tools + RAX rehearsal; scripted model, zero cloud/steps."""
import io
import json
import os
from pathlib import Path
import secrets
import subprocess
import sys
import tempfile
import time

from rax_integration.teacher.prepare import HERE, PROJECT, prepare_session, configure_file_tools
from rax_integration.teacher.run import command_for
from rax_integration.claude_relay import BoundedClaudeRelay
from rax_integration.launch import start_frontend, stop_frontend

CONTROLLER = '''import json
from pathlib import Path
def main(ctx):
    world=json.loads(Path('runtime/world/manifest.json').read_text())
    reply=ctx.call('dexmate_teacher_state')
    state=json.loads(next(b['text'] for b in reply['content'] if b['type']=='text'))
    assert state['world_environment_id']==world['environment_id']
    assert state['step_id']==0
    Path('output/restricted_rehearsal_proof.json').write_text(json.dumps({
        'kind':'OFFLINE SCRIPTED CLI + RAX REHEARSAL', 'world_readable':True,
        'teacher_state_readable':True,'step_id':0,'cloud_inference_requests':0}))
'''


def tool_result(messages):
    for message in reversed(messages):
        for block in reversed(message.get('content',[]) if isinstance(message.get('content'),list) else []):
            if block.get('type')=='tool_result':return block
    raise ValueError('Missing scripted tool result')


def find_bundle(node):
    if isinstance(node,dict):
        if isinstance(node.get('bundle'),str):return node['bundle']
        for value in node.values():
            answer=find_bundle(value)
            if answer:return answer
    if isinstance(node,list):
        for value in node:
            answer=find_bundle(value)
            if answer:return answer
    if isinstance(node,str):
        try:return find_bundle(json.loads(node))
        except ValueError:return None


class Scripted:
    def __init__(self, root, canary):
        self.root=root;self.canary=canary;self.stage=0;self.checks={}
    def __call__(self,path,body,headers):
        if path.endswith('count_tokens'):
            return 200,'application/json',io.BytesIO(b'{"input_tokens":10000}')
        if self.stage==0:
            names={t['name'] for t in body.get('tools',[])}
            self.checks['no_native_execution_or_messaging_tools']=not names & {'Bash','Monitor','SendMessage','CronCreate','Workflow','Skill'}
        else:
            result=tool_result(body['messages'])
            error=result.get('is_error',False)
            if self.stage==1:self.checks['outside_canary_read_denied']=error
            if self.stage==2:self.checks['workspace_write_allowed']=not error
            if self.stage==3:self.checks['world_read_allowed']=not error
            if self.stage==4:self.checks['unadvertised_bash_denied']=error
            if self.stage==5:self.checks['world_write_denied']=error
            if self.stage==6:self.checks['controller_write_allowed']=not error
            if self.stage==7:
                self.bundle=find_bundle(result)
                self.checks['controller_registered']=bool(self.bundle) and not error
            if self.stage==8:self.checks['rehearsal_reply_ok']=not error
        name,args=[
          ('Read',{'file_path':str(self.canary)}),
          ('Write',{'file_path':'output/restricted_filetools_proof.json','content':'{"kind":"OFFLINE SCRIPTED FILE TOOL PROOF"}'}),
          ('Read',{'file_path':'runtime/world/manifest.json'}),
          ('Bash',{'command':'echo MUST_NOT_RUN > output/bash_escape','timeout':1000}),
          ('Write',{'file_path':str(self.root/'teacher_world/physics.json'),'content':'MUST_NOT_WRITE'}),
          ('Write',{'file_path':'code/probe/controller.py','content':CONTROLLER}),
          ('mcp__auto_research__register',{'source':'code/probe'}),
          ('mcp__auto_research__rehearse',{'bundle':getattr(self,'bundle','invalid')}),
          (None,None)][min(self.stage,8)]
        self.stage+=1
        block={'type':'tool_use','id':'restricted_'+str(self.stage),'name':name,'input':{}} if name else {'type':'text','text':''}
        delta={'type':'input_json_delta','partial_json':json.dumps(args)} if name else {'type':'text_delta','text':'Offline restricted file tools and RAX probe finished; no learned model or task motion.'}
        events=[{'type':'message_start','message':{'id':'offline_restricted_'+str(self.stage),'type':'message',
          'role':'assistant','model':body['model'],'content':[],'stop_reason':None,'stop_sequence':None,
          'usage':{'input_tokens':10000,'output_tokens':0}}},
          {'type':'content_block_start','index':0,'content_block':block},
          {'type':'content_block_delta','index':0,'delta':delta},
          {'type':'content_block_stop','index':0},
          {'type':'message_delta','delta':{'stop_reason':'tool_use' if name else 'end_turn','stop_sequence':None},'usage':{'output_tokens':200}},
          {'type':'message_stop'}]
        return 200,'text/event-stream',io.BytesIO(''.join('event: '+e['type']+'\ndata: '+json.dumps(e)+'\n\n' for e in events).encode())


def main():
    from harness import agent_host
    from harness.claude_cli import auto_research as claude
    root=HERE/'runs'/('restricted_probe_'+time.strftime('%Y%m%dT%H%M%S'))
    config,deployed=prepare_session(root)
    limits=json.loads((root/'teacher_policy.json').read_text())['api_limits']
    limits={**limits,'max_cost_usd':100,'max_requests':12,'max_wall_seconds':90}
    fd,canary_name=tempfile.mkstemp(prefix='rax-read-canary-')
    with os.fdopen(fd,'w') as f:f.write('SYNTHETIC_CANARY_MUST_NOT_BE_READ')
    canary=Path(canary_name)
    cap=secrets.token_urlsafe(24);fd,key=tempfile.mkstemp(prefix='rax-restricted-probe-');os.fchmod(fd,0o600)
    with os.fdopen(fd,'w') as f:f.write(cap)
    scripted=Scripted(root,canary)
    relay=BoundedClaudeRelay(limits,'offline-not-a-real-key',cap,root/'requests',transport=scripted)
    server=agent=None;socket=log=None;stdout=stderr='';relay.start()
    try:
        deployed['login'].write_text(json.dumps({'agent_cli':'claude','provider':'custom','model':limits['model'],
           'base_url':relay.url,'key_file':key,'account':None}))
        server,socket,log=start_frontend(config,deployed)
        claude.write_home(config,deployed,socket);configure_file_tools(config,deployed)
        command=command_for(config,deployed,limits)
        environment=claude.agent_environment(config,deployed,socket)
        environment.update(CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC='1',DISABLE_TELEMETRY='1',ENABLE_TOOL_SEARCH='false',
          CLAUDE_CODE_MAX_OUTPUT_TOKENS='8192',ANTHROPIC_DEFAULT_OPUS_MODEL=limits['model'],
          ANTHROPIC_DEFAULT_SONNET_MODEL=limits['model'],ANTHROPIC_DEFAULT_HAIKU_MODEL=limits['model'])
        environment['PATH']=str(HERE/'tools/usr/bin')+':'+environment['PATH']
        agent=agent_host.start(command,cwd=deployed['workspace'],env=environment,stdin=subprocess.DEVNULL,
          stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        agent_host.record(agent,root/'agent.pid')
        try:stdout,stderr=agent.communicate(timeout=90)
        except subprocess.TimeoutExpired:
            agent_host.stop(agent);stdout,stderr=agent.communicate(timeout=10)
    finally:
        if agent is not None:agent_host.stop(agent)
        (root/'agent.pid').unlink(missing_ok=True)
        if server is not None:stop_frontend(server,socket,log)
        relay.close();Path(key).unlink(missing_ok=True);canary.unlink(missing_ok=True)
        (root/'cli_stdout.jsonl').write_bytes(relay.safe_bytes(stdout.encode()))
        (root/'cli_stderr.log').write_bytes(relay.safe_bytes(stderr.encode()))
    proofs=list(root.glob('published/runs/*/exports/restricted_rehearsal_proof.json'))
    scripted.checks['frozen_python_rehearsal_proof']=bool(proofs)
    scripted.checks['bash_not_executed']=not (deployed['workspace']/'output/bash_escape').exists()
    scripted.checks['canary_not_in_cli_output']='SYNTHETIC_CANARY_MUST_NOT_BE_READ' not in stdout
    scripted.checks['cli_clean_exit']=agent.returncode==0
    result={'kind':'OFFLINE SCRIPTED RESTRICTED CLI + RAX PROBE','root':str(root),'checks':scripted.checks,
      'scripted_messages':scripted.stage,'actual_cloud_inference_requests':0,'robot_steps':0}
    (root/'probe_result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    if not scripted.checks or not all(scripted.checks.values()):raise SystemExit(1)


if __name__=='__main__':main()
