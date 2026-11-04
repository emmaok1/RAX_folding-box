"""Scripted Anthropic wire fixture. No learned model, network API, or API charges.

Runs the real installed Claude Code and official RAX harness against deterministic
SSE tool-use replies. This checks plumbing only and must never be called inference.
"""
import io
import json
import os
from pathlib import Path
import tempfile


class ScriptedTransport:
    def __init__(self): self.stage = 0

    def __call__(self, path, body, headers):
        if path.endswith('/count_tokens'):
            return 200,'application/json',io.BytesIO(json.dumps({'input_tokens':8000}).encode())
        if self.stage == 0:
            name,args = 'mcp__auto_research__start_episode',{}
        elif self.stage == 1:
            def locate(node):
                if isinstance(node,dict):
                    if 'state' in node and isinstance(node['state'],dict) and 'tcp_pose' in node['state']:
                        return node['state']['tcp_pose']
                    if node.get('type')=='text':
                        try:
                            result = locate(json.loads(node.get('text','')))
                            if result is not None: return result
                        except ValueError: pass
                    for value in node.values():
                        result = locate(value)
                        if result is not None: return result
                elif isinstance(node,list):
                    for value in reversed(node):
                        result = locate(value)
                        if result is not None: return result
                return None
            tcp = locate(body['messages'])
            if tcp is None: raise ValueError('Fixture could not find image observation / robot TCP tool result')
            row = tcp[:7]+[.3]+tcp[7:14]+[.3]
            row[2] += .002; row[10] += .002
            name,args = 'mcp__auto_research__dexmate_step',{'actions':[row],'repeat':25}
        else: name,args = None,None
        self.stage += 1
        block = {'type':'tool_use','id':'fixture_tool_'+str(self.stage),'name':name,'input':{}} if name else {'type':'text','text':''}
        events = [
            {'type':'message_start','message':{'id':'msg_fixture_'+str(self.stage),'type':'message',
                'role':'assistant','model':body['model'],'content':[],'stop_reason':None,
                'stop_sequence':None,'usage':{'input_tokens':8000,'output_tokens':0}}},
            {'type':'content_block_start','index':0,'content_block':block},
            {'type':'content_block_delta','index':0,'delta':{'type':'input_json_delta','partial_json':json.dumps(args)}} if name else
            {'type':'content_block_delta','index':0,'delta':{'type':'text_delta','text':'OFFLINE SCRIPTED FIXTURE: both TCPs moved in the simulator and updated images returned. No model inference occurred. Folding and physical locking remain incomplete.'}},
            {'type':'content_block_stop','index':0},
            {'type':'message_delta','delta':{'stop_reason':'tool_use' if name else 'end_turn','stop_sequence':None},'usage':{'output_tokens':200}},
            {'type':'message_stop'}]
        raw = ''.join('event: '+e['type']+'\ndata: '+json.dumps(e)+'\n\n' for e in events).encode()
        return 200,'text/event-stream',io.BytesIO(raw)


def run_fixture(config,deployed,settings):
    from .smoke import run_smoke
    fd,path = tempfile.mkstemp(prefix='rax-offline-fixture-')
    with os.fdopen(fd,'w') as stream: stream.write('offline-fixture-not-a-real-api-key')
    try: return run_smoke(config,deployed,settings,Path(path),transport=ScriptedTransport(),offline=True)
    finally: Path(path).unlink(missing_ok=True)
