"""Exercise budget rejection boundaries using only a local scripted transport."""
import io
import json
from pathlib import Path
import urllib.error
import urllib.request

from .claude_relay import BoundedClaudeRelay


def run(root, base_settings):
    checks = {}
    class Transport:
        def __init__(self): self.messages=[]; self.count=0
        def __call__(self,path,body,headers):
            if path.endswith('count_tokens'):
                self.count += 1
                return 200,'application/json',io.BytesIO(b'{"input_tokens": 1000}')
            self.messages.append(body)
            return 200,'application/json',io.BytesIO(b'{"usage":{"input_tokens":1000,"output_tokens":5},"content":[]}')
    def exercise(label, overrides, requests, predicate):
        transport = Transport()
        relay = BoundedClaudeRelay({**base_settings,**overrides},'fixture-secret','fixture-cap',root/label,transport=transport)
        relay.start()
        statuses=[]
        try:
            for body in requests:
                request = urllib.request.Request(relay.url+'/v1/messages',data=json.dumps(body).encode(),
                    headers={'Content-Type':'application/json','x-api-key':'fixture-cap'})
                try:
                    with urllib.request.urlopen(request,timeout=5) as response: statuses.append(response.status)
                except urllib.error.HTTPError as exc: statuses.append(exc.code)
            checks[label]=bool(predicate(relay,transport,statuses))
        finally: relay.close()
        if not checks[label]: raise AssertionError('Relay guard failed: '+label)
    request={'model':base_settings['model'],'max_tokens':5000,'messages':[{'role':'user','content':'local fixture'}]}
    exercise('output_clamped',{},[request],lambda r,t,s:s==[200] and t.messages[0]['max_tokens']==512)
    exercise('request_cap',{'max_requests':1},[request,request],lambda r,t,s:s==[200,400] and len(t.messages)==1)
    exercise('input_per_request_cap',{'max_input_tokens_per_request':1000},[request],lambda r,t,s:s==[400] and not t.messages)
    exercise('input_total_cap',{'max_input_tokens_total':2000},[request,request],lambda r,t,s:s==[200,400] and len(t.messages)==1)
    exercise('output_total_cap',{'max_output_tokens_total':512},[request,request],lambda r,t,s:s==[200,400] and len(t.messages)==1)
    exercise('cost_cap',{'max_cost_usd':.001},[request],lambda r,t,s:s==[400] and not t.messages)
    exercise('model_pinned',{},[{**request,'model':'not-approved'}],lambda r,t,s:s==[400] and not t.messages and t.count==0)
    exercise('token_count_cap',{'max_token_count_requests':1},[request,request],lambda r,t,s:s==[200,400] and t.count==1)
    exercise('wall_cap',{'max_wall_seconds':0},[request],lambda r,t,s:s==[400] and not t.messages and t.count==0)
    (root/'validation.json').write_text(json.dumps({'all_passed':all(checks.values()),'checks':checks,
                    'actual_model_inference_requests':0,'actual_api_cost_usd':0},indent=2))
    return checks
