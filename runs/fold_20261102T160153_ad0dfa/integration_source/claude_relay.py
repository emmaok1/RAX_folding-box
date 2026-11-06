"""Local, bounded Anthropic Messages relay for the unchanged RAX Claude launcher.

Only this operator process has the real API key. No request headers or credential
values are recorded. The CLI sees an ephemeral loopback capability. Each inference
is admitted only after the official count_tokens endpoint fits the token ledger.
Cost admission reserves 2x base input cost (covers 1h cache writes) plus capped
output cost. Reservations are not refunded; retries consume the same ledger.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time
import urllib.error
import urllib.request


def usage_accounting(entries, settings):
    """Per-phase list-price estimate; resumed CLI totals can include prior phases."""
    totals={k:0 for k in ('input_tokens','cache_creation_input_tokens','cache_read_input_tokens','output_tokens')}
    amount=0.
    for entry in entries:
        u=entry['provider_usage']
        for key in totals:totals[key]+=u.get(key,0)
        cache=u.get('cache_creation') or {}
        short=cache.get('ephemeral_5m_input_tokens',0)
        long=cache.get('ephemeral_1h_input_tokens',0)
        unspecified=max(0,u.get('cache_creation_input_tokens',0)-short-long)
        amount+=(settings['input_usd_per_million']*(u.get('input_tokens',0)+.1*u.get('cache_read_input_tokens',0)
                  +1.25*short+2*long+settings['cache_write_multiplier_bound']*unspecified)
                 +settings['output_usd_per_million']*u.get('output_tokens',0))/1e6
    totals['total_input_including_cache']=sum(totals[k] for k in ('input_tokens','cache_creation_input_tokens','cache_read_input_tokens'))
    return totals,round(amount,10)


class BoundedClaudeRelay:
    def __init__(self, settings, secret, capability, root, *, transport=None):
        if settings['endpoint'] != 'https://api.anthropic.com' or settings['provider'] != 'anthropic':
            raise ValueError('Only the direct official Anthropic endpoint is supported')
        self.settings, self.secret, self.capability = settings, secret, capability
        self.root = Path(root)
        self.root.mkdir(parents=True,mode=0o700)
        self.started = time.monotonic()
        self.lock = threading.Lock()
        self.blocked = False
        self.transport = transport or self._https
        self.ledger = {'inference_requests':0, 'token_count_requests':0, 'input_tokens_reserved':0,
                       'output_tokens_reserved':0, 'cost_reserved_usd':0., 'rejections':[], 'usage':[]}
        owner = self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_): pass
            def do_POST(self): owner.handle(self)
            def do_GET(self):
                if self.path.startswith('/v1/models'):
                    self.send_response(200); self.send_header('Content-Type','application/json'); self.end_headers()
                    self.wfile.write(json.dumps({'data':[{'id':settings['model'],'type':'model','display_name':settings['model']}],'has_more':False}).encode())
                else: owner.error(self,404,'Unsupported route')
        self.server = ThreadingHTTPServer(('127.0.0.1',0), Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever,daemon=True)

    @property
    def url(self): return 'http://127.0.0.1:'+str(self.server.server_address[1])
    def start(self): self.thread.start()
    def close(self):
        self.server.shutdown(); self.server.server_close(); self.thread.join(timeout=5)
        self.persist()

    def safe_bytes(self, raw):
        for secret in (self.secret,self.capability):
            if secret: raw = raw.replace(secret.encode(), b'<redacted>')
        return raw

    def persist(self):
        (self.root/'budget.json').write_text(json.dumps({**self.ledger,'limits':self.settings,
                        'elapsed_seconds':time.monotonic()-self.started},indent=2))

    def _https(self, path, body, headers):
        # Fixed official URL: no client-supplied host, redirects or proxy credentials.
        request = urllib.request.Request('https://api.anthropic.com'+path,
            data=json.dumps(body).encode(), headers={**headers, 'x-api-key':self.secret,
            'Content-Type':'application/json','Accept-Encoding':'identity','anthropic-version':'2023-06-01'})
        class NoRedirect(urllib.request.HTTPRedirectHandler):
            def redirect_request(self, *_): return None
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}),NoRedirect())
        response = opener.open(request, timeout=min(45,max(1,self.settings['max_wall_seconds']-(time.monotonic()-self.started))))
        return response.status, response.headers.get('Content-Type','application/json'), response

    def error(self, handler, status, message):
        handler.send_response(status); handler.send_header('Content-Type','application/json'); handler.end_headers()
        handler.wfile.write(json.dumps({'type':'error','error':{'type':'invalid_request_error','message':message}}).encode())

    def handle(self, handler):
        with self.lock:
            try:
                authorized = handler.headers.get('Authorization') == 'Bearer '+self.capability or handler.headers.get('x-api-key') == self.capability
                if not authorized: return self.error(handler,401,'Invalid local capability')
                path = handler.path.split('?',1)[0]
                if path not in ('/v1/messages','/v1/messages/count_tokens'):
                    return self.error(handler,404,'Only Messages and token counts are allowed')
                length = int(handler.headers.get('Content-Length','0'))
                if not 0 < length <= 16*1024*1024: raise ValueError('Request size limit')
                body = json.loads(handler.rfile.read(length))
                s = self.settings
                if self.blocked: raise ValueError('Provider usage exceeded reservation; relay stopped')
                if time.monotonic()-self.started >= s['max_wall_seconds']: raise ValueError('Wall time limit')
                if body.get('model') != s['model']: raise ValueError('Unapproved model')
                if self.ledger['inference_requests'] >= s['max_requests']: raise ValueError('Model request limit')
                headers = {}
                # Provider beta features only; never log or forward caller auth headers.
                if handler.headers.get('anthropic-beta'): headers['anthropic-beta'] = handler.headers['anthropic-beta']
                if self.ledger['token_count_requests'] >= s['max_token_count_requests']: raise ValueError('Token-count request limit')
                body['thinking'] = {'type':'disabled'}
                count_body = {k:v for k,v in body.items() if k in ('model','messages','system','tools','tool_choice','thinking')}
                self.ledger['token_count_requests'] += 1
                status, _, response = self.transport('/v1/messages/count_tokens',count_body,headers)
                with response: counted = json.loads(response.read())
                if status != 200 or not isinstance(counted.get('input_tokens'),int): raise ValueError('Provider token count unavailable')
                # Provider documents small estimation differences; reserve extra headroom.
                tokens = counted['input_tokens']+max(256,(counted['input_tokens']+99)//100)
                if path.endswith('/count_tokens'):
                    handler.send_response(200); handler.send_header('Content-Type','application/json'); handler.end_headers()
                    handler.wfile.write(json.dumps(counted).encode()); self.persist(); return
                maximum = min(int(body.get('max_tokens',s['max_output_tokens_per_request'])),s['max_output_tokens_per_request'])
                if maximum < 1: raise ValueError('Invalid output limit')
                if tokens > s['max_input_tokens_per_request'] or tokens+self.ledger['input_tokens_reserved'] > s['max_input_tokens_total']:
                    raise ValueError('Input token limit')
                if maximum+self.ledger['output_tokens_reserved'] > s['max_output_tokens_total']: raise ValueError('Output token limit')
                reserve = (tokens*s['input_usd_per_million']*s['cache_write_multiplier_bound']+maximum*s['output_usd_per_million'])/1e6
                if self.ledger['cost_reserved_usd']+reserve > s['max_cost_usd']: raise ValueError('Cost limit')
                self.ledger['inference_requests'] += 1
                self.ledger['input_tokens_reserved'] += tokens
                self.ledger['output_tokens_reserved'] += maximum
                self.ledger['cost_reserved_usd'] += reserve
                ident = f"{self.ledger['inference_requests']:03d}"
                body['max_tokens'] = maximum
                # Disable optional reasoning for this deliberately tiny protocol test.
                body['thinking'] = {'type':'disabled'}
                (self.root/(ident+'.request.json')).write_bytes(self.safe_bytes(json.dumps(body).encode()))
                self.persist()
                status, content_type, response = self.transport('/v1/messages',body,headers)
                handler.send_response(status); handler.send_header('Content-Type',content_type); handler.end_headers()
                recorded = bytearray()
                with response:
                    while True:
                        block = response.read1(65536) if hasattr(response,'read1') else response.read(65536)
                        if not block: break
                        recorded.extend(block)
                        handler.wfile.write(block); handler.wfile.flush()
                raw = bytes(recorded)
                (self.root/(ident+'.response.raw')).write_bytes(self.safe_bytes(raw))
                events = []
                if 'text/event-stream' in content_type:
                    for line in raw.splitlines():
                        if line.startswith(b'data: '):
                            try: events.append(json.loads(line[6:]))
                            except ValueError: pass
                else:
                    try: events.append(json.loads(raw))
                    except ValueError: pass
                usage = {}
                for event in events:
                    usage.update((event.get('message') or {}).get('usage') or {})
                    usage.update(event.get('usage') or {})
                self.ledger['usage'].append({'request':ident,'provider_usage':usage,'http_status':status})
                # Actual usage is recorded separately from conservative cost reservations.
                billed_input = sum(usage.get(k,0) for k in ('input_tokens','cache_read_input_tokens','cache_creation_input_tokens'))
                if billed_input > tokens: self.ledger['rejections'].append('Provider input exceeded count margin; further requests blocked')
                if billed_input > tokens or usage.get('output_tokens',0)>maximum:
                    self.blocked = True
                self.persist()
            except (ValueError, OSError, urllib.error.HTTPError) as exc:
                # No raw exception string: upstream errors may contain credentials.
                message = str(exc) if isinstance(exc,ValueError) else type(exc).__name__
                if isinstance(exc,urllib.error.HTTPError):
                    (self.root/'provider_error.json').write_bytes(self.safe_bytes(exc.read()))
                self.ledger['rejections'].append(message); self.persist()
                try: self.error(handler,400,'Bounded smoke stopped: '+message)
                except OSError: pass
