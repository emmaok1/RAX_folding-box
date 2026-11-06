"""Task-local entry points; importing or preparing never sends model requests."""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time
import uuid

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parent
UPSTREAM = HERE/'upstream'
os.environ.setdefault('MUJOCO_GL', 'egl')
for variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[variable] = '1'
os.environ['RAX_ARTIFACT_ROOT'] = str(HERE/'runs')
(HERE/'runs').mkdir(exist_ok=True, mode=0o700)
if not (UPSTREAM/'runtime').exists():
    (UPSTREAM/'runtime').symlink_to('../runs', target_is_directory=True)
sys.path.insert(0, str(UPSTREAM))


def require_cloud_scope():
    scope = json.loads((HERE/'CURRENT_SCOPE.json').read_text())
    if not scope.get('cloud_inference_allowed', False):
        raise ValueError('Current phase is offline-only. New explicit user scope and budget authorization are required before enabling cloud inference.')


def config_for(root):
    from services.controller.config import Configuration, Limits
    limits = Limits(wall_seconds=180, memory_mb=2048, cpus=1., pids=128,
                    scratch_mb=128, artifact_bytes=32*1024*1024)
    return Configuration(root=root, task='cardboard_box_folding', benchmark='dexmate_box',
        exploration_seeds=(11,), formal_seed=100000, eval_seed=0, sim_gpu='cpu',
        development=limits, formal=limits, training=limits, observation_profile='rgb-only',
        agent_cli='claude', agent_provider='anthropic', external_perception=False,
        demonstration_context='none', exploration_envs=1, subagents=0)


def new_root(mode):
    root = HERE/'runs'/(mode+'_'+time.strftime('%Y%m%dT%H%M%S')+'_'+uuid.uuid4().hex[:6])
    root.mkdir(parents=True, mode=0o700)
    return root


def prepare(root, model):
    from harness.codex_cli.auto_research import prepare_workspace
    from harness.claude_cli import auto_research as claude
    config = config_for(root)
    files = prepare_workspace(config, claude.HOME_NAME)
    claude.claude_workspace(files['workspace'])
    binary = claude.claude_binary()
    login = root/'agent-login.json'
    login.write_text(json.dumps({'agent_cli': 'claude', 'provider': 'anthropic', 'model': model,
                                'key_file': None, 'account': None}, indent=2))
    login.chmod(0o600)
    deployed = {'workspace': files['workspace'], 'claude_home': files['home'], 'claude': binary,
                'configuration': files['configuration'], 'login': login,
                'model': model, 'subagents': 0, 'resume_session': None}
    claude.write_home(config, deployed)
    return config, deployed


class MCPClient:
    """The same newline JSON-RPC transport used by the official RAX MCP bridge."""
    def __init__(self, path):
        self.socket = socket.socket(socket.AF_UNIX)
        self.socket.settimeout(60)
        self.socket.connect(str(path))
        self.stream = self.socket.makefile('rwb')
        self.ident = 0

    def request(self, method, params=None):
        self.ident += 1
        self.stream.write(json.dumps({'jsonrpc': '2.0', 'id': self.ident,
                                     'method': method, 'params': params or {}}).encode()+b'\n')
        self.stream.flush()
        result = json.loads(self.stream.readline())
        if 'error' in result: raise RuntimeError(result['error'])
        return result['result']

    def call(self, name, args=None):
        return self.request('tools/call', {'name': name, 'arguments': args or {}})

    def close(self):
        self.stream.close()
        self.socket.close()


def value(result):
    if result.get('isError'): raise RuntimeError(result['content'][0]['text'])
    return json.loads(result['content'][0]['text'])


def start_frontend(config, deployed):
    from harness import agent_host
    from harness.codex_cli.auto_research import start_frontend
    path = agent_host.socket_path(config.root)
    log = (config.root/'frontend.log').open('ab')
    log.launch_start = log.tell()
    server = start_frontend(config, deployed, path, log)
    return server, path, log


def stop_frontend(server, path, log):
    server.terminate()
    try: server.wait(timeout=30)
    except subprocess.TimeoutExpired:
        server.kill()
        server.wait(timeout=5)
    path.unlink(missing_ok=True)
    log.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode', choices=['check', 'prepare', 'smoke', 'fixture'])
    p.add_argument('--config', type=Path, default=HERE/'smoke_config.json')
    p.add_argument('--key-file', type=Path)
    p.add_argument('--approve-budget-usd', type=float)
    args = p.parse_args()
    settings = json.loads(args.config.read_text())
    scope = json.loads((HERE/'CURRENT_SCOPE.json').read_text())
    if args.mode in ('check','fixture','prepare') and not scope.get('integration_checks_allowed',True):
        p.error('Core redesign is in progress. Integration checks/preparation are on hold until the new-version handoff.')
    if args.mode == 'smoke':
        try: require_cloud_scope()
        except ValueError as exc: p.error(str(exc))
        if args.approve_budget_usd is None or args.approve_budget_usd < settings['max_cost_usd']:
            p.error('Smoke inference needs explicit --approve-budget-usd covering smoke_config.json; prepare/check are offline')
        if args.key_file is None:
            p.error('Supply --key-file with a direct Anthropic API key in a private file outside the project')
    root = new_root(args.mode)
    config, deployed = prepare(root, settings['model'])
    if args.mode == 'check':
        from .checks import run_checks
        run_checks(config, deployed)
    elif args.mode == 'fixture':
        from .offline_fixture import run_fixture
        run_fixture(config,deployed,settings)
    elif args.mode == 'smoke':
        from .smoke import run_smoke
        run_smoke(config, deployed, settings, args.key_file)
    else:
        (root/'budget-review.json').write_text(json.dumps(settings, indent=2))
        print(json.dumps({'prepared': True, 'inference_sent': False, 'directory': str(root),
                          'budget_for_review': settings}, indent=2))


if __name__ == '__main__': main()
