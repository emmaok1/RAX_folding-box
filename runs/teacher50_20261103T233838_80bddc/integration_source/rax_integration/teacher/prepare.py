"""Prepare the normal RAX Claude development workspace; never start an episode/API."""
from __future__ import annotations
from dataclasses import replace
import argparse,hashlib,json,shutil,sys,time,uuid
from pathlib import Path
from .bundles import UPSTREAM,persist,restore_workspace,freeze_workspace
from .identity import hash_file
from .memory import MemoryStore
from .world import create_world_snapshot,verify_world_snapshot,world_sources,digest
from ..launch import config_for

HERE=Path(__file__).resolve().parents[1];PROJECT=HERE.parent

def interface_sources():
    paths=list((HERE/'teacher').glob('*.py'))+list((HERE/'motion_planning').glob('*.py'))
    paths+=[HERE/'motion_planning/config.json',HERE/'requirements-planning.txt']
    paths+=[HERE/'upstream/services/dexmate_box'/name for name in ('backend.py','tools.py','registry.py','benchmark.py')]
    paths+=[HERE/'upstream/services/mcp_contract.py',HERE/'upstream/services/controller/config.py',
            HERE/'upstream/services/controller/sandbox.py', HERE/'upstream/harness/claude_cli/auto_research.py',
            HERE/'upstream/harness/codex_cli/auto_research.py', HERE/'upstream/harness/agent_host.py',
            HERE/'claude_relay.py', HERE/'live_view.py', HERE/'launch.py',
            HERE/'tools/sandbox-runtime/manifest.json', HERE/'tools/sandbox-runtime/bin/bwrap']
    return {str(p.relative_to(PROJECT)):hash_file(p) for p in sorted(paths)}

def configuration(root,*,episodes=3,episode_seconds=600):
    from services.controller.config import Limits
    if type(episodes) is not int or not 1<=episodes<=20:raise ValueError('Explicit bounded episode schedule required (1..20)')
    limits=Limits(wall_seconds=episode_seconds,memory_mb=2048,cpus=1.,pids=128,scratch_mb=128,artifact_bytes=32*1024*1024)
    return replace(config_for(Path(root)),exploration_seeds=tuple(range(episodes)),
        development=limits,formal=limits,training=limits,teacher_privileged=True,
        agent_formal=False,agent_effort='high',write_handoff=True,transfer_parts=('memory','code','handoff'),transfer_mb=100)

def api_settings(model,budget_usd=0.,wall_seconds=600,max_requests=40):
    if model != 'claude-sonnet-4-6':raise ValueError('This priced teacher configuration is pinned to claude-sonnet-4-6; review rates before changing model')
    if not isinstance(budget_usd,(int,float)) or isinstance(budget_usd,bool) or not 0<=budget_usd<=1000:raise ValueError('Invalid API ceiling')
    if type(max_requests) is not int or not 1<=max_requests<=40:raise ValueError('Model request cap must be1..40')
    if type(wall_seconds) is not int or not 1<=wall_seconds<=600:raise ValueError('Session wall cap must be1..600 seconds')
    return {'provider':'anthropic','endpoint':'https://api.anthropic.com','model':model,
        'max_cost_usd':float(budget_usd),'max_requests':max_requests,'max_token_count_requests':2*max_requests,
        'max_wall_seconds':wall_seconds,'max_input_tokens_per_request':160000,'max_input_tokens_total':2800000,
        'max_output_tokens_per_request':8192,'max_output_tokens_total':max_requests*8192,
        'reasoning_policy':'teacher_adaptive_high',
        'input_usd_per_million':3,'output_usd_per_million':15,'cache_write_multiplier_bound':2,
        'pricing_source':'https://platform.claude.com/docs/en/about-claude/pricing',
        'pricing_note':'Pinned Sonnet4.6 rates verified2026-11-03; review before future paid grant or model change.'}

def _no_links(path):
    path = Path(path).absolute()
    if any(part.is_symlink() for part in (path, *path.parents)):
        raise ValueError('Source/destination symlink traversal is forbidden')
    return path

def _copy_tree(source,target):
    source,target=_no_links(source),_no_links(target)
    if not source.exists():return
    for p in source.rglob('*'):
        _no_links(p)
        dest=_no_links(target/p.relative_to(source))
        if p.is_dir():dest.mkdir(parents=True,exist_ok=True)
        elif p.is_file():dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,dest)
        else:raise ValueError('Only regular files/directories may be carried')

def final_instruction_hashes(workspace):
    workspace = Path(workspace)
    paths = [workspace/name for name in ('AGENTS.md','TASK.md','ROBOT.md','TEACHER.md',
             'START_PROMPT.md','TEACHER_LIMITS.json','MCP_CONTRACT.json') if (workspace/name).is_file()]
    paths += [p for p in (workspace/'.agents/skills').rglob('*') if p.is_file()]
    return {str(p.relative_to(workspace)):hash_file(_no_links(p)) for p in sorted(paths)}


def publish_memory(store,workspace):
    """Startup/post-agent-exit snapshot: preserve notes, publish pointer last.

    The caller must have stopped the agent. Canonical authority remains the
    private journal; exported files are historical copies, never live truth.
    """
    workspace=_no_links(workspace);staging=store.root/'exports'/uuid.uuid4().hex
    snapshot=store.export_workspace(staging)
    for name in ('memory','handoff'):_no_links(workspace/name).mkdir(parents=True,exist_ok=True)
    for name in ('PROGRESS.md','INDEX.md','TIPS.md'):
        text=(staging/'memory'/name).read_text()
        if name=='INDEX.md':text=text.replace('skills/','operator_skills/')
        _no_links(workspace/'memory'/('OPERATOR_'+name)).write_text(text)
        authored=_no_links(workspace/'memory'/name)
        if not authored.exists():authored.write_text('Current operator snapshot: OPERATOR_'+name+'\n\n'+text)
    for name in ('store','evidence'):_copy_tree(staging/'memory'/name,workspace/'memory'/name)
    _copy_tree(staging/'memory/skills',workspace/'memory/operator_skills')
    for name in ('structured.json','evidence_status.json'):
        shutil.copyfile(staging/'memory'/name,_no_links(workspace/'memory'/name))
    text=(staging/'handoff/README.md').read_text()
    _no_links(workspace/'handoff/OPERATOR_README.md').write_text(text)
    authored=_no_links(workspace/'handoff/README.md')
    if not authored.exists():authored.write_text(text)
    persist(_no_links(workspace/'memory/RESTORE.json'),{'schema':'rax.teacher.memory_startup_snapshot.v1',
        'environment_id':store.environment_id,'revision':snapshot['revision'],'journal_head':snapshot['head'],
        'operator_index':'memory/OPERATOR_INDEX.md','operator_progress':'memory/OPERATOR_PROGRESS.md',
        'operator_handoff':'handoff/OPERATOR_README.md','publication':'complete; startup/post-agent-exit only',
        'authored_notes':'preserved verbatim; historical, not live truth','execution_resumed':False,'episode_started':False})
    return staging

def configure_file_tools(config,deployed):
    # Native CLI Bash sandbox failed an actual offline probe on this host. Only
    # expose file tools; Python controllers execute in proven RAX BundleSandbox.
    from harness import agent_host
    settings_file=Path(deployed['claude_home'])/'settings.json'
    settings=json.loads(settings_file.read_text())
    private=[str(p.resolve()) for p in agent_host.hidden_paths(config.root)]
    deny=settings['permissions'].setdefault('deny',[])
    for path in private:
        for tool in ('Read','Edit','Write'):
            for target in (path,path+'/**'):
                rule=f'{tool}(/{target})'
                if rule not in deny:deny.append(rule)
    for tool in ('Bash','Monitor'):
        if tool not in deny:deny.append(tool)
    # Exact read-only reference exceptions cannot reopen the broad hidden run
    # tree under file-tool deny-first rules. Carve out just this own workspace
    # and the explicit world/published references before emitting file denies.
    references=[Path(config.root)/'teacher_world',Path(config.root)/'published',Path(deployed['workspace'])]
    own_reads=agent_host.hidden_paths(config.root,carved=True,kept=references)
    deny[:]=[r for r in deny if not r.startswith(('Read(//','Write(//','Edit(//'))]
    for path in own_reads:
        for tool in ('Read','Write','Edit'):
            deny.extend([f'{tool}(/{path})',f'{tool}(/{path}/**)'])
    for path in (Path(deployed['workspace'])/'.claude',Path(deployed['workspace'])/'.mcp.json'):
        for tool in ('Write','Edit'):
            deny.extend([f'{tool}(/{path})',f'{tool}(/{path}/**)'])
    for path in (Path(config.root)/'teacher_world',Path(config.root)/'published',Path(deployed['workspace'])/'runtime/world',Path(deployed['workspace'])/'runtime/autonomous_controller'):
        for tool in ('Write','Edit'):
            deny.extend([f'{tool}(/{path})',f'{tool}(/{path}/**)'])
    settings['permissions']['deny']=list(dict.fromkeys(deny))
    persist(settings_file,settings)


def memory_for(root):
    root=Path(root);identity=json.loads((root/'teacher_memory_identity.json').read_text())
    return MemoryStore(root/'teacher_memory',identity,evidence_root=root/'native/episodes')

def checkpoint_session(root):
    """Freeze memory+code+handoff on exit; no formal evaluation or model resume."""
    root=Path(root);store=memory_for(root);checkpoint=store.checkpoint();publish_memory(store,root/'agent-workspace')
    destination=root/'teacher_checkpoints'/uuid.uuid4().hex;destination.parent.mkdir(exist_ok=True)
    manifest=freeze_workspace(root/'agent-workspace',destination,environment_id=store.environment_id,revision=checkpoint['revision'],max_mb=100)
    record={'bundle':str(destination),'manifest':manifest,'environment_id':store.environment_id,'revision':checkpoint['revision']}
    persist(root/'teacher_checkpoint.json',record)
    return record

def prepare_session(root,*,model='claude-sonnet-4-6',episodes=3,episode_seconds=600,
                    budget_usd=0.,wall_seconds=600,max_requests=40,carry_bundle=None,carry_manifest=None):
    """Create fresh files only. Budget configuration is NEVER spending authorization."""
    from harness.codex_cli.auto_research import prepare_workspace
    from harness.claude_cli import auto_research as claude
    root=Path(root).resolve();root.mkdir(parents=True,mode=0o700,exist_ok=False)
    settings=api_settings(model,budget_usd,wall_seconds,max_requests)
    policy={'schema':'rax.teacher.policy.v1','role':'privileged_teacher_in_auto_research',
        'authoritative_capability':'Configuration.teacher_privileged=true','world_policy':'simulator_snapshot',
        'cloud_inference_authorized':False,'training_authorized':False,'formal_evaluation_allowed':False,
        'exploration_episode_ceiling':episodes,'session_wall_seconds':wall_seconds,'api_limits':settings,
        'scope':'Preparation and local validation only until a NEW explicit paid/episode authorization.'}
    persist(root/'teacher_policy.json',policy);(root/'teacher_policy.json').chmod(0o444)
    persist(root/'motion_policy.json',{'world_policy':'simulator_snapshot'})
    persist(root/'fold_policy.json',{'run_kind':'teacher_development','live_view':True,'offline_fixture':False,
        'max_action_tool_calls':200,'max_action_rows':50,'max_episode_starts':episodes})
    persist(root/'cloud_authorization.json',{'allowed':False,'consumed':False,'max_cost_usd':0,
        'max_requests':0,'max_wall_seconds':0,'max_episode_starts':0,
        'task_id':None,'user_authorization':None,'reason':'No paid grant for current teacher-memory implementation'})
    world=create_world_snapshot(PROJECT,root/'teacher_world')
    identity={'schema':'rax.teacher.memory_identity.v1','world_environment_id':world['environment_id'],
        'interface_files':interface_sources(),'information_policy':'privileged simulator teacher; ordinary contract unchanged',
        'motion_policy':'simulator_snapshot','evaluator_policy':'unchanged; physical-locking success withheld'}
    persist(root/'teacher_memory_identity.json',identity)
    cfg=configuration(root,episodes=episodes,episode_seconds=episode_seconds)
    files=prepare_workspace(cfg,claude.HOME_NAME);claude.claude_workspace(files['workspace'])
    # Benchmark overlay may install method skills after the normal build pass.
    # Render that layer with the real RAX recipe/context, never raw Jinja prompts.
    from services.workspace import Setup,recipe_for
    from services.workspace.build import render_sources
    from services.mcp_contract import Contract
    options=frozenset({'handoff'})
    render_sources(recipe_for(options),Setup(Contract.from_config(cfg),options,cfg.task,'claude'),files['workspace'])
    if carry_bundle is not None:
        if carry_manifest is None:raise ValueError('A verified carry manifest is required')
        restored=root/'restored_carry';restore_workspace(carry_bundle,carry_manifest,restored)
        for name in ('memory','code','handoff'):_copy_tree(restored/name,files['workspace']/name)
        _copy_tree(restored/'memory/store',root/'teacher_memory')
        _copy_tree(restored/'memory/evidence',root/'native/episodes')
    store=memory_for(root);publish_memory(store,files['workspace'])
    binary=claude.claude_binary();login=root/'agent-login.json'
    persist(login,{'agent_cli':'claude','provider':'custom','model':model,'base_url':'http://127.0.0.1:9',
        'key_file':None,'account':None,'preparation_only':True})
    deployed={'workspace':files['workspace'],'claude_home':files['home'],'claude':binary,
        'configuration':files['configuration'],'login':login,'model':model,'subagents':0,'resume_session':None}
    claude.write_home(cfg,deployed)
    configure_file_tools(cfg,deployed)
    manifest={'schema':'rax.teacher.prepared.v1','prepared_at_unix_s':time.time(),
        'world_environment_id':world['environment_id'],'memory_environment_id':store.environment_id,
        'teacher_policy_sha256':hash_file(root/'teacher_policy.json'),'configuration_sha256':hash_file(files['configuration']),
        'memory_identity_sha256':hash_file(root/'teacher_memory_identity.json'),
        'final_deployed_instruction_sha256':final_instruction_hashes(files['workspace']),
        'runtime_policy_sha256':{name:hash_file(root/name) for name in ('motion_policy.json','fold_policy.json')},
        'teacher_privileged':True,'cloud_requests':0,'simulation_steps':0,'deployed':{k:str(v) if isinstance(v,Path) else v for k,v in deployed.items()}}
    persist(root/'teacher_prepared.json',manifest)
    return cfg,deployed

def load_prepared(root):
    from services.controller.config import Configuration
    root=Path(root).resolve();manifest=json.loads((root/'teacher_prepared.json').read_text())
    for name,key in [('teacher_policy.json','teacher_policy_sha256'),('configuration.json','configuration_sha256'),('teacher_memory_identity.json','memory_identity_sha256')]:
        if hash_file(root/name)!=manifest[key]:raise ValueError('Prepared source/config identity changed: '+name)
    for name, expected in manifest['runtime_policy_sha256'].items():
        if hash_file(root/name) != expected:raise ValueError('Runtime policy changed: '+name)
    cfg=Configuration.from_dict(json.loads((root/'configuration.json').read_text()))
    if cfg.root!=root or not cfg.teacher_privileged:raise ValueError('Teacher configuration identity mismatch')
    world=verify_world_snapshot(root/'teacher_world')
    if digest(world_sources(PROJECT))!=world['environment_id']:raise ValueError('Live project world changed; prepare a new environment-bound session')
    identity=json.loads((root/'teacher_memory_identity.json').read_text())
    if identity['world_environment_id']!=world['environment_id'] or identity['interface_files']!=interface_sources():raise ValueError('Environment/interface changed; reprepare and revalidate skills')
    deployed={k:Path(v) if k in ('workspace','claude_home','configuration','login') else v for k,v in manifest['deployed'].items()}
    if final_instruction_hashes(deployed['workspace']) != manifest['final_deployed_instruction_sha256']:
        raise ValueError('Final deployed instructions/skills changed; reprepare')
    return cfg,deployed

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--settings',type=Path);parser.add_argument('--model');parser.add_argument('--episodes',type=int)
    parser.add_argument('--api-budget-usd',type=float);parser.add_argument('--wall-seconds',type=int)
    args=parser.parse_args();options={}
    if args.settings:
        options=json.loads(args.settings.read_text())
        if not isinstance(options,dict) or set(options)-{'model','episodes','episode_seconds','budget_usd','wall_seconds','max_requests'}:parser.error('Unknown teacher preparation setting')
    for key,value in [('model',args.model),('episodes',args.episodes),('budget_usd',args.api_budget_usd),('wall_seconds',args.wall_seconds)]:
        if value is not None:options[key]=value
    cfg,deployed=prepare_session(args.root,**options)
    print(json.dumps({'prepared':str(cfg.root),'teacher_privileged':cfg.teacher_privileged,'workspace':str(deployed['workspace']),'cloud_requests':0,'simulation_steps':0,'paid_authorized':False}))

if __name__=='__main__':main()
