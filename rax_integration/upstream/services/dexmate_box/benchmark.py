"""RAX backend hooks for the existing custom task, with no scene changes."""
from pathlib import Path

TASK = 'cardboard_box_folding'
PROJECT = Path(__file__).resolve().parents[4]


def tool_definitions():
    from .tools import tool_definitions
    return tool_definitions()


def backend(config):
    from .backend import DexmateBoxBackend
    return DexmateBoxBackend(config)


def task_names(): return (TASK,)
def task_timeouts(task):
    if task != TASK: raise ValueError('Unknown custom task')
    return {'startup_seconds': 60, 'episode_seconds': 600}


def validate_config(config):
    if config.task != TASK: raise ValueError('Unknown custom task')
    if config.observation_profile != 'rgb-only': raise ValueError('Custom task supports RGB only; select rgb-only')
    if config.demonstration_context != 'none' or config.expert_demonstrations: raise ValueError('No official demonstrations for this custom task')
    if config.exploration_envs != 1 or config.subagents: raise ValueError('This integration uses one environment and no model subagents')


def validate_settings(values): pass
def validate_layouts(config): validate_config(config)
def exploration_seeds(task, collection, start, episodes, **_): return tuple(range(start, start+episodes))
def task_context(config, demonstration, *, live_formal): return (PROJECT/'rax_integration/TASK.md').read_text()
def refresh_workspace(config, workspace):
    if getattr(config, 'teacher_privileged', False):
        from rax_integration.teacher.instructions import refresh_teacher_workspace
        refresh_teacher_workspace(config, workspace)
        return
    # Retain RAX's contract/runtime/skills generation, but give this custom smoke
    # task instructions that do not solicit training or benchmark submission.
    agent='FOLD_AGENT.md' if (Path(config.root)/'fold_policy.json').exists() else 'SMOKE_AGENT.md'
    for source, name in [(agent,'AGENTS.md'), ('ROBOT.md','ROBOT.md')]:
        (Path(workspace)/name).write_text((PROJECT/'rax_integration'/source).read_text())
def reference_mounts(config):
    if getattr(config, 'teacher_privileged', False):
        return [(Path(config.root)/'teacher_world', '/workspace/runtime/world')]
    return []
def fetch_demonstration(demos_root, task, kind):
    if kind != 'none': raise ValueError('No task demonstrations registered')
def provision_demonstration(config, target, demos_root): return {'kind': 'none', 'image_count': 0, 'manifest_path': None}
def episode_reviewer(root): return None
def review_analysis(): return None
def unscoped_call(name, arguments): raise ValueError('No unscoped custom robot tools')
