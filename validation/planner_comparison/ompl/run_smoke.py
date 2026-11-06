"""Reproduce the shared robot smoke with only real OMPL planning."""
from pathlib import Path
import sys, os, time, hashlib, json
STARTED=time.perf_counter()
os.environ.setdefault('MUJOCO_GL','egl')
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from common import run, PROTOCOL
from planner import OMPLPlanner
adapter=None

def plan(start,goal,valid,label):
    global adapter
    if adapter is None:
        adapter=OMPLPlanner(valid.limits,HERE/'run',PROTOCOL['planning_time_limit_seconds'])
    return adapter(start,goal,valid,label)

if __name__=='__main__':
    identity={'route':'OMPL + MuJoCo','planner':'RRTConnect','ompl_version':'2.0.1',
              'mujoco_version':__import__('mujoco').__version__,
              'python':sys.version,'pre_run_import_s':time.perf_counter()-STARTED,
              'own_and_shared_source_sha256':{str(p.relative_to(HERE.parent)):hashlib.sha256(p.read_bytes()).hexdigest()
                for p in [HERE/'planner.py',HERE/'run_smoke.py',HERE.parent/'common.py',HERE.parent/'protocol.json']}}
    report=run(plan,HERE/'run',identity)
    sys.exit(0 if report['result']=='pass' else 1)
