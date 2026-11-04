"""Tiny 2-D RRTConnect branch check, not manipulation or a performance benchmark."""
from pathlib import Path
import sys,time,numpy as np
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'rax_integration/planner_runtime/site-packages'))
from rax_integration.motion_planning.ompl_engine import geometric_path
start=np.array([-.8,0.]);goal=np.array([.8,0.]);deadline=time.monotonic()+3
valid=lambda q:np.linalg.norm(q)>.3
path,method=geometric_path(start,[goal],np.array([[-1,1],[-1,1]]),valid,lambda:time.monotonic()>deadline,{'seed':17,'edge_joint_resolution_rad':.01})
assert method=='RRTConnect' and path is not None
for a,b in zip(path[:-1],path[1:]):
 for u in np.linspace(0,1,300):assert valid((1-u)*a+u*b)
print(method,'non-direct obstacle path validated; no physics stepping')
