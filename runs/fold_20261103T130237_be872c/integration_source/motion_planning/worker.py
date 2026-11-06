"""One isolated OMPL query. Input contains static XML + robot DOFs, no box truth."""
from pathlib import Path
import sys,json,time
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'rax_integration/planner_runtime/site-packages'))
import numpy as np
from importlib.metadata import version
from rax_integration.motion_planning.ompl_engine import geometric_path
from rax_integration.motion_planning.model import StaticRobotModel
from rax_integration.motion_planning.collision import CollisionChecker
from rax_integration.motion_planning.trajectory import retime

def solve(query):
    if version('ompl')!='1.7.0' or __import__('mujoco').__version__!='3.4.0':return {'status':'refused','reason':'DEPENDENCY_VERSION_MISMATCH'}
    cfg=query['config'];started=time.monotonic();stop=Path(query['stop_file']);cancel=lambda:stop.exists() or time.monotonic()-started>cfg['planning_timeout_s']
    robot=StaticRobotModel(query['static_xml'],query['snapshot']);started=time.monotonic();side=query['arm'];checker=CollisionChecker(robot,side,cfg['collision_clearance_m']);start=np.array(query['snapshot']['robot'][side]['q'])
    def refusal(code,**kw):return {'status':'refused','reason':code,'executed':False,'samples':checker.samples,'collision':checker.last_conflict,**kw}
    if not checker.check(start):return refusal('START_IN_COLLISION')
    ik_deadline=started+cfg['planning_timeout_s']*.5
    goals,best=robot.inverse_candidates(side,query['target'],query['snapshot'],cfg,lambda:cancel() or time.monotonic()>=ik_deadline,acceptable=checker.check)
    if stop.exists():return refusal('STOP_REQUESTED')
    if cancel():return refusal('PLANNING_TIMEOUT',goal_ik_best_error=best)
    if not goals:return refusal('IK_NOT_FOUND',goal_ik_best_error=best)
    goals=sorted([q for q in goals if checker.check(q)],key=lambda q:float(np.linalg.norm(q-start)))
    if not goals:return refusal('GOAL_IN_COLLISION',goal_ik_best_error=best)
    def valid(q):return not cancel() and checker.samples<cfg['max_validation_samples'] and checker.check(q)
    found,path_method=geometric_path(start,goals,robot.index[side]['limits'],valid,cancel,cfg)
    if stop.exists():return refusal('STOP_REQUESTED')
    if found is None:return refusal('NO_PATH_WITHIN_BUDGET',goal_ik_best_error=best)
    rows=retime(found,{**cfg,'max_rows':cfg['max_rows']-5},np.array(query['velocity_limits_rad_s']))
    if rows is None:return refusal('TRAJECTORY_TOO_LONG',path_waypoints=len(found),joint_path_span_rad=np.max(np.abs(np.array(found)-start),axis=0).tolist())
    rows+= [rows[-1]]*5
    for a,b in zip([start]+[np.array(x) for x in rows[:-1]],rows):
        if not checker.edge(a,b,cfg,cancel):return refusal('PATH_VALIDATION_FAILED')
    robot.set_arm(side,rows[-1]);p,R=robot.pose(side)
    from scipy.spatial.transform import Rotation
    G=Rotation.from_quat(np.array(query['target']['quaternion_wxyz'])[[1,2,3,0]]);error={'position_m':float(np.linalg.norm(p-query['target']['position_m'])),'orientation_rad':float((G*R.inv()).magnitude())}
    return {'status':'planned','executed':False,'rows':rows,'duration_s':len(rows)*cfg['control_dt_s'],'planned_goal_error':error,'planner':'OMPL1.7.0/RRTConnect','path_method':path_method,'collision_samples':checker.samples,'checked_pairs':len(checker.pairs),'structural_exclusions':checker.excluded,'clearance_lower_bound_m':cfg['collision_clearance_m'],'edge_validation':'Discrete <=0.004 rad L2 during search; final edges <= configured per-joint resolution. Not certified continuous collision checking.','dependency_versions':{'ompl':version('ompl'),'mujoco':'3.4.0'},'runtime_s':time.monotonic()-started}

if __name__=='__main__':
    query=json.loads(Path(sys.argv[1]).read_text());target=Path(sys.argv[2])
    try:result=solve(query)
    except Exception as exc:result={'status':'refused','reason':'PLANNER_ERROR','error_type':type(exc).__name__,'detail':str(exc)}
    target.write_text(json.dumps(result,allow_nan=False))
