"""Bounded repeated OMPL commissioning in the full robot/table/cardboard scene.

Test-selected free-space goals, no model decisions/contact/folding. Refusals and
tracking stops remain failures. Each trial starts through the existing reset.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from environment import FoldingBoxEnv
from rax_integration.motion_planning.model import ROOT
from rax_integration.motion_planning.service import MotionService
from rax_integration.motion_planning.limits import plan_rows,execute_rows
from rax_integration.planning_validation.commission_tracking import protected_hashes, write


def run_trial(out, name, side, position):
    env=FoldingBoxEnv(placement='flat')
    service=MotionService(env,'free_space_commission',out,world_policy='simulator_snapshot')
    trace=[]
    service.on_step=lambda entry,obs,info:trace.append({**entry,'physical_progress':info['progress']})
    initial=env.tcp_pose(side).copy()
    target=initial[:3]+np.array([0,0,.01]) if position is None else np.asarray(position[-1] if np.ndim(position)==2 else position)
    request={'episode_id':'free_space_commission','expected_step_id':0,'arm':side,
             'target':{'frame':'world','tcp':'tcp_R' if side=='right' else 'tcp_L',
                       'position_m':target.tolist(),'quaternion_wxyz':initial[3:].tolist()}}
    report={'category':name,'arm':side,'target':request['target'],'full_original_scene':True,
            'goal_selection':'fixed commissioning goal, not cloud decision or folding policy',
            'source_fingerprint':None,'cloud_requests':0,'status':'FAIL','chunks':[]}
    try:
        if name in ('intermediate','near_box_standoff'):
            waypoints=[[.02,-.39,1.115],[-.02,-.36,1.13]]
            if name=='near_box_standoff':waypoints += [[-.07,-.34,1.13],[-.11,-.32,1.125],[-.15,-.30,1.12]]
        else:waypoints=[target.tolist()]
        report['segments']=[]
        for waypoint in waypoints:
            request['expected_step_id']=env.env_step
            request['target']['position_m']=waypoint
            segment=execute_segment(env,service,request)
            report['segments'].append(segment)
            report['plan']=segment['plan'];report['source_fingerprint']=segment['plan'].get('source_hash')
            report['chunks'].extend(segment.get('chunks',[]))
            if not segment['passed']:
                report['failure']=segment['chunks'][-1] if segment['chunks'] else segment['plan']
                return report
        final=env.tcp_pose(side)
        report.update(final_tcp=final.tolist(),actual_displacement_m=float(np.linalg.norm(final[:3]-initial[:3])),
                      final_physical_progress=env.progress(),
                      max_tracking_error_rad=max((r['max_tracking_error_rad'] for r in trace),default=0),
                      actual_sim_time_s=float(env.data.time),actual_control_rows=env.env_step,
                      hand_panel_contacts_peak=max((r['physical_progress']['hand_panel_contacts'] for r in trace),default=0),
                      robot_panel_contacts_peak=max((r['physical_progress']['robot_panel_contacts'] for r in trace),default=0))
        report['status']='PASS' if (report['hand_panel_contacts_peak']==0 and report['robot_panel_contacts_peak']==0) else 'FAIL'
        return report
    finally:
        report['actual_control_rows']=env.env_step
        report['actual_sim_time_s']=float(env.data.time)
        write(out/'result.json',report);write(out/'control_rows.json',trace);env.close()


def execute_segment(env,service,request):
        report={'passed':False,'chunks':[]}
        before=(env.data.qpos.copy(),env.data.qvel.copy(),env.data.ctrl.copy(),env.env_step,float(env.data.time))
        plan=service.plan(request);report['plan']=plan;report['source_fingerprint']=plan.get('source_hash')
        after=(env.data.qpos.copy(),env.data.qvel.copy(),env.data.ctrl.copy(),env.env_step,float(env.data.time))
        report['plan_nonmutating']=all(np.array_equal(a,b) for a,b in zip(before,after))
        if plan['status']!='planned':return report
        preview=service.preview(plan['plan_id']);report['preview_rows']=len(preview.get('joint_positions_rad',[]))
        if preview['status']!='preview':report['preview']=preview;return report
        max_calls=int(np.ceil(plan_rows(service.cfg)/execute_rows(service.cfg)))
        for _ in range(max_calls):
            result=service.execute(plan['plan_id'],expected_cursor=service.plans[plan['plan_id']].get('cursor',0));report['chunks'].append(result)
            if result['status']!='paused':break
        report['passed']=bool(result['goal_reached'] and report['plan_nonmutating'])
        return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repeats',type=int,default=3,choices=(1,2,3))
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    out=args.output or ROOT/'rax_integration/planning_validation/evidence'/('free_space_commission_'+time.strftime('%Y%m%dT%H%M%S',time.gmtime()))
    out.mkdir(parents=True,exist_ok=False)
    before=protected_hashes();results=[]
    # Near-box is a requested standoff, not an assertion of grasp accessibility.
    # No clearance guard is reduced if the planner refuses this goal.
    cases=[('small_right','right',None),('small_left','left',None),
           ('intermediate','right',[-.02,-.36,1.13]),
           ('near_box_standoff','right',[-.15,-.30,1.12])]
    for name,side,position in cases:
        for repeat in range(args.repeats):
            folder=out/(name+'_'+str(repeat));folder.mkdir()
            result=run_trial(folder,name,side,position);results.append(result)
            print(json.dumps({'case':name,'repeat':repeat,'status':result['status'],
                              'reason':result.get('failure',{}).get('reason'),
                              'rows':result.get('actual_control_rows',0),
                              'goal_error':result['chunks'][-1].get('measured_goal_error') if result['chunks'] else None}),flush=True)
    after=protected_hashes()
    write(out/'results.json',{'results':results,'protected_unchanged':before==after,'protected_hashes':before,
                             'inference_requests':0,'passed':all(r['status']=='PASS' for r in results),
                             'scope':'Full current original scene; bounded test-selected unladen motion only. No contact/grasp/folding/physical-locking claim.'})
    print('EVIDENCE',out,flush=True)
    if before!=after:raise RuntimeError('Protected core source changed')


if __name__=='__main__':main()
