"""Actual local RAX: STOP a paused plan and prove terminal state/no resumed row. No cloud/viewer."""
from pathlib import Path
import json
from rax_integration.launch import new_root,prepare,start_frontend,stop_frontend,MCPClient,value

def main():
    root=new_root('paused_stop_mcp');config,deployed=prepare(root,'claude-sonnet-4-6')
    (root/'motion_policy.json').write_text(json.dumps({'world_policy':'simulator_snapshot','scope':'local bounded STOP test; no cloud/contact/folding'}))
    server,path,log=start_frontend(config,deployed);client=MCPClient(path)
    checks={};detail={};error=None
    try:
        client.request('initialize');initial=value(client.call('start_episode'))
        pose=initial['state']['tcp_pose'][:7];pos=pose[:3].copy();pos[2]+=.01
        plan=value(client.call('dexmate_motion_plan',{'episode_id':initial['episode_id'],'expected_step_id':0,'arm':'right',
            'target':{'frame':'world','tcp':'tcp_R','position_m':pos,'quaternion_wxyz':pose[3:]}}))
        detail['plan']=plan;checks['real_full_scene_plan']=plan.get('status')=='planned'
        if not checks['real_full_scene_plan']:raise RuntimeError('Plan was refused before STOP probe')
        packet=value(client.call('dexmate_motion_execute',{'plan_id':plan['plan_id'],'expected_cursor':0,'max_rows':7}))
        detail['first_chunk']=packet['transition']['motion'];checks['paused_at7']=detail['first_chunk']['status']=='paused' and packet['step_id']==7
        trace=root/'native/episodes'/initial['episode_id']/'operator.jsonl'
        def count():return sum(x.get('event')=='tool_call' and x.get('name')=='dexmate_motion_execute' for x in [json.loads(line) for line in trace.read_text().splitlines()])
        before=count();reply=value(client.call('dexmate_motion_stop',{'reason':'local paused-plan STOP test'}));detail['stop']=reply
        status=value(client.call('dexmate_motion_status',{'plan_id':plan['plan_id']}));detail['status_after_stop']=status
        checks['immediate_terminal_state']=status['plan_state']=='stopped'
        checks['immediate_owner_release']=status['owner_plan'] is None
        checks['stop_ack_not_pending']=reply['pending_execution_boundary'] is False and plan['plan_id'] in reply['terminalized_plans']
        resumed=value(client.call('dexmate_motion_execute',{'plan_id':plan['plan_id'],'expected_cursor':7}))
        refusal=resumed.get('transition',{}).get('motion',{})
        detail['resume_refusal']=refusal
        checks['resume_refused']=refusal.get('reason') in ('PLAN_ALREADY_USED','STOP_REQUESTED','EPISODE_ENDED') and not refusal.get('executed',False)
        # RAX closes the robot-call channel after an ended observation. Verify
        # final state from that observation plus the native durable cursor audit,
        # rather than asking the closed episode for another robot status call.
        cursor=json.loads((root/'motion_planning'/(plan['plan_id']+'.cursor.json')).read_text())
        checks['no_extra_step_or_cursor']=resumed['step_id']==7 and cursor['cursor']==7 and cursor['state']=='stopped'
        checks['no_extra_action_charge']=count()==before
        value(client.call('finish'))
        results=list((root/'native/episodes').glob('*/result.json'))
        checks['native_operator_stop']=bool(results) and all(json.loads(p.read_text())['termination_reason']=='operator_stop' for p in results)
        checks['core_unchanged']=all(json.loads(p.read_text())['source_unchanged'] for p in results)
    except Exception as exc:
        error={'type':type(exc).__name__,'message':str(exc)}
    finally:
        client.close();stop_frontend(server,path,log)
    report={'passed':error is None and all(checks.values()),'checks':checks,'detail':detail,'error':error,'inference_requests':0,
            'scope':'Actual local frontend/gateway and full-scene motor execution7rows; STOP-state test only, no grasp/folding.'}
    (root/'paused_stop_results.json').write_text(json.dumps(report,indent=2));print('EVIDENCE',root);print(json.dumps(report))
    if not report['passed']:raise SystemExit(1)

if __name__=='__main__':main()
