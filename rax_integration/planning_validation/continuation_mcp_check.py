"""Actual local RAX frontend/gateway execution; no agent/provider or cloud calls."""
import json

from rax_integration.launch import new_root, prepare, start_frontend, stop_frontend, MCPClient, value


def main():
    root=new_root('continuation_mcp')
    config,deployed=prepare(root,'claude-sonnet-4-6')
    (root/'motion_policy.json').write_text(json.dumps({'world_policy':'simulator_snapshot',
        'task':'MP-TRACK-CONTINUE-20261104','scope':'local full-scene free-space commissioning; no cloud/contact/folding'}))
    server,path,log=start_frontend(config,deployed)
    client=MCPClient(path);checks={};chunks=[]
    try:
        client.request('initialize')
        definitions=client.request('tools/list')['tools']
        execute=next(t for t in definitions if t['name']=='dexmate_motion_execute')
        checks['discovered_bounded_optional_chunk']=execute['inputSchema']['properties']['max_rows']['maximum']==50
        checks['discovered_cursor_contract']=execute['inputSchema']['properties']['expected_cursor']['minimum']==0
        initial=value(client.call('start_episode'))
        pose=initial['state']['tcp_pose'][:7]
        goal=list(pose[:3]);goal[1]+=.03
        args={'episode_id':initial['episode_id'],'expected_step_id':0,'arm':'right',
              'target':{'frame':'world','tcp':'tcp_R','position_m':goal,'quaternion_wxyz':pose[3:]}}
        plan=value(client.call('dexmate_motion_plan',args))
        checks['full_scene_plan']=plan.get('status')=='planned'
        if plan.get('status')!='planned':
            checks['plan_refusal']=plan
        else:
            preview=value(client.call('dexmate_motion_preview',{'plan_id':plan['plan_id']}))
            checks['whole_plan_exceeds_percall']=preview['total_rows']>50
            obs=value(client.call('dexmate_observe',{'cameras':['back']}))
            checks['plan_preview_nonmutation']=obs['step_id']==0 and obs['sim_time_s']==0
            first=value(client.call('dexmate_motion_execute',{'plan_id':plan['plan_id'],'max_rows':7,'expected_cursor':0}))
            result=first['transition']['motion'];chunks.append(result)
            checks['first_chunk_exact7']=result['rows_executed']==7 and result['cursor']==7
            checks['first_chunk_not_full_goal']=result['status']=='paused' and not result['command_completed'] and not result['goal_reached']
            native_log=root/'native/episodes'/initial['episode_id']/'operator.jsonl'
            def admitted_actions():
                return sum(x.get('event')=='tool_call' and x.get('name')=='dexmate_motion_execute'
                           for x in [json.loads(line) for line in native_log.read_text().splitlines()])
            admitted_before=admitted_actions()
            for label,cursor,reason in [('duplicate',0,'CURSOR_MISMATCH'),('skipped',8,'CURSOR_MISMATCH'),('missing',None,'EXPECTED_CURSOR_REQUIRED')]:
                request={'plan_id':plan['plan_id'],'max_rows':7}
                if cursor is not None:request['expected_cursor']=cursor
                rejected=value(client.call('dexmate_motion_execute',request))
                refusal=rejected['transition']['motion']
                checks[label+'_cursor_no_motion']=refusal.get('reason')==reason and not refusal.get('executed') and rejected['step_id']==7
                checks[label+'_cursor_no_action_budget']=admitted_actions()==admitted_before
            for _ in range(10):
                if result['status']!='paused':break
                packet=value(client.call('dexmate_motion_execute',{'plan_id':plan['plan_id'],'expected_cursor':result['cursor']}))
                result=packet['transition']['motion'];chunks.append(result)
            checks['same_plan_full_completion']=result.get('goal_reached') is True and all(c['plan_id']==plan['plan_id'] for c in chunks)
            checks['cursor_no_duplicate_rows']=sum(c['rows_executed'] for c in chunks)==result['cursor']
            checks['percall_ceiling']=all(c['rows_executed']<=50 for c in chunks)
            status=value(client.call('dexmate_motion_status',{'plan_id':plan['plan_id']}))
            checks['status_cursor_complete']=status['cursor']==status['total_rows'] and status['owner_plan'] is None
        value(client.call('dexmate_motion_stop',{'reason':'finish local commissioning; no further actions'}))
        status=value(client.call('dexmate_motion_status'))
        checks['stop_visible']=status['status']=='stopped'
        value(client.call('finish'))
        native=list((root/'native/episodes').glob('*/result.json'))
        checks['native_stop_result']=bool(native) and all(json.loads(p.read_text())['termination_reason']=='operator_stop' for p in native)
        checks['core_sources_unchanged']=all(json.loads(p.read_text())['source_unchanged'] for p in native)
        checks['native_audit_video']=all((p.parent/'operator.jsonl').exists() and list(p.parent.glob('*.mp4')) for p in native)
    finally:
        client.close();stop_frontend(server,path,log)
    report={'passed':all(v is True for v in checks.values()),'checks':checks,'chunks':chunks,
            'inference_requests':0,'scope':'Actual local RAX/MCP; full original robot/table/cardboard; measured joint execution, no agent/cloud/contact/folding.'}
    (root/'continuation_mcp_results.json').write_text(json.dumps(report,indent=2))
    print('EVIDENCE',root,flush=True);print(json.dumps(report),flush=True)
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
