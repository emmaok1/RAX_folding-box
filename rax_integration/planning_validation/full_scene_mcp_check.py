"""Actual RAX full-scene plan/preview/policy checks; no cloud or physics steps."""
from pathlib import Path
import json
from rax_integration.launch import new_root,prepare,start_frontend,stop_frontend,MCPClient,value

def main():
    root=new_root('full_scene_planning_mcp');config,deployed=prepare(root,'claude-sonnet-4-6')
    policy=root/'motion_policy.json';policy.write_text(json.dumps({'world_policy':'simulator_snapshot','task':'MP-FULL-20261103-072531','information_source':'explicit local simulator collision state; not vision-only'}))
    server,path,log=start_frontend(config,deployed);client=MCPClient(path);checks={}
    try:
        client.request('initialize')
        names=[x['name'] for x in client.request('tools/list')['tools']]
        checks['motion_tools_discovered']=all('dexmate_motion_'+x in names for x in ('plan','preview','execute','status','stop'))
        observation=value(client.call('start_episode'));pose=observation['state']['tcp_pose'][:7]
        position=list(pose[:3]);position[1]+=.012
        arguments={'episode_id':observation['episode_id'],'expected_step_id':0,'arm':'right','target':{'frame':'world','tcp':'tcp_R','position_m':position,'quaternion_wxyz':pose[3:]}}
        plan=value(client.call('dexmate_motion_plan',arguments));checks['full_scene_plan_succeeds']=plan.get('status')=='planned'
        checks['explicit_simulator_policy']=plan.get('world_policy')=='simulator_snapshot'
        checks['full_scene_collision_pairs']=plan.get('checked_pairs',0)>581
        if plan.get('status')=='planned':
            preview=value(client.call('dexmate_motion_preview',{'plan_id':plan['plan_id']}));checks['preview_no_live_step']=preview.get('status')=='preview' and preview.get('live_physics_advanced') is False
            current=value(client.call('dexmate_observe'));checks['plan_and_preview_keep_step_zero']=current['step_id']==0 and current['sim_time_s']==0
            policy.write_text(json.dumps({'world_policy':'robot_static'}))
            result=value(client.call('dexmate_motion_execute',{'plan_id':plan['plan_id']}));checks['changed_policy_prevents_execution']=result['step_id']==0 and result['transition']['motion']['reason']=='POLICY_CHANGED'
        else:checks['plan_detail']=plan
        value(client.call('dexmate_motion_stop',{'reason':'end of no-step full-scene interface check'}));status=value(client.call('dexmate_motion_status'));checks['stop_visible_without_motion']=status['status']=='stopped' and status['step_id']==0
        value(client.call('finish'))
        checks['core_source_unchanged']=all(json.loads(p.read_text())['source_unchanged'] for p in (root/'native/episodes').glob('*/result.json'))
    finally:
        client.close();stop_frontend(server,path,log)
    report={'passed':all(v is True for v in checks.values()),'checks':checks,'inference_requests':0,'simulation_steps':0,'scope':'Full actual scene via real local RAX/MCP plan+preview+policy refusal. No executed trajectory in this check.'}
    (root/'full_scene_mcp_results.json').write_text(json.dumps(report,indent=2));print('EVIDENCE',root,flush=True);print(json.dumps(report),flush=True)
    if not report['passed']:raise SystemExit(1)
if __name__=='__main__':main()
