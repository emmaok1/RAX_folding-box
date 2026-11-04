"""Actual RAX MCP discovery/refusal checks. Zero simulation steps or model calls."""
from pathlib import Path
import json
from rax_integration.launch import new_root,prepare,start_frontend,stop_frontend,MCPClient,value

def main():
    root=new_root('planning_mcp');config,deployed=prepare(root,'claude-sonnet-4-6');server,path,log=start_frontend(config,deployed);c=MCPClient(path);checks={}
    try:
        c.request('initialize');names=[x['name'] for x in c.request('tools/list')['tools']];required=['dexmate_motion_'+x for x in ['plan','preview','execute','status','stop']];checks['all_motion_tools_discovered']=all(x in names for x in required)
        obs=value(c.call('start_episode'));pose=obs['state']['tcp_pose'][:7];args={'episode_id':obs['episode_id'],'expected_step_id':0,'arm':'right','target':{'frame':'world','tcp':'tcp_R','position_m':pose[:3],'quaternion_wxyz':pose[3:]}}
        r=value(c.call('dexmate_motion_plan',args));checks['production_dynamic_coverage_refused']=r['reason']=='INSUFFICIENT_DYNAMIC_WORLD_COVERAGE';checks['planning_no_physics_steps']=r['step_id']==0
        r=value(c.call('dexmate_motion_execute',{'plan_id':'does-not-exist'}));checks['invalid_execution_no_physics_steps']=r['step_id']==0 and r['transition']['motion']['reason']=='PLAN_NOT_FOUND';checks['refused_execution_keeps_control_mode']=r['control_feedback']['mode']=='cartesian_ik'
        value(c.call('dexmate_motion_stop',{'reason':'local MCP STOP validation'}));status=value(c.call('dexmate_motion_status'));checks['stop_visible']=status['status']=='stopped' and status['step_id']==0
        value(c.call('finish'));checks['core_source_unchanged']=all(json.loads(p.read_text())['source_unchanged'] for p in (root/'native/episodes').glob('*/result.json'));assert all(checks.values()),checks
    finally:c.close();stop_frontend(server,path,log)
    (root/'planning_mcp_results.json').write_text(json.dumps({'checks':checks,'passed':all(checks.values()),'inference_requests':0,'simulation_steps':0,'scope':'Actual current box scene refusal/contract only, not tracked execution or cardboard avoidance.'},indent=2));print(root);print(checks)
if __name__=='__main__':main()
