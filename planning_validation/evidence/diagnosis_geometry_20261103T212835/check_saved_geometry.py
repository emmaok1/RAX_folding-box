from pathlib import Path
import json,math
import numpy as np,mujoco
root=Path(__file__).resolve().parents[4]
r=root/'rax_integration/runs/fold_20261103T130237_be872c';ep=next((r/'native/episodes').iterdir());source=ep/'source_snapshot'
m=mujoco.MjModel.from_binary_path(str(ep/'loaded_model.mjb'))
panels=[x['panel'] for x in json.loads((source/'geometry/panels.json').read_text())['panels'] if x['panel']!='base']
rows={}
for line in (ep/'operator.jsonl').open():
 x=json.loads(line)
 if x.get('event') in ('action','planner_joint_action'):rows[x['step_id']]=x
robot_names={side:[f'{prefix}_arm_j{i}' for i in range(1,8)] for side,prefix in [('right','R'),('left','L')]}
def fill(row):
 d=mujoco.MjData(m);assigned=set()
 state=row.get('task_state') or rows[100]['task_state']
 def put(name,value):
  value=np.atleast_1d(value);j=m.joint(name);a=int(m.jnt_qposadr[j.id]);d.qpos[a:a+len(value)]=value;assigned.update(range(a,a+len(value)))
 put('box_free',state['box_pose'])
 for name,value in zip(panels,state['hinge_angles']):put('crease_'+name,value)
 for side,prefix,index in [('right','R',0),('left','L',7)]:
  q=row['measured'][side]['q'] if row['event']=='planner_joint_action' else state['arm_joint_positions'][index:index+7]
  for name,value in zip(robot_names[side],q):put(name,value)
  jaws=row['measured'][side]['jaws'] if row['event']=='planner_joint_action' else state['gripper_pose'][0:2] if side=='right' else state['gripper_pose'][2:4]
  for i,value in enumerate(jaws,1):put(f'{prefix}_gripper_j{i}',value)
 assert len(assigned)==m.nq,(len(assigned),m.nq)
 mujoco.mj_fwdPosition(m,d)
 return d

def contacts(d):
 result=[]
 for c in d.contact:
  a,b=map(int,c.geom);ga,gb=m.geom(a).name,m.geom(b).name;ba,bb=m.body(int(m.geom_bodyid[a])).name,m.body(int(m.geom_bodyid[b])).name
  if not (ba.startswith('dexmate_') or bb.startswith('dexmate_')):continue
  result.append({'geoms':[ga,gb],'bodies':[ba,bb],'distance_m':float(c.dist)})
 return sorted(result,key=lambda x:x['distance_m'])
report={'method':'Frozen loaded MJB + saved qpos reconstruction, mj_fwdPosition only; no mj_step, controller, renderer or force attribution. Planned117 uses preceding raw100 for box state; its static contacts are still exact for reconstructed robot posture.','samples':[]}
for step in (100,117,187,227,247,327):
 row=rows[step];d=fill(row);actual=contacts(d);check={}
 for side,prefix,index in [('right','R',0),('left','L',7)]:
  p=d.site_xpos[m.site('tcp_'+prefix).id];logged=np.array(row['task_state']['tcp_pose'][index:index+3]) if row['event']=='action' else None
  check[side]=float(np.linalg.norm(p-logged)) if logged is not None else None
 # This changes only private diagnostic qpos, never any execution process.
 for side in ('right','left'):
  q=row['applied'][side] if row['event']=='planner_joint_action' else row['motor_references'][side]
  for name,value in zip(robot_names[side],q):d.qpos[int(m.jnt_qposadr[m.joint(name).id])]=value
 mujoco.mj_fwdPosition(m,d)
 sample={'step':step,'tcp_reconstruction_position_difference_m':check,'actual_pose_robot_contacts':actual,'hypothetical_command_pose_robot_contacts':contacts(d)}
 report['samples'].append(sample);print(json.dumps(sample),flush=True)
Path(__file__).with_name('results.json').write_text(json.dumps(report,indent=2))
