"""Bounded read-only alternative seeds for a recorded rejected grasped arc pose."""
from fixture import *
import argparse,hashlib

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--trial',required=True);args=ap.parse_args();out=ROOT/args.trial;e=FoldEnv();m=e.model;d=e.data;a=e.arms['right'];h=m.joint(CREASE).id
 before=hashlib.sha256(d.qpos.tobytes()+d.qvel.tobytes()+d.ctrl.tobytes()).hexdigest();prediction=json.loads((out/'nominal_arc_prediction.json').read_text());failed=prediction['samples'][-1];previous=prediction['samples'][-2];target=np.array(failed['target']);angle=failed['predicted_angle_rad'];scratch=mujoco.MjData(m)
 names=[m.geom(i).name or '' for i in range(m.ngeom)];groups={m.geom(n).id for n in ['jaw_pad_R_1','jaw_pad_R_2','dexmate_R_gripper_l1_collision_118','dexmate_R_gripper_l2_collision_121']};card=m.geom(CARD).id;base=m.geom('card_base_0').id;fixture_names={n for n,_,_ in FIXTURE}
 def describe(q,angle):
  scratch.qpos[:]=d.qpos;scratch.qpos[a['qad']]=q;scratch.qpos[e.grippers['right']['qad']]=.016;scratch.qpos[m.jnt_qposadr[h]]=angle;mujoco.mj_forward(m,scratch);bad=[]
  for c in scratch.contact:
   ids={int(c.geom1),int(c.geom2)};gn=[names[int(c.geom1)],names[int(c.geom2)]]
   intended=card in ids and bool(ids&groups);fixture_base=base in ids and any(n in fixture_names for n in gn)
   if not intended and not fixture_base and (any(n.startswith(('dexmate_','jaw_pad_')) for n in gn) or card in ids) and c.dist<-1e-6:bad.append({'geoms':gn,'distance_m':float(c.dist)})
  jp=np.zeros((3,m.nv));jr=np.zeros_like(jp);mujoco.mj_jacSite(m,scratch,jp,jr,a['site']);J=np.vstack([jp[:,a['vid']],.25*jr[:,a['vid']]])
  return {'bad_contacts':bad,'minimum_joint_margin_rad':float(np.min(np.minimum(q-a['limits'][:,0],a['limits'][:,1]-q))),'weighted_jacobian_singular_values':np.linalg.svd(J,compute_uv=False).tolist()}
 prior=np.array(previous['q']);perturb=np.array([0,0,.2,0,-.2,0,0]);seeds=[('normal_home',a['qcmd'].copy()),('previous_plus',np.clip(prior+perturb,a['limits'][:,0]+.005,a['limits'][:,1]-.005)),('previous_minus',np.clip(prior-perturb,a['limits'][:,0]+.005,a['limits'][:,1]-.005))];results=[]
 for name,seed in seeds:
  start=describe(seed,angle);q,error=e.inverse_kinematics('right',target,iterations=500,seed=seed);end=describe(q,angle);results.append({'seed_name':name,'seed':seed.tolist(),'seed_geometry':start,'iterations':500,'q':q.tolist(),'error':error,**end,'accepted':error['position_m']<=.0003 and error['orientation_rad']<=.003 and not end['bad_contacts'] and end['minimum_joint_margin_rad']>=0})
 after=hashlib.sha256(d.qpos.tobytes()+d.qvel.tobytes()+d.ctrl.tobytes()).hexdigest();assert before==after
 result={'prediction_only':True,'target':failed,'previous_passed':previous,'attempts':results,'execution_state_unchanged':True,'state_sha_before':before,'state_sha_after':after,'conclusion':'Local bounded IK evaluation only; not proof of global unreachability.'};(out/'alternative_seed_diagnosis.json').write_text(json.dumps(result,indent=2));print(json.dumps(results,indent=2));e.close()
if __name__=='__main__':main()
