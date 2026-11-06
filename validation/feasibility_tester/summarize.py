import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=Path('validation/feasibility_tester');out=[]
fig,axs=plt.subplots(3,1,figsize=(9,8),sharex=True)
for k in [1,2]:
 rs=json.loads((p/f'attempt{k}_trace.json').read_text());r=json.loads((p/f'attempt{k}_result.json').read_text())
 r['method']='Execution exclusively FoldingBoxEnv.step motor references. No executed-state, geometry, physics, collision, force, or equality modifications. Initial overhang assumption was incorrect: table world edge −0.55m; front wall edge −0.52m. Trace sampled at50Hz.'
 (p/f'attempt{k}_result.json').write_text(json.dumps(r,indent=2)+'\n')
 maxc=max((dict(c,time_s=x['time_s'],phase=x['phase']) for x in rs for c in x['contacts']),key=lambda c:c['force_N'][0],default=None)
 endphase={phase:next(x for x in reversed(rs) if x['phase']==phase) for phase in ['insert','close','hold','release','retract','settle_after']}
 r['max_contact']=maxc;r['phases']={ph:{'angle_deg':x['angle_deg'],'tcp':x['tcp'],'ik_scratch_error':x['ik'],'contacts':x['contacts']} for ph,x in endphase.items()}
 r['angle_range_deg']=[min(x['angle_deg'] for x in rs),max(x['angle_deg'] for x in rs)]
 r['time_above15_s']=sum(x['angle_deg']>=15 for x in rs)*.02
 r['initial_final_base_translation_m']=(np.array(rs[-1]['box_pose'][:3])-rs[0]['box_pose'][:3]).tolist();r['attempt']=k;out.append(r)
 t=[x['time_s'] for x in rs];axs[0].plot(t,[x['angle_deg'] for x in rs],label=f'Attempt {k}')
 axs[1].plot(t,[sum(c['force_N'][0] for c in x['contacts']) for x in rs],label=f'Attempt {k}')
 axs[2].plot(t,[1000*x['max_box_penetration_m'] for x in rs],label=f'Attempt {k}')
axs[0].axhline(15,ls='--',c='k',label='Required 15°');axs[0].set_ylabel('Front crease (degrees)');axs[0].legend()
axs[1].set_ylabel('Hand/card normal force (N)');axs[2].set_ylabel('Max card penetration (mm)');axs[2].set_xlabel('Simulation time (s)')
for ax in axs:ax.grid(alpha=.25)
fig.suptitle('Motor-only physical attempts: no opposing-jaw grasp; no 15° lift')
fig.tight_layout();fig.savefig(p/'measurements.png',dpi=150)
(p/'summary.json').write_text(json.dumps({'checkpoint_achieved':False,'attempts':out,'sampling_Hz':50,'limitations':['Contact/penetration/warnings sampled after each20ms control step; substep maxima were not recorded.','Two wrist rolls were physically attempted. Four tilts and80 random IK seeds were checked only in scratch kinematics.','Unsuccessful bounded trials do not establish impossibility.']},indent=2)+'\n')
print(json.dumps([{key:r[key] for key in ['attempt','max_angle_deg','angle_range_deg','opposing_contact_frames','max_box_penetration_m_at50Hz','max_contact','initial_final_base_translation_m']} for r in out],indent=2))
