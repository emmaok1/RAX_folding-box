from pathlib import Path
import json,gzip,csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=Path(__file__).resolve().parent;summary=[];fig,ax=plt.subplots(3,1,figsize=(9,8))
for label,folder in [('trial1','load_4p0'),('trial2','trial2')]:
 p=P/folder;r=json.loads((p/'result.json').read_text());trace=json.loads((p/'state_trace.json').read_text()) if (p/'state_trace.json').exists() else json.load(gzip.open(p/'state_trace.json.gz','rt'));final=r.get('final',r.get('stopped_state'));initial=r['initial'];predrag=r['pre_drag'];postdrag=r.get('post_drag',final)
 with gzip.open(p/'substep_forces.csv.gz','rt') as f:ss=list(csv.DictReader(f))
 dr=[x for x in ss if x['phase']=='drag'];a=np.array([float(x['hand_sum_normal_N']) for x in dr]);ft=np.array([float(x['hand_sum_tangent_N']) for x in dr])
 result={'trial':label,'nominal_load_N':r['requested_load_N'],'completed_slide_and_release':bool(r.get('completed_slide_protocol')),'error':r['error'],'simulation_s':r['simulation_seconds'],'substeps':r['physics_substeps_observed'],'base_translation_from_initial_mm':(1000*(np.array(final['box_pose'][:3])-initial['box_pose'][:3])).tolist(),'base_yaw_change_deg':final['base_euler_xyz_deg'][2]-initial['base_euler_xyz_deg'][2],'base_final_roll_pitch_deg':final['base_euler_xyz_deg'][:2],'base_max_abs_roll_pitch_deg':np.max(abs(np.array([x['base_euler_xyz_deg'][:2] for x in trace]+[final['base_euler_xyz_deg'][:2]])),axis=0).tolist(),'actual_tcp_drag_delta_mm':(1000*(np.array(postdrag['tcp'][:3])-predrag['tcp'][:3])).tolist(),'base_drag_delta_mm':(1000*(np.array(postdrag['box_pose'][:3])-predrag['box_pose'][:3])).tolist(),'drag_normal_N_percentiles_0_50_95_100':np.percentile(a,[0,50,95,100]).tolist(),'drag_mean_normal_N':float(a.mean()),'drag_fraction_within_3_to_5N':float(np.mean((a>=3)&(a<=5))),'drag_tangential_N_percentiles_50_95_100':np.percentile(ft,[50,95,100]).tolist(),'peak_sum_hand_normal_N':max(x['hand_sum_normal_N'] for x in r['phase_peaks'].values()),'max_card_penetration_mm':1000*max(x['card_penetration_m'] for x in r['phase_peaks'].values()),'max_robot_penetration_mm':1000*max(x['robot_penetration_m'] for x in r['phase_peaks'].values()),'max_abs_passive_crease_change_deg':max(abs(x['hinges_deg'][n]-initial['hinges_deg'][n]) for x in trace+[final] for n in initial['hinges_deg']),'final_crease_changes_deg':{n:final['hinges_deg'][n]-initial['hinges_deg'][n] for n in initial['hinges_deg']},'final_support_N':sum(x['normal_N'] for x in r['final_table_support']),'final_support_contacts':len(r['final_table_support']),'final_base_support_contacts':sum(any(n=='card_base_0' for n in x['geoms']) for x in r['final_table_support']),'final_hand_normal_N':r['final_hand_sum_normal_N'],'warnings':r['warnings'],'forbidden_contact_events':r['forbidden_contact_events'],'final_geometry':r['final_geometry'],'grasp_clearance_validated':False}
 summary.append(result)
 ts=np.array([x['time_s'] for x in trace]);ax[0].plot(ts,[-1000*(x['box_pose'][0]-initial['box_pose'][0]) for x in trace],label=label+' blank');ax[0].plot(ts,[-1000*(x['tcp'][0]-predrag['tcp'][0]) if x['phase']=='drag' else np.nan for x in trace],'--',label=label+' hand')
 # Bin substep maxima to20ms for display, preserving force spikes.
 st=np.array([float(x['t']) for x in ss]);load=np.array([float(x['hand_sum_normal_N']) for x in ss]);bins=np.floor(st/.02).astype(int);unique=np.unique(bins);force=np.maximum.reduceat(load,np.r_[0,np.where(np.diff(bins))[0]+1]);ax[1].plot(unique*.02,force,label=label)
 ax[2].plot(ts,[max(abs(v-initial['hinges_deg'][n]) for n,v in x['hinges_deg'].items()) for x in trace],label=label)
 # Compress50Hz traces after analysis.
 with gzip.open(p/'state_trace.json.gz','wt') as f:json.dump(trace,f,separators=(',',':'))
 (p/'state_trace.json').unlink(missing_ok=True)
ax[0].set_ylabel('Toward edge displacement (mm)');ax[0].set_ylim(-1,80);ax[1].set_ylabel('Hand normal force (N)');ax[1].axhline(4,c='gray',ls=':',label='Nominal4N');ax[2].set_ylabel('Max passive crease change (deg)');ax[2].set_xlabel('Simulation time (s)')
for a in ax:a.grid(alpha=.3);a.legend()
fig.suptitle('Top-contact sliding: hand slips, blank remains inside table edge');fig.tight_layout();fig.savefig(P/'measurements.png',dpi=150)
(P/'summary.json').write_text(json.dumps({'edge_exposure_achieved':False,'grasp_or_fold_executed':False,'trials':summary},indent=2)+'\n')
print(json.dumps(summary,indent=2))
