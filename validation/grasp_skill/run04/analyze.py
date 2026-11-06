"""Independent post-run measurements and Chinese evidence report; no simulation writes."""
from pathlib import Path
import csv,gzip,json,numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'validation/grasp_skill'

def analysis(folder):
 r=json.loads((folder/'result.json').read_text());states=json.load(gzip.open(folder/'trajectory.json.gz','rt'));contacts=list(csv.DictReader(gzip.open(folder/'contacts.csv.gz','rt')))
 f=list(csv.DictReader(gzip.open(folder/'forces.csv.gz','rt')))
 a={'path':str(folder.relative_to(ROOT)),'success':r['success'],'error':r.get('error'),'force_abort_N':r['force_abort_threshold_N'],'peak_intended_jaw_sum_N':r['peak_sum_normal_N'],'intended_force_overshoot_N':r['force_threshold_overshoot_N'],'normal_home_collision_free':r.get('checks',{}).get('home_collision_free'),'recovery':r.get('recovery'),'contact_geom_pairs':sorted({(x['geom1'],x['geom2']) for x in contacts}),'pad1_contact_samples':sum('jaw_pad_R_1' in (x['geom1'],x['geom2']) for x in contacts),'pad2_contact_samples':sum('jaw_pad_R_2' in (x['geom1'],x['geom2']) for x in contacts),'simulation_s':r['simulation_s'],'warnings':r.get('abort',{}).get('warning') if r.get('abort') else None,'hold_metrics':r.get('hold_metrics')}
 bad=(r.get('abort') or {}).get('unexpected_contacts',[])
 a['peak_unexpected_per_contact_N']=max([x['force_N'] for x in bad]+[r.get('peak_unexpected_contact_N',0.)]);a['peak_unexpected_total_N']=max(sum(x['force_N'] for x in bad),r.get('peak_sum_unexpected_contacts_N',0.))
 if 'prelift' in r:
  pre=r['prelift'];ss=[x for x in states if x['phase'] in ('lift','hold')]
  if ss:
   a['measured_lift']={'command_m':r.get('commanded_lift_m',.025),'max_com_rise_m':max(x['com'][2]-pre['com'][2] for x in ss),'max_tcp_rise_m':max(x['tcp'][2]-pre['tcp'][2] for x in ss),'max_relative_translation_change_m':max(np.linalg.norm(np.array(x['relative_position'])-pre['relative_position']) for x in ss),'max_relative_angle_change_deg':max(np.rad2deg(np.linalg.norm(Rotation.from_matrix(np.array(x['relative_rotation'])@np.array(pre['relative_rotation']).T).as_rotvec())) for x in ss)}
 if states:
  tail=states[-50:];a['last_1s_approximately']={'n_samples':len(tail),'phases':sorted({x['phase'] for x in tail}),'peak_jaw_force_N':max(sum(x['jaw_forces_N']) for x in tail),'min_support_N':min(x['support_force_N'] for x in tail),'com_displacement_m':float(np.linalg.norm(np.array(tail[-1]['com'])-tail[0]['com']))}
 hm=r.get('hold_metrics',{});ml=a.get('measured_lift',{})
 a['physical_support_free_hold_verified']=bool(hm.get('min_com_lift_m',0)>=.010 and hm.get('duration_s',0)>=1 and hm.get('max_support_N',1)<.01 and hm.get('opposing_all_samples',False) and ml.get('max_relative_translation_change_m',1)<.003 and np.deg2rad(ml.get('max_relative_angle_change_deg',999))<.05)
 a['opposing_pad_contacts_verified']=a['pad1_contact_samples']>0 and a['pad2_contact_samples']>0
 a['controlled_place_completed']=bool(r.get('released_supported',False))
 if len(states)>1:
  import matplotlib;matplotlib.use('Agg')
  import matplotlib.pyplot as plt
  fig,axs=plt.subplots(3,1,figsize=(10,8),sharex=True)
  tf=[float(x['time_s']) for x in f];axs[0].plot(tf,[float(x['sum_N']) for x in f],label='Jaw sum normal');axs[0].axhline(5,color='red',ls='--',label='Abort threshold (not force cap)');axs[0].set_ylabel('N');axs[0].legend()
  t=[x['t'] for x in states];z0=r.get('prelift',states[0])['com'][2];axs[1].plot(t,[(x['com'][2]-z0)*1000 for x in states],label='Actual panel COM');axs[1].axhline(10,color='green',ls='--');axs[1].set_ylabel('COM rise (mm)');axs[1].legend()
  axs[2].plot(tf,[float(x['support_N']) for x in f],label='Support normal force',lw=.7);axs[2].set_ylim(0,1);axs[2].set_ylabel('Support (N)');axs[2].set_xlabel('Simulation time (s)')
  for ax in axs:ax.grid(alpha=.25)
  fig.suptitle(folder.parent.name+' / '+folder.name+' — SIMPLIFIED PANEL; full protocol incomplete');fig.tight_layout();fig.savefig(folder/'measurements.png',dpi=150);plt.close(fig)
 (folder/'analysis.json').write_text(json.dumps(a,indent=2,ensure_ascii=False)+'\n');return a

def main():
 records=[analysis(p.parent) for p in sorted(OUT.glob('run*/trial*/result.json'))]
 runs={p.parent.name:json.loads(p.read_text()) for p in sorted(OUT.glob('run*/result.json'))}
 result={'scope':'独立支座单面板诊断，不代表原桌面整盒任务解决','trials':records,'core_unchanged_all_runs':all(x['core_unchanged'] for x in runs.values()),'hash_file_counts':{k:v['hash_file_count'] for k,v in runs.items()}}
 (OUT/'result.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
 print(json.dumps(result,indent=2,ensure_ascii=False))
if __name__=='__main__':main()
