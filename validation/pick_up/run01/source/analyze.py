"""Independent post-run measurements and Chinese evidence report; no simulation writes."""
from pathlib import Path
import csv,gzip,json,hashlib,sys,mujoco,numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'validation/grasp_skill'

def analysis(folder):
 r=json.loads((folder/'result.json').read_text());states=json.load(gzip.open(folder/'trajectory.json.gz','rt'));contacts=list(csv.DictReader(gzip.open(folder/'contacts.csv.gz','rt')))
 f=list(csv.DictReader(gzip.open(folder/'forces.csv.gz','rt')))
 a={'path':str(folder.relative_to(ROOT)),'success':r['success'],'error':r.get('error'),'force_abort_N':r['force_abort_threshold_N'],'peak_intended_jaw_sum_N':r['peak_sum_normal_N'],'intended_force_overshoot_N':r['force_threshold_overshoot_N'],'normal_home_collision_free':r.get('checks',{}).get('home_collision_free'),'recovery':r.get('recovery'),'contact_geom_pairs':sorted({(x['geom1'],x['geom2']) for x in contacts}),'pad1_contact_samples':sum('jaw_pad_R_1' in (x['geom1'],x['geom2']) for x in contacts),'pad2_contact_samples':sum('jaw_pad_R_2' in (x['geom1'],x['geom2']) for x in contacts),'simulation_s':r['simulation_s'],'warning_abort_observed':bool(any((r.get('abort') or {}).get('warning',[]))),'warning_check_every_substep':True,'hold_metrics':r.get('hold_metrics')}
 bad=(r.get('abort') or {}).get('unexpected_contacts',[])
 a['peak_unexpected_per_contact_N']=max([x['force_N'] for x in bad]+[r.get('peak_unexpected_contact_N',0.)]);a['peak_unexpected_total_N']=max(sum(x['force_N'] for x in bad),r.get('peak_sum_unexpected_contacts_N',0.))
 if 'prelift' in r:
  pre=r['prelift'];ss=[x for x in states if x['phase'] in ('lift','hold')]
  if r.get('stopped',{}).get('phase') in ('lift','hold'):ss.append(r['stopped'])
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
  axs[2].plot(tf,[float(x['support_N']) for x in f],label='Support normal force',lw=.7);axs[2].set_ylabel('Support (N)');axs[2].set_xlabel('Simulation time (s)')
  for ax in axs:ax.grid(alpha=.25)
  fig.suptitle(folder.parent.name+' / '+folder.name+' — SIMPLIFIED PANEL; '+('full protocol passed' if r['success'] else 'full protocol incomplete'));fig.tight_layout();fig.savefig(folder/'measurements.png',dpi=150);plt.close(fig)
  if r.get('abort') and r['abort']['sum_normal_N']>5 and 'pad1_N' in f[0]:
   trip=r['abort']['time_s'];zoom=[x for x in f if abs(float(x['time_s'])-trip)<.003];tz=[1000*(float(x['time_s'])-trip) for x in zoom]
   fig,ax=plt.subplots(figsize=(9,4.5));ax.plot(tz,[float(x['sum_N']) for x in zoom],'-o',ms=3,label='Total opposing jaw normal force');ax.plot(tz,[float(x['pad1_N']) for x in zoom],label='Pad 1');ax.plot(tz,[float(x['jaw1_N'])-float(x['pad1_N']) for x in zoom],label='Vendor finger 1');ax.axhline(5,color='red',ls='--',label='Monitored abort threshold');ax.axvline(0,color='gray',ls=':');ax.set(xlabel='Time relative to abort (ms)',ylabel='Normal force (N)',title='0.25 ms observations: one-step overshoot, then abort recovery');ax.grid(alpha=.25);ax.legend();fig.tight_layout();fig.savefig(folder/'force_abort_zoom.png',dpi=150);plt.close(fig)

 (folder/'analysis.json').write_text(json.dumps(a,indent=2,ensure_ascii=False)+'\n');return a

def main():
 records=[analysis(p.parent) for p in sorted(OUT.glob('run*/trial*/result.json'))]
 runs={p.parent.name:json.loads(p.read_text()) for p in sorted(OUT.glob('run*/result.json'))}
 result={'scope':'独立支座单面板诊断，不代表原桌面整盒任务解决','runtime_versions':{'python':sys.version,'mujoco':mujoco.__version__,'numpy':np.__version__,'ompl':'2.0.1'},'physics_timestep_s':.00025,'control_timestep_s':.02,'trials':records,'core_unchanged_all_runs':all(x['core_unchanged'] for x in runs.values()),'hash_file_counts':{k:v['hash_file_count'] for k,v in runs.items()},'experiment_source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((ROOT/'experiments/grasp_skill').glob('*')) if p.is_file()}}
 final_paths=['validation/grasp_skill/run04/trial00','validation/grasp_skill/run04/trial01','validation/grasp_skill/run05_minus2/trial00']
 result['final_validation']=[a for p in final_paths for a in records if a['path']==p]
 result['final_validation_complete']=len(result['final_validation'])==3
 result['final_validation_passed']=sum(a['success'] for a in result['final_validation'])
 result['robustness_established']=False
 (OUT/'result.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
 lines=['单面板电机夹取实验报告','',
 '范围：简化的独立纸板面板+真实支座。原桌面整盒任务未解决/未验证。',
 '面板：原box_base，300×220×4mm，质量26.4g；原质量惯性、材料及接触参数保持。',
 '支座：130×150×220mm，中心(0,-.65,1.03)m，顶面z=1.14m。机器人从robot_config正常home启动。',
 '控制：OMPL RRTConnect 2.0.1接近，原关节PD电机执行；接触阶段为原RightArmImpedance固定增益。',
 '最终控制器闭合分量反馈目标3N，分量上限.30Nm、变化率.05Nm/s；总电机仍含原阻尼和重力补偿并受原±.6Nm限幅。',
 '接触：两颚/板显式mu=.6，支座/板mu=.4；condim3、solref .004 1等均原样保留。',
 '安全观察：每0.25ms检查两颚法向力之和；5N触发终止，不是保证永不超出的硬力上限。非预期机器人接触>0.05N或穿透>0.1mm也终止。',
 '物理保持成功：对向真实颚组承载、COM实际上升>=10mm、脱离支座、保持>=1s，保持窗口和整个抬升相对位移<3mm/转角<.05rad。',
 '完整成功另要求正常控制放回、张开、退手后支座持续承托和稳定。双新增pad接触仅作策略诊断。','']
 for a in records:
  lines += [a['path'], '  完整协议success='+str(a['success'])+'；物理脱支撑保持='+str(a['physical_support_free_hold_verified'])+'；受控放置完成='+str(a['controlled_place_completed']),
   '  两颚峰值法向力='+format(a['peak_intended_jaw_sum_N'],'.6f')+'N；5N阈值超调='+format(a['intended_force_overshoot_N'],'.6f')+'N；非预期接触单点/合计峰值='+format(a['peak_unexpected_per_contact_N'],'.3f')+'/'+format(a['peak_unexpected_total_N'],'.3f')+'N',
   '  pad1/pad2有力接触采样数='+str(a['pad1_contact_samples'])+'/'+str(a['pad2_contact_samples']),
   '  MuJoCo warning触发='+str(a['warning_abort_observed'])+'（每物理步检查）；正常home碰撞验证='+str(a['normal_home_collision_free']),
   '  抬升指标='+json.dumps(a.get('measured_lift'),ensure_ascii=False),
   '  保持指标='+json.dumps(a.get('hold_metrics'),ensure_ascii=False),
   '  错误='+str(a.get('error')), '']
 lines += ['最终三条件：run04/trial00零偏移；run04/trial01 +2mm；run05_minus2/trial00 -2mm。这三个条件使用相同控制、几何、阈值；运行路由不同并非调参。',
 '最终完成条件数='+str(len(result['final_validation']))+'；完整协议通过数='+str(result['final_validation_passed'])+'。小样本且出现力超阈，不可称鲁棒方案。',
 'run04 +2mm在36.652s出现单物理步总法向力9.46646N，前0.25ms为2.45823N；5N超调4.46646N，监测触发后下一恢复步降至.98360N。由此不能声称5N硬保证。',
 '该步pad1与原厂finger1共同承载，pad2未接触，穿透由12.84um降至12.34um。接触切换/力重分配是可能原因，但20ms成对接触日志不足以证明根因。恢复张开、落回支座、退手完成；不是正常受控放置成功。',
 '历史失败不隐藏：run01为home与最初y=-.4m支座严重相交的无效初始化，0.25ms立即停机，恢复也未完成；修正后在任何步进前验证home碰撞。',
 'run02为2N闭合反馈结束后不再调力，抬升时纸板旋转并仍受支座辅助，真实保持失败。',
 'run03采用持续3N反馈，真实脱支撑保持通过；当时错误附加的双pad门槛阻断正常place，保留原success=false和原脚本，不回写历史。',
 'run04仅去除额外pad门槛并增加physical_hold_1s字段，保留全程3mm/.05rad等物理判据；与run03控制/几何/材料参数无变化。',
 '接触几何诊断：原厂两颚指体在板近边先承载，抬升后pad1参与，pad2仍无接触。不能称作双新增pad平行夹持；也不能因此否认真实指体承载的物理夹取。',
 '候选姿态用设计角.012rad的真实pad内面中平面，TCP偏移(-7um,-1.5mm,-1.373mm)；其有限策略不覆盖其他位置/整盒。',
 '核心文件前后SHA256全一致='+str(result['core_unchanged_all_runs'])+'；各运行哈希文件数='+json.dumps(result['hash_file_counts']),
 '模型审计见model_audit.json：机器人碰撞几何、夹爪/板惯性、重力/步长/积分器、执行器限值、显式接触对一致；物体只有自由关节，无物体执行器/焊接。',
 '运行命令见experiments/grasp_skill/README.md。每轮result.json、analysis.json、forces.csv.gz、contacts.csv.gz、trajectory.json.gz、video.mp4、PNG和measurements.png可复查。',
 'run01因首个物理步即中止没有可播放运动视频，保留初始化/失败图和两步接触证据。其余轮均有视频。',
 '参考思路：ManiSkill pick_cube的几何定姿与阶段执行；robosuite manipulation_env._check_grasp的对向几何组判定。未安装或复制完整栈，原有阻抗vendor许可/来源保持。']
 (OUT/'REPORT.txt').write_text('\n'.join(lines)+'\n')

 print(json.dumps(result,indent=2,ensure_ascii=False))
if __name__=='__main__':main()
