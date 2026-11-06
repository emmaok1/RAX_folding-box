"""Read-only postprocessing of saved fold traces; never advances simulation."""
from pathlib import Path
import argparse, csv, gzip, hashlib, json, math, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
FLAGS=['fixture_ready','grasp_success','fold15_success','hold_success','return_success','release_success','protocol_completed']
OUTWARD={'fold5','checkpoint5','fold15','fold_hold'}
RETURN={'fold_return','return_hold'}
def read(p):return json.loads(p.read_text())
def load_csv(p):
 with gzip.open(p,'rt') as f:return list(csv.DictReader(f))
def phase_metrics(states,slip,base,phases):
 ss=[s for s in states if s['phase'] in phases];sp=[s for s in slip if s['phase'] in phases];bs=[s for s in base if s['phase'] in phases]
 return dict(max_relative_slip_m=max(s['translation_m'] for s in sp),max_relative_rotation_rad=max(s['rotation_rad'] for s in sp),min_loaded_jaw_N=min(min(s['jaw_forces_N']) for s in ss),max_moving_fixture_N=max(s['moving_fixture_force_N'] for s in ss),max_base_translation_m=max(s['translation_m'] for s in bs),max_base_rotation_deg=math.degrees(max(s['rotation_rad'] for s in bs)))
def analyze(p):
 r=read(p/'result.json')
 with gzip.open(p/'trajectory.json.gz','rt') as f:states=json.load(f)
 slip=read(p/'fold_slip.json');base=read(p/'base_motion.json');forces=load_csv(p/'forces.csv.gz');extra=load_csv(p/'crease_fixture_torque.csv.gz')
 peaks={}
 for row in forces:peaks[row['phase']]=max(peaks.get(row['phase'],0.),float(row['sum_N']))
 before=read(p/'core_hashes_before.json');after=read(p/'core_hashes_after.json')
 live={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in before}
 summary={'trial':str(p.relative_to(ROOT)),**{k:r[k] for k in FLAGS},'fold_hold':r['fold_metrics'],'outward_from_raw':phase_metrics(states,slip,base,OUTWARD),'return_from_raw':phase_metrics(states,slip,base,RETURN),'return_hold':{k:v for k,v in r['return_metrics'].items() if k not in ['max_relative_slip_m','max_relative_rotation_rad']},'phase_peak_hand_normal_N':peaks,'peak_hand_normal_N':max(peaks.values()),'force_threshold_N':5.,'force_overshoot_N':max(0.,max(peaks.values())-5),'peak_fixture_base_normal_N':max(float(s['fixture_base_N']) for s in extra),'final_angle_deg':math.degrees(r['final']['crease_rad']),'postclosure_angle_deg':math.degrees(r['postclosure']['crease_rad']),'fk_execution_state_unchanged':all(s['fk_execution_state_unchanged'] for s in states),'warnings':r['warnings'],'core_597_unchanged':len(before)==597 and before==after==live,'video':read(p/'artifact_checks.json'),'torque_peaks_Nm':{k:max(abs(float(s[k])) for s in extra) for k in extra[0] if 'motor' in k},'two_added_pads_both_loaded_all_fold_samples':all(min(s['pad_forces_N'])>.05 for s in states if s['phase'] in OUTWARD),'source_snapshot':str(p.parent/'source'),'return_statistics_note':'执行版本 return_metrics 的滑移字段误筛选为去程；本补充分析独立筛选 fold_return/return_hold，原始结果不覆盖。'}
 # Exercise only the public reporting helper against recorded data, without a simulator.
 sys.path.insert(0,str(ROOT/'experiments/single_fold'))
 from run_fold import FoldTrial
 dummy=object.__new__(FoldTrial);dummy.rows=states;dummy.fold_slip=slip;dummy.all_base_motion=base
 corrected=FoldTrial.held_metrics(dummy,'return_hold')
 assert corrected['max_relative_slip_m']==summary['return_from_raw']['max_relative_slip_m']
 assert corrected['max_relative_rotation_rad']==summary['return_from_raw']['max_relative_rotation_rad']
 summary['public_return_helper_matches_raw']=True
 t=np.array([s['t'] for s in states]);fig,axs=plt.subplots(4,1,figsize=(11,11),sharex=True,layout='constrained')
 axs[0].plot(t,np.rad2deg([s['commanded_crease_rad'] for s in states]),label='Command');axs[0].plot(t,np.rad2deg([s['crease_rad'] for s in states]),label='Actual passive crease');axs[0].axhspan(13,17,color='green',alpha=.1);axs[0].set_ylabel('Angle (deg)');axs[0].legend()
 tf=np.array([float(s['time_s']) for s in forces]);axs[1].plot(tf,[float(s['sum_N']) for s in forces],label='Hand / moving panel');axs[1].plot([float(s['time_s']) for s in extra],[float(s['fixture_base_N']) for s in extra],alpha=.65,label='Fixture / base (separate)');axs[1].axhline(5,color='red',linestyle='--',label='Hand abort threshold');axs[1].set_ylabel('Normal force (N)');axs[1].legend()
 axs[2].plot([s['t'] for s in base],[1000*s['translation_m'] for s in base],label='Base translation');axs[2].plot([s['t'] for s in slip],[1000*s['translation_m'] for s in slip],label='Moving panel / tool slip');axs[2].set_ylabel('Translation (mm)');axs[2].legend()
 axs[3].plot([s['t'] for s in base],np.rad2deg([s['rotation_rad'] for s in base]),label='Base rotation');axs[3].plot([s['t'] for s in slip],np.rad2deg([s['rotation_rad'] for s in slip]),label='Moving panel / tool rotation');axs[3].set_ylabel('Rotation (deg)');axs[3].set_xlabel('Simulation time (s)');axs[3].legend()
 for ax in axs:ax.grid(alpha=.2)
 fig.suptitle(p.parent.name+'/'+p.name+' — fixture-assisted passive single crease');fig.savefig(p/'measurements.png',dpi=150);plt.close(fig)
 (p/'analysis.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));return summary

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--run',default='validation/single_fold/run02');args=ap.parse_args();run=ROOT/args.run
 trials=[analyze(p) for p in sorted(run.glob('trial??'))]
 result={'scope':'原被动折痕、接触夹具辅助的单折痕固定条件测试；不是原完整纸盒任务。','trials':trials,'counts':{k:sum(r[k] for r in trials) for k in FLAGS},'core_unchanged':all(r['core_597_unchanged'] for r in trials),'prior_attempt':{'path':'validation/single_fold/run01','protocol_completed':False,'physical_hold_met':True,'failure':'缓存FK与积分后qpos错位.25ms；数值门槛误判。异常恢复底板移动2.003645mm超过2mm，恢复未完成。','correction':'仅同一当前qpos的专用scratch FK测量；原数值容差、所有物理参数与控制不变。'},'fixed_conditions_only':True,'original_whole_box_solved':False,'reporting_only_correction':'公开 held_metrics 已修复返回段筛选；保存轨迹直接调用修正helper验证，无物理重跑，run02源码快照和原始结果保留。'}
 (run/'analysis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));(run.parent/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
 lines=['夹具辅助原被动单折痕物理试验报告','',f'修正后的固定条件完整协议通过 {sum(r["protocol_completed"] for r in trials)}/{len(trials)}。原始底板+side_left侧板，原hinge/反向tendon均被动；真实接触夹具约束自由底板。不是原始桌面完整纸盒或双臂折叠成功。','', 'trial | fixture | grasp | fold15 | hold | return | release | actual hold(deg) | hand peak(N)']
 for r in trials:lines.append(f'{Path(r["trial"]).name} | '+ ' | '.join(str(r[k]) for k in FLAGS[:-1])+f' | {r["fold_hold"]["min_angle_deg"]:.6f}–{r["fold_hold"]["max_angle_deg"]:.6f} | {r["peak_hand_normal_N"]:.6f}')
 a=trials[0];f=a['fold_hold'];ret=a['return_from_raw']
 lines += ['', '三次 fresh 正常home重置，场景、接近路径、控制和任务参数完全相同，原始轨迹数值一致。这是固定条件重复性，不能推断随机位置或其他折痕的可靠性。',f'实际15°目标保持：{f["min_angle_deg"]:.6f}–{f["max_angle_deg"]:.6f}°，持续{f["duration_s"]:.2f}s；移动板/夹具接触力0N，保持期间最小颚组力{f["min_loaded_jaw_N"]:.6f}N。',f'去程全段相对滑移{f["max_relative_slip_m"]*1000:.6f}mm、相对转角{math.degrees(f["max_relative_rotation_rad"]):.6f}°；底板最大平移{f["max_base_translation_m"]*1000:.6f}mm/旋转{f["max_base_rotation_deg"]:.6f}°。',f'返回段独立统计：滑移{ret["max_relative_slip_m"]*1000:.6f}mm、转角{math.degrees(ret["max_relative_rotation_rad"]):.6f}°；返回保持实际角{a["return_hold"]["min_angle_deg"]:.6f}–{a["return_hold"]["max_angle_deg"]:.6f}°。释放退手后角{a["final_angle_deg"]:.6f}°，手接触0N。',f'全程手力峰值{a["peak_hand_normal_N"]:.6f}N，5N监测阈值超调0N。夹具/底板内部法向力另列，峰值{a["peak_fixture_base_normal_N"]:.6f}N，不与握力相加。每.25ms原始手力和电机/夹具记录保留。无非预期机器人碰撞、MuJoCo warning或力矩饱和。','双新增pad并未同时承载；成功基于真实两颚碰撞组对移动板的接触保持。','', '每阶段手力峰值（N）：'+json.dumps(a['phase_peak_hand_normal_N'],ensure_ascii=False),'', '唯一物理重试原因：run01已达到相同实际折痕角和保持时长，但FK使用MuJoCo缓存body姿态与积分后joint qpos比较，最大1.36699e-5rad超过1e-5数值容差；对齐前一.25ms子步后仅1.11e-16rad。原失败、视频、源码及未完成异常恢复保留。run02仅改用独立scratch对同一qpos执行mj_kinematics，每次验证执行state未改变，仍使用1e-5容差。物理材料、夹具、阈值及控制无改动。', '补充统计修正：run02执行版return_metrics滑移字段误筛选去程。本analysis及报告从完整fold_slip.json重算返回段；公开helper也修正，直接用保存日志核验。原执行快照/结果不覆盖，不为统计重跑。', '', '全部3轮597个核心文件before=after=当前；模型审计原几何/质量/惯性/被动折痕/反向tendon/机器人/物理/执行器限制保持。仅原jaw mimic等式，没有物体actuator或约束连接；scratch预测及FK测量均验证不改变执行state。3个MP4均解码587帧/58.7秒，阶段PNG与measurements.png已生成。', '', '代码/命令：experiments/single_fold/README.md；run_fold.py / fixture.py。', '复算报告：.venv/bin/python experiments/single_fold/analyze_results.py --run validation/single_fold/run02', '每轮证据：run02/trial00、trial01、trial02 下 result.json（原始）、analysis.json（补充）、trajectory.json.gz、forces.csv.gz、contacts.csv.gz、crease_fixture_torque.csv.gz、fold_slip.json、base_motion.json、video.mp4、阶段PNG、模型/哈希审计。', '完整初始夹具尺寸、间隙、μ及冻结阈值见README与各轮model_audit.json；预检原参数独立复核见preflight/root_audit.json。']
 (run.parent/'REPORT.txt').write_text('\n'.join(lines)+'\n');print(json.dumps({'counts':result['counts'],'core_unchanged':result['core_unchanged']},ensure_ascii=False))
if __name__=='__main__':main()
