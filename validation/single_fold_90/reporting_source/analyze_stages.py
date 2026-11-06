"""Summarize staged folding evidence, including honest zero-step precheck rejection."""
from pathlib import Path
import argparse,hashlib,json,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[2]
def read(p):return json.loads(p.read_text())
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--run',default='validation/single_fold_90/run01');args=ap.parse_args();run=ROOT/args.run;trials=[]
 for p in sorted(run.glob('trial??')):
  r=read(p/'result.json');before=read(p/'core_hashes_before.json');after=read(p/'core_hashes_after.json');current={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in before};nominal=r.get('nominal_arc_prediction');stages=[]
  for goal in r['stage_goals_deg']:
   completed=next((s for s in r['stage_results'] if s['target_deg']==goal),None);stages.append(completed or {'target_deg':goal,'status':'not_executed' if not r['physics_substeps'] else 'not_completed','actual_angle_deg':None,'hold_success':False})
  summary={'trial':str(p.relative_to(ROOT)),'stage_results':stages,'physics_substeps':r['physics_substeps'],'simulation_s':r['simulation_s'],'failure_phase':r['failure_phase'],'error':r.get('error'),'fixture_ready':r['fixture_ready'],'grasp_success':r['grasp_success'],'staged_success':r['staged_success'],'return_success':r['return_success'],'release_success':r['release_success'],'protocol_completed':r['protocol_completed'],'recovery':r.get('recovery'),'core_file_count':len(before),'core_unchanged':before==after==current,'warnings':r['warnings'],'force_measurements_available':bool(r['physics_substeps']),'hand_force_peak_N':r['peak_hand_normal_N'] if r['physics_substeps'] else None,'fixture_force_peak_N':r['peak_fixture_base_normal_N'] if r['physics_substeps'] else None,'video':read(p/'artifact_checks.json'),'planning_state_unchanged':nominal['execution_state_unchanged'] if nominal else None,'prediction_only_failure':not r['physics_substeps']}
  if nominal:
   samples=nominal['samples'];summary['last_checked_prediction']=samples[-1];summary['last_passed_prediction']=samples[-2] if not nominal['passed'] else samples[-1]
   fig,axs=plt.subplots(2,1,figsize=(9,6),layout='constrained',sharex=True);angles=[s['predicted_angle_deg'] for s in samples]
   axs[0].plot(angles,[1000*s['ik']['position_m'] for s in samples],label='Nominal scratch IK residual');axs[0].axhline(.3,color='red',linestyle='--',label='Frozen threshold');axs[0].set_ylabel('Position error (mm)');axs[0].legend()
   axs[1].plot(angles,[s['ik']['orientation_rad'] for s in samples]);axs[1].axhline(.003,color='red',linestyle='--');axs[1].set_ylabel('Orientation error (rad)');axs[1].set_xlabel('Predicted crease angle (deg) — not actual motion');fig.suptitle('Planning-only precheck: no physical execution')
   for ax in axs:ax.grid(alpha=.25)
   fig.savefig(p/'precheck_residuals.png',dpi=150);plt.close(fig)
  for key,file in [('same_seed_diagnosis','ik_convergence_diagnosis.json'),('alternative_seed_diagnosis','alternative_seed_diagnosis.json')]:
   if (p/file).exists():summary[key]=read(p/file)
  (p/'analysis.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));trials.append(summary)
 summary={'scope':'原夹具/被动折痕，同抓姿30→60→90°分阶段物理任务；先整弧预检。','trials':trials,'physical_staged_successes':sum(t['staged_success'] for t in trials),'core_unchanged':all(t['core_unchanged'] for t in trials),'global_unreachable_claim':False,'no_physics_or_threshold_tuning':True};(run/'analysis.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));(run.parent/'result.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
 lines=['30°→60°→90°夹具辅助单折痕任务：本轮停止于规划预检','', '同一原box_base + box_side_left、原被动hinge/反向tendon、原接触夹具、正常home和抓姿。握力目标3N、原GIC、2°/秒、各60×20ms保持与所有界限不变。成功角目标±2°、保持≥1秒、底板≤2mm/1°、相对滑移<3mm/.05rad；子步5N监测终止。没有物体驱动/焊接/冻结/外力。','', '预检结果：从名义0°以0.5°采样向90°检查，31.5°通过，32°位置误差.429598903mm超过冻结.300mm；姿态误差.002465680rad仍小于.003rad。拒绝点非预期接触列表为空，所有关节在原范围内。尚未检查剩余32.5–90°及返回段；程序按首个失败停止。','', '阶段 | 实际角/保持 | 结果','30° | 无物理数据 | 未执行','60° | 无物理数据 | 未执行','90° | 无物理数据 | 未执行','返回/释放 | 无物理数据 | 未执行','', '物理步骤0、模拟时间0；夹具静置、接近和闭合均未开始。初始PNG为真实初始场景。forces/contacts/crease_fixture_torque仅表头，trajectory为空，不能把0N默认字段解释成实测力或安全通过。没有动态视频；不使用静态/预测画面冒充物理执行视频。','', '有界诊断：同一失败解追加100/500/2000次scratch IK迭代，残差收敛至.361987579mm/.002078124rad，最小关节余量.373066654rad。再检查正常home及前一通过解的两个确定±扰动（j3±.2,j5∓.2rad），各最多500次；三者初始和最终均无非预期碰撞，位置误差分别.362076425/.362174956/.362037251mm，均失败。加权Jacobian最小奇异值分别2.01e-5/2.92e-5/1.50e-5，其余最小约.1396，提示当前解附近接近运动学奇异。所有诊断均只写scratch并检查live state SHA不变。','', '结论仅限于：现有连续IK分支/有限备选初值未通过同抓姿32°目标的规划容差。不是实际折叠失败，也不能证明全局不可达。没有放宽阈值、更换抓姿/夹具、改变物理或执行不安全目标，没有物理重试。后续若继续需要另行评估抓姿/动作策略，当前任务不扩展。','', '后处理问题：零物理帧时worker原先尝试解码不存在MP4而报错；源码已增加零帧分支，原日志保留，补做汇总不重跑。公开15°默认路径仍保留，其报告helper用旧保存日志核验。','', '597项核心文件before=after=当前；原模型惯性/几何/被动折痕/物理/执行器模型审计保留。未触碰基线、rax或slides。','', '可运行命令/完整界限：experiments/single_fold/README_90.md','源码：experiments/single_fold/run_fold.py；diagnose_ik.py（只读有限诊断）；analyze_stages.py（只读汇总）。','证据：validation/single_fold_90/run01/trial00/{result.json,analysis.json,nominal_arc_prediction.json,ik_convergence_diagnosis.json,alternative_seed_diagnosis.json,model_audit.json,core_hashes_before.json,core_hashes_after.json,00_initial.png,precheck_residuals.png}。','执行源码快照：run01/source；后处理修正及报告源码：reporting_source。']
 (run.parent/'REPORT.txt').write_text('\n'.join(lines)+'\n');print(json.dumps({'physical_staged_successes':summary['physical_staged_successes'],'core_unchanged':summary['core_unchanged'],'physics_substeps':[t['physics_substeps'] for t in trials]}))
if __name__=='__main__':main()
