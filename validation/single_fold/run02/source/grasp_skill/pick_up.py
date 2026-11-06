"""Independently runnable fixed-scene pick_up task, reusing the motor-only skill."""
from pathlib import Path
import argparse,hashlib,json,os,subprocess,sys
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
DEFAULT_PATH=ROOT/'validation/grasp_skill/run04/trial00/approach_path.npy'


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def model_audit(env):
    """Read-only compiled-model comparison; no simulation steps or physics writes."""
    import mujoco,numpy as np
    base=mujoco.MjModel.from_xml_path(str(ROOT/'models/scene.xml'));model=env.model
    checks={}
    for name in ['box_base','dexmate_R_gripper_l1','dexmate_R_gripper_l2']:
        checks[name+'_inertia_unchanged']=all(np.array_equal(getattr(base.body(name),a),getattr(model.body(name),a)) for a in ['mass','inertia','ipos','iquat'])
    robot_names=[base.geom(i).name for i in range(base.ngeom) if (base.geom(i).name or '').startswith(('dexmate_','jaw_pad_'))]
    checks['robot_geoms_unchanged']=all(all(np.array_equal(getattr(base.geom(n),a),getattr(model.geom(n),a)) for a in ['size','pos','quat','contype','conaffinity','friction','solref','solimp','margin']) for n in robot_names)
    checks['actuator_limits_unchanged']=np.array_equal(base.actuator_ctrlrange,model.actuator_ctrlrange)
    checks['physics_options_unchanged']=all(np.array_equal(getattr(base.opt,a),getattr(model.opt,a)) for a in ['gravity','timestep','integrator','noslip_iterations'])
    checks['only_original_two_jaw_mimic_equalities']=model.neq==2
    checks['one_free_object_joint']=int((model.jnt_type==0).sum())==1
    checks['no_object_actuator']=not any(model.actuator_trnid[i,0]==model.joint('box_free').id for i in range(model.nu))
    # All explicit panel contact pairs, including support mapped to original table.
    import xml.etree.ElementTree as ET
    from scene import scene_xml
    b=ET.parse(ROOT/'models/scene.xml').getroot();d=ET.fromstring(scene_xml(0.));pair_checks=[]
    for pair in d.find('contact').findall('pair'):
        name=pair.get('geom1');source=b.find(f"contact/pair[@geom1='{name if name!='support_top' else 'table_collision'}'][@geom2='card_base_0']")
        pair_checks.append({k:v for k,v in pair.attrib.items() if k!='geom1'}=={k:v for k,v in source.attrib.items() if k!='geom1'})
    checks['panel_contact_pairs_unchanged']=all(pair_checks)
    return {k:bool(v) for k,v in checks.items()}


def postprocess(out):
    # run.py imports a different analyze.py through legacy sys.path changes.
    # Resolve this task's analysis module explicitly, without changing physics.
    import importlib.util,imageio_ffmpeg
    spec=importlib.util.spec_from_file_location('grasp_skill_analysis',HERE/'analyze.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.analysis(out)
    result=json.loads((out/'result.json').read_text())
    frames,seconds=imageio_ffmpeg.count_frames_and_secs(str(out/'video.mp4'))
    before=json.loads((out/'core_hashes_before.json').read_text());after=json.loads((out/'core_hashes_after.json').read_text())
    artifact={'decoded_frames':frames,'video_seconds':seconds,'logged_frames':result['video_frames'],'frame_count_matches':frames==result['video_frames'],'core_unchanged':before==after,'core_hash_file_count':len(before),'warnings':result['warnings']}
    (out/'artifact_checks.json').write_text(json.dumps(artifact,indent=2));assert artifact['frame_count_matches']


def worker(out,path,trial_class=None,task="pick_up"):
    from run import Trial,hashes
    import mujoco,imageio_ffmpeg
    before=hashes()
    trial=(Trial if trial_class is None else trial_class)(out,0.,approach_path=path,task=task)
    (out/'core_hashes_before.json').write_text(json.dumps(before,indent=2))
    audit=model_audit(trial.e);(out/'model_audit.json').write_text(json.dumps(audit,indent=2))
    if not all(audit.values()):raise RuntimeError('Model invariant mismatch: '+str(audit))
    result=trial.run()
    after=hashes();(out/'core_hashes_after.json').write_text(json.dumps(after,indent=2))
    assert before==after,'Core files changed during task'
    assert mujoco.get_mjcb_control() is None,'Control callback leaked'
    postprocess(out)
    print(json.dumps({k:result[k] for k in ['pickup_success','cleanup_success','protocol_completed','outcome','failure_phase','force_peaks_N']},ensure_ascii=False),flush=True)
    return result


def aggregate(out,trial_dirs,source_path,initial_hashes):
    from run import hashes
    results=[json.loads((p/'result.json').read_text()) for p in trial_dirs]
    audits=[json.loads((p/'model_audit.json').read_text()) for p in trial_dirs]
    artifacts=[json.loads((p/'artifact_checks.json').read_text()) for p in trial_dirs]
    after=hashes();(out/'core_hashes_after.json').write_text(json.dumps(after,indent=2))
    summary={'task':'pick_up','scope':'固定支座单面板；不涉及随机偏移、折盒或原始整张纸盒任务','requested_repetitions':3,'completed_repetitions':len(results),'pickup_passed':sum(r['pickup_success'] for r in results),'cleanup_passed':sum(r['cleanup_success'] for r in results),'protocol_passed':sum(r['protocol_completed'] for r in results),'fixed_offset_y_m':0.,'same_initial_execution_state':len({r['initial_execution_state_sha256'] for r in results})==1,'same_settled_execution_state':len({r['settled_execution_state_sha256'] for r in results})==1,'same_approach_path':len({r['planner']['sha256'] for r in results})==1,'saved_ompl_path':str(source_path.relative_to(ROOT)),'saved_ompl_path_sha256':digest(source_path),'new_motion_planning_performed':False,'core_unchanged':initial_hashes==after,'core_hash_file_count':len(after),'model_invariants_all_passed':all(all(a.values()) for a in audits),'artifacts':artifacts,'trials':results,'source_sha256':{p.name:digest(p) for p in HERE.glob('*') if p.is_file()},'limits':'三次同条件重复仅验证该固定场景；不能外推到其他位置或称一般鲁棒性。5N为0.25ms监测终止阈值，不是硬力保证。'}
    if (out/'resume_notes.json').exists():summary['postprocessing_resume']=json.loads((out/'resume_notes.json').read_text())
    (out/'result.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    lines=['pick_up 固定场景任务报告','',f"夹起保持：{summary['pickup_passed']}/{len(results)}；正常清理：{summary['cleanup_passed']}/{len(results)}；完整协议：{summary['protocol_passed']}/{len(results)}。",'','条件：原300×220×4mm、26.4g base面板；支座中心(0,-.65,1.03)m，顶面1.14m。每轮新建MuJoCo环境，从原robot_config正常home开始，零偏移。','复用已通过的run04/trial00 OMPL RRTConnect路径；每轮验证起终点和密集全机器人碰撞样本，不重新规划。','夹起成功在实际抬升和保持完成时立即单独保存，再运行受控放置/张开/退手清理。清理失败不能抹掉已成立的夹起结果。','成功判据：实际COM上升>=10mm、支座力<.01N、对向真实颚组各>.05N、持续>=1s；整个抬升和保持相对位移<3mm、转角<.05rad。','监测终止：总颚法向力>5N、滑移>8mm或转角>.10rad；5N不是硬力上限，报告实际峰值和超调。','控制参数沿用此前成功版本：目标3N，闭合力矩分量.02–.30Nm、变化率.05Nm/s，原阻尼/重力补偿及原电机±.6Nm限制不变。','双新增pad接触仅诊断；原厂指体是未修改的真实承载接触，允许按两颚组判断。','']
    for i,(p,r) in enumerate(zip(trial_dirs,results)):
        hm=r.get('hold_metrics',{});slip=r['lift_slip_monitor']
        lines += [f"试验{i+1}：pickup_success={r['pickup_success']}；cleanup_success={r['cleanup_success']}；protocol_completed={r['protocol_completed']}；outcome={r['outcome']}",f"  夹起/清理/恢复峰值N：{r['force_peaks_N']}；总5N超调={r['force_threshold_overshoot_N']:.6f}N；失败阶段={r['failure_phase']}",f"  保持：{json.dumps(hm,ensure_ascii=False)}",f"  全程滑移：{json.dumps(slip,ensure_ascii=False)}",f"  warning={r['warnings']}；非预期接触峰值={r['peak_unexpected_contact_N']}N",f"  证据：{p.relative_to(ROOT)}/result.json、pickup_checkpoint.json、trajectory.json.gz、forces.csv.gz、contacts.csv.gz、video.mp4、阶段PNG。",'']
    lines += [f"初始执行状态一致={summary['same_initial_execution_state']}；静置后状态一致={summary['same_settled_execution_state']}；路径一致={summary['same_approach_path']}。",f"核心{len(after)}个文件SHA256前后一致={summary['core_unchanged']}；全部模型不变量验证通过={summary['model_invariants_all_passed']}。",summary['limits'],'先前偏移试验及其失败记录仍保留在validation/grasp_skill；本任务未复测偏移，也未修改其证据。']
    if 'postprocessing_resume' in summary:lines += ['打包后处理曾因同名analyze模块导入冲突退出；首轮物理结果已成功保存。修复为绝对路径导入，验证原技能/场景/路径/核心哈希后只补做后处理和尚未执行的两轮。首轮没有重跑，错误日志保留。']
    (out/'REPORT.txt').write_text('\n'.join(lines)+'\n')
    print(json.dumps({k:summary[k] for k in ['pickup_passed','cleanup_passed','protocol_passed','same_initial_execution_state','same_settled_execution_state','same_approach_path','core_unchanged']},ensure_ascii=False),flush=True)


def main():
    parser=argparse.ArgumentParser(description='固定场景pick_up：一次基准通过后再做两次同条件重复，独立进程运行。')
    parser.add_argument('--out',default='validation/pick_up/run01');parser.add_argument('--path',default=str(DEFAULT_PATH));parser.add_argument('--worker',action='store_true');parser.add_argument('--resume-after-first',action='store_true',help='Verify and postprocess existing completed trial00, then run only missing trials01/02');args=parser.parse_args()
    out=ROOT/args.out;path=Path(args.path).resolve()
    if args.worker:worker(out,path);return
    from run import hashes
    if args.resume_after_first:
        initial=json.loads((out/'core_hashes_before.json').read_text())
        first=out/'trial00';r=json.loads((first/'result.json').read_text())
        assert r['task']=='pick_up' and r['offset_y_m']==0 and r['pickup_success']
        assert initial==hashes()==json.loads((first/'core_hashes_after.json').read_text()),'Resume core hashes differ'
        for name in ['run.py','scene.py','probe.py']:
            assert digest(HERE/name)==digest(out/'source'/name),'Resume physical skill changed: '+name
        assert r['planner']['sha256']==digest(path),'Resume path changed'
        from scene import scene_xml
        assert (first/'diagnostic_scene.xml').read_text()==scene_xml(0.),'Resume scene changed'
        assert not (out/'trial01').exists() and not (out/'trial02').exists(),'Resume only accepts two missing trials'
        snapshots=out/'source_resume';snapshots.mkdir()
        notes={'reason':'First physics trial passed pickup and cleanup, but postprocessing imported wrong legacy analyze module. Fixed explicit file-path import; no physics rerun.','physical_skill_source_verified':['run.py','scene.py','probe.py'],'core_and_path_verified':True,'preserved_error_logs':['trial00.log','../run01.log']}
        (out/'resume_notes.json').write_text(json.dumps(notes,indent=2))
    else:
        out.mkdir(parents=True,exist_ok=False)
        initial=hashes();(out/'core_hashes_before.json').write_text(json.dumps(initial,indent=2))
        snapshots=out/'source';snapshots.mkdir()
    for p in HERE.glob('*'):
        if p.is_file():(snapshots/p.name).write_bytes(p.read_bytes())
    env=os.environ.copy();env.update(MUJOCO_GL='egl',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    def launch(index):
        trial=out/f'trial{index:02d}';log=open(out/f'trial{index:02d}.log','w');cmd=[sys.executable,str(Path(__file__).resolve()),'--worker','--out',str(trial),'--path',str(path)]
        print('START',index,'fixed offset0',flush=True)
        return trial,subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,env=env,cwd=ROOT),log
    if args.resume_after_first:
        first=out/'trial00';postprocess(first);print('RESUMED first-trial postprocessing; physics not rerun',flush=True)
    else:
        first,process,log=launch(0);rc=process.wait();log.close()
        if rc:raise RuntimeError('First worker failed; inspect '+str(out/'trial00.log'))
    dirs=[first];r=json.loads((first/'result.json').read_text());print('FIRST',json.dumps({k:r[k] for k in ['pickup_success','cleanup_success','outcome']},ensure_ascii=False),flush=True)
    if r['pickup_success']:
        # Two independent MuJoCo processes; global callbacks cannot interfere.
        remaining=[launch(i) for i in [1,2]]
        for trial,process,log in remaining:
            rc=process.wait();log.close()
            if rc:raise RuntimeError('Worker failed; inspect '+str(trial))
            dirs.append(trial)
    aggregate(out,dirs,path,initial)
if __name__=='__main__':main()
