"""One bounded next checkpoint: +world-Y10mm transport of the physically held panel."""
from pathlib import Path
import argparse,json,os,subprocess,sys,hashlib
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
from run import Trial,hashes,robot_name
from pick_up import worker as common_worker,DEFAULT_PATH,digest

NEW_PHASES={'transport_out','transport_hold','transport_return','return_hold'}
class TransportTrial(Trial):
    force_feedback_phases=Trial.force_feedback_phases|NEW_PHASES
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.transport_reference=None;self.transport_rows=[]
        self.report.update(transport_success=False,return_success=False,
            transport_spec={'world_axis':'+Y','displacement_m':.010,'duration_s':5.,'hold_steps':60,'return_duration_s':5.,'off_axis_bound_m':.002,'success_y_bounds_m':[.009,.011],'relative_translation_bound_m':.003,'relative_rotation_bound_rad':.05,'online_abort_translation_m':.008,'online_abort_rotation_rad':.10,'return_position_error_bound_m':.002,'force_abort_N':5.,'reference':'actual held panel COM and object/tool relation at END of pickup hold; prior lift deflection excluded'})
    def after_tick(self):
        if self.phase not in NEW_PHASES or self.transport_reference is None:return
        state=self.rows[-1];ref=self.transport_reference
        relative=float(np.linalg.norm(np.array(state['relative_position'])-ref['relative_position']))
        angle=float(np.linalg.norm(Rotation.from_matrix(np.array(state['relative_rotation'])@np.array(ref['relative_rotation']).T).as_rotvec()))
        row={'time_s':state['t'],'phase':self.phase,'com_delta_m':(np.array(state['com'])-ref['com']).tolist(),'tcp_delta_m':(np.array(state['tcp'][:3])-ref['tcp'][:3]).tolist(),'relative_translation_m':relative,'relative_rotation_rad':angle,'jaw_forces_N':state['jaw_forces_N'],'support_force_N':state['support_force_N']}
        self.transport_rows.append(row)
        if relative>.008 or angle>.10:raise RuntimeError(f'Transport slip abort: {relative:.6f}m/{angle:.6f}rad')
    def observe(self,m,d,*args,**kwargs):
        try:super().observe(m,d,*args,**kwargs)
        except RuntimeError:
            # Exact failed substep evidence, including all intended contact pairs.
            contacts=[]
            for i,c in enumerate(d.contact):
                force=np.zeros(6);mujoco.mj_contactForce(m,d,i,force)
                contacts.append({'geoms':[self.names[int(c.geom1)],self.names[int(c.geom2)]],'force_local':force.tolist(),'distance_m':float(c.dist),'position_m':c.pos.tolist()})
            (self.out/'abort_substep_contacts.json').write_text(json.dumps({'time_s':float(d.time),'phase':self.phase,'contacts':contacts},indent=2))
            np.savez(self.out/'abort_state.npz',qpos=d.qpos.copy(),qvel=d.qvel.copy(),ctrl=d.ctrl.copy(),time=float(d.time))
            raise
    def scratch_check(self,lift):
        """Predicted attachment transform ONLY in scratch; actual object stays free."""
        state_before=hashlib.sha256(self.d.qpos.tobytes()+self.d.qvel.tobytes()+self.d.ctrl.tobytes()).hexdigest();time_before=float(self.d.time)
        e=self.e;model=self.m;scratch=mujoco.MjData(model);site=e.arms['right']['site'];arm=e.arms['right'];board=self.d.body('box_base')
        rotation=self.d.site_xmat[site].reshape(3,3).copy();position=self.d.site_xpos[site].copy()
        relative_position=rotation.T@(board.xpos-position);relative_rotation=rotation.T@board.xmat.reshape(3,3)
        seed=self.d.qpos[arm['qad']].copy();samples=[]
        for shift in np.r_[np.linspace(0,.010,11),np.linspace(.009,0,10)]:
            target=lift.copy();target[1]+=shift
            q,error=e.inverse_kinematics('right',target,iterations=100,seed=seed);seed=q
            if error['position_m']>.0002 or error['orientation_rad']>.002:
                (self.out/'transport_scratch_check.json').write_text(json.dumps({'prediction_only':True,'status':'IK rejected','completed_samples':samples,'failed_target':target.tolist(),'error':error},indent=2))
                raise RuntimeError('Transport scratch IK failed '+str(error))
            scratch.qpos[:]=self.d.qpos;scratch.qpos[arm['qad']]=q;mujoco.mj_forward(model,scratch)
            rt=scratch.site_xmat[site].reshape(3,3);pt=scratch.site_xpos[site];predicted_position=pt+rt@relative_position;predicted_rotation=rt@relative_rotation
            scratch.joint('box_free').qpos[:3]=predicted_position;mujoco.mju_mat2Quat(scratch.joint('box_free').qpos[3:],predicted_rotation.ravel());mujoco.mj_forward(model,scratch)
            intended_count=0;rejected=[]
            for contact in scratch.contact:
                ids=[int(contact.geom1),int(contact.geom2)];names=[self.names[g] for g in ids]
                intended=self.card in ids and any(any(g in group for g in ids) for group in self.groups)
                if intended:
                    intended_count+=1
                    if contact.dist<-.001:rejected.append({'geoms':names,'distance_m':float(contact.dist),'reason':'intended penetration >1mm'})
                elif (self.card in ids or any(robot_name(n) for n in names)) and contact.dist< -1e-6:rejected.append({'geoms':names,'distance_m':float(contact.dist)})
            sample={'world_y_shift_m':float(shift),'ik_error':error,'predicted_tcp_m':pt.tolist(),'predicted_panel_body_position_m':predicted_position.tolist(),'intended_contacts_allowed':intended_count,'rejected':rejected};samples.append(sample)
            if rejected:
                (self.out/'transport_scratch_check.json').write_text(json.dumps({'prediction_only':True,'status':'collision rejected','samples':samples},indent=2))
                raise RuntimeError('Transport scratch collision rejected '+json.dumps(sample))
        after=hashlib.sha256(self.d.qpos.tobytes()+self.d.qvel.tobytes()+self.d.ctrl.tobytes()).hexdigest()
        check={'prediction_only':True,'attachment_in_execution':False,'sample_spacing_m':.001,'samples':samples,'execution_state_unchanged':state_before==after and time_before==self.d.time,'live_state_sha256_before':state_before,'live_state_sha256_after':after}
        (self.out/'transport_scratch_check.json').write_text(json.dumps(check,indent=2));self.report['transport_scratch_check']=check
        assert check['execution_state_unchanged']
    def metrics(self,phases,hold_phase):
        rows=[r for r in self.transport_rows if r['phase'] in phases];hold=[r for r in rows if r['phase']==hold_phase]
        delta=np.array([r['com_delta_m'] for r in hold]);all_delta=np.array([r['com_delta_m'] for r in rows]);tcp=np.array([r['tcp_delta_m'] for r in hold])
        return {'hold_duration_s':hold[-1]['time_s']-hold[0]['time_s'],'hold_y_min_m':float(delta[:,1].min()),'hold_y_max_m':float(delta[:,1].max()),'hold_final_com_delta_m':delta[-1].tolist(),'hold_final_tcp_delta_m':tcp[-1].tolist(),'max_abs_x_drift_m':float(abs(all_delta[:,0]).max()),'max_abs_z_drift_m':float(abs(all_delta[:,2]).max()),'max_relative_translation_m':max(r['relative_translation_m'] for r in rows),'max_relative_rotation_rad':max(r['relative_rotation_rad'] for r in rows),'max_support_N':max(r['support_force_N'] for r in rows),'min_loaded_jaw_N':min(min(r['jaw_forces_N']) for r in rows),'hold_max_return_error_m':float(np.linalg.norm(delta,axis=1).max()),'sample_count':len(rows)}
    def retained(self,metrics):
        return bool(metrics['hold_duration_s']>=1 and metrics['max_support_N']<.01 and metrics['min_loaded_jaw_N']>.05 and metrics['max_relative_translation_m']<.003 and metrics['max_relative_rotation_rad']<.05 and metrics['max_abs_x_drift_m']<=.002 and metrics['max_abs_z_drift_m']<=.002)
    def after_pickup(self,target,pre,lift):
        self.phase='transport_precheck';self.transport_reference=self.state();self.report['transport_reference']=self.transport_reference
        self.scratch_check(lift);self.image('06_transport_start')
        outward=lift.copy();outward[1]+=.010
        self.phase='transport_out';print('TRANSPORT +Y10mm',flush=True);self.move(outward,5.)
        self.phase='transport_hold'
        for _ in range(60):self.tick(outward)
        self.image('07_transported');metrics=self.metrics({'transport_out','transport_hold'},'transport_hold');self.report['transport_metrics']=metrics
        self.report['transport_success']=bool(self.retained(metrics) and metrics['hold_y_min_m']>=.009 and metrics['hold_y_max_m']<=.011)
        if not self.report['transport_success']:raise RuntimeError('Transport measured criteria failed '+json.dumps(metrics))
        self.report['transport_completed_at_s']=float(self.d.time);(self.out/'transport_checkpoint.json').write_text(json.dumps({'transport_success':True,'time_s':float(self.d.time),'metrics':metrics},indent=2))
        print('TRANSPORT_SUCCESS '+str(self.out),flush=True)
        self.phase='transport_return';self.move(lift,5.)
        self.phase='return_hold'
        for _ in range(60):self.tick(lift)
        self.image('08_returned');metrics=self.metrics({'transport_return','return_hold'},'return_hold');self.report['return_metrics']=metrics
        self.report['return_success']=bool(self.retained(metrics) and metrics['hold_max_return_error_m']<=.002)
        if not self.report['return_success']:raise RuntimeError('Return measured criteria failed '+json.dumps(metrics))
        self.report['return_completed_at_s']=float(self.d.time);(self.out/'return_checkpoint.json').write_text(json.dumps({'return_success':True,'time_s':float(self.d.time),'metrics':metrics},indent=2))
        print('RETURN_SUCCESS '+str(self.out),flush=True)
    def finalize_task_report(self):
        self.report['force_peaks_N']['transport']=max((self.peaks.get(p,0.) for p in ['transport_out','transport_hold']),default=0.)
        self.report['force_peaks_N']['return']=max((self.peaks.get(p,0.) for p in ['transport_return','return_hold']),default=0.)
        self.report['protocol_completed']=all(self.report[k] for k in ['pickup_success','transport_success','return_success','cleanup_success']);self.report['success']=self.report['protocol_completed']
        self.report['outcome']='all_stages_succeeded' if self.report['protocol_completed'] else next(k+'_failed' for k in ['pickup','transport','return','cleanup'] if not self.report[k+'_success'])
        (self.out/'transport_trace.json').write_text(json.dumps(self.transport_rows,indent=2))


def plot(out):
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    rows=json.loads((out/'transport_trace.json').read_text())
    if not rows:return
    t=np.array([r['time_s'] for r in rows]);delta=np.array([r['com_delta_m'] for r in rows])*1000
    fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
    for i,name in enumerate(['X','Y','Z']):axes[0].plot(t,delta[:,i],label=name)
    axes[0].set_ylabel('Actual COM delta (mm)');axes[0].legend();axes[0].axhline(10,ls='--',color='gray')
    axes[1].plot(t,[r['relative_translation_m']*1000 for r in rows],label='Translation');axes[1].axhline(3,ls='--',color='red');axes[1].set_ylabel('Relative slip (mm)')
    axes[2].plot(t,[sum(r['jaw_forces_N']) for r in rows],label='Jaw normal sum');axes[2].plot(t,[r['support_force_N'] for r in rows],label='Support');axes[2].axhline(5,ls='--',color='red');axes[2].set_ylabel('Force (N), 20ms samples');axes[2].set_xlabel('Simulation time (s)');axes[2].legend()
    for ax in axes:ax.grid(alpha=.25)
    fig.suptitle('Fixed panel +Y10mm transport and return; actual measurements');fig.tight_layout();fig.savefig(out/'transport_measurements.png',dpi=150);plt.close(fig)


def aggregate(out,dirs,path,before):
    reports=[json.loads((p/'result.json').read_text()) for p in dirs];after=hashes();(out/'core_hashes_after.json').write_text(json.dumps(after,indent=2))
    summary={'task':'transport_10mm','scope':'固定场景，+世界Y10mm及返回，无旋转/随机偏移/折叠','requested_repetitions':3,'completed_repetitions':len(reports),'passed':{key:sum(r[key] for r in reports) for key in ['pickup_success','transport_success','return_success','cleanup_success','protocol_completed']},'trials':reports,'same_initial_state':len({r['initial_execution_state_sha256'] for r in reports})==1,'same_settled_state':len({r['settled_execution_state_sha256'] for r in reports})==1,'saved_ompl_path':str(path.relative_to(ROOT)),'path_sha256':digest(path),'core_unchanged':before==after,'core_hash_file_count':len(before),'source_sha256':{p.name:digest(p) for p in HERE.glob('*') if p.is_file()},'limits':'固定场景有界测试，不能外推一般鲁棒性；5N是监测终止阈值，不是硬力上限。'}
    (out/'result.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
    lines=['10mm水平搬移任务报告','',str(summary['passed']),'每轮正常home与相同零偏移面板/支座，复用并重新验证已通过的OMPL接近路径。','夹起成功后，在当前阻抗控制内沿世界+Y命令10mm/5s，保持60个20ms步，再返回10mm/5s，保持后正常放置/张开/退手。','成功阈值在首轮前固定：实际COM的Y位移9–11mm，保持>=1s，无支撑且两颚承载；X/Z各漂移<=2mm；全搬移相对位姿变化<3mm/.05rad。返回COM误差<=2mm。','搬移参考为pickup保持结束时真实COM及物体/工具相对位姿，不把前段夹起的2.6mm偏移重新计为搬移滑移。','原3N力反馈覆盖所有新阶段，参数、几何、材料及5N/8mm/.10rad在线终止阈值均未改变。','scratch路径用当前物体/手关系预测携物位姿，1mm采样检查IK与碰撞；这种预测只发生在scratch，不是执行中的附件约束。','']
    for i,r in enumerate(reports):
        lines += [f'试验{i+1}: '+str({k:r[k] for k in summary['passed']}),'搬移: '+json.dumps(r.get('transport_metrics'),ensure_ascii=False),'返回: '+json.dumps(r.get('return_metrics'),ensure_ascii=False),'峰值N: '+str(r['force_peaks_N'])+f"；5N超调{r['force_threshold_overshoot_N']}N；warning{r['warnings']}",'失败阶段: '+str(r.get('failure_phase'))+'；错误: '+str(r.get('error')),'']
    lines += [f"核心{len(before)}文件前后哈希一致={summary['core_unchanged']}。",'每轮的pickup/transport/return检查点、raw trajectory/forces/contact日志、transport_trace、阶段PNG和MP4均在trial目录；模型审计和视频解码检验沿用独立任务工具。',summary['limits']]
    (out/'REPORT.txt').write_text('\n'.join(lines)+'\n');print(summary['passed'],flush=True)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',default='validation/transport/run01');parser.add_argument('--worker',action='store_true');args=parser.parse_args();out=ROOT/args.out
    if args.worker:
        common_worker(out,DEFAULT_PATH,trial_class=TransportTrial,task='transport_10mm');plot(out);return
    out.mkdir(parents=True,exist_ok=False);before=hashes();(out/'core_hashes_before.json').write_text(json.dumps(before,indent=2));source=out/'source';source.mkdir()
    for p in HERE.glob('*'):
        if p.is_file():(source/p.name).write_bytes(p.read_bytes())
    env=os.environ.copy();env.update(MUJOCO_GL='egl',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    def launch(i):
        folder=out/f'trial{i:02d}';log=open(out/f'trial{i:02d}.log','w');process=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--worker','--out',str(folder)],stdout=log,stderr=subprocess.STDOUT,env=env,cwd=ROOT);print('START',i,flush=True);return folder,process,log
    first,process,log=launch(0);code=process.wait();log.close()
    if code:raise RuntimeError('Worker failed; inspect '+str(first))
    dirs=[first];r=json.loads((first/'result.json').read_text());print('FIRST',r['outcome'],flush=True)
    if r['transport_success']:
        for folder,process,log in [launch(1),launch(2)]:
            code=process.wait();log.close()
            if code:raise RuntimeError('Worker failed; inspect '+str(folder))
            dirs.append(folder)
    aggregate(out,dirs,DEFAULT_PATH,before)
if __name__=='__main__':main()
