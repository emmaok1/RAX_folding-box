"""Motor-only staged grasp diagnostic; no state writes after initialization."""
from scene import *
from probe import candidate
import csv, gzip, hashlib, time, traceback, argparse
from PIL import Image
import imageio.v2 as imageio
sys.path.insert(0,str(ROOT/'experiments/vega_impedance'))
sys.path.insert(0,str(ROOT/'validation/planner_comparison/ompl'))
from adapter import RightArmImpedance
from test_support import Validity,dense,robot_name
from planner import OMPLPlanner

CORE=['environment.py','physics.json','robot_config.json','robot_model.py','models/scene.xml','video_rebuild/config.json']
def hashes():
    paths=[ROOT/p for p in CORE if (ROOT/p).exists()]
    for folder in ['models','video_rebuild','rax']:
        paths.extend(p for p in (ROOT/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    return {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(paths))}

class Trial:
    def __init__(self,out,offset,approach_path=None,task="grasp_diagnostic"):
        self.saved_approach_path=None if approach_path is None else Path(approach_path)
        self.out=out;out.mkdir(parents=True,exist_ok=False);self.e=DiagnosticEnv(offset);self.m=self.e.model;self.d=self.e.data
        (out/'diagnostic_scene.xml').write_text(scene_xml(offset))
        self.phase='home_settle';self.step_count=0;self.rows=[];self.force_rows=[];self.contacts=[];self.peaks={};self.original_step=mujoco.mj_step
        self.names=[self.m.geom(i).name or '' for i in range(self.m.ngeom)];self.card=self.m.geom('card_base_0').id
        self.groups=[{self.m.geom(n).id for n in [f'jaw_pad_R_{i}',f'dexmate_R_gripper_l{i}_collision_'+str(118 if i==1 else 121)]} for i in [1,2]]
        self.force=np.zeros(2);self.pad_force=np.zeros(2);self.support=0.;self.lift_reference=None;self.lift_drift=[];self.bad_peak_N=0.;self.bad_total_peak_N=0.;self.last_trip=None;self.frames=0
        self.writer=imageio.get_writer(out/'video.mp4',fps=10,codec='libx264',quality=7)
        self.report={'task':task,'pickup_success':False,'cleanup_success':False,'protocol_completed':False,'command_sequence_completed':False,'failure_phase':None,'simplified_scene':True,'original_whole_box_solved':False,'offset_y_m':offset,'success':False,'opposing_contact':False,'lift_10mm':False,'hold_1s':False,'released_supported':False,'force_abort_threshold_N':5.,'force_definition':'sum of nonnegative normal forces across both right jaw/card groups; sampled every 0.25ms; threshold is NOT a hard cap','panel':'unchanged original base panel','dimensions_m':[.3,.22,.004],'mass_kg':float(self.m.body('box_base').mass[0]),'stand_size_m':[.13,.15,.22],'stand_top_z_m':1.14,'stand_center_m':[0,-.65,1.03],'commanded_lift_m':.025,'home_right_rad':self.e.cfg['home_right_rad'],'controller':'baseline joint PD for OMPL; existing fixed RightArmImpedance for grasp/lift/place','closure':'nonzero position reference at .05 rad/s down to .04, .006 rad/s to .002; then bounded motor torque feedback target3N, max0.30Nm; feedback continues through lift/hold, torque slew0.05Nm/s','checks':{}}
    def observe(self,m,d,*a,**kw):
        self.original_step(m,d,*a,**kw);self.step_count+=1;self.force[:]=0.;self.pad_force[:]=0.;self.support=0.;pen=0.;bad=[]
        for i,c in enumerate(d.contact):
            ids=[int(c.geom1),int(c.geom2)];names=[self.names[g] for g in ids]
            f=np.zeros(6);mujoco.mj_contactForce(m,d,i,f);fn=max(0.,float(f[0]))
            intended=self.card in ids and any(any(g in group for g in ids) for group in self.groups)
            if intended:
                for k in range(2):
                    if f'jaw_pad_R_{k+1}' in names:self.pad_force[k]+=fn
                for k,group in enumerate(self.groups):
                    if any(g in group for g in ids):self.force[k]+=fn
                pen=max(pen,-float(c.dist))
                if fn>.005 and self.step_count%80==0:self.contacts.append([float(d.time),self.phase,*names,fn,float(c.dist),*c.pos.tolist(),*c.frame[:3].tolist()])
            elif self.card in ids and 'support_top' in names:self.support+=fn
            elif any(robot_name(n) for n in names) and (fn>.05 or c.dist<-.0001):bad.append({'geoms':names,'force_N':fn,'dist_m':float(c.dist)})
        self.bad_peak_N=max(self.bad_peak_N,max((x['force_N'] for x in bad),default=0.));self.bad_total_peak_N=max(self.bad_total_peak_N,sum(x['force_N'] for x in bad))
        total=float(self.force.sum());self.peaks[self.phase]=max(self.peaks.get(self.phase,0.),total)
        self.force_rows.append([float(d.time),self.phase,*self.force,total,self.support,pen,*self.pad_force])
        if total>5 or pen>.001 or bad or d.warning.number.any() or not np.isfinite(d.qpos).all():
            self.last_trip={'time_s':float(d.time),'phase':self.phase,'sum_normal_N':total,'penetration_m':pen,'unexpected_contacts':bad,'warning':d.warning.number.tolist()}
            raise RuntimeError('Monitored abort '+json.dumps(self.last_trip))
    def state(self):
        e=self.e;R=self.d.site_xmat[e.arms['right']['site']].reshape(3,3);p=e.tcp_pose('right')[:3];b=self.d.body('box_base')
        return {'t':float(self.d.time),'phase':self.phase,'tcp':e.tcp_pose('right').tolist(),'target':e.targets['right'].tolist(),'com':b.xipos.tolist(),'object_quat':self.d.joint('box_free').qpos[3:].tolist(),'relative_position':((b.xipos-p)@R).tolist(),'relative_rotation':(R.T@b.xmat.reshape(3,3)).tolist(),'q':self.d.qpos[e.arms['right']['qad']].tolist(),'gripper_q':self.d.qpos[e.grippers['right']['qad']].tolist(),'gripper_command':e.grippers['right']['command'],'effort_scale':e.grippers['right']['effort_scale'],'jaw_forces_N':self.force.tolist(),'pad_forces_N':self.pad_force.tolist(),'support_force_N':self.support}
    def image(self,name):Image.fromarray(self.e.render_camera('diagnostic')).save(self.out/(name+'.png'))
    def tick(self,target=None,q=None,opening=None):
        e=self.e
        if self.phase in ('close_force','grasp_settle','lift','hold','place'):
            g=e.grippers['right'];torque=.4*g['effort_scale'];change=float(np.clip(.002*(3.-self.force.sum()),-.001,.001));g['effort_scale']=float(np.clip(torque+change,.02,.30)/.4)
        if q is not None:e.step_joint_targets({'right':q},None if opening is None else {'right':opening})
        else:
            if target is not None:e.targets['right']=target.copy()
            if opening is not None:e.grippers['right']['command']=opening
            e.step()
        self.rows.append(self.state())
        if self.phase in ('lift','hold') and self.lift_reference is not None:
            from scipy.spatial.transform import Rotation
            st=self.rows[-1];ref=self.lift_reference;dp=float(np.linalg.norm(np.array(st['relative_position'])-ref['relative_position']));angle=float(np.linalg.norm(Rotation.from_matrix(np.array(st['relative_rotation'])@np.array(ref['relative_rotation']).T).as_rotvec()))
            self.lift_drift.append({'time_s':st['t'],'phase':self.phase,'translation_m':dp,'angle_rad':angle})
            if dp>.008 or angle>.10:raise RuntimeError(f'Lift slip/rotation abort: translation={dp:.6f}m angle={angle:.6f}rad')
        if e.env_step%5==0:
            self.writer.append_data(e.render_camera('diagnostic'));self.frames+=1
    def move(self,target,seconds):
        start=self.e.targets['right'].copy();n=round(seconds/self.e.control_dt)
        for i in range(1,n+1):self.tick(start+(target-start)*i/n)
    def run(self):
        mujoco.mj_step=self.observe;start=time.monotonic();controller=None
        try:
            self.report['checks']['home_collision_free']=bool(Validity(self.e)(self.e.arms['right']['qcmd']))
            if not self.report['checks']['home_collision_free']:raise RuntimeError('Invalid initial robot/support/panel collision; rejected before dynamics')
            self.report['initial_execution_state_sha256']=hashlib.sha256(self.d.qpos.tobytes()+self.d.qvel.tobytes()+self.d.ctrl.tobytes()).hexdigest()
            self.image('00_home')
            for _ in range(25):self.tick(q=self.e.arms['right']['qcmd'].copy())
            target,pre,qpre,qgrasp,search=candidate(self.e);self.report['candidate_search']=search;self.report['grasp_target']=target.tolist()
            self.report['initial_state']=self.state()
            valid=Validity(self.e)
            if self.saved_approach_path is None:
                planner=OMPLPlanner(self.e.arms['right']['limits'],self.out,solve_seconds=5.)
                path,meta=planner(self.d.qpos[self.e.arms['right']['qad']].copy(),qpre,valid,'approach')
                if path is None:raise RuntimeError('OMPL no exact path')
            else:
                path=np.load(self.saved_approach_path,allow_pickle=False)
                if not np.allclose(path[0],self.d.qpos[self.e.arms['right']['qad']],atol=1e-6,rtol=0):raise RuntimeError('Saved OMPL path start differs from settled home')
                if not np.allclose(path[-1],qpre,atol=1e-6,rtol=0):raise RuntimeError('Saved OMPL path goal differs from recomputed candidate')
                meta={'planner':'OMPL RRTConnect','version':'2.0.1','mode':'reuse fixed previously verified path, freshly revalidate; no replanning','source':str(self.saved_approach_path.relative_to(ROOT)),'sha256':hashlib.sha256(self.saved_approach_path.read_bytes()).hexdigest(),'original_metadata':json.loads(self.saved_approach_path.with_name('approach_planner.json').read_text())}
                np.save(self.out/'approach_path.npy',path)
            self.report['planner']=meta
            self.report['settled_execution_state_sha256']=hashlib.sha256(self.d.qpos.tobytes()+self.d.qvel.tobytes()+self.d.ctrl.tobytes()).hexdigest()
            samples=list(dense(path));assert all(valid(q) for q in samples);self.report['checks']['dense_path_samples']=len(samples)
            # Independently validate insertion in configuration space with full mesh collision.
            insertion=list(dense([qpre,qgrasp],.003));assert all(valid(q) for q in insertion)
            self.report['checks']['insertion_samples']=len(insertion)
            self.phase='ompl_approach';print('OMPL approach',flush=True)
            for q in dense(path,.2*self.e.control_dt):self.tick(q=q)
            for _ in range(50):self.tick(q=qpre)
            self.image('01_pregrasp')
            # Existing fixed controller; no directional gain adaptation.
            with RightArmImpedance(self.e) as controller:
                self.e.targets['right']=self.e.tcp_pose('right').copy()
                self.phase='insert';self.move(target,5.)
                for _ in range(50):self.tick(target)
                self.image('02_inserted');self.report['inserted']=self.state();print('Closing',flush=True)
                self.phase='close_position';g=self.e.grippers['right'];ref=g['qcmd'];t0=self.d.time
                for _ in range(650):
                    speed=.05 if ref>.04 else .006;ref=max(.00192,ref-speed*self.e.control_dt)
                    self.tick(target,opening=ref/.96)
                    if self.force.sum()>=1.5 or ref<=.00192:break
                self.phase='close_force';g['effort_scale']=.06/.4
                # Existing motor torque mode, modest feedback on measured contact sum.
                for _ in range(300):
                    self.tick(target,opening=0.)
                    if np.all(self.force>.05) and 2.8<=self.force.sum()<=3.2:break
                self.report['closed']=self.state();self.report['opposing_contact']=bool(np.all(self.force>.05));self.image('03_closed')
                if not self.report['opposing_contact']:raise RuntimeError('No opposing loaded jaw groups after bounded closure')
                self.phase='grasp_settle'
                for _ in range(25):self.tick(target)
                prelift=self.state();self.report['prelift']=prelift;self.lift_reference=prelift
                self.phase='lift';lift=target.copy();lift[2]+=.025;print('Lift',flush=True);self.move(lift,5.)
                self.report['after_lift']=self.state();self.image('04_lifted')
                self.phase='hold';hold=[]
                for _ in range(60):
                    self.tick(lift);hold.append(self.state())
                    if self.d.body('box_base').xipos[2]<prelift['com'][2]-.005:raise RuntimeError('Panel dropping below prelift COM')
                heights=np.array([s['com'][2]-prelift['com'][2] for s in hold]);rp=np.array([s['relative_position'] for s in hold]);rot0=np.array(hold[0]['relative_rotation']);angles=[np.linalg.norm(__import__('scipy').spatial.transform.Rotation.from_matrix(np.array(s['relative_rotation'])@rot0.T).as_rotvec()) for s in hold]
                self.report['hold_metrics']={'duration_s':hold[-1]['t']-hold[0]['t'],'min_com_lift_m':float(heights.min()),'max_com_lift_m':float(heights.max()),'relative_translation_drift_m':float(np.linalg.norm(rp-rp[0],axis=1).max()),'relative_angle_drift_rad':float(max(angles)),'opposing_pads_all_samples':all(min(s['pad_forces_N'])>.05 for s in hold),'opposing_all_samples':all(min(s['jaw_forces_N'])>.05 for s in hold),'max_support_N':max(s['support_force_N'] for s in hold)}
                hm=self.report['hold_metrics'];self.report['lift_10mm']=hm['min_com_lift_m']>=.010
                self.report['hold_1s']=bool(self.report['lift_10mm'] and hm['duration_s']>=1 and hm['relative_translation_drift_m']<.003 and hm['relative_angle_drift_rad']<.05 and hm['opposing_all_samples'] and hm['max_support_N']<.01 and max(x['translation_m'] for x in self.lift_drift)<.003 and max(x['angle_rad'] for x in self.lift_drift)<.05)
                self.report['physical_hold_1s']=self.report['hold_1s']
                if not self.report['hold_1s']:raise RuntimeError('Retention failed '+json.dumps(hm))
                # Pickup is finalized BEFORE cleanup; cleanup failure cannot erase it.
                self.report['pickup_success']=True;self.report['pickup_completed_at_s']=float(self.d.time);self.report['pickup_final_state']=self.state()
                (self.out/'pickup_checkpoint.json').write_text(json.dumps({'pickup_success':True,'time_s':float(self.d.time),'hold_metrics':hm},indent=2)+'\n')
                print('PICKUP_SUCCESS '+str(self.out),flush=True)
                self.phase='place';self.move(target,5.)
                for _ in range(25):self.tick(target)
                self.phase='release';g['effort_scale']=1.
                for i in range(1,151):self.tick(target,opening=.3*i/150)
                self.phase='retreat';self.move(pre,5.)
                self.phase='released_settle'
                for _ in range(60):self.tick(pre)
                self.report['final']=self.state();last=self.rows[-50:]
                self.report['released_supported']=bool(all(sum(s['jaw_forces_N'])<.01 and s['support_force_N']>.1 for s in last) and np.linalg.norm(np.array(last[-1]['com'])-last[0]['com'])<.001)
                self.image('05_released');self.report['command_sequence_completed']=True;self.report['cleanup_success']=self.report['released_supported'];self.report['protocol_completed']=self.report['pickup_success'] and self.report['cleanup_success'];self.report['success']=self.report['protocol_completed']
                if not self.report['cleanup_success']:self.report['failure_phase']='released_settle'
        except Exception as exc:
            self.report['failure_phase']=self.phase;self.report['error']=repr(exc);self.report['traceback']=traceback.format_exc();self.report['stopped']=self.state();self.report['abort']=self.last_trip
            # Safest reachable response: open in place, then reverse insertion only
            # if panel remains on support. Never teleport object/robot to reset.
            self.report['recovery']={'attempted':False,'complete':False}
            try:
                self.image('failure');self.phase='abort_open';self.e.targets['right']=self.e.tcp_pose('right').copy();self.e.grippers['right']['effort_scale']=1.
                self.report['recovery']['attempted']=True
                for i in range(100):self.tick(opening=min(.3,self.e.grippers['right']['command']+.003))
                if self.support>.1 and 'pre' in locals():
                    self.phase='abort_retract';self.move(pre,4.);self.report['recovery']['complete']=True
                self.report['recovery']['final']=self.state();self.image('recovered')
            except Exception as rec:self.report['recovery']['error']=repr(rec)
        finally:
            mujoco.mj_step=self.original_step;self.writer.close()
            if controller:self.report['controller_metadata']=controller.metadata()
            self.report.update(wall_seconds=time.monotonic()-start,simulation_s=float(self.d.time),physics_substeps=self.step_count,phase_peak_sum_normal_N=self.peaks,peak_sum_normal_N=max(self.peaks.values(),default=0.),video_frames=self.frames)
            self.report['lift_slip_monitor']={'abort_translation_m':.008,'abort_angle_rad':.10,'success_translation_m':.003,'success_angle_rad':.05,'max_translation_m':max((x['translation_m'] for x in self.lift_drift),default=0.),'max_angle_rad':max((x['angle_rad'] for x in self.lift_drift),default=0.)};self.report['peak_unexpected_contact_N']=self.bad_peak_N;self.report['peak_sum_unexpected_contacts_N']=self.bad_total_peak_N
            pickup_phases={'home_settle','ompl_approach','insert','close_position','close_force','grasp_settle','lift','hold'}
            cleanup_phases={'place','release','retreat','released_settle'}
            self.report['force_peaks_N']={label:max((v for p,v in self.peaks.items() if p in phases),default=0.) for label,phases in [('pickup',pickup_phases),('cleanup',cleanup_phases),('recovery',{'abort_open','abort_retract'})]}
            self.report['warnings']=self.d.warning.number.tolist()
            self.report['outcome']='pickup_and_cleanup_succeeded' if self.report['protocol_completed'] else ('pickup_succeeded_cleanup_failed' if self.report['pickup_success'] else 'pickup_failed')
            self.report['force_threshold_overshoot_N']=max(0.,self.report['peak_sum_normal_N']-5.)
            for name,header,rows in [('forces',['time_s','phase','jaw1_N','jaw2_N','sum_N','support_N','penetration_m','pad1_N','pad2_N'],self.force_rows),('contacts',['time_s','phase','geom1','geom2','normal_N','dist_m','x','y','z','nx','ny','nz'],self.contacts)]:
                with gzip.open(self.out/(name+'.csv.gz'),'wt') as f:w=csv.writer(f);w.writerow(header);w.writerows(rows)
            (self.out/'lift_drift.json').write_text(json.dumps(self.lift_drift,indent=2))
            with gzip.open(self.out/'trajectory.json.gz','wt') as f:json.dump(self.rows,f)
            (self.out/'result.json').write_text(json.dumps(self.report,indent=2)+'\n');self.e.close()
        return self.report

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',default='validation/grasp_skill/run01');ap.add_argument('--offsets',nargs='+',type=float,default=[0.,.002,-.002]);a=ap.parse_args();out=ROOT/a.out;out.mkdir(parents=True,exist_ok=False)
    for p in Path(__file__).parent.glob('*.py'):(out/p.name).write_bytes(p.read_bytes())
    initial=hashes();(out/'core_hashes_before.json').write_text(json.dumps(initial,indent=2))
    reports=[]
    for i,offset in enumerate(a.offsets):
        print('TRIAL',i,offset,flush=True);r=Trial(out/f'trial{i:02d}',offset).run();reports.append(r);print(json.dumps({k:r.get(k) for k in ['success','error','peak_sum_normal_N','hold_metrics','wall_seconds']},ensure_ascii=False),flush=True)
        if i==0 and not r['success']:break
    after=hashes();(out/'core_hashes_after.json').write_text(json.dumps(after,indent=2));summary={'simplified_scene':True,'trials':reports,'core_unchanged':initial==after,'hash_file_count':len(initial)}
    (out/'result.json').write_text(json.dumps(summary,indent=2)+'\n')
    print('RESULT',out,'core_unchanged',initial==after,flush=True)
if __name__=='__main__':main()
