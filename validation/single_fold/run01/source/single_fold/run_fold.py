"""Fixture-assisted original passive single-crease folding, motor execution only."""
from fixture import *
import argparse,csv,gzip,hashlib,os,subprocess,time,traceback
sys.path.insert(0,str(ROOT/'experiments/grasp_skill'))
from run import Trial,hashes,Validity,dense,OMPLPlanner,RightArmImpedance,robot_name
from PIL import Image
import imageio_ffmpeg

FOLD_PHASES={'fold5','checkpoint5','fold15','fold_hold','fold_return','return_hold'}
class FoldTrial(Trial):
 force_feedback_phases=Trial.force_feedback_phases|FOLD_PHASES
 def __init__(self,out,path=None):
  env=FoldEnv();super().__init__(out,0.,task='single_fold',env=env,scene_source=fold_xml(),target_geom_name=CARD)
  self.saved_path=path;self.hinge=self.m.joint(CREASE).id;self.base_ref=None;self.grasp_ref=None;self.arc_ref=None;self.command_angle=0.;self.extra_rows=[];self.fixture_names={n for n,_,_ in FIXTURE};self.base_geom=self.m.geom('card_base_0').id;self.fixture_force=0.;self.moving_fixture_force=0.;self.max_fixture_force=0.;self.all_base_motion=[];self.fold_slip=[]
  self.report.update(fixture_ready=False,grasp_success=False,fold15_success=False,hold_success=False,return_success=False,release_success=False,task='single_fold',panel=PANEL,mass_kg=float(self.m.body(PANEL).mass[0]),dimensions_m=[.09,.217,.004],base_mass_kg=float(self.m.body('box_base').mass[0]),fixture_role='Free base retained solely by lower stand, cap and side/far stops through original table/card contact law; moving panel free of fixture.',thresholds={'fold_target_deg':15.,'angle_bounds_deg':[13.,17.],'hold_s':1.,'base_translation_m':.002,'base_rotation_deg':1.,'relative_slip_m':.003,'relative_angle_rad':.05,'slip_abort_m':.008,'angle_abort_rad':.10,'hand_force_abort_N':5.,'return_angle_bounds_deg':[-2.,2.]},arc_speed_deg_s=2.,model_audit=audit(env))
 def state(self):
  s=super().state();site=self.e.arms['right']['site'];rt=self.d.site_xmat[site].reshape(3,3);pt=self.d.site_xpos[site];moving=self.d.body(PANEL);base=self.d.body('box_base');relative=base.xmat.reshape(3,3).T@moving.xmat.reshape(3,3)
  s.update(com=moving.xipos.tolist(),moving_position=moving.xpos.tolist(),object_quat=moving.xquat.tolist(),relative_position=((moving.xipos-pt)@rt).tolist(),relative_rotation=(rt.T@moving.xmat.reshape(3,3)).tolist(),base_position=base.xpos.tolist(),base_rotation=base.xmat.reshape(3,3).tolist(),crease_rad=float(self.d.qpos[self.m.jnt_qposadr[self.hinge]]),crease_fk_rad=float(np.arctan2(relative[0,2]-relative[2,0],relative[0,0]+relative[2,2])),commanded_crease_rad=self.command_angle,fixture_base_force_N=self.fixture_force,moving_fixture_force_N=self.moving_fixture_force)
  return s
 def observe(self,m,d,*args,**kwargs):
  try:
   super().observe(m,d,*args,**kwargs)
   self.fixture_force=self.moving_fixture_force=0.
   for i,c in enumerate(d.contact):
    ids=[int(c.geom1),int(c.geom2)];names=[self.names[g] for g in ids]
    if not any(n in self.fixture_names for n in names):continue
    force=np.zeros(6);mujoco.mj_contactForce(m,d,i,force);normal=max(0.,float(force[0]))
    if self.base_geom in ids:self.fixture_force+=normal
    if self.card in ids:self.moving_fixture_force+=normal
   self.max_fixture_force=max(self.max_fixture_force,self.fixture_force)
   base=d.body('box_base');right=self.e.arms['right'];g=self.e.grippers['right'];actual=float(d.qpos[self.m.jnt_qposadr[self.hinge]])
   self.extra_rows.append([float(d.time),self.phase,self.command_angle,actual,self.fixture_force,self.moving_fixture_force,*base.xpos.tolist(),*base.xquat.tolist(),*d.actuator_force[right['act']].tolist(),float(d.actuator_force[g['act']])])
   if self.moving_fixture_force>.05:raise RuntimeError('Moving panel touched fixture: '+str(self.moving_fixture_force))
  except RuntimeError:
   contacts=[]
   for i,c in enumerate(d.contact):
    f=np.zeros(6);mujoco.mj_contactForce(m,d,i,f);contacts.append({'geoms':[self.names[int(c.geom1)],self.names[int(c.geom2)]],'force_local':f.tolist(),'distance_m':float(c.dist),'position_m':c.pos.tolist()})
   stem=f'abort_{self.step_count:08d}';(self.out/(stem+'_contacts.json')).write_text(json.dumps({'time_s':float(d.time),'phase':self.phase,'contacts':contacts},indent=2));np.savez(self.out/(stem+'_state.npz'),qpos=d.qpos.copy(),qvel=d.qvel.copy(),ctrl=d.ctrl.copy(),time=float(d.time));raise
 def after_tick(self):
  s=self.rows[-1]
  if self.base_ref is not None:
   p=float(np.linalg.norm(np.array(s['base_position'])-self.base_ref['base_position']));a=float(np.linalg.norm(Rotation.from_matrix(np.array(s['base_rotation'])@np.array(self.base_ref['base_rotation']).T).as_rotvec()))
   self.all_base_motion.append({'t':s['t'],'phase':self.phase,'translation_m':p,'rotation_rad':a})
   if p>.002 or a>np.deg2rad(1.):raise RuntimeError(f'Fixture base motion exceeded2mm/1deg: {p}m {np.rad2deg(a)}deg')
  if self.grasp_ref is not None and self.phase in FOLD_PHASES:
   p=float(np.linalg.norm(np.array(s['relative_position'])-self.grasp_ref['relative_position']));a=float(np.linalg.norm(Rotation.from_matrix(np.array(s['relative_rotation'])@np.array(self.grasp_ref['relative_rotation']).T).as_rotvec()));self.fold_slip.append({'t':s['t'],'phase':self.phase,'translation_m':p,'rotation_rad':a})
   if p>.008 or a>.10:raise RuntimeError(f'Fold slip exceeded8mm/.10rad: {p}m {a}rad')
 def arc_target(self,angle):
  ref=self.arc_ref;rot=Rotation.from_rotvec(np.array(ref['axis'])*(angle-ref['angle'])).as_matrix();position=np.array(ref['anchor'])+rot@(np.array(ref['tcp'][:3])-ref['anchor']);initial=Rotation.from_quat(np.array(ref['tcp'][3:])[[1,2,3,0]]).as_matrix();quat=Rotation.from_matrix(rot@initial).as_quat()[[3,0,1,2]];return np.r_[position,quat]
 def arc(self,goal_deg,phase):
  self.phase=phase;initial=self.command_angle;goal=np.deg2rad(goal_deg);count=max(1,int(np.ceil(abs(goal-initial)/np.deg2rad(2.)/self.e.control_dt)))
  for i in range(1,count+1):self.command_angle=initial+(goal-initial)*i/count;self.tick(self.arc_target(self.command_angle))
 def scratch_arc(self):
  before=hashlib.sha256(self.d.qpos.tobytes()+self.d.qvel.tobytes()+self.d.ctrl.tobytes()).hexdigest();scratch=mujoco.MjData(self.m);seed=self.d.qpos[self.e.arms['right']['qad']].copy();samples=[]
  for angle in np.r_[np.linspace(self.command_angle,np.deg2rad(15.),31),np.linspace(np.deg2rad(15.),0.,31)]:
   target=self.arc_target(angle);q,error=self.e.inverse_kinematics('right',target,iterations=100,seed=seed);seed=q
   scratch.qpos[:]=self.d.qpos;scratch.qpos[self.e.arms['right']['qad']]=q;scratch.qpos[self.m.jnt_qposadr[self.hinge]]=angle;mujoco.mj_forward(self.m,scratch);bad=[]
   for c in scratch.contact:
    ids=[int(c.geom1),int(c.geom2)];names=[self.names[g] for g in ids];intended=self.card in ids and any(any(g in group for g in ids) for group in self.groups)
    fixture_base=self.base_geom in ids and any(n in self.fixture_names for n in names)
    if not intended and not fixture_base and (any(robot_name(n) for n in names) or self.card in ids) and c.dist<-1e-6:bad.append({'geoms':names,'dist_m':float(c.dist)})
   sample={'predicted_angle_rad':float(angle),'ik':error,'bad':bad};samples.append(sample)
   if bad or error['position_m']>.0003 or error['orientation_rad']>.003:
    (self.out/'arc_prediction.json').write_text(json.dumps({'prediction_only':True,'samples':samples,'passed':False},indent=2));raise RuntimeError('Predicted arc rejected '+json.dumps(sample))
  after=hashlib.sha256(self.d.qpos.tobytes()+self.d.qvel.tobytes()+self.d.ctrl.tobytes()).hexdigest();result={'prediction_only':True,'passed':True,'samples':samples,'execution_state_unchanged':before==after,'state_sha_before':before,'state_sha_after':after};assert before==after;(self.out/'arc_prediction.json').write_text(json.dumps(result,indent=2));self.report['arc_prediction']=result
 def held_metrics(self,phase):
  states=[r for r in self.rows if r['phase']==phase];angles=np.rad2deg([s['crease_rad'] for s in states]);slip=[s for s in self.fold_slip if s['phase'] in {'fold5','checkpoint5','fold15','fold_hold'}];base=self.all_base_motion
  return {'duration_s':states[-1]['t']-states[0]['t'],'min_angle_deg':float(angles.min()),'max_angle_deg':float(angles.max()),'max_joint_fk_error_rad':max(abs(s['crease_rad']-s['crease_fk_rad']) for s in states),'min_loaded_jaw_N':min(min(s['jaw_forces_N']) for s in states),'max_moving_fixture_N':max(s['moving_fixture_force_N'] for s in states),'max_relative_slip_m':max((s['translation_m'] for s in slip),default=0.),'max_relative_rotation_rad':max((s['rotation_rad'] for s in slip),default=0.),'max_base_translation_m':max((s['translation_m'] for s in base),default=0.),'max_base_rotation_deg':float(np.rad2deg(max((s['rotation_rad'] for s in base),default=0.)))}
 def run(self):
  start=time.monotonic();mujoco.mj_step=self.observe;controller=None;before=hashes();(self.out/'core_hashes_before.json').write_text(json.dumps(before,indent=2));(self.out/'model_audit.json').write_text(json.dumps(self.report['model_audit'],indent=2))
  try:
   self.report['initial_state']=self.state();self.report['initial_execution_state_sha256']=hashlib.sha256(self.d.qpos.tobytes()+self.d.qvel.tobytes()+self.d.ctrl.tobytes()).hexdigest();valid=Validity(self.e);assert valid(self.e.arms['right']['qcmd']),'Home collision'
   self.image('00_initial');self.phase='fixture_settle'
   for _ in range(50):self.tick(q=self.e.arms['right']['qcmd'].copy())
   recent=self.rows[-25:];span=np.ptp(np.array([s['base_position'] for s in recent]),axis=0);self.report['fixture_ready']=bool(np.linalg.norm(span)<.0005 and self.fixture_force>.05);self.report['settled_state']=self.state();self.report['settle_position_span_m']=span.tolist()
   if not self.report['fixture_ready']:raise RuntimeError('Fixture did not settle '+str(span))
   self.base_ref=self.state();target,pre,qpre,qgrasp,ik=grasp_candidate(self.e);self.report['grasp_target']=target.tolist();self.report['ik']=ik;assert max(x['position_m'] for x in ik.values())<.0002
   valid=Validity(self.e)
   if self.saved_path is None:path,metadata=OMPLPlanner(self.e.arms['right']['limits'],self.out,5.)(self.d.qpos[self.e.arms['right']['qad']].copy(),qpre,valid,'approach')
   else:
    path=np.load(self.saved_path);assert np.allclose(path[0],self.d.qpos[self.e.arms['right']['qad']],atol=1e-6,rtol=0) and np.allclose(path[-1],qpre,atol=1e-6,rtol=0);np.save(self.out/'approach_path.npy',path);metadata={'reused':str(self.saved_path),'sha256':hashlib.sha256(Path(self.saved_path).read_bytes()).hexdigest()}
   if path is None:raise RuntimeError('OMPL failed')
   samples=list(dense(path));assert all(valid(q) for q in samples);assert all(valid(q) for q in dense([qpre,qgrasp],.003));self.report['planner']=metadata;self.report['path_dense_samples']=len(samples)
   self.phase='ompl_approach';print('OMPL approach',flush=True)
   for q in dense(path,.2*self.e.control_dt):self.tick(q=q)
   for _ in range(50):self.tick(q=qpre)
   self.image('01_pregrasp')
   with RightArmImpedance(self.e) as controller:
    self.e.targets['right']=self.e.tcp_pose('right').copy();self.phase='insert';self.move(target,5.)
    for _ in range(50):self.tick(target)
    self.image('02_inserted');self.phase='close_position';g=self.e.grippers['right'];ref=g['qcmd'];print('Closing moving panel',flush=True)
    for _ in range(650):
     ref=max(.00192,ref-(.05 if ref>.04 else .006)*self.e.control_dt);self.tick(target,opening=ref/.96)
     if self.force.sum()>=1.5 or ref<=.00192:break
    self.phase='close_force';g['effort_scale']=.06/.4
    for _ in range(300):
     self.tick(target,opening=0.)
     if np.all(self.force>.05) and 2.8<=self.force.sum()<=3.2:break
    self.phase='grasp_settle'
    for _ in range(25):self.tick(target)
    self.report['grasp_success']=bool(all(min(s['jaw_forces_N'])>.05 for s in self.rows[-25:]));self.report['postclosure']=self.state();self.image('03_grasped')
    if not self.report['grasp_success']:raise RuntimeError('Moving-panel opposing jaw retention failed')
    self.grasp_ref=self.state();self.command_angle=self.grasp_ref['crease_rad'];self.arc_ref={'axis':self.d.xaxis[self.hinge].tolist(),'anchor':self.d.xanchor[self.hinge].tolist(),'angle':self.command_angle,'tcp':self.e.tcp_pose('right').tolist()};self.report['arc_reference']=self.arc_ref
    self.phase='arc_precheck';self.scratch_arc();print('FOLD5',flush=True);self.arc(5.,'fold5');self.phase='checkpoint5'
    for _ in range(20):self.tick(self.arc_target(self.command_angle))
    self.report['checkpoint5']=self.state();self.image('04_fold5');print('FOLD15',flush=True);self.arc(15.,'fold15');self.phase='fold_hold'
    for _ in range(60):self.tick(self.arc_target(self.command_angle))
    metrics=self.held_metrics('fold_hold');self.report['fold_metrics']=metrics;self.report['fold15_success']=bool(13<=metrics['min_angle_deg'] and metrics['max_angle_deg']<=17 and metrics['max_joint_fk_error_rad']<1e-5)
    self.report['hold_success']=bool(self.report['fold15_success'] and metrics['duration_s']>=1 and metrics['min_loaded_jaw_N']>.05 and metrics['max_moving_fixture_N']<.01 and metrics['max_relative_slip_m']<.003 and metrics['max_relative_rotation_rad']<.05 and metrics['max_base_translation_m']<=.002 and metrics['max_base_rotation_deg']<=1.)
    self.image('05_fold15');(self.out/'fold_checkpoint.json').write_text(json.dumps({'fold15_success':self.report['fold15_success'],'hold_success':self.report['hold_success'],'metrics':metrics,'time_s':float(self.d.time)},indent=2));print('FOLD_HOLD',self.report['hold_success'],metrics,flush=True)
    if not self.report['hold_success']:raise RuntimeError('Actual fold/hold criteria failed '+json.dumps(metrics))
    self.arc(0.,'fold_return');self.phase='return_hold'
    for _ in range(60):self.tick(self.arc_target(self.command_angle))
    metrics=self.held_metrics('return_hold');self.report['return_metrics']=metrics;self.report['return_success']=bool(-2<=metrics['min_angle_deg'] and metrics['max_angle_deg']<=2 and metrics['min_loaded_jaw_N']>.05);self.image('06_returned')
    if not self.report['return_success']:raise RuntimeError('Return angle/grasp failed '+json.dumps(metrics))
    self.phase='release';g['effort_scale']=1.;released_target=self.e.targets['right'].copy()
    for i in range(1,151):self.tick(released_target,opening=.3*i/150)
    self.phase='retreat';self.move(pre,5.);self.phase='released_settle'
    for _ in range(60):self.tick(pre)
    last=self.rows[-50:];self.report['release_success']=bool(all(sum(s['jaw_forces_N'])<.01 and abs(np.rad2deg(s['crease_rad']))<=2 for s in last));self.report['final']=self.state();self.image('07_released')
  except Exception as exc:
   self.report.update(error=repr(exc),failure_phase=self.phase,stopped=self.state(),abort=self.last_trip,traceback=traceback.format_exc());self.report['recovery']={'attempted':False,'complete':False}
   try:
    self.image('failure');self.phase='abort_open';self.e.targets['right']=self.e.tcp_pose('right').copy();self.e.grippers['right']['effort_scale']=1.;self.report['recovery']['attempted']=True
    for _ in range(100):self.tick(opening=min(.3,self.e.grippers['right']['command']+.003))
    if 'pre' in locals():self.phase='abort_retract';self.move(pre,4.);self.report['recovery']['complete']=True
    self.report['recovery']['final']=self.state();self.image('recovered')
   except Exception as recovery:self.report['recovery']['error']=repr(recovery)
  finally:
   mujoco.mj_step=self.original_step;self.writer.close();self.report['protocol_completed']=all(self.report[k] for k in ['fixture_ready','grasp_success','fold15_success','hold_success','return_success','release_success']);self.report['success']=self.report['protocol_completed'];self.report['warnings']=self.d.warning.number.tolist();self.report['phase_peak_hand_normal_N']=self.peaks;self.report['peak_hand_normal_N']=max(self.peaks.values(),default=0.);self.report['hand_force_overshoot_N']=max(0.,self.report['peak_hand_normal_N']-5.);self.report['peak_fixture_base_normal_N']=self.max_fixture_force;self.report['peak_unexpected_robot_contact_N']=self.bad_peak_N;self.report['simulation_s']=float(self.d.time);self.report['physics_substeps']=self.step_count;self.report['video_frames']=self.frames;self.report['wall_seconds']=time.monotonic()-start
   if controller:self.report['controller_metadata']=controller.metadata()
   for name,header,rows in [('forces',['time_s','phase','jaw1_N','jaw2_N','sum_N','moving_support_N','penetration_m','pad1_N','pad2_N'],self.force_rows),('contacts',['time_s','phase','geom1','geom2','normal_N','dist_m','x','y','z','nx','ny','nz'],self.contacts),('crease_fixture_torque',['time_s','phase','commanded_crease_rad','actual_crease_rad','fixture_base_N','moving_fixture_N','base_x','base_y','base_z','base_qw','base_qx','base_qy','base_qz']+[f'right_motor_{i}_Nm' for i in range(1,8)]+['gripper_motor_Nm'],self.extra_rows)]:
    with gzip.open(self.out/(name+'.csv.gz'),'wt') as f:w=csv.writer(f);w.writerow(header);w.writerows(rows)
   with gzip.open(self.out/'trajectory.json.gz','wt') as f:json.dump(self.rows,f)
   (self.out/'base_motion.json').write_text(json.dumps(self.all_base_motion));(self.out/'fold_slip.json').write_text(json.dumps(self.fold_slip));after=hashes();(self.out/'core_hashes_after.json').write_text(json.dumps(after,indent=2));self.report['core_unchanged']=before==after;self.report['core_hash_file_count']=len(before);(self.out/'result.json').write_text(json.dumps(self.report,indent=2));self.e.close()
  return self.report

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',default='validation/single_fold/run01');ap.add_argument('--worker',action='store_true');ap.add_argument('--path');args=ap.parse_args();out=ROOT/args.out
 if args.worker:
  r=FoldTrial(out,args.path).run();frames,seconds=imageio_ffmpeg.count_frames_and_secs(str(out/'video.mp4'));(out/'artifact_checks.json').write_text(json.dumps({'frames':frames,'seconds':seconds,'frame_count_matches':frames==r['video_frames']},indent=2));print(json.dumps({k:r.get(k) for k in ['fixture_ready','grasp_success','fold15_success','hold_success','return_success','release_success','error','failure_phase','peak_hand_normal_N','hand_force_overshoot_N']},ensure_ascii=False),flush=True);return
 out.mkdir(parents=True,exist_ok=False);source=out/'source';source.mkdir()
 for folder in [Path(__file__).parent,ROOT/'experiments/grasp_skill']:
  dest=source/folder.name;dest.mkdir()
  for p in folder.glob('*'):
   if p.is_file():(dest/p.name).write_bytes(p.read_bytes())
 env=os.environ.copy();env.update(MUJOCO_GL='egl',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
 def launch(i,path=None):
  folder=out/f'trial{i:02d}';log=open(out/f'trial{i:02d}.log','w');cmd=[sys.executable,str(Path(__file__).resolve()),'--worker','--out',str(folder)]
  if path is not None:cmd+=['--path',str(path)]
  return folder,subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,env=env,cwd=ROOT),log
 first,p,log=launch(0);code=p.wait();log.close()
 if code:raise RuntimeError('Worker error; inspect '+str(first))
 dirs=[first];r=json.loads((first/'result.json').read_text())
 if r['hold_success']:
  for folder,p,log in [launch(1,first/'approach_path.npy'),launch(2,first/'approach_path.npy')]:
   code=p.wait();log.close()
   if code:raise RuntimeError('Worker error '+str(folder))
   dirs.append(folder)
 reports=[json.loads((p/'result.json').read_text()) for p in dirs];summary={'task':'fixture_assisted_single_fold','trials':reports,'passed':{k:sum(r[k] for r in reports) for k in ['fixture_ready','grasp_success','fold15_success','hold_success','return_success','release_success','protocol_completed']},'original_whole_box_solved':False,'core_unchanged':all(r['core_unchanged'] for r in reports)};(out/'result.json').write_text(json.dumps(summary,indent=2));print(summary['passed'],flush=True)
if __name__=='__main__':main()
