"""Bounded cached joint plans with guarded <=50-row continuation.

Default world policy refuses unknown objects. Explicit simulator_snapshot is
privileged full-scene collision input, not vision-only perception.
"""
from pathlib import Path
import json,uuid,time,subprocess,sys,threading,os
from collections import OrderedDict
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
from .model import ROOT,SIDES,static_xml,robot_snapshot,source_epoch,source_hash,digest,StaticRobotModel,world_snapshot,world_matches,world_is_quiet
from .collision import CollisionChecker
from .operator_stop import request_stop
from .telemetry import motor_snapshot
from .limits import plan_rows,execute_rows

class MotionService:
    def __init__(self,env,episode_id,run_root,*,scene_text=None,on_step=None,can_step=None,world_policy="robot_static"):
        if scene_text is not None and not getattr(env,'static_planning_fixture',False):raise ValueError('Custom static scene is diagnostic-fixture-only')
        if world_policy not in ('robot_static','simulator_snapshot'):raise ValueError('Unknown collision-world policy')
        self.world_policy=world_policy
        self.owner_plan=None
        self.recovery_required=None
        self.created_epoch=source_epoch();self.env=env;self.episode=episode_id;self.root=Path(run_root);self.out=self.root/'motion_planning';self.out.mkdir(parents=True,exist_ok=True);self.stop_file=self.out/'STOP';self.cfg=json.loads((Path(__file__).parent/'config.json').read_text());self.xml,self.omitted=static_xml(scene_text,include_dynamic=world_policy=="simulator_snapshot");self.static_hash=digest(self.xml);self.model=StaticRobotModel(self.xml,self.snapshot());self.plans=OrderedDict();self.active=None;self.mutex=threading.Lock();self.last=None;self.on_step=on_step;self.can_step=can_step;self.created_policy=self.policy_identity()
    def policy_identity(self):
        path=self.root/'motion_policy.json'
        return digest({'selected_world_policy':self.world_policy,'runtime_policy':path.read_text() if path.exists() else None})
    def snapshot(self):
        state=robot_snapshot(self.env)
        if self.world_policy=='simulator_snapshot':state['world']=world_snapshot(self.env)
        return state
    def coverage(self):
        return ('simulator-state-assisted full-scene collision snapshot; not vision-only or contact/folding proof'
                if self.world_policy=='simulator_snapshot' else 'static scene only; dynamic/held objects unsupported')
    def world_unchanged(self,start,current):
        return world_matches(start.get('world',{}),current.get('world',{}),self.cfg.get('world_translation_tolerance_m',.001),self.cfg.get('world_angle_tolerance_rad',.005))
    def world_version(self,snapshot):
        return digest({'model':self.static_hash,'policy':self.world_policy,'world':snapshot.get('world',{})})
    def log(self,event,**data):
        with (self.out/'audit.jsonl').open('a') as f:f.write(json.dumps({'event':event,'wall_unix_s':time.time(),'episode_id':self.episode,'step_id':self.env.env_step,**data},allow_nan=False)+'\n')
    def refuse(self,reason,**kw):
        r={'status':'refused','reason':reason,'executed':False,'episode_id':self.episode,'step_id':self.env.env_step,**kw};self.log('refusal',result=r);self.last=r;return r
    def status(self,plan_id=None):
        p=self.plans.get(plan_id) if plan_id else None
        return {'status':'stopped' if self.stop_file.exists() else 'executing' if self.active else 'paused' if self.owner_plan else 'recovery_required' if self.recovery_required else 'idle','episode_id':self.episode,'step_id':self.env.env_step,'active_plan':self.active,'owner_plan':self.owner_plan,'plan_state':p['state'] if p else None,'cursor':p.get('cursor',0) if p else None,'total_rows':len(p['result']['rows']) if p else None,'recovery_required':self.recovery_required,'last_result':self.last,'planner':'OMPL1.7.0/RRTConnect','control_dt_s':self.cfg['control_dt_s'],'max_rows_per_call':execute_rows(self.cfg),'max_rows':self.cfg['max_rows'],'max_plan_rows':plan_rows(self.cfg),'max_execute_rows':execute_rows(self.cfg),'max_trajectory_rows':plan_rows(self.cfg),'dynamic_coverage':'simulator_snapshot' if self.world_policy=='simulator_snapshot' else 'unknown' if self.omitted else 'static_scene_only','world_policy':self.world_policy,'cardboard_avoidance_verified':False,'held_object_planning':'unsupported','moving_jaw_planning':'unsupported','perception':'not vision-only; local simulator geometry' if self.world_policy=='simulator_snapshot' else 'not implemented; unknown dynamics globally refuse','synchronized_two_arm_planning':'unsupported','stop_semantics':'MCP stop may queue behind dispatcher. Operator STOP file is checked before every control row. Guard-stopped paths cannot resume; explicit operator-authorized reset/recommissioning is required, never an automatic reference reset.'}
    def stop(self,reason='operator_stop'):
        # Publish cancellation first; never wait for the execution mutex.
        request_stop(self.root,reason)
        terminalized=[];pending=True
        if self.mutex.acquire(blocking=False):
            try:
                pending=False;owner=self.owner_plan
                ids=[ident for ident,p in self.plans.items() if p['state'] in ('planned','paused')]
                # Preserve the physically relevant paused owner's recovery cause.
                for ident in sorted(ids,key=lambda value:value==owner):
                    self.terminate(ident,'STOP_REQUESTED');terminalized.append(ident)
            finally:self.mutex.release()
        # During an active row the executor alone finalizes its plan at the next
        # cancellation check. A callback stop must not mutate its cursor mid-row.
        self.log('stop_requested',reason=reason,terminalized_plans=terminalized,pending_execution_boundary=pending)
        return {'status':'stop_requested' if pending else 'stopped','executed':False,'reason':reason,
                'terminalized_plans':terminalized,'pending_execution_boundary':pending}
    def precondition(self,s):
        if self.stop_file.exists():return 'STOP_REQUESTED'
        if not world_is_quiet(s.get('world',{}),self.cfg.get('world_linear_velocity_limit_m_s',.02),self.cfg.get('world_angular_velocity_limit_rad_s',.05)):return 'WORLD_MOVING'
        if abs(self.env.control_dt-self.cfg['control_dt_s'])>1e-12 or abs(self.env._n_substeps*self.env.model.opt.timestep-self.env.control_dt)>1e-12:return 'TIMING_MISMATCH'
        for side in SIDES:
            a=s['robot'][side]
            if not all(np.isfinite(a[k]).all() for k in ['q','v','qcmd','jaws','jaw_v']):return 'NONFINITE_ROBOT_STATE'
            if max(np.abs(a['v']))>self.cfg['start_velocity_tolerance_rad_s'] or max(np.abs(a['jaw_v']))>self.cfg['start_velocity_tolerance_rad_s']:return 'START_NOT_SETTLED'
            if max(np.abs(np.array(a['qcmd'])-a['q']))>self.cfg['start_reference_tolerance_rad']:return 'START_REFERENCE_MISMATCH'
            if max(np.abs(np.array(a['jaws'])-a['jaw_reference']))>self.cfg['jaw_motion_limit_rad']:return 'JAW_REFERENCE_MISMATCH'
        return None
    def plan(self,args):
        if self.active or self.owner_plan:return self.refuse('ARM_OWNERSHIP_BUSY')
        if self.recovery_required:return self.refuse('RECOVERY_REQUIRED',recovery=self.recovery_required)
        if self.created_policy!=self.policy_identity():return self.refuse('POLICY_CHANGED_RECREATE_SERVICE')
        if self.created_epoch!=source_epoch():return self.refuse('SOURCE_CHANGED_RECREATE_SERVICE')
        if args['episode_id']!=self.episode or args['expected_step_id']!=self.env.env_step:return self.refuse('STALE_EPISODE_OR_STEP')
        s=self.snapshot();bad=self.precondition(s)
        if bad:return self.refuse(bad)
        # No pose/hinge lookup or scene-state collision oracle for omitted bodies.
        if self.omitted:return self.refuse('INSUFFICIENT_DYNAMIC_WORLD_COVERAGE',coverage='Unknown dynamic objects are not read or collision-certified; no cardboard avoidance claim.')
        side=args['arm'];target=args['target'];model=self.model;idx=model.index[side]
        goal=dict(target);q=np.array(goal['quaternion_wxyz'],dtype=float);goal['quaternion_wxyz']=(q/np.linalg.norm(q)).tolist()
        epoch=source_epoch();sid=source_hash();ident=uuid.uuid4().hex;input_file=self.out/(ident+'.query.json');output_file=self.out/(ident+'.worker.json');worker_log=self.out/(ident+'.worker.log')
        query={'static_xml':self.xml,'snapshot':s,'arm':side,'target':goal,'config':self.cfg,'stop_file':str(self.stop_file),'velocity_limits_rad_s':self.env.arms[side]['velocity'].tolist()};input_file.write_text(json.dumps(query,allow_nan=False));self.log('planning_started',query=ident,arm=side,source_hash=sid,world_hash=self.world_version(s))
        process=None;reason=None;t=time.monotonic()
        with worker_log.open('wb') as log:
            try:
                process=subprocess.Popen([sys.executable,'-I',str(Path(__file__).parent/'worker.py'),str(input_file),str(output_file)],stdout=log,stderr=subprocess.STDOUT,env={**os.environ,'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1','NUMEXPR_NUM_THREADS':'1'})
                while process.poll() is None:
                    if self.stop_file.exists():reason='STOP_REQUESTED';break
                    if time.monotonic()-t>self.cfg['worker_timeout_s']:reason='PLANNER_PROCESS_TIMEOUT';break
                    time.sleep(.01)
                if reason:
                    process.terminate()
                    try:process.wait(timeout=1)
                    except subprocess.TimeoutExpired:process.kill();process.wait()
                if reason:return self.refuse(reason)
                if process.returncode!=0:return self.refuse('PLANNER_PROCESS_EXIT',returncode=process.returncode,worker_log=worker_log.name)
            finally:
                if process is not None and process.poll() is None:process.kill();process.wait()
        result=json.loads(output_file.read_text())
        if result['status']!='planned':return self.refuse(result.pop('reason'),**{k:v for k,v in result.items() if k not in ['status','executed']})
        if not 0<len(result.get('rows',[]))<=plan_rows(self.cfg):return self.refuse('TOTAL_TRAJECTORY_LIMIT')
        result['rows']=tuple(tuple(float(v) for v in row) for row in result['rows'])
        if epoch!=source_epoch() or digest(self.snapshot())!=digest(s):return self.refuse('WORLD_OR_START_CHANGED_DURING_PLANNING')
        self.plans[ident]={'state':'planned','cursor':0,'arm':side,'target':goal,'snapshot':s,'epoch':epoch,'source_hash':sid,'world_hash':self.world_version(s),'model_hash':self.static_hash,'trajectory_hash':digest(result['rows']),'result':result}
        while len(self.plans)>self.cfg['max_cached_plans']:self.plans.popitem(last=False)
        answer={k:v for k,v in result.items() if k not in ['rows','structural_exclusions']};answer.update(plan_id=ident,episode_id=self.episode,step_id=self.env.env_step,source_hash=sid,world_version=self.world_version(s),coverage=self.coverage(),world_policy=self.world_policy);self.log('planned',result=answer);self.last=answer;return answer
    def preview(self,plan_id):
        p=self.plans.get(plan_id)
        if p is None:return self.refuse('PLAN_NOT_FOUND')
        if p['state'] not in ('planned','paused'):return self.refuse('PLAN_NOT_EXECUTABLE')
        reason=self.stale(p)
        if reason:
            self.invalidate(plan_id,reason);return self.refuse(reason)
        cursor=p.get('cursor',0)
        return {'status':'preview','executed':False,'plan_id':plan_id,'arm':p['arm'],'cursor':cursor,'total_rows':len(p['result']['rows']),'next_chunk_rows':min(execute_rows(self.cfg),len(p['result']['rows'])-cursor),'joint_names':[self.env.model.joint(int(j)).name for j in self.env.arms[p['arm']]['joint_ids']],'joint_positions_rad':[list(row) for row in p['result']['rows'][cursor:]],'control_dt_s':self.cfg['control_dt_s'],'duration_s':p['result']['duration_s'],'trajectory_hash':p.get('trajectory_hash'),'coverage':self.coverage(),'live_physics_advanced':False}
    def stale(self,p):
        if self.recovery_required:return 'RECOVERY_REQUIRED'
        if self.created_policy!=self.policy_identity():return 'POLICY_CHANGED'
        if self.stop_file.exists():return 'STOP_REQUESTED'
        if self.omitted:return 'INSUFFICIENT_DYNAMIC_WORLD_COVERAGE'
        if p['epoch']!=source_epoch() or p.get('model_hash',p['world_hash'])!=self.static_hash:return 'SOURCE_OR_WORLD_CHANGED'
        if p.get('trajectory_hash')!=digest(p['result']['rows']):return 'TRAJECTORY_CHANGED'
        s=self.snapshot();start=p['snapshot']
        if not self.world_unchanged(start,s):return 'WORLD_GEOMETRY_CHANGED'
        if p.get('cursor',0):
            boundary=p['boundary_snapshot']
            if s['step']!=boundary['step'] or abs(s['time']-boundary['time'])>1e-12:return 'CONTINUATION_STEP_CHANGED'
            if digest(s.get('world',{}))!=digest(boundary.get('world',{})):return 'CONTINUATION_WORLD_CHANGED'
            if not world_is_quiet(s.get('world',{}),self.cfg.get('world_linear_velocity_limit_m_s',.02),self.cfg.get('world_angular_velocity_limit_rad_s',.05)):return 'WORLD_MOVING'
            for side in SIDES:
                for key in ('q','v','qcmd','jaws','jaw_v'):
                    if not np.isfinite(s['robot'][side][key]).all():return 'NONFINITE_ROBOT_STATE'
                    if not np.array_equal(s['robot'][side][key],boundary['robot'][side][key]):return 'CONTINUATION_STATE_CHANGED'
                if s['robot'][side]['opening']!=boundary['robot'][side]['opening'] or s['robot'][side]['jaw_reference']!=boundary['robot'][side]['jaw_reference']:return 'JAW_COMMAND_CHANGED'
                if max(abs(np.array(s['robot'][side]['q'])-s['robot'][side]['qcmd']))>self.cfg['tracking_error_limit_rad']:return 'TRACKING_ERROR'
            return None
        if s['step']!=start['step'] or abs(s['time']-start['time'])>1e-12:return 'STALE_STEP'
        for side in SIDES:
            for key in ['q','jaws','qcmd']:
                if max(np.abs(np.array(s['robot'][side][key])-start['robot'][side][key]))>self.cfg['start_position_tolerance_rad']:return 'START_CHANGED'
            if s['robot'][side]['opening']!=start['robot'][side]['opening']:return 'JAW_COMMAND_CHANGED'
        return self.precondition(s)
    def invalidate(self,plan_id,reason):
        p=self.plans[plan_id];p['state']='invalidated'
        if self.owner_plan==plan_id:self.owner_plan=None
        if p.get('cursor',0):
            self.recovery_required={'plan_id':plan_id,'reason':reason,'policy':'A partially executed path was invalidated. Explicit operator-authorized reset/recommissioning required; no reference reset or raw hold recovery.'}
        self.checkpoint_plan(plan_id)
    def checkpoint_plan(self,plan_id):
        """Durable audit cursor, deliberately NOT restart-resumable robot authority."""
        p=self.plans[plan_id]
        target=self.out/(plan_id+'.cursor.json');tmp=target.with_suffix('.tmp')
        data={'episode_id':self.episode,'plan_id':plan_id,'state':p['state'],'cursor':p.get('cursor',0),'total_rows':len(p['result']['rows']),'trajectory_hash':p.get('trajectory_hash'),'source_hash':p['source_hash'],'policy_identity':self.created_policy,'world_hash':p['world_hash'],'boundary_snapshot':p.get('boundary_snapshot'),'restart_recovery':'refuse; process restart requires explicit operator recovery, never restore execution authority from this audit file'}
        with tmp.open('w') as stream:
            json.dump(data,stream,allow_nan=False);stream.flush();os.fsync(stream.fileno())
        os.replace(tmp,target)
    def terminate(self,plan_id,reason):
        """Fail closed on external budget/episode failure without stepping."""
        p=self.plans.get(plan_id)
        if p is None:return self.refuse(reason,plan_id=plan_id)
        if p['state'] in ('completed','stopped','invalidated'):return self.refuse('PLAN_ALREADY_USED',plan_id=plan_id,terminal_reason=reason)
        p['state']='stopped'
        if self.owner_plan==plan_id:self.owner_plan=None
        self.recovery_required={'plan_id':plan_id,'reason':reason,'policy':'Terminal execution/budget failure; no resume/old-target hold. Explicit operator-authorized reset and fresh commissioning required.'}
        self.checkpoint_plan(plan_id)
        result={'status':'stopped','reason':reason,'executed':False,'rows_executed':0,'plan_id':plan_id,'cursor':p.get('cursor',0),'remaining_rows':len(p['result']['rows'])-p.get('cursor',0),'continuation_available':False,'command_completed':False,'goal_reached':False,'recovery_required':self.recovery_required}
        self.log('execution_terminated_without_step',result=result);self.last=result;return result
    def cursor_refusal(self,plan_id,expected_cursor=None):
        """Read-only admission shared with backend; execute rechecks under mutex."""
        p=self.plans.get(plan_id)
        if p is None:return {'reason':'PLAN_NOT_FOUND','plan_id':plan_id}
        cursor=p.get('cursor',0)
        if expected_cursor is None:
            if cursor:return {'reason':'EXPECTED_CURSOR_REQUIRED','plan_id':plan_id,'cursor':cursor}
            return None  # Backward-compatible first execution only.
        if type(expected_cursor) is not int or expected_cursor<0:
            return {'reason':'INVALID_EXPECTED_CURSOR','plan_id':plan_id,'cursor':cursor}
        if expected_cursor!=cursor:
            return {'reason':'CURSOR_MISMATCH','plan_id':plan_id,'cursor':cursor,'expected_cursor':expected_cursor}
        return None
    def execute(self,plan_id,max_rows=None,*,expected_cursor=None):
        limit=execute_rows(self.cfg) if max_rows is None else max_rows
        if type(limit) is not int or not 1<=limit<=execute_rows(self.cfg):return self.refuse('INVALID_CHUNK_SIZE')
        if not self.mutex.acquire(blocking=False):return self.refuse('ARM_OWNERSHIP_BUSY')
        try:
            p=self.plans.get(plan_id)
            if p is None:return self.refuse('PLAN_NOT_FOUND')
            if self.stop_file.exists():
                if p['state'] in ('planned','paused'):self.terminate(plan_id,'STOP_REQUESTED')
                return self.refuse('STOP_REQUESTED',plan_id=plan_id,cursor=p.get('cursor',0))
            if p['state'] not in ('planned','paused'):return self.refuse('PLAN_ALREADY_USED',recovery_required=p['state']=='stopped')
            if self.owner_plan not in (None,plan_id):return self.refuse('ARM_OWNERSHIP_BUSY')
            cursor_error=self.cursor_refusal(plan_id,expected_cursor)
            if cursor_error:return self.refuse(**cursor_error)
            bad=self.stale(p)
            if bad:
                self.invalidate(plan_id,bad);return self.refuse(bad)
            rows=p['result']['rows']
            if len(rows)>plan_rows(self.cfg):return self.refuse('TOTAL_TRAJECTORY_LIMIT')
            self.active=plan_id;self.owner_plan=plan_id;p['state']='executing';side=p['arm'];other=next(s for s in SIDES if s!=side);s=p['snapshot'];start_cursor=p.get('cursor',0);p['cursor']=start_cursor;trace=[];reason=None;wall=time.monotonic();checker=CollisionChecker(self.model,side,self.cfg['collision_clearance_m'])
            for row in rows[start_cursor:start_cursor+limit]:
                if self.stop_file.exists():reason='STOP_REQUESTED';break
                if self.created_policy!=self.policy_identity():reason='POLICY_CHANGED';break
                if p['epoch']!=source_epoch():reason='SOURCE_CHANGED';break
                if self.can_step and not self.can_step():reason='STOP_REQUESTED' if self.stop_file.exists() else 'EPISODE_WALL_LIMIT';break
                qs={side:np.array(row),other:np.array(s['robot'][other]['q'])};grips={v:s['robot'][v]['opening'] for v in SIDES}
                current=self.snapshot()
                if not world_is_quiet(current.get('world',{}),self.cfg.get('world_linear_velocity_limit_m_s',.02),self.cfg.get('world_angular_velocity_limit_rad_s',.05)):reason='WORLD_MOVING';break
                if not self.world_unchanged(s,current):reason='WORLD_GEOMETRY_CHANGED';break
                self.model.load(current)
                if not checker.edge(np.array(current['robot'][side]['q']),qs[side],self.cfg,lambda:self.stop_file.exists()):
                    reason='STOP_REQUESTED' if self.stop_file.exists() else 'NEXT_EDGE_COLLISION';break
                # Cancellation/policy may change while the edge check runs.
                if self.stop_file.exists():reason='STOP_REQUESTED';break
                if self.created_policy!=self.policy_identity():reason='POLICY_CHANGED';break
                if p['epoch']!=source_epoch():reason='SOURCE_CHANGED';break
                if self.can_step and not self.can_step():reason='STOP_REQUESTED' if self.stop_file.exists() else 'EPISODE_WALL_LIMIT';break
                before=self.env.env_step;row_wall=time.monotonic();row_sim=float(self.env.data.time)
                try:obs,reward,done,truncated,info=self.env.step_joint_targets(qs,grips)
                except Exception as exc:reason='EXECUTION_ERROR';self.log('execution_exception',kind=type(exc).__name__);break
                measured=self.snapshot();qerr=max(np.max(abs(np.array(measured['robot'][v]['q'])-qs[v])) for v in SIDES);reference_error=max(np.max(abs(np.array(measured['robot'][v]['qcmd'])-qs[v])) for v in SIDES)
                p['cursor']+=1;p['boundary_snapshot']=measured
                entry={'step_id':self.env.env_step,'sim_time_s':self.env.data.time,'requested':{v:qs[v].tolist() for v in SIDES},'grippers':grips,'applied':{v:self.env.arms[v]['qcmd'].tolist() for v in SIDES},'measured':measured['robot'],'max_tracking_error_rad':float(qerr),'max_reference_error_rad':float(reference_error),
                       'motor_sample_last_substep':motor_snapshot(self.env),
                       'trajectory_row':p['cursor'],'control_row_timing':{'expected_sim_dt_s':self.env.control_dt,'actual_sim_dt_s':float(self.env.data.time)-row_sim,'physics_dt_s':float(self.env.model.opt.timestep),'physics_substeps':self.env._n_substeps,'execution_wall_s':time.monotonic()-row_wall}};trace.append(entry);self.log('joint_action',plan_id=plan_id,**entry)
                entry.update(terminated=bool(done),truncated=bool(truncated))
                self.checkpoint_plan(plan_id)
                if self.on_step:
                    try:self.on_step(entry,obs,info)
                    except Exception as exc:
                        reason='STOP_REQUESTED' if self.stop_file.exists() else 'CALLBACK_ERROR'
                        self.log('execution_callback_error',plan_id=plan_id,error_type=type(exc).__name__,detail=str(exc),stop_requested=self.stop_file.exists())
                        break
                if self.stop_file.exists():reason='STOP_REQUESTED';break
                if self.env.env_step!=before+1 or not all(np.isfinite(measured['robot'][v]['q']).all() and np.isfinite(measured['robot'][v]['v']).all() for v in SIDES) or any(self.env.data.warning.number):reason='INVALID_SIMULATION';break
                if qerr>self.cfg['tracking_error_limit_rad']:reason='TRACKING_ERROR';break
                if reference_error>1e-6:reason='REFERENCE_CLIPPED';break
                if max(abs(np.array(measured['robot'][other]['q'])-s['robot'][other]['q']))>self.cfg['inactive_motion_limit_rad']:reason='INACTIVE_ARM_MOVED';break
                if any(max(abs(np.array(measured['robot'][v]['jaws'])-s['robot'][v]['jaws']))>self.cfg['jaw_motion_limit_rad'] for v in SIDES):reason='JAW_MOVED';break
                self.model.load(measured)
                if not checker.check(np.array(measured['robot'][side]['q'])):reason='MEASURED_CLEARANCE_VIOLATION';break
                if done or truncated:reason='EPISODE_ENDED';break
            self.model.load(self.snapshot());pos,R=self.model.pose(side);G=Rotation.from_quat(np.array(p['target']['quaternion_wxyz'])[[1,2,3,0]]);error={'position_m':float(np.linalg.norm(pos-p['target']['position_m'])),'orientation_rad':float((G*R.inv()).magnitude())};complete=p['cursor']==len(rows) and reason is None;reached=complete and error['position_m']<=self.cfg['measured_goal_position_tolerance_m'] and error['orientation_rad']<=self.cfg['measured_goal_orientation_tolerance_rad'];p['state']='stopped' if reason else 'completed' if complete else 'paused'
            if p['state']!='paused':self.owner_plan=None
            if reason:
                self.recovery_required={'plan_id':plan_id,'reason':reason,'policy':'No resume/reference reset. Explicit operator-authorized episode reset and fresh settled-start commissioning required; do not use Cartesian hold as recovery.'}
            self.checkpoint_plan(plan_id)
            result={'status':p['state'],'reason':reason,'plan_id':plan_id,'executed':bool(trace),'rows_executed':len(trace),'rows_planned':len(rows),'cursor_start':start_cursor,'cursor':p['cursor'],'remaining_rows':len(rows)-p['cursor'],'continuation_available':p['state']=='paused','chunk_completed':reason is None,'command_completed':complete,'goal_reached':bool(reached),'measured_goal_error':error,'wall_seconds':time.monotonic()-wall,'stop_ack_delay_wall_s':(max(0.,time.time()-json.loads(self.stop_file.read_text())['wall_unix_s']) if reason=='STOP_REQUESTED' else None),'recovery_required':self.recovery_required,'task_success':False,'coverage':self.coverage(),'world_policy':self.world_policy};self.log('execution_finished',result=result);self.last=result;return result
        finally:self.active=None;self.mutex.release()
