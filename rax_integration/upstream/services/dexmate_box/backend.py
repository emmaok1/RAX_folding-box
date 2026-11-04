"""Trusted RAX adapter around the actual FoldingBoxEnv; no XML/physics edits.

Unlike the DexJoCo process adapter this runs the existing lightweight environment
in the host frontend process. Claude only reaches it through RAX's serialized MCP
gateway. Task/object state stays operator-only unless an explicit teacher contract grants it.
"""
import base64
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import uuid

import numpy as np
from services.mcp_contract import Contract
from services.mcp_reply import _png_data
from services.json_io import jsonable
from services.simulator.io import reject_unless
from .benchmark import PROJECT
from . import tools as T


def fingerprints():
    paths = [PROJECT/'environment.py', PROJECT/'robot_config.json', PROJECT/'physics.json',
             PROJECT/'box_geometry.py', PROJECT/'geometry/panels.json']
    paths += sorted(p for p in (PROJECT/'models').rglob('*') if p.is_file())
    return {str(p.relative_to(PROJECT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


class DexmateBoxBackend:
    evaluates_from_record = True

    def __init__(self, config):
        self.config = config
        self.contract = Contract.from_config(config, isolated=True)
        self.frame_workspace = Path(config.root)/'native/workspace'
        self.root = Path(config.root)/'native/episodes'
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.env = None
        self.episode_id = None
        self.finished = False
        self.termination_reason = None
        self.video = None
        self.step_id = 0
        self.motion = None
        self._last_control_mode = 'cartesian_ik'
        policy = Path(config.root)/'smoke_policy.json'
        self.smoke_policy = json.loads(policy.read_text()) if policy.exists() else None
        self.teacher_world_id = None
        self.teacher_memory_id = None
        if self.contract.teacher_privileged:
            if str(PROJECT) not in sys.path: sys.path.insert(0, str(PROJECT))
            from rax_integration.teacher.world import verify_world_snapshot
            manifest = verify_world_snapshot(Path(config.root)/'teacher_world')
            self.teacher_world_id = manifest['environment_id']
            identity_file = Path(config.root)/'teacher_memory_identity.json'
            if identity_file.exists():
                from rax_integration.teacher.identity import identity_id
                self.teacher_memory_id = identity_id(json.loads(identity_file.read_text()))
        self._action_calls = 0
        self._action_rows = 0
        fold = Path(config.root)/'fold_policy.json'
        self.fold_policy = json.loads(fold.read_text()) if fold.exists() else None

    def tools(self): return self.contract.select(T.tool_definitions())
    @staticmethod
    def validate(name, arguments): T.validate_request(name, arguments)

    def log(self, event, **fields):
        with (self.output/'operator.jsonl').open('a') as f:
            f.write(json.dumps(jsonable({'event': event, 'wall_unix_s': time.time(),
                                       'episode_id': self.episode_id, 'step_id': self.step_id,
                                       'sim_time_s': float(self.env.data.time), **fields}), allow_nan=False)+'\n')

    def start(self, *, formal, seed):
        if self.env is not None: self.close()
        sys.path.insert(0, str(PROJECT)) if str(PROJECT) not in sys.path else None
        from environment import FoldingBoxEnv
        self.episode_id = 'dexmate_'+uuid.uuid4().hex
        self.output = self.root/self.episode_id
        self.output.mkdir(mode=0o700)
        self.started = time.monotonic()
        sources_before_load = fingerprints()
        self.env = FoldingBoxEnv(seed=seed, image_obs=False,
                                 placement='flat' if self.contract.teacher_privileged else 'centered')
        if self.contract.teacher_privileged:
            from rax_integration.teacher.world import world_sources, digest
            if digest(world_sources(PROJECT)) != self.teacher_world_id:
                self.env.close(); self.env = None
                raise RuntimeError('Teacher world snapshot does not match loaded world')
        if sources_before_load != fingerprints():
            self.env.close()
            self.env = None
            raise RuntimeError('Task sources changed during environment creation; wait for the scene rebuild to finish')
        self.step_id = 0
        self.motion = None
        self._last_control_mode = 'cartesian_ik'
        self.finished = False
        self.termination_reason = None
        self._closed = False
        self._last_video_step = -1
        self._video_index = 0
        self.wall_limit = (self.config.formal if formal else self.config.development).wall_seconds
        # The scene, cameras, cardboard, solver and materials are exactly those loaded by the task.
        self.source_hashes = sources_before_load
        (self.output/'source_hashes.json').write_text(json.dumps(self.source_hashes, indent=2))
        # Preserve the loaded source/configuration and compiled model even if a
        # separate geometry/material worker rebuilds the shared task afterward.
        snapshot = self.output/'source_snapshot'
        for name in ('environment.py','robot_config.json','physics.json','box_geometry.py',
                     'geometry/panels.json','models/scene.xml'):
            raw = (PROJECT/name).read_bytes()
            if hashlib.sha256(raw).hexdigest() != self.source_hashes[name]:
                self.env.close(); self.env = None
                raise RuntimeError('Task sources changed while capturing the episode snapshot')
            target = snapshot/name; target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes(raw)
        import mujoco
        mujoco.mj_saveModel(self.env.model,str(self.output/'loaded_model.mjb'),None)
        self.log('reset', seed=seed, mode='custom_check' if formal else 'exploration',
                 physical_progress=self.env.progress(), action_layout=self.action_layout(),
                 runtime_configuration={'robot':self.env.cfg,'physics':self.env.physical_params})
        w, h = self.env.render_spec.width, self.env.render_spec.height
        self.video_log = (self.output/'video_encoder.log').open('wb')
        self.video = subprocess.Popen(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
            '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{w}x{h}', '-r', '10', '-i', '-',
            '-an', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '23', '-pix_fmt', 'yuv420p',
            str(self.output/'episode.mp4')], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
            stderr=self.video_log)
        self._video_frame()
        return self._observe()

    def _publish_live(self, phase):
        if not self.fold_policy or not self.fold_policy.get('live_view'): return
        import os
        target = Path(self.config.root)/'live_state.npz'
        with target.with_suffix('.tmp').open('wb') as stream:
            np.savez(stream, qpos=self.env.data.qpos, qvel=self.env.data.qvel,
                     time=float(self.env.data.time), step=self.step_id, phase=phase,
                     model_path=str(self.output/'loaded_model.mjb'),
                     episode=self.episode_id, wall=time.time())
        os.replace(target.with_suffix('.tmp'), target)

    def _video_frame(self):
        if self.video is None or self.step_id == self._last_video_step: return
        self.video.stdin.write(self.env.render_camera('back').tobytes())
        self._last_video_step = self.step_id
        self._publish_live('executing RAX action' if self.step_id else 'awaiting model action')
        self.log('video_frame', frame_index=getattr(self, '_video_index', 0), camera='back')
        self._video_index = getattr(self, '_video_index', 0)+1

    def _hold_row(self):
        return np.concatenate([np.r_[self.env.targets[s], self.env.grippers[s]['command']]
                               for s in ('right', 'left')]).tolist()

    def action_layout(self):
        if self.env is None: return T.action_layout()
        return T.action_layout(self.env.control_dt,float(self.env.model.opt.timestep),
                               self.env.cfg.get('gripper_closing_torque_Nm',0))

    def status(self):
        return {'task': self.config.task, 'task_kind': 'custom task integrated with RAX',
                'official_dexjoco_benchmark': False,
                'robot': 'Dexmate Vega 1 U, two 7-joint arms, two coupled DexGripper S jaws',
                'active': self.env is not None and not self.finished, 'episode_ended': self.finished,
                'episode_id': self.episode_id, 'step_id': self.step_id,
                'action_call_count':self._action_calls,'actual_action_rows':getattr(self,'_action_rows',0),
                'action_layout': self.action_layout(), 'cameras': list(T.CAMERAS),
                'state_order': 'right then left, exactly as the existing environment',
                'control_frequency_hz': self.action_layout()['timing']['control_frequency_hz'],
                'physics_seconds_per_step': self.action_layout()['timing']['control_dt_s'],
                'episode_step_limit': int(round(self.env.cfg['episode_seconds']/self.env.control_dt)) if self.env else None,
                'max_action_rows': T.MAX_ROWS,
                'physical_locking': 'unvalidated; existing task withholds success',
                'camera_calibration': 'provisional virtual optics, RGB only'}

    def _control_feedback(self):
        # Solver scratch-pose residual and live-physics tracking are distinct.
        tracking={}
        for side in ('right','left'):
            actual=np.asarray(self.env.tcp_pose(side),dtype=float)
            reference=np.asarray(self.env.targets[side],dtype=float)
            valid=(actual.shape==(7,) and reference.shape==(7,) and
                   np.isfinite(actual).all() and np.isfinite(reference).all() and
                   np.linalg.norm(actual[3:])>1e-12 and np.linalg.norm(reference[3:])>1e-12)
            if valid:
                delta=actual[:3]-reference[:3]
                dot=abs(float(np.dot(actual[3:]/np.linalg.norm(actual[3:]),reference[3:]/np.linalg.norm(reference[3:]))))
                position=float(np.linalg.norm(delta));orientation=float(2*np.arccos(np.clip(dot,0.,1.)))
                tracking[side]={'data_valid':True,'actual_minus_reference_position_m':delta.tolist(),
                    'position_m':position,'orientation_rad':orientation,
                    'within_reference_tolerance':bool(position<=.005 and orientation<=.05)}
            else:
                tracking[side]={'data_valid':False,'actual_minus_reference_position_m':None,
                    'position_m':None,'orientation_rad':None,'within_reference_tolerance':False}
        return {'mode':self._last_control_mode,
                'ik_residual':self.env.ik_errors if self._last_control_mode=='cartesian_ik' else None,
                'ik_residual_semantics':'Residual of IK scratch solution; NOT actual TCP tracking error or proof of arrival.',
                'tcp_references':self.env.targets,'reference_tracking':tracking,
                'reference_tracking_semantics':'Measured TCP versus current commanded TCP reference. For planned motion this is the current joint-reference FK, not the final trajectory goal.',
                'reference_tolerance':{'position_m':.005,'orientation_rad':.05}}

    def _observe(self, cameras=T.CAMERAS, transition=None):
        reject_unless(self.env is not None, 'Start an episode first')
        observation = self.env._compute_observation()
        # Preserve RAX's proprioception-only model interface. Raw task state remains in operator.jsonl.
        state = {k: v for k, v in observation['state'].items()
                 if k in ('tcp_pose', 'arm_joint_positions', 'arm_joint_velocities', 'gripper_pose')}
        state['gripper_opening_commands'] = {s: self.env.grippers[s]['command'] for s in ('right', 'left')}
        mapping = {'ego': 'head', 'back': 'back', 'wrist_left': 'handcam_rgb_left', 'wrist_right': 'handcam_rgb_right'}
        images, attachments, image_files = [], [], []
        ident = uuid.uuid4().hex[:10]
        directory = self.output/'observations'/f'{self.step_id:06d}_{ident}'
        directory.mkdir(parents=True)
        for camera in cameras:
            rgb = self.env.render_camera(mapping[camera])
            data = _png_data(rgb)
            images.append({'type': 'image', 'mimeType': 'image/png', 'data': data})
            attachments.append({'kind': 'rgb', 'camera': camera, 'content_index': len(images)})
            path = directory/(camera+'.png')
            path.write_bytes(base64.b64decode(data))
            image_files.append({'camera': camera, 'path': str(path.relative_to(self.output)),
                                'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
        packet = jsonable({'episode_id': self.episode_id, 'step_id': self.step_id,
                  'task': self.config.task, 'state': state, 'images': list(cameras),
                  'attachments': attachments, 'episode_ended': self.finished,
                  'sim_time_s': float(self.env.data.time), 'action_layout': self.action_layout(),
                  'control_feedback': self._control_feedback(),
                  **({'transition': transition} if transition else {})})
        (directory/'observation.json').write_text(json.dumps(packet, indent=2))
        self.log('observation', files=image_files, observation=packet,
                 task_state=observation['state'], physical_progress=self.env.progress())
        self._publish_live('awaiting model decision' if not self.finished else 'attempt stopped')
        if getattr(getattr(self, 'contract', None), 'teacher_privileged', False):
            from rax_integration.teacher.state import teacher_state
            packet['teacher_state'] = teacher_state(self)
            self._pin_teacher_state(packet['teacher_state'])
            (directory/'observation.json').write_text(json.dumps(packet, indent=2))
        return packet, images

    def _pin_teacher_state(self, packet):
        # Immutable per-observation evidence, relative to the memory evidence root.
        name = 'teacher_states/'+str(self.step_id)+'_'+uuid.uuid4().hex+'.json'
        packet['evidence_id'] = self.episode_id+'/'+name
        target = self.output/name
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open('x') as stream:
            json.dump(packet, stream, allow_nan=False)
        target.chmod(0o444)

    def _motion_service(self):
        if self.motion is None:
            from rax_integration.motion_planning.service import MotionService
            policy_file=Path(self.config.root)/'motion_policy.json'
            policy=json.loads(policy_file.read_text()) if policy_file.exists() else {}
            self.motion=MotionService(self.env,self.episode_id,self.config.root,
                on_step=self._motion_row,can_step=self._motion_can_step,
                world_policy=policy.get('world_policy','robot_static'))
        return self.motion

    def _motion_can_step(self):
        if self.finished:return False
        if self.wall_limit is not None and time.monotonic()-self.started>=self.wall_limit:
            self.finished,self.termination_reason=True,'wall_time_limit'
            return False
        return True

    def _teacher_contact_audit(self):
        if not getattr(getattr(self, 'contract', None), 'teacher_privileged', False): return {}
        from rax_integration.teacher.state import contact_snapshot
        return {'teacher_contacts': contact_snapshot(self.env),
                'teacher_contact_semantics': 'control-end sample; substep-only contacts may be missed'}

    def _motion_row(self,entry,obs,info):
        self.step_id=self.env.env_step
        self._action_rows=getattr(self,'_action_rows',0)+1
        self.log('planner_joint_action',**entry,physical_progress=info['progress'],
                 **self._teacher_contact_audit())
        self._publish_live('executing planned joint trajectory')
        if self.step_id%5==0:self._video_frame()
        if entry.get('terminated') or entry.get('truncated'):
            self.finished=True
            self.termination_reason='physical_success' if entry.get('terminated') else 'simulation_time_limit'

    def call(self, name, args):
        self.contract.require_robot(name, args)
        T.validate_request(name, args)
        if name in T.MOTION_TOOLS:
            reject_unless(self.env is not None,'Start an episode first')
            if name not in ['dexmate_motion_status','dexmate_motion_stop'] and self.finished:
                if name=='dexmate_motion_execute':
                    result=self._motion_service().terminate(args['plan_id'],'EPISODE_ENDED')
                    self._write_result();return self._observe(transition={'motion':result})
                reject_unless(False,'Episode ended')
            motion=self._motion_service()
            if name=='dexmate_motion_plan':return motion.plan(args),[]
            if name=='dexmate_motion_preview':return motion.preview(args['plan_id']),[]
            if name=='dexmate_motion_status':return motion.status(args.get('plan_id')),[]
            if name=='dexmate_motion_stop':
                result=motion.stop(args.get('reason','MCP stop'))
                self.finished,self.termination_reason=True,'operator_stop';self._write_result()
                return result,[]
            reject_unless(not self.smoke_policy,'Historical smoke policy does not permit planned joint execution')
            preview=motion.preview(args['plan_id'])
            if preview.get('status')!='preview':
                if preview.get('reason')=='STOP_REQUESTED':
                    self.finished,self.termination_reason=True,'operator_stop';self._write_result()
                return self._observe(transition={'motion':preview})
            cursor_error=motion.cursor_refusal(args['plan_id'],args.get('expected_cursor'))
            if cursor_error:return self._observe(transition={'motion':motion.refuse(**cursor_error)})
            if self.fold_policy and self._action_calls>=self.fold_policy['max_action_tool_calls']:
                result=motion.terminate(args['plan_id'],'ACTION_CALL_LIMIT')
                self.finished,self.termination_reason=True,'action_call_limit';self._write_result()
                return self._observe(transition={'motion':result})
            row_cap=min(50,self.fold_policy['max_action_rows']) if self.fold_policy else 50
            if 'max_rows' in args:reject_unless(args['max_rows']<=row_cap,'Folding action-row cap')
            chunk_rows=min(args.get('max_rows',row_cap),len(preview['joint_positions_rad']))
            if not self._motion_can_step():
                result=motion.terminate(args['plan_id'],'EPISODE_WALL_LIMIT')
                self._write_result()
                return self._observe(transition={'motion':result})
            if hasattr(self.env,'cfg') and hasattr(self.env,'data'):
                remaining=max(0,int(np.floor((self.env.cfg['episode_seconds']-self.env.data.time)/self.env.control_dt+1e-8)))
                if not remaining:
                    result=motion.terminate(args['plan_id'],'SIMULATION_TIME_LIMIT')
                    self.finished,self.termination_reason=True,'simulation_time_limit';self._write_result()
                    return self._observe(transition={'motion':result})
                chunk_rows=min(chunk_rows,remaining)
            self._action_calls+=1
            self.log('tool_call',name=name,arguments=args,requested_rows=chunk_rows,trajectory_remaining_rows=len(preview['joint_positions_rad']),trajectory_cursor=preview.get('cursor',0))
            result=motion.execute(args['plan_id'],max_rows=chunk_rows,expected_cursor=args.get('expected_cursor'));self.step_id=self.env.env_step
            if result.get('executed'):self._last_control_mode='planned_joint_references'
            if result.get('reason')=='STOP_REQUESTED':self.finished,self.termination_reason=True,'operator_stop'
            if self.finished:self._write_result()
            return self._observe(transition={'motion':result})
        if name == 'dexmate_teacher_memory':
            reject_unless(self.contract.teacher_privileged, 'Teacher capability required')
            from rax_integration.teacher.memory_api import call_memory
            return call_memory(self, args), []
        if name == 'dexmate_teacher_state':
            reject_unless(self.env is not None, 'Start an episode first')
            from rax_integration.teacher.state import teacher_state
            packet = teacher_state(self)
            self._pin_teacher_state(packet)
            self.log('teacher_observation', observation=packet)
            return packet, []
        if name == 'dexmate_status': return self.status(), []
        if name == 'dexmate_observe': return self._observe(args.get('cameras', T.CAMERAS))
        reject_unless(self.env is not None and not self.finished, 'Episode ended or not started')
        if name == 'dexmate_step': rows = args['actions']*args.get('repeat',1)
        else:
            row = self._hold_row()
            if name == 'dexmate_gripper':
                for s, value in args['openings'].items(): row[7 if s == 'right' else 15] = value
            rows = [row]*args['steps']
        if self.smoke_policy:
            reject_unless(name == 'dexmate_step', 'Smoke policy permits one Cartesian action call only')
            reject_unless(self._action_calls < self.smoke_policy['max_action_tool_calls'], 'Smoke action call limit')
            reject_unless(len(rows) <= self.smoke_policy['max_action_rows'], 'Smoke action row limit')
            for row in rows:
                for s, i in [('right',0),('left',8)]:
                    pose = self.env.tcp_pose(s)
                    q = np.asarray(row[i+3:i+7]); q = q/np.linalg.norm(q)
                    reject_unless(np.linalg.norm(np.asarray(row[i:i+3])-pose[:3]) <= .005,
                                  'Smoke translation limit is 5 mm from the current measured TCP')
                    reject_unless(abs(np.dot(q,pose[3:7])) > 1-1e-6, 'Smoke must preserve measured orientation')
                    reject_unless(abs(row[i+7]-self.env.grippers[s]['command']) <= .05,
                                  'Smoke gripper opening change exceeds 0.05')
            self._action_calls += 1
        if self.fold_policy:
            reject_unless(self._action_calls < self.fold_policy['max_action_tool_calls'], 'Folding action-call cap')
            reject_unless(len(rows) <= self.fold_policy['max_action_rows'], 'Folding action-row cap')
            self._action_calls += 1
        reject_unless(not (Path(self.config.root)/'motion_planning/STOP').exists(),'Operator STOP requested')
        reject_unless(self.motion is None or not (self.motion.active or self.motion.owner_plan),'Planned motion owns both arms, including paused continuation')
        reject_unless(self.motion is None or not self.motion.recovery_required,'Tracking/guard stop requires explicit operator-authorized reset/recommissioning; Cartesian hold is not recovery')
        self._last_control_mode='cartesian_ik'
        self.log('tool_call', name=name, arguments=args)
        executed = []
        for row in rows:
            tick = time.monotonic()
            if (Path(self.config.root)/'motion_planning/STOP').exists():
                self.finished,self.termination_reason=True,'operator_stop';break
            if self.wall_limit is not None and time.monotonic()-self.started >= self.wall_limit:
                self.finished, self.termination_reason = True, 'wall_time_limit'
                break
            action = {s: np.asarray(row[i:i+8], dtype=float) for s, i in [('right', 0), ('left', 8)]}
            obs, reward, terminated, truncated, info = self.env.step(action)
            self.step_id = self.env.env_step
            self._action_rows=getattr(self,'_action_rows',0)+1
            executed.append(self._hold_row())
            finite_state = np.isfinite(self.env.data.qpos).all() and np.isfinite(self.env.data.qvel).all()
            warnings = {str(i): int(w.number) for i, w in enumerate(self.env.data.warning) if w.number}
            self.log('action', requested_row=row, executed_row=executed[-1],
                     motor_references={s: a['qcmd'] for s, a in self.env.arms.items()},
                     joint_positions=obs['state']['arm_joint_positions'], motor_controls=self.env.data.ctrl,
                     task_state=obs['state'], physical_progress=info['progress'],
                     ik_residual=info['ik_residual'], reward=reward,
                     terminated=terminated, truncated=truncated, physics_warnings=warnings,
                     **self._teacher_contact_audit())
            if self.step_id % 5 == 0: self._video_frame()
            if self.fold_policy and self.fold_policy.get('live_view'): time.sleep(max(0.,self.env.control_dt-(time.monotonic()-tick)))
            if terminated or truncated or not finite_state or warnings:
                self.finished = True
                self.termination_reason = ('physical_success' if terminated else 'simulation_time_limit' if truncated
                                           else 'physics_invalid_state' if not finite_state else 'physics_warning')
                break
        if self.finished: self._write_result()
        return self._observe(transition={'requested_rows': len(rows), 'executed_rows': len(executed),
                     'executed_actions': executed, 'episode_ended': self.finished,
                     'termination_reason': self.termination_reason})

    def evaluate(self, agent_assessment=None, confidence=None):
        if self.env is None: raise RuntimeError('No custom episode to evaluate')
        progress = self.env.progress()
        self.log('evaluation', physical_progress=progress, agent_assessment=agent_assessment)
        return {'status': 'completed', 'evaluation_kind': 'custom_physical_box_folding',
                'task_complete': bool(progress['succeed']), 'episode_id': self.episode_id,
                'step_id': self.step_id, 'task': self.config.task,
                'completion_blocker': 'physical locking has not passed independent validation',
                'official_dexjoco_benchmark': False}

    def _write_result(self):
        progress = self.env.progress()
        result = {'episode_id': self.episode_id, 'step_id': self.step_id, 'sim_time_s': float(self.env.data.time),
                  'termination_reason': self.termination_reason,
                  'integration_executed': self.step_id > 0, 'partial_task_progress': progress,
                  'complete_folding_with_physical_locking': bool(progress['succeed']),
                  'task_kind': 'custom task integrated with RAX', 'official_dexjoco_benchmark': False,
                  'source_unchanged': fingerprints() == self.source_hashes}
        (self.output/'result.json').write_text(json.dumps(jsonable(result), indent=2))
        return result

    def close(self):
        if self.env is None or getattr(self, '_closed', False): return
        try:
            if not self.finished:
                self.finished = True
                self.termination_reason = 'operator_or_agent_finish'
            self._video_frame()
            self._write_result()
            self._publish_live('attempt finished')
            self.log('episode_end', termination_reason=self.termination_reason)
        finally:
            if self.video is not None:
                self.video.stdin.close()
                code = self.video.wait(timeout=20)
                self.video = None
                self.video_log.close()
                if code: raise RuntimeError('Episode video encoding failed; see video_encoder.log')
            self.env.close()
            self._closed = True
