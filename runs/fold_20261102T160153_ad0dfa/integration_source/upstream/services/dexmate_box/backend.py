"""Trusted RAX adapter around the actual FoldingBoxEnv; no XML/physics edits.

Unlike the DexJoCo process adapter this runs the existing lightweight environment
in the host frontend process. Claude only reaches it through RAX's serialized MCP
gateway. Task/object state is kept in operator records, not model observations.
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
        policy = Path(config.root)/'smoke_policy.json'
        self.smoke_policy = json.loads(policy.read_text()) if policy.exists() else None
        self._action_calls = 0
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
        self.env = FoldingBoxEnv(seed=seed, image_obs=False)
        if sources_before_load != fingerprints():
            self.env.close()
            self.env = None
            raise RuntimeError('Task sources changed during environment creation; wait for the scene rebuild to finish')
        self.step_id = 0
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
                'action_layout': self.action_layout(), 'cameras': list(T.CAMERAS),
                'state_order': 'right then left, exactly as the existing environment',
                'control_frequency_hz': self.action_layout()['timing']['control_frequency_hz'],
                'physics_seconds_per_step': self.action_layout()['timing']['control_dt_s'],
                'episode_step_limit': int(round(self.env.cfg['episode_seconds']/self.env.control_dt)) if self.env else None,
                'max_action_rows': T.MAX_ROWS,
                'physical_locking': 'unvalidated; existing task withholds success',
                'camera_calibration': 'provisional virtual optics, RGB only'}

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
                  **({'transition': transition} if transition else {})})
        (directory/'observation.json').write_text(json.dumps(packet, indent=2))
        self.log('observation', files=image_files, observation=packet,
                 task_state=observation['state'], physical_progress=self.env.progress())
        self._publish_live('awaiting model decision' if not self.finished else 'attempt stopped')
        return packet, images

    def call(self, name, args):
        self.contract.require_robot(name, args)
        T.validate_request(name, args)
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
        self.log('tool_call', name=name, arguments=args)
        executed = []
        for row in rows:
            tick = time.monotonic()
            if self.wall_limit is not None and time.monotonic()-self.started >= self.wall_limit:
                self.finished, self.termination_reason = True, 'wall_time_limit'
                break
            action = {s: np.asarray(row[i:i+8], dtype=float) for s, i in [('right', 0), ('left', 8)]}
            obs, reward, terminated, truncated, info = self.env.step(action)
            self.step_id = self.env.env_step
            executed.append(self._hold_row())
            finite_state = np.isfinite(self.env.data.qpos).all() and np.isfinite(self.env.data.qvel).all()
            warnings = {str(i): int(w.number) for i, w in enumerate(self.env.data.warning) if w.number}
            self.log('action', requested_row=row, executed_row=executed[-1],
                     motor_references={s: a['qcmd'] for s, a in self.env.arms.items()},
                     joint_positions=obs['state']['arm_joint_positions'], motor_controls=self.env.data.ctrl,
                     task_state=obs['state'], physical_progress=info['progress'],
                     ik_residual=info['ik_residual'], reward=reward,
                     terminated=terminated, truncated=truncated, physics_warnings=warnings)
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
