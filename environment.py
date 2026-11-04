"""Current Dexmate Vega 1 U bimanual cardboard folding environment.
Robot execution controls only arm and gripper motors. Box joints are passive.
"""
from pathlib import Path
from dataclasses import dataclass
import json
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from gymnasium import Env,spaces
from box_geometry import capture_geometry
ROOT=Path(__file__).resolve().parent
@dataclass(frozen=True)
class GymRenderingSpec:
    width:int=640
    height:int=480

class FoldingBoxEnv(Env):
    metadata={'render_modes':['rgb_array','human'],'render_fps':50}
    def __init__(self,seed=0,image_obs=False,render_mode='rgb_array',render_spec=GymRenderingSpec(),placement='centered'):
        self.cfg=json.loads((ROOT/'robot_config.json').read_text());self.physical_params=json.loads((ROOT/'physics.json').read_text())
        self.geometry=json.loads((ROOT/'geometry/panels.json').read_text());self.panels=[p['panel'] for p in self.geometry['panels']]
        self.model=mujoco.MjModel.from_xml_path(str((ROOT/'models/scene.xml').resolve()));self.data=mujoco.MjData(self.model)
        self._model=self.model;self._data=self.data
        self.model.opt.timestep=self.cfg['physics_dt_s'];self.control_dt=self.cfg['control_dt_s'];self._n_substeps=int(round(self.control_dt/self.model.opt.timestep))
        self.model.vis.global_.offwidth=render_spec.width;self.model.vis.global_.offheight=render_spec.height
        self.render_spec=render_spec;self.image_obs=image_obs;self.render_mode=render_mode;self.placement=placement;self._renderer=None
        self.arms={};self.grippers={};self.targets={};self.ik_errors={};self._ik_data=mujoco.MjData(self.model)
        for side,prefix in [('left','L'),('right','R')]:
            names=[f'{prefix}_arm_j{i}' for i in range(1,8)];ids=np.array([self.model.joint(n).id for n in names]);qad=self.model.jnt_qposadr[ids];vid=self.model.jnt_dofadr[ids]
            limits=self.model.jnt_range[ids].copy();act=np.array([self.model.actuator('motor_'+n).id for n in names])
            self.arms[side]={'prefix':prefix,'joint_ids':ids,'qad':qad,'vid':vid,'act':act,'limits':limits,'site':self.model.site('tcp_'+prefix).id,'qcmd':np.zeros(7),'velocity':np.array([2.4,2.4,2.7,2.7,2.7,2.7,2.7])}
            js=[self.model.joint(f'{prefix}_gripper_j{i}').id for i in [1,2]]
            self.grippers[side]={'qad':self.model.jnt_qposadr[js],'vid':self.model.jnt_dofadr[js],'act':self.model.actuator('motor_'+prefix+'_gripper_j1').id,'command':.3,'qcmd':.288,'effort_scale':1.}
        self.flex_names=[self.model.joint(i).name for i in range(self.model.njnt) if (self.model.joint(i).name or '').startswith('flex_')]
        self.camera_names=['head','handcam_rgb_left','handcam_rgb_right'];self.camera_id=tuple(self.model.camera(n).id for n in self.camera_names)
        lo=np.r_[np.full(3,-np.inf),np.full(4,-1.),0.];hi=np.r_[np.full(3,np.inf),np.ones(4),1.]
        self.action_space=spaces.Dict({s:spaces.Box(lo,hi,dtype=np.float64) for s in self.arms})
        self.observation_space=spaces.Dict({'state':spaces.Dict({n:spaces.Box(-np.inf,np.inf,(size,),np.float64) for n,size in [('tcp_pose',14),('arm_joint_positions',14),('arm_joint_velocities',14),('gripper_pose',4),('box_pose',7),('hinge_angles',len(self.panels)-1),('flex_angles',len(self.flex_names))]})})
        if image_obs:self.observation_space.spaces['images']=spaces.Dict({n:spaces.Box(0,255,(render_spec.height,render_spec.width,3),np.uint8) for n in ['ego','wrist_left','wrist_right']})
        self.reset(seed=seed)

    def tcp_pose(self,side,data=None):
        d=self.data if data is None else data;a=self.arms[side];q=np.empty(4);mujoco.mju_mat2Quat(q,d.site_xmat[a['site']]);return np.r_[d.site_xpos[a['site']],q]

    def reset(self,*,seed=None,options=None):
        super().reset(seed=seed);mujoco.mj_resetData(self.model,self.data)
        if options and 'placement' in options:self.placement=options['placement']
        for s,a in self.arms.items():
            home=np.array(self.cfg['home_'+s+'_rad']);self.data.qpos[a['qad']]=home;a['qcmd']=home.copy()
            g=self.grippers[s];self.data.qpos[g['qad']]=.288;g['qcmd']=.288;g['command']=.3;g['effort_scale']=1.
        if self.placement not in ['centered','flat']:
            raise ValueError('Current video rebuild supports centered or flat reset only.')
        if self.placement=='centered':
            for n,angle in self.geometry['material_layout']['initial_angles_deg'].items():
                self.data.joint('crease_'+n).qpos[0]=np.deg2rad(angle)
        mujoco.mj_forward(self.model,self.data)
        self.targets={s:self.tcp_pose(s) for s in self.arms};self.ik_errors={s:{'position_m':0.,'orientation_rad':0.} for s in self.arms}
        self.env_step=0;self._release_since=None
        return self._compute_observation(),{'succeed':False,'robot':'Dexmate Vega 1 U','placement':self.placement}

    def inverse_kinematics(self,side,target,iterations=24,seed=None):
        """Robot-only kinematic scratch data; never writes execution/object state."""
        a=self.arms[side];d=self._ik_data;d.qpos[:]=self.data.qpos;q=a['qcmd'].copy() if seed is None else np.asarray(seed).copy()
        desired=Rotation.from_quat(np.asarray(target[3:7])[[1,2,3,0]])
        for _ in range(iterations):
            d.qpos[a['qad']]=q;mujoco.mj_kinematics(self.model,d);mujoco.mj_comPos(self.model,d)
            pos=target[:3]-d.site_xpos[a['site']];rot=(desired*Rotation.from_matrix(d.site_xmat[a['site']].reshape(3,3)).inv()).as_rotvec()
            if np.linalg.norm(pos)<.0001 and np.linalg.norm(rot)<.001:break
            jp=np.zeros((3,self.model.nv));jr=np.zeros_like(jp);mujoco.mj_jacSite(self.model,d,jp,jr,a['site'])
            J=np.vstack([jp[:,a['vid']],.25*jr[:,a['vid']]]);err=np.r_[pos,.25*rot]
            dq=J.T@np.linalg.solve(J@J.T+.0001*np.eye(6),err)
            q=np.clip(q+np.clip(dq,-.15,.15),a['limits'][:,0]+.005,a['limits'][:,1]-.005)
        d.qpos[a['qad']]=q;mujoco.mj_kinematics(self.model,d)
        pos=target[:3]-d.site_xpos[a['site']];rot=(desired*Rotation.from_matrix(d.site_xmat[a['site']].reshape(3,3)).inv()).as_rotvec()
        return q,{'position_m':float(np.linalg.norm(pos)),'orientation_rad':float(np.linalg.norm(rot))}

    def _set_action(self,action):
        if action is None:return
        for s in self.arms:
            x=np.asarray(action[s],dtype=float)
            if x.shape!=(8,) or not np.isfinite(x).all():raise ValueError('Each side needs finite [xyz, wxyz, open_fraction] (8 values).')
            if not np.allclose(x[:7],0):
                n=np.linalg.norm(x[3:7])
                if n<1e-6:raise ValueError('TCP quaternion must be nonzero.')
                self.targets[s]=np.r_[x[:3],x[3:7]/n]
            self.grippers[s]['command']=float(np.clip(x[7],0,1))

    def step(self,action=None):
        self._set_action(action)
        for s,a in self.arms.items():
            q,self.ik_errors[s]=self.inverse_kinematics(s,self.targets[s]);delta=a['velocity']*self.control_dt
            a['qcmd']+=np.clip(q-a['qcmd'],-delta,delta)
            g=self.grippers[s];delta_g=self.cfg['gripper_max_speed_rad_s']*self.control_dt
            g['qcmd']+=float(np.clip(.96*g['command']-g['qcmd'],-delta_g,delta_g))
        return self._advance()

    def step_joint_targets(self,joints,grippers=None):
        """Explicit motor-reference interface for deterministic robot commissioning.

        This changes references, never qpos/qvel or any box state. Same torque and
        velocity limits and physical substeps as Cartesian step().
        """
        for s,a in self.arms.items():
            q=np.asarray(joints.get(s,a['qcmd']),dtype=float)
            if q.shape!=(7,) or not np.isfinite(q).all():raise ValueError('Expected seven finite arm joint references.')
            q=np.clip(q,a['limits'][:,0],a['limits'][:,1]);delta=a['velocity']*self.control_dt
            a['qcmd']+=np.clip(q-a['qcmd'],-delta,delta)
            g=self.grippers[s]
            if grippers is not None and s in grippers:g['command']=float(np.clip(grippers[s],0,1))
            dg=self.cfg['gripper_max_speed_rad_s']*self.control_dt
            g['qcmd']+=float(np.clip(.96*g['command']-g['qcmd'],-dg,dg))
            self._ik_data.qpos[:]=self.data.qpos;self._ik_data.qpos[a['qad']]=a['qcmd'];mujoco.mj_kinematics(self.model,self._ik_data)
            self.targets[s]=self.tcp_pose(s,self._ik_data)
        return self._advance()

    def _advance(self):
        kp=np.array(self.cfg['joint_kp_Nm_rad']);kd=np.array(self.cfg['joint_kd_Nms_rad'])
        for _ in range(self._n_substeps):
            for s,a in self.arms.items():
                u=kp*(a['qcmd']-self.data.qpos[a['qad']])-kd*self.data.qvel[a['vid']]+self.data.qfrc_bias[a['vid']]
                ranges=self.model.actuator_ctrlrange[a['act']];self.data.ctrl[a['act']]=np.clip(u,ranges[:,0],ranges[:,1])
                g=self.grippers[s];u=self.cfg['gripper_kp_Nm_rad']*(g['qcmd']-self.data.qpos[g['qad'][0]])-self.cfg['gripper_kd_Nms_rad']*self.data.qvel[g['vid'][0]]+self.data.qfrc_bias[g['vid'][0]]
                if g['command']<=.001:u=-self.cfg.get('gripper_closing_torque_Nm',0)*g['effort_scale']-self.cfg['gripper_kd_Nms_rad']*self.data.qvel[g['vid'][0]]+self.data.qfrc_bias[g['vid'][0]]
                self.data.ctrl[g['act']]=np.clip(u,-self.cfg['gripper_max_torque_Nm'],self.cfg['gripper_max_torque_Nm'])
            mujoco.mj_step(self.model,self.data)
        self.env_step+=1;pr=self.progress();obs=self._compute_observation();success=pr['succeed'];truncated=self.data.time>=self.cfg['episode_seconds']
        return obs,float(success),bool(success),bool(truncated and not success),{'succeed':success,'progress':pr,'ik_residual':self.ik_errors}

    def _compute_observation(self):
        state={'tcp_pose':np.concatenate([self.tcp_pose(s) for s in ['right','left']]),
            'arm_joint_positions':np.concatenate([self.data.qpos[self.arms[s]['qad']] for s in ['right','left']]),
            'arm_joint_velocities':np.concatenate([self.data.qvel[self.arms[s]['vid']] for s in ['right','left']]),
            'gripper_pose':np.concatenate([self.data.qpos[self.grippers[s]['qad']] for s in ['right','left']]),
            'box_pose':self.data.joint('box_free').qpos.copy(),
            'hinge_angles':np.array([float(self.data.joint('crease_'+n).qpos[0]) for n in self.panels if n!='base']),
            'flex_angles':np.array([float(self.data.joint(n).qpos[0]) for n in self.flex_names])}
        o={'state':state}
        if self.image_obs:o['images']=dict(zip(['ego','wrist_left','wrist_right'],self.render()))
        return o

    def render(self):return [self.render_camera(n) for n in self.camera_names]
    def render_camera(self,name='back'):
        if self._renderer is None:self._renderer=mujoco.Renderer(self.model,self.render_spec.height,self.render_spec.width)
        self._renderer.update_scene(self.data,camera=name);return self._renderer.render().copy()

    def progress(self):
        angles={n:float(self.data.joint('crease_'+n).qpos[0]) for n in self.panels if n!='base'}
        near=lambda ns,t:all(abs(angles[n]-t)<.22 for n in ns)
        walls=['body_back','body_front','side_left','side_right'];corners=[n for n in angles if 'corner' in n]
        counts={'table':0,'hand':0,'robot':0};penetration=0.
        for c in self.data.contact:
            gs=[int(c.geom1),int(c.geom2)];names=[self.model.geom(g).name or '' for g in gs]
            if not any(n.startswith('card_') for n in names):continue
            k=0 if names[0].startswith('card_') else 1;bn=self.model.body(int(self.model.geom_bodyid[gs[1-k]])).name or ''
            counts['table']+=names[1-k]=='table_collision';counts['robot']+=bn.startswith('dexmate_');counts['hand']+=bn.startswith('dexmate_') and 'gripper' in bn;penetration=max(penetration,-float(c.dist))
        base=self.data.body('box_base');upright=base.xmat.reshape(3,3)[2,2]>.95;v=int(self.model.joint('box_free').dofadr[0]);speed=float(np.linalg.norm(self.data.qvel[v:v+6]));hinge_speed=float(np.max(abs(self.data.qvel[v+6:])));slow=speed<.03 and hinge_speed<.05
        open_geom=bool(upright and near(walls,np.pi/2) and near(corners,np.pi/2));closed=bool(open_geom and near(['lid','lid_front','lid_side_left','lid_side_right'],np.pi/2))
        tabs=capture_geometry(self);capture=all(x['geometric_capture_candidate'] for x in tabs)
        if closed and capture and counts['robot']==0 and slow and penetration<.0001:
            if self._release_since is None:self._release_since=self.data.time
        else:self._release_since=None
        persisted=bool(self._release_since is not None and self.data.time-self._release_since>=self.cfg['release_seconds'])
        return {'design':'video_rebuild','stabilized':bool(counts['table'] and speed<.03),'body_walls_and_corners':open_geom,'stable_open_box_geometry':bool(open_geom and slow and penetration<.0001),'closed_geometry':closed,'tab_slot_relationships':tabs,'locking_geometry_candidate':capture,'released_persistence':persisted,'required_release_seconds':self.cfg['release_seconds'],'hand_panel_contacts':int(counts['hand']),'robot_panel_contacts':int(counts['robot']),'table_panel_contacts':int(counts['table']),'max_box_hinge_speed_rad_s':hinge_speed,'max_box_penetration_m':penetration,'physical_locking_verified':False,'succeed':False,'note':'Video-inspired rigid-panel approximation. Geometric closure alone does not establish physical locking.'}
    def _compute_success_metrics(self):return self.progress()['succeed']
    def _compute_success(self):return self._compute_success_metrics()
    def close(self):
        if self._renderer is not None:self._renderer.close();self._renderer=None
