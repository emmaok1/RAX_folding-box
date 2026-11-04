"""Explicit static diagnostic scene, NOT the cardboard task or avoidance evidence.
The robot/table/cameras and controller method are unchanged. The dynamic box
subtree is omitted by construction in this isolated fixture, never the task XML.
"""
import json,numpy as np,mujoco
from environment import FoldingBoxEnv
from rax_integration.motion_planning.model import ROOT,static_xml,robot_snapshot,StaticRobotModel

class StaticRobotFixture(FoldingBoxEnv):
    static_planning_fixture=True
    def __init__(self):
        self.fixture_xml,_=static_xml();self.model=mujoco.MjModel.from_xml_string(self.fixture_xml);self.data=mujoco.MjData(self.model);self._model=self.model;self._data=self.data;self.cfg=json.loads((ROOT/'robot_config.json').read_text());self.control_dt=self.cfg['control_dt_s'];self._n_substeps=round(self.control_dt/self.model.opt.timestep);self._ik_data=mujoco.MjData(self.model);self._renderer=None;self.arms={};self.grippers={};self.env_step=0
        for side,prefix in [('right','R'),('left','L')]:
            ids=np.array([self.model.joint(f'{prefix}_arm_j{i}').id for i in range(1,8)]);qad=self.model.jnt_qposadr[ids];vid=self.model.jnt_dofadr[ids];act=np.array([self.model.actuator(f'motor_{prefix}_arm_j{i}').id for i in range(1,8)]);home=np.array(self.cfg['home_'+side+'_rad']);self.data.qpos[qad]=home;self.arms[side]={'prefix':prefix,'joint_ids':ids,'qad':qad,'vid':vid,'act':act,'limits':self.model.jnt_range[ids],'site':self.model.site('tcp_'+prefix).id,'qcmd':home.copy(),'velocity':np.array([2.4,2.4,2.7,2.7,2.7,2.7,2.7])}
            ids=np.array([self.model.joint(f'{prefix}_gripper_j{i}').id for i in [1,2]]);self.grippers[side]={'qad':self.model.jnt_qposadr[ids],'vid':self.model.jnt_dofadr[ids],'act':self.model.actuator(f'motor_{prefix}_gripper_j1').id,'command':.3,'qcmd':.288,'effort_scale':1.};self.data.qpos[self.grippers[side]['qad']]=.288
        mujoco.mj_forward(self.model,self.data);self.targets={s:self.tcp_pose(s) for s in self.arms};self.ik_errors={s:{'position_m':0.,'orientation_rad':0.} for s in self.arms}
    def _compute_observation(self):return {'state':{'tcp_pose':np.concatenate([self.tcp_pose(s) for s in ['right','left']])}}
    def progress(self):return {'succeed':False,'fixture':'static diagnostic; no cardboard'}
    def inverse_kinematics(self,*args,**kwargs):raise AssertionError('Joint planner execution must never call Cartesian IK')

def nearby_target(env,side):
    model=StaticRobotModel(env.fixture_xml,robot_snapshot(env));q=env.data.qpos[env.arms[side]['qad']].copy();q[1]+=.018 if side=='right' else -.018;model.set_arm(side,q);p,R=model.pose(side)
    return {'frame':'world','tcp':'tcp_R' if side=='right' else 'tcp_L','position_m':p.tolist(),'quaternion_wxyz':R.as_quat()[[3,0,1,2]].tolist()}
