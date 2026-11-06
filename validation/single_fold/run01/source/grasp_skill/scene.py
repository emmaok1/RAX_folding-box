"""Separate, physically supported single-panel scene; baseline robot is copied intact."""
from pathlib import Path
import copy, json, sys, xml.etree.ElementTree as ET
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from environment import FoldingBoxEnv, GymRenderingSpec


def scene_xml(offset=0.):
    root=ET.parse(ROOT/'models/scene.xml').getroot()
    for node in root.iter():
        if 'file' in node.attrib:node.set('file',str(ROOT/'models'/node.get('file')))
    world=root.find('worldbody');base=world.find("body[@name='box_base']")
    for child in list(base):
        if child.tag=='body':base.remove(child)
    base.set('pos',f'0 {-0.65+offset} 1.1422');base.set('quat','1 0 0 0')
    # Fixed, real pedestal rising 220 mm from the unchanged table. Card is free.
    support=ET.SubElement(world,'body',name='diagnostic_support',pos='0 -0.65 1.03')
    ET.SubElement(support,'geom',name='support_top',type='box',size='.065 .075 .11',rgba='.28 .36 .43 1',friction='1 .005 .0001')
    root.remove(root.find('tendon'))
    contact=root.find('contact')
    for c in list(contact):
        if c.tag=='pair' and c.get('geom2')!='card_base_0':contact.remove(c)
    p=copy.deepcopy(contact.find("pair[@geom1='table_collision']"));p.set('geom1','support_top');contact.append(p)
    ET.SubElement(world,'camera',name='diagnostic',pos='0.6 -1.1 1.65',xyaxes='.74 .67 0 -.26 .29 .92',fovy='48')
    return ET.tostring(root,encoding='unicode')


class DiagnosticEnv(FoldingBoxEnv):
    def __init__(self,offset=0.,xml_string=None):
        self.cfg=json.loads((ROOT/'robot_config.json').read_text())
        self.model=mujoco.MjModel.from_xml_string(scene_xml(offset) if xml_string is None else xml_string);self.data=mujoco.MjData(self.model)
        self._model=self.model;self._data=self.data;self.model.opt.timestep=self.cfg['physics_dt_s']
        self.control_dt=self.cfg['control_dt_s'];self._n_substeps=round(self.control_dt/self.model.opt.timestep)
        self.render_spec=GymRenderingSpec();self._renderer=None;self.model.vis.global_.offwidth=640;self.model.vis.global_.offheight=480
        self.arms={};self.grippers={};self.targets={};self.ik_errors={};self._ik_data=mujoco.MjData(self.model)
        for side,prefix in [('left','L'),('right','R')]:
            names=[f'{prefix}_arm_j{i}' for i in range(1,8)];ids=np.array([self.model.joint(n).id for n in names]);qad=self.model.jnt_qposadr[ids];vid=self.model.jnt_dofadr[ids]
            self.arms[side]={'prefix':prefix,'joint_ids':ids,'qad':qad,'vid':vid,'act':np.array([self.model.actuator('motor_'+n).id for n in names]),'limits':self.model.jnt_range[ids].copy(),'site':self.model.site('tcp_'+prefix).id,'qcmd':np.array(self.cfg['home_'+side+'_rad']),'velocity':np.array([2.4,2.4,2.7,2.7,2.7,2.7,2.7])}
            js=[self.model.joint(f'{prefix}_gripper_j{i}').id for i in [1,2]]
            self.grippers[side]={'qad':self.model.jnt_qposadr[js],'vid':self.model.jnt_dofadr[js],'act':self.model.actuator('motor_'+prefix+'_gripper_j1').id,'command':.3,'qcmd':.288,'effort_scale':1.}
            self.data.qpos[qad]=self.arms[side]['qcmd'];self.data.qpos[self.grippers[side]['qad']]=.288
        mujoco.mj_forward(self.model,self.data)
        self.targets={s:self.tcp_pose(s) for s in self.arms};self.env_step=0
    def _compute_observation(self):return {}
    def progress(self):return {'succeed':False}
