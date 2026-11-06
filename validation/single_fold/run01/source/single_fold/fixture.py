"""Original base + original passive side_left crease; contact-only rigid fixture."""
from pathlib import Path
import copy,json,sys,xml.etree.ElementTree as ET
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'experiments/grasp_skill'))
from scene import DiagnosticEnv
PAIR=['box_base','box_side_left'];CREASE='crease_side_left';PANEL='box_side_left';CARD='card_side_left_0'
BASE_CENTER=np.array([.09,-.65,1.1422])
# All dimensions below are full sizes in world axes, fixed furniture/fixture.
FIXTURE=[
 ('support_top',[.0905,-.65,1.03],[.299,.240,.220]), # x[-.059,.240], 1mm from hinge
 ('fixture_cap',[.105,-.65,1.1493],[.270,.2364,.010]), # underside1.1443, initial100um top gap
 ('fixture_y_minus',[.0905,-.7642,1.14715],[.299,.008,.0143]),
 ('fixture_y_plus',[.0905,-.5358,1.14715],[.299,.008,.0143]),
 ('fixture_far_stop',[.2442,-.65,1.14715],[.008,.2364,.0143]),
]

def fold_xml():
 root=ET.parse(ROOT/'models/scene.xml').getroot()
 for node in root.iter():
  if 'file' in node.attrib:node.set('file',str(ROOT/'models'/node.get('file')))
 world=root.find('worldbody');base=world.find("body[@name='box_base']")
 for child in list(base):
  if child.tag=='body' and child.get('name')!=PANEL:base.remove(child)
 base.set('pos',' '.join(map(str,BASE_CENTER)));base.set('quat','1 0 0 0')
 tendon=root.find('tendon')
 for child in list(tendon):
  if child.get('name')!='passive_reverse_side_left':tendon.remove(child)
 contact=root.find('contact')
 for child in list(contact):
  if child.tag=='pair' and child.get('geom2') not in ['card_base_0',CARD]:contact.remove(child)
 fixture=ET.SubElement(world,'body',name='single_fold_fixture')
 for name,pos,size in FIXTURE:
  ET.SubElement(fixture,'geom',name=name,type='box',pos=' '.join(map(str,pos)),size=' '.join(str(x/2) for x in size),rgba='.28 .36 .43 1',friction='1 .005 .0001')
  for card in ['card_base_0',CARD]:
   pair=copy.deepcopy(contact.find(f"pair[@geom1='table_collision'][@geom2='{card}']"));pair.set('geom1',name);contact.append(pair)
 ET.SubElement(world,'camera',name='diagnostic',pos='0.6 -1.1 1.65',xyaxes='.74 .67 0 -.26 .29 .92',fovy='48')
 return ET.tostring(root,encoding='unicode')

class FoldEnv(DiagnosticEnv):
 def __init__(self):super().__init__(xml_string=fold_xml())


def grasp_candidate(env):
 body=env.data.body(PANEL);Rpanel=body.xmat.reshape(3,3);R=Rpanel@np.array([[0,0,1],[0,-1,0],[1,0,0.]])
 position=body.xpos+Rpanel@np.array([-.078,0,0])-R@np.array([-.000007,-.0015,-.0013731])
 target=np.r_[position,Rotation.from_matrix(R).as_quat()[[3,0,1,2]]];pre=target.copy();pre[:3]-=R[:,2]*.055
 qpre,epre=env.inverse_kinematics('right',pre,iterations=250);qgrasp,egrasp=env.inverse_kinematics('right',target,iterations=250,seed=qpre)
 return target,pre,qpre,qgrasp,{'pre':epre,'grasp':egrasp}


def audit(env):
 original=ET.parse(ROOT/'models/scene.xml').getroot();new=ET.fromstring(fold_xml());result={}
 for body_name in PAIR:
  b=original.find(f".//body[@name='{body_name}']");n=new.find(f".//body[@name='{body_name}']")
  for tag in ['inertial','geom']:
   result[body_name+'_'+tag+'_exact']=all(ET.tostring(x)==ET.tostring(y) for x,y in zip(b.findall(tag),n.findall(tag)))
 j=original.find(f".//joint[@name='{CREASE}']");k=new.find(f".//joint[@name='{CREASE}']");result['crease_attributes_exact']=j.attrib==k.attrib
 a=original.find("tendon/fixed[@name='passive_reverse_side_left']");b=new.find("tendon/fixed[@name='passive_reverse_side_left']");result['reverse_tendon_exact']=ET.tostring(a)==ET.tostring(b)
 result['original_crease_attributes']=j.attrib;result['original_reverse_tendon']=ET.tostring(a,encoding='unicode')
 model=env.model;baseline=mujoco.MjModel.from_xml_path(str(ROOT/'models/scene.xml'))
 result['compiled_pair_inertia_exact']=all(all(np.array_equal(getattr(baseline.body(n),p),getattr(model.body(n),p)) for p in ['mass','inertia','ipos','iquat']) for n in PAIR)
 result['robot_geom_exact']=all(all(np.array_equal(getattr(baseline.geom(n),p),getattr(model.geom(n),p)) for p in ['pos','quat','size','contype','conaffinity','friction','solref','solimp','margin']) for n in [baseline.geom(i).name for i in range(baseline.ngeom)] if n and n.startswith(('dexmate_','jaw_pad_')))
 result['physics_exact']=all(np.array_equal(getattr(baseline.opt,p),getattr(model.opt,p)) for p in ['gravity','timestep','integrator','noslip_iterations'])
 result['actuator_limits_exact']=np.array_equal(baseline.actuator_ctrlrange,model.actuator_ctrlrange);result['no_object_actuator']=not any(model.joint(int(model.actuator_trnid[i,0])).name in ['box_free',CREASE] for i in range(model.nu))
 result['equality_count']=model.neq;result['object_joints']=[model.joint(i).name for i in range(model.njnt) if model.joint(i).name in ['box_free',CREASE]]
 result['fixture_geometry']=[{'name':n,'center_m':p,'size_m':s} for n,p,s in FIXTURE]
 result['declared_initial_clearances_m']={'bottom':.0002,'cap':.0001,'lateral_each':.0002,'far_stop':.0002,'stand_left_of_base_crease_gap':.001,'cap_from_crease':.03}
 result['base_center_m']=BASE_CENTER.tolist();result['hinge_world_initial_m']=env.data.xanchor[model.joint(CREASE).id].tolist();result['axis_world_initial']=env.data.xaxis[model.joint(CREASE).id].tolist()
 return result

if __name__=='__main__':
 sys.path.insert(0,str(ROOT/'experiments/vega_impedance'));from test_support import Validity,dense
 env=FoldEnv();v=Validity(env);goal,pre,qp,qg,err=grasp_candidate(env)
 result=audit(env);result.update(home_valid=bool(v(env.arms['right']['qcmd'])),target=goal.tolist(),pre=pre.tolist(),ik=err,pre_valid=bool(v(qp)),grasp_valid=bool(v(qg)),insertion_valid=all(v(q) for q in dense([qp,qg],.003)))
 result['initial_penetrating_contacts']=[{'geoms':[env.model.geom(int(c.geom1)).name,env.model.geom(int(c.geom2)).name],'distance_m':float(c.dist)} for c in env.data.contact if c.dist<0]
 out=ROOT/'validation/single_fold/preflight';out.mkdir(exist_ok=True);(out/'scene.xml').write_text(fold_xml());(out/'audit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
