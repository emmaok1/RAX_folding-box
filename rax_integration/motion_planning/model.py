"""Robot state and collision snapshots with explicit world-policy provenance."""
from pathlib import Path
import hashlib,json,xml.etree.ElementTree as ET
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2]
SIDES=('right','left')

def digest(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()
def source_files():
    return [ROOT/x for x in ['environment.py','robot_model.py','robot_config.json','physics.json','box_geometry.py','geometry/panels.json','models/scene.xml']]+sorted((ROOT/'models').rglob('*'))+sorted(Path(__file__).parent.glob('*.py'))+[Path(__file__).parent/'config.json',ROOT/'rax_integration/requirements-planning.txt']+[ROOT/'rax_integration/upstream/services/dexmate_box'/x for x in ['backend.py','tools.py','registry.py']]+[ROOT/'rax_integration'/x for x in ['fold.py','claude_relay.py']]+sorted((ROOT/'rax_integration/planner_runtime/site-packages').rglob('*.so'))+sorted((ROOT/'rax_integration/planner_runtime/site-packages').glob('ompl-*.dist-info/METADATA'))
def source_epoch():return [(str(p.relative_to(ROOT)),p.stat().st_mtime_ns,p.stat().st_size) for p in source_files() if p.is_file()]
def source_hash():return digest([(str(p.relative_to(ROOT)),hashlib.sha256(p.read_bytes()).hexdigest()) for p in source_files() if p.is_file()])

def static_xml(text=None, *, include_dynamic=False):
    """Drop dynamic subtrees without consulting their transforms or live state."""
    root=ET.fromstring(text if text is not None else (ROOT/'models/scene.xml').read_text());removed=[]
    world=root.find('worldbody')
    def strip(parent):
        for b in list(parent.findall('body')):
            free=b.find('freejoint') is not None or any(j.get('type')=='free' for j in b.findall('joint'))
            if free:removed.append(b.get('name','unnamed_dynamic'));parent.remove(b)
            else:strip(b)
    if not include_dynamic:strip(world)
    joints={j.get('name') for j in world.iter('joint')};geoms={g.get('name') for g in world.iter('geom')};bodies={b.get('name') for b in world.iter('body')}
    for tag in ['contact','equality','tendon','actuator','sensor']:
        parent=root.find(tag)
        if parent is None:continue
        for item in list(parent):
            valid=True
            for n in item.iter():
                for a in ['joint','joint1','joint2']:
                    if n.get(a) and n.get(a) not in joints:valid=False
                for a in ['geom1','geom2']:
                    if n.get(a) and n.get(a) not in geoms:valid=False
                for a in ['body1','body2']:
                    if n.get(a) and n.get(a) not in bodies:valid=False
            if not valid:parent.remove(item)
    for n in root.iter():
        if n.get('file'):n.set('file',str((ROOT/'models'/n.get('file')).resolve()))
    return ET.tostring(root,encoding='unicode'),removed

def robot_snapshot(env):
    """Read named ROBOT DOFs only, never a full qpos/qvel copy or box state."""
    robot={}
    for side in SIDES:
        a,g=env.arms[side],env.grippers[side]
        robot[side]={'q':env.data.qpos[a['qad']].tolist(),'v':env.data.qvel[a['vid']].tolist(),
                     'qcmd':a['qcmd'].tolist(),'jaws':env.data.qpos[g['qad']].tolist(),
                     'jaw_v':env.data.qvel[g['vid']].tolist(),'jaw_reference':float(g['qcmd']),
                     'opening':float(g['command'])}
    return {'robot':robot,'step':int(env.env_step),'time':float(env.data.time)}

def world_snapshot(env):
    """Explicit simulator-assisted collision input; never enabled by default."""
    robot_q={int(i) for side in SIDES for i in np.r_[env.arms[side]['qad'],env.grippers[side]['qad']]}
    joints={}
    for jid in range(env.model.njnt):
        qa=int(env.model.jnt_qposadr[jid]);va=int(env.model.jnt_dofadr[jid])
        if qa in robot_q:continue
        kind=int(env.model.jnt_type[jid]);nq,nv={0:(7,6),1:(4,3),2:(1,1),3:(1,1)}[kind]
        name=env.model.joint(jid).name
        if not name:raise ValueError('Collision-world joints must have stable names')
        joints[name]={'type':kind,'qpos':env.data.qpos[qa:qa+nq].tolist(),'qvel':env.data.qvel[va:va+nv].tolist()}
    return {'provenance':'MuJoCo measured simulator state; privileged local collision input','joints':joints}

def world_is_quiet(world,linear_limit=.02,angular_limit=.05):
    """Snapshot planning does not predict rapidly moving obstacles."""
    for state in world.get('joints',{}).values():
        v=np.asarray(state['qvel']);kind=state['type']
        if not np.isfinite(v).all():return False
        if kind==0:
            if np.linalg.norm(v[:3])>linear_limit or np.linalg.norm(v[3:])>angular_limit:return False
        elif np.max(np.abs(v))>(linear_limit if kind==2 else angular_limit):return False
    return True

def world_matches(a,b,translation_tolerance_m=.001,angle_tolerance_rad=.005):
    """Compare object geometry state without confusing quaternion signs."""
    aj,bj=a.get('joints',{}),b.get('joints',{})
    if set(aj)!=set(bj):return False
    for name,x in aj.items():
        y=bj[name]
        if x['type']!=y['type']:return False
        q,r=np.array(x['qpos']),np.array(y['qpos'])
        if not np.isfinite(q).all() or not np.isfinite(r).all():return False
        kind=x['type']
        if kind==0:
            if np.linalg.norm(q[:3]-r[:3])>translation_tolerance_m:return False
            qr,rr=Rotation.from_quat(q[3:][[1,2,3,0]]),Rotation.from_quat(r[3:][[1,2,3,0]])
            if (qr*rr.inv()).magnitude()>angle_tolerance_rad:return False
        elif kind==1:
            qr,rr=Rotation.from_quat(q[[1,2,3,0]]),Rotation.from_quat(r[[1,2,3,0]])
            if (qr*rr.inv()).magnitude()>angle_tolerance_rad:return False
        elif np.max(np.abs(q-r))>(translation_tolerance_m if kind==2 else angle_tolerance_rad):return False
    return True

class StaticRobotModel:
    def __init__(self,xml,snapshot):
        self.xml=xml;self.m=mujoco.MjModel.from_xml_string(xml);self.d=mujoco.MjData(self.m);self.index={}
        root=ET.fromstring(xml);self.exclusions={frozenset([x.get('body1'),x.get('body2')]) for x in root.findall('./contact/exclude')}
        for side,prefix in [('right','R'),('left','L')]:
            js=[self.m.joint(f'{prefix}_arm_j{i}').id for i in range(1,8)];gs=[self.m.joint(f'{prefix}_gripper_j{i}').id for i in [1,2]]
            self.index[side]={'q':self.m.jnt_qposadr[js],'v':self.m.jnt_dofadr[js],'jaws':self.m.jnt_qposadr[gs],'limits':self.m.jnt_range[js],'site':self.m.site('tcp_'+prefix).id}
        self.load(snapshot)
    def load(self,snapshot):
        for name,state in snapshot.get('world',{}).get('joints',{}).items():
            joint=self.m.joint(name);qa=int(self.m.jnt_qposadr[joint.id]);va=int(self.m.jnt_dofadr[joint.id])
            self.d.qpos[qa:qa+len(state['qpos'])]=state['qpos']
            self.d.qvel[va:va+len(state['qvel'])]=state['qvel']
        for s in SIDES:
            k=self.index[s];self.d.qpos[k['q']]=snapshot['robot'][s]['q'];self.d.qpos[k['jaws']]=snapshot['robot'][s]['jaws']
        mujoco.mj_fwdPosition(self.m,self.d)
    def set_arm(self,side,q):self.d.qpos[self.index[side]['q']]=q;mujoco.mj_fwdPosition(self.m,self.d)
    def pose(self,side):
        site=self.index[side]['site'];return self.d.site_xpos[site].copy(),Rotation.from_matrix(self.d.site_xmat[site].reshape(3,3))
    def inverse_candidates(self,side,target,snapshot,cfg,cancel,acceptable=None):
        k=self.index[side];lo,hi=k['limits'].T;goal=Rotation.from_quat(np.asarray(target['quaternion_wxyz'])[[1,2,3,0]]);rng=np.random.default_rng(cfg['seed']);found=[];best=None
        for seed in range(cfg['goal_ik_seeds']):
            q=np.array(snapshot['robot'][side]['q']) if seed==0 else rng.uniform(lo+.01,hi-.01)
            for _ in range(220):
                if cancel():return found,best
                self.set_arm(side,q);pos,R=self.pose(side);dp=np.array(target['position_m'])-pos;dr=(goal*R.inv()).as_rotvec();err=[float(np.linalg.norm(dp)),float(np.linalg.norm(dr))]
                if best is None or sum(err)<sum(best):best=err
                if err[0]<=cfg['goal_position_tolerance_m'] and err[1]<=cfg['goal_orientation_tolerance_rad']:
                    if not any(np.max(abs(q-x))<.01 for x in found):found.append(q.copy())
                    if seed==0 and np.linalg.norm(q-np.array(snapshot['robot'][side]['q']))<.15 and (acceptable is None or acceptable(q)):
                        self.load(snapshot);return found,best
                    break
                jp=np.zeros((3,self.m.nv));jr=np.zeros_like(jp);mujoco.mj_jacSite(self.m,self.d,jp,jr,k['site']);J=np.vstack([jp[:,k['v']],.25*jr[:,k['v']]]);dq=J.T@np.linalg.solve(J@J.T+1e-4*np.eye(6),np.r_[dp,.25*dr]);q=np.clip(q+np.clip(dq,-.12,.12),lo+1e-5,hi-1e-5)
        self.load(snapshot);return found,best
