"""Snapshot compiled MuJoCo collision hulls/kinematics into a Tesseract URDF.
Only right seven arm joints remain moving. Every other body follows snapshot.
Collision filtering exactly mirrors MuJoCo masks, welded/parent bodies, excludes.
"""
from pathlib import Path
import json,xml.etree.ElementTree as X
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull

def nums(v):return ' '.join(format(float(x),'.17g') for x in v)
def tf(p,r):
 t=np.eye(4);t[:3,:3]=r;t[:3,3]=p;return t
def origin(e,t):X.SubElement(e,'origin',xyz=nums(t[:3,3]),rpy=nums(Rotation.from_matrix(t[:3,:3]).as_euler('xyz')))
def fixed(root,name,parent,child,t):
 j=X.SubElement(root,'joint',name=name,type='fixed');X.SubElement(j,'parent',link=parent);X.SubElement(j,'child',link=child);origin(j,t)
def export(env,out):
 out=Path(out).resolve();out.mkdir(parents=True,exist_ok=True);(out/'meshes').mkdir(exist_ok=True)
 m=env.model;d=mujoco.MjData(m);d.qpos[:]=env.data.qpos;d.qpos[env.arms['right']['qad']]=0;mujoco.mj_kinematics(m,d)
 bodies=[tf(d.xpos[b],d.xmat[b].reshape(3,3)) for b in range(m.nbody)]
 root=X.Element('robot',{'name':'mujoco_snapshot','tesseract:make_convex':'true'});X.SubElement(root,'link',name='world')
 active={int(m.jnt_bodyid[j]):int(j) for j in env.arms['right']['joint_ids']}
 for b in range(1,m.nbody):
  name=m.body(b).name;parent=m.body(int(m.body_parentid[b])).name if m.body_parentid[b] else 'world';X.SubElement(root,'link',name=name)
  t=np.linalg.inv(bodies[m.body_parentid[b]])@bodies[b]
  if b in active:
   jid=active[b];assert np.linalg.norm(m.jnt_pos[jid])<1e-12
   j=X.SubElement(root,'joint',name=m.joint(jid).name,type='revolute');X.SubElement(j,'parent',link=parent);X.SubElement(j,'child',link=name);origin(j,t)
   X.SubElement(j,'axis',xyz=nums(m.jnt_axis[jid]));X.SubElement(j,'limit',lower=str(m.jnt_range[jid,0]),upper=str(m.jnt_range[jid,1]),effort='200',velocity='2.4');X.SubElement(j,'dynamics',damping='0',friction='0')
  else:fixed(root,'fixed_'+name,parent,name,t)
 # every geom gets its own leaf to preserve per-geom masks exactly
 geoms=[];meshes={};manifest=[]
 for g in range(m.ngeom):
  if not (m.geom_contype[g]|m.geom_conaffinity[g]):continue
  name='geom_'+str(g);link=X.SubElement(root,'link',name=name);c=X.SubElement(link,'collision');shape=X.SubElement(c,'geometry');typ=int(m.geom_type[g]);mesh=int(m.geom_dataid[g]);b=int(m.geom_bodyid[g])
  t=np.linalg.inv(bodies[b])@tf(d.geom_xpos[g],d.geom_xmat[g].reshape(3,3))
  if typ==mujoco.mjtGeom.mjGEOM_MESH:
   if mesh not in meshes:
    v=m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh]+m.mesh_vertnum[mesh]].copy();h=ConvexHull(v);p=out/'meshes'/f'hull_{mesh}.obj'
    p.write_text('\n'.join(['v '+nums(x) for x in v]+['f '+' '.join(str(int(x)+1) for x in face) for face in h.simplices]));meshes[mesh]=p
   X.SubElement(shape,'mesh',filename=str(meshes[mesh].resolve()),scale='1 1 1')
  elif typ==mujoco.mjtGeom.mjGEOM_BOX:X.SubElement(shape,'box',size=nums(2*m.geom_size[g]))
  elif typ==mujoco.mjtGeom.mjGEOM_PLANE:
   # Infinite MuJoCo floor conservatively represented over 200m footprint.
   X.SubElement(shape,'box',size='200 200 2');t[:3,3]+=t[:3,:3]@np.array([0,0,-1.])
  else:raise RuntimeError(f'Unsupported active geom type {typ}')
  fixed(root,'fixed_geom_'+str(g),m.body(b).name if b else 'world',name,t);geoms.append(g)
  manifest.append({'id':g,'name':m.geom(g).name,'link':name,'body':m.body(b).name,'type':typ,'mesh':mesh,'contype':int(m.geom_contype[g]),'conaffinity':int(m.geom_conaffinity[g])})
 sid=env.arms['right']['site'];b=int(m.site_bodyid[sid]);X.SubElement(root,'link',name='tcp_R');fixed(root,'tcp_joint',m.body(b).name,'tcp_R',np.linalg.inv(bodies[b])@tf(d.site_xpos[sid],d.site_xmat[sid].reshape(3,3)))
 srdf=X.Element('robot',name='mujoco_snapshot');group=X.SubElement(srdf,'group',name='right');X.SubElement(group,'chain',base_link='world',tip_link='tcp_R')
 excluded=set(map(int,m.exclude_signature));allowed=[]
 for i,g in enumerate(geoms):
  for h in geoms[i+1:]:
   b,c=map(int,[m.geom_bodyid[g],m.geom_bodyid[h]]);w,v=map(int,[m.body_weldid[b],m.body_weldid[c]])
   mask=bool((m.geom_contype[g]&m.geom_conaffinity[h])or(m.geom_contype[h]&m.geom_conaffinity[g]))
   parent=bool(w==v or (w and v and int(m.body_weldid[m.body_parentid[w]])==v) or (v and w and int(m.body_weldid[m.body_parentid[v]])==w))
   ex=((min(b,c)<<16)+max(b,c)) in excluded
   if not mask or parent or ex:
    reason='mask' if not mask else 'welded_or_parent' if parent else 'explicit_exclude';X.SubElement(srdf,'disable_collisions',link1='geom_'+str(g),link2='geom_'+str(h),reason=reason);allowed.append([g,h,reason])
 plugins=out/'kinematics.yaml';plugins.write_text('''kinematic_plugins:
  search_libraries: [tesseract_kinematics_kdl_factories]
  fwd_kin_plugins:
    right:
      default: KDLFwdKinChain
      plugins:
        KDLFwdKinChain:
          class: KDLFwdKinChainFactory
          config: {base_link: world, tip_link: tcp_R}
  inv_kin_plugins:
    right:
      default: KDLInvKinChainLMA
      plugins:
        KDLInvKinChainLMA:
          class: KDLInvKinChainLMAFactory
          config: {base_link: world, tip_link: tcp_R}
''')
 X.SubElement(srdf,'kinematics_plugin_config',filename=str(plugins.resolve()));X.SubElement(srdf,'contact_managers_plugin_config',filename='package://tesseract/support/urdf/contact_manager_plugins.yaml')
 X.indent(root);X.indent(srdf);X.ElementTree(root).write(out/'scene.urdf');X.ElementTree(srdf).write(out/'scene.srdf')
 (out/'manifest.json').write_text(json.dumps({'geoms':manifest,'allowed_collision_pairs':allowed,'floor_approximation':'infinite plane replaced by200x200x2m box with exact top plane','snapshot_qpos':env.data.qpos.tolist()},indent=2))
 return out/'scene.urdf',out/'scene.srdf'

def verify(env,robot):
 m=env.model;d=mujoco.MjData(m);d.qpos[:]=env.data.qpos;names=[m.joint(j).name for j in env.arms['right']['joint_ids']];rng=np.random.default_rng(8);errors=[]
 for q in [env.data.qpos[env.arms['right']['qad']]]+[rng.uniform(env.arms['right']['limits'][:,0],env.arms['right']['limits'][:,1]) for _ in range(5)]:
  d.qpos[env.arms['right']['qad']]=q;mujoco.mj_kinematics(m,d);robot.env.setState(names,q);state=robot.env.getState()
  for g in range(m.ngeom):
   if not (m.geom_contype[g]|m.geom_conaffinity[g])or m.geom_type[g]==mujoco.mjtGeom.mjGEOM_PLANE:continue
   mat=state.link_transforms['geom_'+str(g)].matrix;errors.append(float(np.max(abs(mat-tf(d.geom_xpos[g],d.geom_xmat[g].reshape(3,3))))))
  mat=state.link_transforms['tcp_R'].matrix;errors.append(float(np.max(abs(mat-tf(d.site_xpos[env.arms['right']['site']],d.site_xmat[env.arms['right']['site']].reshape(3,3))))))
 assert max(errors)<1e-10,max(errors)
 return {'samples':6,'max_transform_abs_error':max(errors),'active_geoms':int(np.sum((m.geom_contype|m.geom_conaffinity)!=0))}
