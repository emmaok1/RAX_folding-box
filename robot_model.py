"""Faithful URDF kinematics/inertias with explicit MuJoCo actuation and jaw pads."""
from pathlib import Path
import json,hashlib,xml.etree.ElementTree as ET
import numpy as np,trimesh
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent
ROBOT=ROOT/'models/dexmate';SRC=ROBOT/'source';URDF=SRC/'robots/humanoid/vega_1u/vega_1u_gripper.urdf'
MESH=ROBOT/'meshes';MESH.mkdir(exist_ok=True)
def vals(node,key,default):return np.fromstring(node.get(key,default),sep=' ') if node is not None else np.fromstring(default,sep=' ')
def fmt(x):return ' '.join(f'{float(v):.10g}' for v in np.asarray(x).ravel())
def origin(node):
    o=node.find('origin');p=vals(o,'xyz','0 0 0');r=Rotation.from_euler('xyz',vals(o,'rpy','0 0 0'));return p,r

def mesh_parts(path,visual):
    key=hashlib.sha256(str(path.relative_to(SRC)).encode()).hexdigest()[:14];manifest=MESH/(key+('_visual.json' if visual else '_collision.json'))
    if manifest.exists():return json.loads(manifest.read_text())
    scene=trimesh.load(str(path),force='scene',process=False);parts=[]
    for i,n in enumerate(scene.graph.nodes_geometry):
        tf,gn=scene.graph[n];m=scene.geometry[gn].copy();m.apply_transform(tf)
        # Preserve actual GLB primitive geometry, including its node transform.
        color=np.array([.62,.65,.68,1.])
        mat=getattr(m.visual,'material',None)
        if mat is not None:
            c=getattr(mat,'baseColorFactor',None)
            if c is None:c=getattr(mat,'main_color',None)
            if c is not None:
                color=np.array(c,dtype=float)
                if color.max()>1:color/=255
                if len(color)==3:color=np.r_[color,1]
        v=np.asarray(m.vertices);_,sv,vt=np.linalg.svd(v-v.mean(0),full_matrices=False)
        if sv[-1]<1e-8:
            if not visual:continue  # planar paint/decal has no collision volume
            normal=vt[-1];nv=len(v);faces=np.asarray(m.faces)
            m=trimesh.Trimesh(vertices=np.vstack([v+normal*1e-6,v-normal*1e-6]),faces=np.vstack([faces,faces[:,::-1]+nv]),process=False)
        fn=f'{key}_{"v" if visual else "c"}_{i}.obj';m.export(MESH/fn,file_type='obj',include_color=False,include_texture=False)
        parts.append({'file':'dexmate/meshes/'+fn,'rgba':color.tolist(),'vertices':len(m.vertices),'faces':len(m.faces)})
    manifest.write_text(json.dumps(parts,indent=2));return parts

def add_robot(arena):
    cfg=json.loads((ROOT/'robot_config.json').read_text());ur=ET.parse(URDF).getroot();links={x.get('name'):x for x in ur.findall('link')};joints={};children={}
    for j in ur.findall('joint'):
        parent=j.find('parent').get('link');child=j.find('child').get('link');children.setdefault(parent,[]).append(j);joints[child]=j
    rootlink=next(n for n in links if n not in joints);assets=arena.find('asset');wb=arena.find('worldbody')
    act=ET.SubElement(arena,'actuator');eq=ET.SubElement(arena,'equality');contact=ET.SubElement(arena,'contact');bodies={};meshid=0
    root=ET.SubElement(wb,'body',name='dexmate_mount',pos=fmt(cfg['mount_position_m']),euler=fmt([0,0,cfg['mount_yaw_rad']]))
    info={'movable_joints':[],'fixed_joints':[],'mesh_parts':0,'hand_collision_bodies':[],'custom_jaw_pads':[]}
    def visit(name,parent):
        nonlocal meshid
        j=joints.get(name);p=np.zeros(3);rot=Rotation.identity();active=False
        if j is not None:
            p,rot=origin(j);jn=j.get('name');axis=vals(j.find('axis'),'xyz','1 0 0')
            if jn in ['Lift','torso_flip'] or jn.startswith('head_j'):
                value=cfg['manual_stand_lift_m'] if jn=='Lift' else cfg['manual_stand_flip_rad'] if jn=='torso_flip' else cfg['head_joints_rad'][int(jn[-1])-1]
                if j.get('type')=='prismatic':p=p+rot.apply(axis*value)
                else:rot=rot*Rotation.from_rotvec(axis*value)
            elif j.get('type')!='fixed':active=True
        b=ET.SubElement(parent,'body',name='dexmate_'+name,pos=fmt(p),quat=fmt(rot.as_quat()[[3,0,1,2]]));bodies[name]=b
        if active:
            limit=j.find('limit');jn=j.get('name');arm='_arm_j' in jn
            ET.SubElement(b,'joint',name=jn,type='hinge',axis=fmt(axis),range=fmt([float(limit.get('lower')),float(limit.get('upper'))]),damping=str(cfg['arm_damping_Nms_rad'] if arm else .005),armature=str(cfg['arm_armature_kgm2'] if arm else .00002))
            info['movable_joints'].append({'name':jn,'urdf_limit':limit.attrib,'axis':axis.tolist()})
            if arm:ET.SubElement(act,'motor',name='motor_'+jn,joint=jn,ctrlrange=fmt([-float(limit.get('effort')),float(limit.get('effort'))]))
            elif j.find('mimic') is None:ET.SubElement(act,'motor',name='motor_'+jn,joint=jn,ctrlrange=fmt([-cfg['gripper_max_torque_Nm'],cfg['gripper_max_torque_Nm']]))
            else:ET.SubElement(eq,'joint',name='coupling_'+jn,joint1=jn,joint2=j.find('mimic').get('joint'),polycoef='0 1 0 0 0',solref=fmt(cfg.get('gripper_mimic_solref',[.004,1])),solimp=fmt(cfg.get('gripper_mimic_solimp',[.9,.95,.001,.5,2])))
        elif j is not None:info['fixed_joints'].append(j.get('name'))
        l=links[name];ine=l.find('inertial')
        if ine is not None:
            ip,ir=origin(ine);ii=ine.find('inertia');I=np.array([[float(ii.get('ixx')),float(ii.get('ixy')),float(ii.get('ixz'))],[float(ii.get('ixy')),float(ii.get('iyy')),float(ii.get('iyz'))],[float(ii.get('ixz')),float(ii.get('iyz')),float(ii.get('izz'))]])
            I=ir.as_matrix()@I@ir.as_matrix().T
            ET.SubElement(b,'inertial',pos=fmt(ip),mass=ine.find('mass').get('value'),fullinertia=fmt([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]))
        for visual in [True,False]:
            for node in l.findall('visual' if visual else 'collision'):
                pos,r=origin(node);geo=node.find('geometry');mesh=geo.find('mesh')
                if mesh is None:raise ValueError('Unhandled URDF primitive in '+name)
                path=(URDF.parent/mesh.get('filename')).resolve();scale=vals(mesh,'scale','1 1 1')
                for part in mesh_parts(path,visual):
                    mn=f'dexmate_mesh_{meshid}';meshid+=1
                    ET.SubElement(assets,'mesh',name=mn,file=part['file'],scale=fmt(scale),inertia='shell')
                    kw={'name':f'dexmate_{name}_{"visual" if visual else "collision"}_{meshid}','type':'mesh','mesh':mn,'pos':fmt(pos),'quat':fmt(r.as_quat()[[3,0,1,2]]),'mass':'0','rgba':fmt(part['rgba'] if visual else [.25,.3,.35,1]),'group':'2' if visual else '3'}
                    if visual:kw.update(contype='0',conaffinity='0')
                    ET.SubElement(b,'geom',**kw)
        if '_gripper_' in name:info['hand_collision_bodies'].append('dexmate_'+name)
        if name.endswith(('gripper_l1','gripper_l2')):
            side=name[0];jaw=int(name[-1]);sign=1 if jaw==1 else -1;w=cfg['jaw_pad_half_width_m']
            pad=cfg['jaw_pad_design'];angle=pad['design_jaw_angle_rad'];slope=np.tan(angle);a=(.0258-pad['design_board_gap_m']/2)/np.cos(angle);z0,z1=pad['z_limits_link_m'];thick=pad['thickness_m'];x0=a+slope*z0;x1=a+slope*z1
            verts=np.array([[x,y,z] for y in [-w,w] for x,z in [(x0-thick,z0),(x0,z0),(x1,z1),(x1-thick,z1)]]);verts[:,0]*=sign
            mn=f'jaw_pad_mesh_{side}_{jaw}';ET.SubElement(assets,'mesh',name=mn,vertex=fmt(verts),inertia='shell')
            ET.SubElement(b,'geom',name=f'jaw_pad_{side}_{jaw}',type='mesh',mesh=mn,rgba='.1 .13 .16 1',mass='0',friction=fmt(cfg['jaw_pad_friction']),priority='1',solref='.004 1',solimp='.95 .99 .0001',margin='.00001')
            ET.SubElement(b,'site',name=f'pad_{side}_{jaw}',pos=fmt([sign*.026,0,.132]),size='.002',rgba='0 0 0 0')
            # Uniform parallelepiped pad inertia, then parallel-axis combination
            # with the official link inertia. No missing or double-counted pad mass.
            E=np.array([[sign*thick,sign*slope*(z1-z0),0],[0,0,2*w],[0,z1-z0,0]])
            pm=abs(np.linalg.det(E))*pad['density_kg_m3'];pc=verts.mean(0);C=E@E.T/12;PI=pm*(np.trace(C)*np.eye(3)-C);oldmass=float(ine.find('mass').get('value'));total=oldmass+pm;com=(oldmass*ip+pm*pc)/total
            shift=lambda v:np.dot(v,v)*np.eye(3)-np.outer(v,v)
            combined=I+oldmass*shift(ip-com)+PI+pm*shift(pc-com);bi=b.find('inertial');bi.set('pos',fmt(com));bi.set('mass',str(total));bi.set('fullinertia',fmt([combined[0,0],combined[1,1],combined[2,2],combined[0,1],combined[0,2],combined[1,2]]))
            info['custom_jaw_pads'].append({'side':side,'jaw':jaw,'vertices_link_frame_m':verts.tolist(),'added_mass_kg':pm,'note':'Custom engineered elastomer pad; positive face slope fixes heel interference. Geometry/material not manufacturer measurements.'})
        if name.endswith('gripper_base'):
            side=name[0];ET.SubElement(b,'site',name='tcp_'+side,pos='0 0 .16',size='.005',rgba='0 0 0 0')
        if name.endswith('gripper_base'):
            side=name[0];ET.SubElement(b,'camera',name='handcam_rgb_'+('left' if side=='L' else 'right'),pos='0 -.085 .09',xyaxes='1 0 0 0 -.76 .65',fovy='85')
        if name=='head_l3':ET.SubElement(b,'camera',name='head',pos='.04 0 .05',xyaxes='0 -1 0 .65 0 .76',fovy='85')
        for child in children.get(name,[]):visit(child.find('child').get('link'),b)
    visit(rootlink,root)
    # Manufacturer SRDF adjacent-link exclusions only; no Never/Default blanket masks.
    sr=ET.parse(SRC/'robots/humanoid/vega_1u/vega_1u.srdf').getroot();exclusions=[]
    for e in sr.findall('disable_collisions'):
        if e.get('reason')!='Always':continue
        a,c=e.get('link1'),e.get('link2')
        if a in bodies and c in bodies:ET.SubElement(contact,'exclude',body1='dexmate_'+a,body2='dexmate_'+c);exclusions.append([a,c])
    info.update(source='models/dexmate/provenance.json',srdf_always_exclusions=exclusions,config=cfg)
    (ROBOT/'conversion.json').write_text(json.dumps(info,indent=2));return info
