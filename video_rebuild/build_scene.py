"""New SI-coordinate video-inspired box. Does not import the old dieline."""
from pathlib import Path
import sys,json,hashlib,copy,xml.etree.ElementTree as ET
import mujoco
import numpy as np
from shapely.geometry import Polygon,box
from shapely.affinity import rotate
from shapely import constrained_delaunay_triangles,get_parts
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from robot_model import add_robot,fmt

def parts(poly):
    ps=list(get_parts(constrained_delaunay_triangles(poly)))
    changed=True
    while changed:
        changed=False
        for i in range(len(ps)):
            for j in range(i+1,len(ps)):
                u=ps[i].union(ps[j])
                if u.geom_type=='Polygon' and not u.interiors and abs(u.convex_hull.area-u.area)<1e-12:
                    ps[i]=u;ps.pop(j);changed=True;break
            if changed:break
    assert abs(sum(x.area for x in ps)-poly.area)<1e-10
    return ps

def main():
    c=json.loads((ROOT/'video_rebuild/config.json').read_text());p=json.loads((ROOT/'physics.json').read_text());r=json.loads((ROOT/'robot_config.json').read_text())
    t=c['thickness_m'];sig=c['basis_weight_g_m2']/1000;w=c['base_width_m']/2;d=c['base_depth_m']/2;h=c['wall_height_m'];g=c['layer_clearance_m'];inset=t+g;cw=c['corner_flap_width_m'];lw=w-2*inset;ly=d+h;ld=2*d-inset;lfy=ly+ld;fh=c['lid_front_flap_depth_m']
    # Input JSONs are authoritative: rebuilding never rewrites material,
    # crease, contact, gripper effort or controller choices.
    if not np.isclose(p['thickness_m'],t) or not np.isclose(p['basis_weight_g_m2']/1000,sig):
        raise ValueError('Geometry/material mismatch: update thickness and basis weight consistently in video_rebuild/config.json and physics.json.')
    if not np.isclose(p['density_kg_m3'],sig/t):
        raise ValueError('physics.json density must equal basis weight / thickness for this uniform-panel model.')
    law=p['crease_law']
    if not 0<=law['fold_stiffness_Nm_rad_per_m']<=law['reverse_stiffness_Nm_rad_per_m']:
        raise ValueError('Passive directional spring requires reverse stiffness >= fold stiffness >=0.')
    if not 0<=r['gripper_closing_torque_Nm']<=r['gripper_max_torque_Nm']:
        raise ValueError('Gripper closing effort must be nonnegative and within the configured limit.')
    arena=ET.parse(ROOT/'models/arena.xml').getroot();arena.find('option').set('timestep',str(r['physics_dt_s']));arena.find('option').set('integrator','implicitfast');arena.find('option').set('noslip_iterations','0');add_robot(arena)
    # Derive placement from the unchanged real table, so changing thickness
    # cannot silently initialize the base inside the tabletop.
    resolved=copy.deepcopy(arena)
    for node in resolved.iter():
        if node.get('file'):node.set('file',str((ROOT/'models'/node.get('file')).resolve()))
    am=mujoco.MjModel.from_xml_string(ET.tostring(resolved,encoding='unicode'));ad=mujoco.MjData(am);mujoco.mj_forward(am,ad);table=am.geom('table_collision').id
    if abs(ad.geom_xmat[table].reshape(3,3)[2,2])<.999999:raise ValueError('Automatic height requires the existing horizontal table.')
    tabletop=float(ad.geom_xpos[table,2]+am.geom_size[table,2])
    if c.get('auto_table_height',False):c['root_position_m']=[*c['root_xy_m'],tabletop+t/2+c['table_clearance_m']]
    if c['root_position_m'][2]-t/2<tabletop-1e-7:raise ValueError('Initial base intersects table; correct root placement.')
    wb=arena.find('worldbody');asset=arena.find('asset');contact=arena.find('contact');tendons=ET.SubElement(arena,'tendon')
    panels={}
    def add(name,parent,poly,anchor=(0,0,0),axis=(1,0,0),target=np.pi/2):panels[name]={'parent':parent,'poly':poly,'origin':np.array(anchor,dtype=float),'axis':np.array(axis,dtype=float),'target':target}
    add('base',None,box(-w,-d,w,d))
    add('body_front','base',box(-w+inset,-d-h,w-inset,-d),(0,-d,0),(-1,0,0))
    slot_z=h-t/2-fh;slot=box(-c['slot_width_m']/2,-d-slot_z-c['slot_height_m']/2,c['slot_width_m']/2,-d-slot_z+c['slot_height_m']/2)
    panels['body_front']['poly']=panels['body_front']['poly'].difference(slot)
    add('body_back','base',box(-w+inset,d,w-inset,d+h),(0,d,0),(1,0,0))
    # Side-wall end reliefs leave genuine clearance around front/back walls.
    for side,sgn in [('right',1),('left',-1)]:
        poly=Polygon([(sgn*w,-d+g),(sgn*(w+h-.008),-d+g),(sgn*(w+h),-d+g+.008),(sgn*(w+h),d-g-.008),(sgn*(w+h-.008),d-g),(sgn*w,d-g)])
        add('side_'+side,'base',poly,(sgn*w,0,0),(0,-sgn,0))
    for end,ys in [('front',-1),('back',1)]:
        yy=sorted([ys*(d+.006),ys*(d+h-.006)])
        for side,sgn in [('right',1),('left',-1)]:
            xx=sorted([sgn*(w-inset),sgn*(w-inset+cw)])
            add(end+'_corner_'+side,'body_'+end,box(xx[0],yy[0],xx[1],yy[1]),(sgn*(w-inset),sum(yy)/2,0),(0,-sgn,0))
    add('lid','body_back',box(-lw,ly,lw,lfy),(0,ly,0),(1,0,0))
    for side,sgn in [('right',1),('left',-1)]:
        xx=sorted([sgn*lw,sgn*(lw+c['lid_side_flap_width_m'])])
        add('lid_side_'+side,'lid',box(xx[0],ly+inset,xx[1],lfy-inset),(sgn*lw,(ly+lfy)/2,0),(0,-sgn,0))
    fw=lw-inset
    add('lid_front','lid',Polygon([(-fw,lfy),(fw,lfy),(fw,lfy+fh-.008),(fw-.008,lfy+fh),(-fw+.008,lfy+fh),(-fw,lfy+fh-.008)]),(0,lfy,0),(1,0,0))
    tw=c['tongue_width_m']/2;tl=c['tongue_length_m'];ty=lfy+fh
    add('locking_tongue','lid_front',Polygon([(-tw,ty),(tw,ty),(tw,ty+tl-.003),(tw-.003,ty+tl),(-tw+.003,ty+tl),(-tw,ty+tl-.003)]),(0,ty,0),(1,0,0),-np.pi/2)
    bodies={};rows=[];massrows=[];features=[];law=p['crease_law']
    for name,q in panels.items():
        parent=q['parent'];o=q['origin'];poly=q['poly'];color='.68 .48 .27 1' if name=='base' or 'corner' in name else '.88 .24 .08 1'
        if parent is None:
            body=ET.SubElement(wb,'body',name='box_base',pos=fmt(c['root_position_m']),quat=fmt([np.cos(c['root_yaw_rad']/2),0,0,np.sin(c['root_yaw_rad']/2)]));ET.SubElement(body,'freejoint',name='box_free')
        else:
            body=ET.SubElement(bodies[parent],'body',name='box_'+name,pos=fmt(o-panels[parent]['origin']))
            # The hinge lies on a material surface; child offset preserves the
            # flat blank. This is a passive finite-thickness approximation.
            length=poly.boundary.intersection(panels[parent]['poly'].boundary).length
            if length<1e-5:length=min(poly.bounds[2]-poly.bounds[0],poly.bounds[3]-poly.bounds[1])
            ET.SubElement(body,'joint',name='crease_'+name,type='hinge',axis=fmt(q['axis']),pos=fmt([0,0,t/2 if q['target']>0 else -t/2]),range=fmt(p['hinge_range_rad'] if q['target']>0 else p['reverse_hinge_range_rad']),damping=str(law['damping_Nms_rad_per_m']*length),stiffness=str(law['fold_stiffness_Nm_rad_per_m']*length),springref=str(p['spring_reference_rad']),frictionloss=str(law['frictionloss_Nm_per_m']*length),armature=str(p['hinge_armature_kgm2']),solreffriction=fmt(p['hinge_friction_solref']),solimpfriction=fmt(p['hinge_friction_solimp']))
            td=ET.SubElement(tendons,'fixed',name='passive_reverse_'+name,stiffness=str((law['reverse_stiffness_Nm_rad_per_m']-law['fold_stiffness_Nm_rad_per_m'])*length),springlength='0 10' if q['target']>0 else '-10 0',limited='false');ET.SubElement(td,'joint',joint='crease_'+name,coef='1')
        bodies[name]=body
        mass=poly.area*sig;cx,cy=np.array(poly.centroid.coords[0])-o[:2];moment=np.zeros(3)
        for tri in get_parts(constrained_delaunay_triangles(poly)):
            xy=np.array(tri.exterior.coords)[:3]-o[:2];x,y=xy.T;tm=tri.area*sig;moment+=tm*np.array([(x@x+x.sum()**2)/12,(y@y+y.sum()**2)/12,(x@y+x.sum()*y.sum())/12])
        xx,yy,xy=moment-mass*np.array([cx*cx,cy*cy,cx*cy]);zz=mass*t*t/12
        panel_mass=mass;COM=np.array([cx,cy,0.]);I=np.array([[yy+zz,-xy,0],[-xy,xx+zz,0],[0,0,xx+yy]]);feature_mass=0.
        gf=c.get('grasp_features',{})
        if gf.get('enabled') and name not in ['base','locking_tongue']:
            direction=np.array([0.,1.,0.]);flat=None
            if name in ['body_front','body_back']:
                sign=-1 if name=='body_front' else 1;dist=gf['wall_front_distance_m'] if sign<0 else gf['wall_back_distance_m'];flat=np.array([gf.get('wall_front_lateral_m',0) if sign<0 else 0.,sign*(d+dist)])
            elif name in ['side_left','side_right']:
                sign=1 if name=='side_right' else -1;flat=np.array([sign*(w+gf['side_distance_m']),0.]);direction=np.array([1.,0.,0.])
            elif 'corner' in name:
                sign=1 if name.endswith('right') else -1;ys=-1 if name.startswith('front') else 1;flat=np.array([sign*(w-inset+gf['corner_distance_m']),ys*(d+gf['corner_parent_height_m'])]);direction=np.array([1.,0.,0.])
            elif name.startswith('lid_side_'):
                sign=1 if name.endswith('right') else -1;flat=np.array([sign*(lw+c['lid_side_flap_width_m']/2),(ly+lfy)/2]);direction=np.array([1.,0.,0.])
            elif name=='lid':flat=np.array([0.,lfy-gf['lid_free_edge_inset_m']])
            elif name=='lid_front':flat=np.array([gf.get('lid_front_lateral_m',0),lfy+gf['lid_front_distance_m']])
            direction=np.abs(q['axis']);feature_height=gf.get('panel_heights_m',{}).get(name,gf['height_m'])
            dims=np.array([gf['thickness_m'],gf['width_m'],feature_height]) if direction[0] else np.array([gf['width_m'],gf['thickness_m'],feature_height])
            yaw=np.deg2rad(gf.get('yaw_deg',0));Q=np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1.]])
            footprint=rotate(box(flat[0]-dims[0]/2,flat[1]-dims[1]/2,flat[0]+dims[0]/2,flat[1]+dims[1]/2),gf.get('yaw_deg',0),origin=flat)
            if footprint.difference(poly).area>1e-10:raise ValueError('Grip aid footprint outside material: '+name)
            fc=np.r_[flat-o[:2],t/2+dims[2]/2];feature_mass=float(np.prod(dims)*gf['density_kg_m3']);FI=np.diag(feature_mass/12*np.array([dims[1]**2+dims[2]**2,dims[0]**2+dims[2]**2,dims[0]**2+dims[1]**2]));FI=Q@FI@Q.T;newCOM=(mass*COM+feature_mass*fc)/(mass+feature_mass)
            shift=lambda x:np.dot(x,x)*np.eye(3)-np.outer(x,x)
            I=I+mass*shift(COM-newCOM)+FI+feature_mass*shift(fc-newCOM);mass+=feature_mass;COM=newCOM
            cp=p['contacts']['cardboard_cardboard'];ET.SubElement(body,'geom',name='card_'+name+'_grip_aid',type='box',pos=fmt(fc),quat=fmt([np.cos(yaw/2),0,0,np.sin(yaw/2)]),size=fmt(dims/2),mass='0',rgba='.06 .45 .95 1',priority='1',condim=str(cp['condim']),friction=fmt([cp['sliding_mu'],cp['torsional_m'],cp['rolling_m']]),solref=fmt(cp['solref']),solimp=fmt(cp['solimp']),margin=str(cp['margin_m']))
            features.append({'panel':name,'center_body_m':fc.tolist(),'halfsize_m':(dims/2).tolist(),'mass_kg':feature_mass,'grasp_point_body_m':np.r_[flat-o[:2],t/2+dims[2]-.003].tolist(),'closing_axis_body':(Q@direction).tolist(),'approach_axis_body':[0.,0.,-1.]})
        ET.SubElement(body,'inertial',pos=fmt(COM),mass=str(mass),fullinertia=fmt([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]));massrows.append({'body':'box_'+name,'area_m2':poly.area,'panel_mass_kg':panel_mass,'grasp_feature_mass_kg':feature_mass,'mass_kg':mass})
        for k,part in enumerate(parts(poly)):
            v=np.c_[np.array(part.exterior.coords)[:-1]-o[:2],np.zeros(len(part.exterior.coords)-1)];vs=np.vstack([v+[0,0,-t/2],v+[0,0,t/2]]);mn='newbox_'+name+'_'+str(k)
            ET.SubElement(asset,'mesh',name=mn,vertex=fmt(vs),inertia='shell')
            ET.SubElement(body,'geom',name='card_'+name+'_'+str(k),type='mesh',mesh=mn,mass='0',rgba=color,priority='1',condim=str(p['contacts']['cardboard_cardboard']['condim']),friction=fmt([p['contacts']['cardboard_cardboard']['sliding_mu'],p['contacts']['cardboard_cardboard']['torsional_m'],p['contacts']['cardboard_cardboard']['rolling_m']]),solref=fmt(p['contacts']['cardboard_cardboard']['solref']),solimp=fmt(p['contacts']['cardboard_cardboard']['solimp']),margin=str(p['contacts']['cardboard_cardboard']['margin_m']))
        rows.append({'panel':name,'parent':parent,'polygon_flat_m':list(map(list,poly.exterior.coords)),'holes_flat_m':[list(map(list,z.coords)) for z in poly.interiors],'area_m2':poly.area,'origin_flat_m':o.tolist(),'axis_parent_frame':q['axis'].tolist() if parent else None,'target_rad':q['target'] if parent else 0})
    cards=[g for g in wb.iter('geom') if (g.get('name') or '').startswith('card_')];fingers=[g.get('name') for g in wb.iter('geom') if (g.get('name') or '').startswith('jaw_pad_') or any(z in (g.get('name') or '') for z in ['_gripper_l1_collision_','_gripper_l2_collision_'])]
    for ca in cards:
        for category,names in [('fingertip_cardboard',fingers),('table_cardboard',['table_collision'])]:
            cp=p['contacts'][category]
            for n in names:ET.SubElement(contact,'pair',geom1=n,geom2=ca.get('name'),condim=str(cp['condim']),friction=fmt([cp['sliding_mu']]*2+[cp['torsional_m'],cp['rolling_m'],cp['rolling_m']]),solref=fmt(cp['solref']),solimp=fmt(cp['solimp']),margin=str(cp['margin_m']))
    ET.indent(arena);ET.ElementTree(arena).write(ROOT/'models/scene.xml',encoding='unicode')
    geo={'coordinate_system':'SI meters, flat-blank local XY; no PDF coordinate claims','design':'video_rebuild','panels':rows,'flex_regions':[],'base_position_m':c['root_position_m'],'material_layout':c,'masses':massrows,'grasp_features':features,'slots_flat_m':[list(map(list,slot.exterior.coords))],'source':'local reference video; geometry inferred/engineered, not traced from a manufacturer template'}
    (ROOT/'geometry/panels.json').write_text(json.dumps(geo,indent=2));(ROOT/'material_layout.json').write_text(json.dumps(c,indent=2))
    print('NEW VIDEO BOX:',len(rows),'panels',len(cards),'collision prisms',sum(x['mass_kg'] for x in massrows),'kg')
    (ROOT/'video_rebuild/provenance.json').write_text(json.dumps({'video':str(ROOT.parent/'2b3a9cc52088540b8d5fb29466b82659.mp4'),'sha256':hashlib.sha256((ROOT.parent/'2b3a9cc52088540b8d5fb29466b82659.mp4').read_bytes()).hexdigest(),'config':c,'reconstruction_limits':'See VIDEO_NOTES.md; no exact dimensions/material/topology proven by video.'},indent=2))
if __name__=='__main__':main()
