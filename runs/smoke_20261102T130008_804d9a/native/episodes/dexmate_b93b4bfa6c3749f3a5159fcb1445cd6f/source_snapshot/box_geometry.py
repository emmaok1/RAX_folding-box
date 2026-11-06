"""Measured panel-relative capture geometry for actual native tab/slot shapes."""
import numpy as np

def capture_geometry(e):
    base=e.data.body('box_base');B=base.xmat.reshape(3,3);th=float(e.physical_params['thickness_m'])
    def flat(pt):return np.array([(pt[0]-641.1627)*.0254/72,-(pt[1]-963.3518)*.0254/72,0.])
    def source_pt(pt):
        from material_dieline import map_pdf
        return map_pdf(pt,e.geometry.get('material_layout'))
    def world(panel,pt,flex=False):
        if flex:
            spec=next(f for f in e.geometry['flex_regions'] if f['parent']==panel);name=spec['name'];origin=spec['origin_flat_m']
        else:spec=next(p for p in e.geometry['panels'] if p['panel']==panel);name=panel;origin=spec['origin_flat_m']
        b=e.data.body('box_'+name);return B.T@(b.xpos+b.xmat.reshape(3,3)@(flat(pt)-origin)-base.xpos)
    rows=[]
    for side,si,neck,tip in [('right',0,33.73,24.24),('left',1,1248.59,1258.09)]:
        panel='side_'+side+'_inner';f=next((f for f in e.geometry.get('flex_regions',[]) if f['parent']==panel),None)
        slot=np.array(e.geometry['slots_pdf_pt'][si]);a=flat(slot.min(0));b=flat(slot.max(0));lo=np.minimum(a,b);hi=np.maximum(a,b)
        upper=world(panel,source_pt([neck,963.3507]));lower=world(panel,[(max(v[0] for v in f['polygon_pdf_pt']) if side=='right' else min(v[0] for v in f['polygon_pdf_pt'])),float(source_pt([neck,963.3507])[1])])
        t=-upper[2]/(lower[2]-upper[2]) if abs(lower[2]-upper[2])>1e-12 else np.inf
        entry=upper+t*(lower-upper) if np.isfinite(t) else np.full(3,np.nan)
        through=bool(0<=t<=1 and np.all(entry[:2]>lo[:2]+th/2) and np.all(entry[:2]<hi[:2]-th/2))
        tail=world(panel,source_pt([tip,963.3507]),flex=f is not None)
        overlap=(tail[0]-hi[0]) if side=='right' else (lo[0]-tail[0])
        captured=bool(through and overlap>.0004 and -.003<tail[2]<-th/2 and lo[1]<tail[1]<hi[1])
        contacts=0;force_contacts=0
        bid=e.model.body('box_'+(f['name'] if f else panel)).id;bb=e.model.body('box_base').id
        for c in e.data.contact:
            bodies={int(e.model.geom_bodyid[int(c.geom1)]),int(e.model.geom_bodyid[int(c.geom2)])}
            if bodies=={bid,bb}:contacts+=1
        rows.append({'side':side,'entry_base_m':entry.tolist() if np.isfinite(entry).all() else None,'tail_base_m':tail.tolist(),'delta_base_m':(tail-(lo+hi)/2).tolist(),'near_slot':through,'through_slot':through,'under_base_overlap_m':float(overlap),'captured_under_base':captured,'tab_base_contacts':contacts})
    return rows
