"""Whole-section, finite-thickness tongue/slot checks; no locking claim."""
import numpy as np
from scipy.spatial import ConvexHull

def capture_geometry(e):
    specs={p['panel']:p for p in e.geometry['panels']};front=e.data.body('box_body_front');F=front.xmat.reshape(3,3);origin=np.array(specs['body_front']['origin_flat_m']);slot=np.array(e.geometry['slots_flat_m'][0]);lo=slot.min(0);hi=slot.max(0);center=np.r_[(lo+hi)/2,0]-origin;half=(hi-lo)/2;th=e.physical_params['thickness_m'];tol=.0001
    tongue=e.data.body('box_locking_tongue');T=tongue.xmat.reshape(3,3);poly=np.array(specs['locking_tongue']['polygon_flat_m'])[:-1];o=np.array(specs['locking_tongue']['origin_flat_m']);tip=poly[np.isclose(poly[:,1],poly[:,1].max())].mean(0);root=F.T@(tongue.xpos-front.xpos)-center;tip=F.T@(tongue.xpos+T@(np.r_[tip,0]-o)-front.xpos)-center
    sections={str(z):[] for z in [-th/2,0.,th/2]};allverts=[]
    for g in range(e.model.ngeom):
        if not (e.model.geom(g).name or '').startswith('card_locking_tongue_'):continue
        mesh=int(e.model.geom_dataid[g]);v=e.model.mesh_vert[e.model.mesh_vertadr[mesh]:e.model.mesh_vertadr[mesh]+e.model.mesh_vertnum[mesh]];world=e.data.geom_xpos[g]+v@e.data.geom_xmat[g].reshape(3,3).T;v=(world-front.xpos)@F-center;allverts.extend(v.tolist());h=ConvexHull(v);edges={tuple(sorted((int(a),int(b)))) for face in h.simplices for a,b in zip(face,np.roll(face,1))}
        for zkey,pts in sections.items():
            z=float(zkey)
            for a,b in edges:
                if (v[a,2]-z)*(v[b,2]-z)<=0 and abs(v[b,2]-v[a,2])>1e-12:
                    u=(z-v[a,2])/(v[b,2]-v[a,2]);pts.append((v[a]+u*(v[b]-v[a]))[:2].tolist())
    fits=bool(all(len(pts)>=3 and np.all(np.abs(np.array(pts))<=half-tol) for pts in sections.values()))
    direction=bool(root[2]>th/2-tol and tip[2]<-th/2-tol);depth=max(0.,-float(tip[2])-th/2);through=bool(fits and direction)
    return [{'feature':'front_tongue_slot','through_slot':through,'whole_section_fits':fits,'insertion_direction_valid':direction,'tip_front_frame_m':tip.tolist(),'insertion_depth_m':depth,'geometric_capture_candidate':bool(through and depth>.003),'physical_retention_verified':False}]
