"""Read-only diagnostics of exact corrected model; no execution or planning."""
from pathlib import Path
import sys,json,numpy as np
HERE=Path(__file__).resolve().parent;sys.path[:0]=[str(HERE/'deps'),str(HERE.parent)]
from tesseract_robotics.planning import Robot
from tesseract_robotics.tesseract_collision import ContactResultMap,ContactRequest,ContactTestType
from common import PROTOCOL
p=HERE/'run'/'model_approach';r=Robot.from_files(p/'scene.urdf',p/'scene.srdf')
manifest=json.loads((p/'manifest.json').read_text());lookup={x['link']:x['name'] for x in manifest['geoms']};start=np.array([-.8,-.28,0,-1.25,0,-.45,0]);goal=np.array(PROTOCOL['approach_q_right']);names=r.get_joint_names('right')
cm=r.env.getDiscreteContactManager();cm.setActiveCollisionObjects(r.env.getActiveLinkNames());cm.setDefaultCollisionMargin(.005)
rows=[]
for t in np.linspace(0,1,101):
 r.set_joints(dict(zip(names,start+(goal-start)*t)));cm.setCollisionObjectsTransform(r.env.getState().link_transforms);result=ContactResultMap();cm.contactTest(result,ContactRequest(ContactTestType.ALL));contacts=result.flattenCopyResults()
 for c in contacts:
  rows.append({'t':float(t),'distance_m':float(c.distance),'names':[lookup.get(n,n) for n in c.link_names]})
audit={'sampling':'101points along shared direct jointspace seed (diagnostic only)','contacts_within_5mm':len(rows),'min_distance_m':min([x['distance_m'] for x in rows],default=None),'worst_contacts':sorted(rows,key=lambda x:x['distance_m'])[:15]};(HERE/'run'/'direct_seed_collision_audit.json').write_text(json.dumps(audit,indent=2));print(json.dumps(audit,indent=2))
