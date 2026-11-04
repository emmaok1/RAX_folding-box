"""Fetch only the official Dexmate URDF and its actual mesh dependency closure."""
from pathlib import Path
import json,hashlib,urllib.request,concurrent.futures,posixpath,xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'models/dexmate/source';OUT.mkdir(parents=True,exist_ok=True)
REPO='dexmate-ai/dexmate-urdf'
commit='ffcc62500fea1e2e7600411b4ec9f3b420bb780a'
model='robots/humanoid/vega_1u/vega_1u_gripper.urdf'
def get(path):
 out=OUT/path;out.parent.mkdir(parents=True,exist_ok=True)
 if not out.exists():out.write_bytes(urllib.request.urlopen(f'https://raw.githubusercontent.com/{REPO}/{commit}/{path}',timeout=60).read())
 return out
urdf=get(model);r=ET.parse(urdf).getroot();paths={model,'LICENSE','README.md','robots/humanoid/vega_1u/vega_1u.srdf'}
for m in r.iter('mesh'):paths.add(posixpath.normpath(posixpath.join(posixpath.dirname(model),m.attrib['filename'])))
with concurrent.futures.ThreadPoolExecutor(8) as pool:list(pool.map(get,sorted(paths)))
meta={'repository':'https://github.com/'+REPO,'commit':commit,'urdf':model,'license':'Apache-2.0, see source/LICENSE','sha256':{p:hashlib.sha256((OUT/p).read_bytes()).hexdigest() for p in sorted(paths)}}
(ROOT/'models/dexmate/provenance.json').write_text(json.dumps(meta,indent=2));print('Fetched',len(paths),'files from',commit,flush=True)
