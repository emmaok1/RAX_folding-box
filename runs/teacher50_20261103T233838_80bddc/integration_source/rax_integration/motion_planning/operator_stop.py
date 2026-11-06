"""Out-of-band operator cancellation; no robot state or controller imports."""
from pathlib import Path
import argparse,json,time,os

def request_stop(root,reason='operator_stop'):
    p=Path(root)/'motion_planning/STOP';p.parent.mkdir(parents=True,exist_ok=True);tmp=p.with_suffix('.tmp');tmp.write_text(json.dumps({'reason':reason,'wall_unix_s':time.time()}));os.replace(tmp,p);return p
if __name__=='__main__':
    a=argparse.ArgumentParser();a.add_argument('run_root',type=Path);a.add_argument('--reason',default='operator_stop');v=a.parse_args();print(request_stop(v.run_root,v.reason))
