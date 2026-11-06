import json
from pathlib import Path
def main(ctx):
    world=json.loads(Path('runtime/world/manifest.json').read_text())
    reply=ctx.call('dexmate_teacher_state')
    state=json.loads(next(b['text'] for b in reply['content'] if b['type']=='text'))
    assert state['world_environment_id']==world['environment_id']
    assert state['step_id']==0
    Path('output/restricted_rehearsal_proof.json').write_text(json.dumps({
        'kind':'OFFLINE SCRIPTED CLI + RAX REHEARSAL', 'world_readable':True,
        'teacher_state_readable':True,'step_id':0,'cloud_inference_requests':0}))
