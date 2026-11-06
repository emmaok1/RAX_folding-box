"""Use the actual RAX persistence, transfer and bundle-verification lifecycle."""
from __future__ import annotations
import json,sys,uuid,os
from pathlib import Path

UPSTREAM=Path(__file__).resolve().parents[1]/'upstream'
if str(UPSTREAM) not in sys.path:sys.path.insert(0,str(UPSTREAM))
from services.controller.storage import persist,verify
from services.transfer import pack,unpack

def freeze_workspace(workspace,destination,*,environment_id,revision,max_mb=10):
    """Frozen transfer bundle only; never rehearsal, submission or execution."""
    workspace,destination=Path(workspace),Path(destination)
    if not (workspace/'memory').is_dir() or not (workspace/'handoff').is_dir():
        raise ValueError('RAX memory/ and handoff/ are required')
    manifest,record=pack(destination,workspace=workspace,parts=('memory','code','handoff'),max_mb=max_mb,
        metadata={'teacher_memory':True,'environment_id':environment_id,'memory_revision':revision,'stage':'local_memory_checkpoint','cloud_requests':0})
    if record['excluded'] or any(record['parts'].get(k) is None for k in ('memory','handoff')) or record['skipped']:
        raise ValueError('Incomplete transfer bundle; not registered: '+json.dumps(record['excluded'] or record['skipped']))
    verify(destination,manifest)
    persist(destination.parent/(destination.name+'.manifest.json'),manifest)
    return manifest

def restore_workspace(package,manifest,destination):
    """Restore only into a fresh directory; upstream verification precedes copying."""
    package,destination=Path(package),Path(destination)
    if destination.exists():raise ValueError('Restore destination must not already exist')
    verify(package,manifest)
    record=json.loads((package/'package.json').read_text())
    if not record.get('teacher_memory'):raise ValueError('Not a teacher-memory checkpoint')
    staging=destination.with_name('.'+destination.name+'-'+uuid.uuid4().hex)
    staging.mkdir(mode=0o700,parents=True)
    parts=unpack(package,workspace=staging,context='clear')
    if not {'memory','handoff'}<=set(parts):raise ValueError('Incomplete memory checkpoint')
    os.replace(staging,destination)
    return {'workspace':str(destination),'parts':parts,'environment_id':record['environment_id'],'memory_revision':record['memory_revision']}
