"""Content identities for teacher memory, never robot/simulator state access."""
from __future__ import annotations
import hashlib,json,os,stat
from pathlib import Path,PurePosixPath

def canonical(value):
    return json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode('utf-8')

def identity_id(identity):
    if not isinstance(identity,dict) or not identity:
        raise ValueError('Environment identity must be a nonempty mapping')
    return hashlib.sha256(canonical(identity)).hexdigest()

def hash_file(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()

def capture_identity(sources,*,metadata=None):
    """Caller selects scene/controller/evaluator/interface/policy sources explicitly."""
    if not isinstance(sources,dict) or not sources:raise ValueError('Named identity sources required')
    result={'schema_version':1,'sources':{str(k):hash_file(v) for k,v in sorted(sources.items())},'metadata':metadata or {}}
    identity_id(result)
    return result

def evidence_reference(root,path):
    """Hash one regular evidence file beneath root; reject symlink traversal."""
    root=Path(root).absolute();relative=PurePosixPath(str(path))
    if relative.is_absolute() or not relative.parts or '..' in relative.parts or '\\' in str(path):
        raise ValueError('Evidence must be a relative path beneath evidence_root')
    fd=os.open('/',os.O_RDONLY|os.O_DIRECTORY)
    try:
        for name in root.parts[1:]+relative.parts[:-1]:
            child=os.open(name,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW,dir_fd=fd)
            os.close(fd);fd=child
        child=os.open(relative.name,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK,dir_fd=fd)
        with os.fdopen(child,'rb') as stream:
            if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):raise ValueError('Evidence must be a regular file')
            digest=hashlib.sha256();size=0
            for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block);size+=len(block)
        return {'path':relative.as_posix(),'sha256':digest.hexdigest(),'bytes':size}
    finally:os.close(fd)

def reference_valid(root,reference):
    try:return evidence_reference(root,reference['path'])==reference
    except (OSError,ValueError,KeyError):return False
