"""Operator-owned, content-addressed world references; never imports the simulator."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

CORE = ('environment.py', 'box_geometry.py', 'robot_config.json', 'physics.json',
        'geometry/panels.json')


def digest(files):
    return hashlib.sha256(json.dumps(files, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def world_sources(project):
    project = Path(project).resolve()
    names = set(CORE)
    names.update(str(p.relative_to(project)) for p in (project/'models').rglob('*') if p.is_file())
    # Preserve original relative asset spellings. Refuse missing/external dependencies.
    for name in list(names):
        if not name.endswith('.xml'):
            continue
        xml = project/name
        root = ET.parse(xml).getroot()
        compiler = root.find('compiler')
        dirs = compiler.attrib if compiler is not None else {}
        for node in root.iter():
            value = node.get('file')
            if not value:
                continue
            prefix = dirs.get('meshdir', '') if node.tag == 'mesh' else dirs.get('texturedir', '') if node.tag == 'texture' else ''
            dep = (xml.parent/prefix/value).resolve()
            if not dep.is_relative_to(project) or not dep.is_file():
                raise ValueError(f'Unresolved world dependency in {name}: {value}')
            names.add(str(dep.relative_to(project)))
    return {name: hashlib.sha256((project/name).read_bytes()).hexdigest() for name in sorted(names)}


def create_world_snapshot(project, destination):
    """Copy all scene assets/configs, verify a stable source set, then publish atomically."""
    project, destination = Path(project).resolve(), Path(destination).resolve()
    if destination.exists():
        raise FileExistsError(destination)
    sources = world_sources(project)
    destination.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix='.world-', dir=destination.parent))
    try:
        for name, expected in sources.items():
            raw = (project/name).read_bytes()
            if hashlib.sha256(raw).hexdigest() != expected:
                raise RuntimeError('World changed during snapshot')
            target = staging/name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(raw)
            target.chmod(0o444)
        if sources != world_sources(project):
            raise RuntimeError('World changed during snapshot')
        manifest = {'schema': 'rax.teacher.world.v1', 'environment_id': digest(sources),
                    'files': sources, 'privilege': 'teacher_only',
                    'state_semantics': 'Static world definition, NOT live state or a successful demonstration.',
                    'physical_completion': 'Current evaluator always withholds succeed; physical locking unvalidated.'}
        (staging/'manifest.json').write_text(json.dumps(manifest, indent=2))
        (staging/'manifest.json').chmod(0o444)
        os.replace(staging, destination)
        return manifest
    finally:
        if staging.exists():
            shutil.rmtree(staging)


def verify_world_snapshot(directory):
    directory = Path(directory)
    manifest = json.loads((directory/'manifest.json').read_text())
    files = manifest['files']
    if manifest['environment_id'] != digest(files):
        raise ValueError('World manifest identity mismatch')
    for name, expected in files.items():
        path = directory/name
        if Path(name).is_absolute() or '..' in Path(name).parts or path.is_symlink():
            raise ValueError('Unsafe world entry')
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError(f'World snapshot changed: {name}')
    return manifest
