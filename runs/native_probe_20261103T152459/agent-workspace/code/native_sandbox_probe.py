from pathlib import Path
import json, socket, os
workspace=Path.cwd()
assert workspace.name=='agent-workspace'
world=Path('runtime/world')
manifest=json.loads((world/'manifest.json').read_text())
assert len(manifest['files'])>200
try:
    list(Path('/home/jennyok/.config/rax').iterdir())
except OSError:
    pass
else:
    raise AssertionError('operator credential directory readable')
try:
    os.chmod(world/'physics.json',0o644)
except OSError:
    pass
else:
    raise AssertionError('world reference writable')
for address in [('1.1.1.1',443),('127.0.0.1',80)]:
    try:
        c=socket.create_connection(address,timeout=.3)
    except OSError:
        continue
    c.close()
    raise AssertionError('network escaped native sandbox')
Path('output/native_sandbox_proof.json').write_text(json.dumps({
 'kind':'OFFLINE SCRIPTED NATIVE CLI SANDBOX PROBE',
 'world_readable':True,'world_readonly':True,'operator_credentials_unreadable':True,
 'network_blocked':True,'workspace_writable':True,'cloud_requests':0,'robot_steps':0}))
print('OFFLINE NATIVE SANDBOX PROBE PASSED; NO MODEL OR ROBOT TASK')