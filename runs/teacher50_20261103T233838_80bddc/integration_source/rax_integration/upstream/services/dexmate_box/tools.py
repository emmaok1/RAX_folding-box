"""Dexmate schemas. Entire chunks are checked before any physics or references change."""
import math
import json
from pathlib import Path
from services.simulator.io import reject_unless
from .registry import TOOLS

MAX_ROWS = 50
CAMERAS = ('back', 'ego', 'wrist_left', 'wrist_right')


def action_layout(control_dt_s=None, physics_dt_s=None, closing_torque_Nm=None):
    cfg = (json.loads((Path(__file__).resolve().parents[4]/'robot_config.json').read_text())
           if any(v is None for v in (control_dt_s,physics_dt_s,closing_torque_Nm)) else {})
    control_dt_s = cfg['control_dt_s'] if control_dt_s is None else control_dt_s
    physics_dt_s = cfg['physics_dt_s'] if physics_dt_s is None else physics_dt_s
    closing_torque_Nm = cfg.get('gripper_closing_torque_Nm',0) if closing_torque_Nm is None else closing_torque_Nm
    substeps = int(round(control_dt_s/physics_dt_s))
    return {'dim': 16, 'order': ['right', 'left'], 'per_arm':
            ['world_x_m', 'world_y_m', 'world_z_m', 'qw', 'qx', 'qy', 'qz', 'open_fraction'],
            'frame': 'MuJoCo world, right handed, z up', 'orientation': 'active local-to-world quaternion wxyz',
            'tool': 'tcp_R / tcp_L, [0,0,0.16] m in each gripper_base frame, same axes',
            'gripper': {'command_unit':'opening fraction, 0 closed to 1 open',
                        'positive_opening_target_rad':'0.96 * fraction',
                        'zero_opening_mode':'bounded closing torque + damping' if closing_torque_Nm else 'position PD',
                        'configured_closing_torque_Nm':closing_torque_Nm},
            'timing': {'control_dt_s':control_dt_s, 'physics_dt_s':physics_dt_s,
                       'substeps_per_row':substeps,'control_frequency_hz':1/control_dt_s,
                       'both_arms_staged_before_physics':True},
            'semantics': 'absolute motor-driven TCP references via existing IK + PD; no teleportation',
            'zero_pose': 'all first seven values zero for an arm holds its previous TCP target'}


def tool_definitions():
    empty = {'type': 'object', 'properties': {}, 'additionalProperties': False}
    steps = {'type': 'integer', 'minimum': 1, 'maximum': MAX_ROWS}
    return motion_definitions()+[
        {'name': 'dexmate_teacher_memory', 'description': 'TEACHER ONLY: restore persistent progress/experience, record experiments or candidate skills, checkpoint. Candidate notes never verify success; verification is operator-only. No physics.', 'inputSchema': {'type':'object','additionalProperties':False,'properties':{'operation':{'type':'string','enum':['read','record_experiment','record_claim','update_progress','register_skill','checkpoint']},'payload':{'type':'object'}}}},
        {'name': 'dexmate_teacher_state', 'description': 'TEACHER ONLY: fresh privileged simulator ground truth, measured robot/reference state, panel world poses, hinge velocities and sampled contact forces. Does not step physics. State is not a successful grasp demonstration.', 'inputSchema': empty},
        {'name': 'dexmate_status', 'description': 'Robot embodiment, exact action layout, cameras, and episode limits. No physics.', 'inputSchema': empty},
        {'name': 'dexmate_observe', 'description': 'Current camera RGB images and measured robot proprioception. Does not advance physics. Camera subset optional.',
         'inputSchema': {'type': 'object', 'additionalProperties': False, 'properties': {'cameras': {'type': 'array', 'minItems': 1, 'maxItems': 4, 'uniqueItems': True, 'items': {'type': 'string', 'enum': list(CAMERAS)}}}}},
        {'name': 'dexmate_step', 'description': 'Execute synchronized bimanual absolute TCP actions. Each 16-D row is right [xyz metres, quaternion wxyz, opening 0..1], then left same. One row = 20 ms, both arms at once. Optional repeat repeats a single row, avoiding duplicate JSON; at most 50 executed rows. Returns updated RGB observation.',
         'inputSchema': {'type': 'object', 'required': ['actions'], 'additionalProperties': False, 'properties': {'repeat': steps, 'actions': {'type': 'array', 'minItems': 1, 'maxItems': MAX_ROWS, 'items': {'type': 'array', 'minItems': 16, 'maxItems': 16, 'items': {'type': 'number'}}}}}},
        {'name': 'dexmate_hold', 'description': 'Maintain current TCP and gripper references for steps * 20 ms. Physics continues; returns updated observation.',
         'inputSchema': {'type': 'object', 'required': ['steps'], 'additionalProperties': False, 'properties': {'steps': steps}}},
        {'name': 'dexmate_gripper', 'description': 'Set either or both synchronized jaw openings, holding both current TCP references. 0 closed, 1 open; runs steps * 20 ms.',
         'inputSchema': {'type': 'object', 'required': ['openings', 'steps'], 'additionalProperties': False, 'properties': {'steps': steps, 'openings': {'type': 'object', 'minProperties': 1, 'additionalProperties': False, 'properties': {s: {'type': 'number', 'minimum': 0, 'maximum': 1} for s in ('right', 'left')}}}}},
    ]


def finite(x):
    return isinstance(x, (float, int)) and not isinstance(x, bool) and math.isfinite(x)


def validate_request(name, args):
    if name == 'dexmate_teacher_memory':
        reject_unless(isinstance(args, dict) and set(args) <= {'operation','payload'}, 'Invalid memory keys')
        reject_unless(args.get('operation','read') in ('read','record_experiment','record_claim','update_progress','register_skill','checkpoint'), 'Invalid memory operation')
        reject_unless(isinstance(args.get('payload',{}),dict), 'Memory payload must be an object')
        reject_unless(len(json.dumps(args,allow_nan=False)) <= 256*1024, 'Memory payload too large')
        return
    if name in MOTION_TOOLS:
        validate_motion(name,args);return
    reject_unless(name in TOOLS and isinstance(args, dict), 'Unknown tool or arguments must be an object')
    allowed = {'dexmate_teacher_state': (), 'dexmate_status': (), 'dexmate_observe': ('cameras',), 'dexmate_step': ('actions','repeat'),
               'dexmate_hold': ('steps',), 'dexmate_gripper': ('openings', 'steps')}[name]
    reject_unless(not (set(args) - set(allowed)), 'Unknown arguments')
    if name == 'dexmate_observe' and 'cameras' in args:
        cs = args['cameras']
        reject_unless(isinstance(cs, list) and 1 <= len(cs) <= 4 and all(isinstance(c, str) and c in CAMERAS for c in cs) and len(set(cs)) == len(cs), 'Invalid cameras')
    if name == 'dexmate_step':
        rows = args.get('actions')
        reject_unless(isinstance(rows, list) and 1 <= len(rows) <= MAX_ROWS, 'Expected 1..50 action rows')
        if 'repeat' in args:
            reject_unless(len(rows) == 1 and type(args['repeat']) is int and 1 <= args['repeat'] <= MAX_ROWS,
                          'Repeat requires one row and a count 1..50')
        for row in rows:
            reject_unless(isinstance(row, list) and len(row) == 16 and all(finite(x) for x in row), 'Expected sixteen finite numbers per row')
            for start in (0, 8):
                pose = row[start:start+7]
                reject_unless(all(x == 0 for x in pose) or math.hypot(*pose[3:7]) > 1e-6, 'Nonzero TCP quaternion required')
                reject_unless(0 <= row[start+7] <= 1, 'Opening must be in [0,1]')
    if name in ('dexmate_hold', 'dexmate_gripper'):
        reject_unless(type(args.get('steps')) is int and 1 <= args['steps'] <= MAX_ROWS, 'Expected steps 1..50')
    if name == 'dexmate_gripper':
        op = args.get('openings')
        reject_unless(isinstance(op, dict) and bool(op) and not set(op) - {'left', 'right'} and all(finite(v) and 0 <= v <= 1 for v in op.values()), 'Expected left/right openings in [0,1]')

# Stage-1 proposal implemented as distinct tools; joint paths never use the
# legacy 16-D Cartesian interpretation. Scene policy is enforced by the host.
MOTION_TOOLS = frozenset({'dexmate_motion_plan','dexmate_motion_preview','dexmate_motion_execute','dexmate_motion_status','dexmate_motion_stop'})

def motion_definitions():
    ident={'type':'string','minLength':1,'maxLength':64}
    empty={'type':'object','properties':{},'additionalProperties':False}
    target={'type':'object','additionalProperties':False,'required':['frame','tcp','position_m','quaternion_wxyz'],'properties':{'frame':{'type':'string','enum':['world']},'tcp':{'type':'string','enum':['tcp_L','tcp_R']},'position_m':{'type':'array','minItems':3,'maxItems':3,'items':{'type':'number'}},'quaternion_wxyz':{'type':'array','minItems':4,'maxItems':4,'items':{'type':'number'}}}}
    return [
      {'name':'dexmate_motion_plan','description':'Plan one unladen arm in free space using OMPL CPU and the operator-selected collision world policy reported by dexmate_motion_status. Explicit simulator_snapshot uses full current simulator geometry with quasi-static guards; default unknown cardboard coverage refuses. No physics stepping, intentional contact or held-object planning. Both arms and measured jaws must be settled.','inputSchema':{'type':'object','additionalProperties':False,'required':['episode_id','expected_step_id','arm','target'],'properties':{'episode_id':ident,'expected_step_id':{'type':'integer','minimum':0},'arm':{'type':'string','enum':['right','left']},'target':target}}},
      *[{'name':'dexmate_motion_'+name,'description':description,'inputSchema':{'type':'object','additionalProperties':False,'required':['plan_id'],'properties':{'plan_id':ident}}} for name,description in [('preview','Return robot joint trajectory for a cached plan without advancing physics.'),('execute','Execute a fresh single-use joint plan via existing motor-reference controller; <=50 rows, 20ms per row. Reports command completion separately from measured goal. No Cartesian IK re-solving.')]],
      {'name':'dexmate_motion_status','description':'Planner ownership, last result, limits and coverage. No physics.','inputSchema':{'type':'object','additionalProperties':False,'properties':{'plan_id':ident}}},
      {'name':'dexmate_motion_stop','description':'Request cancellation. MCP calls serialize, so use operator out-of-band STOP for interruption during an executing call. Does not advance physics.','inputSchema':{'type':'object','additionalProperties':False,'properties':{'reason':{'type':'string','maxLength':256}}}},
    ]

def validate_motion(name,args):
    expected={'dexmate_motion_plan':({'episode_id','expected_step_id','arm','target'},set()),'dexmate_motion_preview':({'plan_id'},set()),'dexmate_motion_execute':({'plan_id'},set()),'dexmate_motion_status':(set(),{'plan_id'}),'dexmate_motion_stop':(set(),{'reason'})}
    required,optional=expected[name];reject_unless(isinstance(args,dict) and required<=set(args) and set(args)<=required|optional,'Invalid motion-tool keys')
    for key in ['plan_id','episode_id']:
        if key in args:reject_unless(isinstance(args[key],str) and 0<len(args[key])<=64,'Invalid '+key)
    if 'reason' in args:reject_unless(isinstance(args['reason'],str) and len(args['reason'])<=256,'Invalid stop reason')
    if name=='dexmate_motion_plan':
        reject_unless(args['arm'] in ('right','left'),'Invalid arm');reject_unless(type(args['expected_step_id']) is int and args['expected_step_id']>=0,'Invalid step');t=args['target'];reject_unless(isinstance(t,dict) and set(t)=={'frame','tcp','position_m','quaternion_wxyz'},'Invalid target keys');reject_unless(t['frame']=='world' and t['tcp']==('tcp_R' if args['arm']=='right' else 'tcp_L'),'Target TCP/frame mismatch')
        for key,n in [('position_m',3),('quaternion_wxyz',4)]:reject_unless(isinstance(t[key],list) and len(t[key])==n and all(finite(x) for x in t[key]),'Invalid '+key)
        reject_unless(math.hypot(*t['quaternion_wxyz'])>1e-6,'Quaternion must be nonzero')
