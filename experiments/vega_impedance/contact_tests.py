"""Matched-controller short pushing and contact-overtravel tests; CPU only."""
from pathlib import Path
from contextlib import nullcontext
import sys, json, time, hashlib, csv, gzip
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from PIL import Image
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0,str(ROOT))
from environment import FoldingBoxEnv
from test_support import Validity, dense, robot_name
from adapter import RightArmImpedance
OUT = ROOT/'validation/community_contact_tests'
SOURCES = ['environment.py','models/scene.xml','physics.json','robot_config.json','video_rebuild/config.json']


class Episode:
    def __init__(self, task, route):
        self.task, self.route = task, route
        self.out = OUT/(task+'_'+route)
        self.out.mkdir(parents=True,exist_ok=True)
        self.env = FoldingBoxEnv(placement='flat')
        self.m, self.d = self.env.model, self.env.data
        self.original_step = mujoco.mj_step
        self.phase = 'setup'
        self.active = False
        self.first = None
        self.steps = 0
        self.peaks = {}
        self.rows = []
        self.force_rows = []
        self.names = [self.m.geom(i).name or '' for i in range(self.m.ngeom)]
        self.hand = {i for i,n in enumerate(self.names) if n.startswith('jaw_pad_R_') or ('dexmate_R_gripper_' in n and '_collision_' in n)}
        prefix = 'card_base_' if task=='press05mm' else 'card_side_right_'
        self.target_geoms = {i for i,n in enumerate(self.names) if n.startswith(prefix)}
        self.contact_pairs = set()
        self.report = {'task':task,'controller':route,'protocol_completed':False,
                       'force_trip_N':5.,'penetration_trip_m':.001,
                       'source_sha256':{n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in SOURCES}}

    def __enter__(self):
        mujoco.mj_step = self.observe
        return self

    def __exit__(self,*args):
        mujoco.mj_step = self.original_step
        self.env.close()

    def observe(self,m,d,*args,**kwargs):
        self.original_step(m,d,*args,**kwargs)
        self.steps += 1
        peak=total=hand_pen=card_pen=0.
        force_on_box=np.zeros(3)
        for i,c in enumerate(d.contact):
            ids=[int(c.geom1),int(c.geom2)]
            names=[self.names[g] for g in ids]
            if any(n.startswith('card_') for n in names):
                card_pen=max(card_pen,-float(c.dist))
            if not any(robot_name(n) for n in names):continue
            f=np.zeros(6)
            mujoco.mj_contactForce(m,d,i,f)
            intended=any(g in self.hand for g in ids) and any(g in self.target_geoms for g in ids)
            if not intended and (f[0]>.05 or c.dist<-.0001):
                raise RuntimeError('Unexpected robot contact: '+str((names,float(f[0]),float(c.dist))))
            if intended:
                if not self.active and f[0]>.01:raise RuntimeError('Approach touched before controller comparison')
                fn=max(0.,float(f[0]))
                peak=max(peak,fn);total+=fn
                hand_pen=max(hand_pen,-float(c.dist))
                world=c.frame.reshape(3,3).T@f[:3]
                force_on_box += world if ids[1] in self.target_geoms else -world
                if fn>.01:
                    self.contact_pairs.add(tuple(names))
                    if self.first is None and self.phase in ('touch','edge_touch'):
                        self.first={'time_s':float(d.time),'force_N':fn,'geoms':names,'position_m':c.pos.tolist()}
        entry=self.peaks.setdefault(self.phase,{'peak_sum_normal_N':0.,'peak_per_contact_N':0.,'hand_box_penetration_m':0.,'all_card_penetration_m':0.})
        for k,v in zip(entry,[total,peak,hand_pen,card_pen]):entry[k]=max(entry[k],v)
        if self.active:self.force_rows.append([float(d.time),self.phase,total,peak,hand_pen,*force_on_box])
        if total>5 or hand_pen>.001 or card_pen>.001:raise RuntimeError('5N/1mm trip threshold exceeded: '+str((total,hand_pen,card_pen)))
        arm=self.env.arms['right'];q=d.qpos[arm['qad']]
        if np.any(q<arm['limits'][:,0]-.005) or np.any(q>arm['limits'][:,1]+.005):raise RuntimeError('Right joint range exceeded')
        if d.warning.number.any() or not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():raise RuntimeError('Warning/nonfinite dynamics')

    def state(self):
        e=self.env
        return {'time_s':float(self.d.time),'phase':self.phase,'tcp':e.tcp_pose('right').tolist(),
                'target':e.targets['right'].tolist(),'box_pose':self.d.joint('box_free').qpos.tolist(),
                'right_q':self.d.qpos[e.arms['right']['qad']].tolist(),
                'box_euler_deg':Rotation.from_matrix(self.d.body('box_base').xmat.reshape(3,3)).as_euler('xyz',degrees=True).tolist(),
                'hinges_deg':{n:float(np.rad2deg(self.d.joint('crease_'+n).qpos[0])) for n in e.panels if n!='base'}}

    def tick(self,target=None,joints=None):
        e=self.env
        if joints is not None:e.step_joint_targets({'right':joints})
        elif target is None:e.step()
        else:
            a={s:np.r_[e.targets[s],e.grippers[s]['command']] for s in e.arms}
            a['right']=np.r_[target,.3];e.step(a)
        if self.active:self.rows.append(self.state())

    def image(self,name):
        Image.fromarray(self.env.render_camera('back')).save(self.out/(name+'.png'))

    def clearance(self):
        base=self.d.body('box_base');result=[]
        for g in self.hand:
            if not self.names[g].startswith('jaw_pad_R_'):continue
            mesh=self.m.geom_dataid[g];a=self.m.mesh_vertadr[mesh];n=self.m.mesh_vertnum[mesh]
            world=self.m.mesh_vert[a:a+n]@self.d.geom_xmat[g].reshape(3,3).T+self.d.geom_xpos[g]
            local=(world-base.xpos)@base.xmat.reshape(3,3)
            assert (abs(local[:,0])<.15).all() and (abs(local[:,1])<.11).all()
            result.append(float(local[:,2].min())-.002)
        return min(result)

    def prepare(self):
        for _ in range(25):self.tick()
        self.report['initial_box_pose']=self.d.joint('box_free').qpos.tolist()
        if self.task=='press05mm':
            path=json.loads((HERE/'approach_path.json').read_text())
            speed=.4
        else:
            fixture=json.loads((HERE/'side_push_fixture.json').read_text())
            assert fixture['scene_sha256']==self.report['source_sha256']['models/scene.xml']
            path=fixture['path'];speed=.3
        valid=Validity(self.env)
        samples=list(dense(path))
        assert all(valid(q) for q in samples),'Recorded path is not collision-free'
        self.report['path_validation_samples']=len(samples)
        self.phase='approach'
        for q in dense(path,speed*self.env.control_dt):self.tick(joints=q)
        for _ in range(50):self.tick(joints=self.env.arms['right']['qcmd'].copy())
        if self.task=='press05mm':
            self.phase='near_surface';target=self.env.targets['right'].copy()
            for _ in range(500):
                if self.clearance()<=.002:break
                target[2]-=.0001;self.tick(target)
            else:raise RuntimeError('No near-surface start')
            for _ in range(50):self.tick(target)
            self.report['initial_clearance_m']=self.clearance()
            assert .0005<self.clearance()<.004
        self.report['comparison_start']=self.state()
        self.report['initial_state_sha256']=hashlib.sha256(self.d.qpos.tobytes()+self.d.qvel.tobytes()+self.d.ctrl.tobytes()).hexdigest()
        self.image('before')

    def touch(self):
        if self.task=='press05mm':
            self.phase='touch';target=self.env.targets['right'].copy();axis=2
        else:
            self.phase='edge_touch';target=np.array(json.loads((HERE/'side_push_fixture.json').read_text())['target_pose']);axis=0
        initial=target.copy()
        for k in range(1,1201):
            target[axis]=initial[axis]-k*.0005*self.env.control_dt
            self.tick(target)
            if self.first is not None:break
        else:raise RuntimeError('No intended contact within12mm')
        self.report['first_contact']=self.first
        self.report['touch_state']=self.state();self.image('contact')
        return target

    def task_action(self,target):
        initial=target.copy()
        if self.task=='press05mm':
            self.phase='press05mm'
            for k in range(1,51):
                target=initial.copy();target[2]-=.0005*k/50;self.tick(target)
            self.phase='pressed_hold'
            for _ in range(50):self.tick(target)
            self.report['commanded_extra_downward_reference_m']=.0005
        else:
            self.phase='push10mm'
            box_start=np.array(self.report['initial_box_pose'][:3])
            for k in range(1,1001):
                target=initial.copy();target[0]-=k*.0005*self.env.control_dt;self.tick(target)
                if self.d.body('box_base').xpos[0]-box_start[0]<=-.010:break
            self.report['commanded_push_after_touch_m']=float(target[0]-initial[0])
        self.report['after_action']=self.state();self.image('after_action')
        return target

    def retreat(self,target):
        self.phase='retreat';initial=target.copy();end=target.copy();end[2]+=.015
        if self.task=='sidepush10mm':end[0]+=.005
        for k in range(1,151):self.tick(initial+(end-initial)*k/150)
        self.phase='released_settle'
        for _ in range(100):self.tick(end)
        self.report['final']=self.state();self.image('released')
        self.report['released']=not any(r[2]>.01 for r in self.force_rows[-80:])
        assert self.report['released'],'Did not release contact'

    def save(self,controller):
        r=self.report
        r.update({'phase_metrics':self.peaks,'warnings':self.d.warning.number.tolist(),'physics_substeps':self.steps,
                  'contact_pairs':[list(p) for p in sorted(self.contact_pairs)],
                  'controller_details':controller.metadata() if controller else {'type':'Baseline IK/joint PD'},
                  'simulation_s':float(self.d.time)})
        r['peak_normal_N']=max((p['peak_per_contact_N'] for p in self.peaks.values()),default=0.)
        r['peak_sum_normal_N']=max((p['peak_sum_normal_N'] for p in self.peaks.values()),default=0.)
        r['max_hand_box_penetration_m']=max((p['hand_box_penetration_m'] for p in self.peaks.values()),default=0.)
        stats={}
        for phase in sorted({x[1] for x in self.force_rows}):
            vals=np.array([x[2:] for x in self.force_rows if x[1]==phase])
            stats[phase]={'mean_sum_normal_N':float(vals[:,0].mean()),'std_sum_normal_N':float(vals[:,0].std()),
                          'mean_box_force_xyz_N':vals[:,3:6].mean(axis=0).tolist(),
                          'peak_abs_box_force_xyz_N':np.max(abs(vals[:,3:6]),axis=0).tolist()}
        r['force_statistics']=stats
        if 'final' in r:
            r['blank_translation_mm']=((np.array(r['final']['box_pose'][:3])-r['initial_box_pose'][:3])*1000).tolist()
            r['max_final_hinge_change_deg']=max(abs(r['final']['hinges_deg'][n]-r['comparison_start']['hinges_deg'][n]) for n in r['final']['hinges_deg'])
        if 'after_action' in r:
            r['hand_action_delta_mm']=((np.array(r['after_action']['tcp'][:3])-r['touch_state']['tcp'][:3])*1000).tolist()
        if self.task=='sidepush10mm':r['translation_goal_achieved']=bool('blank_translation_mm' in r and r['blank_translation_mm'][0]<=-9.)
        with gzip.open(self.out/'forces.csv.gz','wt',newline='') as f:
            w=csv.writer(f);w.writerow(['time_s','phase','sum_normal_N','peak_normal_N','hand_penetration_m','box_force_x_N','box_force_y_N','box_force_z_N']);w.writerows(self.force_rows)
        with gzip.open(self.out/'states.json.gz','wt') as f:json.dump(self.rows,f)
        (self.out/'result.json').write_text(json.dumps(r,indent=2)+'\n')
        return r


def run_case(task,route):
    controller=None
    with Episode(task,route) as s:
        start=time.monotonic()
        try:
            s.prepare()
            context=RightArmImpedance(s.env) if route=='community' else nullcontext()
            with context as controller:
                s.active=True
                target=s.touch();target=s.task_action(target);s.retreat(target)
                if controller is not None:assert controller.calls>0
                s.report['protocol_completed']=True
        except Exception as exc:
            s.report['error']=repr(exc);s.report['stopped_state']=s.state()
            try:s.image('stopped')
            except Exception:pass
        s.report['wall_seconds']=time.monotonic()-start
        result=s.save(controller)
    print(json.dumps({k:result.get(k) for k in ['task','controller','protocol_completed','error','peak_sum_normal_N','blank_translation_mm','translation_goal_achieved','wall_seconds']},ensure_ascii=False),flush=True)
    return result


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    manifest=json.loads((HERE/'UPSTREAM.json').read_text())
    for item in manifest['files'].values():assert hashlib.sha256((HERE/item['local']).read_bytes()).hexdigest()==item['sha256']
    fixture=manifest['side_push_fixture'];assert hashlib.sha256((HERE/fixture['file']).read_bytes()).hexdigest()==fixture['sha256']
    cases=[run_case(task,route) for task in ['press05mm','sidepush10mm'] for route in ['baseline','community']]
    matched={task:len({r.get('initial_state_sha256') for r in cases if r['task']==task})==1 and all('initial_state_sha256' in r for r in cases if r['task']==task) for task in ['press05mm','sidepush10mm']}
    assert mujoco.get_mjcb_control() is None
    report={'upstream':manifest,'matched_initial_states':matched,'cases':cases,
            'experiment_source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [HERE/'contact_tests.py',HERE/'adapter.py',HERE/'test_support.py',HERE/'side_push_fixture.json']},
            'limits':'One deterministic trial each. Position-reference overtravel is not force regulation. Fixed side-push pose; no gain tuning. Force trip observes an exceedance, does not cap it. No physics/model edits or state teleportation.'}
    (OUT/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    print('MATCHED',matched,flush=True)


if __name__=='__main__':main()
