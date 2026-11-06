"""Explicit runtime-geometry pair distances; no implicit contact-mask loopholes."""
import numpy as np,mujoco
class CollisionChecker:
    def __init__(self,robot,side,clearance):
        self.robot=robot;self.side=side;self.clearance=clearance;self.samples=0;self.last_conflict=None;m=robot.m
        physical=[g for g in range(m.ngeom) if m.geom_contype[g] or m.geom_conaffinity[g]]
        is_robot=lambda g:(m.body(int(m.geom_bodyid[g])).name or '').startswith('dexmate_')
        active=lambda g:(m.body(int(m.geom_bodyid[g])).name or '').startswith('dexmate_'+('R' if side=='right' else 'L')+'_')
        self.pairs=[];self.excluded=[]
        for n,a in enumerate(physical):
            for b in physical[n+1:]:
                if not (active(a) or active(b)):continue
                ba,bb=int(m.geom_bodyid[a]),int(m.geom_bodyid[b]);na,nb=m.body(ba).name,m.body(bb).name
                reason=None
                if ba==bb:reason='same rigid body'
                elif is_robot(a) and is_robot(b) and m.body_weldid[ba]==m.body_weldid[bb]:reason='same welded assembly'
                elif frozenset([na,nb]) in robot.exclusions:reason='existing explicit source exclusion'
                elif m.body_parentid[ba]==bb or m.body_parentid[bb]==ba:reason='native adjacent joint-attachment geometry (parent-child filter)'
                if reason:self.excluded.append({'bodies':[na,nb],'reason':reason});continue
                self.pairs.append((a,b))
    def check(self,q):
        self.samples+=1;self.robot.set_arm(self.side,q);m,d=self.robot.m,self.robot.d
        for a,b in self.pairs:
            if m.geom_type[a]!=mujoco.mjtGeom.mjGEOM_PLANE and m.geom_type[b]!=mujoco.mjtGeom.mjGEOM_PLANE:
                if np.linalg.norm(d.geom_xpos[a]-d.geom_xpos[b])>m.geom_rbound[a]+m.geom_rbound[b]+self.clearance:continue
            # distmax saturates at clearance; broadphase also skips distant pairs.
            # This is a threshold test, never an exact global minimum measurement.
            distance=float(mujoco.mj_geomDistance(m,d,a,b,self.clearance,None))
            if distance<self.clearance-1e-9:
                self.last_conflict={'geoms':[m.geom(a).name,m.geom(b).name],'distance_m':distance};return False
        return True
    def edge(self,a,b,cfg,cancel):
        count=max(1,int(np.ceil(np.max(abs(np.asarray(b)-a))/cfg['edge_joint_resolution_rad'])))
        for u in np.linspace(0,1,count+1):
            if cancel() or self.samples>=cfg['max_validation_samples'] or not self.check((1-u)*np.asarray(a)+u*np.asarray(b)):return False
        return True
