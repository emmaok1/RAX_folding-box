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
        # Same conservative sphere broad phase, batched instead of802 tiny
        # Python/numpy calls for every sample. Exact near-pair distance checks
        # and the original pair/exclusion coverage below remain unchanged.
        self._a=np.asarray([a for a,b in self.pairs],dtype=int)
        self._b=np.asarray([b for a,b in self.pairs],dtype=int)
        self._plane=(m.geom_type[self._a]==mujoco.mjtGeom.mjGEOM_PLANE)|(m.geom_type[self._b]==mujoco.mjtGeom.mjGEOM_PLANE)
        self._radius=m.geom_rbound[self._a]+m.geom_rbound[self._b]+self.clearance+1e-12
    def check(self,q):
        self.samples+=1;self.robot.set_arm(self.side,q);m,d=self.robot.m,self.robot.d
        offset=d.geom_xpos[self._a]-d.geom_xpos[self._b]
        near=self._plane|(np.einsum('ij,ij->i',offset,offset)<=self._radius**2)
        for i in np.flatnonzero(near):
            a,b=self.pairs[i]
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
