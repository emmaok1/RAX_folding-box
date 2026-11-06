"""Established OMPL geometric search, with caller-owned validity/cancellation."""
import numpy as np
from ompl import base as ob,geometric as og,util as ou

def geometric_path(start,goals,limits,valid,cancel,cfg):
    dimension=len(start);space=ob.RealVectorStateSpace(dimension);bounds=ob.RealVectorBounds(dimension)
    for i,(lo,hi) in enumerate(limits):bounds.setLow(i,float(lo));bounds.setHigh(i,float(hi))
    space.setBounds(bounds);ss=og.SimpleSetup(space);ou.RNG.setSeed(cfg['seed']);ss.setStateValidityChecker(ob.StateValidityCheckerFn(lambda s:valid(np.array([s[i] for i in range(dimension)]))));space.setLongestValidSegmentFraction(cfg['edge_joint_resolution_rad']/space.getMaximumExtent());planner=og.RRTConnect(ss.getSpaceInformation());planner.setRange(.15);ss.setPlanner(planner)
    for goal in goals:
        if cancel():break
        a,b=ob.State(space),ob.State(space)
        for j in range(dimension):a[j]=float(start[j]);b[j]=float(goal[j])
        ss.clear();ss.setStartAndGoalStates(a,b,1e-7);ss.setup()
        if ss.getSpaceInformation().checkMotion(a(),b()):return [np.asarray(start).copy(),np.asarray(goal).copy()],'OMPL-checked direct connection'
        done=ss.solve(ob.PlannerTerminationCondition(ob.PlannerTerminationConditionFn(cancel)))
        if done and ss.haveExactSolutionPath():
            # Every shortcut is checked by the same SpaceInformation validator.
            ss.simplifySolution(.02);path=ss.getSolutionPath();return [np.array([path.getState(i)[j] for j in range(dimension)]) for i in range(path.getStateCount())],'RRTConnect'
    return None,None
