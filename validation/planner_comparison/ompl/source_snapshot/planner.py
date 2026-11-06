"""Real OMPL RRTConnect adapter. No fallback planner or physics mutation."""
from pathlib import Path
import sys
import time
import json
import hashlib
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / 'deps'))
_import_started = time.perf_counter()
from ompl import base as ob, geometric as og, util as ou
OMPL_IMPORT_SECONDS = time.perf_counter() - _import_started
ou.RNG.setSeed(2026)

class OMPLPlanner:
    def __init__(self, limits, output_dir=HERE, solve_seconds=5.0):
        self.limits = np.asarray(limits, dtype=float)
        self.output_dir = Path(output_dir)
        self.solve_seconds = float(solve_seconds)
        self.calls = []
        # Keep nanobind objects alive until shutdown: see wheel's upstream
        # lifetime/refcount warnings captured in stderr. Physics independent.
        self._objects = []

    def __call__(self, start, goal, validity, label):
        started = time.perf_counter()
        space = ob.RealVectorStateSpace(7)
        bounds = ob.RealVectorBounds(7)
        for i, (low, high) in enumerate(self.limits):
            bounds.setLow(i, float(low)); bounds.setHigh(i, float(high))
        space.setBounds(bounds)
        # Euclidean state-space interpolation <= 0.005 rad L2 implies each
        # joint increment <=0.005 rad. Discrete checks, not a CCD guarantee.
        segment_l2_rad = 0.005
        space.setLongestValidSegmentFraction(segment_l2_rad / space.getMaximumExtent())
        ss = og.SimpleSetup(space)
        counts = {'checks': 0, 'valid': 0, 'invalid': 0}
        def is_valid(state):
            q = np.array([state[i] for i in range(7)])
            okay = bool(validity(q))
            counts['checks'] += 1
            counts['valid' if okay else 'invalid'] += 1
            return okay
        ss.setStateValidityChecker(is_valid)
        initial = space.allocState(); target = space.allocState()
        for i in range(7):
            initial[i] = float(start[i]); target[i] = float(goal[i])
        ss.setStartAndGoalStates(initial, target)
        planner = og.RRTConnect(ss.getSpaceInformation())
        planner.setRange(0.15)
        ss.setPlanner(planner)
        ss.setup()
        setup_seconds = time.perf_counter() - started
        solve_started = time.perf_counter()
        status = ss.solve(self.solve_seconds)
        solve_seconds = time.perf_counter() - solve_started
        exact = bool(ss.haveExactSolutionPath())
        metadata = {'planner': 'OMPL RRTConnect', 'version': '2.0.1',
                    'label': label, 'rng_seed_initial': 2026,
                    'joint_dimension': 7, 'joint_limits_rad': self.limits.tolist(),
                    'range_rad': 0.15, 'collision_segment_l2_rad': segment_l2_rad,
                    'solve_budget_s': self.solve_seconds, 'setup_s': setup_seconds,
                    'ompl_import_s': OMPL_IMPORT_SECONDS, 'solve_s': solve_seconds,
                    'status': str(status), 'exact_solution': exact,
                    'collision_checks_solve': dict(counts), 'fallback_used': False}
        path = None
        if exact:
            native_path = ss.getSolutionPath()
            metadata['raw_waypoint_count'] = native_path.getStateCount()
            metadata['raw_path_l2_rad'] = native_path.length()
            simplify_started = time.perf_counter()
            # OMPL's own shortcutting validates each candidate using same checker.
            ss.simplifySolution(0.2)
            metadata['simplification_s'] = time.perf_counter() - simplify_started
            native_path = ss.getSolutionPath()
            path = np.array([[native_path.getState(j)[i] for i in range(7)]
                             for j in range(native_path.getStateCount())])
            metadata['waypoint_count'] = len(path)
            metadata['path_l2_rad'] = native_path.length()
            metadata['ompl_path_check'] = bool(native_path.check())
            metadata['collision_checks_total'] = dict(counts)
            np.save(self.output_dir / (label + '_path.npy'), path)
        metadata['adapter_total_s'] = time.perf_counter() - started
        self.calls.append(metadata)
        (self.output_dir / (label + '_planner.json')).write_text(json.dumps(metadata, indent=2))
        self._objects.append((space,bounds,ss,initial,target,planner,is_valid,status))
        return path, metadata
