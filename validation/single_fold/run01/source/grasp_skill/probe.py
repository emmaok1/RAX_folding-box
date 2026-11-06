"""Bounded offline geometry/IK search. Never advances or teleports execution."""
from scene import *
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(ROOT/'experiments/vega_impedance'))
from test_support import Validity

def candidate(e):
    # Local x closes vertically; local z enters horizontally from near x edge.
    R=np.array([[0,0,1],[0,-1,0],[1,0,0.]])
    quat=Rotation.from_matrix(R).as_quat()[[3,0,1,2]]
    base=e.data.body('box_base').xpos
    # Pad mesh mean is z=.158627, x=-7um, y=-1.5mm relative to gripper base.
    center=base+np.array([-.138,0,0]);target=np.r_[center-R@np.array([-.000007,-.0015,-.0013731]),quat]
    pre=target.copy();pre[0]-=.055
    valid=Validity(e)
    trials=[]
    for roll in [0,np.pi]:
        if roll: quat=Rotation.from_matrix(R@Rotation.from_rotvec([0,0,roll]).as_matrix()).as_quat()[[3,0,1,2]];target[3:]=quat;pre[3:]=quat
        seeds=[e.arms['right']['qcmd'],np.array(e.cfg['home_right_rad'])+np.array([0,0,0,-.3,0,0,0])]
        for seed in seeds:
            q,err=e.inverse_kinematics('right',pre,iterations=250,seed=seed)
            ok=err['position_m']<.001 and err['orientation_rad']<.01 and valid(q)
            trials.append({'pre':pre.tolist(),'q':q.tolist(),'err':err,'valid':ok})
            if ok:
                qg,eg=e.inverse_kinematics('right',target,iterations=250,seed=q)
                if eg['position_m']<.001 and eg['orientation_rad']<.01 and valid(qg):return target,pre,q,qg,trials
    raise RuntimeError('No candidate '+json.dumps(trials))
if __name__=='__main__':
 e=DiagnosticEnv();print(candidate(e))
