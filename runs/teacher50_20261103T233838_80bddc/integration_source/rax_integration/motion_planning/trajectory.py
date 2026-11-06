"""Stop-at-waypoint quintic retiming; endpoints at exact 20 ms intervals."""
import numpy as np
def retime(path,cfg,velocity_limits):
    rows=[];speed=np.minimum(velocity_limits,cfg['speed_limit_rad_s']);acc=cfg['acceleration_limit_rad_s2'];dt=cfg['control_dt_s']
    for a,b in zip(path[:-1],path[1:]):
        delta=np.abs(np.asarray(b)-a)
        if max(delta)<1e-8:continue
        duration=max(float(np.max(1.875*delta/speed)),float(np.sqrt(np.max(5.774*delta/acc))),dt)
        steps=int(np.ceil(duration/dt))
        if len(rows)+steps>cfg['max_rows']:return None
        for t in range(1,steps+1):
            u=t/steps;rows.append((np.asarray(a)+(10*u**3-15*u**4+6*u**5)*(np.asarray(b)-a)).tolist())
    return rows or [np.asarray(path[-1]).tolist()]
