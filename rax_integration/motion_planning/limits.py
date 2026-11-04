"""Separate whole-plan storage from per-call execution; legacy steps stay50."""


def plan_rows(cfg):
    value=cfg.get('max_plan_rows',cfg.get('max_trajectory_rows',cfg['max_rows']))
    if type(value) is not int or not 1<=value<=500:
        raise ValueError('max_plan_rows must be an integer1..500')
    return value


def execute_rows(cfg):
    value=cfg.get('max_execute_rows',cfg['max_rows'])
    if type(value) is not int or not 1<=value<=50:
        raise ValueError('max_execute_rows must be an integer1..50')
    return min(value,cfg['max_rows'],50)
