"""Bounded display orchestration; the motion service remains execution authority.

Importing this module creates no simulator, viewer, agent or process.
"""
import math


def execute_for_display(service, plan_id, cancelled):
    chunks=[];cursor=0
    plan_limit=service.cfg.get('max_plan_rows',service.cfg.get('max_trajectory_rows',service.cfg['max_rows']))
    execute_limit=service.cfg.get('max_execute_rows',service.cfg['max_rows'])
    limit=math.ceil(plan_limit/execute_limit)
    for _ in range(limit):
        if cancelled():
            service.stop('operator/display cancelled before next chunk')
            return {'chunks':chunks,'last_result':chunks[-1] if chunks else None,'cancelled':True}
        result=service.execute(plan_id,expected_cursor=cursor)
        chunks.append(result)
        if result.get('status')!='paused':
            stopped=bool(cancelled())
            if stopped:service.stop('operator/display cancelled during execution')
            return {'chunks':chunks,'last_result':result,'cancelled':stopped}
        next_cursor=result.get('cursor')
        if type(next_cursor) is not int or next_cursor<=cursor:
            service.stop('display received invalid/nonadvancing paused cursor')
            return {'chunks':chunks,'last_result':result,'cancelled':True,'refusal':'DISPLAY_INVALID_CURSOR'}
        cursor=next_cursor
    service.stop('bounded display continuation call limit reached')
    return {'chunks':chunks,'last_result':chunks[-1] if chunks else None,
            'cancelled':True,'refusal':'DISPLAY_CONTINUATION_CALL_LIMIT'}
