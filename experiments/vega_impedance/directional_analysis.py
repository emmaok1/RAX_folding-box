"""Evaluate the original/directional profile test without additional motion."""
from pathlib import Path
import gzip, hashlib, json
import numpy as np
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OUT=ROOT/'validation/directional_control'


def main():
    cases={}
    for name in ['original','directional']:
        path=OUT/('sidepush10mm_'+name)
        r=json.loads((path/'result.json').read_text())
        for source,sha in r['source_sha256'].items():assert hashlib.sha256((ROOT/source).read_bytes()).hexdigest()==sha,source
        for source,sha in r['experiment_source_sha256'].items():assert hashlib.sha256((HERE/source).read_bytes()).hexdigest()==sha,source
        with gzip.open(path/'states.json.gz','rt') as f:states=json.load(f)
        push=[s for s in states if s['phase']=='push10mm']
        tracking={}
        if push:
            error=np.array([np.array(s['tcp'][:3])-s['target'][:3] for s in push])*1000
            tracking={'max_abs_xyz_mm':np.max(abs(error),axis=0).tolist(), 'mean_abs_xyz_mm':np.mean(abs(error),axis=0).tolist()}
        final=r.get('final',r.get('stopped_state'))
        delta=(np.array(final['box_pose'][:3])-r['initial_box_pose'][:3])*1000
        yaw=final['box_euler_deg'][2]-r['comparison_start']['box_euler_deg'][2]
        yaw=(yaw+180)%360-180
        source_hinges=r['comparison_start']['hinges_deg']
        max_hinge=max((abs(s['hinges_deg'][n]-source_hinges[n]) for s in states for n in source_hinges),default=0.)
        cases[name]={'protocol_completed':r['protocol_completed'],'error':r.get('error'),
                     'state_type':'after_release' if 'final' in r else 'stopped_mid_run',
                     'released':r.get('released',False),
                     'toward_edge_mm':float(-delta[0]), 'side_drift_mm':float(delta[1]),
                     'yaw_change_deg':yaw,'hand_action_delta_mm':r.get('hand_action_delta_mm'),
                     'peak_sum_normal_N':r['peak_sum_normal_N'],
                     'max_hand_box_penetration_m':r['max_hand_box_penetration_m'],
                     'warnings':r['warnings'],'hand_tracking_during_push':tracking,
                     'max_transient_hinge_change_deg':max_hinge,
                     'controller_details':r['controller_details'],
                     'initial_state_sha256':r['initial_state_sha256']}
    old,new=cases['original'],cases['directional']
    matched=old['initial_state_sha256']==new['initial_state_sha256']
    assert matched,'Unmatched initial states'
    comparable=all(c['protocol_completed'] and c['released'] for c in cases.values())
    outcome={'matched_initial_state':matched,'cases':cases,'released_results_comparable':comparable,
             'deltas_modified_minus_original':{
                 'toward_edge_mm':new['toward_edge_mm']-old['toward_edge_mm'],
                 'absolute_side_drift_mm':abs(new['side_drift_mm'])-abs(old['side_drift_mm']),
                 'absolute_yaw_deg':abs(new['yaw_change_deg'])-abs(old['yaw_change_deg']),
                 'peak_force_N':new['peak_sum_normal_N']-old['peak_sum_normal_N']},
             'limits':'One trial/profile, same contact geometry/motion/physics. WorldY/Z gains only. No force regulation or global guarantee. A smaller hand tracking error need not improve object motion.'}
    lines=['世界坐标方向性刚度修改：单次同姿态10mm侧推对照。',
           '推动X刚度保持900N/m；Y由900升至2700，Z由900升至1800。',
           '平移阻尼按刚度比例平方根缩放；旋转/零空间控制、姿态、几何、摩擦、OMPL路径和电机限制不变。',
           f'起始qpos/qvel/ctrl完全相同：{matched}；两组均完成撤离：{comparable}。','']
    for name,c in cases.items():
        lines += [name+': '+json.dumps({k:c[k] for k in ['protocol_completed','error','state_type','toward_edge_mm','side_drift_mm','yaw_change_deg','peak_sum_normal_N','hand_action_delta_mm','hand_tracking_during_push','max_hand_box_penetration_m','warnings']},ensure_ascii=False)]
    lines+=['','差值（修改版−原版，侧偏/转动比较绝对值）：'+json.dumps(outcome['deltas_modified_minus_original'],ensure_ascii=False),
            '这是一项局部增益修改的单次实验，不证明一般优越性；不自动替换默认控制器。',
            '5N是离散观测后中止阈值，不是硬力上限。没有执行更大位移、抓取或折叠。',
            '上游控制源码和基线模型/物理参数保持不变。']
    (OUT/'comparison.json').write_text(json.dumps(outcome,indent=2)+'\n')
    (OUT/'REPORT.txt').write_text('\n'.join(lines)+'\n')
    print(json.dumps(outcome['deltas_modified_minus_original'],ensure_ascii=False))
    for name,c in cases.items():print(name,json.dumps({k:c[k] for k in ['protocol_completed','error','toward_edge_mm','side_drift_mm','yaw_change_deg','peak_sum_normal_N']},ensure_ascii=False))


if __name__=='__main__':main()
