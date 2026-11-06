"""Analyze extended contact tests without running more physics."""
from pathlib import Path
import json, gzip, hashlib
import numpy as np
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OUT=ROOT/'validation/community_contact_tests'


def main():
    report=json.loads((OUT/'comparison.json').read_text())
    assert all(report['matched_initial_states'].values()), 'Comparison starts differ'
    for name,sha in report['experiment_source_sha256'].items():
        assert hashlib.sha256((HERE/name).read_bytes()).hexdigest()==sha,name
    rows=[]
    for case in report['cases']:
        for name,sha in case['source_sha256'].items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==sha,name
        out=OUT/(case['task']+'_'+case['controller'])
        with gzip.open(out/'states.json.gz','rt') as f:states=json.load(f)
        reference=case.get('comparison_start',{}).get('hinges_deg',{})
        max_hinge=max((abs(state['hinges_deg'][n]-reference[n]) for state in states for n in reference),default=0.)
        row={k:case.get(k) for k in ['task','controller','protocol_completed','error','translation_goal_achieved','blank_translation_mm','hand_action_delta_mm','peak_sum_normal_N','max_hand_box_penetration_m','released','warnings','wall_seconds']}
        row['max_transient_hinge_change_deg']=max_hinge
        if 'final' in case:
            row['base_yaw_change_deg']=case['final']['box_euler_deg'][2]-case['comparison_start']['box_euler_deg'][2]
        row['force_statistics']=case['force_statistics']
        row['controller_details']=case['controller_details']
        if case['task']=='sidepush10mm' and case.get('blank_translation_mm'):
            row['toward_table_edge_mm']=-case['blank_translation_mm'][0]
        rows.append(row)
    lines=['社区阻抗控制扩展对照：接触后0.5mm参考位置下压、10mm侧推。',
           '同一任务的两种控制器从相同qpos/qvel/ctrl开始；固定几何、摩擦、增益、控制频率和接近路径。',
           '记录每0.25ms物理步。5N是检测后中止阈值，不是强制力上限。','']
    for row in rows:
        lines.append(f"{row['task']} / {row['controller']}: protocol_completed={row['protocol_completed']}; error={row['error']}; peak_total_normal={row['peak_sum_normal_N']:.6f}N")
        if row['task']=='press05mm':
            hold=row['force_statistics'].get('pressed_hold',{})
            lines.append(f"  下压指令后停留1s：平均法向力{hold.get('mean_sum_normal_N')}N，标准差{hold.get('std_sum_normal_N')}N。")
        else:
            lines.append(f"  纸盒最终平移{row['blank_translation_mm']}mm；手在推动阶段运动{row['hand_action_delta_mm']}mm；偏航变化{row.get('base_yaw_change_deg')}deg；10mm目标（1mm容差）={row['translation_goal_achieved']}。")
        lines.append(f"  最大瞬态折痕变化={row['max_transient_hinge_change_deg']:.6f}deg；撤离={row['released']}；手/箱最大穿透={row['max_hand_box_penetration_m']}m。")
    lines+=['','局限：每种组合仅一次确定性实验；0.5mm是参考位置变化，不能解释为纸板被压缩0.5mm。',
            '阻抗控制不是恒力控制。保持同一侧推姿态，不重新优化接触策略，不调控制器增益。',
            '脚本完成、无警告、低峰值力均不等于实现了10mm物体平移。未执行更大位移、抓取或折叠。',
            'CPU执行开销较高；未验证实时性。基线和上游控制模块保持不变。']
    summary={'matched_initial_states':report['matched_initial_states'],'rows':rows,
             'limits':report['limits'],'report_source_sha256':report['experiment_source_sha256']}
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    (OUT/'REPORT.txt').write_text('\n'.join(lines)+'\n')
    for row in rows:
        print(json.dumps({k:row.get(k) for k in ['task','controller','protocol_completed','error','peak_sum_normal_N','toward_table_edge_mm','translation_goal_achieved']},ensure_ascii=False))


if __name__=='__main__':main()
