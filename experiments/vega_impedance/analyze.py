"""Summarize recorded smoke data without executing more robot motion."""
from pathlib import Path
import gzip, hashlib, json
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/'validation/community_control'


def main():
    comparison = json.loads((OUT/'comparison.json').read_text())
    rows = []
    for case in comparison['cases']:
        path = OUT/(case['kind']+'_'+case['route'])
        with gzip.open(path/'state_trace.json.gz','rt') as f:
            trace = json.load(f)
        tracking = {}
        for phase in sorted({r['phase'] for r in trace}):
            errors = [float(np.linalg.norm(np.array(r['tcp'][:3])-r['target'][:3])) for r in trace if r['phase']==phase]
            tracking[phase] = {'mean_error_m':float(np.mean(errors)), 'max_error_m':max(errors), 'last_error_m':errors[-1]}
        assert case['result']=='pass', (case['kind'],case['route'],case.get('error'))
        assert not any(case['warnings'])
        for name,value in case['source_sha256'].items():
            assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==value, name
        rows.append({'task':case['kind'], 'controller':case['route'], 'status':case['result'],
                     'movement_m':case.get('actual_displacement_m'),
                     'final_position_error_m':case.get('final_tcp_error_m'),
                     'peak_contact_normal_N':case['peak_contact_normal_N'],
                     'max_penetration_m':case['max_hand_base_penetration_m'],
                     'released':case.get('released'), 'wall_seconds':case['wall_seconds'],
                     'tracking':tracking, 'controller_details':case['controller']})
    touch = {r['controller']:r for r in rows if r['task']=='touch'}
    reduction = 1-touch['community']['peak_contact_normal_N']/touch['baseline']['peak_contact_normal_N']
    summary = {'rows':rows, 'matched_initial_states':comparison['matched_initial_states'],
               'observed_peak_force_reduction_fraction':reduction,
               'conclusion':'Pinned CPU control module integrates and passes these movement/contact smokes. This does not establish better pushing, grasping, force regulation, or realtime performance.'}
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    text = ['CPU社区控制模块复用验证：四项测试通过。',
            '上游提交：'+comparison['upstream']['commit'],
            '基线模型、物理参数和源码哈希一致；两种控制器在各自对照中起始状态完全相同。','']
    for row in rows:
        text.append(f"{row['task']} / {row['controller']}: {row['status']}; peak force={row['peak_contact_normal_N']:.6f}N; final error={row['final_position_error_m']}; wall={row['wall_seconds']:.3f}s")
    text += ['', f'本次轻触峰值下降{reduction*100:.2f}%，仅单次确定性对照，不能推断一般优越性。',
             '移动过程的最大位置误差两者均约1.1mm；不能只凭稳态终点误差评价整个轨迹。',
             '复用的是姿态阻抗控制，不是恒力控制；5N是离散观测的中止阈值，不是力上限保证。',
             '上游模块与许可证原样保留，仅本地适配七个右臂DOF、tcp_R与现有电机力矩限制。',
             '左臂和夹爪继续使用原控制器，躯干保持固定。源默认增益未调参。',
             'CPU-only；未导入cuRobo/CUDA。上游PoseTrajectory未使用；本次使用20ms位姿目标与零期望速度/加速度。',
             '每0.25ms物理步计算力矩及观测接触。Python控制计算开销较高，尚未验证实时性。',
             '没有进行新的推箱、抓取、折叠或物理锁定测试；实验未替换基线默认控制器。',
             '复现：见experiments/vega_impedance/README.md；force_trace.csv.gz与state_trace.json.gz保留离散证据。']
    (OUT/'REPORT.txt').write_text('\n'.join(text)+'\n')
    print(json.dumps({'four_cases_pass':True,'matched':comparison['matched_initial_states'],'observed_force_reduction':reduction},ensure_ascii=False))


if __name__=='__main__':main()
