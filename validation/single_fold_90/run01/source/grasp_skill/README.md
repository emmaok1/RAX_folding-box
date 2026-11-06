# 单面板物理夹取诊断

这是独立的简化场景，**不证明原桌面整张纸盒任务已解决**。程序从现有 Vega1U + DexGripper S 正常 home 出发，使用 OMPL RRTConnect 和原有电机控制；在接触阶段复用现有固定参数 `RightArmImpedance`。

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 .venv/bin/python experiments/grasp_skill/run.py --out validation/grasp_skill/new_run
```

输出目录必须不存在，避免覆盖先前证据。依赖已有 `.venv`、`experiments/vega_impedance` 和任务本地 `validation/planner_comparison/ompl/deps`（OMPL 2.0.1）。每次运行先做一个零偏移 smoke；仅成功后执行 y=+2mm、-2mm 两个全新初始化、重算抓取姿态的试验。零偏移smoke失败则不运行偏移试验；smoke通过后两偏移为独立新初始化，即使其中一个失败仍执行另一个。可用 `--offsets -0.002` 只执行尚未测过的条件，避免重跑已通过试验。

场景直接保留 `models/scene.xml` 中机器人、桌子、夹爪、执行器、碰撞排除项；仅取原 `box_base` 面板（300×220×4mm，26.4g，等效密度100kg/m³、面密度400g/m²，原惯性、接触属性）和真实固定支座（130×150×220mm，桌面 z=.92m，顶面 z=1.14m）。当前支座中心是 (0,-.65,1.03)m，板中心初始为 (0,-.65+offset,1.1422)m。板保持自由体，靠重力落在支座上；其近侧悬边给下颚约180mm高度净空。没有焊接、吸附、mocap、板直接受控、重力关闭或执行时 qpos 写入。

夹面/纸板接触显式保持 μ=.6、支座/纸板 μ=.4、condim=3、solref=.004 1、solimp=.99 .999 .0001 .5 2、margin=10µm。板本体碰撞参数不变。支座沿用桌面接触对参数，不将夹爪自身默认 μ=.8 错当成板/夹爪接触系数。

抓取几何针对真实 DexGripper 网格：闭合轴局部 x、进入方向局部 z，设计角 .012rad 时 pad 内表面间距4mm，pad 网格相对 TCP 的中心偏移约 (-7µm,-1.5mm,-1.373mm)。预抓位置比抓取位置退55mm。检查正常 home、预抓、插入终点和密集关节路径的全机器人碰撞；离线 IK 使用 scratch data。OMPL 路径由原关节 PD 电机执行，接触阶段切到现有固定阻抗实现。

闭合首先缓慢降低位置参考，保持非零命令以避免默认 .4Nm 突变；随后用已有力矩模式与 `effort_scale` 反馈调节，目标为两颚法向力总和3N，恒定闭合力矩分量范围 .02–.30Nm，该分量变化率最多 .05Nm/s（原电机控制仍叠加原阻尼/重力补偿，总电机力矩按原生±.6Nm限幅）；反馈持续至抬升、保持及放置阶段（原限制仍为 .6Nm）。5N 是每0.25ms观察的终止阈值，**不是硬力上限**；越界可能发生并会报告超调。每个非预期机器人接触也检查并终止。异常时尝试原位张开，仅板仍受支座支撑时尝试退回预抓点；失败会明确记录未完成回撤。

成功要求：两组对向颚均有>0.05N实测接触、实际板 COM 比抬升前高至少10mm（命令抬升25mm）、至少1秒保持、从抬升前开始相对手爪平移漂移<3mm/角漂移<.05rad；在线若超过8mm/.10rad则终止，并分别记录两侧pad实际接触（pad策略诊断，不替代真实两颚承载/保持判据）、无支座承托，再控制放回、张开、退手，板继续稳定受支撑。指令结束、接触检测与物理保持分别报告。

每轮输出 `result.json`、`forces.csv.gz`（每物理步）、`contacts.csv.gz`（20ms接触采样）、`trajectory.json.gz`、MP4及阶段PNG；每次运行还保存前后核心文件 SHA256、实验脚本快照与生成的独立 MJCF。碰撞检查是离散采样，不声称连续碰撞保证。

## 思路来源与复用边界

- 阅读了 [ManiSkill pick_cube](https://github.com/mani-skill/ManiSkill/blob/main/mani_skill/examples/motionplanning/panda/solutions/pick_cube.py) 的对象几何定姿、预抓—闭合—移动阶段组织；它原为 Panda/SAPIEN，未安装/复制其运行栈。本实验独立实现适配 DexGripper 的少量几何和状态机。
- 阅读了 [robosuite `_check_grasp`](https://github.com/ARISE-Initiative/robosuite/blob/master/robosuite/environments/manipulation/manipulation_env.py) 的对向几何组同时接触判定；本实验额外验证实际质心抬升、无支撑保持、相对姿态与释放，未将接触等同夹稳。
- 直接 import 仓库既有 `adapter.py`（其 vendor 许可证和 UPSTREAM.json 保持原处）、`test_support.py`、OMPL `planner.py` 与 `FoldingBoxEnv` 的电机接口。未覆盖旧实验或修改核心文件。

已知开发过程：`validation/grasp_skill/run01` 保留了最初支座 y=-.4m 与正常 home 相交导致的初始化失败（非抓取失败）；该缺陷修复为前置碰撞拒绝与 y=-.65m 的无碰撞布置。不得将 run01 的“预期夹爪/板力0N”解读为无碰撞，大的非预期碰撞力见该轮 abort 详情及总报告。

分析所有已保存运行：`.venv/bin/python experiments/grasp_skill/analyze.py`。`run02` 是闭合反馈未延续到抬升阶段的旧版本；`run03` 是有持续力反馈和全抬升滑移监测的有界修正版。每轮代码快照可复查版本差异。

`run04` 仅修正 `run03` 把双pad接触误作必需成功条件的额外门槛：保留双pad诊断字段，成功依实际对向颚组、脱支撑抬升、全过程位姿保持与正常放置/释放。控制器、场景、轨迹、材料、力阈值均相同，两个运行目录内代码快照可直接diff。

最终验证条件：`run04/trial00` 零偏移、`run04/trial01` +2mm、`run05_minus2/trial00` -2mm。+2mm出现实测9.466N瞬态并按5N监测阈值中止（超调4.466N）；不得把单次基准通过描述成鲁棒成功。全历史与最终条件分别汇总在 `validation/grasp_skill/result.json` 和 `REPORT.txt`。

最终固定控制版本实测结果：零偏移完整通过（保持COM升高21.55–22.11mm，峰值3.770N）；+2mm在保持阶段因9.466N瞬态中止，恢复完成；-2mm完整通过（21.54–22.08mm，峰值4.283N）。三条件2通过/1失败，未证明鲁棒性。两次通过均已完成正常受控放置、张开与退手；并非仅抬升指令完成。所有通过/失败的新增pad2接触都为零，真实承载包含原厂指体。
