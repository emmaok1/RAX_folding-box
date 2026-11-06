# Vega 1U 社区阻抗控制：独立复用实验

复用项目：<https://github.com/luaiabuelsamen/vega_curobo>，固定提交和文件校验和见 [UPSTREAM.json](UPSTREAM.json)。`vendor/control.py` 和 MIT 许可证保持上游原样；本实验只使用 `GeometricImpedanceController` 类。完整 cuRobo/GPU 规划框架及上游直接写 qpos 的演示均不引入。

`adapter.py` 将类接到当前模型的右臂七个关节和 `tcp_R`。在每个物理步的 MuJoCo control callback 中，使用社区模块计算右臂力矩，并按现有电机范围裁剪。左臂和夹爪继续使用原控制器，躯干固定。回调在退出时恢复。执行过程只设置电机输入，不修改物体或机器人的执行状态。

社区原默认增益保留。零空间姿态参考在激活时记录；输入为20ms位姿目标，期望速度与加速度设为零。因此本实验不是上游带轨迹前馈的完整复现。该模块不自带碰撞避免、关节范围保护或恒力控制；实验脚本独立检查碰撞、关节范围、力和数值状态。

## 复现

从仓库根目录运行，复用现有 `.venv`。控制计算只依赖已有 NumPy/MuJoCo；截图使用当前已安装的 Pillow。若在新环境复现，先安装根目录的 `requirements.txt`，再安装本目录的 `requirements.txt`：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MUJOCO_GL=egl \
  .venv/bin/python experiments/vega_impedance/compare.py
.venv/bin/python experiments/vega_impedance/analyze.py
```

实际验证环境：MuJoCo3.4.0、NumPy1.26.4、Python3.11，CPU。基线的模型、配置和 `environment.py` 保持不变。`compare.py` 在独立进程内临时设置全局 MuJoCo 回调，因此不要将其与另一个物理仿真线程放进同一进程运行。

实验结果写入被忽略的 `validation/community_control/`，再次运行会覆盖该目录中同名测试结果。

## 测试范围

- 原控制器与社区控制器分别执行：空载向上5mm（1s平滑移动、2s停留）；以0.5mm/s轻触箱底后停留1s并撤离。
- 同一任务的两个控制器从完全相同的qpos/qvel/ctrl开始。接触测试的接近段统一回放 `approach_path.json`：这是先前OMPL2.0.1生成的路径，每次运行都在MuJoCo scratch数据上独立密集检查。这里只比较控制执行，不重新评测OMPL规划性能。
- 每0.25ms观测接触和数值状态；意外碰撞、5N总法向力或1mm手/箱穿透触发中止。阈值是观测后停止条件，不是物理力上限。
- `test_support.py` 提供独立碰撞检查与路径插值，不依赖之前的临时实验目录。

先运行对照再运行 `analyze.py`，会生成中文报告及包含运动过程误差的汇总。结果只覆盖这两个小测试，不证明推箱、抓取、折叠、可靠恒力控制或实时执行。

社区控制模块可能增加CPU开销。后续若需要实时执行，应单独评估控制频率、计算时间和离散稳定性。

## 扩展接触对照

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MUJOCO_GL=egl \
  .venv/bin/python experiments/vega_impedance/contact_tests.py
.venv/bin/python experiments/vega_impedance/contact_analysis.py
```

输出位于 `validation/community_contact_tests/`，不覆盖前一轮测试。两个控制器分别执行：

1. 箱底轻触后，参考位置以0.5mm/s再下移0.5mm，停留1s并撤离；这不是恒力命令，也不要求纸板实际压缩。
2. 统一回放 `side_push_fixture.json` 中先前OMPL生成并逐次检查的路径，接触同一侧板边缘后，以0.5mm/s发出10mm水平推动指令并撤离。不向下加载、不调增益、不改变接触姿态。物体实际平移与手的运动分别记录；达到9mm视为10mm目标的1mm容差内。

每个组合仅一次确定性测试，不能由此推断控制器的一般优越性。发生阈值中止时，记录触发步的真实峰值与停止状态，不声称已完成撤离。

## 世界坐标方向性增益修改

`directional_adapter.py` 是可选适配层，原 `adapter.py` 与上游源码均未修改。推动X方向保持900N/m；横向Y增至2700N/m，高度Z增至1800N/m，线性阻尼按刚度比例平方根调整。旋转控制和零空间增益保持不变。

弹性矩阵转换到期望工具坐标系，阻尼矩阵转换到实际工具坐标系，确保修改作用于指定世界方向。该修改不使用物体状态反馈、不更换接触姿态，也不提供恒力控制。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MUJOCO_GL=egl \
  .venv/bin/python experiments/vega_impedance/directional_test.py --variant original
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MUJOCO_GL=egl \
  .venv/bin/python experiments/vega_impedance/directional_test.py --variant directional
.venv/bin/python experiments/vega_impedance/directional_analysis.py
```

两组可在独立进程并行运行；各自输出到 `validation/directional_control/` 的不同目录。先运行两组再分析。测试保持原10mm侧推流程，分别报告手的跟踪误差、物体前进量、侧偏、偏航和真实接触峰值；不能只凭手跟踪更准判定任务改善。该配置不会成为基线默认值。
