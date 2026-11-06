# 同夹具的30°→60°→90°单折痕任务

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
 .venv/bin/python experiments/single_fold/run_fold.py \
 --out validation/single_fold_90/new_run \
 --path validation/single_fold/run02/trial00/approach_path.npy \
 --stages 30 60 90
```

输出目录必须不存在。省略`--stages`仍执行原5°检查点→15°保持任务。新任务复用原`box_base + box_side_left`、被动`crease_side_left`和反向tendon、接触夹具、正常home与抓姿，详情见同目录README.md。没有物体驱动器、焊接、冻结或直接加力。

先检查名义完整0→90→0圆弧（最大0.5°采样）与原关节限制、全几何碰撞。初步预检使用scratch中的名义闭合手指q=.016rad，执行环境仍在home张开；预检若拒绝，物理步骤为0，只有初始PNG，没有动态视频。预检通过才执行静置、原OMPL接近与闭合；抓稳后依据实际关节轴/锚点/TCP关系再次检查整个圆弧。预测只改scratch，执行状态SHA在前后验证相同。

各阶段目标30/60/90°，速度2°/s，每阶段保持60×20ms，首末日志跨度1.18s；真实joint角须在目标±2°，同一qpos的独立scratch FK交叉核对误差<1e-5rad。移动板/手相对变化从抓稳参考开始累计，<3mm/.05rad；底板从静置参考开始累计≤2mm/1°；真实两颚组各>.05N；移动板/夹具力<.01N。每.25ms手法向力>5N停止（监测阈值，不是硬力上限），既有滑移8mm/.10rad及底板/碰撞/warning终止保留。握力目标3N、闭合分量.02–.30Nm、变化率.05Nm/s及原GIC不变。夹具内部力另列。

每个完成阶段保存`stage_30_checkpoint.json`等；后续失败不抹掉已完成阶段。全部90°保持通过后才返回0°并释放，返回与释放单独评估。第一轮全部保持通过才再做两次同条件fresh重复；失败停下，不自动修改物理/抓姿或继续折叠。两新增pad仅诊断，真实对向颚组接触是抓持依据。

## 首次预检结果

`validation/single_fold_90/run01`在32°名义目标拒绝：位置残差.429599mm超过.3mm规划容差，姿态残差.00246568rad小于.003rad；无非预期碰撞、关节均在范围内。31.5°是此前最后通过的采样，不能视为实际物理30°成功。物理步骤0、没有力/运动实测；不能由局部IK拒绝宣称物理不可能。

追加100/500/2000次同seed纯scratch迭代后，32°残差趋于.361988mm/.00207812rad，最小关节限位余量.37307rad。该有界诊断仅检查求解器收敛，不改变抓姿/控制/阈值。原结果、源码、初始PNG、MJCF及597项核心哈希保留。

零物理帧时原worker误尝试解码不存在的MP4；已修复零帧后处理，原异常日志保留，已有结果仅补充artifact_checks和汇总，未重跑物理。

根代理授权的额外一次有界只读seed诊断也未通过：正常home及前一通过解j3±.2/j5∓.2rad的两个确定扰动，各500次；位置误差均约.362mm，初始/最终无非预期碰撞。最小加权Jacobian奇异值约1.5–2.9e-5，提示当前解附近接近奇异。没有成功分支，因此没有物理重试。完整结果见`validation/single_fold_90/REPORT.txt`和`result.json`；不宣称全局不可达。

只读复算：
```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python \
 experiments/single_fold/analyze_stages.py --run validation/single_fold_90/run01
```
