# 夹具辅助的单折痕物理测试

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  .venv/bin/python experiments/single_fold/run_fold.py \
  --out validation/single_fold/new_run \
  --path validation/single_fold/run01/trial00/approach_path.npy
```

输出目录须不存在。`--path`省略时首轮用原OMPL RRTConnect重新规划；提供路径时必须重新通过起点、终点及密集碰撞验证。首轮实际15°保持通过后，最多两个相同条件的新环境重复，分别独立进程执行。此测试只有一个原始被动折痕，不包含原完整桌面纸盒、双臂协作或其他折叠目标。

## 物体与真实夹具

直接从原`models/scene.xml`复制`box_base`及`box_side_left`，删除其他面板。底板300×220×4mm、26.4g；侧板原削角轮廓90×217×4mm、7.7864g。两板质量、惯性、网格与接触参数保持原样。底板仍通过`box_free`自由运动，侧板只有原`crease_side_left`被动hinge，没有物体执行器、焊接、冻结、mocap或直接加力。

原折痕轴局部(0,+1,0)、局部锚点(0,0,.002)m，范围[-.12,1.78]rad，阻尼.00434Nms/rad、刚度.000217Nm/rad、干摩擦.0217Nm、armature1e-8；原反向被动tendon（stiffness.032333、springlength0 10）保留。底板初始中心(.09,-.65,1.1422)m，因此世界折痕锚点(-.06,-.65,1.1442)m、侧板自由边x=-.15m。

固定世界夹具仅通过接触承力：底座299×240×220mm，顶面1.14m、x[-.059,.24]；上盖270×236.4×10mm，x[-.03,.24]、下表面1.1443m；两侧8mm厚挡壁及远端8mm挡块。初始底/盖/侧/远端正间隙分别.2/.1/.2/.2mm；底座距折痕1mm，上盖距折痕30mm，所有夹具在底板侧，不跨移动侧板。下底座承重，上盖限制底板上掀，两侧与远端机械挡块限制平移/转动；底板仍可在间隙及接触顺应性内微动，并实际测量其位移。夹具/纸板显式接触采用原桌面/纸板μ=.4和原solref/solimp/condim等参数，手/纸板显式μ=.6不变。

## 动作与预先冻结判据

正常home无碰撞检查 → 1秒夹具静置 → OMPL电机接近 → 移动板自由边缓慢闭合 → 两颚实际承载 → 原hinge圆弧先到绝对5°检查点，再到绝对15°并保持1.2秒 → 同弧返回绝对0° → 张开、退手。**不执行旧单面板25mm自由抬升。**

圆弧以抓稳后的实际折痕轴/锚点和实际TCP为参考，位置和方向同时绕该轴旋转，角速度2°/秒，保持手与移动板关系。先在专用scratch中预测原hinge角和机器人姿态做碰撞/IK检查；这些qpos改写只用于scratch预测，执行始终由原电机及现有固定阻抗控制驱动。握力仍为目标3N、闭合力矩分量.02–.30Nm、变化率.05Nm/s，原阻尼/重力补偿及执行器限制不变。

独立输出`fixture_ready / grasp_success / fold15_success / hold_success / return_success / release_success`：

- 真实折痕角13–17°持续至少1秒，读取原被动joint的实际qpos，并与同一qpos计算的板间FK相对角交叉核对，数值容差1e-5rad。
- 底板相对初始静置参考平移≤2mm、旋转≤1°；全部动作记录底板真实运动。
- 移动板相对夹爪位姿以闭合稳定后的状态为参考，折叠保持要求滑移<3mm、转角<.05rad，两侧真实颚组各>.05N、移动板不接触夹具。双新增pad只是诊断，非成功必要条件。
- 回到真实折痕0±2°且仍有对向颚承载；张开退手后无手接触并保持0±2°。失败与异常恢复单独报告。

每.25ms监测手/移动板总法向力，>5N终止；5N是观察阈值，不是硬力保证。滑移>8mm或转角>.10rad、底板>2mm/1°、非预期机器人接触或MuJoCo warning也终止。夹具/底板内部接触力另列，不与握力相加。保存真实hinge与命令角、每物理步手力/夹具力/电机力矩、20ms实际轨迹/手接触、异常完整接触与状态、视频和阶段PNG。

## 证据与测量修正

所有运行目录保留MJCF/源码快照、原参数审计及597项核心文件前后哈希。`run01`真实达到14.116–14.567°并保持1.18秒，但使用了MuJoCo缓存FK与积分后qpos的不同时间点比较，额外数值门槛误判；与前一个.25ms子步对齐后误差仅1.11e-16rad。其原始失败和未完成的异常恢复记录不覆盖。

`run02`只修正测量为专用`MjData`复制同一当前qpos后`mj_kinematics`，并逐次核对执行状态哈希；保留1e-5容差、所有物理阈值/控制/夹具完全不变。还纠正了继承的旧单面板描述字段。正常返回/释放须重新完整实测，不能把首轮物理保持直接算成完整协议成功。

完整结果见`validation/single_fold/REPORT.txt`及`result.json`：修正后的`run02`固定条件3/3完成折叠、保持、返回与释放。每轮保留原始数据和视频。

只读复算（不推进物理）：
```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  .venv/bin/python experiments/single_fold/analyze_results.py \
  --run validation/single_fold/run02
```

`run02`执行快照中`return_metrics`的滑移字段误用了去程筛选；补充`analysis.json`从`fold_return/return_hold`原始日志独立计算真实返回值。公开`held_metrics`已修正，并直接用保存数据验证，未为统计重跑。原始执行源码与结果保留，不覆盖。
