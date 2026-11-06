# 独立 pick_up 任务（固定场景）

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  .venv/bin/python experiments/grasp_skill/pick_up.py \
  --out validation/pick_up/new_run
```

输出目录必须不存在，避免覆盖证据。程序先执行一次零偏移试验；该次 `pickup_success` 成立后，在两个独立进程中执行同条件的第二、第三次重复。每次都新建环境，从 `robot_config.json` 正常 home 出发。没有偏移随机化、折盒动作或更改材料。

默认复用 `validation/grasp_skill/run04/trial00/approach_path.npy` 中已验证的 OMPL RRTConnect 路径。每轮重算相同对象几何对应的抓取姿态，验证路径起点/终点，再对密集路径样本做完整机器人碰撞检测。路径由原关节 PD 电机执行，接触阶段复用现有固定 `RightArmImpedance`。这是固定任务重复性验证，不是新规划器测试。路径来源、SHA256和原规划器元数据保存在各轮 `result.json`；可用 `--path` 明确指定相同格式的已验证路径。

物体仍为原300×220×4mm、26.4g单面板，等效密度100kg/m³、面密度400g/m²；支座位置为(0,-.65,1.03)m、顶面z=1.14m。机器人、桌子、夹爪、质量惯性、摩擦/接触参数、重力和执行器限制均不变。板是自由体，不用焊接、mocap、直接加力或执行中qpos改写。这是简化单面板任务，不能代表原桌面整盒任务解决。

任务执行：电机接近 → 缓慢闭合与法向力反馈 → 命令抬升25mm → 保持 → 受控放回、张开、退手。控制参数沿用成功版本：总颚法向力目标3N；闭合力矩分量范围 .02–.30Nm，变化率 .05Nm/s；其后仍叠加原阻尼/重力补偿并按原电机±.6Nm限幅。

- `pickup_success`：实际板COM比抬升前高至少10mm；至少1秒完全脱离支座；对向真实颚组各承载>0.05N；整个抬升及保持相对位移<3mm、角变化<.05rad。此结果在清理前立即写入 `pickup_checkpoint.json`。
- `cleanup_success`：正常受控放置、张开、退手后，最后约1秒（50个20ms状态，首尾跨度.98秒）颚力<.01N、支座承托>0.1N、板COM漂移<1mm。
- `protocol_completed`：上述两者均成立。`command_sequence_completed`仅表示正常动作序列走完；失败另列 `outcome`、`failure_phase`、`error` 和恢复情况。清理失败不会抹掉真实夹起成功。

新增双pad是否同时接触单独报告，**不是**物理成功硬条件；真实原厂指体也是未修改的颚接触面。力峰分别按 `pickup`、`cleanup`、`recovery` 输出，避免把放置力峰误当作抬升力峰。

每0.25ms监测颚法向力之和，超过5N即终止；这是观察后停止阈值，不保证没有瞬态超调。滑移>8mm或转角>.10rad也会终止，异常时尝试张开与受支座承托后的退手。之前+2mm试验曾有9.466N瞬态，这份证据仍在 `validation/grasp_skill`；本任务不重测偏移，不声称一般鲁棒性。

输出包含中文 `REPORT.txt`、总 `result.json`、脚本快照、597项核心文件前后SHA256，以及每轮独立的 `result.json`、夹起检查点、轨迹、每物理步力日志、20ms成对接触日志、视频、阶段PNG、测量图、模型不变量和视频解码核对。依赖现有 `.venv`、已保存OMPL路径和仓库原有阻抗/OMPL模块；第三方思路和许可证边界见同目录 `README.md`。

如果首轮物理结果已完成、仅后处理失败，可用 `--resume-after-first` 恢复：它只接受已有且 `pickup_success=true` 的 `trial00`、尚不存在的 `trial01/02`，并先核对冻结技能源码、场景、路径和核心哈希。随后仅补首轮后处理及两次缺失重复，不重跑首轮。恢复日志与追加源码快照独立保存。本次首轮曾遇到同名分析模块导入冲突；修复为明确文件路径导入，原错误日志保留，物理动作没有重做。
