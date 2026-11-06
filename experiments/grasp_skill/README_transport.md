# 独立 transport_10mm 检查点

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  .venv/bin/python experiments/grasp_skill/transport.py \
  --out validation/transport/new_run
```

新输出目录必须不存在。先运行一次完整流程；仅首轮 `transport_success` 成立后，再运行两个相同固定条件的新环境重复（独立进程）。不改变纸板初始位置、不随机化、不旋转或折叠。

沿用 `pick_up.py` 已验证的同一面板、支座、正常home、固定OMPL接近路径、抓取姿态和3N反馈控制。新动作仅为：夹起保持完成 → 世界+Y方向10mm、5秒（2mm/s）→ 保持60个20ms步 → 原路返回10mm、5秒 → 保持60步 → 原正常受控放置、张开和退手。各新阶段继续使用原阻抗与闭合力反馈，未更改任何增益、材料或执行器限制。

**首轮前固定的判据**：

- `pickup_success` 延用原真实夹起判据，在搬移前独立保存。
- `transport_success`：最终保持首尾跨度≥1秒，实际面板COM的世界Y位移在9–11mm；全搬移/保持的X及Z漂移各≤2mm；无支撑（<.01N）、对向真实颚组各>.05N；全过程物体/工具相对变化<3mm/.05rad。搬移参考取初始夹起保持结束时实际COM和相对姿态，因此不重复计算前段夹起的约2.6mm挠移。
- `return_success`：实际COM距搬移起点≤2mm，维持抓持与无支撑，返回阶段也满足同一相对变化及X/Z界限。
- `cleanup_success` 单独检查正常放回、张开、退手后的支撑和稳定；`protocol_completed` 要求四项均成功。后续失败不抹掉已完成的前一检查点。

实际执行仍是自由物体与电机控制。新路径先在scratch状态中按当前物体/工具关系预测携物姿态，以1mm间距做IK/碰撞检查；预测中允许已有颚组/纸板接触，拒绝其他机器人或物体碰撞。此预测不写入执行状态，不是物理附件、焊接或吸附；保存前后执行状态哈希。

5N仍是每0.25ms观察后的终止阈值，不是硬力上限；相对滑移>8mm或转角>.10rad也终止。分别保存夹起、去程/保持、返回、清理及恢复力峰。双新增pad接触仅为诊断，不是替代真实颚组承载的成功门槛。若发生异常，保留当步完整接触和qpos/qvel/ctrl快照以便离线检查，不用快照驱动物理执行。

产物包括中文REPORT/result、pickup/transport/return检查点、每物理步forces日志、20ms实际trajectory/contact日志、transport_trace、视频与阶段图、scratch预测检查、源码快照及核心文件前后哈希。原模型和所有旧验证证据保留。这只验证当前简化固定单面板的下一动作检查点，不代表一般搬运或整盒能力。
