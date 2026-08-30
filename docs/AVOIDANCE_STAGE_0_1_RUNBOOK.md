# MPPI 避障阶段 0/1 录制、回放与验收手册

## 1. 安全边界

`avoidance_baseline.sh` 只启停诊断录制，不发布导航目标或速度。实际运动必须由操作员确认路径、软障碍、急停和人员配合后，通过正式云平台任务链下发。不得用本手册中的录制命令替代运动授权。

Shadow 回放固定使用 ROS Domain 77，所有录制速度话题重映射到 `/shadow/recorded/*`。脚本发现 Shadow Domain 中存在 SDK 速度 Bridge 时会拒绝运行。

## 2. 标准场景

阶段 0/1 使用以下固定名称：

| 场景 | 目的 | 阶段 1 必须通过 |
|---|---|---:|
| `straight` | 无障碍直线基线、摆动和耗时 | 是 |
| `turn` | 正常转弯、角速度反转 | 是 |
| `narrow_passage` | 可通行窄通道与误停车 | 是 |
| `static_box` | 静态软障碍局部绕行 | 是 |
| `wall_corner` | 墙角膨胀区与贴边行为 | 是 |
| `person_crossing` | 阶段 2/3 前的动态目标原始基线 | 否，不用于宣称动态预测已完成 |

## 3. 现场录制

确认环境安全后，先启动录制：

```bash
cd /home/dogrobot
robot/script/robot/avoidance_baseline.sh start static_box trial01
```

随后由操作员通过正式云平台下发任务。任务结束后记录人工结论：

```bash
robot/script/robot/avoidance_baseline.sh stop pass "软纸箱；无接触；目标到达"
```

如果未完成现场复核，必须使用 `unreviewed`；发生接触、越界、目标失败或人工接管则使用 `fail` 或 `aborted`。

每个包包含：

- `avoidance_scenario.json`：场景、人工结论与备注；
- `avoidance_summary.json`：证据完整性、速度链、Collision Monitor 干预、前向间距、摆动和 MPPI 性能；
- `system_metrics/resource_summary.json`：CPU、GPU、RAM 和进程资源分位数；
- MCAP：原始可回放证据。

普通任务结束时，大包指标会以低优先级后台生成，避免阻塞任务终态上报；`avoidance_baseline.sh stop` 则等待本次场景汇总完成后再返回。

## 4. 无本体运动 Shadow 回放

只回放并生成指标：

```bash
robot/script/robot/avoidance_shadow_replay.sh BAG_DIR --rate 1.0
```

给一个不含 SDK Bridge 的候选 Shadow 栈输入相同数据：

```bash
robot/script/robot/avoidance_shadow_replay.sh BAG_DIR --rate 1.0 -- \
  ros2 launch candidate_package avoidance_shadow.launch.py use_sim_time:=true
```

可用 `--start-offset` 和 `--duration` 截取问题片段。`--duration` 由外层安全定时器实现，兼容 ROS 2 Humble。

## 5. 阶段 1 判定

单个场景只有同时满足以下条件，`stage_1_acceptance.passed` 才为 true：

1. 所有必需证据话题存在；
2. 存在真实候选运动命令；
3. MPPI 最差窗口 P99 小于 40 ms；
4. 场景属于阶段 1 静态场景；
5. 操作员确认无接触/碰撞且目标成功，记录 outcome=`pass`。

最终阶段 1 验收要求 `straight`、`turn`、`narrow_passage`、`static_box`、`wall_corner` 各至少一个通过包。任一场景失败时回退到上一稳定参数，不同时修改感知过滤、MPPI 权重和本体控制参数。
