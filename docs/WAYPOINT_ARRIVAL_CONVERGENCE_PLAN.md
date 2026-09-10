# 航点到达与到达后位姿处理计划

## 目标与顺序

将“航点到达”定义为：完成当前航点要求的 NDT/RTK 静止校正后，XY
连续稳定满足航点位置容差。最终 yaw 和由旋转引起的 XY 偏移属于到达后
处理，不得延迟或撤销航点的已到达状态。

1. Nav2 到点并确认停车稳定。
2. 完成 NDT/RTK 静止校正。
3. 使用校正后的定位连续验收 XY；不合格时仍由 Nav2 重新接近。
4. XY 合格后立即持久化并上报 `waypoint_reached`，同时启动播报、航点动作和 dwell。
5. 执行 `require_yaw` 最终航向；旋转后 XY 超差时保持目标 yaw，以机体坐标系 `vx/vy` 直接微调。
6. 到达后位姿处理及阻塞型播报/dwell 全部完成后，才发送下一航点或完成任务。

## 验收阈值

| 航点策略 | XY | yaw | 到达后处理 |
| --- | ---: | ---: | --- |
| `stop_and_confirm` | 0.45 m | 0.25 rad（仅 `require_yaw`） | yaw 后 XY 超差时保持航向微调 |
| `precision` | 0.15 m | 0.25 rad | yaw 后保持航向微调 |
| `dock` | 0.08 m | 0.087 rad | 使用专用 docking 流程 |

XY 到达要求连续 3 个新鲜定位样本合格。室外航点还必须满足原有
FAST-LIO 与 fixed RTK 联合门槛，RTK 到地图点击点不超过 1.5 m。

## 微调安全边界

- 通过 `/cmd_vel_raw` 发送速度，使命令继续经过 collision monitor 后到 `/cmd_vel`。
- 10 Hz 控制，平移速度不超过 0.08 m/s，航向角速度不超过 0.10 rad/s。
- 不设置初始距离上限和总时限；每轮使用 0.50 m 滚动方向净空检查。
- 扫描过期、定位不新鲜或校正仍在 smoothing 时先发零速，允许 2 秒恢复。
- 确认有障碍或暂态异常持续超过 2 秒时立即零速并安全暂停。
- 暂停、恢复或 Edge 重启后继续到达后处理，不重新导航已到达航点。

## 状态与事件

- `waypoint_reached` 表示校正后 XY 已稳定验收，并立即增加 `completed_waypoints`。
- 使用持久化阶段 `xy_reached`、`heading_pending`、`heading_aligned`、
  `xy_adjusting`、`post_arrival_ready` 区分航点已到达与后处理完成。
- `task.recovery_active` 继续报告校正、位置确认、最终航向、联合验收和微调阶段。
- yaw/XY 后处理失败只暂停任务，不回退航点状态；恢复时不得重复播报或执行航点动作。

## 测试与部署

- 覆盖单帧跳变、连续 XY 验收、NDT smoothing、室外 RTK 门槛。
- 覆盖到达事件早于 yaw、播报/动作/dwell 并行、阻塞门槛等待。
- 覆盖大于 0.50 m 的 yaw 后偏移、超过原 8 秒仍继续微调、障碍及 2 秒暂态恢复。
- 覆盖暂停恢复和本地上下文迁移，确认不重复到达、不重新导航。
- `pass_through` 和 docking 保持原有专用语义。
- 自动化验证通过后，只在机器人无活动任务时重启 Edge Agent；真实运动测试需先确认场地安全。
