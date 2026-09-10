# 航点到达位姿收敛修复计划

## 目标

修复 `require_yaw=true` 航点完成最终航向后，因为旋转漂移再次进入通用“朝向航点并前进”流程而额外转半圈的问题。适用于使用 NDT 或 RTK 绝对校正的室内、室外巡检；`pass_through` 和纯 LIO 行驶流程保持不变。

## 处理顺序

1. Nav2 到达并确认停车稳定。
2. 执行当前模式要求的 NDT/RTK 静止校正，等待新鲜且稳定的绝对定位结果。
3. 使用校正后的 XY 检查航点位置；首次超差仍由 Nav2 重新接近，同时清空本航点的校正、航向阶段状态。
4. XY 合格后执行 `require_yaw` 最终航向。
5. 同时验收 XY 和 yaw，不再只验收终点 XY。
6. 普通或 precision 航点在旋转后轻微 XY 漂移时，保持目标航向，以机体坐标系 `vx/vy` 做一次有界微调；禁止调用通用 `_send_from()`。
7. 无法安全、稳定地同时收敛时暂停，错误码 `ARRIVAL_POSE_CONVERGENCE_FAILED`。

## 验收阈值

| 航点策略 | XY | yaw | 旋转后处理 |
| --- | ---: | ---: | --- |
| `stop_and_confirm` | 0.45 m | 0.25 rad（仅 `require_yaw`） | 容差内直接完成，超差且不超过 0.50 m 时保持航向微调 |
| `precision` | 0.15 m | 0.25 rad | 保持航向微调 |
| `dock` | 0.08 m | 0.087 rad | 继续使用专用 docking 流程，不使用通用微调 |

室外航点还必须继续满足原有 FAST-LIO 与 fixed RTK 对地图点击点的联合门槛。

## 微调安全边界

- 通过 `/cmd_vel_raw` 发送速度，使命令继续经过 collision monitor 后到 `/cmd_vel`。
- 10 Hz 控制，平移速度不超过 0.08 m/s，航向角速度不超过 0.10 rad/s，最长 8 秒。
- 单次允许修正的初始 XY 偏差不超过 0.50 m。
- 使用 `/laser_scan` 全 360° 点云按实际 `vx/vy` 方向检查扫掠走廊；扫描年龄不得超过 0.50 秒。
- 障碍物、扫描过期、定位不新鲜/异常、任务状态改变或超时，立即发零速并安全暂停。
- 最多两轮“最终航向 + 联合验收”；不能同时满足 XY 和 yaw 时不得带错误朝向完成。

## 可观测性

- 使用 `task.recovery_active` 报告 `correction`、`position_approach`、`heading_alignment`、`pose_verification`、`heading_preserving_adjustment` 阶段。
- 平台将 `ARRIVAL_POSE_CONVERGENCE_FAILED` 显示为明确的中文暂停原因。

## 测试

- 复现点 3：目标 180°，旋转漂移后不得再次朝航点转约 180°。
- 复现反向点 1：目标 0°，不得以错误航向完成。
- 验证校正、XY、航向、联合验收的严格顺序。
- 覆盖普通、precision、dock、pass-through、室内、室外 RTK 门槛。
- 覆盖过期扫描、方向障碍、定位失效、超时和两轮重试耗尽。
- 运行 Edge 定向测试与完整测试；平台提示映射运行前端测试。
