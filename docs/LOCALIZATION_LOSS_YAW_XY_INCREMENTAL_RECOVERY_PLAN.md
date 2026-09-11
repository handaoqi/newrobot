# 定位丢失后的航向恢复与分段 XY 逼近计划

## 1. 目标与边界

定位连续丢失后，机器人必须先取消当前导航并确认停车。恢复阶段不沿用失效的全局路径，也不将当前航点直接标记完成；在重新获得可信绝对航向和位置后，从当前未完成航点重新规划。

本方案增加一个 Edge 内部的 `localization_recovery_approach` 状态机：

```text
LOCALIZATION_LOST
  → SAFE_STOPPED
  → YAW_OBSERVABILITY
  → ABSOLUTE_POSE_CONFIRMED
  → TARGET_HEADING_ALIGN
  → XY_INCREMENTAL_APPROACH
  → CURRENT_WAYPOINT_REPLAN
  → NORMAL_PATROL
```

它不替代现有的 NDT/FAST-LIO/RTK 重定位，也不替代 Nav2 的路径规划。它只在“已停车、已得到可信但仍需小范围接近航点的绝对 pose”条件下提供受限动作。

## 2. 核心原则

1. 原始 IMU/FAST-LIO yaw 只能作为转动控制的相对量，不可单独视为全局航向恢复成功。
2. 小幅转动的目的，是获得不同观测角度以改善 NDT/激光匹配或 RTK 航向的可观测性；必须由外部绝对观测确认结果。
3. 绝对航向未确认前禁止任何平移。不得根据旧 raw pose、旧全局路径或纯里程计向前行驶。
4. 每次平移都通过 `/cmd_vel_raw → Collision Monitor → /cmd_vel`，不得绕过慢行、停止、定位健康和雷达新鲜度门控。
5. 每一小段动作后必须停车并重新验收；只要任一安全条件不成立，立即清零速度并回到定位恢复/安全保持。

## 3. 状态机设计

### 3.1 SAFE_STOPPED

- 复用现有 `on_localization_lost()`：取消 FollowWaypoints、停止运动、持久化 `current_waypoint_index` 与到点阶段。
- 必须确认 `is_robot_stopped()`；未确认停车时不进入恢复动作。
- 申请 `EDGE_LOCALIZATION` recovery lease。BT 的恢复转向、Edge 障碍后退/绕行不得同时持有动作权。
- 急停、人工接管、低电量、越界、建图进行中时禁止进入后续阶段。

### 3.2 YAW_OBSERVABILITY

目的不是“用 raw yaw 校正 raw yaw”，而是以有限转动获得可用于绝对定位的观测。

- 每次原地转动 `10°–15°`，单次后停车并等待新鲜激光/定位；
- 总转角上限 `45°`，最多 3 次；
- 使用 Collision Monitor 门控的旋转命令；若旋转被安全层阻断，保持停车；
- 每次转动后触发/等待现有 RTK、NDT、FAST-LIO 绝对定位判定；
- 仅当连续 3 帧满足以下条件时进入下一阶段：
  - 定位样本新鲜；
  - `absolute_stable=true`；
  - `correction_smoothing_active=false`；
  - `lio_motion_anomaly=false`；
  - 航向变化和位置结果在策略容差内一致；
  - 室外 RTK 地图还需满足 RTK 固定/可导航条件。

总转角耗尽仍未获得绝对 pose 时，返回定位恢复线程或 `SAFE_HOLD`，不允许平移。

### 3.3 TARGET_HEADING_ALIGN

- 用确认后的 map pose 和当前未完成航点计算目标方位；
- 原地转向至目标方向，目标误差建议不大于 `10°`，并连续 3 帧确认；
- 航向不稳定、定位过期或雷达过期时立即停止；
- 此阶段只解决面向方向，不将航向成功误判为航点到达。

### 3.4 XY_INCREMENTAL_APPROACH

- 每段前进 `0.05–0.10 m`，禁止横向盲移；
- 单次恢复总平移建议上限 `0.50 m`，最多 5 段，建议总时限 45 秒；
- 前进前和执行中均检查：前向净空、雷达时效、Collision Monitor 输出、定位新鲜度、平滑校正和 LIO 异常；
- 每段结束后发布零速度，重新执行绝对定位验收；
- 若当前位置已在当前航点的 XY/航向策略容差内，直接进入 `CURRENT_WAYPOINT_REPLAN` 的到点复验分支；
- 预算、超时、障碍、定位异常或重定位失败时进入 `SAFE_HOLD`，保留当前航点和恢复原因。

### 3.5 CURRENT_WAYPOINT_REPLAN

- 不跳过 `current_waypoint_index`；用确认后的当前 pose 对该航点重新发起单航点导航；
- FollowWaypoints → WaypointFollower → NavigateToPose BT 重新生成路径；旧路径不复用；
- 若当前航点实际已到达，执行现有到点 XY/航向、播报、动作、dwell 的幂等确认；
- 只有当前航点完成后，才由正常流程派发下一个航点。

## 4. 与现有模块的职责分界

| 模块 | 本方案中的职责 | 不承担的职责 |
|---|---|---|
| Collision Monitor | 每次转动/前进的最终速度安全裁决 | 绝对定位判断 |
| 定位恢复（RTK/NDT/FAST-LIO） | 生成与确认绝对 pose | 直接决定路线业务完成 |
| Edge TaskExecutor | 状态机、预算、停车、持久化、当前航点重发 | 绕过 Nav2 长距离导航 |
| BT Navigator | 当前航点重新下发后的规划、局部控制与 BT 恢复 | 定位丢失期间继续运行旧目标 |
| Behavior Server | 受 BT 调用的 Spin/Wait 等行为 | 对定位失效时自行平移 |
| 中心 PatrolLoopService | 5 秒安全观察、恢复 episode、任务级重试与人工继续 | 直接发速度或替代 Edge 定位决策 |

## 5. 预算与并发控制

- 使用现有 RecoveryArbiter；`EDGE_LOCALIZATION` 在本状态机结束前保持唯一动作所有权。
- BT 的 `RecoveryLeaseScope`、障碍后退和侧移绕行拿不到租约时必须放弃动作，不能并发运动。
- 中心 `task.recover.v1` 与本地恢复并发时：本地仍在恢复则返回 `in_progress`；本地已恢复运行则返回 `already_recovered`。
- 本状态机的转动、平移预算应计入 Edge 恢复快照，并与中心每 episode 最多 10 次恢复命令分别审计。

## 6. 实施步骤

1. 为 TaskExecutor 增加持久化的恢复子阶段、转动/平移预算和最后一次绝对 pose 验收结果。
2. 为 EdgeAgent 的本地定位恢复线程增加 `YAW_OBSERVABILITY` 回调，先完成无平移观测再允许接近动作。
3. 抽取现有出发前转向和到点微调的安全门控，形成可复用的“停车—动作—新鲜样本确认”原语；不得复制并弱化门控。
4. 增加 `XY_INCREMENTAL_APPROACH`，使用受 Collision Monitor 保护的速度通道，并在每段后执行绝对定位复验。
5. 成功后仅重发当前航点；到点动作和 dwell 必须沿用既有幂等状态。
6. 扩展遥测与事件：阶段、每段距离、总预算、定位质量、阻断原因、恢复 lease 和是否转入 BT 重规划。
7. 更新 GuardDuty 文案：展示“航向观测恢复”“分段接近”“重新规划当前航点”；安全保持时继续按钮沿用现有 recovery episode 机制。

## 7. 测试与上线门禁

### 自动化测试

- raw yaw 可用但绝对定位未稳定：只能转动，绝不发送平移；
- 小幅转动后 NDT/RTK 恢复：连续 3 帧确认后才允许前进；
- 每段前进后 pose 稳定：正确重发当前航点；
- 当前点已经到达：不重复播报、动作、dwell；
- 扫描过期、Collision Monitor 限速/停止、平滑校正、LIO 异常、人工接管、急停、低电量：立即停止并不前进；
- 转角/距离/时间预算耗尽：保持停车，中心收到可解释的恢复状态；
- BT Spin 与 Edge 定位恢复同时请求：恢复租约只允许一个动作所有者；
- 重启 Edge 后：恢复子阶段、预算、当前航点和幂等到点状态可恢复。

### 真机灰度顺序

1. 无任务静态验证：只观察状态与事件，不发动作；
2. 空旷区域：仅验证小角度转动和绝对航向恢复；
3. 空旷短距离：验证单段前进、停车和复验；
4. 单航点巡逻：验证重新规划到当前点；
5. 含播报/动作/dwell 的多航点巡逻：验证不重复副作用；
6. 室外 RTK 与室内 NDT 分别验收后再全量启用。

上线前必须确认：Collision Monitor、定位健康门控、恢复租约、中心人工继续、遥测审计和回滚开关均可用。任何未确认的绝对定位不得进入平移阶段。
