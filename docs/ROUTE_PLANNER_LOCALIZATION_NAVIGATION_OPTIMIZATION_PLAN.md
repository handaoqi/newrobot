# 路径规划、定位与导航直线防摆实施计划

## 1. 界面与调试包

- “仿真和回放检查”“回放调试台”归入“巡检任务”子菜单，保留原 URL。
- 路线选择下方提示缩小字号、禁止换行；“上次执行时间”内容和格式保持不变。
- 保存路线按钮上方增加“录制导航调试包”，默认关闭并保存到路线 `record_rosbag`。
- 路线执行未显式传参时继承路线配置，单次执行参数可覆盖；巡检任务模板原有开关保持独立。

## 2. 室外/过渡场景定位

下发地图、初始化定位、主动重定位统一优先 RTK fixed。连续 3 个新鲜样本满足 fixed、双天线航向有效且 RTK/FAST-LIO 偏差小于 0.30 m 后立即完成；限定时间内失败则有界回退到建图原点、周边、手选点/路线航点和全局匹配，不无限等待。室内和 `local_only` 保持 NDT 渐进定位。

## 3. MPPI 参数与差异

当前为 `time_steps=56`、`model_dt=0.05`、`batch_size=1000`、`motion_model="DiffDrive"`。调整为：

```yaml
motion_model: "DiffDrive"
time_steps: 40
model_dt: 0.05
batch_size: 1200
```

预测时域由 2.8 s 缩短为 2.0 s，采样轨迹数量增加 20%，总时域样本由 56000 降至 48000，预计降低内存和总体计算量，同时提高单周期采样多样性。`DiffDrive` 保持原大小写和值不变。

直线 Profile：`wz_std=0.04`、`gamma=0.03`、`wz_max=0.28`、PathFollow 权重 14、PathAngle 权重 3、最大角度 0.30，直线无遮挡时关闭 PathAlignCritic。

## 4. RPP 参数与实现

普通直线巡航使用：

```yaml
desired_linear_vel: 0.22
lookahead_dist: 1.2
min_lookahead_dist: 0.6
max_lookahead_dist: 1.8
use_velocity_scaled_lookahead_dist: true
lookahead_time: 2.5
use_regulated_linear_velocity_scaling: true
max_angular_vel: 0.30
rotate_to_heading_threshold: 0.52
rotate_to_heading_angular_vel: 0.22
angular_deadband: 0.03
```

自研 RPP 增加上述参数声明和动态更新。前视距离按 `clamp(max(lookahead_dist, abs(vx)*lookahead_time), min, max)` 计算；曲率调速映射现有调速逻辑。终点接近和精确航向阶段继续使用短前视低速参数。

## 5. 共享速度平滑与验收

- 角速度死区 `0.03 rad/s`。
- 角速度最大加速度/减速度调整为 `0.8/-0.8 rad/s²`。
- 测试长直线、轻微折线、转弯、直线遇障碍和终点航向，记录 `wz` 正负切换、峰值、横向误差、到点率、避障干预和 MPPI 周期性能。
- 验收目标：直线段 `wz` 正负切换下降至少 50%，横向误差不超过 0.15 m，到点率、避障和安全停车能力不下降。

## 6. 航段速度档位：默认微速，按需提高到下一个航点

### 6.1 目标与边界

路线的每个非末尾航点增加“到下个点速度”选择。该字段描述**从当前点出发至下一点**的航段，而不是到达当前点后的定位校正、转向和 XY 微调速度。

- 缺省值为 `micro`（微速），历史路线缺失字段也按 `micro` 解释；新建、复制、预演、自动任务和保安值守循环必须一致。
- 微速是默认航段档位，使用远控微速的 `2/7` 导航比例：`vx=0.30 m/s`、`vy=0.225 m/s`、`wz=0.525 rad/s`。微速全程固定为 `0.30 m/s` 上限；终点接近、到点转向、XY 微调和恢复动作继续使用各自原有低速 profile，不继承航段档位。
- 只有用户明确选择低速、中速或高速时，才在该航段提高局部控制器的巡航上限；不修改全局默认速度，不影响其他航段。
- 所有档位均受统一配置的导航 SDK 三轴安全上限、导航边界限速、Collision Monitor 减速/硬急停、局部代价地图和终点接近 profile 的共同约束。当前 SDK 上限为 `0.50`，本计划会同步提高其可配置上限以容纳明确选择的低/中/高速；档位不是绕过安全链路的权限。

远程控制是低/中/高速参数的唯一来源：`speed_slow`、`speed_normal`、`speed_fast` 分别触发厂商慢/正常/快档；页面输入倍率为 `2.5:3.5:5.0`。自动航段明确选择低/中/高速时，采用该页面对应的三轴参数，并同步抬高 SDK 与边界配置上限。

### 6.2 统一档位参数：前进、横移、转向

档位模型统一为 `vx`（前后）、`vy`（横移）和 `wz`（转向）。远程控制页现有的档位行为继续保留：每次切换分别下发 `speed_slow`、`speed_normal` 或 `speed_fast`，并使用前进基准 `0.60`、横移基准 `0.45`、转向基准 `1.05` 乘以档位倍率；低/中/高的倍率分别是 `2.5/3.5/5.0`。底层虚拟遥控再按 `remote_full_scale_vx/vy/yaw_rate` 归一化，并发送厂商慢/正常/快档命令。

新的共享速度档位配置必须显式包含三轴，供远控页面读取/展示，也供 Edge 做安全校验：

```yaml
teleop_speed_profiles:
  micro:  { scale: 1.75, vendor_action: speed_micro }
  low:    { scale: 2.50, vendor_action: speed_slow }
  medium: { scale: 3.50, vendor_action: speed_normal }
  high:   { scale: 5.00, vendor_action: speed_fast }
```

远控与自动导航使用同一份低/中/高三轴数值配置：远控仍走厂商虚拟摇杆/步态档，自动导航走 Nav2 SDK Bridge；后者当前在三轴均有 `0.50` 的最终夹限。本计划把 SDK 与导航边界的上限同步提高到远控高速档，确保路线明确选择的低/中/高速使用远控监控页面当前参数而不会被截断。

### 6.3 自动导航的新配置与推荐默认值

在 Edge 的 `safety` 配置中增加与远控监控页相同的绝对三轴参数：

```yaml
# 微速：远控微速 1.05/0.7875/1.8375 的 2/7；也是路线字段缺失时的默认值。
navigation_speed_micro_mps: 0.300000
navigation_turn_micro_rps: 0.525000
navigation_lateral_micro_mps: 0.225000

# 前进/后退（vx）：远控页 0.60 × 2.50/3.50/5.00。
navigation_speed_low_mps: 1.500000
navigation_speed_medium_mps: 2.100000
navigation_speed_high_mps: 3.000000

# 转向（wz）：远控页 1.05 × 2.50/3.50/5.00。
navigation_turn_low_rps: 2.625000
navigation_turn_medium_rps: 3.675000
navigation_turn_high_rps: 5.250000

# 横移（vy）：远控页 0.45 × 2.50/3.50/5.00。
navigation_lateral_low_mps: 1.125000
navigation_lateral_medium_mps: 1.575000
navigation_lateral_high_mps: 2.250000

# Nav2 SDK Bridge 的三轴最终上限。必须不小于相应最高航段档位。
sdk_max_vx: 3.000000
sdk_max_vy: 2.250000
sdk_max_yaw_rate: 5.250000

# 仅放宽“限速区可配置的最大值”；既有区域的实际限速值不自动提高。
navigation_boundary_speed_limit_max_mps: 3.000000
```

低/中/高速数值直接来自远控监控页：`vx/vy/wz=1.50/1.125/2.625`、`2.10/1.575/3.675`、`3.00/2.25/5.25`（单位依次为 m/s、m/s、rad/s）。航点微速保留已确认的 `0.30/0.225/0.525`，即远控微速的 `2/7`。SDK 当前 `0.50` 三轴硬夹限必须同步提高到高速值，否则低/中/高速航段会被静默截断。导航边界的当前校验上限和 Edge `set_boundary_speed_limit()` 的 `0.30` 夹限也必须提高至 `3.00 m/s`，否则限速区无法表达与航段相同的速度范围；**已有任何限速区的已保存值保持不变**。

实际生效速度取所有限速的最小值：

`min(航段档位值, 边界限速, Collision Monitor 动态限速, SDK 三轴上限)`。

当前 MPPI `motion_model="DiffDrive"` 必须保持不变，因此自动航段不会输出横移 `vy`：`navigation_lateral_*` 只作为统一配置、能力读回和未来 Omni/显式横移动作的预留，不能让页面暗示“选择高速即可横移导航”。当前自动导航实际应用的是 `vx` 和 `wz`；远程控制的横移继续按其厂商遥控链执行。若未来要让自动导航横移，必须独立评审并切换为 Omni 模型、重新验证代价函数/足端步态/避障和停车策略，不能混入本计划。

配置加载时必须拒绝或归一化非法值（非数、非正、逆序、档位高于 SDK 上限），并在状态/诊断中同时上报“请求档位、档位上限、SDK 上限、边界上限、最终控制器上限、当前安全限速原因”。

### 6.4 执行语义

1. 平台在路线 `waypoints[i].navigation_speed_level` 保存 `micro|low|medium|high`；末点不展示、不保存该字段。路线快照在任务启动时冻结，当前巡检循环中修改路线不改变已启动任务。
2. Edge 在派发 `i → i+1` 前读取 `waypoints[i]` 的档位；缺失字段按 `micro`，使用 `vx/vy/wz=0.30/0.225/0.525`。低/中/高分别使用远控页的 `1.50/1.125/2.625`、`2.10/1.575/3.675`、`3.00/2.25/5.25`。
3. MPPI 下发 `FollowPath.vx_max` 与档位 `FollowPath.wz_max`；RPP 下发 `RPP.desired_linear_vel` 与 `RPP.max_angular_vel`；iLQR 下发 `ILQR.desired_linear_vel` 与 `ILQR.max_angular_vel`。`vy` 在 DiffDrive 自动导航中始终为零。MPPI 的 `vx_min` 不因提高直线速度而放宽，防止提速后倒车或 `wz` 摆动变大。
4. 微速航段全程固定为 `0.30 m/s` 上限（仍取边界、Collision Monitor 和 SDK 中更小者），不按距离抬升或额外降速；只有低/中/高速采用速度包络，不再按“路径过半”这一固定位置减速。每次反馈按 `min(档位 vx 上限, 边界限速, Collision Monitor 限速, sqrt(v_final² + 2 × a_decel × 剩余距离))` 得出期望线速度，并以 `a_accel/a_decel` 平滑升降；长直线才会逐步达到档位上限，短航段会自动保持低速。MPPI/RPP/iLQR 的曲率调速和 `wz` 上限继续约束转弯，终点 profile 始终切回既有精靠近速度。
5. 短航段（默认阈值 `1.0 m`）、终点接近、精确到点、到点后的 NDT/RTK 校正、航向调整、XY 微调、重定位/自愈，均不启用提速；仍使用既有安全低速。
6. 新的非零目标、边界进入、Collision Monitor 介入、暂停/取消/恢复时立即撤销该航段的临时速度参数；下一个航段重新从其冻结档位计算，不能让高速参数泄漏到后续航段。

### 6.5 实施项与验收

1. 平台 serializer/protocol 校验枚举，任务快照保留该字段；前端新增点、旧路线归一化、复制路线和下拉框默认均改为 `micro`。
2. Edge 增加 `vx/vy/wz` 三轴配置和严格校验；远控页面通过能力/配置接口读取同一份档位名称、倍率与厂商动作，不再固化另一份不一致的档位表。把 `vel_cmd_udp_publisher` 的 `sdk_max_vx/vy/yaw_rate` 从 `0.50` 提高到对应远控高速的 `3.00/2.25/5.25`；任何配置不一致则启动失败而非静默夹断。
3. 在 `TaskExecutor` 的单段派发/进度更新中管理临时速度，调用 `RosAdapter` 统一写 MPPI/RPP/iLQR 的 `vx/wz` 参数；Edge 明确记录 DiffDrive 未应用 `vy`。边界限速接口始终取更小值。
4. 增加平台序列化、任务快照、远控档位读回、Edge 分段切换、`vx/wz` 参数写入、DiffDrive 横移抑制、边界覆盖、暂停恢复、短航段和三种控制器的测试。特别验证：未选择档位的历史路线按微速 `0.30/0.225/0.525` 生效。
5. 实机分别录制微速、低、中、高速的直线、短航段和转弯 MCAP；验收实际 `/cmd_vel`、SDK 输出、速度包络、远控横移/转向读回、Collision Monitor 干预、`wz` 正负切换、到点率和停车确认。先在无障碍直线验证 SDK 实际输出不再在 0.50 截断，再验证短航段和边界区仍以制动/较小限速优先。任一档位使 P99 控制周期、摆动、避障或到点率恶化时，回退该档位配置和 SDK/边界上限。
