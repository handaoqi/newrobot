# RoamerX 四层 MPPI 动态避障整体架构

> 文档日期：2026-08-28
> 适用平台：RoamerX / ZSL-1A-07 / ROS 2 Humble / Nav2
> 状态：架构设计，待分阶段实施

## 1. 文档目的

本文设计 RoamerX 机器狗的四层 MPPI 动态避障架构，使自主导航同时具备：

- 远距离通道阻塞判断和全局改道；
- 中距离动态目标预测和局部绕行；
- 近距离独立减速、停车和受控脱困；
- 定位、传感器、通信或本体异常时的硬安全保护。

本文中的“MPPI”指当前项目使用的 Model Predictive Path Integral 局部控制器。如果业务所称“MPPL”代表另一种算法，其输入输出仍应接入本文定义的局部预测控制层和统一安全接口。

## 2. 当前系统现状

当前自主导航速度链为：

```text
Nav2 Controller Server / MPPI
  -> /cmd_vel_nav
  -> Velocity Smoother
  -> /cmd_vel_raw
  -> Collision Monitor
  -> /cmd_vel
  -> mode_status_publisher
  -> vel_cmd_udp_publisher
  -> 机器狗 SDK
```

当前已具备：

- MPPI 控制器，控制频率 20 Hz；
- 2.8 秒预测区间：`56 × 0.05 s`；
- 1000 条候选轨迹；
- 激光雷达自身点云滤除和 `/laser_scan`；
- Collision Monitor 前向减速区和停车区；
- 定位状态门控、传感器健康监控和 SDK 速度 watchdog；
- Nav2、遥控、本体姿态和安全状态的基本控制链。

当前主要缺口：

1. `local_costmap.obstacle_layer.enabled=false`，局部代价地图没有实时障碍。
2. MPPI 的 `CostCritic`、`PathFollowCritic`、`PathAlignCritic` 等避障相关评价器被关闭。
3. 当前 MPPI 主要跟踪路径，动态障碍只能触发 Collision Monitor 减速或停车，不能主动绕行。
4. 障碍物没有跨帧跟踪、速度估计和未来位置预测。
5. Collision Monitor 主要覆盖机器狗前方，后退、侧移和旋转缺乏完整方向安全区。
6. 遥控速度 `/teleop_cmd_vel` 直接进入 SDK Bridge，没有统一经过后置安全仲裁。
7. 当前运动模型为 `DiffDrive`，配置的 `vy_max` 不会产生真正的全向侧移避障轨迹。

## 3. 总体架构

### 3.1 数据与控制流

```text
激光雷达 / 深度相机 / 里程计 / 定位 / 本体状态
                        │
                        ▼
       自身滤除 + 时间同步 + 障碍聚类与跟踪
                        │
             ┌──────────┴──────────┐
             ▼                     ▼
       局部时空风险场          全局通道阻塞事件
             │                     │
             ▼                     ▼
      第2层 MPPI预测绕行      第1层 全局改道决策
             │
             ▼
        速度平滑与命令仲裁
             │
             ▼
      第3层 Collision Monitor
             │
             ▼
        第4层 本体硬安全控制
             │
             ▼
            SDK
```

### 3.2 统一速度接口

目标速度链调整为：

```text
MPPI          -> /cmd_vel_nav
远程控制      -> /cmd_vel_teleop
辅助行为      -> /cmd_vel_behavior
                         │
                         ▼
                 command_mux
                         │
                         ▼
              /cmd_vel_candidate
                         │
                         ▼
       Collision Monitor + 健康状态门控
                         │
                         ▼
                  /cmd_vel_safe
                         │
                         ▼
               mode_status -> SDK Bridge
```

控制原则：

- 自主导航命令经过全部四层。
- 远程控制可以绕过全局规划和 MPPI，但不能绕过近场与硬安全层。
- 急停命令具有最高优先级，不受 command mux 当前控制权影响。
- 切换控制源时清空旧命令，禁止恢复后重放历史非零速度。

## 4. 公共感知底座

公共感知层不计入四层避障防线，但为所有层提供统一障碍证据。

### 4.1 输入

- `/front_lidar`：原始点云；
- `/laser_scan`：经过机身自身滤除的二维激光；
- 深度点云或双目深度，可选；
- `/odom/nav2`：连续局部里程计；
- `/localization_info`：全局定位状态；
- `/tf`、`/tf_static`：坐标转换；
- `/sensor_health`：传感器频率和时间戳健康状态。

### 4.2 处理链

```text
点云时间同步
  -> 机身自身点剔除
  -> 地面及低矮噪声过滤
  -> 局部点云聚类
  -> 跨帧数据关联
  -> 常速度 EKF
  -> 静态/动态分类
  -> 未来轨迹预测
```

### 4.3 动态障碍接口

新增消息建议：

```text
/dynamic_obstacles/tracks
  header
  track_id
  classification
  position_x/y
  velocity_x/y
  acceleration_x/y
  radius
  covariance
  confidence
  first_seen
  last_seen
```

同时发布：

- `/dynamic_obstacles/predictions`：按 MPPI 时间步预测的位置序列；
- `/dynamic_obstacles/markers`：Foxglove/RViz 可视化；
- `/avoidance/perception_status`：输入延迟、轨迹数量和数据质量。

### 4.4 时间与失效要求

- 激光数据年龄超过 200 ms：进入限速状态；
- 超过 500 ms：输出停车请求；
- 跟踪目标短暂丢失后保留 0.3～0.8 s；
- 动态目标静态化必须经过持续观测，不能因单帧速度接近零就写入静态地图；
- 所有输入必须使用传感器时间戳，不使用处理完成时间代替采样时间。

## 5. 第一层：全局绕行与任务决策

### 5.1 定位

- 作用距离：约 3～15 m；
- 运行频率：1～2 Hz；
- 处理持续封路、全局路径失效和局部绕行长期失败；
- 不参与近距离速度闭环。

### 5.2 职责

1. 使用静态地图生成主路径。
2. 将持续存在超过 2～3 s 的障碍判定为候选通道阻塞。
3. MPPI 连续绕行失败或原地停车超过 5 s 时触发重新规划。
4. 区分短暂横穿人员与真正封路，避免全局路径频繁抖动。
5. 对全局无路、路径超时和任务超时给出明确状态。

### 5.3 室内与室外策略

室内：

- 允许把稳定障碍写入有生命周期的临时全局障碍层；
- 障碍消失后自动清除，不修改原始静态地图；
- 重新规划时优先保持原巡检方向和途经点顺序。

室外 RTK：

- 不把瞬时激光点写入全局静态代价地图；
- 保持 RTK 主路线，只生成局部绕行走廊；
- 持续封路时由行为层生成临时旁路点或等待人工确认。

### 5.4 状态机

```text
CLEAR
  ├─ 动态目标横穿 -> YIELD
  ├─ 局部存在可行空隙 -> LOCAL_BYPASS
  ├─ 障碍持续封路 -> GLOBAL_REPLAN
  ├─ 全局无可行路径 -> BLOCKED_WAIT
  └─ 超时或高风险 -> MANUAL_TAKEOVER
```

状态切换必须带滞回：

- `YIELD -> CLEAR`：连续安全不少于 0.8 s；
- `LOCAL_BYPASS -> GLOBAL_REPLAN`：连续失败不少于 3 次；
- `BLOCKED_WAIT -> REPLAN`：障碍显著变化后再触发，避免循环重规划。

## 6. 第二层：MPPI 时空预测绕行

### 6.1 定位

- 作用距离：约 0.6～4 m；
- 控制频率：20 Hz；
- 预测时间：目标 3.0～3.5 s；
- 负责动态目标预测、局部绕行和回归主路径。

### 6.2 动态风险模型

对轨迹第 `k` 个时间步：

```text
t_k = k × model_dt
p_obstacle(t_k) = p_0 + v × t_k
r_safe = r_robot + r_obstacle + r_margin + k_sigma × sigma_position
```

动态障碍代价至少考虑：

- 预测距离；
- 预计碰撞时间 TTC；
- 障碍速度方向；
- 人员目标附加安全距离；
- 目标预测不确定度；
- 机器狗制动距离和控制延迟。

### 6.3 MPPI 代价函数

```text
J =
  W_path       × 路径偏差
+ W_static     × 静态障碍风险
+ W_dynamic    × 动态障碍预测风险
+ W_ttc        × 碰撞时间风险
+ W_control    × 控制变化率
+ W_reverse    × 倒车惩罚
+ W_lateral    × 横移偏好或惩罚
+ W_terminal   × 终点状态
```

硬约束直接淘汰轨迹：

- 足端/机身 footprint 与障碍相交；
- TTC 小于硬阈值；
- 超出地图、坡度或速度边界；
- 需要执行传感器盲区内的高速后退；
- 定位或传感器健康状态不满足运动条件。

### 6.4 Critic 设计

启用现有评价器：

- `ConstraintCritic`：速度和本体约束；
- `CostCritic`：静态及即时障碍；
- `PathFollowCritic`：绕行后回归路线；
- `PathAlignCritic`：以较低权重保持行进方向；
- `TwirlingCritic`：限制无意义原地旋转。

新增评价器：

- `DynamicObstacleCritic`：时间对齐的动态障碍距离；
- `TTCCritic`：预计碰撞时间；
- `BlindZoneCritic`：惩罚传感器不可见区域内的后退和侧移；
- `HumanComfortCritic`：人员目标的侧向余量、接近速度和从背后接近风险。

### 6.5 运动模型

提供两个运行配置：

`Omni`：

- 用于室内宽通道；
- 允许机器狗横移绕过行人；
- 必须限制横向加速度和横移过程中机身旋转。

`DiffDrive`：

- 用于窄过道和室外 RTK 直线巡检；
- 减少左右摆动；
- `vy` 固定为零。

当前配置为 `DiffDrive`，因此现有 `vy_max` 不会产生全向侧移轨迹。切换 `Omni` 前必须先验证 SDK 速度坐标、低姿态横移能力和足端安全边界。

### 6.6 初始建议参数

```yaml
controller_frequency: 20.0
time_steps: 60
model_dt: 0.05
batch_size: 1200
iteration_count: 1
temperature: 0.3
vx_max: 0.30
vx_min: -0.12
vy_max: 0.20
wz_max: 0.35
```

参数必须通过 NX 实机 benchmark 确认，要求 MPPI 单周期 P99 小于 40 ms，不允许因增加采样数量造成控制周期超时。

## 7. 第三层：近场减速和反射停车

### 7.1 定位

- 作用距离：约 0.35～1.5 m；
- 目标频率：30～50 Hz；
- 独立于 MPPI 的最终软件安全屏障；
- 只能放行、限速、减速或停车，不生成主动绕行轨迹。

### 7.2 方向安全区

将当前前向多边形扩展为：

- `ForwardStop`：前进停车；
- `ForwardSlow`：前进减速；
- `LeftShiftStop`：左横移停车；
- `RightShiftStop`：右横移停车；
- `ReverseStop`：后退停车；
- `RotationSweepStop`：旋转时机身扫掠区。

安全区应根据速度动态伸缩：

```text
d_stop = v × reaction_time + v² / (2 × safe_deceleration) + fixed_margin
```

### 7.3 脱困限制

当前允许纯旋转和后退命令绕过前向停车区。目标架构要求：

- 旋转只能在完整机身扫掠区为空时放行；
- 后退只能在后向传感器确认安全时正常放行；
- 无后向传感器时，只允许基于最近自由空间记忆执行低速、限距离盲退；
- 建议盲退速度不超过 0.05 m/s、单次距离不超过 0.15 m；
- 脱困期间任何新障碍或传感器超时都立即停车。

## 8. 第四层：本体硬安全与失效保护

### 8.1 定位

- 响应目标：小于 100 ms；
- 独立于 Nav2、MPPI 和全局行为树；
- 由后置速度门控与 SDK Bridge 双重执行。

### 8.2 触发条件

- 定位状态不是 Normal（当前协议值 `status != 3`）；
- 激光、点云、IMU 或 TF 超时；
- 定位位置跳变或时间回退；
- SDK 通信断开；
- 本体进入急停、安全状态或遥控权丢失；
- 非零速度命令超过 watchdog 时间；
- 本体姿态、倾角、电机或电量状态不允许运动；
- 上层控制源冲突或命令时间戳异常。

### 8.3 动作序列

1. 立即向 SDK 输出零速度。
2. 清空 MPPI、平滑器、命令仲裁器和 SDK Bridge 的旧速度缓存。
3. 取消当前 Nav2 动作。
4. 发布统一 `SAFE_HOLD` 状态和原因码。
5. 按故障类型决定保持站立、低姿态或安全趴下。
6. 只有定位、传感器、控制权和目标姿态同时恢复后才允许非零速度。

## 9. 场景配置

| 参数 | 室内正常 | 室外 RTK | 低姿态/匍匐 |
|---|---:|---:|---:|
| 最大前进速度 | 0.30 m/s | 0.30 m/s | 0.12 m/s |
| 最大后退速度 | 0.08 m/s | 0.05 m/s | 0.05 m/s |
| 最大横移速度 | 0.20 m/s | 0～0.10 m/s | 0.08 m/s |
| MPPI 模型 | Omni | DiffDrive | Omni/低速 |
| 动态预测 | 启用 | 启用 | 启用 |
| 临时全局障碍层 | 启用 | 禁用 | 按室内策略 |
| 人员停车余量 | 0.45 m | 0.60 m | 0.35 m |
| 盲区后退 | 禁止/严格限距 | 禁止/严格限距 | 严格限距 |

低姿态模式必须保持“单一目标姿态”状态锁。动态避障只能调整速度，不得通过非零速度命令隐式切换到站立模式。

## 10. 状态、诊断与录包

新增统一状态：

```text
/avoidance/state
  mode: CLEAR | YIELD | BYPASS | SLOW | STOP | REPLAN | SAFE_HOLD
  active_layer
  reason_code
  obstacle_track_id
  minimum_clearance
  minimum_ttc
  command_source
  timestamp
```

建议录制：

- `/front_lidar`、`/laser_scan`；
- `/dynamic_obstacles/tracks`；
- `/dynamic_obstacles/predictions`；
- `/local_costmap/costmap_raw`；
- `/transformed_global_plan`；
- `/trajectories` 或降采样后的 MPPI 候选轨迹；
- `/cmd_vel_nav`、`/cmd_vel_candidate`、`/cmd_vel_safe`；
- `/polygon_stop`、`/polygon_slowdown`；
- `/avoidance/state`；
- `/sensor_health`、`/localization_info`；
- `/robot_motion_state`、`/mode_switch_cmd`。

必须记录每一层修改速度的前后值和原因，才能区分“MPPI 主动绕行”“Collision Monitor 限速”和“硬安全停车”。

## 11. 软件模块建议

### 11.1 新增模块

```text
robot/src/navigation/src/navigo_dynamic_obstacle_tracker/
robot/src/navigation/src/navigo_mppi_controller/src/critics/dynamic_obstacle_critic.cpp
robot/src/navigation/src/navigo_mppi_controller/src/critics/ttc_critic.cpp
robot/src/navigation/src/navigo_command_mux/
```

### 11.2 修改模块

- `robot_navigo/params/navigo_params.yaml`：场景配置和评价器参数；
- `navigo_mppi_controller`：时空障碍输入和新 Critic；
- `navigo_collision_monitor`：方向多边形、速度相关安全距离和状态输出；
- `navigation_launch.py`：跟踪器和命令仲裁器启动；
- `vel_cmd_udp_publisher.cpp`：只接收安全速度并强化 watchdog；
- Edge Agent：配置切换、状态上报、任务阻塞处理和人工接管；
- rosbag 脚本：记录动态避障证据链。

## 12. 分阶段实施计划

### 阶段 0：基线和回放环境

- 固化当前直线、转弯、窄通道和人员横穿 rosbag；
- 建立避障指标计算脚本；
- 补齐 `/cmd_vel_nav -> /cmd_vel_safe` 全链路录包；
- 建立无本体运动的离线 Shadow 回放。

### 阶段 1：恢复静态局部绕行

- 按室内配置启用 `local_costmap.obstacle_layer`；
- 启用并调低 `CostCritic` 初始权重；
- 启用 `PathFollowCritic` 和低权重 `PathAlignCritic`；
- 保持 Collision Monitor 作为独立停车层；
- 验证静态纸箱、墙角和窄通道绕行。

### 阶段 2：动态目标跟踪

- 实现聚类、数据关联和 EKF；
- 输出目标速度、协方差和预测轨迹；
- 在录包回放中验证目标 ID 稳定性和速度误差；
- 此阶段只发布数据，不控制机器狗。

### 阶段 3：MPPI 动态 Critic

- 接入 `DynamicObstacleCritic` 和 `TTCCritic`；
- 先以 Shadow 模式比较原始与新控制输出；
- 验证横穿、迎面、同向慢行和突然停止目标；
- 确认 NX 上 P99 控制周期满足要求。

### 阶段 4：统一安全仲裁

- Nav2、辅助行为和遥控接入 command mux；
- 所有非急停速度经过 Collision Monitor；
- 增加左右、后方和旋转安全区；
- 增加控制源切换清零和旧命令禁止重放。

### 阶段 5：全局阻塞与重规划

- 增加 `YIELD/BYPASS/REPLAN/BLOCKED_WAIT` 状态机；
- 室内启用临时全局障碍层；
- 室外采用局部走廊和临时旁路点；
- 接入云端任务状态和人工接管提示。

### 阶段 6：实机灰度

1. 架空轮或安全支架验证速度方向。
2. 0.05 m/s 空场验证。
3. 0.10 m/s 软障碍验证。
4. 0.20 m/s 静态绕行。
5. 人员低速横穿。
6. 窄通道、盲角和多人交叉场景。
7. 室外 RTK 路线验证。

每一阶段失败都回退到上一层稳定策略，不允许同时调整感知、MPPI 和本体控制参数。

## 13. 验收指标

### 13.1 硬指标

- 碰撞次数：0；
- 安全层失效后继续输出非零速度：0 次；
- 传感器超时到停车：不超过 500 ms；
- 定位状态异常到停车：不超过 100 ms；
- MPPI 控制周期 P99：小于 40 ms；
- 命令源切换后旧速度重放：0 次。

### 13.2 效果指标

- 静态障碍绕行成功率；
- 动态人员横穿避让成功率；
- 最小人员间距；
- 最小 TTC；
- 误停车率；
- 停车后恢复成功率；
- 局部绕行失败后的全局重规划成功率；
- 路线完成时间增量；
- 左右摇摆和角速度反转次数。

### 13.3 回归场景

- 宽通道单人横穿；
- 迎面来人；
- 同向慢行人员；
- 人员突然停止；
- 两人反向交叉；
- 窄过道不可绕行；
- 静态纸箱和临时推车；
- 激光短暂丢帧；
- 定位状态从 3 切换为异常；
- 站立和低姿态分别执行前进、后退、横移、旋转；
- 远程控制和自主导航切换。

## 14. 架构决策结论

1. MPPI 负责“选择如何绕”，Collision Monitor 负责“当前还能不能动”，两者不能合并。
2. 动态目标必须使用时间对齐的轨迹预测，不能只把当前点写入二维代价地图。
3. 全局地图只接受稳定障碍，瞬时人员不得污染静态地图。
4. 遥控可以绕过自主规划，但不能绕过软件和本体安全层。
5. 机器狗侧移避障必须使用 Omni 模型并经过单独实机验证。
6. 后退和旋转恢复必须有方向可见性证据，不能无条件绕过前向停车区。
7. 所有层必须输出可回放的状态、输入、输出和原因码，保证问题能够离线复现。
