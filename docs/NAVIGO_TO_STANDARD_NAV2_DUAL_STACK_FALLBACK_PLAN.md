# Navigo 自研组件与标准 Nav2 双栈回退方案

状态：**方案已形成，尚未实施**

基线日期：2026-08-30

适用环境：ROS 2 Humble、NX_XG3588、RoamerX Edge、四足机器狗导航

## 1. 目标与结论

本文档解决以下问题：

1. 明确当前自研 Navigo 与标准 Nav2 的功能和接口差异；
2. 判断现有云端任务、地图、定位和运动控制链能否复用；
3. 给出不影响现网默认行为的双栈改造方案；
4. 定义从 Navigo 切换到标准 Nav2、失败后自动切回 Navigo 的流程；
5. 给出无运动验证、仿真/回放和实机验收门槛。

结论：**可以实现可控、快速的任务级回退，但当前不能直接替换包名或热切换。**

当前系统具有较好的对外兼容基础：云端和 Edge 使用标准 `nav2_msgs`、`/follow_waypoints`、`/map_server/load_map`，主要生命周期节点也采用标准名称。真正的阻碍位于内部插件 ABI、参数、行为树库、进程识别和三项机器狗定制行为。

推荐分两级推进：

- **L1（推荐生产回退模式）**：标准 Nav2 核心组件，保留必要的机器狗安全适配和定位/运动控制外围节点；
- **L2（最终纯标准模式）**：全部导航核心使用标准 Nav2，自研能力迁移到标准插件组合或 Nav2 之外的应用/安全适配层。

在户外 RTK 直线规划、终点朝向和定位失效速度门控通过等价性验收前，标准模式不得替代现网 Navigo 默认模式。

## 2. 当前基线

### 2.1 已安装组件

本机审计结果：

| 项目 | 当前状态 |
| --- | --- |
| ROS 发行版 | Humble |
| 自研 Navigo 包版本标记 | `1.1.18` |
| 官方 Nav2 核心包 | 已安装，`1.1.20` |
| `nav2_bringup` | **未安装**，APT 源存在 `1.1.20` 候选版本 |
| 现网启动入口 | `robot/script/robot/start_navigation_real.sh` |
| 现网导航 launch | `robot_navigo navigation_bringup.launch.py` |
| 现网组件容器 | `navigo_container` |
| 定位 | 独立 PCD 定位栈，`/localization_info.status == 3` 为有效 |
| 导航地图 | YAML + 栅格图，由 Map Server 消费 |
| 运动输出 | `/cmd_vel` 经持久化 UDP 桥发送到机器狗控制器 |

官方核心包已具备 `map_server`、`controller_server`、`planner_server`、`behavior_server`、`velocity_smoother`、`collision_monitor`、`bt_navigator` 和 `waypoint_follower` 可执行程序。缺少 `nav2_bringup` 不妨碍编写自有标准 launch，但为了版本一致和后续维护，应在实施阶段显式安装并锁定版本。

### 2.2 当前端到端链路

```text
云平台任务/路线
    │ MQTT
    ▼
RoamerX Edge
    │ nav2_msgs/action/FollowWaypoints
    ▼
waypoint_follower → bt_navigator → planner_server → controller_server
                                                        │ cmd_vel_nav
                                                        ▼
                                              velocity_optimizer
                                                        │ cmd_vel_raw
                                                        ▼
                                               collision_monitor
                                                        │ cmd_vel
                                                        ▼
                                           UDP 机器狗运动控制桥

地图链：平台地图版本 → Edge 激活软链接 → PCD 定位加载 + YAML Map Server 加载
定位链：PCD/NDT/FAST-LIO → map/odom/base_link TF + /odom/nav2 + /localization_info
```

### 2.3 当前代码绑定点

| 绑定位置 | 当前绑定 |
| --- | --- |
| 启停脚本 | `robot_navigo`、`navigo_container` 进程模式 |
| 导航 launch | `navigo_*` 包和组件类 |
| 参数文件 | `navigo_*` 插件类型、`velocity_optimizer` 节点名 |
| Edge 栈检测 | 识别 `robot_navigo navigation_bringup.launch.py` 和 `navigo_container` |
| Edge 动态参数 | 下发自研直线规划和终点朝向参数 |
| 行为树 | 加载 `navigo_*_bt_node` 插件库 |
| 插件基类 | `navigo_core` / `navigo_costmap_2d`，不是 `nav2_core` / `nav2_costmap_2d` |

因此，自研和官方插件不能在同一个 Controller/Planner Server 内随意混装。即使类名和算法名称相似，Pluginlib 的基类类型也不同。

## 3. 功能差异

### 3.1 可直接复用的对外契约

| 契约 | Navigo | 标准 Nav2 | 结论 |
| --- | --- | --- | --- |
| 航点任务 | `/follow_waypoints` + `nav2_msgs/action/FollowWaypoints` | 支持 | 可复用 |
| 单点导航 | `/navigate_to_pose` | 支持 | 可复用 |
| 地图加载 | `/map_server/load_map` + `nav2_msgs/srv/LoadMap` | 支持 | 可复用 |
| 生命周期节点名 | planner/controller/BT/waypoint | 标准命名 | 可保持一致 |
| 地图、里程计、机器人坐标系 | `map` / `odom` / `base_link` | 标准约定 | 可复用 |
| 速度消息 | 当前为 `geometry_msgs/Twist` 链 | Humble 可配置 | 需固定消息类型和 remap |
| 地图版本校验 | 云端与 Edge 业务能力 | Nav2 不负责 | 与导航实现解耦 |

只要标准模式维持上述名称，云平台 API、任务模型和页面无需感知底层导航实现。Edge 需要适配进程检测和自研参数操作，但不需要改变云端协议。

### 3.2 内部组件映射

| 自研组件 | 标准候选 | 差异与处理 |
| --- | --- | --- |
| `navigo_map_server` | `nav2_map_server` | YAML 栅格地图服务兼容；需验证图像解析、QoS 和在线换图 |
| `navigo_path_planner` | `nav2_planner` | Server 接口兼容，内部基类不同 |
| `navigo_navfn_planner` | `nav2_navfn_planner` | 官方没有当前直线优先/失败回退参数 |
| `navigo_path_controller` | `nav2_controller` | Server 接口兼容；GoalChecker 行为不同 |
| `navigo_mppi_controller` | `nav2_mppi_controller` | 参数大体同源，需逐项核对 Humble 1.1.20 参数和默认值 |
| `navigo_costmap_2d` | `nav2_costmap_2d` | 插件 ABI 不兼容，配置中的插件类必须全部替换 |
| `navigo_behaviors` | `nav2_behaviors` | 行为名称相近，插件类和 BT 库名不同 |
| `navigo_bt_navigator` | `nav2_bt_navigator` | Action 接口兼容，BT XML 和插件库必须迁移 |
| `navigo_waypoint_follower` | `nav2_waypoint_follower` | FollowWaypoints 与 WaitAtWaypoint 可映射 |
| `navigo_velocity_optimizer` | `nav2_velocity_smoother` | 节点名和生命周期清单不同，需保留速度话题链 |
| `navigo_collision_monitor` | `nav2_collision_monitor` | 官方基础能力可用，但缺少当前定位门控等定制语义 |

### 3.3 三项阻断“直接回退”的定制能力

#### 3.3.1 户外 RTK 直线路径

当前自研 NavFn 增加：

- `GridBased.prefer_straight_line`：直接生成起点到目标点的 0.25 m 采样直线；
- `GridBased.allow_straight_line_fallback`：占据栅格规划失败时仍生成 GPS 直线路径。

标准 NavFn 的公开参数主要是 `tolerance`、`use_astar`、`allow_unknown` 和终点朝向策略，不提供上述语义。直接切换会使户外 RTK 巡逻重新依赖占据栅格，可能出现规划拒绝、路线绕行或行为改变。

处理选项：

1. L1 阶段保留一个基于 `nav2_core::GlobalPlanner` 重新实现的兼容插件；
2. 标准模式第一阶段只开放室内任务，户外任务继续使用 Navigo；
3. L2 阶段由 Edge 生成标准 `nav_msgs/Path` 并调用 FollowPath，或使用经验证的标准路线能力替代。

不建议在没有实测前简单删除两个参数并继续执行户外任务。

#### 3.3.2 巡逻点与对接点朝向

当前自研 GoalChecker：

- 普通巡逻点收到 `/navigation/require_goal_yaw=false` 时，只校验 XY；
- 充电/对接终点收到 `true` 时，使用 `required_yaw_goal_tolerance` 强制校验朝向；
- Edge 会动态调整对接精度。

标准模式可使用两个官方 GoalChecker 实现等价行为：

- 普通航点：`nav2_controller::PositionGoalChecker`；
- 对接终点：`nav2_controller::SimpleGoalChecker`；
- 通过标准 GoalChecker Selector 或分离的导航行为树选择插件。

实施后 Edge 不再写入 `required_yaw_goal_tolerance`，而是切换标准 GoalChecker 并设置标准 `xy_goal_tolerance` / `yaw_goal_tolerance`。

#### 3.3.3 定位失效速度门控

当前自研 Collision Monitor 额外实现：

- 订阅 `/localization_info`；
- 从未收到定位、定位超时或 `status != 3` 时禁止运动；
- 前向停止区触发时，允许纯旋转或倒退脱困；
- 支持停止确认周期和传感器性能诊断。

标准 Collision Monitor 能提供传感器多边形减速/停车，但不能直接替代当前所有定制语义。它是 CPU 级辅助安全层，不应当被当作安全认证硬件。

L1 推荐保留自研安全节点，或在标准 Collision Monitor 后增加独立 `localization_velocity_guard`。L2 可复用现有 UDP 运动桥的定位健康门控作为最后一道防线，但必须验证：

- 定位丢失到零速度的最坏时延；
- 零速度持续发布行为；
- 导航恢复和远程控制的优先级；
- 旋转/倒退脱困不会穿越安全区；
- Edge 进程异常和 ROS 图中断时默认安全。

## 4. 目标架构

### 4.1 双栈选择

统一保留入口：

```bash
/home/dogrobot/robot/script/robot/start_navigation_real.sh start
```

新增实现选择：

```bash
NAV_STACK_IMPL=navigo   # 默认，现网行为不变
NAV_STACK_IMPL=nav2     # 标准 Nav2 回退模式
```

目标结构：

```text
                         ┌──────────────────────────────┐
云平台 / Edge 公共契约 ─▶│ /follow_waypoints / load_map │
                         │ lifecycle / TF / cmd_vel     │
                         └──────────────┬───────────────┘
                                        │ NAV_STACK_IMPL
                       ┌────────────────┴────────────────┐
                       ▼                                 ▼
             Navigo 生产配置                    标准 Nav2 回退配置
             navigo_params.yaml                 nav2_params.yaml
             navigo_container                   nav2_container
                       └────────────────┬────────────────┘
                                        ▼
                          公共 PCD 定位与地图版本契约
                                        ▼
                         公共速度安全链与 UDP 运动桥
```

### 4.2 配置与进程原则

1. `NAV_STACK_IMPL` 缺省必须为 `navigo`，避免部署后无意切换；
2. 两种实现使用独立参数文件，禁止在一个 YAML 中堆叠两套插件类型；
3. 状态检查优先检查生命周期、Action、地图服务、TF 和速度链，不再仅依赖进程字符串；
4. 任何时刻只允许一个导航容器存在；
5. 定位栈默认不随导航实现切换而重启；
6. 切换前必须取消活动 Action 并确认输出零速度；
7. 标准模式启动失败时自动停止残留标准组件，再恢复 Navigo；
8. 不通过卸载 Navigo 包实现回退，两个构建产物并存。

### 4.3 标准模式速度链

标准模式必须显式 remap 为：

```text
nav2_controller / nav2_behaviors
            │ /cmd_vel_nav
            ▼
nav2_velocity_smoother
            │ /cmd_vel_raw
            ▼
安全门控 / nav2_collision_monitor
            │ /cmd_vel
            ▼
vel_cmd_udp_pub
```

不得让 Controller Server 绕过平滑器或安全门控直接发布到 `/cmd_vel`。

## 5. 改造清单

### 5.1 依赖与版本

1. 安装并锁定 `ros-humble-nav2-bringup=1.1.20-*`；
2. 记录所有 `ros-humble-nav2-*` 的精确版本；
3. 禁止标准模式混用 Rolling/Jazzy 参数文档；
4. 构建时先 source `/opt/ros/humble/setup.bash`，再 source 项目 overlay；
5. 检查 overlay 中没有误覆盖同名 `nav2_*` 官方包。

### 5.2 Robot 侧

计划新增或修改：

| 文件/包 | 改造 |
| --- | --- |
| `robot/script/robot/start_navigation_real.sh` | 增加实现选择、实现无关的启停/状态判断、失败自动回退 |
| `robot_nav2`（建议新增包） | 仅放标准 Nav2 的 real-robot launch、参数和 BT XML |
| `robot_nav2/params/nav2_params.yaml` | 将所有插件类型转换为 `nav2_*`，保持实机速度、footprint、TF 和话题配置 |
| `robot_nav2/launch/navigation_bringup.launch.py` | 启动官方组件并保留现有传感器、定位和机器狗外围节点 |
| 安全适配节点 | L1 保留自研安全门控；L2 再迁移为独立实现或标准配置 |

建议不要直接把 `robot_navigo` launch 改成官方组件。独立包更容易审计、并行安装和一键回滚。

### 5.3 Edge 侧

| 文件 | 改造 |
| --- | --- |
| `navigation_stack_adapter.py` | 状态识别同时支持两种容器；增加当前实现和回退结果字段 |
| `ros_adapter.py` | 将动态参数按实现分组；标准模式使用官方参数/选择器 |
| `task_executor.py` | 切换期间暂停、取消、记录未完成航点并恢复 |
| 配置模型 | 增加 `navigation.stack_impl`，默认 `navigo` |
| 单元测试 | 覆盖启动、换图、标准启动失败、恢复 Navigo、任务断点续跑 |

云平台不需要新增两套任务模型。可选增加只读字段 `navigation_impl`，用于运维展示当前机器人使用的导航实现。

### 5.4 参数迁移规则

必须逐项迁移，禁止只做字符串全局替换：

```yaml
# 示例，不是可直接部署的完整配置
controller_server:
  ros__parameters:
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
    pass_through_goal_checker:
      plugin: "nav2_controller::PositionGoalChecker"
    docking_goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"

planner_server:
  ros__parameters:
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
```

迁移时重点核对：

- MPPI 参数名、类型、动态参数能力和 Humble 默认值；
- `Twist` / `TwistStamped` 配置；
- costmap 插件类名和基类；
- BT XML 中插件端口和库名；
- Waypoint Follower 的 pause、失败续跑语义；
- Velocity Smoother 的输入输出 remap；
- Lifecycle Manager 的节点名和启动顺序；
- `map_subscribe_transient_local`、地图加载 QoS 和在线换图行为。

## 6. 切换与自动回退流程

### 6.1 Navigo → 标准 Nav2

```text
1. 禁止接收新任务
2. 暂停当前任务并取消 FollowWaypoints Goal
3. 确认 /cmd_vel 连续为零
4. 记录任务 ID、地图 ID/版本、当前航点和剩余航点
5. 停止 Navigo 导航组件，保留有效定位
6. 启动标准 Nav2
7. 等待全部 lifecycle active
8. 验证地图版本、Map Server、TF、定位 status=3、Action 和速度链
9. 恢复未完成航点；未通过则进入自动回退
10. 恢复接收新任务
```

“平滑”定义为任务级受控暂停和续跑，不承诺运行中组件零停机热替换。

### 6.2 标准 Nav2 启动失败 → Navigo

触发条件包括：

- 任一必需生命周期节点未进入 `active [3]`；
- `/follow_waypoints` 不可用；
- `/map_server/load_map` 不可用或地图加载失败；
- 地图 ID/版本不一致；
- `/localization_info.status != 3`；
- `map → odom → base_link` TF 不连通或超时；
- `/cmd_vel` 安全链发布者/订阅者不符合预期；
- 标准模式启动后出现异常非零速度；
- 任务恢复失败。

回退步骤：

```text
1. 发布并保持零速度
2. 取消标准 Nav2 Action
3. 停止标准容器及残留进程
4. 验证没有重复 controller/collision monitor
5. 使用原 Navigo 参数启动
6. 重新检查定位、地图、TF、生命周期和速度链
7. 从记录的未完成航点恢复；失败则任务进入人工处理
8. 记录切换原因、两侧日志和最终实现
```

单次切换只允许一次自动反向恢复，避免两个实现反复重启。回退仍失败时保持零速度，并上报 `NAV_STACK_FALLBACK_FAILED`。

### 6.3 地图版本约束

标准 Nav2 不解决 `MAP_VERSION_MISMATCH`。双栈必须继续遵守：

1. 任务携带的地图 ID/版本与机器人激活地图一致；
2. Edge 激活地图后同时加载定位 PCD 和导航 YAML；
3. 两个服务均成功后才更新机器人当前地图版本；
4. 切换导航实现不得改变当前地图软链接；
5. 标准模式启动后重新读取/确认 Map Server 消费的 YAML；
6. 版本不一致时禁止显示位置、禁止恢复任务、禁止输出运动速度。

## 7. 验证计划

所有验证按阶段推进。未得到明确授权时，不启动实机运动。

### 7.1 P0：静态与构建检查

- 标准包依赖解析成功；
- 新标准 launch 可完成语法检查；
- 参数文件可被 `RewrittenYaml` 解析；
- Pluginlib 能加载所有声明插件；
- 两种模式的进程模式不会互相误杀；
- 默认值保持 `navigo`；
- Edge 单元测试覆盖两种状态输出和失败回退。

### 7.2 P1：无运动 ROS 图验证

抬起机器狗或切断运动使能，在不允许运动的条件下验证：

- `/planner_server`、`/controller_server`、`/bt_navigator`、`/waypoint_follower` 均为 `active [3]`；
- `/follow_waypoints` 类型与 Edge 客户端一致；
- Map Server 可加载当前 YAML；
- PCD 定位保持 `status=3`；
- TF 连通且只有预期发布者；
- Controller 输出经过 smoother 和安全节点；
- 切换前后没有重复容器、重复速度发布者或残留 Action；
- 定位失效时最终 `/cmd_vel` 为零。

### 7.3 P2：离线回放/仿真

- 使用已有导航诊断 rosbag 回放 `/cmd_vel_raw`、`/cmd_vel`、停止区和减速区；
- 比较 Navigo 与标准模式的全局路径、局部路径、速度、角速度和停止响应；
- 覆盖无障碍、临时障碍、路径阻塞、旋转脱困、倒退脱困；
- 覆盖任务取消、暂停、续跑、换图和生命周期节点崩溃；
- 标准模式失败时验证自动恢复 Navigo。

### 7.4 P3：室内低速实机

准入条件：P0、P1、P2 全部通过，并取得实机运动授权。

- 限速不高于 0.20 m/s；
- 先执行短直线，再执行 3 点闭环路线；
- 验证巡逻点不强制旋转；
- 验证障碍停止、减速、取消和急停；
- 连续 10 次无异常速度、无生命周期退出、无地图错配。

### 7.5 P4：户外 RTK 与充电对接

- 比较直线路径行为和目标点可达率；
- 覆盖栅格未知区、规划失败和 RTK 短时抖动；
- 对接终点必须严格验证 XY 和 yaw；
- 覆盖对接失败、取消和回退；
- 定位状态非 3、定位超时、TF 中断时必须立即禁止运动；
- 连续完成规定路线和对接回归后，标准模式才可开放对应任务类型。

## 8. 验收门槛

### 8.1 接口验收

- 云端无需区分两套任务协议；
- Edge 的 FollowWaypoints、取消、反馈和结果语义一致；
- 在线换图不产生旧 PCD + 新 YAML 或新 PCD + 旧 YAML 的混合状态；
- 状态接口能显示当前实现和最近一次回退原因。

### 8.2 行为验收

- 相同地图、路线和定位输入下，室内任务成功率不低于现网基线；
- 普通航点不因朝向要求原地打转；
- 对接点满足专用 XY/yaw 精度；
- 户外直线路径能力有明确等价实现，或标准模式明确禁止户外任务；
- 任务切换后从未完成航点恢复，不重复整个路线。

### 8.3 安全验收

- 任一实现都只有一条最终速度输出链；
- 定位无效、地图不匹配、TF 断裂和安全传感器超时均输出零速度；
- 标准模式启动失败不会留下可继续输出速度的残留 Controller；
- 自动回退失败后保持停车，不自动无限重试；
- 远程控制与自主导航速度仲裁保持现网优先级。

### 8.4 性能验收

- Controller 周期、CPU、内存和容器稳定性不劣于现网可接受基线；
- `/cmd_vel` 端到端延迟和抖动满足机器狗控制要求；
- 地图加载、生命周期激活和任务恢复耗时可观测；
- 连续运行期间无新增 core、内存持续增长或 ROS 图重复节点。

## 9. 实施阶段

| 阶段 | 交付物 | 默认行为 |
| --- | --- | --- |
| A. 基线固化 | 版本清单、接口契约测试、现网数据基线 | Navigo |
| B. 标准配置 | `robot_nav2` launch、参数、BT、依赖锁定 | Navigo |
| C. Edge 双栈 | 实现选择、状态识别、自动回退、单元测试 | Navigo |
| D. 无运动验证 | 生命周期、地图、TF、Action、速度安全报告 | Navigo |
| E. 回放/仿真 | 差异报告和参数调整 | Navigo |
| F. 室内灰度 | 单机、低速、限定任务类型 | 可单机选择 Nav2 |
| G. 户外/对接 | 直线规划和 GoalChecker 等价性验收 | 按能力开放 |
| H. 生产切换 | 可配置默认实现、监控和回退 SOP | 评审后决定 |

每一阶段独立提交。不得在同一提交中同时引入标准栈、切默认值并移除 Navigo。

## 10. 风险与决策

| 风险 | 等级 | 控制措施 |
| --- | --- | --- |
| 插件 ABI 不兼容导致启动失败 | 高 | 两套独立参数/launch；启动前 Pluginlib 检查 |
| 户外路线不再直线或规划失败 | 高 | 标准模式先禁用户外；实现等价能力后开放 |
| 定位失效仍输出速度 | 高 | 保留独立安全门控；故障注入验收 |
| 充电对接朝向不满足 | 高 | 双 GoalChecker + Selector；单独对接回归 |
| Twist 类型/话题 remap 错误 | 高 | 图契约测试；检查唯一速度链 |
| BT 插件名或端口版本漂移 | 中 | 固定 Humble 1.1.20；使用对应版本 BT XML |
| 在线换图消费者不同步 | 高 | PCD + YAML 原子加载；失败不更新当前版本 |
| 进程识别误判导致双栈并存 | 高 | 生命周期 + PID/容器双检查；启动前互斥锁 |
| 标准模式性能退化 | 中 | rosbag 对比和持续运行基线 |

决策建议：

1. 批准实施 L1 双栈，不立即追求纯标准；
2. Navigo 继续作为默认实现，直至全部安全和行为门槛通过；
3. 标准模式先面向室内普通巡逻；
4. 户外 RTK 和充电对接分别设置能力开关；
5. 完成至少一次“标准启动失败 → 自动恢复 Navigo”的实机静止演练后，才允许灰度。

## 11. 参考

仓库证据：

- `robot/script/robot/start_navigation_real.sh`
- `robot/src/navigation/src/robot_navigo/launch/navigation_launch.py`
- `robot/src/navigation/src/robot_navigo/params/navigo_params.yaml`
- `robot/src/navigation/src/navigo_navfn_planner/src/navfn_planner.cpp`
- `robot/src/navigation/src/navigo_path_controller/plugins/simple_goal_checker.cpp`
- `robot/src/navigation/src/navigo_collision_monitor/src/collision_monitor_node.cpp`
- `edge-agent/roamerx_edge/navigation_stack_adapter.py`
- `edge-agent/roamerx_edge/ros_adapter.py`

官方资料：

- [Nav2 Configuration Guide](https://docs.nav2.org/configuration/)
- [NavFn Planner](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/planners_plugins/configuring_navfn/)
- [Waypoint Follower](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/waypoint_follower/)
- [Lifecycle Manager](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_lifecycle_manager/)
- [Using Collision Monitor](https://docs.nav2.org/tutorials/docs/using_collision_monitor.html)

## 12. 本文档未执行的操作

- 未安装 `nav2_bringup`；
- 未创建标准 Nav2 launch 或参数文件；
- 未修改 `NAV_STACK_IMPL` 或现网默认启动方式；
- 未重启定位、导航、Edge Agent 或机器狗运动桥；
- 未下发任务、地图或速度命令；
- 未进行实机运动验证；
- 未提交或推送 Git。
