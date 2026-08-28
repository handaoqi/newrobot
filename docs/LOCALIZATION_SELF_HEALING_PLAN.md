# 定位丢失恢复与自愈方案

制定日期：2026-08-25
状态：三个 P0 项（§2.1 / §2.2 / §2.3）**已于 2026-08-25 落成代码并编译通过**，见各节的"已实现"块与 §4.1。§2.4–§2.6 仍是方案。
**尚未重启定位服务，改动未在实机生效。**

---

## 0. 结论摘要

自愈框架**已经相当完整，不需要重建**。检测有三条链路、恢复有边缘与机器人两层、任务侧有暂停/续跑，代码质量也不差。

真正的问题是六个具体缺口，其中三个最关键：

1. **恢复循环无上限、无告警** —— 定位丢不回来时任务永久 `paused`，不升级为失败、不产告警、运维侧完全看不见。
2. **定位丢失根本不产生 `alert.event`** —— `emit_system_alert` 全项目**只有 1 处生产调用**，且只为 SLAM 发散。定位丢失在平台 `InspectionEvent` 里查不到。
3. **`ReinitializeGlobalLocalization` 插件已编译注册但无任何行为树引用** —— 现成能力闲置，Nav2 恢复行为里实际没有重定位环节。

> 与 [IMU 漂移诊断](IMU_DRIFT_DIAGNOSIS_AND_REMEDIATION_PLAN.md) 的关系：那份文档降低定位丢失的**发生频率**（修掉 0.703 °/s 的陀螺零偏），本文处理丢失**之后**的处置。两者互补，不重叠。**建议先做 IMU 整改**——如果丢失频率本身能降一个数量级，本文很多补丁的紧迫性会下降。

---

## 1. 现状：已有的三层自愈

### 1.1 检测层（`edge-agent/roamerx_edge/ros_adapter.py`）

三条独立链路：

| 链路 | 位置 | 判据 |
| --- | --- | --- |
| A：定位状态异常 | `:304` `_on_localization` | `Localization.status != 3` 连续 `localization_loss_samples`(5) 帧 |
| B：NDT 匹配退化 | `:393` `_on_scan_matching_status` | `!has_converged` 或 `matching_error >= ndt_failure_score`(0.5)，连续 `ndt_failure_samples`(3) 帧 |
| C：绝对定位源确认 | `:423` `_absolute_localization_stable` | `active_source ∈ {ndt_imu, rtk_imu}` 且 `absolute_stable` |

状态码语义（`telemetry_collector.py:14-20`）：
```python
LOCALIZATION_STATUS = {0: "initializing", 1: "relocalizing", 2: "relocalized", 3: "normal", 4: "lost"}
```

### 1.2 边缘恢复层（`edge-agent/roamerx_edge/app.py`）

```
检测触发 → _handle_task_localization_loss (:540)
         → task_executor.on_localization_lost()   任务 running → pausing → paused
         → 无活动任务 / 取不到 _localization_recovery_lock → return
         → Thread("task-localization-restart") → _recover_task_localization (:556)
```

`_recover_task_localization` 的策略：取最后可信位姿（内存 → SQLite 持久化），第 1 次尝试先 `restart_localization()` 重启定位节点，然后重播 `/initialpose`（`wait_seconds=30.0, required_normal_samples=3, require_absolute=True`）。3 次快速重试（间隔 5 s）耗尽后休 30 s 开新一轮。

参数（`config.py:62-78`，与部署值 `runtime/nx-edge/conf/edge-agent.yaml:72-85` 一致）：

| 参数 | 值 |
| --- | ---: |
| `localization_stable_seconds` | 3.0 |
| `localization_loss_samples` | 5 |
| `ndt_failure_score` | 0.5 |
| `ndt_failure_samples` | 3 |
| `localization_recovery_attempts` | 3 |
| `localization_recovery_retry_seconds` | 5.0 |
| `localization_recovery_cycle_seconds` | 30.0 |

### 1.3 机器人端自愈层（`robot/src/localization/.../localization_nodelet.cpp`）

独立于边缘的第二层：
- `:1462-1487` NDT 连续失败 ≥ `runtime_relocalization_failure_threshold`(10) 帧，从最后可信位姿调度一次有界全局重定位，失败后 `runtime_relocalization_retry_seconds`(5.0) 重新武装。
- `:1166-1188` RTK 自动恢复，`auto_recovery_retry_seconds: 5.0`。
- `:1388-1425` 源仲裁优先级：`bridge_active` → `rtk_primary` → `ndt_imu` → `rtk usable` → `imu_odom_bridge` → `unavailable`。

### 1.4 任务层（`edge-agent/roamerx_edge/task_executor.py`）

- `:291 on_localization_lost` —— 仅在 `state == "running"` 生效，发 `task.pausing`（`reason_code="LOCALIZATION_LOST"`）并携带丰富诊断（最后可信位姿、原始位姿、定位质量、决策快照、当前航点索引）。然后 `cancel_navigation()` + `stop_motion()`。**cancel 未被确认也不失败**，只 warning 并保持 paused——这是有意为之的安全边界。
- `:353 on_localization_recovered` —— 从 `_nearest_remaining_waypoint_index` 续跑。
- `:849-862` 第二种定位暂停：到点但只有航位推算时，发 `task.paused`（`code="ABSOLUTE_LOCALIZATION_REQUIRED"`）。

### 1.5 人工/平台侧手段（均已实现）

| 能力 | 实现 |
| --- | --- |
| 手动设初始位姿 | `nav.initial_pose` → `ros_adapter.py:707 set_initial_pose` |
| RTK 播种 | `ros_adapter.py:802 set_initial_pose_from_rtk` → 服务 `/localization/seed_from_rtk` |
| **主动重定位** | `nav.relocalize` → `ros_adapter.py:873 active_relocalize`：**8 个偏航角 + 4 个 ±1 m 平移共 12 个候选**，逐个试 `set_initial_pose`，全程 `motion_commanded: False`（不动机器人） |
| 重启定位/导航 | `nav.restart` / `nav.recover` / `restart-localization` |
| 前端入口 | `GuardDutyPage.vue:463-556`「初始化定位」、`RoutePlannerPage.vue:1207-1284`「初始化定位 / 主动重定位」 |

前端诊断展示已经很完整，`TaskExecutionPage.vue:406-424 localizationDebugItems()` 就有十几项：地图一致性、定位状态、当前定位源、决策依据、绝对定位确认、RTK 质量/地图坐标/航向、桥接余量、桥接拒绝原因、NDT 分数、内点率等。

---

## 2. 六个确认的缺口

### 2.1 恢复循环无上限、无告警（P0）

`app.py:556-562`：

```python
def _recover_task_localization(self) -> None:
    try:
        attempts = max(1, int(self.config.safety.localization_recovery_attempts))
        quick_retry = max(0.5, self.config.safety.localization_recovery_retry_seconds)
        cycle_retry = max(1.0, self.config.safety.localization_recovery_cycle_seconds)
        first_cycle = True
        while first_cycle or self.task_executor.is_paused_for_localization():
```

这是**无限循环**。定位恢复不了时的实际表现是：任务永久停在 `paused`，机器人停在原地，每 30 秒重试一轮，直到有人发现。不升级为 `failed`、不产生告警、不通知运维。

`app.py:263-264` 上报了 `nav.recover` / `nav.relocalize` 能力，但 capabilities 里没有任何"恢复进度"字段，平台无法感知恢复线程跑到第几轮。

**补法建议**
- 增加恢复轮次上限配置，超限后升级：产告警 → 任务转 `failed`（或明确的 `awaiting_operator` 态）。
- 每轮把恢复状态（轮次、已耗时、上次失败原因）纳入 `telemetry.status` 上报，前端在任务执行页展示。
- 保留"无限重试"作为可配置选项——某些场景下机器人停在原地等人来确实是最安全的，但**必须可见**。

> **2026-08-25 已实现**
>
> - `SafetyConfig` 新增 `localization_recovery_max_cycles`，**默认 0 = 不限轮次**——保留了上面那条"停在原地等人最安全"的选项，但现在它是显式配置，而不是唯一行为。
> - 超限后升级为 `localization_recovery_failed` 告警（severity `critical`，payload 带 `recovery_cycles`），不再静默。
> - 每轮把 `{cycle, elapsed_seconds, last_failure_reason}` 通过 `telemetry.on_localization_recovery()` 送进状态快照的 `localization.recovery`，平台侧可见恢复线程跑到第几轮。恢复成功的回调会清空该对象并重新武装告警去重。
> - 测试：`test_app_localization_recovery.py` 由 3 个用例扩到 7 个，新增 `test_recovery_escalates_once_the_cycle_budget_is_spent`（断言轮次序列 `[1, 2]`、只发一条 `critical`）等。
>
> **实现注记**：这些测试用 `object.__new__(EdgeAgentApplication)` 构造应用、不跑 `__init__`，所以每加一个协作者都必须在测试里手工挂上——这是扩测试而不是放宽生产代码的原因。

### 2.2 定位丢失不产生告警（P0）

```bash
$ grep -rn "emit_system_alert" edge-agent/ --include=*.py | grep -v "def emit_system_alert"
edge-agent/roamerx_edge/app.py:437:            self.alerts.emit_system_alert(
edge-agent/tests/test_app_localization_recovery.py:96:  （测试桩）
```

**全项目唯一的生产调用点在 `app.py:437`，且只为 `slam_diverged`。**

平台侧 `message_handlers.py:264-276` 的 `alert.event` 分支能正常接收并特判 `slam_diverged` → 触发语音播报，链路是通的。但定位丢失、NDT 退化、恢复失败**都不走这条链路**，只以 `task.pausing` 事件形式存在，平台 `InspectionEvent` 里查不到，也不进告警统计。

**补法建议**：为 `localization_lost`、`localization_recovery_failed`、`ndt_degraded` 增加 `emit_system_alert` 调用。payload 结构现成（`alert_bridge.py:47-67` 已有 `event_id / event_type / severity / occurred_at / task_execution_id / map_id / pose / source / attributes`），且 `on_localization_lost` 已经收集了全部诊断字段，直接复用即可。注意加去重（参照 `_mapping_divergence_notified` 的做法），避免抖动时刷屏。

> **2026-08-25 已实现**
>
> 三种情况都接上了 `emit_system_alert`，分级不同：
>
> | 事件 | severity | 触发点 |
> | --- | --- | --- |
> | `localization_lost` | `high` | `_handle_task_localization_loss` |
> | `ndt_degraded` | `medium` | NDT 退化链路 |
> | `localization_recovery_failed` | `critical` | 恢复轮次超限（§2.1） |
>
> NDT 退化刻意比丢失低一级：它是"还在跑但质量在掉"，和"已经丢了"不是同一件事，同级会让 `high` 告警失去意义。
> 去重参照 `_mapping_divergence_notified`：由**真实的恢复事件**重新武装（收到 recovered 回调时清标志），而不是靠超时清除。
>
> 另有 Phase 2 的 `imu_cross_check_mismatch`（`medium`）走同一条通道，见 [IMU 漂移诊断](IMU_DRIFT_DIAGNOSIS_AND_REMEDIATION_PLAN.md) §2.5。平台侧 `message_handlers.py` 未改动。

### 2.3 `ReinitializeGlobalLocalization` 插件闲置（P1，性价比最高）

插件**已完整编译并注册**：

```
navigo_behavior_tree/plugins/action/reinitialize_global_localization_service.cpp:32-33
    factory.registerNodeType<...>("ReinitializeGlobalLocalization");
navigo_behavior_tree/navigo_tree_nodes.xml:131        <Action ID="ReinitializeGlobalLocalization">
navigo_bt_navigator/src/bt_navigator.cpp:54           "navigo_reinitialize_global_localization_service_bt_node"
robot_navigo/params/navigo_params.yaml:40             - navigo_reinitialize_global_localization_service_bt_node
```

但 `navigo_bt_navigator/behavior_trees/` 下 **13 棵行为树中零引用**。其中 5 棵有恢复子树，用的都是 `ClearEntireCostmap` / `Spin` / `BackUp` / `Wait`：

```
navigate_through_poses_w_replanning_and_recovery.xml       ← 巡检主用
navigate_to_pose_w_replanning_and_recovery.xml
navigate_to_pose_w_replanning_goal_patience_and_recovery.xml
nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml
navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml
```

也就是说：**Nav2 的恢复行为里完全没有重定位环节**。导航失败时只会清代价地图、原地转、后退、等待——如果根因是定位漂了，这些动作全都无效。

**补法建议**：在恢复子树里加入 `ReinitializeGlobalLocalization`，放在 `ClearEntireCostmap` 之后、`BackUp` 之前。这是**改一个 XML 就能启用的现成能力**，工作量极小。需注意与 §1.3 机器人端自愈、§1.2 边缘恢复的触发条件互斥，避免三层同时抢着重定位。

> **2026-08-25 已实现 —— 但上面"改一个 XML 就行"的判断是错的**
>
> 插件确实已编译已注册，但**注册 ≠ 存在服务端**。全工程没有任何节点提供 `/reinitialize_global_localization`，而 `bt_service_node.hpp:85-92` 的 `wait_for_service` 失败时**抛 `std::runtime_error`**——`BtServiceNode::on_configure` 抛异常会让**整棵行为树加载失败**。
> 按原方案只改 XML，结果会是把"恢复行为对定位漂移无效"变成"导航根本起不来"。
>
> 因此实际改了两处：
>
> 1. `localization_nodelet.cpp` 新增 `std_srvs::srv::Empty` 服务端 `/reinitialize_global_localization`。回调只**武装**一次全局重定位（从最后可信位姿重新播种、退出 converged 态、开 `gl_once_gate_`、清 IMU 与里程计预测缓存），昂贵的 ICP 仍在 `points_callback` 里跑——回调立即返回，不占服务线程。无可信位姿历史时只记 WARN（`std_srvs::Empty` 没有返回失败的手段），调用方一律收到成功。
> 2. `navigate_through_poses_w_replanning_and_recovery.xml`（巡检主用）的 `RoundRobin name="RecoveryActions"` 里，在 `ClearingActions` 之后、`Spin` / `BackUp` 之前插入该节点。
>
> 互斥按方案要求处理：回调里置 `runtime_relocalization_attempted_ = true`，把后续重试的所有权交给 `points_callback` 顶部的重试武装逻辑，避免行为树反复驱动同一条路径。
>
> 编译：`localization` 1m44s、`navigo_bt_navigator` 3.31s，均 exit 0，install 产物已核对。
>
> **计划里的路径也是错的**：行为树在 `robot/src/navigation/src/navigo_bt_navigator/behavior_trees/`，不是 `robot/src/navigo_bt_navigator/`。

### 2.4 `ScanMatchingStatus` 可静默失效（P1）

`ros_adapter.py:44-53`：`ScanMatchingStatus` 消息 import 失败时，**整条链路 B（NDT 退化检测）被静默禁用**，只留一条 warning 日志（`:148-151`）。同时 `localization.quality` 遥测也一起消失。

后果：NDT 分数、内点率在前端全部显示为空，而系统看起来"正常运行"。前端 `TaskExecutionPage.vue` / `RemoteControlPage.vue` 的 NDT 展示会静默变空白。

**补法建议**：把这个降级状态**显式暴露**为一个健康字段上报到平台，而不是只写日志。

### 2.5 边缘侧无 clear costmap 接口（P2）

`NavigationStackAdapter`（151 行）除 `reload_map` 外全是 `subprocess` 调 `start_navigation_real.sh`，没有暴露清代价地图。`ClearEntireCostmap` 只存在于机器人端 BT 内部。

定位恢复后代价地图里可能残留着按错误位姿写入的障碍物，续跑前不清会导致路径规划失败。**补法**：在 `_recover_task_localization` 成功后、`on_localization_recovered` 续跑前，调用一次清代价地图。

### 2.6 进程重启不续巡（P2，需确认是否要改）

`task_executor.py:120-125` 构造时强制把持久化的任务状态置为 `interrupted`，`report_startup_interruption()`（`:273-278`）直接 `_fail("EDGE_RESTARTED")`。

`app.py:341` 的注释写明这是有意为之：*"continue/hold/report_only intentionally never auto-start motion after process restart"*——进程重启后绝不自动恢复运动，这个安全边界是对的。

但后果是 Edge Agent 崩溃或重启后巡检任务直接失败，需人工重下。**补法建议**：不改变"不自动运动"的原则，但补一份明确的人工接管 SOP，并让平台侧对 `EDGE_RESTARTED` 有专门提示而非当作普通失败。

---

## 3. 恢复策略分级建议

当前只有一种恢复手段（重启定位节点 + 重播 initialpose），且无限重试。建议改为**分级升级**，从廉价到昂贵逐级尝试，每级有明确的触发条件与放弃判据。

| 级别 | 手段 | 现状 | 触发条件建议 |
| --- | --- | --- | --- |
| L0 | 等待自然恢复 | 机器人端 `:1462` 已有 | 丢失后前 N 秒，不做任何动作（很多抖动会自愈） |
| L1 | 重播最后可信位姿 | **已实现**（`app.py:584`） | L0 超时 |
| L2 | **主动重定位 12 候选搜索** | `ros_adapter.py:873` **已实现但未接入自动流程** | L1 连续失败 |
| L3 | 全局重定位 | `use_global_localization_init` 已有；BT 插件闲置见 §2.3 | L2 失败 |
| L4 | RTK 播种 | `/localization/seed_from_rtk` **已实现** | 室外且 RTK quality 为 fixed |
| L5 | 回退到最近强定位关键帧 | **未实现** | L2/L3 失败且有可用轨迹历史 |
| L6 | 停机 + 告警 + 等人工 | 当前是隐式的（永久 paused） | 上述全失败，**必须产告警** |

**注意 L2 是最大的现成红利**：`active_relocalize` 的 12 候选搜索（8 偏航 + 4 平移）已完整实现且全程不动机器人（`motion_commanded: False`，安全），但目前**只能靠人工从前端点按钮触发**。接进自动恢复流程的工作量很小。

**L5 的实现思路**：`local_store.py:211-220` 已按 `map_id + map_version` 持久化最后可信位姿；轨迹历史在 `TrajectoryPoint` 表里带 `localization_status` 字段。可以据此找到最近一个定位质量良好的位置，引导机器人退回去重新定位。但这一级涉及**自主运动**，安全约束需要仔细设计，优先级最低。

---

## 4. 优先级与实施建议

| 优先级 | 项 | 工作量 | 理由 |
| --- | --- | --- | --- |
| ~~**P0**~~ 已实现 | §2.2 定位丢失产告警 | 小 | 现在运维完全看不见定位问题，可观测性缺失是最大风险 |
| ~~**P0**~~ 已实现 | §2.1 恢复轮次上限 + 状态上报 | 小 | 永久 paused 且无人知晓 |
| ~~**P1**~~ 已实现 | §2.3 行为树接入 `ReinitializeGlobalLocalization` | ~~**极小**（改 XML）~~ **中**：还得补服务端 | 现成能力闲置；但缺服务端时只改 XML 会让整棵树加载失败，见该节 |
| **P1** | §3 L2 主动重定位接入自动流程 | 小 | 现成能力，只差编排 |
| **P1** | §2.4 `ScanMatchingStatus` 降级显式化 | 小 | 防止静默失效 |
| **P2** | §2.5 恢复后清代价地图 | 中 | 需要新增适配器接口 |
| **P2** | §2.6 重启接管 SOP | 小（文档） | 不改代码原则 |
| **P3** | §3 L5 回退到强定位关键帧 | 大 | 涉及自主运动，安全设计复杂 |

**实施顺序建议**：先做 [IMU 漂移整改](IMU_DRIFT_DIAGNOSIS_AND_REMEDIATION_PLAN.md) 的 P0 项，再回来评估本文。如果零偏修复后定位丢失频率大幅下降，P2/P3 的项可以推迟甚至不做。

### 4.1 已落地清单（2026-08-25）

| 文件 | 改动 |
| --- | --- |
| `edge-agent/roamerx_edge/app.py` | 三处 `emit_system_alert`；恢复轮次上限与升级；恢复状态逐轮上报 |
| `edge-agent/roamerx_edge/config.py` | `SafetyConfig.localization_recovery_max_cycles` |
| `edge-agent/roamerx_edge/telemetry_collector.py` | `on_localization_recovery()` + 状态快照 `localization.recovery` |
| `robot/src/localization/.../localization_nodelet.cpp` | 新增 `/reinitialize_global_localization` 服务端 |
| `.../behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml` | 恢复子树接入重定位节点 |
| `edge-agent/tests/test_app_localization_recovery.py` | 3 → 7 个用例 |

**未做**：§2.4 / §2.5 / §2.6 仍是方案；§3 的 L2 主动重定位仍未接入自动流程。
**未验证**：§5 的第 1 项（人为制造定位丢失）需要重启定位栈，尚未执行。

---

## 5. 验证方法

1. **人为制造定位丢失**：遮挡雷达、或在长走廊等自相似环境跑任务，观察：
   - 是否产生 `alert.event`（改前：不会）
   - 平台 `InspectionEvent` 是否有记录（改前：没有）
   - 恢复轮次是否可见（改前：不可见）
2. **离线回放**：用 `runtime/nx-edge/data/rosbags/mapping/` 下的 59 个包（其中含定位不良片段）跑 `replay_navigation_inputs.py`，验证检测阈值是否合理。
3. **现有测试**：`edge-agent/tests/test_app_localization_recovery.py` 已覆盖恢复状态机（3 个用例：快速重试后开新周期、无任务不起恢复、发散告警只发一次）。任何改动都需扩展该测试。
4. **统计口径**：改前/改后对比同一路线的 `task.pausing` + `LOCALIZATION_LOST` 次数、平均恢复耗时、需人工干预的比例。前端 `taskMapState.js:36-60` 的丢失点标记已可直接用于对比。

---

## 6. 相关文档

- [IMU 漂移诊断与整改方案](IMU_DRIFT_DIAGNOSIS_AND_REMEDIATION_PLAN.md) —— 降低丢失频率，**建议先做**
- [地图管线方案](MAP_PIPELINE_MCAP_FOXGLOVE_AND_ROUTE_PREVIEW_PLAN.md) —— §4.3 弱定位区段预演与本文的丢失数据同源
- [导航传感器说明](NAVIGATION_SENSOR_DESCRIPTION.md) —— 源仲裁与 TF 链路
- [路径规划 RTK 与地图管理执行计划](ROUTE_PLANNER_RTK_AND_MAPPING_EXECUTION_PLAN.md) —— 定位初始化的前端流程
