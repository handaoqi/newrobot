# 地图管线方案：mcap 录制、Foxglove 离线优化与路线几何预演

制定日期：2026-08-25
状态：方案，**尚未落任何代码改动**

---

## 0. 现状纠正（重要）

用户需求描述为「地图管理里使用 rosbag 录制数据」，但勘察结果是：

> **rosbag 录制早已完整实现并在生产使用中。** 磁盘上已有 **59 个建图包，共 34 GB**。

因此本方案的真实问题不是"实现录制"，而是四件事：**格式切换到 mcap、补上完全缺失的生命周期管理、接入 Foxglove 做离线优化、以及平台端路线几何预演**。

### 已有的录制能力

| 组件 | 位置 | 说明 |
| --- | --- | --- |
| 建图录制脚本 | `robot/script/robot/mapping_rosbag.sh` | `start` / `stop` / `status`，返回单行 JSON |
| 任务录制脚本 | `robot/script/robot/navigation_rosbag.sh` | 复用上者，换 `BAG_ROOT` 并追加 5 个导航话题 |
| Edge 调用方（建图） | `edge-agent/roamerx_edge/mapping_adapter.py:869-941` | 覆盖 RTK 准备 → 预热 → 正式建图全程 |
| Edge 调用方（任务） | `edge-agent/roamerx_edge/rosbag_recorder.py` + `task_executor.py:1143` | 标签 `task_<execution_id前8位>` |
| 包元数据解析 | `edge-agent/roamerx_edge/recording_manifest.py` | 产出 `recording_manifest.yaml`，含缺失话题校验与 RTK 有效区间 |
| 前端开关 | `platform/frontend/src/views/MapsPage.vue:72` | `record_rosbag: true` 默认勾选，实时显示时长/大小/路径 |

固定录制的 9 个话题（`mapping_rosbag.sh:93-103`）：

```
/front_lidar  /front_lidar/imu  /fix  /rtk_pvh  /rtk/ntrip_status
/odom/localization_odom  /slam_odom  /tf  /tf_static
```

`ROSBAG_EXTRA_TOPICS` 环境变量可临时追加。

### 已有的离线回放能力

| 工具 | 说明 |
| --- | --- |
| `robot/script/robot/replay_mapping_rosbag.sh` | 隔离 `ROS_DOMAIN_ID=77`，用 `-p storage.data_path:=$OUTPUT_ROOT` 重跑 mapping 二进制 + `ros2 bag play`，**可调 rate / acc_cov / gyr_cov** |
| `robot/script/robot/replay_navigation_inputs.py` | `rosbag2_py.SequentialReader`，按 live 时序重发 `/front_lidar`、`/front_lidar/imu`、`/odom/mc_odom` |

这两个都是开发者本地 CLI，**未被 Edge Agent 或平台调用**——但它们正是本方案离线优化闭环的现成地基。

---

## 1. 子方案 A：录制格式切换到 mcap + 生命周期管理

### 1.1 磁盘现状（紧急）

```
runtime/nx-edge/data/rosbags/mapping/     34 GB / 59 个包
runtime/nx-edge/data/jszr/map/            19 GB / 140 个地图会话
整盘                                       233 GB，已用 171 GB（78%），剩余 50 GB
```

**当前没有任何轮转或清理机制。** 唯一的保护是 `mapping_rosbag.sh:76-80` 的启动前拒绝：

```bash
MIN_FREE_GB="${MIN_FREE_GB:-10}"
if [ "${available_kb}" -lt "${required_kb}" ]; then
  echo "ERROR: less than ${MIN_FREE_GB}GB free under ${BAG_ROOT}" >&2; exit 2
fi
```

即：**一直录到只剩 10 GB 才拒绝开新包，且从不删旧包**。按单包平均 576 MB、每次建图约 2 分钟估算，剩余 50 GB 约可再录 85 次建图，之后建图功能会直接拒绝启动。没有 systemd timer、cron 或任何 `find -mtime` 清理脚本。

**这是本方案里最紧急的一项，优先级高于 mcap 切换本身。**

### 1.2 切换到 mcap

前置条件**已满足**（已实机验证）：

```
/opt/ros/humble/lib/librosbag2_storage_mcap.so           存在
ros-humble-rosbag2-storage-mcap  0.15.16-1jammy          已安装
ros-humble-mcap-vendor           0.15.16-1jammy          已安装
```

当前 `mapping_rosbag.sh:110` 的 `ros2 bag record` **不带 `-s` 参数**，默认走 sqlite3，产出 `.db3`。

**改动点**

1. `robot/script/robot/mapping_rosbag.sh` — `ros2 bag record` 加 `-s mcap`。建议做成可配置（`ROSBAG_STORAGE=${ROSBAG_STORAGE:-mcap}`），便于回退。
2. `edge-agent/roamerx_edge/recording_manifest.py:84` — 当前直接用 `sqlite3.connect(f"file:{db}?mode=ro", uri=True)` 打开 `.db3` 查 `/fix` 时间戳区间（`_rtk_valid_intervals`）。这段必须改为走 `rosbag2_py.SequentialReader`，才能同时支持两种格式。
3. **存量兼容**：磁盘上 59 个 `.db3` 包不能失效。`recording_manifest.py` 需要按 `metadata.yaml` 里的 `storage_identifier` 分派，`.db3` 走旧路径、`.mcap` 走新路径。
4. `robot/script/robot/replay_mapping_rosbag.sh` 和 `replay_navigation_inputs.py`（后者硬编码了 `storage_id="sqlite3"`）需同步适配。

**收益**：mcap 是 Foxglove 的原生格式，可直接拖进 Foxglove 打开，无需转换；压缩率和随机读性能均优于 sqlite3，对 34 GB 的存量问题也有帮助。

**风险**：低。`.gitignore:24-26` 已经同时排除 `*.bag` / `*.db3` / `*.mcap`，不会误提交。

### 1.3 生命周期管理（2026-08-25 已实现）

> **实现状态**：本节的建议已落成代码，见 §1.3.1。以下"建议要素"保留为需求原文，实现与它的三处偏离在 §1.3.2 单独说明。

需要新增一套清理策略，建议要素：

- **保留策略**：按数量 + 按时间 + 按空间三重上限（例如保留最近 N 个 或 M 天内，且总量不超过 X GB）。
- **豁免规则**：被 `map_manifest.json` 的 `raw_recording` 字段引用的包不能删——这是地图与原始数据的溯源链接。当前最新地图的 manifest 里就有：
  ```json
  "raw_recording": "/home/dogrobot/runtime/nx-edge/data/rosbags/mapping/20260825_161739_V1"
  ```
- **触发方式**：systemd timer 或 Edge Agent 的周期任务（`app.py` 已有 `_outbox_loop` 等多个循环可参照）。
- **可观测性**：清理动作与当前占用上报到平台，避免"静默删数据"。
- **同样适用于地图目录**：140 个会话 19 GB，同样无清理机制。

#### 1.3.1 实现落点

| 文件 | 作用 |
| --- | --- |
| `robot/script/robot/prune_runtime_storage.py` | 独立保留策略工具。默认 **dry run**，只有 `--apply` 才真删。输出 `roamerx.storage-retention.v1` JSON 报告（当前占用 + 每个 root 的会话数/总量/受保护项/删除项/失败项） |
| `robot/script/robot/test_prune_runtime_storage.py` | 11 个单元测试，覆盖全部豁免规则与选择语义。与 `test_replay_mapping_trajectory.py` 同为同目录同级测试，`scripts/test_agents.sh` 不收集它，需 `cd robot/script/robot && python3 -m pytest test_prune_runtime_storage.py` |
| `robot/script/robot/mapping_rosbag.sh` | 新增 `reclaim_space_if_needed`（`start` 时触发）与 `prune [--apply]` 子命令 |
| `edge-agent/roamerx_edge/system_telemetry.py` | 新增 `_poll_storage()`，纳入既有 `poll()` |
| `edge-agent/roamerx_edge/telemetry_collector.py` | 新增 `on_storage()` 与状态快照里的 `storage` 对象 |
| `edge-agent/roamerx_edge/config.py` | `TelemetryConfig` 新增 `storage_probe_path` / `storage_retention_report_glob` |

**默认上限**：bag 保留最近 10 个、30 天、20 GiB；地图保留最近 20 个、60 天、12 GiB。全部可由命令行覆盖。

**触发方式**：`mapping_rosbag.sh start` 在**空间检查之前**调用 `reclaim_space_if_needed`。只有剩余空间低于 `PRUNE_TRIGGER_GB`（默认 `MIN_FREE_GB * 2` = 20 GB）才会真删——**稳态录制永远不删任何东西**。这条路径要解决的正是"慢速泄漏变成建图功能硬性停摆"：以前低于 10 GB 直接 `exit 2`，现在先回收最旧的无引用包。清理失败不阻断录制，空间闸门仍是唯一权威。
`navigation_rosbag.sh` 只是 `exec` 到同一脚本并覆盖 `BAG_ROOT`，自动继承该行为、作用域限于自己的 root。

**可观测性**：脚本把报告写到 `${BAG_ROOT}/.retention.json`（放在包旁边而非 `STATE_DIR`——后者在 `/tmp` 下且按 uid 分目录，Edge Agent 找不到）。Edge Agent 的 `_poll_storage` 按 `storage_retention_report_glob` 取最新一份，连同 statvfs 占用一起进 `status.storage`：

```json
"storage": {
  "path": "/home/dogrobot/runtime/nx-edge/data",
  "available": true, "free_gib": 47.8, "used_percent": 78.4,
  "last_retention": {
    "generated_at_unix": 1787667912.0, "applied": true,
    "reclaimed_bytes": 16404021248, "failed_count": 0,
    "deleted_count": 11,
    "deleted": [{"path": "...", "size_bytes": ..., "reason": "beyond 20GiB budget"}]
  }
}
```
`_poll_storage` **不遍历** bag/地图目录——那是每个探测周期几百次 stat。占用来自 statvfs，逐会话明细来自脚本已经写好的报告。删除清单截断到 10 条（`deleted_count` 保留真实条数），避免一次大清理把每条状态消息都变成删除清单。

**豁免规则（四条，全部有测试）**：

1. 被任一 `map_manifest.json` 的 `raw_recording` 引用的包。**manifest 读不出来时同样豁免**——读不出正是最强的"别删"信号。
2. 地图根下任何 symlink 解析进去的会话目录（`mapping_adapter._refresh_current_map_links` 写的当前地图链接）。
3. `--exclude` 传入的路径。脚本自动把**正在录的包**传进去（它太新，还不可能被任何 manifest 引用）。
4. 目录名不匹配 `^\d{8}_\d{6}` 的一切。这条是本轮勘察加的：地图根下有 `manual_edits/`，里面按平台 map id 存放人工编辑后的地图（`map_101_legacy-mapdata-101` …，共 19 个），**没有任何录制能重新生成它**。它此前只是靠 mtime 落在 `keep_recent` 窗口里侥幸没被选中。

#### 1.3.2 与建议要素的三处偏离

1. **三重上限是"或"不是"且"。** 最初按字面实现成"同时超过全部上限才删"，实测选中 0 字节：磁盘已经 78% 满，但没有任何一个包超过 30 天，尺寸预算永远咬不动。改成"最近 `keep_recent` 个无条件保留，之后年龄超限**或**总量超预算即可回收"后，同一次 dry run 选出 19.3 GiB。近期数据由 `keep_recent` 保护，不需要靠这个"且"来兜底。
2. **自动路径带 `--skip-maps`，只清 bag。** 地图会话可以被云平台按名字直接激活（`map_activation_adapter._resolve_source_dir` 会解析 `map_dir/<map_version>`），删掉一个正在平台地图列表里的会话是对外可见且难以撤销的。地图清理保留在 `mapping_rosbag.sh prune --apply`（手动，需要操作者对着平台地图列表决定），不进自动触发。
3. **未安装 systemd timer。** 建议里 timer 与周期任务二选一；已实现的是"录制启动时按需触发 + 平台可见"。`prune` 子命令就是给 timer 预留的入口，需要时再加 unit——本轮不装、不 `daemon-reload`。

#### 1.3.3 首次 dry run 结果（2026-08-25，未执行 `--apply`）

```
free 47.8 GiB -> 若执行可回收 19.3 GiB（约 67 GiB）
rosbags/mapping     sessions=64  protected=22  would_delete=11  reclaim=15.3 GiB
rosbags/navigation  sessions=2   protected=0   would_delete=0   reclaim=0.0 GiB
jszr/map            sessions=143 protected=2   would_delete=17  reclaim=4.0 GiB
```
22 个受保护的 mapping 包全部是某张地图的 `raw_recording`；地图侧 2 个受保护项分别是当前地图 symlink 目标和 `manual_edits/`。**本轮只跑 dry run，没有删除任何数据。**

---

## 2. 子方案 B：Foxglove 接入与地图离线优化

### 2.1 前置条件已满足

```
ros-humble-foxglove-bridge       3.4.3-2jammy      已安装
ros-humble-rosbridge-suite       2.0.5-1jammy      已安装
```

但**全仓库零集成**：没有 launch、没有 systemd unit、没有配置、无 8765 端口引用、前端无 `@foxglove/*` 依赖。唯一提及在 `robot/docs/read_预操作.md:11`，是一条手工命令：

```bash
export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 address:=192.168.234.234
```

### 2.2 为什么离线优化是刚需

最新地图的 `map_manifest.json` 显示：

```json
"keyframe_count": 85,
"loop_closure_count": 0,
"loop_status": "no_valid_loop",
"trajectory_source": "raw"
```

**回环检测从未成功触发过**，位姿图优化实际一直退化为直接输出原始 LIO 轨迹。而回环基础设施是齐全的：

- `edge-agent/roamerx_edge/map_loop_closure.py`（720 行）—— 纯 Python Scan-Context 实现，参数 `RINGS=20, SECTORS=60, MAX_RADIUS_M=80.0, CANDIDATE_TOP_K=5, MIN_KEYFRAME_GAP=30, YAW_SEARCH_STEPS=60`
- `robot/src/slam/src/src/global_factor_graph.cpp` —— GTSAM 位姿图，自定义 `LeverArmPositionFactor` / `HeadingFactor` + Huber 鲁棒核
- ROS 服务 `/slam/global_optimize`（`std_srvs/srv/Trigger`），由 `mapping_adapter.py:1333` 调用

**排查方向**（需要 Foxglove 的可视化才好定位）：`MIN_KEYFRAME_GAP=30` 对 85 个关键帧是否过严、几何验证阈值是否过紧、还是描述子本身在当前场景下不可区分。

### 2.3 离线优化闭环设计

```
存量 .db3 / 新录 .mcap
        ↓  replay_mapping_rosbag.sh（隔离 ROS_DOMAIN_ID=77，不干扰生产）
   离线重跑 mapping 二进制 → 新的 keyframes/ + scan_context/
        ↓  foxglove_bridge（同一隔离 domain，端口 8765）
   Foxglove 网页端：点云 / TF / 轨迹 / 回环候选可视化
        ↓  人工确认回环、圈选动态障碍
   /slam/global_optimize  +  map_loop_closure.py
        ↓
   优化后的 map.pcd / map.pgm / trajectory_optimized.csv
```

**关键设计点**

1. **隔离**：离线回放必须用独立 `ROS_DOMAIN_ID`（现有脚本已用 77），Foxglove bridge 也起在该 domain，绝不能干扰生产 domain 24。
2. **失败降级已有先例**：`map_package_finalize.py:52` 的 `finalize_loop_closure` 失败时保留 raw 图，`map_loop_closure.py:182-184` 明确不做 SE2 warp（注释："False-loop SE2 rebuilds previously warped map.pcd"）。离线优化必须沿用同样的保守原则——**优化失败绝不破坏已有地图**。
3. **人工确认动态障碍**：`slam/config/config.yaml:73-80` 的 `dynamic_filter.enable: false`，注释说明动态物体在"平台的人工地图清理流程"中处理。前端已有 2D Canvas 擦除（`MapsPage.vue:1451-1480` → `POST /maps/{id}/manual-clean/`），但那只能擦 PGM 栅格，**擦不了 PCD 点云**。Foxglove 三维视图正好补上这一块。

**改动点**

- 新增 `foxglove_bridge` 的 launch + systemd unit（参照 `edge-agent/systemd/roamerx-*.service` 的既有写法），支持按需启停而非常驻——8 核已经很紧张（见 IMU 诊断文档 §1.5）。
- 把 `replay_mapping_rosbag.sh` 从开发者 CLI 提升为 Edge Agent 可调用的能力，并在平台侧加"离线重优化"入口。
- 网络暴露：bridge 监听 `ws://<机器人IP>:8765`，需明确访问控制策略。

---

## 3. 子方案 C：嵌入平台前端

### 3.1 现状

`platform/frontend` **完全没有三维能力**：

```json
dependencies: echarts, hls.js, mpegts.js, vue, vue-echarts, vue-router
devDependencies: @playwright/test, @vitejs/plugin-vue, vite
```

无 three.js / roslibjs / potree / deck.gl / @foxglove/*。

现有地图渲染方式：
- 后端 `views.py:2423 MapDataPreviewView` 用 PIL 把 `map.pgm` 转 PNG（`max_size=1200`）
- 前端 `<img>` 显示 + 绝对定位 SVG 覆盖层画轨迹/路径/朝向（`RoutePlannerPage.vue:2420-2480`）
- **`map.pcd` 前端完全不渲染**，只作为文件参与打包、下载、激活

### 3.2 两条路径

| | iframe 嵌入 | `@foxglove/*` 组件 |
| --- | --- | --- |
| 工作量 | 小 | 大 |
| 与现有 Vue 状态联动 | 弱（需 postMessage） | 强 |
| 鉴权 | 需处理跨域与 token 传递 | 走前端现有 `api.js` 体系 |
| 视觉一致性 | Foxglove 自带 UI，与平台风格割裂 | 可定制 |
| 升级维护 | 跟随 Foxglove 版本 | 需自行维护 |

**建议**：先做 iframe 嵌入验证价值，确认运维确实用得上之后再评估组件化。集成位置：`MapsPage.vue`（地图详情页加"三维查看"入口）和 `RoutePlannerPage.vue`（规划时对照点云）。

**注意**：Foxglove bridge 连的是**机器人本机**的 ROS domain，而平台前端跑在云端。需要明确网络路径——是通过已有的 `roamerx-cloud-tunnel.service` 隧道，还是仅限内网访问。这一点必须先定，否则集成方式会推倒重来。

---

## 4. 子方案 D：巡检路线几何预演

按确认，**不做物理仿真**（Gazebo / Isaac 需要独立 x86 GPU 主机，Jetson 上跑不动），只做平台端纯几何推演。

### 4.1 现状：完全没有任何预演能力

全量搜索 `dry.?run|simulat|rehears|预演|仿真|gazebo|isaac|mock_mode` 在 `edge-agent`、`platform/backend/monitoring`、`platform/frontend/src`、`robot/script` **均无命中**。

最接近的三个都不是任务预演：
- `mapping.slam_start` 的 SLAM warmup（`mapping_adapter.py:291`）—— 跑真实 SLAM 但不记正式关键帧
- `OriginLockMonitor.prepare()`（`origin_lock.py:170`）—— RTK 实时预览
- `replay_*.sh` —— 离线 rosbag 重放（开发者 CLI）

### 4.2 预演内容

在 `RoutePlannerPage.vue`（4493 行，全仓最大文件）与后端之间加一层校验，对已有地图做纯几何推演：

| 检查项 | 数据来源 |
| --- | --- |
| 路径可达性 | `map.pgm` 栅格 + 后端已有的 `_read_pgm_dimensions` / `_parse_simple_map_yaml` |
| 碰撞膨胀 | `map.pgm` + 机器人尺寸；`pcd2pgm` 已按 `thre_z_min: 0.05 / thre_z_max: 0.75` 投影出可碰撞层 |
| 点位可见性 | `keyframes/keyframes.csv` + `keyframe_visibility_filter.py`（已存在，默认关闭） |
| 预计耗时 | 路径长度 + 各点 `dwell_seconds`（`task_service.py:73-94` 的规范化结构里已有） |
| **弱定位区段高亮** | 见下 |
| 地图约束校验 | **复用现成的** `map_coordinate.py` 的 `constraints_from_manifest` / `validate_route_against_map` |

### 4.3 弱定位区段：数据全都是现成的

这是预演里最有价值的一项，且不需要新采数据：

1. **历史定位丢失点** —— `platform/frontend/src/services/taskMapState.js:36-60` 的 `buildLocalizationLossMarkers` 已经在从 `event.reason_code === 'LOCALIZATION_LOST' && event.event_type === 'task.pausing'` 提取丢失位置并画在地图上。把历史累积起来就是弱定位热力图。
2. **历史定位质量** —— `TrajectoryPoint.localization_status` 字段已落库（`message_handlers.py:566`），每条轨迹点都带定位状态。
3. **关键帧密度** —— `keyframes/keyframes.csv`，关键帧稀疏的区段天然是 NDT 匹配薄弱区。
4. **Scan-Context 可区分度** —— `scan_context/descriptors.bin` + `index.json`，描述子相似度高的区域意味着环境自相似（长走廊、空旷广场），是重定位失败高发区。

**建议实现方式**：后端新增一个只读的路线校验接口，输入 route + map，输出结构化的告警列表；前端在规划器里以覆盖层形式呈现。复用后端已有的 `task_service.py:38 normalize_waypoints` 保证坐标语义一致。

**明确不做**：不模拟机器人动力学、不模拟传感器、不预测实际定位结果。这是**静态几何检查 + 历史数据统计**，用于"下发任务前发现明显问题"，不是仿真器。

---

## 5. 优先级建议

| 优先级 | 项 | 理由 |
| --- | --- | --- |
| ~~**P0**~~ 已实现 | §1.3 rosbag / 地图生命周期管理 | 磁盘 78% 已用，仅剩 50 GB 且无任何清理，会硬性阻断建图功能。2026-08-25 落成，见 §1.3.1；尚未执行 `--apply`，磁盘占用仍是现状 |
| **P1** | §1.2 mcap 切换 | Foxglove 接入的前置；对存量 34 GB 也有压缩收益 |
| **P1** | §2 Foxglove + 离线优化 | 直接服务于「回环从未成功」这个已确认缺陷 |
| **P2** | §4 路线几何预演 | 纯平台侧，不阻塞其他项，可并行 |
| **P2** | §3 前端嵌入 | 依赖 §2 完成，且需先定网络路径 |

---

## 6. 相关文档

- [IMU 漂移诊断与整改方案](IMU_DRIFT_DIAGNOSIS_AND_REMEDIATION_PLAN.md) —— 本文的离线回放基础设施是该文验证方法的载体
- [定位丢失恢复与自愈方案](LOCALIZATION_SELF_HEALING_PLAN.md) —— §4.3 的弱定位区段数据与该文的丢失检测同源
- [SLAM 采集与世界位姿计划](SLAM_DATA_CAPTURE_AND_WORLD_POSE_PLAN.md) —— 9 个录制话题与 Scan-Context 参数的原始定义
- [地图管理 RTK、里程计与诊断录制计划](MAP_ORIGIN_RTK_ODOM_DIAGNOSTIC_PLAN.md) —— 已提出"同步诊断 rosbag"，与 §1 有重叠，实施时需合并考虑
- [地图原点锁定 SOP](MAP_ORIGIN_LOCK_SOP_PLAN.md) —— 注意该文 `:840` 描述 bag 随 ZIP 上传到 `diagnostics/rosbag/`，但代码 `mapping_adapter.py:1628-1632` 已改为只留本地并在 metadata 里放 `local_rosbag_dir` 指针，**文档与代码存在漂移，需修正**
