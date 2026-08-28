# 工作区未提交改动梳理（2026-08-25）

分支：`docs/frontend-docs-sync`，基线 `31c2068`。快照时间 2026-08-25 17:0x。

`git status --porcelain` 共 **117 项**：69 项修改（M）、6 项删除（D）、42 项未跟踪（??）。
跟踪文件净变化：**75 files changed, +3346 / -938**（已扣除本次梳理对 `docs/README.md` 的索引更新）。

> ⚠️ **工作区在本次梳理期间仍在被改动。** 快照过程中 `robot/src/slam/src/src/mapping_alg.cpp` 从 `+189/-29` 变为 `+207/-29`（+18 行），同时后台有 `colcon build --packages-select robot_slam` 在编译。说明有另一个会话正在并行编辑 SLAM 代码。
> **提交前请重新执行 `git status` 与 `git diff --numstat HEAD` 核对**，本文档的行数仅供分类参考，不可作为提交依据。

本文档只做分类与处置建议，不代为删除、暂存或提交。

---

## 1. 规模分布

| 模块 | 文件数 | 增 | 删 |
| --- | ---: | ---: | ---: |
| `edge-agent` | 22 | +2026 | -283 |
| `platform/frontend` | 25 | +523 | -300 |
| `robot` | 12 | +433 | -74 |
| `platform/backend` | 6 | +258 | -14 |
| `docs` | 8 | +51 | -104 |
| `platform/scripts` | 1 | +61 | 0 |
| `yuwang.skill` | 1 | 0 | -164 |

改动最大的十个文件：

| 文件 | +/- |
| --- | ---: |
| `edge-agent/roamerx_edge/origin_lock.py` | +400 / -66 |
| `edge-agent/roamerx_edge/mapping_adapter.py` | +347 / -96 |
| `edge-agent/tests/test_mapping_adapter.py` | +291 / -11 |
| `edge-agent/roamerx_edge/map_loop_closure.py` | +209 / -38 |
| `platform/frontend/src/views/MapsPage.vue` | +190 / -15 |
| `robot/src/slam/src/src/mapping_alg.cpp` | +207 / -29 ⚠️ 正在被并行编辑 |
| `edge-agent/tests/test_origin_lock.py` | +148 / -21 |
| `edge-agent/roamerx_edge/ros_adapter.py` | +133 / -5 |
| `edge-agent/tests/test_task_state_machine.py` | +102 / -0 |
| `robot/src/navigation/.../vel_cmd_udp_publisher.cpp` | +82 / -30 |

---

## 2. 建议的提交拆分

这批改动不是一个主题，混在一起提交会让 `git log` 和后续回滚都很难用。建议拆成四个提交。

### C1 — 建图原点锁定、回环检测与地图优化

**最大的一簇**，占 edge-agent 全部改动和 robot 侧 SLAM 改动。对应 `docs/MAP_ORIGIN_LOCK_SOP_PLAN.md` 和 `docs/SLAM_DATA_CAPTURE_AND_WORLD_POSE_PLAN.md` 里已规划的 P0–P3。

Edge Agent（修改）：
```
edge-agent/roamerx_edge/origin_lock.py            +400/-66
edge-agent/roamerx_edge/mapping_adapter.py        +347/-96
edge-agent/roamerx_edge/map_loop_closure.py       +209/-38
edge-agent/roamerx_edge/ros_adapter.py            +133/-5
edge-agent/roamerx_edge/app.py                     +77/-1
edge-agent/roamerx_edge/task_executor.py           +72/-26
edge-agent/roamerx_edge/{config,command_processor,map_package_finalize,
                         media_client,navigation_stack_adapter,
                         recording_manifest,alert_bridge,system_telemetry}.py
edge-agent/config.example.yaml
```
Edge Agent（新增）：
```
edge-agent/roamerx_edge/map_optimization_summary.py
edge-agent/roamerx_edge/rtk_origin.py
edge-agent/tests/test_map_loop_closure.py
edge-agent/tests/test_map_optimization_summary.py
edge-agent/tests/test_media_client.py
edge-agent/tests/test_navigation_stack_adapter.py
edge-agent/tests/test_rtk_origin.py
```
Robot 侧：
```
robot/src/slam/src/src/mapping_alg.cpp             +207/-29  ⚠️ 并行编辑中
robot/src/slam/src/src/global_factor_graph.cpp
robot/src/slam/src/include/{mapping_alg,global_factor_graph}.h
robot/src/slam/src/config/config.yaml
robot/script/robot/mapping_rosbag.sh
robot/script/rtk_ntrip_bridge.py
robot/script/robot/replay_mapping_trajectory.py        （新增）
robot/script/robot/test_replay_mapping_trajectory.py   （新增）
```
平台侧：
```
platform/frontend/src/views/MapsPage.vue           +190/-15
platform/frontend/src/utils/mappingWorkflowState.js
platform/backend/monitoring/{serializers,views,message_handlers}.py
platform/backend/monitoring/test_{mapping_workflow,p0_handlers,force_delete_and_maps}.py
platform/backend/monitoring/test_summary_apis.py       （新增）
```
配套测试（修改）：`edge-agent/tests/test_{mapping_adapter,origin_lock,map_package_finalize,command_processor,task_state_machine,system_telemetry,app_localization_recovery}.py`

> 注意：`MapsPage.vue` 的 +190 属于建图工作流（状态步进、`record_rosbag` 复选框、SLAM 发散提示），归 C1 而非前端视觉簇。

### C2 — 前端平板视觉同步与视频宿主抽象移除

删除一层共享视频宿主抽象，并同步平板端视觉基线。

删除：
```
platform/frontend/src/components/SharedLiveVideoHost.vue      -46
platform/frontend/src/composables/useSharedVideoStream.js     -60
platform/frontend/tests/sharedVideoStream.test.js             -42
docs/FRONTEND_MONITORING_VIDEO_ARCHITECTURE.md                （随之作废）
docs/FRONTEND_MONITORING_VIDEO_ARCHITECTURE.docx
```
修改：
```
platform/frontend/src/components/LiveVideoPlayer.vue           +7/-...
platform/frontend/src/style.css                               +78 净
platform/frontend/src/views/DashboardLayout.vue                +53 净
platform/frontend/src/views/DashboardOverview.vue              +82 净
platform/frontend/src/views/GuardDutyPage.vue                  +62 净
platform/frontend/tests/tablet/tablet-portrait.spec.js         +83/-2
platform/frontend/{playwright.config.js,nginx.conf,.env.example,README.md}
```
新增：
```
platform/frontend/tests/tablet/video-player-mount.spec.js
platform/frontend/tests/tablet/__screenshots__/tablet-2000-landscape-chromium/
```
截图基线更新（二进制）：`android-800-chromium`、`ipad-768-webkit`、`ipad-834-webkit` 各三张。

> ⚠️ 这个提交删掉了 `FRONTEND_MONITORING_VIDEO_ARCHITECTURE.md`，但 `docs/README.md:12` 和 `:34` 仍然链接它。提交前必须同步修掉这两个死链，否则违反本仓库文档规则第 5 条。

### C3 — 遥控姿态状态机重构

把布尔标志 `low_posture_lock_` 换成显式枚举 `RequestedPosture`，姿态语义更清晰。

```
robot/src/navigation/src/robot_navigo/src/remote_velocity_mode.hpp   （新增，枚举定义）
robot/src/navigation/src/robot_navigo/src/vel_cmd_udp_publisher.cpp  +82/-30
robot/src/navigation/src/robot_navigo/test/                          （新增测试目录）
robot/src/navigation/src/robot_navigo/{CMakeLists.txt,package.xml}
robot/script/robot/{start_navigation_real.sh,charge_pile_arbiter.sh}
platform/frontend/src/views/RemoteControlPage.vue                    +47/-35
```

### C4 — 部署与文档

```
platform/scripts/deploy_cloud_platform.sh    +61
platform/deploy/nginx/                       （新增）
docs/DEPLOY_CONSOLIDATION_AND_NGINX_FIRST_DEPLOY_PLAN.md   （新增）
docs/FAST_LIO_SLAM_ENU_GPS_IMU_UNIFIED_PLAN.md             （新增）
docs/MAP_ORIGIN_RTK_ODOM_DIAGNOSTIC_PLAN.md                （新增）
docs/ROUTE_PLANNER_RTK_AND_MAPPING_EXECUTION_PLAN.md       （新增）
docs/前端总体架构.md + 四份 .docx                            （新增）
docs/{README,ROAMERX_SYSTEM_ARCHITECTURE,INSPECTION_TASK_ROUTE_ARCHITECTURE,
      MAP_ORIGIN_LOCK_SOP_PLAN,SLAM_DATA_CAPTURE_AND_WORLD_POSE_*}.md   （修改）
yuwang.skill/SKILL.md   （删除 -164，确认是否有意）
```

---

## 3. 需要单独处理的两类文件

### 3.1 家目录污染（优先级最高）

**Git 仓库根目录就是 `/home/dogrobot`**，即开发用户的家目录。而 `.gitignore` 只有 29 行，针对的全是构建产物和密钥后缀，**没有任何一条覆盖用户级点文件**。结果是用户日常使用产生的状态文件全部出现在未跟踪列表里：

```
.Xauthority                      X11 认证 cookie
.bash_history                    shell 历史
.claude.json                     Claude Code 配置
.claude/                         会话记录、工具输出缓存
.codex/                          Codex 会话记录
.codex-dev-agent/
.npm/                            npm 缓存
.dbus/
.viminfo
.bashrc.before-env-fix-20260821
.bashrc.before-sync-20260821
.profile.before-env-fix-20260821
.codex-robot-write-test
```

**风险**：`.bash_history`、`.claude.json`、`.claude/`、`.codex/` 都可能含有主机名、内网 IP、路径、token 片段或 NTRIP 凭据。一次 `git add -A` 就会把它们提交上去，直接违反本仓库文档规则第 4 条。注意 `.ssh/` 目前恰好因为权限（`drwx------`）没被列出，但同样没有 ignore 规则保护。

**建议**：在 `.gitignore` 补一段用户级排除（本轮不改文件，仅给出建议内容）：

```gitignore
# 仓库根即家目录，排除用户级状态文件
/.Xauthority
/.bash_history
/.viminfo
/.claude/
/.claude.json
/.codex/
/.codex-dev-agent/
/.cursor/
/.npm/
/.dbus/
/.nv/
/.cache/
/.config/
/.local/
/.ros/
/.ssh/
/.vscode-server/
/.pytest_cache/
/.wget-hsts
/*.before-*
```

同时确认 `.bashrc` / `bashrc` / `.profile` 这三个**已被跟踪**的文件是否确实应该纳入版本管理——如果是部署基线的一部分则保留，否则应 `git rm --cached`。

### 3.2 权限探测残留

四个 0 字节文件，是此前排查目录写权限时留下的：

```
edge-agent/roamerx_edge/.write_test
platform/frontend/src/views/.write_test
robot/src/slam/src/include/global_factor_graph.h.write_test
.codex-robot-write-test
```

直接删除即可，不需要保留。

---

## 4. 提交前检查清单

1. `docs/README.md` 的两处 `FRONTEND_MONITORING_VIDEO_ARCHITECTURE` 链接（第 12、34 行）在 C2 中会变成死链，必须同步修正。
2. `yuwang.skill/SKILL.md` 的删除（-164）确认是有意为之。
3. 先补 `.gitignore` 再做任何 `git add -A`，否则家目录状态文件会被卷进去。
4. 删除四个 `.write_test` 残留。
5. C1 涉及 edge-agent 与 robot 两端协议对齐，提交前跑 `scripts/test_agents.sh`；C2 跑 `cd platform/frontend && npm test && npm run build`；C3 需要重新 `colcon build --packages-select robot_navigo`。
6. 平板截图基线是二进制，确认是有意更新而非环境差异导致的误差重录。

---

## 5. 与本轮优化方案的关系

C1 修改的 `robot/src/slam/src/{config/config.yaml, src/mapping_alg.cpp, CMakeLists 相关}` 与 [IMU 漂移诊断与整改方案](IMU_DRIFT_DIAGNOSIS_AND_REMEDIATION_PLAN.md) 的整改点位于同一批文件。**建议先把 C1 提交固化，再动 IMU 整改**，避免两套改动纠缠在同一个未提交工作区里难以区分和回滚。

`docs/MAP_ORIGIN_RTK_ODOM_DIAGNOSTIC_PLAN.md`（C4 中新增）已经提出"建图全程里程计源切换和同步诊断 rosbag"，与 [地图管线方案](MAP_PIPELINE_MCAP_FOXGLOVE_AND_ROUTE_PREVIEW_PLAN.md) 的 mcap 切换有重叠，实施时需要合并考虑。
