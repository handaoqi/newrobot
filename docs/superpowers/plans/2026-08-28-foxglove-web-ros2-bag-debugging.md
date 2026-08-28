# 机器狗 ROS2 数据可视化与 Bag 回放调试实施计划

> 日期：2026-08-28（Asia/Shanghai）
> 状态：方案完成，尚未实施
> 决策约束：完全离线、自托管、不依赖 Foxglove 账户；先完成桌面端验证，再进入 Web 端
> 交付边界：本文只规划实施，不代表相应代码、测试或验收已经完成

## 0. 先说结论

- [x] 不把 `studio.foxglove.dev`、`app.foxglove.dev`、Foxglove 云账户或在线布局服务放入运行链路。
- [x] 默认宿主锁定为 **Lichtblick Desktop/Web 1.28.1**（Foxglove Studio 开源代码的社区延续，MPL-2.0），用同一个 `.foxe` 扩展先在桌面端通过验收，再部署固定版本的自托管 Web 镜像。官方 Foxglove Desktop 3.0.0 的离线独立许可只保留为商业授权后的可选验证项，不是交付依赖。
- [x] 标准 rosbag2 SQLite3 目录始终是源交付物：`<bag>/metadata.yaml + <bag>/*.db3`；不接受孤立 `.db3` 作为合格交付。
- [x] 回放采用三条互补路径，并在实施第一阶段做最小兼容性验证：
  1. 桌面/Web 直接打开完整 `.db3` 文件集：仅在标准消息、分片、中文路径和性能实测全部通过后标记“直接支持”；
  2. `ros2 bag play --clock` + `foxglove_bridge`：用于算法节点在环和 ROS2 原生回放，不把 Humble 的命令行播放器虚构成可由浏览器任意 seek 的服务；
  3. 保留原始 `.db3`，生成带 schema 和索引的 MCAP 派生物：作为桌面/Web 的推荐直接回放格式，承载完整时间轴控制和自定义消息。
- [x] 实时路径固定为 ROS2 → 本机/LAN `foxglove_bridge:8765` → 桌面或浏览器；回放主路径固定为 SQLite3 源 Bag → 校验/转换 → MCAP → 宿主原生播放器。
- [x] 实时模式与回放模式在同一实例中互斥；切换时必须停止订阅、取消加载、清空 TF/消息/时间缓存并用 generation token 丢弃迟到数据。
- [x] 地图、TF、路径、点云/雷达、相机、Plot、Raw Messages 和时间轴复用宿主原生面板；只自定义“巡逻状态/连接状态/话题健康/告警汇总”面板，避免重新实现成熟的 3D 和播放器。
- [x] 不建议拆成多个相互独立的计划。本文是一份统一主计划，以“兼容性门禁 → 桌面端 → Web 端 → 加固交付”分阶段；两端共享话题契约、示例 Bag、扩展包和验收用例，拆开会造成契约漂移。

## 1. 资料阅读清单与事实提取

### 1.1 已完整读取的主要资料

- [x] `/home/dogrobot/docs/yuwang/基于Foxglove Web插件实现ROS2 Bag回放调试方案（替代桌面版）.docx`
- [x] `/home/dogrobot/docs/yuwang/Foxglove Studio ROS2 Bag回放调试界面设计.docx`
- [x] `/home/dogrobot/docs/机器狗导航蛇形前进（不走直线）故障分析.docx`
- [x] `/home/dogrobot/docs/LIO作为室内导航主定位源方案(1).docx`
- [x] `/home/dogrobot/docs/离线重生成地图：LIO-SAM 地图生成完整分析与代码参考(1).docx`
- [x] `/home/dogrobot/docs/MAP_PIPELINE_MCAP_FOXGLOVE_AND_ROUTE_PREVIEW_PLAN.md`
- [x] `/home/dogrobot/docs/NAVIGATION_SENSOR_DESCRIPTION.md`
- [x] `/home/dogrobot/docs/MAP_ORIGIN_RTK_ODOM_DIAGNOSTIC_PLAN.md`
- [x] `/home/dogrobot/docs/SLAM_DATA_CAPTURE_AND_WORLD_POSE_PLAN.md`
- [x] `/home/dogrobot/docs/前端总体架构.md`
- [x] `/home/dogrobot/docs/README.md`
- [x] `/home/dogrobot/README.md`
- [x] `/home/dogrobot/robot/README.md`
- [x] `/home/dogrobot/robot/AGENTS.md`
- [x] `/home/dogrobot/platform/frontend/README.md`
- [x] `/home/dogrobot/platform/docs/edge-and-video-pipeline.md`
- [x] `/home/dogrobot/platform/docs/api-and-payloads.md`
- [x] 当前工作树已删除的 `docs/FRONTEND_MONITORING_VIDEO_ARCHITECTURE.md`：只从 Git `HEAD` 历史版本读取，未恢复、未修改。

### 1.2 已筛选并分析的代码、脚本、配置和制品

- [x] `/home/dogrobot/robot/script/robot/mapping_rosbag.sh`
- [x] `/home/dogrobot/robot/script/robot/navigation_rosbag.sh`
- [x] `/home/dogrobot/robot/script/robot/replay_mapping_rosbag.sh`
- [x] `/home/dogrobot/robot/script/robot/recording_manifest.py`
- [x] `/home/dogrobot/edge-agent/roamerx_edge/rosbag_recorder.py` 及调用链
- [x] `/home/dogrobot/robot/script/robot/start_navigation_real.sh` 及导航启动/参数文件
- [x] `/home/dogrobot/robot/src/navigation/src/robot_navigo` 中地图、TF、速度和启动相关源码
- [x] `/home/dogrobot/robot/src/localization/localization` 中定位、坐标和 QoS 相关源码
- [x] `/home/dogrobot/platform/frontend/package.json`、`package-lock.json`、Vite/Playwright 配置、路由、视图和测试目录
- [x] `/home/dogrobot/platform`、`robot/src`、`edge-agent` 的目录组织和 README
- [x] Git 状态、当前分支和最近 12 条提交记录
- [x] 现存 rosbag2 SQLite3 与 MCAP 目录的 `metadata.yaml`、大小、持续时间、话题、频率和 QoS；只读检查，未播放、重建或改写 Bag。
- [x] 当前 ROS graph 的节点和话题类型；只读查询，没有启动导航或长期服务。

### 1.3 文档读取失败和替代方式

| 文件/能力 | 失败原因 | 替代处理 | 完整性 |
|---|---|---|---|
| 上述 `.docx` 的 `python-docx` 读取 | 系统 Python 及 `platform/.venv` 均报 `ModuleNotFoundError: No module named 'docx'` | 将 `.docx` 作为 OpenXML ZIP，只读解析 `word/document.xml`、表格、页眉和页脚；统一按 Unicode/UTF-8 输出 | 已读取正文和表格；原文件未修改 |
| `docs/FRONTEND_MONITORING_VIDEO_ARCHITECTURE.md` 工作树版本 | 当前 Git 状态为删除，文件不存在 | 只读 `git show HEAD:docs/FRONTEND_MONITORING_VIDEO_ARCHITECTURE.md` | 仅作历史依据，并明确不是当前工作树事实 |
| `/runtime/.../navigation/20260828_122342_indoor_patrol_diag/metadata.yaml` | 目录中只有约 558 MiB `.db3`，缺少元数据；`ros2 bag info` 明确失败 | 保留为负向测试候选，不执行 `ros2 bag reindex`，避免改写用户数据 | 已记录失败路径与原因 |

### 1.4 主设计文档中需要纠正的结论

- [x] `@foxglove/studio-web` 当前 npm 查询返回 404，不能使用文档中的全局安装命令。
- [x] “Web 与桌面 100% 一致”“浏览器可靠打开任意孤立 `.db3`”均没有项目实测依据，不能作为已支持事实。
- [x] `.db3` 不自带自定义 ROS2 消息定义；Lichtblick 1.28.1 的实现也明确只解码内置类型。自定义消息必须通过 MCAP 内嵌 schema，或在 ROS 环境中经 bridge 播放。
- [x] 项目已安装的是 `foxglove_bridge`，不是必须改用 rosbridge；本机 `9090` 端口当前被 `mihomo` 占用，默认桥接端口应继续使用 `8765`。
- [x] 文档中的若干话题（如 `/ukf/odometry`、`/scan_cloud`、`/nav2/planner/plan`）并非当前仓库确认的真实名称，只能作为 UI 意图，不能直接固化。

## 2. 已确认的环境、版本和项目现状

### 2.1 版本基线

| 项目 | 已确认值 | 依据/影响 |
|---|---|---|
| 操作系统 | Ubuntu 22.04.5 LTS，aarch64，Linux 5.15.148-rt-tegra | 当前 NX 主机；离线包需包含 arm64 运行制品 |
| ROS2 | Humble | `robot/AGENTS.md`、安装包和 CLI |
| Python | 3.10.12 | ROS2 Python 示例包按 3.10 编写 |
| Node.js / npm | 22.22.3 / 10.9.8 | 当前前端可用；pnpm 和 Corepack 未安装 |
| 前端 | Vue 声明 `^3.4.15`、lock 为 3.5.33；Vite 4.5.14 | 现有运营平台保持 Vue；扩展独立构建，不强塞入 Vue 组件树 |
| 包管理器 | npm + `package-lock.json` | 新扩展也采用 npm 锁文件，除非兼容性实验要求固定的宿主构建链 |
| 测试 | Node `node:test`、Playwright 1.62.1、Vite build | 复用现有 CI 习惯 |
| `foxglove_bridge` | 3.4.3 | 已安装；默认端口 8765 |
| rosbag2 / sqlite / MCAP 插件 | 0.15.16 | `rosbag2_storage_mcap` 已安装；支持 record/play/info/reindex/convert |
| `rmw_zenoh_cpp` | 0.1.8 | 生产 `ROS_DOMAIN_ID=24`；回放隔离域继续用 77 |
| 官方 Foxglove 产品 | 查询日当前版本 3.0.0 | 只作 API/商业许可对照，不进入默认运行链路 |
| Foxglove Extension npm | `@foxglove/extension` 3.0.0；`create-foxglove-extension` 1.3.2 | 必须在兼容性任务里验证与离线宿主的 API 交集，不能仅凭版本号假设兼容 |
| 自托管宿主 | Lichtblick 1.28.1，commit `189a4bf107f1320d938f9fda891425972e38110a` | 桌面/Web 同仓、`.foxe` 本地加载、`.db3`/MCAP/WS 数据源；锁 tag 和镜像 digest |

硬件事实：8 核 Cortex-A78AE（最高约 1.984 GHz）、15 GiB RAM；检查时可用内存不足 1 GiB、磁盘约 45 GiB 可用。性能基线必须在“NX 上桥接、操作员 x86_64 电脑上渲染”和“NX 本机浏览器压力场景”分别记录，不应把高带宽点云和视频默认全量堆到 NX 浏览器。

### 2.2 当前可复用能力

- [x] `mapping_rosbag.sh` 已按完整目录录制 SQLite3 Bag，包含磁盘空间门禁、信号清理、命名清洗、状态 JSON 和保留策略。
- [x] `navigation_rosbag.sh` 已扩展导航话题白名单，可作为生产录制入口的基础。
- [x] `recording_manifest.py` 已能读取 `metadata.yaml` 并检查固定话题，可扩展为通用 Bag 契约检查器。
- [x] `replay_mapping_rosbag.sh` 已有隔离域 77、进程清理和话题过滤，可扩展 `--clock`、QoS override 和明确的回放模式；它当前没有浏览器控制接口。
- [x] 已有多个标准 SQLite3 Bag 和一个 MCAP 可用于兼容性实验，不需要为了第一轮验证启动真实机器狗。
- [x] 运营前端已有相机链路：RTSP → ffmpeg → ZLMediaKit → FLV/HLS。它不是 ROS 图像话题，Web 调试页应按“平台视频”单独接入或通过可选 ROS 适配器接入，不应伪称现有 `/camera/...`。
- [x] 前端已有轨迹回看，但它是业务 GPS/位姿轨迹，不是 rosbag 播放器；可复用视觉风格，不能复用为 Bag 时间轴实现。
- [x] `foxglove_bridge` 已安装；当前没有运行，启动参数尚未纳入仓库或 systemd。

### 2.3 已确认的 Bag 样本

| 路径 | 格式/状态 | 持续时间/规模 | 用途 |
|---|---|---|---|
| `/home/dogrobot/runtime/nx-edge/data/rosbags/navigation/20260808_002131_smoke_test` | 完整 SQLite3 目录 | 6.684 s，约 34.5 MiB，2,153 条 | 快速直读/桥接冒烟 |
| `/home/dogrobot/runtime/nx-edge/data/rosbags/navigation/20260808_002915_task_0f7411c4` | 完整 SQLite3 目录 | 317.07 s，约 1.7 GiB，167,278 条 | 中等 Bag 性能、跳转和内存 |
| `/home/dogrobot/runtime/nx-edge/data/rosbags/mapping/20260825_170835_V1` | 完整 SQLite3 目录 | 107.393 s，30,707 条 | 地图/定位/TF 兼容 |
| `/home/dogrobot/runtime/nx-edge/data/patrol-data/nav_sway_20260826_1555` | 完整 MCAP 目录 | 167.792 s，约 30.5 MiB，41,390 条 | MCAP 主回放基线 |
| `/home/dogrobot/runtime/nx-edge/data/rosbags/navigation/20260828_122342_indoor_patrol_diag` | 不完整 SQLite3 | 缺 `metadata.yaml` | 只读负向样本，不修复原件 |

### 2.4 Git 状态与近期变更

- [x] 当前分支：`docs/frontend-docs-sync`。
- [x] 工作树包含大量用户已有的 modified/deleted/untracked 文件，涉及 docs、edge-agent、frontend、navigation、localization、slam 等；实施时必须创建独立分支/工作树，提交前逐路径审查，禁止覆盖这些改动。
- [x] 最近提交集中在 2026-08-25 的前端响应式布局、路线规划、UKF 航点定位、SLAM/ENU 安全保持等。新调试入口要避免和 `DashboardLayout.vue` 的现有未提交修改冲突。

## 3. 仍存在的不确定项、假设和确认方法

| 状态 | 项目 | 影响 | 确认方法和截止门禁 |
|---|---|---|---|
| 已由用户确认 | 完全离线、自托管、无 Foxglove 账户 | 排除官方 SaaS/embed 默认路线 | 已固定，不再询问 |
| 已由用户确认 | 先桌面、后 Web | 决定阶段顺序 | Web 阶段必须等待桌面 M2 通过 |
| 待确认假设 | 操作员桌面目标 OS 为 Ubuntu 22.04 x86_64 | 决定离线安装包架构 | M0 采集 `uname -m`、OS 和 GPU；若另含 Windows，单列安装包但不改变数据架构 |
| 待确认假设 | 可接受 Lichtblick 作为 Foxglove-compatible 离线宿主 | 名称和许可证交付 | M0 法务/产品签字；若拒绝，只能采购官方 standalone/custom self-host 许可或改为自研播放器，需重新评审预算 |
| 待确认假设 | Web 首期支持 Chromium 119+ | File API、WASM、WebGL 能力 | M0 在目标浏览器跑 capability probe；不满足则使用文件上传/HTTP Range 后端 |
| 待确认假设 | Web 可在机器人专网 HTTP 运行；跨网段统一 HTTPS/WSS | mixed-content/CORS | M3 部署拓扑评审；非 localhost 必须同源反代或部署证书 |
| 待确认假设 | 实机没有标准 ROS 相机、电池、诊断话题 | 面板实机覆盖度 | M1 在导航运行时重新采集 `ros2 topic list -t`、`topic info -v`；缺失项按“平台数据/可选适配器”标识 |
| 待确认假设 | 浏览器一次只加载一个 Bag 数据源 | 状态和内存上限 | M0 产品确认；本期明确拒绝多 Bag 同时播放，可允许同格式分片作为一个逻辑 Bag |
| 待确认假设 | 120 秒 demo 的上限 80 MiB 可接受 | 离线制品体积 | M2 实测；超过则降低相机频率/质量，不降低 TF/里程计时间正确性 |

当前没有仍需立即向用户追问、会改变推荐架构的问题。若产品不接受 Lichtblick，才触发一次新的架构决策；普通话题别名、颜色和面板尺寸都走配置。

## 4. 范围、非目标和安全边界

### 4.1 本期范围

- [ ] 离线桌面宿主安装、扩展安装、实时 WS、DB3/MCAP 回放及桌面 E2E。
- [ ] 离线 Web 宿主镜像、扩展持久化/预装、同源 WS、文件回放及 Web E2E。
- [ ] 实时/回放互斥状态机、连接/重连/清理和清晰错误。
- [ ] 原生面板布局：地图、位姿、目标、计划、轨迹、TF、LaserScan/PointCloud2、相机、Plot、日志和时间轴。
- [ ] 自定义巡逻概览：任务阶段、巡逻点、成功/失败、电池、诊断、话题频率/延迟/更新时间。
- [ ] 标准 SQLite3 Bag 录制/校验/交付及 MCAP 派生转换。
- [ ] 无真机的确定性室内巡逻发布器、一键脚本、完整文档和测试。

### 4.2 明确非目标

- [x] 不从网页向真实机器狗发布 `/cmd_vel`、目标点或任意控制指令。
- [x] bridge 默认移除 `clientPublish`、services、parameters 等写能力，只开放只读话题和 connection graph。
- [x] 不建设云端 Bag 仓库、多租户、账户、权限或付费系统。
- [x] 不替代宿主所有通用面板，不重写 3D、图像和时间轴。
- [x] 不要求真实机器狗作为唯一测试条件。
- [x] 不提交大型 Bag 到普通 Git；没有确认 Git LFS 规范前，只提交生成器、manifest、校验和及小型测试 fixture。
- [x] 不做与目标无关的仓库重构，不触碰现有脏工作树文件。

若未来加入控制，必须另立设计：观测/控制进程隔离、身份鉴权、急停、速度限制、dead-man、命令超时、审计和实机安全评审；本计划不预留隐式发布通道。

## 5. 技术方案比较与推荐

### 5.1 宿主方案

| 方案 | 离线/无账户 | 开发量 | 兼容性 | 部署 | 结论 |
|---|---:|---:|---|---|---|
| 官方 Foxglove Web / Embed | 否；自托管 viewer 需定制 Enterprise 协议，Embed 还涉及组织登录/席位 | 低 | 官方最佳 | 商业授权和专门交付 | 不作为默认方案 |
| 官方 Foxglove Desktop standalone → 官方自托管 | 可离线但需要 Enterprise standalone license；Web 仍需单独协议 | 低～中 | 官方最佳 | 许可是关键依赖 | 仅在公司已采购时作为附加验收 |
| Lichtblick Desktop 1.28.1 → Lichtblick Web 1.28.1 | 是 | 中 | 兼容 Foxglove WebSocket、`.foxe`、DB3、MCAP；API 必须实测 | 桌面包 + 固定 Docker/静态制品 | **推荐** |
| 自研 Vue 播放器 | 是 | 很高 | 全部自行维护 | 可嵌现平台 | 只在社区宿主被否决时重新立项 |

### 5.2 三种 Bag 方案的最小验证矩阵

| 方案 | 浏览器兼容性 | 开发量 | 跨平台 | 性能 | 部署复杂度 | 用户体验 | 定位 |
|---|---|---:|---|---|---|---|---|
| A. 直接加载 rosbag2 SQLite3 `.db3` | 宿主声明支持，但自定义类型无 schema，分片/大文件/中文路径待实测 | 低 | 桌面/Web | SQL.js/WASM 可能产生内存压力 | 低 | 原生时间轴最佳 | 标准 demo 的快捷路径；必须以实测为准 |
| B. `ros2 bag play --clock` → `foxglove_bridge` | 只要 WS 可达即兼容 | 中 | 浏览器跨平台；播放机需 ROS2 Humble | 流式、可过滤，适合节点在环 | 需 ROS 环境、域和进程治理 | Humble CLI 可暂停/速率/步进，但没有完整浏览器任意 seek API | 算法在环、兼容兜底，不作为主文件播放器 |
| C. SQLite3 保留 + 转 MCAP | 最佳；schema、自定义消息和索引自包含 | 中 | 最好 | 顺序读取、索引跳转较好 | 增加一次离线转换和校验 | 原生时间轴完整 | **桌面/Web 主回放推荐** |

明确推荐：A 通过时保留“直接打开 DB3”入口；C 是所有正式演示 Bag 的默认浏览器制品；B 用于需要 ROS2 节点参与的回放。任何 A 的失败都显示“此 Bag 需转换或桥接”，不能显示“已直接支持”。

## 6. 系统架构、数据流和模块边界

### 6.1 总体结构

```text
实时：ROS2 Domain 24 ──只读话题──> foxglove_bridge:8765 ──WS/WSS──> Desktop/Web
                                              │                         ├─ 原生 3D/Image/Plot/Timeline
                                              └─ allowlist              └─ Patrol Overview 扩展

录制：ROS2 ──ros2 bag record(sqlite3)──> bag_dir/{metadata.yaml, *.db3}
                                             ├─ validator + manifest + sha256
                                             └─ ros2 bag convert ──> derived/*.mcap

文件回放：DB3 文件集（条件支持）或 MCAP ──宿主原生 player──> 统一时间轴和所有面板

节点在环回放：bag_dir ──ros2 bag play --clock, Domain 77──> ROS2 节点/foxglove_bridge
```

### 6.2 模式状态机

`idle → connecting → live → disconnecting → idle`；`idle → loading → ready → playing ⇄ paused → ended → idle`；任意异步状态可进入 `cancelling` 或 `error`。

- [ ] 实时与文件回放互斥；连接实时前关闭文件 player，加载文件前关闭 WS。
- [ ] 每次切换递增 `sourceGeneration`；所有消息、timer、promise 回调先比较 generation，旧源数据直接丢弃。
- [ ] cleanup 顺序：禁止新事件 → abort load/reconnect → unsubscribe → close socket/player → clear timer/TF/message cache → reset UI。
- [ ] 重复点击 connect/load/play 通过幂等命令或 `busy` gate 合并，不生成重复订阅/子进程。
- [ ] 刷新只恢复非敏感配置、布局、允许的 WS URL 和面板选项；不自动恢复本地 `File` 句柄、不自动连接或播放。
- [ ] 单实例只允许一个逻辑 Bag；同一个 Bag 的多 `.db3` 分片必须作为一次有序选择，拒绝混合两个 metadata 身份。

### 6.3 模块边界

| 模块 | 职责 | 不负责 |
|---|---|---|
| `platform/foxglove-extension` | 话题契约、时间/健康计算、状态面板、host adapter、布局 | 3D 渲染、Bag SQL 解析、ROS 进程启动 |
| 宿主原生面板 | 文件 player、时间轴、3D、Image、Plot、Raw Messages | 巡逻业务语义 |
| `roamerx_patrol_demo` | 确定性标准 ROS 消息、地图、轨迹、可选传感器 | 真机控制、生产导航算法 |
| bridge launch/config | 只读 allowlist、端口、超时、域隔离 | Bag 索引和转换 |
| Bag 工具/脚本 | 录制、校验、转换、manifest、进程清理 | 浏览器任意本地文件访问 |
| `platform/foxglove-web` | 固定宿主、离线镜像、同源反代、CSP、默认布局/扩展 | 账号/云存储 |
| 现有 Vue 前端 | 可选入口链接和部署导航 | 承担播放器内部实现 |

### 6.4 UI 面板复用决策

| 区域 | 实现 |
|---|---|
| 顶部数据源、连接、模式、文件选择 | 宿主原生 datasource；扩展只显示只读摘要，不复制连接器 |
| 地图、机器人位姿、目标、plan、实际轨迹 | 原生 3D 面板，固定 frame `map`，配置 topic aliases |
| TF/TF2 | 原生 3D transforms + Raw Messages；扩展汇总断链/陈旧/外推告警 |
| LaserScan / PointCloud2 | 原生 3D；默认 LaserScan，点云按需开启并设 size/decay |
| 相机 | 原生 Image；实机若仅有 FLV/HLS，则在现有平台页显示，不混进 ROS pane |
| 速度、姿态、IMU、里程计 | 原生 Plot/Gauge；布局预置常用表达式 |
| 电池、诊断、任务状态、巡逻点 | 自定义 Patrol Overview 面板 |
| 话题搜索、类型、订阅 | 原生 Topics/Raw Messages；自定义面板补充频率、延迟、最近更新时间和预期契约 |
| Bag 播放/暂停/停止/倍速/跳转/单步/循环 | 文件模式使用宿主原生时间轴和快捷键；“停止”定义为暂停并 seek 到起点；bridge 模式明确标记为外部播放器控制 |
| 日志/警告 | 原生 Log/Raw Messages + 扩展契约告警 |

## 7. ROS2 话题、时间、TF 和 QoS 契约

所有名称都由 `topic-map.yaml` 覆盖；下表“实机确认”来自代码、现有 Bag 或当前 graph，“demo 默认”只用于新示例。

| 语义 | 实机话题 / demo 默认 | 消息类型 | 预期频率 | QoS（R/D/H/depth） | stamp / frame | 展示/录制/异常 |
|---|---|---|---:|---|---|---|
| TF | `/tf` | `tf2_msgs/msg/TFMessage` | 约 20～114 Hz（按变换总量） | reliable/volatile/keep_last/100 | 每个 transform stamp；父子 frame | 3D；必录；断链、循环、未来/过去外推分别告警 |
| 静态 TF | `/tf_static` | `tf2_msgs/msg/TFMessage` | 启动一次 | reliable/transient_local/keep_last/1 | 静态 frame | 3D；必录；缺失即阻止相关传感器定位 |
| 仿真时钟 | 无实机发布 / `/clock` | `rosgraph_msgs/msg/Clock` | demo 50 Hz；play 可设 40 Hz | reliable/volatile/keep_last/10 | `clock` | demo/回放必录或由 player 发布；停滞、回退、跳变重置窗口 |
| 地图 | Nav2 运行时 `/map` / `/map` | `nav_msgs/msg/OccupancyGrid` | 一次/更新 | reliable/transient_local/keep_last/1 | `header.stamp`, `map` | 3D；演示必录；无地图仍显示 odom 轨迹并告警 |
| 定位里程计 | `/odom/localization_odom`、`/odom/nav2` / `/odom` | `nav_msgs/msg/Odometry` | 实机约 8～17 Hz；demo 20 Hz | reliable/volatile/keep_last/10 | `header.stamp`, `odom→base_link` | 3D/Plot；必录；乱序丢弃并计数 |
| 机器人底盘里程计 | `/odom/mc_odom` | `nav_msgs/msg/Odometry` | 实机约 94～100 Hz | reliable/volatile/keep_last/20 | `odom→base_link` | 对比 Plot；生产建议录制；浏览器默认降到 20 Hz |
| 速度指令观测 | `/cmd_vel`、`/cmd_vel_raw` | `geometry_msgs/msg/Twist` | 实机约 14～19 Hz；demo 20 Hz | reliable/volatile/keep_last/10 | 类型无 header，使用接收/日志时间 | Plot；必录；只订阅，禁止发布 |
| 激光点云 | `/front_lidar` | `sensor_msgs/msg/PointCloud2` | 约 10 Hz | best_effort/volatile/keep_last/10 | `livox_frame` | 3D；按需录；默认限点/decay，积压时丢旧帧 |
| 激光扫描 | `/laser_scan` / `/scan` | `sensor_msgs/msg/LaserScan` | 10 Hz | best_effort/volatile/keep_last/5 | `base_scan` | 3D；演示必录；无 TF 则隐藏并报 frame |
| IMU | `/front_lidar/imu` / `/imu/data` | `sensor_msgs/msg/Imu` | 实机约 200 Hz；demo 50 Hz | best_effort/volatile/keep_last/5（录制历史曾为 100） | `imu_link`/实际 frame | Plot；演示必录；UI 降采样至 25 Hz |
| GNSS | `/fix` | `sensor_msgs/msg/NavSatFix` | 约 10 Hz | 现场发布 QoS 待运行态确认；订阅 best_effort/volatile/5 | `header.stamp`, GNSS frame | 诊断辅助；生产按需录；QoS 不匹配提示 |
| RTK | `/rtk_pvh` | `robots_dog_msgs/msg/UniRtkPvh` | 约 10 Hz | best_effort/volatile/keep_last/1 | 消息时间/自定义 | 自定义类型必须 MCAP 或 bridge；生产按需录 |
| 全局路径 | `/plan` / `/plan` | `nav_msgs/msg/Path` | 更新时/约 0.66 Hz | reliable/volatile/keep_last/1 | pose stamps, `map` | 3D；必录；空路径显示“无可用规划” |
| 转换后路径 | `/transformed_global_plan` | `nav_msgs/msg/Path` | 更新时 | reliable/volatile/keep_last/1 | `map`/控制 frame | 3D；生产按需录 |
| 目标点 | 运行态待确认 `/goal_pose` / `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 巡逻点切换 | reliable/volatile/keep_last/1 | `map` | 3D；演示必录；frame 不存在拒绝绘制 |
| 实际轨迹 | 无固定生产话题 / `/patrol/trajectory` | `nav_msgs/msg/Path` | demo 2 Hz | reliable/volatile/keep_last/2 | `map` | 3D；演示必录；长度上限 5,000 poses |
| ROS 相机 | 当前未发现 / `/camera/front/image/compressed` | `sensor_msgs/msg/CompressedImage` | demo 2 Hz | best_effort/volatile/keep_last/2 | `camera_optical_frame` | Image；演示可选但正式 demo 开启；坏 JPEG 单帧告警 |
| 电池 | 当前未发现 / `/battery_state` | `sensor_msgs/msg/BatteryState` | 1 Hz | reliable/volatile/keep_last/10 | header stamp | 自定义面板；演示必录；NaN 显示未知 |
| 诊断 | 当前未发现 / `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 1 Hz | reliable/volatile/keep_last/10 | header stamp | 告警面板；演示必录；按 level 聚合 |
| 扫描匹配状态 | `/status` | `localization/msg/ScanMatchingStatus` | 约 8.6 Hz | reliable/volatile/keep_last/5 | 自定义 | MCAP/bridge；标准 demo 不依赖此类型 |
| 定位信息 | `/localization_info` | `robots_dog_msgs/msg/Localization` | 约 8.5 Hz | reliable/volatile/keep_last/10 | 自定义 | MCAP/bridge；缺包时给“schema 不可用” |
| 巡逻状态 | 无固定标准 / `/patrol/status` | `std_msgs/msg/String`，JSON schema `roamerx.patrol-status.v1` | 状态变化 + 1 Hz | reliable/transient_local/keep_last/1 | JSON 内含 `stamp_ns` | 自定义面板；演示必录；schema/version 校验失败不崩溃 |

缩写：R=reliability，D=durability，H=history。录制端通过 QoS override 配置同时兼容 sensor-data best-effort 和 `/tf_static` transient-local，不能用一个全局 QoS 覆盖所有话题。

### 7.1 时间规则

- [ ] 文件面板全部以 **log time** 为默认横轴；消息 header stamp 只用于延迟、乱序和 TF 诊断。
- [ ] demo 从固定 epoch `1700000000.000000000` 开始，随机数使用固定 seed 20260828；相同配置产生相同语义序列。
- [ ] live 真机保持 `use_sim_time=false`，不发布 `/clock`。
- [ ] 节点在环回放在独立 Domain 77 启动，播放器使用 `--clock 40`；所有参与回放的 ROS 节点必须在启动前设 `use_sim_time=true`，bridge 自身不依赖仿真时间来传输。
- [ ] `/clock` 缺失：文件模式仍可用 log time；节点在环模式阻止需要 sim time 的节点并显示错误。
- [ ] `/clock` 停止超过 500 ms 显示 paused；回退或跳转时清空滑动频率/延迟窗口和 TF temporal cache，允许播放器正常 seek，不误报乱序风暴。

### 7.2 TF 规则

- [ ] 生产约定 `map → odom → base_link`，`base_link → livox_frame` 为静态外参；当前工厂外参记录为平移 `(0.382765605,-0.046855740,0.513445457)`、四元数 `(0.007172121,-0.043589169,-0.009509268,0.998978536)`，不得由 demo 覆盖生产 TF。
- [ ] demo 使用独立 Domain 77，并发布 `map → odom` 静态、`odom → base_link` 动态、`base_link → base_scan/imu_link/camera_link` 静态。
- [ ] 诊断区分：未知 frame、树断裂、同一 child 多父、循环、数据过旧、未来/过去外推；循环/多父为红色，陈旧/外推为黄色。

## 8. 演示场景与标准 Bag 交付契约

### 8.1 确定性室内巡逻

- [ ] 120 秒、10 m × 8 m 简单室内栅格地图，分辨率 0.05 m；起点 `(1,1,0)`。
- [ ] 巡逻点依次为 P1 `(7,1)`、P2 `(7,6)`、P3 `(2,6)`、P4 `(2,3)`，最后返回起点。
- [ ] 线速度默认 0.35 m/s，角速度上限 0.6 rad/s，所有插值按固定时步计算。
- [ ] P2 停留 5 秒；P3 前模拟一处障碍并生成可重复的局部绕行段，同时 diagnostics 从 WARN 恢复 OK。
- [ ] 连续发布 odom、TF、trajectory、cmd_vel、goal、plan、patrol status；默认开启 LaserScan、IMU、battery、diagnostics 和 640×360 JPEG 2 Hz 相机。
- [ ] 电池从 90% 线性降至约 86%；最后状态 `COMPLETED` 并返回起点允许误差 0.10 m、航向误差 3°。

参数：`topic_namespace`、`publish_rate_hz`、`waypoints`、`linear_speed_mps`、`duration_sec`、`publish_camera`、`publish_scan`、`publish_imu`、`bag_root`、`bag_name`、`seed`。非法 namespace、越界 waypoint、非正频率/时长必须在节点启动前失败并给参数名。

### 8.2 Bag 目录与制品

```text
artifacts/indoor_patrol_20260828_001/
├── metadata.yaml
└── indoor_patrol_20260828_001_0.db3

artifacts/indoor_patrol_20260828_001-derived/
├── indoor_patrol_20260828_001.mcap
├── manifest.json
└── SHA256SUMS
```

- [ ] SQLite3 使用 `ros2 bag record -s sqlite3` 明确指定，不依赖默认值。
- [ ] 白名单：`/clock /map /tf /tf_static /odom /cmd_vel /scan /camera/front/image/compressed /imu/data /battery_state /diagnostics /plan /goal_pose /patrol/trajectory /patrol/status`，namespace 启用时统一改写。
- [ ] 预期 120±1 秒；消息总量和每话题计数由 manifest 设置上下限；默认 Bag ≤80 MiB，MCAP 派生物 ≤80 MiB。超限首先降低 JPEG quality 到 65 或相机 1 Hz，不能删除时间/TF 核心话题。
- [ ] `metadata.yaml`、至少一个非空 `.db3`、SQLite `PRAGMA integrity_check=ok`、话题白名单、消息类型、时长和结束状态全部通过后才生成 `READY` manifest。
- [ ] 不把大型 Bag 提交 Git。CI 生成 10 秒小 fixture；正式 120 秒制品由离线发布包或受控共享盘交付，附 SHA-256 和生成配置。

## 9. 准确的预计文件路径

下列文件目前均不存在，标记“新增”；明确列为修改的文件只有在对应阶段且工作树冲突处理完后才改。

### 9.1 扩展（新增）

- [ ] `/home/dogrobot/platform/foxglove-extension/package.json`
- [ ] `/home/dogrobot/platform/foxglove-extension/package-lock.json`
- [ ] `/home/dogrobot/platform/foxglove-extension/tsconfig.json`
- [ ] `/home/dogrobot/platform/foxglove-extension/src/index.ts`
- [ ] `/home/dogrobot/platform/foxglove-extension/src/host/hostAdapter.ts`
- [ ] `/home/dogrobot/platform/foxglove-extension/src/core/sourceState.ts`
- [ ] `/home/dogrobot/platform/foxglove-extension/src/core/topicContract.ts`
- [ ] `/home/dogrobot/platform/foxglove-extension/src/core/time.ts`
- [ ] `/home/dogrobot/platform/foxglove-extension/src/core/health.ts`
- [ ] `/home/dogrobot/platform/foxglove-extension/src/panels/PatrolOverview.tsx`
- [ ] `/home/dogrobot/platform/foxglove-extension/config/topic-map.yaml`
- [ ] `/home/dogrobot/platform/foxglove-extension/layouts/roamerx-indoor-patrol.json`
- [ ] `/home/dogrobot/platform/foxglove-extension/tests/sourceState.test.ts`
- [ ] `/home/dogrobot/platform/foxglove-extension/tests/topicContract.test.ts`
- [ ] `/home/dogrobot/platform/foxglove-extension/tests/time.test.ts`
- [ ] `/home/dogrobot/platform/foxglove-extension/tests/health.test.ts`
- [ ] `/home/dogrobot/platform/foxglove-extension/tests/patrolOverview.test.tsx`
- [ ] `/home/dogrobot/platform/foxglove-extension/tests/integration/liveBridge.test.ts`
- [ ] `/home/dogrobot/platform/foxglove-extension/tests/e2e/desktop.spec.ts`
- [ ] `/home/dogrobot/platform/foxglove-extension/desktop.playwright.config.ts`
- [ ] `/home/dogrobot/platform/foxglove-extension/README.md`

### 9.2 ROS2 demo 与 Bag 工具（新增）

- [ ] `/home/dogrobot/robot/src/tools/roamerx_patrol_demo/package.xml`
- [ ] `/home/dogrobot/robot/src/tools/roamerx_patrol_demo/setup.py`
- [ ] `/home/dogrobot/robot/src/tools/roamerx_patrol_demo/setup.cfg`
- [ ] `/home/dogrobot/robot/src/tools/roamerx_patrol_demo/resource/roamerx_patrol_demo`
- [ ] `/home/dogrobot/robot/src/tools/roamerx_patrol_demo/roamerx_patrol_demo/__init__.py`
- [ ] `/home/dogrobot/robot/src/tools/roamerx_patrol_demo/roamerx_patrol_demo/scenario.py`
- [ ] `/home/dogrobot/robot/src/tools/roamerx_patrol_demo/roamerx_patrol_demo/patrol_publisher.py`
- [ ] `/home/dogrobot/robot/src/tools/roamerx_patrol_demo/roamerx_patrol_demo/bag_contract.py`
- [ ] `/home/dogrobot/robot/src/tools/roamerx_patrol_demo/launch/indoor_patrol_demo.launch.py`
- [ ] `/home/dogrobot/robot/src/tools/roamerx_patrol_demo/launch/foxglove_readonly.launch.py`
- [ ] `/home/dogrobot/robot/src/tools/roamerx_patrol_demo/config/indoor_patrol.yaml`
- [ ] `/home/dogrobot/robot/src/tools/roamerx_patrol_demo/config/record_qos.yaml`
- [ ] `/home/dogrobot/robot/src/tools/roamerx_patrol_demo/config/bridge.yaml`
- [ ] `/home/dogrobot/robot/src/tools/roamerx_patrol_demo/maps/indoor_demo.yaml`
- [ ] `/home/dogrobot/robot/src/tools/roamerx_patrol_demo/maps/indoor_demo.pgm`
- [ ] `/home/dogrobot/robot/src/tools/roamerx_patrol_demo/test/test_scenario.py`
- [ ] `/home/dogrobot/robot/src/tools/roamerx_patrol_demo/test/test_topic_contract.py`
- [ ] `/home/dogrobot/robot/src/tools/roamerx_patrol_demo/test/test_bag_contract.py`
- [ ] `/home/dogrobot/robot/src/tools/roamerx_patrol_demo/test/test_time_and_tf.py`
- [ ] `/home/dogrobot/robot/script/robot/foxglove_demo.sh`
- [ ] `/home/dogrobot/robot/script/robot/test_foxglove_demo.py`

### 9.3 Web 自托管（M3 才新增）

- [ ] `/home/dogrobot/platform/foxglove-web/vendor.lock`
- [ ] `/home/dogrobot/platform/foxglove-web/Dockerfile`
- [ ] `/home/dogrobot/platform/foxglove-web/docker-compose.yml`
- [ ] `/home/dogrobot/platform/foxglove-web/nginx.conf`
- [ ] `/home/dogrobot/platform/foxglove-web/default-layout.json`
- [ ] `/home/dogrobot/platform/foxglove-web/scripts/build-offline-bundle.sh`
- [ ] `/home/dogrobot/platform/foxglove-web/scripts/verify-offline-bundle.sh`
- [ ] `/home/dogrobot/platform/foxglove-web/tests/offline-assets.test.js`
- [ ] `/home/dogrobot/platform/foxglove-web/tests/foxglove-web.spec.js`
- [ ] `/home/dogrobot/platform/foxglove-web/README.md`
- [ ] `/home/dogrobot/platform/frontend/src/views/FoxgloveDebugPage.vue`（可选同源入口）
- [ ] 修改 `/home/dogrobot/platform/frontend/src/router/index.js` 和 `/home/dogrobot/platform/frontend/src/views/DashboardLayout.vue`（仅在现有脏改动已合并/隔离后）
- [ ] `/home/dogrobot/platform/frontend/tests/foxgloveDebugEntry.test.js`

### 9.4 文档（新增/修改）

- [ ] `/home/dogrobot/platform/foxglove-extension/README.md`
- [ ] `/home/dogrobot/robot/src/tools/roamerx_patrol_demo/README.md`
- [ ] `/home/dogrobot/platform/foxglove-web/README.md`
- [ ] 修改 `/home/dogrobot/docs/README.md` 增加入口（先处理当前用户改动）
- [ ] 本计划和 `/home/dogrobot/docs/superpowers/plans/Foxglove实时同步及回放功能说明.docs`

## 10. 分阶段实施任务（依赖顺序、TDD、命令和验收）

每个任务执行固定循环：先提交失败测试/检查（红）→ 最小实现（绿）→ 运行目标测试和相邻回归 → 独立审查。下面的“预期”是验收标准，不是已经发生的测试结果。

### M0：事实冻结和兼容性门禁（3～5 人日）

#### T0.1 离线宿主和环境锁定

- [ ] **依赖**：无。
- [ ] **输入**：目标桌面机器、Lichtblick v1.28.1 tag、官方许可结论。
- [ ] **先写失败检查**：`vendor.lock` 缺 tag/commit/image digest/license/arm64+x86_64 包任一字段时 `verify-offline-bundle.sh` 非零退出。
- [ ] **实现输出**：锁定桌面安装包 SHA-256、Web OCI image digest、源 commit、MPL-2.0 NOTICE、离线 npm cache 清单；运行时断网也不请求外部域名。
- [ ] **命令**：`bash platform/foxglove-web/scripts/verify-offline-bundle.sh`。
- [ ] **预期**：完整包退出 0；删除任一锁定制品的复制件后退出非 0 并打印缺失文件，不修改原制品。
- [ ] **验收**：产品/法务确认 Lichtblick；否则停止后续并触发官方商业许可或自研重新评审。

#### T0.2 方案 A：DB3 直接加载验证

- [ ] **依赖**：T0.1。
- [ ] **输入**：6.7 秒、317 秒现存 Bag 的复制件；10 秒 demo fixture；完整目录和孤立 `.db3` 对照。
- [ ] **先写失败测试**：缺 metadata、多 Bag 混选、自定义类型、损坏复制件、中文/空格路径分别定义期待 alert；源 Bag 只读挂载。
- [ ] **实现**：桌面和 Web 分别选择所有 `.db3` 分片；记录可见 topic、首尾时间、seek、内存峰值和错误文本。
- [ ] **命令**：`ros2 bag info <bag_dir>`；随后执行 `npx playwright test tests/foxglove-web.spec.js --grep 'direct db3'`。
- [ ] **预期**：标准消息能播放、暂停、0.5×/1×/2×、跳转和循环；自定义消息若无 schema 必须明确标为不可解码。孤立 DB3 不能被文档称为标准交付。
- [ ] **验收**：桌面/Web 各形成一行 `compatibility-report.json`；只有所有关键标准话题和 1.7 GiB Bag 稳定项通过，Web 才标“DB3 直接支持”。

#### T0.3 方案 B：ros2 bag play + bridge 验证

- [ ] **依赖**：T0.1。
- [ ] **输入**：smoke Bag、Domain 77、只读 allowlist。
- [ ] **先写失败测试**：Domain 24、缺 `/clock`、QoS 不匹配、bridge 端口占用和重复启动都必须 preflight 失败；不得杀死非本脚本进程。
- [ ] **最小实现**：`ros2 bag play <dir> --clock 40 --start-paused` 与 bridge 3.4.3；以 PID 文件和进程组清理。
- [ ] **命令**：`ROS_DOMAIN_ID=77 ros2 bag play <bag_dir> --clock 40 --start-paused`；另一终端 `ROS_DOMAIN_ID=77 ros2 launch roamerx_patrol_demo foxglove_readonly.launch.py`。
- [ ] **预期**：WS 在 2 秒内可连接，话题和 `/clock` 可见，CLI 空格暂停、右方向键步进、上下调整速率；界面明确显示“外部 ROS 播放器”，不提供伪 seek。
- [ ] **验收**：节点在环场景完成；证明 Humble 原生 player 没有浏览器任意 seek 控制面后，把完整 UI 控制需求绑定到文件 player。

#### T0.4 方案 C：DB3 → MCAP 验证

- [ ] **依赖**：T0.2。
- [ ] **输入**：完整 SQLite3 Bag 复制件、convert output YAML。
- [ ] **先写失败测试**：输入缺 metadata、空间不足（需要输入大小 1.3 倍可用空间）、目标已存在、转换中断、计数不一致都必须失败且不发布 READY manifest。
- [ ] **最小实现**：调用 Humble `ros2 bag convert`，输出临时目录；`ros2 bag info`、topic/count/duration、MCAP footer/index/schema 检查通过后原子重命名。
- [ ] **命令**：`ros2 bag convert -i <db3_dir> -o <convert.yaml>`；`ros2 bag info <mcap_dir>`；`sha256sum <mcap>`。
- [ ] **预期**：标准和自定义类型在 MCAP 可解码；首尾时间差 ≤1 ns、每 topic count 一致、seek 通过；源目录 mtime/sha256 不变。
- [ ] **验收**：MCAP 成为 Web 主回放制品；转换失败时 UI/CLI 给出可行动错误并保留源 Bag。

#### T0.5 同一 `.foxe` 的桌面/Web API 探针

- [ ] **依赖**：T0.1。
- [ ] **输入**：Lichtblick Desktop/Web 1.28.1；Extension API 能力列表。
- [ ] **先写失败测试**：探针面板断言 `subscribe`、`onRender`、settings、saveState、cleanup 和 datasource metadata；任一缺失输出不兼容报告。
- [ ] **最小实现**：仅显示 host/version/capability，不接业务话题；分别安装同一包。API import 由 `hostAdapter.ts` 隔离，不能让业务层直接依赖宿主私有 API。
- [ ] **命令**：`cd platform/foxglove-extension && npm ci --offline && npm test && npm run package`。
- [ ] **预期**：同一 `.foxe` 在桌面 filesystem loader 和 Web IndexedDB loader 安装、重启恢复、卸载；断网抓包没有外部请求。
- [ ] **验收**：通过才进入 M1；若当前 `@foxglove/extension` 3.0 API 不兼容，固定 `@lichtblick/suite@1.28.1` 并保持 adapter 边界，不能声称官方 3.0 二进制兼容。

### M1：扩展内核和桌面实时观测（6～8 人日）

#### T1.1 配置与话题契约

- [ ] **依赖**：T0.5。
- [ ] **测试输入**：真实 topic map、namespace、空字符串、重复 alias、非法消息类型和频率边界。
- [ ] **红/绿**：先写 `topicContract.test.ts`，再实现 schema 校验、默认值、override 合并和可读错误。
- [ ] **接口**：`loadTopicContract(raw) -> ValidatedContract`；禁止静默吞掉未知字段。
- [ ] **命令/预期**：`npm test -- topicContract.test.ts`；合法配置通过，非法项精确指出 YAML 路径。
- [ ] **验收**：表 7 的全部语义映射可覆盖，不依赖硬编码实机 namespace。

#### T1.2 时间、频率、延迟和健康计算

- [ ] **依赖**：T1.1。
- [ ] **先测**：纳秒进位、零时间、clock 回退、乱序、未来 stamp、无 header、滑动窗口清理、50 天大时间戳。
- [ ] **接口**：`toNsec(Time): bigint`、`observe(topic,event,clock)`、`reset(reason)`；使用 bigint，禁止 JS number 存纳秒 epoch。
- [ ] **实现**：5 秒滑动窗口，频率 EWMA；延迟负值超过 50 ms 标 clock mismatch；陈旧阈值为 `max(3/expectedHz, 500ms)`。
- [ ] **命令/预期**：`npm test -- time.test.ts health.test.ts`；边界用例全部通过且 clock seek 后窗口为空。
- [ ] **验收**：实时和文件 log-time 两种时基结果可解释。

#### T1.3 互斥模式状态机和资源清理

- [ ] **依赖**：T1.2。
- [ ] **先测**：双击连接/加载、加载中取消、断线指数退避、live→file→live、旧 generation 消息、页面卸载。
- [ ] **接口**：`dispatch(SourceEvent) -> SourceState`，副作用由 host adapter 注入；退避 0.5/1/2/4/8 秒并加 0～20% jitter，上限 5 次。
- [ ] **命令/预期**：`npm test -- sourceState.test.ts`；每条转移确定，cleanup spy 恰好调用一次。
- [ ] **验收**：无重复订阅、无悬挂 timer/socket/player、取消能在 1 秒内回 idle。

#### T1.4 Patrol Overview 面板

- [ ] **依赖**：T1.1～T1.3。
- [ ] **先测**：缺话题、坏 JSON、未知 schema version、NaN 电量、诊断 ERROR、陈旧/乱序、中文状态文本、1,000 topic 更新。
- [ ] **最小实现**：连接状态、模式、当前巡逻点/阶段、任务结果、电池、诊断和可搜索话题健康表；不实现播放按钮。
- [ ] **命令/预期**：`npm test -- patrolOverview.test.tsx && npm run build && npm run package`；React 清理无 warning，包可安装。
- [ ] **验收**：面板自身崩溃隔离；异常消息显示 topic 和原因，不让整个宿主白屏。

#### T1.5 只读 bridge 与桌面实时布局

- [ ] **依赖**：T1.4。
- [ ] **先测**：配置静态检查必须拒绝 `clientPublish`、services、参数写能力、`0.0.0.0` 公网暴露和非 allowlist topic。
- [ ] **实现**：默认 `127.0.0.1:8765`；LAN 模式显式绑定机器人私网 IP；allowlist 仅表 7 必需话题；send buffer 初始 10 MiB，连接超时 5 秒。
- [ ] **命令/预期**：`ROS_DOMAIN_ID=77 ros2 launch roamerx_patrol_demo foxglove_readonly.launch.py`；桌面连接 `ws://127.0.0.1:8765` 并且 publish/service UI 不可用。
- [ ] **验收**：地图/pose/path/scan 和 Patrol Overview 同步，断桥后 1 秒内显示断开并按策略重连。

### M2：确定性 demo、SQLite3/MCAP 和桌面端验收（7～10 人日）

#### T2.1 场景纯函数和地图

- [ ] **依赖**：T1.1。
- [ ] **先测**：固定 waypoint 采样、速度/角速度上限、P2 停留、绕障路径、终点误差、相同 seed 输出 hash 一致。
- [ ] **接口**：`Scenario.sample(t_ns) -> RobotSample`，无 ROS 副作用；地图像素和 world 坐标互转有单测。
- [ ] **命令/预期**：`colcon test --packages-select roamerx_patrol_demo --pytest-args -k scenario`；120 秒末返回起点，断言公差通过。
- [ ] **验收**：不启动真机和 Nav2 也能生成完整语义序列。

#### T2.2 ROS2 发布器、TF 和 QoS

- [ ] **依赖**：T2.1。
- [ ] **先测**：参数非法、header/frame、TF 无环、静态 QoS、传感器 QoS、clock 单调、可选话题关闭。
- [ ] **最小实现**：标准消息 publisher；`/patrol/status` 使用版本化 JSON；相机画面由时间/位置绘制而非外部素材。
- [ ] **命令/预期**：`ROS_DOMAIN_ID=77 ros2 launch roamerx_patrol_demo indoor_patrol_demo.launch.py duration_sec:=10`；`ros2 topic hz` 在容差 ±10%，`tf2_tools view_frames` 无环。
- [ ] **验收**：不依赖 `robots_dog_msgs` 仍能运行；可选实机自定义 topic 只由扩展适配。

#### T2.3 录制、校验、转换和一键脚本

- [ ] **依赖**：T0.4、T2.2。
- [ ] **先测**：不存在/空格/中文路径、穿越 `..`、缺 metadata、损坏 SQLite、未完成写入、消息包缺失、磁盘不足、重复 start/stop、SIGINT/TERM、目标已存在。
- [ ] **接口**：`foxglove_demo.sh preflight|live|record|info|convert|play|stop|all`；所有生成路径 `realpath` 后必须位于配置 `bag_root`，默认不提供 delete 子命令。
- [ ] **最小实现**：复用现有录制脚本的磁盘门禁/进程组清理思想；临时目录 `.partial-<uuid>`，成功后原子 rename；manifest 记录版本、命令、config hash、topic count、sha256。
- [ ] **命令/预期**：`python3 robot/script/robot/test_foxglove_demo.py`；`robot/script/robot/foxglove_demo.sh all --duration 120 --output <allowed_root>`；结束后无残留进程，DB3/MCAP 都通过 info。
- [ ] **验收**：一条命令完成发布、录制、停止、校验、转换；任何阶段失败均清理本脚本子进程但保留 `.partial` 供诊断。

#### T2.4 桌面端完整 E2E

- [ ] **依赖**：T1.5、T2.3。
- [ ] **前置**：断网；Lichtblick Desktop 1.28.1；扩展 `.foxe`；Domain 77；demo artifacts。
- [ ] **步骤**：启动 demo/bridge → 连接实时 → 观察地图/pose/path/status/scan/camera → 录制完整 DB3 → 停止实时 → 直接 DB3（若 T0.2 通过）或 MCAP → 暂停/继续/0.5×/2×/seek/单步/loop → 多面板同轴 → 注入异常。
- [ ] **命令/预期**：`npx playwright test --config desktop.playwright.config.ts --grep '@desktop-e2e'`；全程无外部网络请求，关键截图/trace/metrics 落到测试产物。
- [ ] **判定**：所有关键面板时间误差 ≤100 ms；seek 后 1 秒内恢复一致视图；重复操作不崩溃；异常文本包含 source/topic/解决建议。
- [ ] **里程碑门禁 M2**：桌面端全部通过且缺陷清零后，才允许创建 Web 部署变更。

### M3：完全离线自托管 Web（6～9 人日）

#### T3.1 固定 Web 宿主和扩展预装/持久化

- [ ] **依赖**：M2 通过。
- [ ] **先测**：镜像 tag 漂移、digest 不匹配、缺 WASM/font/worker、扩展刷新丢失、断网外连都失败。
- [ ] **实现**：基于 v1.28.1 源 commit 或固定 digest 构建；将默认 layout 只读注入；扩展包通过受控首次安装或 build-time preload，IndexedDB 版本升级有迁移验证。
- [ ] **命令/预期**：`docker compose -f platform/foxglove-web/docker-compose.yml build --pull=false`；`verify-offline-bundle.sh`；网络禁用后页面仍完整加载。
- [ ] **验收**：浏览器刷新仍有扩展/布局；清空站点数据后能从本地包恢复；无账户界面阻断。

#### T3.2 Web 文件、WS、CORS/CSP 和大文件策略

- [ ] **依赖**：T3.1。
- [ ] **先测**：HTTP 页面连 WSS/HTTPS mixed content、错误 Origin、无 Range、超限文件、取消选择、中文路径、多个 Bag、页面卸载。
- [ ] **实现**：首选用户手势 File picker（浏览器只能读取用户选择的文件）；远端文件仅允许配置 Bag root 的只读 Range endpoint，`realpath` containment、扩展名/大小/并发校验；同源反代 WS，CSP 禁止任意外连和脚本。
- [ ] **限制**：单 Bag 10 GiB 软上限、15 GiB 硬拒绝；并发加载 1；内存缓存 512 MiB 上限；PointCloud 最新帧优先；所有阈值配置化。
- [ ] **命令/预期**：`npx playwright test platform/foxglove-web/tests/foxglove-web.spec.js --grep 'security|files|websocket'`；越权路径返回 403，超限 413，取消回 idle，页面无泄露路径。
- [ ] **验收**：不存在任意服务器路径浏览、目录穿越、`eval` 或从 Bag 执行代码。

#### T3.3 现有 Vue 平台入口

- [ ] **依赖**：T3.2；先解决 `DashboardLayout.vue` 和路由的当前用户改动。
- [ ] **先测**：入口权限文案、断网 URL、返回导航、窄屏提示；不测试跨域 iframe 黑盒。
- [ ] **实现**：优先同源新窗口/路由跳转 `/foxglove/`；只有 CSP、base path、键盘快捷键和 WebGL 实测无冲突才用 iframe。
- [ ] **命令/预期**：`cd platform/frontend && npm test -- foxgloveDebugEntry.test.js && npm run build`。
- [ ] **验收**：入口明确标注“只读调试”，不会挤压现有运营页面，也不会让 Foxglove 宿主依赖 Vue 运行状态。

#### T3.4 Web 完整 E2E

- [ ] **依赖**：T3.3。
- [ ] **前置/步骤**：与 T2.4 相同，再覆盖 Chrome 目标版本、刷新恢复、WS 断连、1.7 GiB Bag、内存、tab background/foreground。
- [ ] **命令/预期**：`npx playwright test platform/foxglove-web/tests/foxglove-web.spec.js --grep '@web-e2e' --trace on`；所有动作满足第 12 节基线。
- [ ] **验收**：断网、无账户、无桌面客户端情况下完成从打开页面到实时观测和历史回放的全部主流程。

### M4：故障加固、文档与发布（4～6 人日）

#### T4.1 故障矩阵自动化

- [ ] **依赖**：M3。
- [ ] 为第 11 节每个条件建立 fixture/注入器和断言；原 Bag 不改，损坏样本由测试临时复制后修改。
- [ ] `npm test`、`colcon test`、Playwright 三层报告按 case id 聚合。
- [ ] 验收：每个错误有稳定 code、中文摘要、技术详情和恢复动作；后台/页面均可继续使用或安全返回 idle。

#### T4.2 性能和 2 小时稳定性

- [ ] **依赖**：T4.1。
- [ ] 采集 Chrome Performance/heap、Playwright trace、`pidstat -rud`、`docker stats`、bridge topic statistics、`nvidia-smi`/`tegrastats`（目标机器可用哪个用哪个）。
- [ ] 使用 120 秒 demo、1.7 GiB DB3/MCAP 和 live loop；2 小时后执行 20 次 live/file 切换。
- [ ] 验收：达到第 12 节基线，无单调无界内存增长、无残留 PID/FD/socket。

#### T4.3 文档、离线包和发布验收

- [ ] **依赖**：T4.2。
- [ ] 新用户在一台干净、断网机器按 README 完成安装、启动、录制、info、转换、回放；记录所有命令输出。
- [ ] 输出桌面 arm64/x86_64 包（按目标确认）、Web OCI tar、扩展 `.foxe`、demo 源码、配置、10 秒 fixture、120 秒 release artifact、SBOM、NOTICE、SHA256SUMS。
- [ ] 以第 14 节 DoD 签字；未通过项不能以文档措辞弱化。

## 11. 边界条件与可测试行为

| 条件 | 预期行为 |
|---|---|
| 路径不存在/越界 | 加载前拒绝；显示规范化路径和允许根，不创建文件 |
| 缺 `metadata.yaml` | 判定“不完整 rosbag2 目录”；提供“复制后 reindex”命令，不改原件 |
| DB3 损坏/未完成/元数据不一致 | `integrity_check`、size/count 对比失败；不转换、不标 READY |
| 缺预期话题 | 播放其余话题；健康表逐项标缺失，关键 `/tf`/pose 缺失使 3D 场景判失败 |
| 消息类型不存在/自定义包未装 | DB3 直读标 schema 不可用；建议 MCAP/bridge；扩展不崩溃 |
| Domain ID 不一致 | preflight 列出期望 77/实际值并拒绝回放；不触碰生产 24 |
| QoS 不兼容 | 显示 offered/requested 差异和建议 override；传感器用 best-effort，静态 TF 用 transient-local |
| clock 缺失/停止/跳变/回退 | 按 7.1 处理；seek 被视为合法时间重置，不污染统计 |
| TF 断裂/循环/不存在/外推 | 分类告警，相关 marker 隐藏；其他面板继续 |
| stamp 乱序 | 计数并丢弃对窗口有害的样本；Raw Messages 仍可检查原消息 |
| WS 失败/断开 | 5 秒超时；有限退避；可手动取消；禁止无限 toast |
| 刷新 | 恢复布局/配置，不恢复本地文件权限、不自动播放 |
| 模式切换 | 原子 cleanup；旧 generation 数据不可进入新模式 |
| 多 Bag | 拒绝不同 Bag；允许同一 metadata 的分片有序合并 |
| Bag/磁盘/内存过大 | 预检空间；10 GiB warning、15 GiB hard limit；达到 cache 上限丢旧可视帧而非崩溃 |
| 相机/点云过载 | 相机 2～10 Hz 配置，点云抽点/decay；队列长度 1，优先最新帧 |
| 中文、空格、非 ASCII | argv 数组传参、全路径加引号、UTF-8；E2E fixture 覆盖 |
| 重复点击 | busy gate/幂等状态机；只生成一个 load/player/process group |
| 中途取消 | AbortController + 子进程 SIGINT，1 秒内 UI idle，部分文件保留并标 `.partial` |
| ROS 节点异常退出 | supervisor 收集 exit code；清理同组 bridge/player/recorder，不杀无关 PID |
| 版本不兼容 | 启动时比较 lock；拒绝未验证 major/minor，显示实测矩阵链接 |

## 12. 性能基线和测量方法

基线先在目标 x86_64 操作员机（推荐渲染位置）验收，再在当前 NX 上记录降级数据；如果硬件不同，M0 记录规格并以同样工作负载修正阈值，修正必须评审，不能测试后随意放宽。

| 指标 | 目标 | 测量 |
|---|---:|---|
| Web 首屏（缓存冷/热） | ≤5 s / ≤2 s 到可选数据源 | Playwright navigation/performance entries |
| `.foxe` 面板加载 | ≤1 s | host performance mark |
| LAN WS 建立 | p95 ≤2 s，timeout 5 s | connection state timestamps |
| ROS publish → UI render | Laser/odom p95 ≤250 ms；状态 p95 ≤500 ms | 消息 stamp/receive/render 三点 trace，时钟同步后测 |
| 120 s MCAP 初始可播 | ≤3 s | picker confirmed 到 ready |
| 1.7 GiB 中等 Bag 初始可播 | ≤10 s | 同上；不要求全文件进内存 |
| seek 恢复 | demo p95 ≤500 ms；1.7 GiB p95 ≤2 s | seek action 到关键 3 面板同轴 |
| Camera | 默认 2 Hz demo；实机上限 10 Hz，允许自适应降至 2 Hz | rendered frame counter |
| LaserScan / PointCloud | Scan 10 Hz；PointCloud 默认 5 Hz、队列 1 | render stats/drop count |
| 2 小时内存 | 稳态 30 分钟后 slope <20 MiB/h；切换 20 次回到基线 +15% 内 | heap snapshots/RSS |
| NX bridge CPU/RAM | bridge p95 单核 <35%，RSS <500 MiB（demo） | pidstat/tegrastats |
| 网络 | demo 默认 <20 Mbit/s；启用点云/高码率图像必须显示实际带宽 | interface bytes + topic stats |

## 13. 可直接执行的命令清单

以下命令是实施完成后的合同；当前仓库还没有新包/脚本，因此现在执行其中新增路径会失败。

```bash
# 构建 ROS demo
cd /home/dogrobot/robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select roamerx_patrol_demo
source install/setup.bash

# ROS 单元测试
colcon test --packages-select roamerx_patrol_demo --event-handlers console_direct+
colcon test-result --verbose

# 扩展构建、单测、打包（离线缓存准备完成后）
cd /home/dogrobot/platform/foxglove-extension
npm ci --offline
npm test
npm run build
npm run package

# 一键确定性演示：独立域，不接真实机器狗
cd /home/dogrobot
ROS_DOMAIN_ID=77 robot/script/robot/foxglove_demo.sh all \
  --duration 120 \
  --output /home/dogrobot/runtime/nx-edge/data/rosbags/demo

# 启动发布器和只读 bridge（拆分调试）
ROS_DOMAIN_ID=77 ros2 launch roamerx_patrol_demo indoor_patrol_demo.launch.py
ROS_DOMAIN_ID=77 ros2 launch roamerx_patrol_demo foxglove_readonly.launch.py

# 明确录制 SQLite3 完整目录
ROS_DOMAIN_ID=77 ros2 bag record -s sqlite3 \
  --qos-profile-overrides-path /home/dogrobot/robot/src/tools/roamerx_patrol_demo/config/record_qos.yaml \
  -o /home/dogrobot/runtime/nx-edge/data/rosbags/demo/indoor_patrol_20260828_001 \
  /clock /map /tf /tf_static /odom /cmd_vel /scan \
  /camera/front/image/compressed /imu/data /battery_state /diagnostics \
  /plan /goal_pose /patrol/trajectory /patrol/status

# 检查 Bag（参数必须是目录）
ros2 bag info /home/dogrobot/runtime/nx-edge/data/rosbags/demo/indoor_patrol_20260828_001

# ROS 节点在环回放
ROS_DOMAIN_ID=77 ros2 bag play \
  /home/dogrobot/runtime/nx-edge/data/rosbags/demo/indoor_patrol_20260828_001 \
  --clock 40 --start-paused --rate 1.0

# 受控转换；convert.yaml 由脚本生成并留档
ros2 bag convert \
  -i /home/dogrobot/runtime/nx-edge/data/rosbags/demo/indoor_patrol_20260828_001 \
  -o /home/dogrobot/runtime/nx-edge/data/rosbags/demo/convert.yaml

# Web 离线构建/验证/E2E
cd /home/dogrobot
docker compose -f platform/foxglove-web/docker-compose.yml build --pull=false
bash platform/foxglove-web/scripts/verify-offline-bundle.sh
docker compose -f platform/foxglove-web/docker-compose.yml up -d
cd platform/foxglove-web
npx playwright test tests/foxglove-web.spec.js --trace on
docker compose down

# 现有 Vue 前端回归（仅有入口集成时）
cd /home/dogrobot/platform/frontend
npm test
npm run build
npx playwright test
```

所有长期命令在自动测试中使用 timeout/fixture teardown；人工演示使用脚本的 `stop`，发送 SIGINT 并等待 rosbag 写完 metadata，超时后才 SIGTERM，禁止直接 `kill -9` 导致未完成 Bag。

## 14. 测试金字塔与端到端验收

### 14.1 测试金字塔

| 层级 | 覆盖 | 每次提交命令 | 通过标准 |
|---|---|---|---|
| 单元（数量最多、秒级） | 配置解析、话题映射、bigint 时间、播放状态机、消息适配器、健康窗口、异常值、cleanup；scenario、地图坐标、Bag contract | `cd platform/foxglove-extension && npm test`；`cd robot && colcon test --packages-select roamerx_patrol_demo && colcon test-result --verbose` | 全部退出 0；时间/状态分支覆盖率 ≥90%，其余新增核心模块语句/分支覆盖率均 ≥80% |
| 集成（每个 PR） | demo publisher→bridge→WS；record/info/play/convert；clock/use_sim_time；TF/map/path/image/sensor 同步；模式切换 | `ROS_DOMAIN_ID=77 python3 robot/script/robot/test_foxglove_demo.py`；`cd platform/foxglove-extension && npm run test:integration` | 每个进程有 timeout/teardown；topic count、QoS、clock、schema、清理断言全部通过 |
| E2E（里程碑/发布） | 桌面和 Web 的真实宿主、文件选择、布局、时间轴、异常、刷新、性能 | `npx playwright test --config platform/foxglove-extension/desktop.playwright.config.ts --grep '@desktop-e2e'`；`npx playwright test platform/foxglove-web/tests/foxglove-web.spec.js --grep '@web-e2e' --trace on` | E2E-01～05 全通过，截图/trace/metrics/版本归档；禁止仅靠人工观察判绿 |

集成 fixture 统一由 pytest 创建临时目录和 Domain 77；每个用例最长 180 秒，退出时断言没有本用例 PID、端口、SQLite 文件句柄和 ROS node 残留。损坏 Bag 只由完整 fixture 复制后截断/改 metadata 生成，禁止操作 `/runtime` 原件。

### 14.2 端到端场景

### E2E-01 实时巡逻、录制与历史回放主流程

- [ ] **前置**：断网；Domain 77；桌面 M2 或 Web M3 宿主；扩展和布局已安装；目标目录剩余 ≥2 GiB。
- [ ] **操作**：启动 demo 和 bridge；连接实时源；观察地图、pose、path、goal、status、scan、camera；启动 SQLite3 录制；任务完成后正常停止；运行 info/contract/convert；断开实时；打开 DB3（若兼容门禁通过）或 MCAP；执行 pause/resume/0.5×/2×/seek/step/loop。
- [ ] **预期**：状态按 P1～P4～返回顺序；P2 停 5 秒；P3 绕障告警后恢复；目录有 metadata 和 DB3；所有原生面板共享时间轴。
- [ ] **判定**：最终位姿误差 ≤0.10 m；关键面板相对时间 ≤100 ms；topic counts 在 manifest 范围；无外部请求/账户提示/残留进程。

### E2E-02 实时/回放切换和断连

- [ ] **前置**：实时连接并加载过一个文件。
- [ ] **操作**：live→file→live 各 10 次；连接中双击；播放中刷新；断开 bridge 15 秒后恢复；加载中取消。
- [ ] **预期**：状态机可读、有限重连、每次仅一个源、旧轨迹/TF 不污染新源。
- [ ] **判定**：订阅数、socket、timer 和进程回到基线；取消 ≤1 秒；内存符合第 12 节。

### E2E-03 时间、TF 和 QoS

- [ ] **前置**：生成可注入 clock jump、TF 缺边、错误 frame 和 QoS mismatch 的 10 秒 fixture。
- [ ] **操作**：分别播放 `/clock` 缺失/停止/回退，删除一条静态 TF，制造同 child 多父，以 reliable subscriber 连接 best-effort publisher 对照。
- [ ] **预期**：每类错误独立分类；seek 清窗口；相关 marker 隐藏但宿主不崩溃；QoS 给 offered/requested 建议。
- [ ] **判定**：错误 code、topic/frame、恢复建议与用例快照一致。

### E2E-04 Bag 和文件安全

- [ ] **前置**：临时复制 Bag 后构造缺 metadata、截断 DB3、count 不一致、中文空格目录、越界路径、超限 sparse file。
- [ ] **操作**：info/convert/Web picker/Range endpoint 分别加载；重复 load 并取消。
- [ ] **预期**：合法 Unicode 路径通过；其余在正确层失败；原始 Bag hash 不变；越界 403、超限 413。
- [ ] **判定**：无页面崩溃、无任意文件内容泄露、无半成品 READY。

### E2E-05 中等 Bag 与高带宽降级

- [ ] **前置**：1.7 GiB Bag、模拟 10 Hz PointCloud2/10 Hz camera 流。
- [ ] **操作**：连续回放 2 小时、随机 seek 100 次、后台 tab 5 分钟、前后台切换。
- [ ] **预期**：自动丢旧可视帧，时间轴和状态话题保持；恢复前台后 2 秒内追上当前时间。
- [ ] **判定**：第 12 节性能/内存阈值全部满足，浏览器 watchdog 未触发。

## 15. 风险、规避和回滚

| 风险 | 触发信号 | 规避 | 回滚 |
|---|---|---|---|
| Lichtblick API 与当前 Foxglove API 分叉 | 同一 `.foxe` 无法安装或 context 缺能力 | T0.5 探针；host adapter；只用稳定交集 | 固定 Lichtblick-native API，官方兼容列为非承诺；不推进 Web 前先桌面通过 |
| DB3 直读自定义消息失败 | unsupported datatype | 标准 demo；MCAP 内嵌 schema；bridge | UI 关闭 direct 标识，引导转换/桥接 |
| 大 DB3 使浏览器 OOM | RSS/heap 超基线 | MCAP、Range、单 Bag、缓存上限、点云降采样 | 回退桌面或 bridge 流式路径 |
| Humble player 无 Web seek API | bridge 模式无法拖轴 | 文件 player 满足交互；bridge 只做节点在环 | 不实现危险的 shell HTTP 控制；若业务强需另审 SDK PlaybackControl 服务 |
| 离线依赖不完整 | 断网 build/runtime 失败 | OCI/npm/desktop 包镜像化、SHA、SBOM、断网 CI | 回滚到上一份签名 bundle/digest |
| Web mixed content/CORS | HTTPS 页面 WS 失败 | 同源 Nginx、WSS、严格 Origin | 暂时 LAN HTTP 同源部署，不放宽任意 Origin |
| NX 资源不足 | swap/磁盘/CPU 高 | 操作员机渲染、bridge 按需、空间门禁 | 禁用点云/相机，保持核心状态/Scan |
| 脏工作树冲突 | 路由/脚本已有未提交修改 | 独立 worktree；路径级小提交 | revert 本功能独立提交，不 reset 用户修改 |
| 许可证/产品名称误导 | 把 fork 称为官方 Foxglove | UI/README 明确宿主和许可证 | 移除品牌混用；切换已采购官方许可需单独验收 |

回滚单位：扩展 `.foxe`、Web image digest、layout JSON、bridge launch/config、demo 包分别版本化。升级失败时恢复上一个 digest/foxe/layout；Bag 源目录永远只读，不参与软件回滚。

## 16. 里程碑、工作量与关键依赖

| 里程碑 | 输出 | 估算 | 关键依赖/退出条件 |
|---|---|---:|---|
| M0 兼容门禁 | 三方案报告、宿主/API/离线锁 | 3～5 人日 | 目标桌面 OS、Lichtblick 接受、DB3/MCAP 实测 |
| M1 桌面实时 | `.foxe`、只读 bridge、布局 | 6～8 人日 | M0 API 探针通过 |
| M2 demo + 桌面回放 | 标准 DB3、MCAP、脚本、桌面 E2E | 7～10 人日 | M1；磁盘空间；ROS Humble |
| M3 自托管 Web | 固定镜像、同源入口、Web E2E | 6～9 人日 | **M2 全通过**；Chromium/部署拓扑 |
| M4 加固交付 | 故障/性能报告、离线包、文档 | 4～6 人日 | M3；目标硬件 |

总计 26～38 人日，建议 1 名前端/扩展、1 名 ROS2、1 名测试/部署协作，日历约 4～6 周。关键路径是 T0.5 → M1 → T2.4 → M3；不能用并行 Web 开发绕过桌面门禁。

## 17. Definition of Done

- [ ] 完全断网、无 Foxglove 账户时，桌面端和自托管 Web 均可启动；所有运行制品有锁定版本和 SHA-256。
- [ ] 浏览器稳定连接机器狗只读 ROS2 数据；断连、超时和重连有明确状态。
- [ ] 地图、位姿、轨迹、导航状态和至少 LaserScan/相机之一同步展示；demo 默认二者都展示。
- [ ] 标准 SQLite3 源目录被校验和交付；Web 若直读门禁失败，能经已验证转换或 bridge 稳定回放，且 UI 不谎报直读。
- [ ] MCAP 派生物可解码自定义 schema，并与 DB3 计数、时长、首尾时间一致。
- [ ] 播放、暂停、停止（回到起点）、倍速、跳转、单步、循环和多面板时间同步通过桌面和 Web E2E。
- [ ] `/clock`、log time、`use_sim_time`、TF 和 QoS 的正常/异常测试通过。
- [ ] 确定性室内巡逻脚本无需真机，产生 `metadata.yaml + *.db3` 完整目录，manifest 为 READY。
- [ ] 实时/回放切换 20 次无旧数据污染、重复订阅、残留进程/FD 或超限内存增长。
- [ ] 路径、损坏 Bag、缺 schema、Domain、QoS、clock、TF、WS、大文件和高带宽异常全部有自动化断言且页面不崩溃。
- [ ] 单元、集成、桌面 E2E、Web E2E、2 小时稳定性和断网安装验收均通过并归档报告。
- [ ] 新用户仅按 README 从零完成安装、启动、实时观察、录制、info、转换和回放。
- [ ] 源码、配置、UI、ROS 通信和 Bag 工具分层；无 `eval`、无控制指令、无任意路径读取。
- [ ] 所有关键技术结论都能追溯到仓库、实测报告或第 19 节的一手资料。

## 18. 文档、部署和用户交付清单

- [ ] 桌面安装手册：架构、离线包、扩展安装/升级/回滚、布局导入。
- [ ] Web 部署手册：镜像 digest、端口、Nginx、CSP、WS/WSS、Origin、备份。
- [ ] 用户手册：实时/回放切换、DB3/MCAP 选择、时间轴和每个面板。
- [ ] ROS 手册：Domain、bridge allowlist、QoS、`/clock`、`use_sim_time`、TF。
- [ ] Bag 手册：完整目录、录制、info、校验、转换、manifest、损坏恢复副本流程。
- [ ] demo 手册：参数、巡逻点、期望事件、一键命令、安全停止。
- [ ] 故障排查：错误 code → 原因 → 检查命令 → 恢复动作。
- [ ] 测试报告：单元/集成/E2E/性能/断网，包含版本和硬件。
- [ ] 发布制品：`.foxe`、desktop packages、OCI tar、10 秒 fixture、120 秒 Bag/MCAP、SBOM、NOTICE、SHA256SUMS。
- [ ] 运维清单：日志位置、磁盘水位、端口占用、进程/FD、升级和回滚。

## 19. 一手资料与查询记录

查询日期均为 **2026-08-28**；以下只用于版本/能力判断，实施仍以锁定版本的离线实测为门禁。

- [x] Foxglove 本地文件连接：`.mcap`、`.db3`、多文件和 DB3 消息定义限制：[Local data](https://docs.foxglove.dev/docs/visualization/connecting/local-data)
- [x] Foxglove 文件播放、seek、速率和 loop：[Playback](https://docs.foxglove.dev/docs/visualization/playback)
- [x] Foxglove 扩展和面板 API：[Extensions](https://docs.foxglove.dev/docs/extensions)、[Extension API](https://docs.foxglove.dev/docs/extensions/extension-api)
- [x] Foxglove WebSocket bridge 参数和能力：[Foxglove Bridge](https://docs.foxglove.dev/docs/fleet/bridge)、[foxglove_bridge README](https://github.com/foxglove/foxglove-sdk/blob/main/ros/src/foxglove_bridge/README.md)
- [x] 官方 self-hosted embedded viewer 的定制 Enterprise 限制：[Self-hosted embedded visualization](https://docs.foxglove.dev/docs/embed/self-hosted)
- [x] 官方独立桌面许可和离线激活：[Standalone license](https://docs.foxglove.dev/docs/standalone-license)
- [x] Foxglove 3.0.0 版本记录：[Foxglove v3.0.0](https://docs.foxglove.dev/changelog/foxglove/v3.0.0)
- [x] ROS2 rosbag2 源码、CLI 和存储插件：[rosbag2](https://github.com/ros2/rosbag2)
- [x] MCAP rosbag2 插件和 preset：[rosbag2_storage_mcap](https://github.com/ros-tooling/rosbag2_storage_mcap)
- [x] Lichtblick 代码、许可证、桌面/Web 构建：[Lichtblick repository](https://github.com/lichtblick-suite/lichtblick)
- [x] 锁定宿主发布版：[Lichtblick v1.28.1](https://github.com/lichtblick-suite/lichtblick/releases/tag/v1.28.1)
- [x] 浏览器文件选择的用户手势/安全上下文限制：[File System API](https://developer.mozilla.org/en-US/docs/Web/API/File_System_API)、[Secure contexts](https://developer.mozilla.org/en-US/docs/Web/Security/Secure_Contexts)
- [x] 远端大文件的 Range 行为：[HTTP range requests](https://developer.mozilla.org/en-US/docs/Web/HTTP/Guides/Range_requests)

## 20. 本次计划工作的完成状态

- [x] 已只读分析要求范围内的文档、代码、配置、Bag 元数据、Git 状态和版本。
- [x] 已记录无法使用 `python-docx` 和缺 metadata Bag 的失败与替代方式。
- [x] 已依据用户补充将“离线无账户、桌面先行、Web 后置”写入硬门禁。
- [x] 已给出两端共享的一份实施计划、精确文件路径、任务依赖、测试命令、验收和 DoD。
- [x] 未编写业务代码、未启动长期服务、未修改原始文档或现有业务文件。
