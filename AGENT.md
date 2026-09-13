# RoamerX 本地开发与部署目录

## 本地开发基线

- 所有后续本地开发均在 `/home/dogrobot` 进行；它是远端 `github-private/main` 的独立检出，并应保持跟踪远端 `main`。
- 保持仓库原始结构，直接在 `/home/dogrobot/{robot,edge-agent,dev-agent,platform,...}` 中修改；不要再把开发源码放入旧的运行目录。
- 常用命令：`cd /home/dogrobot && git pull --ff-only`、`git status`、`scripts/test_agents.sh`。
- 全量构建、发布、数据恢复与验收流程见 [PROJECT_DEPLOYMENT_MANUAL.md](PROJECT_DEPLOYMENT_MANUAL.md)。
- `/home/dogrobot` 是唯一源码、Git 检出和 ROS 构建目录。systemd 服务也必须从该目录加载代码。
- 项目级架构分析和人工操作手册统一维护在 `/home/dogrobot/docs`；组件 README 只保留局部说明并链接到该目录。
- 三端运行时统一由 `/home/dogrobot/runtime` 描述；NX 的地图、rosbag、模型、Edge 状态和设备配置实际存放在 `runtime/nx-edge/{data,conf,install}`。
- `/home/robot` 仅保留厂商软件、开发用户状态和兼容软链接，不再作为本项目运行数据的主目录。

## 统一运行时

三个运行时都必须包含 `bin/data/conf/scripts/docs/install`，职责不可混放：

| 运行时 | 本机目录 | 部署目标 |
| --- | --- | --- |
| 云平台 | `runtime/platform` | `/opt/roamerx/runtime/platform` |
| NX Edge | `runtime/nx-edge` | `/home/dogrobot/runtime/nx-edge` |
| 3588 运控覆盖层 | `runtime/3588-motion` | `/home/firefly/dogrobot-runtime` |

- `bin` 只放稳定运维入口，`scripts` 放安装/迁移/部署工具。
- `data` 和真实 `conf` 不提交 Git；仅提交 `.gitkeep`、README 和 example。
- `install` 保存可重复安装清单、模型和厂商原始安装包；系统包版本快照见 `runtime/nx-edge/install/installed-system-packages.lock`。
- NX 旧路径（如 `/home/robot/.jszr`、`/home/robot/rosbags`）只是兼容链接，新代码必须使用 runtime 主路径。

## 顶层目录对应

| `/home/dogrobot` 源码目录 | NX 运行/发布目标 | 3588 运行/发布目标 | 说明 |
| --- | --- | --- | --- |
| `robot/` | 直接在 `/home/dogrobot/robot/` 构建和运行 | 不发布完整 ROS 工作区 | 构建产物位于 `robot/{build,install,log}`。 |
| `edge-agent/` | 源码从 `/home/dogrobot/edge-agent/` 运行；配置和数据位于 `runtime/nx-edge/{conf,data}` | 不发布 | NX Edge Agent 通过 SSH 控制 3588。 |
| `dev-agent/` | 源码从 `/home/dogrobot/dev-agent/` 运行 | 不发布 | Codex 主会话归档在 `runtime/nx-edge/data/codex/sessions/`，恢复说明见 `runtime/nx-edge/docs/CODEX_SESSION_BACKUP.md`。 |
| `mcp_server/` | 顶层目录仅含测试，不发布运行代码 | 不发布 | 运行代码来自 `robot/mcp_server/`；现役 MCP service 从 `/home/dogrobot/robot/mcp_server/` 启动。 |
| `platform/` | 不发布到 NX | 不发布 | Docker 方式发布到云端 `/opt/roamerx/source/platform`，状态在 `/opt/roamerx/runtime/platform`。 |
| `deploy/`、`scripts/` | 从 `/home/dogrobot` 执行，不常驻部署 | 不发布 | `deploy/nx-edge/deploy.sh` 发布 `robot/` 与 `edge-agent/`；`deploy/3588-motion/` 只发布 3588 运控覆盖层。 |
| `backup/`、`patrol_data/` | 不发布 | 不发布 | 分别为私有备份和测试 rosbag；禁止覆盖设备。 |

## 3588 的例外资产

3588 (`firefly@192.168.234.1`，别名 `3588`) 使用厂商运动控制镜像。只更新以下仓库管理资产，**不要**同步 `/home/dogrobot`、完整 ROS 工作区或 Edge Agent：

| `/home/dogrobot` 来源 | 3588 目标 |
| --- | --- |
| `runtime/3588-motion/install/charge-pile/` | `/home/firefly/dogrobot-runtime/install/charge-pile/`（保留 ARM64 可执行权限） |
| `robot/script/robot/charge_pile_controller.sh` | `/usr/local/sbin/roamerx-charge-pile-controller` |
| `robot/systemd/roamerx-charge-pile.service` | `/etc/systemd/system/roamerx-charge-pile.service` |
| `edge-agent/tools/leg_power/` | 在 3588 构建并安装为 `/usr/local/bin/roamerx-leg-power` |

## 发布防护

1. 开发、测试和提交只在 `/home/dogrobot`；发布时使用明确目标路径，绝不以目录名猜测目标。
2. 不覆盖 `runtime/*/data`、真实 `runtime/*/conf`、地图、rosbag、模型缓存、密钥或 Codex 会话；Codex 会话只能使用归档/恢复脚本迁移。
3. 修改服务后，确保代码指向 `/home/dogrobot`，配置和状态指向对应 runtime，禁止重新写回旧 `/home/robot` 数据目录。
4. 修改 3588 systemd unit 后，先核对目标文件，再执行 `systemctl daemon-reload`，只重启受影响服务。
5. `roamerx-robot-mcp.service` 的本机 override 为 `/etc/systemd/system/roamerx-robot-mcp.service.d/override.conf`，将它指向 `/home/dogrobot/robot/mcp_server/roamerx_robot_mcp.py`，并仅监听 `127.0.0.1:8095`（MCP endpoint: `/mcp`）。

## 平台 Docker 发布

生产 Compose 位于 `runtime/platform/compose.yaml`，包含 API、device worker、scheduler、frontend、Mosquitto 和 ZLMediaKit。使用 `runtime/platform/bin/platformctl` 管理；首次部署先执行 `init`、填写 `conf/platform.env` 并生成 MQTT 密码文件。不得让旧 systemd 平台和 Docker 平台同时连接同一数据库及 MQTT 身份。
