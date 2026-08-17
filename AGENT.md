# RoamerX 部署目录对应表

本文件记录远端 `main` 与两块板卡的部署关系，供后续发布和排障使用。

> 本地开发根目录已迁移到 `/home/main`。本目录
> `/home/robot/genisom_roamerx_open` 仅保留为 NX 现役运行工作区；不要再在此处开展新功能开发。

## 基线与约定

- 基线：`github-private/main`，提交 `f65b8449c7bd66bb6523d2c14613f95559487b29`（2026-08-17 已用 `git ls-remote` 核验）。当前本地 `main` 跟踪该远端；不要将不同步的 `origin/main` 当作此表基线。
- NX：`robot@ubuntu-desktop`，工作区根目录为 `/home/robot/genisom_roamerx_open`。
- 3588：`firefly@192.168.234.1`（SSH 别名 `3588`）。它运行厂商运动控制镜像；**不要**把完整仓库、ROS 工作区或 Edge Agent 同步到 3588。
- “源码目录”是远端 Git 中的目录；“运行目录”是板上实际服务使用的目录。`-` 与 `_` 的目录名不可混用。
- 以下状态于 2026-08-17 只读核验：NX 存在工作区、`/home/robot/edge_agent`、`dev_agent` 和 `mcp_server`；3588 仅存在充电桩资产与安装后的两个程序，未部署仓库工作区或 Edge Agent。

## 顶层目录

| 远端 `main` 目录 | NX 部署/用途 | 3588 部署/用途 | 发布要点 |
| --- | --- | --- | --- |
| `robot/` | 内容平铺到 `/home/robot/genisom_roamerx_open/`；例如 `robot/src/` -> `/home/robot/genisom_roamerx_open/src/`，并在此目录生成 `build/`、`install/`、`log/`。 | 不部署完整目录。只有下表列出的充电桩资产和启动器需要上板。 | `deploy/robot/deploy.sh` 同步此目录（排除构建产物），`--build` 在 NX 执行 `./build.sh all`。 |
| `edge-agent/` | 现行运行目录：`/home/robot/edge_agent/`；配置、SQLite 状态和日志留在此目录，不可被源码覆盖。 | 不部署。NX Edge Agent 通过 SSH 管理 3588 的服务、音频、充电与遥测。 | 包装脚本默认同步到 `/home/robot/edge_agent/`，但仓库内样例服务使用 `/opt/roamerx/edge_agent`；实机当前使用前者，更新 unit 前不要切换到后者。 |
| `dev-agent/` | 服务实际使用 `/home/robot/genisom_roamerx_open/dev_agent/`（历史下划线目录）。 | 不部署；它只从 NX 通过 SSH 读取 3588 的麦克风。 | 没有自动部署包装脚本；发布时把 `dev-agent/` 的内容复制到现行 `dev_agent/` 目录，并保留 `/home/robot/.local/state/roamerx-dev-agent/`。 |
| `mcp_server/` | 顶层目录当前仅含 MCP 测试；不作为运行代码部署。 | 不部署。 | 运行代码的来源是 `robot/mcp_server/`，见下表。 |
| `platform/` | 不部署到 NX（开发/测试副本可以存在）。 | 不部署。 | 云端路径为 `/opt/roamerx/current/{backend,frontend}`；由 `deploy/cloud/deploy.sh` 调用 `platform/scripts/deploy_cloud_platform.sh` 发布。 |
| `deploy/` | 不常驻部署；从仓库工作区执行发布包装脚本。 | 不部署。 | `robot/deploy.sh` 管 NX，`cloud/deploy.sh` 管云端；`controller/README.md` 说明 3588 使用厂商镜像。 |
| `scripts/` | 不常驻部署；例如在检出仓库中执行 `scripts/test_agents.sh`。 | 不部署。 | 仅开发/验证工具。 |
| `backup/` | 不部署；私有配置快照，供人工比对/选择性恢复。 | 不部署；其中 `3588_*` 是 3588 的备份，不是发布包。 | 禁止直接解压覆盖运行中的设备，且不得提交或外传其中可能含有的凭据。 |
| `patrol_data/` | 不部署；测试 rosbag 数据。 | 不部署。 | 真实地图位于 NX `/home/robot/.jszr/map/`，真实 rosbags 不在 Git。 |

## 3588 例外资产（唯一需要从仓库侧更新的内容）

| 源码/资产路径（远端 `main`） | 3588 目标路径 | 现机核验与说明 |
| --- | --- | --- |
| `robot/third_party/charge_pile_3588/` | `/home/firefly/charge_pile_v1.0.3b/charge_pile_xg_lib_v1.0.3b/` | 已存在。此为厂商充电桩包的逐字节备份；复制时保留 ARM64 二进制的可执行权限。 |
| `robot/script/robot/charge_pile_controller.sh` | `/usr/local/sbin/roamerx-charge-pile-controller` | 已存在。服务据此读取 `/var/lib/roamerx-charge-pile/state` 并执行 `dog_send_three_states/` 中的厂商二进制。 |
| `robot/systemd/roamerx-charge-pile.service` | `/etc/systemd/system/roamerx-charge-pile.service` | 已存在，当前 `active` 但 `disabled`；变更后须在 3588 上执行 `systemctl daemon-reload`，只重启该服务。 |
| `edge-agent/tools/leg_power/{leg_power.cpp,power_mcu.proto,install_on_3588.sh}` | 构建后为 `/usr/local/bin/roamerx-leg-power` | 已存在。必须在 3588 此目录中运行安装脚本（需 `protoc`、eCAL、protobuf）；不要将 NX 编译产物跨架构复制。 |

3588 端已核验不存在：`/home/robot/genisom_roamerx_open`、`/opt/roamerx/edge_agent`。运动控制、`robot-launch`、相机、RKNN、`spline_daemon` 等均为厂商镜像管理范围，不由本仓库构建或覆盖。

## NX 服务与运行路径

| 功能 | NX 代码/启动路径 | 配置与持久数据 | 对应服务 |
| --- | --- | --- | --- |
| ROS 导航、定位、建图 | `/home/robot/genisom_roamerx_open/{src,script,install}` | 地图：`/home/robot/.jszr/map/`；构建产物：工作区的 `{build,install,log}` | 由 Edge Agent 与 `script/robot/start_*_real.sh` 生命周期管理。 |
| Edge Agent | `/home/robot/edge_agent/`（现机 unit 的 `WorkingDirectory`；其启动脚本会引用工作区 ROS 产物） | `/home/robot/edge_agent/config.yaml`、`data/`、RTK 配置 | `roamerx-edge-agent.service` |
| 常驻遥控桥 | ROS 二进制来自 `/home/robot/genisom_roamerx_open/install/robot_navigo/` | 无独立源码目录 | `roamerx-teleop-bridge.service` |
| Dev Agent / 本地语音 | `/home/robot/genisom_roamerx_open/dev_agent/` | 会话状态：`/home/robot/.local/state/roamerx-dev-agent/`；语音模型：`/home/robot/.local/share/roamerx-voice/` | `roamerx-dev-agent.service`、`roamerx-local-asr.service` |
| Robot MCP | `/home/main/robot/mcp_server/`（现役 service 通过本机 override 从此启动） | `/etc/roamerx/robot-mcp.env`（仅环境变量，不入库） | `roamerx-robot-mcp.service`；`127.0.0.1:8095/mcp` |

## 发布防护

1. 先确认目标：导航/ROS/Edge/Dev/MCP 变更默认在 NX；3588 只限上一节列出的资产。
2. 不同步 NX 的 `/home/robot/edge_agent/config.yaml`、`data/`、地图、rosbag、`build/`、`install/`、`log/` 或任何密钥。
3. `deploy/robot/deploy.sh` 当前只同步 `robot/` 和 `edge-agent/`；它**不会**发布 `dev-agent/`、MCP、3588 充电桩资产或云平台。
4. 远端样例 `edge-agent/systemd/roamerx-edge-agent.service` 指向 `/opt/roamerx/edge_agent` 与 `/etc/roamerx/edge-agent.yaml`，但当前 NX unit 指向 `/home/robot/edge_agent`。二者属于不同代部署布局；先显式迁移服务和数据，再采用 `/opt` 布局，不能混用。
5. Robot MCP 的运行路径由 `/etc/systemd/system/roamerx-robot-mcp.service.d/override.conf` 覆盖为 `/home/main/robot/mcp_server/roamerx_robot_mcp.py`；不要恢复为旧工作区中缺失的 `mcp_server/roamerx_robot_mcp.py`。
