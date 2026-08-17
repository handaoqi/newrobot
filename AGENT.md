# RoamerX 本地开发与部署目录

## 本地开发基线

- 所有后续本地开发均在 `/home/dogrobot` 进行；它是远端 `github-private/main` 的独立检出，并应保持跟踪远端 `main`。
- 保持仓库原始结构，直接在 `/home/dogrobot/{robot,edge-agent,dev-agent,platform,...}` 中修改；不要再把开发源码放入旧的运行目录。
- 常用命令：`cd /home/dogrobot && git pull --ff-only`、`git status`、`scripts/test_agents.sh`。
- 全量构建、发布、数据恢复与验收流程见 [PROJECT_DEPLOYMENT_MANUAL.md](PROJECT_DEPLOYMENT_MANUAL.md)。
- `/home/robot/genisom_roamerx_open`、`/home/robot/edge_agent` 仍是**现役 NX 运行目录**。在明确迁移 systemd、配置和 ROS 产物之前，不移动、不删除、不把服务路径改到 `/home/dogrobot`。

## 顶层目录对应

| `/home/dogrobot` 源码目录 | NX 运行/发布目标 | 3588 运行/发布目标 | 说明 |
| --- | --- | --- | --- |
| `robot/` | 内容发布至 `/home/robot/genisom_roamerx_open/`；`robot/src/` 对应其 `src/` | 不发布完整 ROS 工作区 | 在 NX 构建产生 `{build,install,log}`。 |
| `edge-agent/` | 当前服务目录 `/home/robot/edge_agent/`，保留 `config.yaml`、`data/` 和日志 | 不发布 | NX Edge Agent 通过 SSH 控制 3588。 |
| `dev-agent/` | 当前服务目录 `/home/robot/genisom_roamerx_open/dev-agent/` | 不发布 | 无自动发布脚本；保留 `/home/robot/.local/state/roamerx-dev-agent/`。 |
| `mcp_server/` | 顶层目录仅含测试，不发布运行代码 | 不发布 | 运行代码来自 `robot/mcp_server/`；现役 MCP service 从 `/home/dogrobot/robot/mcp_server/` 启动。 |
| `platform/` | 不发布到 NX | 不发布 | 发布到云端 `/opt/roamerx/current/{backend,frontend}`。 |
| `deploy/`、`scripts/` | 从 `/home/dogrobot` 执行，不常驻部署 | 不发布 | `deploy/robot/deploy.sh` 只发布 `robot/` 与 `edge-agent/`。 |
| `backup/`、`patrol_data/` | 不发布 | 不发布 | 分别为私有备份和测试 rosbag；禁止覆盖设备。 |

## 3588 的例外资产

3588 (`firefly@192.168.234.1`，别名 `3588`) 使用厂商运动控制镜像。只更新以下仓库管理资产，**不要**同步 `/home/dogrobot`、完整 ROS 工作区或 Edge Agent：

| `/home/dogrobot` 来源 | 3588 目标 |
| --- | --- |
| `robot/third_party/charge_pile_3588/` | `/home/firefly/charge_pile_v1.0.3b/charge_pile_xg_lib_v1.0.3b/`（保留 ARM64 可执行权限） |
| `robot/script/robot/charge_pile_controller.sh` | `/usr/local/sbin/roamerx-charge-pile-controller` |
| `robot/systemd/roamerx-charge-pile.service` | `/etc/systemd/system/roamerx-charge-pile.service` |
| `edge-agent/tools/leg_power/` | 在 3588 构建并安装为 `/usr/local/bin/roamerx-leg-power` |

## 发布防护

1. 开发、测试和提交只在 `/home/dogrobot`；发布时使用明确目标路径，绝不以目录名猜测目标。
2. 不覆盖 NX 的地图、rosbag、`/home/robot/edge_agent/config.yaml`、`data/`、密钥、`build/`、`install/` 或 `log/`。
3. 仓库内 Edge Agent 样例 unit 指向 `/opt/roamerx/edge_agent`，而现机 unit 指向 `/home/robot/edge_agent`；这是两代布局。未完成显式迁移前，继续使用现机路径。
4. 修改 3588 systemd unit 后，先核对目标文件，再执行 `systemctl daemon-reload`，只重启受影响服务。
5. `roamerx-robot-mcp.service` 的本机 override 为 `/etc/systemd/system/roamerx-robot-mcp.service.d/override.conf`，将它指向 `/home/dogrobot/robot/mcp_server/roamerx_robot_mcp.py`，并仅监听 `127.0.0.1:8095`（MCP endpoint: `/mcp`）。
