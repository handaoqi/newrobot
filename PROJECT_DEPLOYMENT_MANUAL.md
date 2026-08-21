# RoamerX 全项目构建、部署与数据恢复说明书

> 适用范围：私有仓库 `main`、云平台、NVIDIA Jetson Orin NX（以下简称 **NX**）和 Firefly RK3588（以下简称 **3588**）。
>
> 本说明书基于 2026-08-17 的只读现场核验编写。它描述“重建/发布/恢复”的受控流程，**不应在机器人处于可运动状态时直接照抄执行**。所有密码、Token、MQTT 凭据、私钥、Wi-Fi、RTK/NTRIP 及证书均只从受控密码库或原机安全备份恢复，禁止写入 Git 或本文档。

## 1. 目标、范围和当前基线

### 1.1 目标

从私有仓库导出同一份工程到 `/home/dogrobot`，以该目录作为唯一的本地开发与发布源；恢复并验证：

1. 云端 Django/Vue/MQTT 平台与 HTTPS 入口；
2. NX 上的 ROS 2 导航、Edge Agent、开发助手、语音和 Robot MCP；
3. 3588 上的厂商运控环境，以及本项目维护的充电桩与腿部电源增量组件；
4. 云端数据库/媒体、NX 地图/运行配置、3588 设备配置等持久数据。

### 1.2 源码基线

| 项目 | 值 |
| --- | --- |
| 私有远端 | `git@github.com:talent5978/dogrobot.git` |
| 分支 | `main` |
| 本次核验提交 | `f65b8449c7bd66bb6523d2c14613f95559487b29` |
| 开发根目录 | `/home/dogrobot` |
| 本地开发原则 | 仅在 `/home/dogrobot` 修改、测试、提交；不在现役运行目录直接开发。 |

### 1.3 三端拓扑

```text
浏览器
  │ HTTPS :443
  ▼
39.107.250.69 (cloud)
  nginx ──> Django API :8081 ──> /opt/roamerx/shared/db.sqlite3
  │                              /opt/roamerx/shared/media
  └── MQTT :1884
            │
            ▼
NX (robot@ubuntu-desktop) ── SSH/ROS/Zenoh ──> 3588 (firefly@192.168.234.1)
  Edge Agent / ROS 2 / MCP                         厂商 robot-launch / 运控 / 传感器
```

真实业务链路为：

```text
Browser -> Django API -> device worker -> MQTT
-> NX Edge Agent -> Nav2 -> /cmd_vel -> SDK bridge -> 3588 motion controller
```

### 1.4 目录和职责

| `/home/dogrobot` 目录 | 发布目标 | 是否完整部署到 3588 |
| --- | --- | --- |
| `platform/` | 云端 `/opt/roamerx/current/{backend,frontend}` | 否 |
| `robot/` | NX `/home/dogrobot/robot/`（原地构建和运行） | 否 |
| `edge-agent/` | NX `/home/dogrobot/edge-agent/`，运行数据在 `/home/robot/edge_agent/` | 否 |
| `dev-agent/` | NX `/home/dogrobot/dev-agent/`，会话状态在 `/home/robot/` | 否 |
| `robot/mcp_server/` | NX `/home/dogrobot/robot/mcp_server/` | 否 |
| `robot/third_party/charge_pile_3588/` | 3588 充电桩厂商包目录 | 是，仅此增量资产 |
| `edge-agent/tools/leg_power/` | 3588 本机编译为 `/usr/local/bin/roamerx-leg-power` | 是，仅此增量组件 |
| `backup/`、`patrol_data/` | 不作为发布包 | 否 |

## 2. 现场基线快照

以下是编写本说明书时的实际状态，用于恢复后的比对；状态会随运行模式变化，尤其是 3588 的 `robot-launch` eggs 和充电桩服务。

| 环境 | 已核验状态 |
| --- | --- |
| 云端 | Ubuntu 22.04；`/opt/roamerx/current -> /opt/roamerx/releases/20260816_person_follow_fix`；中心 API、device worker、scheduler、nginx 为 `active`。 |
| HTTPS | `https://39.107.250.69/` 与 `https://39.107.250.69/api/maps/` 均返回 HTTP 200。 |
| 云端持久数据 | `/opt/roamerx/shared/db.sqlite3` 约 2.4 GB、`media/` 约 756 MB、`logs/` 约 84 MB。 |
| 云端 MQTT | `0.0.0.0:1884` 有 `mosquitto` 进程监听；但 `mosquitto.service` 显示 `inactive`。恢复前必须核对该进程的监管方式与配置，不能仅按 unit 状态假定 Broker 不可用。 |
| NX | Ubuntu 22.04；`roamerx-edge-agent`、`roamerx-teleop-bridge`、`roamerx-local-asr`、`roamerx-robot-mcp`、`roamerx-zenoh` 为运行状态；`roamerx-dev-agent` 可能处于启动中。 |
| 3588 | Ubuntu 22.04；厂商 `robot-launch` 可查询；充电桩资产、`roamerx-leg-power`、`roamerx-charge-pile-controller` 已存在。充电桩服务是否启动取决于充电/散热运行模式。 |

## 3. 通用安全规则与准备

1. 先做备份并核验，再停止服务或替换文件；从不以 `--delete`、递归删除或直接解压备份的方式覆盖运行系统。
2. 真实机器狗恢复前，确认机器人已进入批准的安全维护状态；不得通过 `kill` 绕过受保护的运控/遥控服务。
3. 每个服务仅加载受控环境文件，例如云端 `/opt/roamerx/shared/center.env`、NX `/home/robot/edge_agent/config.yaml` 和 `/etc/roamerx/robot-mcp.env`。示例配置只能作为字段模板。
4. 启用或重启服务前先检查路径、所有者和端口；恢复后先做只读健康检查，不发送移动、建图、充电或姿态指令。
5. 任何配置恢复均应在临时目录解包后逐文件比对。设备网络、蓝牙配对、SSH、代理、MQTT、RTK/NTRIP 配置可能含有密钥且与设备绑定。

建议在三个环境均保留恢复记录：提交 SHA、备份 SHA-256、开始/结束时间、执行人、服务状态、验证结果和回滚点。

## 4. 从私有仓库导出工程到 `/home/dogrobot`

### 4.1 首次检出

在具备私有仓库 SSH 权限的 NX 或开发机执行：

```bash
sudo install -d -o dogrobot -g robot -m 0755 /home/dogrobot
git clone --branch main --single-branch \
  git@github.com:talent5978/dogrobot.git /home/dogrobot
cd /home/dogrobot
git rev-parse HEAD
git status --short --branch
```

预期：工作目录跟踪 `origin/main`，且初始提交为本说明书所记录的 SHA 或经审查后的更高版本。

### 4.2 日常更新与发布前冻结

```bash
cd /home/dogrobot
git fetch origin main
git status --short
git pull --ff-only
git rev-parse HEAD
```

发布前记录 SHA，避免把未提交的本地改动带入 NX 或云端。生产数据、`config.yaml`、密钥、`build/`、`install/`、`log/`、地图与 rosbag 均不应放入 Git。

### 4.3 先执行的本地校验

```bash
cd /home/dogrobot
scripts/test_agents.sh

cd platform/backend
python3 manage.py test monitoring

cd ../frontend
npm ci
npm test
npm run build
```

测试失败时不要继续发布。云端当前没有 Node.js，因此前端必须在 `/home/dogrobot/platform/frontend` 或 CI 中构建后再同步。

## 5. 云平台：构建、配置、服务和数据

### 5.1 当前目标机和运行路径

| 项目 | 当前值 |
| --- | --- |
| SSH 目标 | `root@39.107.250.69`（别名 `cloud`） |
| 公网入口 | `https://39.107.250.69/` |
| 当前 release | `/opt/roamerx/releases/20260816_person_follow_fix` |
| 当前软链接 | `/opt/roamerx/current` |
| 后端源码 | `/opt/roamerx/current/backend` |
| 前端静态文件 | `/opt/roamerx/current/frontend/dist` |
| 环境文件 | `/opt/roamerx/shared/center.env` |
| SQLite 数据库 | `/opt/roamerx/shared/db.sqlite3` |
| 媒体文件 | `/opt/roamerx/shared/media` |
| 日志 | `/opt/roamerx/shared/logs` |
| Python | `/root/miniconda/envs/py310/bin/python`（现场为 Python 3.10） |

### 5.2 需要的软件包和运行时

云端为 Ubuntu 22.04。以下是重建时的基线，安装前应先与现有主机和组织镜像源核对：

```bash
sudo apt update
sudo apt install -y \
  git rsync curl ca-certificates sqlite3 \
  nginx mosquitto mosquitto-clients \
  openssl python3-venv
```

当前服务使用 Miniconda 的 `py310` 环境，而非系统 Python。创建或恢复该环境后安装后端依赖：

```bash
/root/miniconda/bin/conda create -y -n py310 python=3.10
/root/miniconda/envs/py310/bin/python -m pip install --upgrade pip
/root/miniconda/envs/py310/bin/python -m pip install \
  -r /opt/roamerx/current/backend/requirements.txt
```

后端依赖由 `platform/backend/requirements.txt` 定义，包括 Django、DRF、`paho-mqtt`、`faster-whisper`、Pillow、`psycopg` 等。当前生产数据库是 SQLite；`psycopg` 和 `POSTGRES_*` 仅为可选 PostgreSQL 方案，不能在恢复现网时误设置 `POSTGRES_DB`。

前端依赖以 `/home/dogrobot/platform/frontend/package-lock.json` 为准，使用 Node.js/npm 在**构建机**执行 `npm ci && npm run build`。不要在当前云端假定存在 Node.js/npm。

### 5.3 环境文件

从 `platform/deploy/center.env.example` 复制字段名称，但不要覆盖生产环境文件或照搬其中的占位值。当前 SQLite 布局至少需要保留以下非敏感字段关系：

```dotenv
DJANGO_DEBUG=false
DJANGO_ALLOWED_HOSTS=39.107.250.69
DJANGO_CSRF_TRUSTED_ORIGINS=https://39.107.250.69
PUBLIC_BASE_URL=https://39.107.250.69
SQLITE_DB_PATH=/opt/roamerx/shared/db.sqlite3
DJANGO_MEDIA_ROOT=/opt/roamerx/shared/media
```

`DJANGO_SECRET_KEY`、全部 `MQTT_*` 凭据/证书路径、设备密钥及任何数据库口令必须由密码库或原 `center.env` 恢复。现场 Broker 端口为 `1884`；实际 TLS、认证和上游设置必须以现役 `center.env` 与 Broker 配置为准，不要因模板默认 `8883` 而直接改写。

推荐权限：

```bash
sudo install -d -m 0750 /opt/roamerx/shared/{media,logs}
sudo chmod 0640 /opt/roamerx/shared/center.env
```

所有者应与实际 systemd 服务用户一致；先用 `systemctl show -p User <unit>` 和 `stat` 核对，不要盲目改为某个固定用户。

### 5.4 构建与发布

正式发布入口为：

```bash
cd /home/dogrobot
deploy/cloud/deploy.sh
```

该入口会调用 `platform/scripts/deploy_cloud_platform.sh`，在本机构建 `platform/frontend`，同步：

- `frontend/dist/` -> `cloud:/opt/roamerx/current/frontend/dist/`；
- `backend/` -> `cloud:/opt/roamerx/current/backend/`（不含 Python 缓存与 `data/`）；
- 执行 Django 迁移并重启 API 与 device worker。

可用 `--frontend-only` 或 `--backend-only` 缩小变更面。发布脚本不发布 NX/3588，也不应同步 `/opt/roamerx/shared` 的数据库、媒体、日志或环境文件。

新建 release 时，先在独立目录准备代码和依赖，例如：

```text
/opt/roamerx/releases/<timestamp>_<short-sha>/
  backend/
  frontend/dist/
```

完成迁移、健康检查和人工批准后才原子切换 `/opt/roamerx/current` 软链接。切换前必须记录旧 release 路径，作为回滚点。

### 5.5 云端 systemd、Nginx 和 MQTT

当前服务定义的核心命令如下：

| 服务 | 工作目录与启动命令 | 说明 |
| --- | --- | --- |
| `roamerx-center-api.service` | `/opt/roamerx/current`；`python backend/manage.py runserver 127.0.0.1:8081` | 仅监听本机，由 Nginx 反代。 |
| `roamerx-device-worker.service` | `/opt/roamerx/current`；`python backend/manage.py run_device_worker` | 负责 MQTT 命令、ACK、结果和在线状态。 |
| `roamerx-patrol-scheduler.service` | `/opt/roamerx/current`；`python backend/manage.py run_patrol_scheduler --interval 10` | 巡检日历调度。 |
| `nginx.service` | HTTPS :443、HTTP :80、平台 HTTP :8088 | `/api/` 反代到 `127.0.0.1:8081`，静态根为 `frontend/dist`。 |
| MQTT Broker | 现场有 `mosquitto` 在 :1884 监听 | 先确定它由哪个 unit/容器/进程监管；不要在未确认前同时启动第二个 Broker。 |

Nginx 已配置 `39.107.250.69` 的 Let's Encrypt IP 证书，HTTPS 站点根为 `/opt/roamerx/current/frontend/dist`，并反代 `/api/`、`/media/`、`/live/`。任何恢复后应先运行：

```bash
ssh cloud 'nginx -t && systemctl status nginx --no-pager'
curl --noproxy '*' -fsSI https://39.107.250.69/
curl --noproxy '*' -fsSI https://39.107.250.69/api/maps/
```

常规启停和状态检查：

```bash
ssh cloud 'systemctl status \
  roamerx-center-api roamerx-device-worker roamerx-patrol-scheduler nginx --no-pager'

ssh cloud 'systemctl restart \
  roamerx-center-api roamerx-device-worker roamerx-patrol-scheduler'
```

Broker 恢复步骤必须另行确认：先用 `ss -ltnp | grep :1884`、`ps -fp <pid>` 和 `systemctl status mosquitto` 找到当前监管源，再仅修复那一处。当前“监听存在但 unit inactive”是应处理的运维偏差。

### 5.6 云端数据备份和恢复

Git 不含业务数据。恢复对象是：

```text
/opt/roamerx/shared/db.sqlite3
/opt/roamerx/shared/media/
/opt/roamerx/shared/logs/        # 仅用于排障，不是业务恢复的权威数据
/opt/roamerx/shared/center.env   # 受控机密，不进入普通备份包
```

**备份**（维护窗口外可使用 SQLite 在线备份；大量写入期间仍建议安排窗口）：

```bash
ssh cloud 'set -e
  stamp=$(date +%Y%m%d_%H%M%S)
  mkdir -p /opt/roamerx/backups/$stamp
  sqlite3 /opt/roamerx/shared/db.sqlite3 ".backup /opt/roamerx/backups/$stamp/db.sqlite3"
  rsync -aHAX --numeric-ids /opt/roamerx/shared/media/ /opt/roamerx/backups/$stamp/media/
  sha256sum /opt/roamerx/backups/$stamp/db.sqlite3 > /opt/roamerx/backups/$stamp/SHA256SUMS'
```

**恢复**（必须有批准的备份副本和维护窗口）：

1. 在云端以只读方式校验 SHA-256、`sqlite3 <db> 'PRAGMA quick_check'`，并将数据库和媒体解压/同步到临时目录；绝不直接解压到 `shared/`。
2. 记录当前 release 与数据快照；停止 API、worker、scheduler，确保没有 SQLite 写入者。
3. 保留现有 `db.sqlite3` 和 `media/` 的带时间戳副本，再将已验证的数据拷入实际路径。媒体使用不带 `--delete` 的 `rsync -aHAX --numeric-ids`，避免误删新文件。
4. 按恢复前的 `stat` 还原所有者和权限；执行 `python backend/manage.py migrate --plan`，确认后才执行 `migrate --noinput`。
5. 依次启动 API -> worker -> scheduler -> Nginx，检查 HTTPS、`/api/maps/`、管理端登录、设备 MQTT 连接与媒体访问。

数据库、媒体和代码必须保持同一兼容窗口；数据库来自较新版本时，不得回滚到不兼容的旧代码。

## 6. NX：ROS、Edge Agent、开发助手和 MCP

### 6.1 目标路径与现役布局

| 用途 | 路径 |
| --- | --- |
| 开发源 | `/home/dogrobot` |
| ROS 源码、构建与运行工作区 | `/home/dogrobot/robot` |
| Edge 运行配置与状态 | `/home/robot/edge_agent/config.yaml`、`/home/robot/edge_agent/data/` |
| 地图 | `/home/robot/.jszr/map/` |
| Dev Agent 现役目录 | `/home/dogrobot/dev-agent/` |
| Robot MCP 现役代码 | `/home/dogrobot/robot/mcp_server/` |
| Robot MCP endpoint | `http://127.0.0.1:8095/mcp` |

NX 为 Ubuntu 22.04 + ROS 2 Humble + Jetson/厂商驱动环境。完整传感器、Livox、RTK、SDK、Zenoh、GPU、设备 udev 规则及运行配置不是仅靠仓库即可从零生成；新机必须先使用批准的 NX 基础镜像或配置备份，再部署本项目源码。

### 6.2 ROS 与构建依赖

先按 ROS 2 官方 Ubuntu 22.04/Humble 流程安装 `ros-humble-desktop`，随后在 NX 安装仓库定义的导航依赖：

```bash
cd /home/dogrobot/robot
sudo apt update
bash script/dep/ros2_dep.sh
```

`ros2_dep.sh` 安装的主要包包括：

```text
ros-humble-controller-interface / controller-manager / control-toolbox
ros-humble-pcl-conversions / pcl-msgs / pcl-ros
ros-humble-slam-toolbox / behaviortree-cpp / navigation2 / rviz2 / vision-msgs
libsdl2-dev / libsdl2-ttf-dev / libompl-dev
gcc / g++ / make / build-essential / ninja-build / git
```

`script/dep/install_all.sh` 还会调用 Gazebo 和手柄依赖脚本；仅在需要相应功能时执行，并先审查其内容。NX 现场已使用 Ubuntu 22.04、Node 22/npm 10、Python 3.10、Nav2、SLAM Toolbox、PCL、protobuf 等运行时；新机版本应与这套基线兼容。

安装 Python 依赖时使用相应目录的 `requirements.txt`：

```bash
python3 -m pip install -r /home/dogrobot/edge-agent/requirements.txt
python3 -m pip install -r /home/dogrobot/dev-agent/requirements.txt
python3 -m pip install -r /home/dogrobot/robot/mcp_server/requirements.txt
```

生产环境建议在虚拟环境或受控 Python 站点包中安装并记录版本；不要因安装依赖而替换 ROS 的系统 Python。

### 6.3 发布 ROS 工作区与构建

ROS 工作区直接在统一 Git 检出中构建，不再复制到第二套源码目录：

```bash
cd /home/dogrobot/robot
source /opt/ros/humble/setup.bash
./build.sh all
source install/setup.bash
```

远程新机部署也可使用包装脚本；目标路径默认同样是 `/home/dogrobot`：

```bash
cd /home/dogrobot
deploy/robot/deploy.sh --host robot@<NX_IP> --build
```

源码和构建产物统一位于 `/home/dogrobot`。地图、rosbag、Edge 配置与数据、密钥、模型和会话仍位于 `/home/robot`，发布或拉取代码时不得覆盖这些运行数据。

### 6.4 NX 服务清单和启动顺序

| 服务 | 职责 | 当前路径/要求 |
| --- | --- | --- |
| `roamerx-zenoh.service` | ROS 2 Zenoh 路由 | Edge 启动前必须可用。 |
| `roamerx-edge-agent.service` | MQTT、任务、遥测、地图/充电协调 | 代码从 `/home/dogrobot/edge-agent` 加载，配置为 `/home/robot/edge_agent/config.yaml`。 |
| `roamerx-teleop-bridge.service` | 常驻 SDK/虚拟遥控桥 | 使用 ROS 工作区的 `robot_navigo`；服务受保护，不能用普通停服务绕过安全设计。 |
| `roamerx-local-asr.service` | NX 本地语音识别 | 本地监听 `127.0.0.1:18080`，使用语音模型缓存。 |
| `roamerx-dev-agent.service` | 远程开发/语音任务桥 | 使用 `/home/dogrobot/dev-agent` 与 Edge 运行配置。 |
| `roamerx-robot-mcp.service` | 本机 Robot Control MCP | 通过 `/etc/systemd/system/roamerx-robot-mcp.service.d/override.conf` 从 `/home/dogrobot/robot/mcp_server/roamerx_robot_mcp.py` 启动，监听 `127.0.0.1:8095`。 |

正常重建后的低风险检查顺序：

```bash
sudo systemctl daemon-reload
sudo systemctl enable roamerx-zenoh roamerx-edge-agent \
  roamerx-local-asr roamerx-dev-agent roamerx-robot-mcp

sudo systemctl start roamerx-zenoh
sudo systemctl start roamerx-edge-agent
sudo systemctl start roamerx-local-asr roamerx-dev-agent roamerx-robot-mcp

systemctl status roamerx-zenoh roamerx-edge-agent roamerx-local-asr \
  roamerx-dev-agent roamerx-robot-mcp --no-pager
sudo ss -ltn | grep -E '127.0.0.1:(18080|8095)'
```

`roamerx-teleop-bridge` 仅在机器人已完成现场安全确认、SDK/3588 链路已核验且运行模式要求常驻遥控桥时由既定运维流程启用。不要将“验证服务已安装”等同于“可以发运动指令”。

导航/建图只使用下列已批准的运行脚本：

```bash
cd /home/dogrobot/robot
script/robot/start_navigation_real.sh status
script/robot/start_navigation_real.sh start
script/robot/start_mapping_real.sh start
```

`start` 会影响实体机器人，必须在设备和场地均已确认安全时才执行；恢复验收优先运行 `status`、`ros2 node list`、`ros2 topic echo --once` 等只读命令。

### 6.5 NX 数据恢复

NX 私有备份位于旧运行工作区：

```text
/home/robot/genisom_roamerx_open/backup/nx_20260812_002113/
  nx-system-config.tar.gz
  nx-application-config.tar.gz
  SHA256SUMS
  snapshots/
```

它不随 `/home/dogrobot` 的 Git 检出完整提供。归档包含 systemd、网络、音频、Edge 配置/状态、ROS/RTK 配置、校准、Dev Agent 等；但**不含**约 15 GB 的地图点云/图像、rosbag、大型构建产物、生成的检测日志/截图。因此恢复还必须有单独的地图和 rosbag 备份。

安全恢复流程：

1. 将归档复制到受限临时目录，校验 `sha256sum -c SHA256SUMS`，再以 `tar -tf` 检查清单。
2. 解压到临时目录，不直接解压到 `/`、`/home/robot` 或正在运行的工作区。
3. 在机器人处于批准维护状态时，逐项比较并恢复。优先恢复 Edge 的 `config.yaml`、RTK/NTRIP 文件、`sixents_no_sdk.ini`、必要的 systemd unit、校准和地图元数据；保留从密码库恢复的设备凭据。
4. 用独立数据备份恢复 `/home/robot/.jszr/map/`，保持 `map.yaml`、`map.pgm`、`map.pcd`、`map.txt` 及活动软链接的一致性。不要把错误地图应用到正在运行的导航栈。
5. 重新发布 ROS 源码、执行 `./build.sh all`、`systemctl daemon-reload`，只重启受影响服务，并逐项做只读验收。

## 7. 3588：厂商运控环境和增量组件

### 7.1 边界

3588 的 `robot-launch`、运控 eggs、相机服务、功耗遥测、GENISOM SDK、网络和低层硬件配置来自**批准的厂商基础镜像**，并不能从本 Git 仓库完整构建。新 3588 的正确顺序是：

1. 恢复/刷入经过批准的厂商镜像；
2. 核验 `robot-launch list`、网络、时间、音频、相机、SDK 和基础运控；
3. 再安装本仓库管理的充电桩和腿部电源增量资产；
4. 最后由 NX 进行只读通信和状态验证。

禁止将完整 `/home/dogrobot`、NX ROS 工作区或 NX Edge Agent rsync 到 3588。

### 7.2 3588 所需软件与当前资产

3588 当前为 Ubuntu 22.04，已存在 `protoc` 3.12.4。构建 `leg_power` 至少需要：

```bash
sudo apt update
sudo apt install -y build-essential protobuf-compiler libprotobuf-dev pkg-config
```

还需要 eCAL 的开发库与 `ecal_core` 链接信息。eCAL 应来自经批准的厂商镜像或同一架构的软件源；在安装前验证：

```bash
protoc --version
pkg-config --modversion protobuf
pkg-config --exists ecal_core
```

| `/home/dogrobot` 源 | 3588 目标 |
| --- | --- |
| `robot/third_party/charge_pile_3588/` | `/home/firefly/charge_pile_v1.0.3b/charge_pile_xg_lib_v1.0.3b/` |
| `robot/script/robot/charge_pile_controller.sh` | `/usr/local/sbin/roamerx-charge-pile-controller` |
| `robot/systemd/roamerx-charge-pile.service` | `/etc/systemd/system/roamerx-charge-pile.service` |
| `edge-agent/tools/leg_power/` | 在 3588 构建，产物 `/usr/local/bin/roamerx-leg-power` |

### 7.3 安装充电桩和腿部电源增量

在确认充电/运控维护窗口后，从 NX 或受控构建机传输：

```bash
rsync -a /home/dogrobot/robot/third_party/charge_pile_3588/ \
  3588:/home/firefly/charge_pile_v1.0.3b/charge_pile_xg_lib_v1.0.3b/

scp /home/dogrobot/robot/script/robot/charge_pile_controller.sh \
  3588:/tmp/roamerx-charge-pile-controller
ssh 3588 'sudo install -o root -g root -m 0755 \
  /tmp/roamerx-charge-pile-controller /usr/local/sbin/roamerx-charge-pile-controller'

scp /home/dogrobot/robot/systemd/roamerx-charge-pile.service \
  3588:/tmp/roamerx-charge-pile.service
ssh 3588 'sudo install -o root -g root -m 0644 \
  /tmp/roamerx-charge-pile.service /etc/systemd/system/roamerx-charge-pile.service && \
  sudo systemctl daemon-reload'

rsync -a /home/dogrobot/edge-agent/tools/leg_power/ 3588:/tmp/roamerx-leg-power/
ssh 3588 'cd /tmp/roamerx-leg-power && bash install_on_3588.sh'
```

`robot/third_party/charge_pile_3588/` 内含 ARM64 厂商二进制，必须使用 `rsync -a` 保留模式。充电桩 service 的启动会触及蓝牙/充电状态；它不是普通无状态 Web 服务。仅在已验证状态文件、串口、蓝牙、厂商包和安全条件后按运维策略启动。不得因服务为 `inactive` 就在未知充电状态下直接强行启动。

### 7.4 3588 配置恢复

私有归档位于：

```text
/home/robot/genisom_roamerx_open/backup/3588_20260812_000539/
  3588-config-files.tar.gz
  SHA256SUMS
  snapshots/
```

该归档包含 systemd、`robot-launch` 文件、运控/监控配置、充电桩、网络、蓝牙、PulseAudio 和 chrony 等。它不是通用镜像，也不能直接解压到运行中的 3588；其中有设备专属凭据。

恢复方法：校验 SHA-256 -> 解压到临时目录 -> 与 `snapshots/` 和现机逐文件比对 -> 只恢复目标单元/配置 -> `systemctl daemon-reload` -> 仅重启受影响服务 -> 用 `robot-launch list` 核验。备份拍摄时是散热待机模式，不能把其“仅少数 egg 运行”的快照误当作正常工作模式模板。

## 8. 三端服务启动与验收顺序

### 8.1 推荐顺序

1. 云端：数据、环境文件、Nginx 语法和 API；
2. 云端：API -> device worker -> scheduler；确认 Broker 的实际监管和监听；
3. 3588：厂商 `robot-launch`、时间、网络、传感器/运控的只读状态；
4. NX：Zenoh -> Edge Agent -> 本地 ASR/Dev Agent/Robot MCP；
5. NX：在安全维护和定位/传感器条件均满足时，再按需要启用遥控桥、导航或建图；
6. 端到端：云端只读页面/API、MQTT presence/telemetry、NX ROS 只读状态；最后才由批准人员执行受控机器人功能测试。

### 8.2 非运动验收清单

**云端：**

```bash
curl --noproxy '*' -fsSI https://39.107.250.69/
curl --noproxy '*' -fsSI https://39.107.250.69/api/maps/
ssh cloud 'systemctl is-active roamerx-center-api roamerx-device-worker roamerx-patrol-scheduler nginx'
ssh cloud 'ss -ltnp | grep -E ":(443|8088|8081|1884)\\b"'
```

**NX：**

```bash
systemctl is-active roamerx-zenoh roamerx-edge-agent \
  roamerx-local-asr roamerx-dev-agent roamerx-robot-mcp
curl -fsS http://127.0.0.1:8095/mcp || true  # 仅确认本机 endpoint 可达
source /opt/ros/humble/setup.bash
source /home/dogrobot/robot/install/setup.bash
ros2 node list
ros2 topic echo /localization_info --once
```

**3588：**

```bash
ssh 3588 'robot-launch list'
ssh 3588 'test -x /usr/local/bin/roamerx-leg-power && echo leg-power-ok'
ssh 3588 'test -x /usr/local/sbin/roamerx-charge-pile-controller && echo charge-controller-ok'
```

验收阶段不得用“发送导航目标”“遥控前进”“启动充电桩”等方式代替健康检查。

## 9. 发布、回滚和排障

### 9.1 发布前检查

- 记录 `/home/dogrobot` 的 SHA，确认工作树干净或变更已审查；
- 云端完成 SQLite + 媒体备份，NX/3588 配置和地图备份可用；
- 审查迁移文件和数据库兼容性；
- 检查三端磁盘空间、时间同步、网络/SSH 和服务状态；
- 确认发布不会使机器人从维护安全状态转为可运动状态。

### 9.2 回滚原则

| 环境 | 回滚点 | 回滚方式 |
| --- | --- | --- |
| 云端代码 | 前一 `/opt/roamerx/releases/<release>` | 在维护窗口将 `current` 切回已记录 release，重启 API/worker/scheduler，验证 HTTPS。 |
| 云端数据 | SQLite 在线备份 + 媒体快照 | 停止写入者后按第 5.6 节恢复；数据迁移不可逆时先做兼容性评估。 |
| NX 代码 | `/home/dogrobot` 的前一 SHA | 切换到已审查 SHA，在 `robot/` 重新构建，再重启受影响服务。 |
| NX 配置/地图 | 私有备份 + 独立地图备份 | 临时目录比对后逐项恢复；绝不整包覆盖。 |
| 3588 | 厂商镜像 + 私有配置归档 | 先恢复厂商基线，再逐项恢复充电桩/配置，不覆盖运行中的运控系统。 |

### 9.3 已知重点问题

1. **云端 Node 缺失：** 当前云主机无 `node`/`npm`；前端应在 `/home/dogrobot` 或 CI 构建，再通过发布脚本同步。
2. **MQTT 监管不一致：** 当前 :1884 有 Mosquitto 进程，但 `mosquitto.service` 是 inactive；先找出真实管理方式再修复，避免双 Broker 或端口冲突。
3. **源码与运行数据分离：** Edge 源码在 `/home/dogrobot/edge-agent`，配置和持久数据在 `/home/robot/edge_agent`；不要把运行配置提交到 Git。
4. **3588 不是 Git 可完全复现环境：** Git 只包含充电桩和腿部电源增量资产；完整低层环境依赖厂商镜像。
5. **私有备份未随新检出完整携带：** `/home/dogrobot/backup/` 仅有可入库文件；NX/3588 配置归档须从安全备份介质或旧运行目录取得。
6. **地图不在 NX 配置归档中：** 地图点云和图像需独立备份/恢复，并保证 symlink、yaml、pgm、pcd 一致。

## 10. 参考入口

- `/home/dogrobot/AGENT.md`：本地开发与部署目录的简明约定。
- `/home/dogrobot/deploy/cloud/deploy.sh`、`platform/scripts/deploy_cloud_platform.sh`：云端发布入口。
- `/home/dogrobot/deploy/robot/deploy.sh`：NX 发布包装器。
- `/home/dogrobot/deploy/controller/README.md`：3588 厂商镜像边界。
- `/home/dogrobot/platform/docs/`：平台架构、API、数据和运维文档。
- `/home/dogrobot/robot/AGENTS.md`、`robot/STARTUP_GUIDE.md`：真实机器人运行约束与 ROS 启动说明。
- 旧运行工作区下的 `backup/nx_20260812_002113/` 与 `backup/3588_20260812_000539/`：私有配置恢复材料。
