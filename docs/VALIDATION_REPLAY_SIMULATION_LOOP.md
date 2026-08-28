# Foxglove 在线仿真回放检查闭环

## 已实现范围

第一阶段采用 Bag-in-the-loop：操作员上传任务录制的单文件 MCAP，平台创建不可变检查作业，独立 CPU Runner 在隔离 ROS Domain 中重放输入、可选启动受信算法栈、录制派生结果并执行规则/黄金基线检查。检查结果和 MCAP 制品回传平台，可在自托管 Lichtblick（Foxglove 兼容 Web）中查看。

MATRiX/UE 模式已经保留 Profile、Runner 类型和后端接口，但在没有 `matrix` Runner 时会以 `RUNNER_UNAVAILABLE` 明确拒绝，不会伪装成仿真成功。

```text
浏览器 / MCP(AI)
       |
       v
平台 API ---- 作业/租约/检查/基线 ---- PostgreSQL 或 SQLite
   |                         |
   | 预签名分片              | 领取/心跳/完成（Runner Token）
   v                         v
MinIO/S3 <-------------- 独立 CPU Runner
                              |
                              +-- rosbag play + /clock
                              +-- 受信算法启动命令
                              +-- rosbag record (MCAP)
                              +-- 规则/黄金基线分析
                              +-- 只读 foxglove_bridge

浏览器 -- 60 秒作业票据 --> Validation Gateway --> Runner 私网 WebSocket
```

## 关键目录

- `platform/backend/monitoring/validation_views.py`：录制、作业、Runner、报告、制品和实时票据 API。
- `platform/backend/monitoring/services/validation_service.py`：幂等创建、租约、重试、取消、判定和黄金基线。
- `platform/validation-runner/`：CPU Runner、MCAP 分析器及 MATRiX 后端契约。
- `platform/validation-gateway/`：鉴权后只代理私网、限定端口的 Foxglove WebSocket。
- `platform/frontend/src/views/ValidationJobsPage.vue`：上传、创建、进度、检查、取消、基线与回放入口。
- `robot/mcp_server/roamerx_robot_mcp.py`：AI 可调用的受限检查工具。

## 部署

### 1. 平台

复制 `runtime/platform/conf/platform.env.example` 为 `platform.env`，至少替换以下密钥：

```dotenv
DJANGO_SECRET_KEY=...
VALIDATION_OBJECT_STORE_SECRET_KEY=...
VALIDATION_RUNNER_TOKEN=...
VALIDATION_GATEWAY_TOKEN=...
```

将 `VALIDATION_OBJECT_STORE_PUBLIC_ENDPOINT` 配成浏览器能访问的 MinIO/S3 HTTPS 地址。平台编排包含 MinIO、桶初始化、API、Validation Gateway 和前端。自托管 Lichtblick 的固定构建应放在：

```text
runtime/platform/install/lichtblick-web/dist/
```

然后执行现有平台 Compose 启动流程。启动时后端会应用 `0061_validation_replay_pipeline` 迁移并种入 `navigation-localization` 默认 Profile。

### 2. 独立 CPU Runner

Runner 应部署在与平台/网关互通的私网或 VPN 主机，而不是机器人控制域。构建：

```bash
docker build -t roamerx/validation-runner:local platform/validation-runner
```

运行时至少设置：

```dotenv
ROAMERX_PLATFORM_URL=https://platform.example.com
ROAMERX_VALIDATION_RUNNER_TOKEN=与平台一致
ROAMERX_VALIDATION_RUNNER_ID=cpu-runner-1
ROAMERX_VALIDATION_PRIVATE_ADDRESS=10.20.0.12
ROAMERX_VALIDATION_WORK_ROOT=/var/lib/roamerx-validation
ROAMERX_STACK_GIT_SHA=<commit>
ROAMERX_STACK_IMAGE_DIGEST=<immutable image digest>
```

`ROAMERX_VALIDATION_PRIVATE_ADDRESS` 必须能由 Validation Gateway 解析到私有或回环地址。实时桥端口固定在 `8800..9499`，无需向公网暴露。

默认 Profile 未设置 `runner_config.launch_command` 时，Runner 执行“录制契约检查”，不会声称已经重算算法。要启用真正的闭环重算，由管理员在 Profile 中配置受信命令；入口只允许 `ros2` 或 `/opt/roamerx/bin/validation-launch`，AI 和作业请求不能覆盖它。

## 检查语义

默认导航/定位检查包括：

- 必需话题完整性和各话题时间戳单调性；
- TF child 多父节点冲突；
- `/odom/localization_odom`、`/odom/nav2` 相邻位姿跳变；
- `/cmd_vel_raw` 角速度指令正负翻转频率；
- 相对黄金基线的消息量、时长和位姿跳变退化。

硬失败或任一 `FAIL` 令作业失败；缺少黄金基线、存在警告或不可判定项时为 `WARN`；只有规则通过且有可比基线时才为 `PASS`。没有 GPU/MATRiX Runner 时场景仿真作业拒绝入队。

## AI 接口与权限边界

MCP 暴露以下能力：

- 列出录制和 Profile；
- 仅凭 `recording_id`、`profile_id`、可选 baseline/config 创建沙箱作业；
- 查询、取消作业；
- 获取结构化报告和有界证据。

MCP 不接收文件路径、Shell 命令、环境变量、ROS Domain、镜像或启动命令。AI 不能批准黄金基线、修改 Profile、操作 Runner、连接机器人控制话题或创建 MATRiX 资源。黄金基线批准仍是平台操作员动作。

## 实时与完整回放

算法重放期间，Runner 启动只读 `foxglove_bridge`，仅允许订阅时钟、TF、地图、里程计、速度指令、激光/点云、路径和诊断类话题。参数、服务、客户端发布和资产访问均关闭。网页先请求绑定当前用户与作业的 60 秒票据；网关再次向平台验证作业仍在运行，并拒绝公网地址和非验证端口。

作业结束后实时入口关闭。完整结果通过带过期签名的 HTTP Range 下载路径交给 Lichtblick，因此可拖动时间轴，不依赖 Runner 继续在线。

## 当前限制与下一阶段

- 当前录制入口是网页/HTTP 按需上传；机器人任务结束后的自动分片同步尚未默认启用，避免未经选择持续占用蜂窝链路。
- 单文件 MCAP 是第一阶段输入契约；多 chunk rosbag2 目录需在上传前合并或选择目标 MCAP。
- MATRiX/UE 的 `SimulationBackend` 是失败关闭的适配接口，待 GPU Runner 和场景资产版本规范就绪后接入。
- 规则先覆盖导航/定位的确定性信号；感知真值、碰撞、场景事件和视频证据属于 MATRiX 阶段。

## 验证命令

```bash
cd platform/backend
python manage.py test monitoring.test_validation
python manage.py check
python manage.py makemigrations --check --dry-run

cd platform/frontend
npm run build
npm run test:foxglove -- --grep "loads with every declared panel"

cd ../..
pytest -q mcp_server/tests/test_roamerx_robot_mcp.py
python3 -m py_compile platform/validation-runner/*.py platform/validation-gateway/gateway.py
```
