# RoamerX 三端数据库初始化手册

更新时间：2026-08-21
适用人员：平台部署、机器人运维和现场开发人员

## 1. 操作前检查

1. 确认源码位于 `/home/dogrobot`，真实配置位于对应 `runtime/*/conf`。
2. 生产数据库初始化前先做备份；不要删除现有 `db.sqlite3` 或 `edge.db`。
3. 密码和 Token 只写入受控环境文件，不直接写入命令历史或本文档。
4. Docker 平台和旧 systemd 平台不能同时连接同一数据库与 MQTT 身份。
5. 3588 初始化不需要启动运控，禁止为验证数据库而发送物理动作。

## 2. 云平台 Docker 初始化

```bash
cd /home/dogrobot
runtime/platform/bin/platformctl init
```

编辑 `runtime/platform/conf/platform.env`，至少配置：

```text
DJANGO_SECRET_KEY
DJANGO_ALLOWED_HOSTS
DJANGO_CSRF_TRUSTED_ORIGINS
PUBLIC_BASE_URL
MQTT_USERNAME
MQTT_PASSWORD
PLATFORM_OPERATOR_USERNAME
PLATFORM_OPERATOR_PASSWORD
PLATFORM_ROBOT_CODE
PLATFORM_ROBOT_NAME
```

执行：

```bash
runtime/platform/scripts/prepare-mqtt-password.sh
runtime/platform/bin/platformctl init-db
runtime/platform/bin/platformctl up
runtime/platform/bin/platformctl verify
```

`init-db` 运行 Django migrations，再执行幂等命令 `initialize_platform`。容器正常启动时也会重复执行，因此断电重启不会插入重复数据。

## 3. 旧 systemd 云平台初始化

当前云主机仍可使用以下方式：

```bash
ssh root@39.107.250.69
set -a
source /opt/roamerx/shared/center.env
set +a
cd /opt/roamerx/current
/root/miniconda/envs/py310/bin/python backend/manage.py migrate --noinput
PLATFORM_OPERATOR_PASSWORD='<由安全渠道提供>' \
  /root/miniconda/envs/py310/bin/python backend/manage.py initialize_platform
```

已有操作员和机器人会显示 `exists`。首次初始化缺少 `PLATFORM_OPERATOR_PASSWORD` 时命令必须失败，防止创建默认弱密码。

仅开发演示环境可使用：

```bash
python backend/manage.py initialize_platform --with-demo-data
```

生产环境保持 `ENABLE_DEMO_SEED=false`，否则会创建演示地图、路线、任务和排班。

## 4. 平台验收

```bash
python backend/manage.py showmigrations monitoring
python backend/manage.py check
python backend/manage.py shell -c \
  "from monitoring.models import SpeechTemplate; print(SpeechTemplate.objects.count())"
```

验收标准：

- 所有迁移显示 `[X]`，当前至少包含 `0070_remove_legacy_low_battery_template`。
- Django `check` 无错误。
- 全新正式库只有指定操作员、指定机器人和基础播报/技能配置。
- `SpeechTemplate` 至少包含架构文档列出的 12 个名称；当前基线总数为 12，且不应存在已废弃的“低电量自动回充”模板。
- 全新正式库不包含地图、路线、任务和排班样例。

## 5. NX Edge 初始化

```bash
cd /home/dogrobot
runtime/nx-edge/bin/nxctl init-db
```

该命令会：

1. 从 `runtime/nx-edge/conf/edge-agent.yaml` 读取 SQLite 路径。
2. 创建缺失的五张 Edge 表。
3. 执行 `PRAGMA integrity_check`。
4. 输出每张表的记录数。
5. 保留现有命令、任务、outbox 和可信定位，不清空数据。

验收输出必须包含：

```text
integrity=ok
agent_metadata=<count>
outbox=<count>
processed_commands=<count>
task_context=<count>
trajectory_sequence=<count>
```

新库的五个 count 应全部为 `0`。现役设备 count 非零属于正常运行数据。

## 6. 3588 初始化

```bash
cd /home/dogrobot
runtime/3588-motion/scripts/deploy.sh
ssh 3588 /home/firefly/dogrobot-runtime/bin/motionctl init-state
ssh 3588 /home/firefly/dogrobot-runtime/scripts/verify.sh
```

3588 没有项目数据库。`init-state` 只在 `/var/lib/roamerx-charge-pile/state` 缺失时写入 `unknown`；已有 `lying` 状态会显示 `preserved` 并保持不变。该命令不会启动腿部运控或充电动作。

## 7. 备份与恢复

### 云平台

- 同时备份业务数据库和 `media`；只恢复数据库会导致事件图片失联。
- 使用 `runtime/platform/scripts/backup.sh` 或 SQLite 在线备份接口。
- 恢复前停止 API、device worker 和 scheduler 三个写入者。
- 恢复后依次执行 migrations、`initialize_platform` 和健康检查。

### NX Edge

- 同时备份 `edge.db`、`conf` 和地图目录。
- 高频写入时不要直接复制单个 SQLite 文件；先停止 Edge Agent 或使用 SQLite backup。
- 丢失 `edge.db` 后可以重建表，但未上报命令结果、任务恢复上下文和最后可信定位无法凭空恢复。

### 3588

- 备份状态文件和仓库管理的覆盖层配置。
- 厂商镜像、egg、标定和 MCU 参数按厂商镜像流程恢复，不作为数据库处理。

## 8. 常见问题

| 现象 | 原因与处理 |
| --- | --- |
| 初始化后出现演示地图 | 检查 `ENABLE_DEMO_SEED`，生产必须为 `false` |
| 首次初始化提示缺少密码 | 设置 `PLATFORM_OPERATOR_PASSWORD` 后重新执行 |
| Edge `integrity_check` 失败 | 停止 Edge Agent，保留故障库，先从备份恢复，禁止直接删库 |
| 模板少于 12 条，或仍存在“低电量自动回充” | 确认最新迁移（至少 `0070`）已应用，再执行 `initialize_platform` 补缺 |
| 3588 显示 `preserved ...=lying` | 正常，表示保留当前充电模式，而非初始化失败 |
| Docker 与 systemd 同时启动失败 | 停止旧平台服务，只保留一套 API/worker/scheduler |
