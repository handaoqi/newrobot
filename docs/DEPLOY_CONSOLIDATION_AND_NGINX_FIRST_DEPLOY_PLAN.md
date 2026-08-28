# 三端部署目录收敛与Nginx首次部署执行计划

## 1. 总结

保留四个正式入口：

```text
deploy/
├── platform/   # 云平台与Nginx
├── nx-edge/    # NX机器人主机
├── 3588/       # RK3588控制器及受管覆盖层
└── backup/     # 三端备份
```

将 `deploy/controller/README.md` 合并到 `deploy/3588/README.md`，删除空的 `deploy/controller/`。`cloud/`、`robot/` 暂留一个版本作为兼容转发入口并输出弃用提示。

## 2. 实施改造

### 2.1 目录与入口收敛

- `deploy/3588/` 成为控制器唯一正式入口，包含说明、部署和验证脚本。
- 控制器只部署项目管理的充电桩、运动控制覆盖层和状态验证，不覆盖厂商固件、凭据、GENISOM SDK及 `robot-launch`。
- `deploy/cloud/deploy.sh` 转发到 `platform/deploy.sh`，`deploy/robot/deploy.sh` 转发到 `nx-edge/deploy.sh`；文档不再引用兼容入口。
- `platform/deploy/`、`edge-agent/systemd/`、`robot/systemd/` 等继续作为组件资源源文件，根目录 `deploy/` 只负责安装编排。

### 2.2 云平台与Nginx首次部署

- Docker Compose是唯一正式云端运行方式；旧 `platform/scripts/deploy_cloud_platform.sh` 改为兼容转发，不再执行Conda迁移或重启旧systemd服务。
- `deploy/platform/deploy.sh` 增加：
  - `--start`
  - `--verify`
  - `--install-nginx`
  - `--domain DOMAIN`
  - `--cert-email EMAIL`
  - `--frontend-only`
  - `--backend-only`
  - 真正命令级 `--dry-run`
- Compose前端端口改为仅监听 `127.0.0.1:8088`；宿主机Nginx负责公网80/443入口。
- 新增宿主机站点模板：
  - HTTP端口用于ACME验证并跳转HTTPS；
  - HTTPS反向代理到 `127.0.0.1:8088`；
  - 转发真实IP、Host、协议和WebSocket头；
  - `client_max_body_size 2g`；
  - 上传和长请求超时1800秒；
  - 前端容器继续负责 `/api/`、`/media/`、`/live/` 和gzip，宿主机不重复压缩。
- 首次安装流程固定为：
  1. 校验域名DNS指向目标云主机，80/443可用；
  2. 安装Nginx、Certbot和Nginx插件；
  3. 备份已有 `/etc/nginx` 站点；
  4. 安装临时HTTP站点并执行 `nginx -t`；
  5. 启动Compose并验证本地8088；
  6. Certbot签发证书并启用HTTP到HTTPS跳转；
  7. 再次执行 `nginx -t`、reload及公网HTTPS检查；
  8. 验证Certbot续期定时器。
- 部署失败时恢复原站点配置并reload；不删除已有证书、数据库、MQTT凭据或媒体数据。
- 同步更新生产环境中的 `DJANGO_ALLOWED_HOSTS`、`DJANGO_CSRF_TRUSTED_ORIGINS` 和 `PUBLIC_BASE_URL`；已有凭据只校验，不自动覆盖。

### 2.3 NX与3588部署清单

- NX生产默认启用当前实际运行的Edge Agent、Zenoh、遥控桥、5G共享、开发代理、本地ASR、视频推理和Robot MCP。
- `roamerx-mapping.service` 安装但保持disabled/inactive，由Edge Agent按需启动。
- `roamerx-cloud-tunnel.service` 仅通过显式可选参数安装和启用。
- 正确安装Edge lifecycle drop-in、本地ASR drop-in和person-follow tmpfiles配置，并执行 `systemd-tmpfiles --create`。
- 新增 `deploy/nx-edge/verify.sh`，检查服务、ROS节点、唯一里程计发布者、运行时配置和版本指纹。
- 3588部署增加充电桩service安装、`robot-launch list`、受管二进制校验及控制器状态验证；不替换厂商 `robot-launch.service`。
- 所有systemd安装使用“源文件、目标路径、required/optional、enable/on-demand、验证命令”的显式清单，不再靠散落数组维护。

## 3. 版本、接口与兼容

- 三端统一生成 `release.json`，记录Git提交、部署时间、目标主机、二进制SHA256、配置SHA256、systemd包SHA256和部署入口版本。
- 运行时位置固定为：
  - 云端：`/opt/roamerx/runtime/platform/release.json`
  - NX：`/home/dogrobot/runtime/nx-edge/release.json`
  - 3588：`/home/firefly/dogrobot-runtime/release.json`
- `verify.sh` 检查运行文件与发布清单一致；不一致时部署失败，建图启动额外返回 `MAPPING_DEPLOYMENT_MISMATCH`。
- 所有dry-run打印完整rsync、安装、enable、restart、Nginx和健康检查命令，但不写文件、不连接服务。
- 兼容入口保留一个发布周期；随后删除 `deploy/cloud/` 和 `deploy/robot/`。

## 4. 测试与验收

- Shell脚本通过 `bash -n`、ShellCheck和参数错误测试。
- dry-run断言三端无文件、服务或远端状态变化。
- 目录测试确认 `deploy/controller/` 已删除、README已进入 `deploy/3588/`、旧入口仍能正确转发。
- Docker执行 `compose config --quiet`，验证旧systemd云服务处于停止状态。
- Nginx首次部署验证：
  - 8088仅本机可访问；
  - 80跳转443；
  - TLS证书域名、有效期和自动续期正常；
  - `/`、`/api/maps/`、`/media/` 和实时流代理正常；
  - 大文件上传不受默认1 MB限制；
  - 故障注入后可恢复旧Nginx配置。
- NX验证必需服务active、mapping服务disabled且可按需启动、drop-in生效。
- 3588验证受管覆盖层和充电桩服务，不修改厂商运行时。
- 三端部署后比较 `release.json` 与目标Git提交，并完成平台API、MQTT、地图上传和机器人在线状态验收。
- 实施时保留当前未提交改动，先纳入并确认 `platform/deploy/nginx/` 资源，再提交合并到私有仓库 `main`。
