# 运维、测试与排障

[返回文档中心](./project-docs-index.md)

## 1. 快速验证清单

### 1.1 后端

```powershell
.venv\Scripts\python.exe manage.py migrate
.venv\Scripts\python.exe runserver 127.0.0.1:8000
```

健康检查：

```powershell
curl http://127.0.0.1:8000/api/health/
```

### 1.2 前端

访问：

```text
http://127.0.0.1:5173/
```

### 1.3 ZLMediaKit

```powershell
docker compose -f docker-compose.zlmediakit.yml up -d
docker ps --filter name=playground-zlmediakit
```

### 1.4 板端推流或识别

仅推流：

```powershell
.venv\Scripts\python.exe run_stream.py --config config.yaml
```

完整闭环：

```powershell
.venv\Scripts\python.exe run_edge.py --config config.yaml
```

## 2. 常见问题

### 2.1 前端登录后接口 401

可能原因：

- `localStorage.inspection_token` 为空或旧 Token 失效。
- Django 后端没有运行。
- 前端 `API_BASE` 指向错误地址。

处理：

1. 浏览器清理 `localStorage.inspection_token` 和 `localStorage.inspection_user`。
2. 重新登录。
3. 检查 `frontend/src/services/api.js` 中的 `API_BASE`。
4. 确认后端 `/api/health/` 正常。

### 2.2 端口被占用

常用端口：

| 端口     | 组件            |
| -------- | --------------- |
| `8000` | Django          |
| `5173` | Vite            |
| `1935` | ZLMediaKit RTMP |
| `8080` | ZLMediaKit HTTP |
| `8554` | ZLMediaKit RTSP |

处理：

- Django 可换端口：

```powershell
..\.venv\Scripts\python.exe manage.py runserver 127.0.0.1:8001
```

- 前端换后端地址需同步修改 `frontend/src/services/api.js`。

### 2.3 事件图片不显示

可能原因：

- `snapshot_url` 为空。
- 媒体上传失败。
- `MEDIA_URL` 或 `MEDIA_ROOT` 配置异常。
- 前端访问的 URL 不可达。
- 文件被删除。

处理：

1. 检查事件详情中的 `snapshot_url`。
2. 打开 URL 确认图片是否可访问。
3. 检查 `backend/media/device-media/` 是否存在文件。
4. 检查板端日志 `bot-version/data/telemetry/telemetry.jsonl`。
5. 若是 SHA-256 校验失败，确认上传前后文件未变化。

### 2.4 实时视频不播放

可能原因：

- ZLMediaKit 未启动。
- 板端没有推流。
- `video.play_urls` 指向错误地址。
- 浏览器无法访问 `127.0.0.1` 对应服务。
- RTSP 源不可用。
- 推流重复，ZLMediaKit 返回 `Already publishing`。

处理：

1. 确认 ZLMediaKit 容器运行。
2. 确认 `run_stream.py` 或 `run_edge.py` 正在运行。
3. 浏览器直接访问 FLV/HLS URL。
4. 使用 ffmpeg 测试 RTSP：

```powershell
ffmpeg -rtsp_transport tcp -i rtsp://192.168.234.1:8554/test -t 5 -f null -
```

5. 若已启动 `run_stream.py`，不要再让 `run_edge.py` 推同一个流。

### 2.5 板端无法打开摄像头或 RTSP

可能原因：

- 摄像头编号错误。
- RTSP 地址、账号、密码错误。
- 网络不通。
- 摄像头只支持 UDP 或 TCP 中的一种。
- OpenCV/ffmpeg 不支持对应编码。

处理：

- 使用 ffmpeg 单独验证。
- 将 `video.rtsp_transport` 在 `tcp` 与 `udp` 间切换。
- 调整 `video.open_timeout_seconds` 和 `video.read_timeout_seconds`。
- 确认板端机器能访问摄像头网段。

### 2.6 板端上报失败

可能原因：

- 后端未启动。
- `telemetry.endpoint` 地址错误。
- 网络不可达。
- JSON 字段不满足后端 serializer 约束。
- 后端返回 400，但板端只记录为发送失败。

处理：

1. 检查后端健康检查。
2. 检查 `bot-version/config.yaml` 的 `telemetry.endpoint`。
3. 查看 `bot-version/data/telemetry/telemetry.jsonl`。
4. 查看后端终端错误输出。
5. 对照 [前后端接口与数据传输格式](./api-and-payloads.md) 检查字段。

### 2.7 重复事件过多

可能原因：

- `detection.tracking_enabled=false`。
- `duplicate_alert_seconds` 太短。
- 跟踪阈值不适合当前画面。
- 摄像头画面抖动导致目标被认为是新目标。

处理：

- 开启 `tracking_enabled`。
- 调大 `duplicate_alert_seconds`。
- 调整 `tracker_iou_threshold`。
- 适当提高 `min_box_area` 或 `confidence`。

## 3. 测试建议

### 3.1 后端接口测试

建议覆盖：

- 登录成功/失败。
- Token 保护接口未登录返回 401。
- `/api/events/` 的筛选、搜索、排序、分页。
- `/api/events/<id>/handle/` 的状态校验和复核结论校验。
- `/api/telemetry/ingest/` 的正常上报、重复 `sequence_id`、字段校验。
- `/api/device/media/upload/` 的文件上传、SHA-256 校验失败。

### 3.2 前端功能测试

建议覆盖：

- 登录跳转。
- 未登录访问工作台自动跳转登录。
- 监测中心无视频流时显示静态图。
- 有 FLV/HLS 地址时播放器初始化。
- 事件筛选、搜索、排序、滚动加载。
- 待处理事件归档后从待处理列表移除。
- 机器人详情切换。
- 统计图正常渲染。

### 3.3 板端联调测试

建议覆盖：

- 摄像头打开失败后的重连。
- RTSP 断流后的重连。
- 无检测结果时只上报状态。
- 检测命中后上传抓拍并上报事件。
- 后端不可达时本地 JSONL 日志记录失败。
- ZLMediaKit 断开后推流线程重试。

## 4. 生产化建议

### 4.1 后端

- 关闭 `DEBUG`。
- 设置严格的 `ALLOWED_HOSTS`。
- 按真实前端域名配置 CORS。
- 将 SQLite 替换为 PostgreSQL 或 MySQL。
- 使用环境变量管理 `SECRET_KEY`、数据库密码、设备密钥。
- 对板端接口增加 `X-Device-Key` 校验、签名或 mTLS。
- 增加分页默认限制和限流。
- 增加结构化日志和异常监控。

### 4.2 前端

- 将 `API_BASE` 改为环境变量，例如 `.env.production`。
- 对控制类按钮接入真实后端接口。
- 引入全局请求错误处理和登录过期处理。
- 增加实时刷新机制，如 WebSocket 或 SSE。

### 4.3 视频

- 为 RTMP 推流 URL 增加 token。
- 使用 HTTPS 域名提供 HLS/FLV。
- 根据网络环境选择低延迟 FLV 或兼容性更好的 HLS。
- 监控在线流状态。

### 4.4 板端

- 模型文件随版本管理。
- 支持离线缓存和断点补传。
- 上报失败时做指数退避。
- 设备密钥安全存储。
- 增加 watchdog 或系统服务守护。

## 5. 相关文档

- [项目整体架构](./project-architecture.md)
- [环境配置与启动](./environment-setup.md)
- [前后端接口与数据传输格式](./api-and-payloads.md)
- [数据存储说明](./data-storage.md)
- [网站功能说明](./website-features.md)
- [板端识别与视频链路](./edge-and-video-pipeline.md)
