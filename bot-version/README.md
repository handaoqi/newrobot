# Bike Bot

面向机器人板端的最小可用项目，完成以下链路：

- 读取摄像头或 RTSP 视频流
- 调用 YOLO 模型实时识别自行车
- 实时显示识别框、置信度和 FPS 预览
- 按约定 JSON 结构上报遥测、状态和告警事件
- 将每次发往服务器的 JSON 保存到本地日志文件
- 保存抓拍图，并可回传可访问 URL

## 1. 安装

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

如果你想直接使用 `python -m bike_bot.main` 这类模块启动方式，再额外执行一次：

```bash
pip install -e .
```

## 2. 配置

复制并修改配置文件：

```bash
copy config.example.yaml config.yaml
```

关键字段：

- `video.source`: 摄像头编号或 RTSP 地址
- `video.rtsp_transport`: `tcp` 或 `udp`
- `video.open_timeout_seconds`: RTSP 建连超时时间
- `video.read_timeout_seconds`: RTSP 读帧超时时间
- `model.path`: 你的 YOLO 模型路径，例如 `models/bike.pt`
- `telemetry.endpoint`: 远程服务地址，例如 `http://10.0.0.8:8000/api/telemetry/ingest/`
- `telemetry.device_key`: 如服务端启用 `X-Device-Key`，这里填写
- `snapshot.public_base_url`: 若抓拍图片经 nginx/对象存储暴露，这里填写访问前缀
- `display.enable`: 是否弹出实时识别预览窗口
- `storage.telemetry_log_path`: 本地保存上报 JSON 的日志文件

## 3. 启动板端程序

```bash
python run_edge.py --config config.yaml
```

如果你已经执行过 `pip install -e .`，也可以使用：

```bash
python -m bike_bot.main --config config.yaml
```

## 4. 运行效果

- 程序启动后会弹出实时预览窗口，显示检测框、置信度、目标数量和 FPS
- 按 `q` 或 `Esc` 可关闭预览并停止程序
- 每次向服务器发送的数据都会追加写入 `data/telemetry/telemetry.jsonl`

## 5. 上报策略

- 状态遥测：默认每 `2s` 上报一次
- 心跳：默认每 `5s` 上报一次
- 告警事件：检测命中后实时上报
- 抓拍：默认保存至 `snapshots/`，并拼接 `snapshot_url`

## 6. 部署建议

- 开发阶段先使用 `*.pt` 通过 `ultralytics` 跑通
- 上板后建议导出为 ONNX / TensorRT 以降低延迟
- 使用板端硬件解码摄像头流，避免 CPU 成为瓶颈
- 告警尽量做冷却时间控制，避免同一目标连续刷屏
- 生产环境建议补充 `X-Device-Key`、请求签名或双向证书

## 7. 目录

```text
src/bike_bot/
  main.py
  config.py
  detector.py
  runtime.py
  telemetry.py
  models.py
config.example.yaml
```
