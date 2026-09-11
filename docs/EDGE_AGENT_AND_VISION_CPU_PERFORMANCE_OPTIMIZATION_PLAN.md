# edge-agent 与视觉推理 CPU 性能优化执行记录

更新时间：2026-09-10

状态：已实施、已部署、已完成静态导航满载验收。

## 1. 目标和结论

本轮同时处理两条高 CPU 链路：

1. `roamerx-edge-agent` 的高频 ROS 回调、共享锁和无界追帧风险；
2. `roamerx-bike-bot` 因 ONNX Runtime Provider 错误而回退 CPU 的视觉推理。

现场基线中 edge-agent 约 72% CPU，CPU 版视觉约 78.51%。实施后，在 LiDAR、定位、Nav2、碰撞监控、视觉检测和直播同时运行的 60 秒窗口中：

| 进程 | 实施前 | 实施后 | 结果 |
|---|---:|---:|---|
| edge-agent Python | 约 72% | 31.16% | 达到 25–35% 目标 |
| 视觉 Python | 78.51% | 17.98% | GPU 推理恢复 |
| 视觉 Python + 直播 FFmpeg | 约 80% | 约 19–20% | 明显低于 35% |

导航、安全和定位状态回调没有被降频；优化只作用于 IMU 交叉检查、里程计遥测、激光摘要和扫描匹配诊断数组。

## 2. 根因

### 2.1 edge-agent

原实现存在四个持续开销：

- `/front_lidar/imu` 每条约 200 Hz 消息获取锁并更新比较窗口；
- `/odom/localization_odom` 每条约 30 Hz 消息更新共享 telemetry 字典；
- `/laser_scan` 每帧遍历全部点并重复计算 `sin/cos`；
- `/status` 正常帧也复制完整预测标签和误差数组。

这些工作运行在共享 ROS executor 与 telemetry 锁上，造成回调争用。完整 SLAM 建图后端并不是 edge-agent CPU 的根因，不能通过停止导航必需的 FAST-LIO 里程计前端来规避。

### 2.2 视觉

systemd 曾加载用户目录中的 CPU 版 ONNX Runtime：

```text
providers=['AzureExecutionProvider', 'CPUExecutionProvider']
GR3D_FREQ 0%
```

配置中的 `tensorrt_enabled: true` 只表达意图，不能让 CPU 构建自动获得 TensorRT/CUDA Provider。修复方式是给服务使用独立 Jetson GPU 环境，并验证会话的实际 Provider。

## 3. 已实施改动

### 3.1 回调可观测性

新增 10 秒聚合指标 `edge_callback_perf`，每个窗口记录调用数、处理数、覆盖丢弃数、频率、总耗时、P99 和最大耗时。日志至少包含：

```text
imu_callback
odometry_callback
scan_callback
scan_matching_callback
telemetry_snapshot
callback_queue_depth
stale_sample_dropped
```

指标内存有界，禁止逐帧 INFO 日志。`stale_sample_dropped` 表示 latest-only 策略主动覆盖的旧样本，不是队列积压或数据通道故障。

### 3.2 高频 ROS 回调

配置入口为 `ros_callback_optimization`：

```yaml
ros_callback_optimization:
  enabled: true
  metrics_enabled: true
  metrics_interval_seconds: 10
  imu_sample_rate_hz: 25
  odometry_telemetry_rate_hz: 10
  scan_processing_rate_hz: 10
  scan_matching_full_detail_interval_seconds: 10
```

实施内容：

- IMU：订阅回调仅原子替换最新样本，25 Hz 定时消费；5 秒比较窗口与异常规则不变。现场的 `/highlevel_robotstate` 尚未提供，因此 `imu_cross_check.enabled` 继续保持 `false`，避免无意义订阅 200 Hz Livox IMU；latest-only 路径由自动化测试覆盖。
- 里程计：约 30 Hz 输入只保存最新值，10 Hz 更新 telemetry；`/localization_info` 的定位安全判定仍逐帧执行。
- 激光：队列固定为一个最新样本，10 Hz 处理；按扫描几何缓存前半平面的 `sin/cos`，仅遍历可能进入前方和左右安全区的角度。
- 扫描匹配：收敛、阈值和恢复触发仍逐帧计算。完整预测数组只在异常、状态变化或每 10 秒诊断采样时复制。
- 执行器：控制/安全与 telemetry/诊断使用不同的 mutually-exclusive callback group，继续由三线程 executor 驱动。
- 系统探测：保持已有 10 秒采样、冷却模式 15 秒采样和 30 秒陈旧判定，不在 ROS 高频回调中执行外部探测。

关闭 `ros_callback_optimization.enabled` 可恢复原始逐消息处理，作为单一回滚开关。

### 3.3 视觉 GPU、FP16 和 engine cache

视觉 systemd 使用独立环境：

```text
/home/dogrobot/runtime/nx-edge/data/vision/venv
onnxruntime-gpu 1.23.0
```

实际 Provider 顺序：

```text
TensorrtExecutionProvider
CUDAExecutionProvider
CPUExecutionProvider
```

已启用 TensorRT FP16，固定缓存路径为：

```text
/home/dogrobot/runtime/nx-edge/data/vision/trt-cache/yolo11n
```

安装脚本会验证 CUDA 与 OpenCV GStreamer，并调用 `tools/prewarm_vision_runtime.py` 用实际模型执行两次推理。首次 engine 编译因此发生在服务启动前，不进入导航任务窗口。现场预热结果：第一次 220.77 ms，第二次 20.93 ms；服务稳定窗口 `session_run` 约 12–13 ms。

`model.allow_cpu_fallback` 现在显式控制 CPU 兜底：

- `true`：GPU 失效时服务继续以限频模式运行，持续上报 warning 和 `CRITICAL vision_provider_degraded`；
- `false`：没有可用 GPU Provider 时模型加载失败，不静默回退 CPU。

现场配置为 `true`，保证监控服务连续性。

### 3.4 视觉 CPU 侧策略

- YOLO 后处理保持 NumPy 向量化；
- 视频采集采用 GStreamer `nvv4l2decoder`，应用队列长度为 1；
- 直播继续由 FFmpeg video-copy 独立推送，不叠加检测框；
- 常规检测固定 2 Hz，人员跟随固定 5 Hz；
- 自行车告警确认 3 帧，事件冷却 10 秒。

日志中的 `dropped_frames=80/84` 是把约 10 fps 的硬件解码输入主动采样为 2 Hz 时丢弃旧帧，`avg_input_age` 约 1 ms，说明没有积压。

## 4. 导航与建图进程边界

本计划中的“导航阶段完整运行”专指完整的导航与定位链路，并不表示完整 SLAM 建图后端仍在运行。静态导航验收时，Nav2、地图定位、避障和 FAST-LIO 里程计前端同时运行，未发送运动目标。

`/lio_odometry` 参数实测：

```text
frontend.odometry_only=true
global_optimization.enable=false
keyframe_record.enable=false
publish.map_en=false
publish.world_points_en=false
```

`/odom/lio_odom` 由 `/lio_odometry` 单一发布，实测约 10.00 Hz，并由 `/localization` 消费。`planner_server`、`controller_server`、`bt_navigator` 和 `collision_monitor` 均存在。完整 `roamerx-mapping.service` 保持 inactive/disabled。

2026-09-11更新：正式建图、导航FAST-LIO前端和NDT/VGICP地图匹配现统一采用前向240°点云，减少正后方无建图对应点参与LIO计算；原始`/front_lidar`及其360° LaserScan避障链保持不裁剪，因此该调整不缩小碰撞监控安全视场。导航launch保留单一参数回滚入口，可在运动回放发现前侧特征不足时恢复FAST-LIO为360°。

因此导航阶段的进程边界是“完整导航与定位链路 + FAST-LIO 里程计前端”，而不是完整 SLAM 建图进程。地图保存后停止全局优化、关键帧记录和地图发布等建图后端，但任何回滚均不得停止导航定位必需的 `/lio_odometry`。

## 5. 现场验收结果

### 5.1 edge-agent 回调窗口

稳定 10 秒窗口代表值：

| 指标 | 输入/处理频率 | P99 | 说明 |
|---|---:|---:|---|
| `odometry_callback` | 30 Hz 输入 | 0.03–0.13 ms | 仅替换最新值 |
| `odometry_processing` | 10 Hz | 0.29–0.63 ms | telemetry 汇总 |
| `scan_callback` | 10 Hz 输入 | 0.03–0.09 ms | 仅替换最新值 |
| `scan_processing` | 9.8–10.1 Hz | 0.89–2.02 ms | 障碍摘要 |
| `telemetry_snapshot` | 由状态循环触发 | 0.95–2.57 ms | 单次快照 |

`callback_queue_depth` 最大为 2。里程计每 10 秒约覆盖 200 个旧样本，符合 30 Hz 输入到 10 Hz 汇总的设计；没有追赶过期队列。

### 5.2 视觉窗口

```text
effective_fps=2.00
avg_session_run_ms=12–13
providers=TensorrtExecutionProvider,CUDAExecutionProvider,CPUExecutionProvider
avg_input_age_ms≈1
```

15 秒 `tegrastats` 窗口捕获到 `GR3D_FREQ` 24%、44% 和 94%，证明推理实际进入 GPU。视觉 Python 60 秒平均 17.98% CPU。

### 5.3 服务与测试

```text
roamerx-edge-agent.service  active / enabled
roamerx-bike-bot.service    active / enabled
roamerx-mapping.service     inactive / disabled
```

自动化回归：

- edge-agent：421 passed；
- 视觉：28 passed；
- Python 编译检查、YAML 配置加载、安装脚本语法检查通过；
- TensorRT 实际模型预热通过。

## 6. 回滚方案

1. edge-agent 行为异常时，将 `ros_callback_optimization.enabled` 改为 `false` 并重启 edge-agent；保留指标开关以比较回滚前后。
2. TensorRT engine 异常时保留 cache 和日志，先回退 CUDA Provider；是否继续 CPU 由 `model.allow_cpu_fallback` 决定。
3. 不通过 `CPUQuota` 隐藏 Provider 或算法问题。
4. 不删除 engine cache；需要重建时使用预热脚本生成新缓存并在非任务窗口切换。
5. 不停止导航必需的 `/lio_odometry`；只停止完整建图服务。

## 7. 执行顺序完成情况

- [x] 建立进程、回调、Provider、GPU 和视频采样基线；
- [x] 恢复 ONNX Runtime TensorRT/CUDA，并启用 FP16；
- [x] 固定 engine cache，加入安装期预热与显式 CPU fallback；
- [x] 完成 IMU、里程计、激光、匹配状态的 latest-only/限频策略；
- [x] 加入 callback group 隔离、队列覆盖指标和 telemetry 快照计时；
- [x] 完成自动化回归、服务重启和静态导航满载验收；
- [x] 确认完整 SLAM 建图服务退出，仅保留 FAST-LIO 里程计前端与 `/odom/lio_odom` 运行。
