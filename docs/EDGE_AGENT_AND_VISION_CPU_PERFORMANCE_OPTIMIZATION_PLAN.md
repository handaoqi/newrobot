# edge-agent 与视觉推理 CPU 性能优化计划

## 2026-09-10 视觉链路执行结果

- 视觉服务已切换到独立环境 `/home/dogrobot/runtime/nx-edge/data/vision/venv`，避免用户目录中的 CPU 版 ONNX Runtime 覆盖 Jetson GPU 版。
- 实际 Provider 为 `TensorrtExecutionProvider, CUDAExecutionProvider, CPUExecutionProvider`；预热后 `session_run` 约 11–12 ms。
- RTSP 使用 GStreamer `nvv4l2decoder` 硬件解码，失败时自动回退 OpenCV FFmpeg。
- 常规采样和检测稳定为 2.00 Hz；人员跟随保持 5 Hz；告警确认 3 帧、冷却 10 秒。
- 同机实测：视觉 Python 进程由 78.51% CPU 降至 20.40%（60 秒窗口）；包含视频直推 FFmpeg 后约 21.85%，达到不高于 35% 的目标。
- CPU-only 回退仍可运行，但会写入 `CRITICAL vision_provider_degraded` 并将运行状态持续保持为 `warning`。

更新时间：2026-09-06

## 1. 目标与范围

本计划同时处理两条高 CPU 链路：

1. `roamerx-edge-agent` 的高频 ROS 回调、锁竞争和 telemetry 更新；
2. `roamerx-bike-bot` 视觉检测从 TensorRT/CUDA 回退到 CPU 的问题。

导航定位、避障、遥控和安全状态必须保持实时。本计划只降低非关键数据处理频率，不降低控制命令、碰撞监控、定位丢失/恢复和导航动作反馈的实时性。

完整建图 SLAM 不属于本计划的常驻进程。导航阶段只保留定位所需的 LIO 里程计前端，详见第 7 节。

## 2. 现场基线与结论

### 2.1 edge-agent

当前进程：

```text
python3 /home/dogrobot/edge-agent/run_edge_agent.py \
  --config /home/dogrobot/runtime/nx-edge/conf/edge-agent.yaml
```

现场采样结果：

| 指标 | 当前值 |
|---|---:|
| 进程 CPU | 约 72% |
| 线程数 | 28 |
| 主热点线程 | 约 42% |
| 第二热点线程 | 约 9% |
| `/front_lidar/imu` | 约 200 Hz |
| `/odom/localization_odom` | 约 26 Hz |
| `/laser_scan` | 约 10 Hz |
| `/rtk_pvh` | 约 10 Hz |

主要开销点：

- `_on_lidar_imu()` 每条 200 Hz 消息都获取锁、写入窗口并进行周期判断；
- `_on_odometry()` 每条消息都更新 telemetry 字典和传感器时间信息；
- `_on_scan()` 遍历全部 `LaserScan.ranges`，逐点执行三角函数；
- `_on_scan_matching_status()` 在正常状态下也复制预测标签和误差数组；
- 多个回调共用 telemetry 锁，可能导致 ROS 执行器线程争用。

因此当前 72% CPU 更像是高频 Python 回调和数据整理的持续开销，不应通过停止 edge-agent 解决。

### 2.2 视觉检测

当前视觉服务：

```text
/usr/bin/python3 /home/dogrobot/platform/bot-version-test/run_edge.py \
  --config /home/dogrobot/runtime/nx-edge/conf/bike-bot.yaml
```

当前 ONNX Runtime：

```text
/home/dogrobot/.local/lib/python3.10/site-packages/onnxruntime
providers=['AzureExecutionProvider', 'CPUExecutionProvider']
```

日志已明确记录：

```text
TensorRTExecutionProvider is unavailable
falling back to CPUExecutionProvider
```

同时 `tegrastats` 显示：

```text
GR3D_FREQ 0%
```

说明当前 Python ONNX Runtime 是 CPU 构建，配置中的 `tensorrt_enabled: true` 没有使 TensorRT 生效。系统虽然存在 `libnvinfer.so`、CUDA 和 cuBLAS 库，但 Python 包没有暴露 CUDA/TensorRT Provider。

视觉检测单次推理约 160–200 ms，检测频率约 2 Hz，并持续丢弃输入帧。CPU 推理是 `roamerx-bike-bot` 约 80% CPU 的首要原因。

## 3. 总体原则

1. 先建立回调和推理 Provider 基线，再实施改动。
2. TensorRT/CUDA 依赖必须与 systemd 实际使用的 `/usr/bin/python3` 对齐。
3. TensorRT 请求失败时必须显式告警，不能把 CPU 回退伪装成 GPU 已启用。
4. 高频传感器采用“最新值覆盖旧值”，不追赶过期队列。
5. 控制、安全和定位状态回调不降频、不丢弃。
6. 不直接修改 `CPUQuota` 掩盖算法和 Provider 问题。
7. 不在导航阶段停止 `/lio_odometry`；只停止完整建图后端。

## 4. 实施阶段

### 阶段 0：基线与可观测性

记录：

- 两个 Python 服务的 PID、CPU、RSS、线程数；
- `tegrastats` 中的 `GR3D_FREQ`、CPU、内存和 SWAP；
- `onnxruntime.__file__`、版本、`get_available_providers()` 和 `get_device()`；
- 视觉 `detection_window`、平均推理时间、丢帧数；
- edge-agent 每个回调的调用次数、总耗时、P99、最大耗时和丢弃数。

回调计时采用 10 秒汇总，禁止逐条 INFO 日志。新增指标至少覆盖：

```text
imu_callback
odometry_callback
scan_callback
scan_matching_callback
telemetry_snapshot
callback_queue_depth
stale_sample_dropped
```

### 阶段 1：恢复视觉 GPU 推理

1. 在与 systemd 相同的 `/usr/bin/python3` 环境安装与 Jetson/CUDA/TensorRT 版本匹配的 `onnxruntime-gpu`。
2. 验证：

   ```bash
   /usr/bin/python3 -c \
     "import onnxruntime as ort; print(ort.__version__, ort.get_available_providers(), ort.get_device())"
   ```

   必须出现 `TensorrtExecutionProvider` 和 `CUDAExecutionProvider`。
3. 推理 Provider 顺序固定为：

   ```text
   TensorrtExecutionProvider → CUDAExecutionProvider → CPUExecutionProvider
   ```

4. 启用 FP16 和 TensorRT engine cache，缓存目录使用运行时数据盘的固定路径。
5. 首次编译 engine 单独记录耗时和失败原因，禁止在导航任务执行过程中首次编译。
6. 若 TensorRT 不可用：
   - 记录 ERROR；
   - 上报视觉能力降级状态；
   - 是否允许 CPU 兜底必须由显式配置决定。

验收条件：

- 会话实际 Provider 包含 TensorRT；
- `GR3D_FREQ` 在推理窗口出现明显 GPU 活动；
- 平均推理耗时显著低于当前 160–200 ms；
- 不再出现“TensorRT requested but is not active”。

### 阶段 2：视觉 CPU 侧优化

仅在 GPU Provider 恢复后实施：

- 将 YOLO 后处理改为 NumPy 向量化，去除逐候选框 Python 循环；
- 保持检测与直播分离，直播继续使用视频 copy，不叠加检测框；
- 检测线程只处理最新帧，明确限制队列长度为 1；
- 根据实测决定是否将检测输入缩放到 720p/固定尺寸；
- 保持 2–5 Hz 检测策略，不以提高检测频率换取 CPU 占用。

### 阶段 3：edge-agent 高频回调降频与合并

#### 3.1 IMU

将 `/front_lidar/imu` 从每条 200 Hz 全量处理改为：

- 原子保存最新样本；
- 20–25 Hz 定时器采样，或每 8–10 条消息取 1 条；
- 5 秒窗口和异常判定逻辑保持不变；
- `/highlevel_robotstate` 采用同样的最新值策略。

#### 3.2 里程计与 telemetry

- 保存最新 `/odom/localization_odom`；
- telemetry 以 5–10 Hz 汇总；
- 定位安全状态仍由 `/localization_info` 实时驱动，不跟随 telemetry 降频。

#### 3.3 激光扫描

- 使用 latest-only 队列；
- 预计算角度的 `sin/cos`；
- 只遍历前方和左右安全区域需要的角度区间；
- 保持障碍物判断频率不低于 5–10 Hz。

#### 3.4 匹配状态

- 正常状态只保留摘要字段；
- 仅在状态变化、异常或诊断请求时复制完整预测数组；
- DEBUG 日志限频并采用采样输出。

### 阶段 4：锁、执行器和队列隔离

- 高频回调只写轻量 latest-value 缓存；
- telemetry 快照时再一次性获取锁；
- 使用有界队列，满载时丢弃旧数据而不是阻塞 ROS executor；
- 将控制/安全回调与 telemetry/诊断回调分配到不同 callback group；
- 检查周期性 ROS CLI、子进程探测是否重复执行，结果增加缓存和冷却时间。

## 5. 导航阶段的 SLAM 进程边界

导航期间完整建图 SLAM 应退出。当前名为 `mapping` 的进程实际节点为 `/lio_odometry`，参数为：

```text
frontend.odometry_only=true
global_optimization.enable=false
keyframe_record.enable=false
publish.map_en=false
publish.world_points_en=false
```

它是定位所需的 LIO 里程计前端，必须保留并继续发布 `/odom/lio_odom`。只有 `unified_mapping.launch.py` 启动的完整建图后端、关键帧、全局优化和地图输出进程应在地图保存后停止。

## 6. 验收指标

### edge-agent

- CPU 从当前约 72% 降至目标 25–35%；
- IMU 队列不积压；
- 控制命令延迟无明显增加；
- 定位、安全、碰撞和导航动作状态无丢失；
- callback P99、最大耗时和丢帧数可在日志中查询。

### 视觉检测

- 实际 Provider 为 TensorRT/CUDA，而不是仅 CPU；
- `GR3D_FREQ` 在检测时有 GPU 活动；
- CPU 占用明显下降；
- 检测窗口不再持续大量丢帧；
- TensorRT 加载失败能直接定位到库、版本、Provider 或 engine cache 原因。

### 导航

- 完整 SLAM 后端未运行；
- `/lio_odometry` 保持运行；
- `/odom/lio_odom` 持续发布；
- Nav2、定位和避障功能不受 edge-agent 降频影响。

## 7. 回滚方案

- edge-agent 降频通过配置开关控制，异常时恢复原始订阅处理频率；
- TensorRT Provider 失败时恢复已验证的 CUDA/CPU fallback，但必须保留 ERROR 和能力降级告警；
- 不删除 TensorRT engine cache，保留失败日志和版本信息；
- 任何回滚不得停止导航必需的 `/lio_odometry`。

## 8. 执行顺序与提交边界

1. 完成阶段 0 基线；
2. 修复并验证 ONNX Runtime/TensorRT/CUDA 加载；
3. 再实施视觉后处理优化；
4. 实施 edge-agent IMU、里程计、激光和状态回调优化；
5. 做静态、导航、遥控和障碍物回归；
6. 更新执行记录、部署配置和服务重启步骤；
7. 通过现场验收后再提交和推送代码。

本文件只保存分析和执行计划，本轮不直接安装依赖、不修改 systemd、不重启服务。
