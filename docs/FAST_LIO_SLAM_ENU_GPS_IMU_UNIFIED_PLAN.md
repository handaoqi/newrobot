# FAST-LIO-SAM ENU、GPS与标准IMU因子统一改造计划

## 1. 总结

基于现有 `robot_slam`/FAST-LIO-SAM 实现，不引入上游 LIO-SAM。室外建图流程固定为：

```text
锁定RTK原点
→ 停止Nav2和localization
→ 启动专用ENU转换节点
→ 严格注入lat0/lon0/alt0
→ 启动FAST-LIO-SAM
→ useGPS:true、useImuFactor:true
→ 激光、IMU、GPS、回环进入同一GTSAM因子图
```

室内建图保持 `useGPS:false`，不启动ENU节点，但启用标准IMU因子。

## 2. 核心改造

### 2.1 建图服务互斥

- 原点锁定阶段允许暂时保留 `localization` 获取检查里程计。
- SLAM预热前必须执行 `full-stop`，关闭Nav2和定位节点。
- 启动前检查 `/localization`、Nav2生命周期节点和旧SLAM进程均不存在；未关闭成功则返回 `MAPPING_STACK_STOP_FAILED`。
- SLAM启动后检查 `/odom/localization_odom` 只有FAST-LIO-SAM一个发布者，防止双定位源和TF冲突。

### 2.2 专用ENU转换节点

- 在 `robot_slam` 增加 `slam_enu_converter` 节点：
  - 输入：`/fix`。
  - 输出：`/gnss/enu_odom`，类型为 `nav_msgs/Odometry`。
  - `frame_id=enu`，时间戳严格沿用 `/fix.header.stamp`。
  - 位置协方差从 `NavSatFix` 转换并保留，状态无效或数据过期时不发布有效ENU观测。
- 必填参数为 `lat0`、`lon0`、`alt0`；来源只能是本次锁定生成的 `gnss_origin.yaml`。
- 参数缺失、非有限、越界、三项全零、原点会话ID不匹配或文件校验失败时拒绝启动SLAM。
- Mapping Adapter生成本次启动专用参数文件；systemd建图单元改为启动包含ENU节点和FAST-LIO-SAM的统一launch。
- 室内模式不启动该节点，禁止复用上一次室外原点。

### 2.3 室外关键帧坐标

当前 `robot_slam/mapping` 已在 `<地图目录>/keyframes/keyframes.csv` 保存：

- `x/y/z/yaw`：雷达在 `map` 坐标系下的位姿，`yaw` 单位为弧度；
- `world_x/y/z` 和四元数：IMU/状态位姿；
- `lidar_x/y/z` 和四元数：完整雷达位姿；
- `rtk_latitude/longitude/altitude`、`rtk_heading_deg`：RTK位置和双天线航向。

现有 `x/y/yaw` 不是明确标识的原始ENU坐标。改造后，室外关键帧增加：

- `enu_x/enu_y/enu_z/enu_yaw`；
- `enu_valid`、`enu_origin_session_id`、`enu_origin_sha256`；
- ENU数据时间戳、数据年龄和位置/航向协方差。

`x/y/yaw` 继续保持现有含义，避免破坏旧地图、回环检测和救援脚本兼容性。

### 2.4 GPS和标准IMU因子图

- 增加明确配置：
  - 室外：`useGPS: true`、`useImuFactor: true`；
  - 室内：`useGPS: false`、`useImuFactor: true`。
- 正式建图关闭现有 `applyGnssCorrection` 直接改写FAST-LIO状态的路径，GPS仅通过GTSAM因子进入后端，避免重复融合。
- 因子图状态由单一 `Pose3` 扩展为每个关键帧的：
  - 位姿 `X(i)`；
  - 速度 `V(i)`；
  - IMU偏置 `B(i)`。
- 使用GTSAM `PreintegrationCombinedParams` 和 `CombinedImuFactor`：
  - 配置重力方向和大小；
  - 使用静止初始化获得初始姿态、速度和加速度计/陀螺仪偏置；
  - 使用真实IMU时间戳积分；
  - 加入偏置随机游走和完整预积分协方差；
  - `imu_gap`、时间回退或无效协方差区间不加入IMU因子并记录原因。
- 激光相邻关键帧因子、GPS位置因子、双天线航向因子、IMU预积分因子和回环因子加入同一张图。
- GPS因子读取 `/gnss/enu_odom`，保留杆臂补偿、RTK质量门控、协方差门控和Huber鲁棒核。
- 即使没有回环，只要存在有效IMU或GPS因子也执行图优化；回环不再是启动全局优化的前置条件。
- 扩展 `/slam/global_optimization_status` 和地图清单，记录各类因子数量、`useGPS`、`useImuFactor`、重力、初始偏置、原点哈希及优化前后误差。

### 2.5 建图完成后的回环检测与优化信息条

- 在停止建图后，将“回环检测与全局优化”设为地图保存和打包之间的独立可见阶段：

```text
保存原始地图
→ 扫描回环候选
→ 校验并接受/拒绝回环
→ 触发GTSAM全局优化
→ 重建点云、栅格和优化轨迹
→ 打包上传
```

- 地图管理页面增加单行主信息条，不能把回环优化过程笼统显示为“保存中”。信息条至少支持以下状态：
  - `等待触发`：原始地图尚未完成；
  - `检测中`：显示已扫描关键帧数量和候选回环数量；
  - `优化中`：显示已接受回环数量、因子总数和运行时间；
  - `重建地图中`：显示点云、栅格和轨迹重建进度；
  - `优化完成`：显示纠正量摘要和优化前后误差；
  - `无有效回环`：明确说明仍使用GPS、IMU和激光因子完成优化，或保持原始轨迹；
  - `已回退原始地图`：回环校验或优化失败时说明回退原因，不能显示为建图整体失败；
  - `优化失败`：仅在原始地图也不可用时阻止打包，并展示错误码和错误信息。
- 信息条必须展示触发依据和时间：触发来源、触发时间、检测耗时、优化耗时、地图重建耗时及数据更新时间（`x秒前`）。触发来源区分自动保存触发、人工重试和离线rosbag回放。
- 实时状态以 `/slam/global_optimization_status` 为主；完成后将同一份摘要持久化到 `map_manifest.json`、`mapping_trace.json` 和云端地图元数据，页面刷新、服务重启或上传完成后仍能恢复展示。
- `/slam/global_optimization_status` 扩展为可关联本次地图的结构化状态，至少包含：
  - `mapping_session_id`、地图目录、阶段、进度、更新时间和成功/回退状态；
  - 关键帧数、候选回环数、接受/拒绝回环数及拒绝原因统计；
  - 激光、GPS、双天线航向、IMU预积分和回环因子数量；
  - `error_before`、`error_after`、误差改善比例和优化迭代次数；
  - 原始轨迹、优化轨迹和最终地图实际采用的轨迹来源。

#### 2.5.1 位置和航向纠正展示

- 纠正量必须由同一关键帧的 `trajectory_raw.csv` 与 `trajectory_optimized.csv` 对齐计算，禁止使用不同时间或不同索引的样本直接相减。
- 对每个关键帧记录并可查询：
  - 原始 `x/y/z/yaw` 和优化后 `x/y/z/yaw`；
  - `delta_x/delta_y/delta_z`；
  - 平面位置纠正量 `sqrt(delta_x² + delta_y²)`；
  - 采用角度归一化到 `[-180°, 180°]` 后的 `delta_yaw_deg`；
  - 造成该段纠正的主要约束来源：回环、GPS、航向、IMU或组合约束。
- 信息条默认精简显示：

```text
回环优化完成 · 接受7/12个回环 · 平均位置纠正0.18m · 最大0.64m · 平均航向纠正1.7° · 最大5.2° · 图误差下降70.4%
```

- 展开详情后显示：平均值、RMS、P95、最大值及其关键帧编号；同时显示起点、终点和每个已接受回环两端的优化前后位置与航向差。
- 地图轨迹视图叠加原始轨迹与优化轨迹：原始轨迹使用虚线，优化轨迹使用实线；对纠正量超过阈值的关键帧绘制位移箭头和航向弧线。阈值默认位置 `0.30 m`、航向 `3°`，并允许配置。
- 室外地图额外展示 `map` 与ENU下的纠正量，确认优化没有破坏锁定的ENU原点和航向；若起点平移超过 `0.20 m` 或全局航向偏移超过 `2°`，显示醒目告警并禁止自动激活。
- 所有角度在接口中使用弧度，在页面中统一转换为度并明确标注；位置统一使用米。

## 3. 发散与救援地图

### 3.1 当前判定

- 位姿出现NaN/Inf：立即发散。
- 绝对Z超过5米：立即发散。
- 单帧位移超过1.5米、单帧Z跳变超过1米或速度超过3米/秒，连续3帧后发散。
- 连续10帧没有有效匹配点目前只标记为 `degraded`。
- 现有救援脚本按关键帧速度超过2米/秒或Z跳变超过0.2米截断；找不到异常时排除末尾5帧；剩余不足10个关键帧则救援失败。

### 3.2 改造后行为

- 每次健康检查记录 `last_healthy_keyframe`、触发指标、时间戳和发散原因。
- 发生 `SLAM_DIVERGED` 后立即停止关键帧采集和机器人运动，保留已经落盘的数据。
- 救援优先使用SLAM记录的最后健康关键帧；旧地图缺少该字段时才使用现有速度/Z启发式截断。
- 自动生成并上传标记为 `rescued_degraded` 的救援地图，但 `auto_activate=false`，必须人工检查点云、轨迹和原点后才能设为活动地图。
- GPS跳变、错误回环或优化失败只回退到原始轨迹，不直接把失败优化结果作为救援地图。

## 4. 部署、测试与验收

- 统一版本清单记录：Git提交、FAST-LIO-SAM二进制SHA256、Edge Agent版本、静态配置SHA256、动态原点SHA256和systemd启动路径。
- Mapping Adapter启动前校验版本清单；二进制、配置或服务引用不一致时返回 `MAPPING_DEPLOYMENT_MISMATCH`。
- systemd只允许从同一安装目录启动统一launch；部署后执行daemon-reload，并重启Edge Agent，建图服务保持按需启动。
- 自动化测试覆盖：
  - 建图前Nav2和localization确实停止；
  - 经纬高原点映射到ENU零点，东西北方向、海拔、时间戳和协方差正确；
  - 缺失或错误lat0/lon0/alt0时拒绝启动；
  - 静止、匀速、带IMU偏置、GPS中断、GPS异常值和无回环轨迹；
  - 每个有效关键帧区间生成一个IMU因子，速度和偏置状态可收敛；
  - 室外有效RTK时GPS因子数量大于零，室内始终为零；
  - 关键帧同时保留兼容的 `x/y/yaw` 和可审计的ENU字段；
  - 发散截断、旧地图兼容、救援地图禁止自动启用；
  - 修改安装配置或二进制后版本校验能够阻止启动。
  - 保存完成后信息条依次经过回环检测、图优化、地图重建和完成状态，页面刷新后仍能恢复；
  - 原始/优化轨迹按关键帧正确对齐，位置和航向纠正量、平均值、P95、最大值及角度跨越±180°时计算正确；
  - 无回环、回环被全部拒绝、优化失败后回退原图和正常接受回环四种路径均有明确状态，且只有原始地图也不可用时才阻止打包；
  - 室外优化超过ENU起点或航向保护阈值时禁止自动激活，室内地图不错误执行ENU一致性门控。
- 使用九话题rosbag回放验收，检查时间同步、ENU输出、因子数量、优化前后误差、最终轨迹和地图重建。
- 最终提交合并到私有仓库 `main`，构建、部署、服务重启和运行时版本指纹全部一致后才标记生效。

### 4.1 2026-08-25 执行记录

- 已完成第2.5节：SLAM发布优化/重建/完成状态；Edge持久化 `optimization_summary.json`，并将摘要写入 `map_manifest.json`、逐关键帧纠正写入 `mapping_trace.json`；地图包和云端元数据均携带优化结果。
- 地图管理页已部署单行优化信息条、展开统计、原始/优化轨迹叠加、显著纠正箭头和室外ENU保护告警。
- 室内运行门控已加固：实时GNSS融合、ENU对齐收集、RTK位置因子和双天线航向因子均受 `use_gnss_fusion_` 控制；摘要发现室内RTK因子非零时自动回退并禁止激活。
- 地图139完成端到端验收：108个关键帧、540个候选、接受2个回环；激光因子107、回环因子2、RTK位置/航向因子均为0；图误差 `7.5167 → 2.6326`，下降64.98%；检测48.091秒、优化和重建3.643秒。
- 自动化验证：Edge 171项、SLAM 7项、云端13项通过，前端构建成功；Edge Agent、云端API/worker和公网静态资源已部署生效。
- 截至4.1记录时，标准IMU速度/偏置预积分因子、统一版本指纹门禁及完整rosbag回放矩阵尚未完成；后续执行结果见4.2。

### 4.2 2026-08-25 标准IMU因子、版本门禁与九话题回放记录

- 标准IMU后端已落地：因子图状态扩展为每关键帧 `X(i)/V(i)/B(i)`，使用带ENU重力的 GTSAM `PreintegratedCombinedMeasurements + CombinedImuFactor`；偏置随机游走保留在组合因子的15维协方差中，首帧增加速度与偏置先验。
- 已删除不安全的 `Pose3` IMU增量因子配置；原始加速度按FAST-LIO静止初始化尺度转换为 `m/s²`，每个关键帧区间持久化真实 `dt/acc/gyro`、线性化偏置、速度、重力和协方差。进程恢复时可从 `preint_*.json` schema v2重新装载测量。
- 无回环不再跳过图优化：Edge在关键帧数大于1时总会调用 `/slam/global_optimize`，由NDT、IMU及室外有效RTK共同约束；回环数量可以为0。
- `trajectory_covariance.json` schema v2和 `/slam/global_optimization_status` 已增加 `use_imu_factor`、重力、IMU/偏置/速度先验因子数、末速度、末加速度计偏置和末陀螺仪偏置。
- 云端ZIP现在包含全部 `imu_preintegration/preint_*.json`，诊断大小统计也包含这些文件；地图清单明确记录 `use_imu_factor:true`。
- 增加部署指纹门禁：`mapping-deployment.json`记录Git提交、SLAM二进制、静态配置、Mapping Adapter和systemd单元SHA256；运行配置已强制校验，不一致时以 `MAPPING_DEPLOYMENT_MISMATCH` 阻止建图。
- 自动化验证通过：`robot_slam`标准IMU测试验证静止重力补偿及关闭IMU退化路径；Edge Agent 173项测试通过。最终SLAM二进制SHA256为 `6aa3dc219344867e86f137ee4a7096bbe14db217d16de43bcc7d8c27f78f40e6`。
- 九话题真实rosbag `/home/dogrobot/runtime/nx-edge/data/rosbags/mapping/20260825_170835_V1` 已在隔离ROS Domain回放：107.393秒、30,707条消息、九话题完整。针对Zenoh录包增加Fast DDS回放QoS覆盖，修正 `avoid_ros_namespace_conventions`。
- 回放验收结果：125个关键帧、125个预积分文件、124个IMU组合因子、124个偏置转移约束、124个NDT相邻因子、2个回环；室内RTK位置/航向因子均为0；图误差 `2183940.591 → 17.62485`，优化与地图重建成功。
- 回放地图实际打包结果：ZIP 7.783 MiB、机器狗地图目录54.809 MiB、125个关键帧、25.069米、建图/保存/优化流程180.9秒、诊断数据3,102,187字节（133个文件）；ZIP内含125个预积分文件，允许自动激活。
- Edge Agent已于18:00重启并连接云端MQTT；`roamerx-mapping.service`保持按需启动，下一次正式建图将加载新二进制和配置。
