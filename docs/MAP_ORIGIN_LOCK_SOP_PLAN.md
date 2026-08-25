# 地图管理：RTK 锁原点与 ENU 建图 SOP（可落地执行计划）

> 版本：v2.2（软件实现版）
> 更新时间：2026-08-24
> 适用范围：地图管理页面、Edge Agent、FAST-LIO-SAM、地图包、平台地图激活与任务绑定

## 0. 计划目标与当前状态

本计划要解决两个问题：

1. 室内建图和室外建图必须走不同的安全门控流程。
2. 室外建图的 RTK/双天线航向/ENU 原点必须在启动正式建图前固定，并能随地图包恢复和审计。

当前工程已完成前端—后端—Edge—SLAM 软件链路实现；现场 RTK 实机验收仍必须按第 7 节执行并留档：

| 能力 | 当前状态 | 说明 |
|---|---|---|
| `indoor` / `outdoor` 建图类型 | 已实现 | 页面显式分流；室内关闭 GNSS 原点/融合，室外严格要求锁原点 |
| `gnss_origin.yaml` 保存 | 已有 | 记录原点经纬高、对齐状态、ENU 到 map 的变换信息 |
| Scan-Context 指纹库 | 已有本地生成链路 | 保存时生成完整二进制指纹库和索引 |
| 回环候选与几何验证 | 已有 Edge 保存链路 | 候选记录在 `scan_context/loop_candidates.csv` |
| 回环轨迹优化 | 已完成 | accepted/geometric_verified 候选进入 GTSAM；优化后重建 `map.pcd`，保留 `map_raw.pcd` |
| C++ GTSAM 回环接入 | 已完成 | C++ 自动读取并转换 `scan_context/loop_candidates.csv`，生成 `loop_closures.csv` 后加入统一因子图 |
| 室外锁原点 10 秒门控 | 已实现，待实机验收 | 三话题质量证据、三项检查连续计时、30 分钟 TTL、断电恢复和人工航向复核已接通 |

原点锁定不得通过在源代码中写死某个场地经纬度实现。原点必须是每次地图会话独立生成、不可被后续普通 RTK 样本覆盖的会话数据。

### 0.1 IMU 来源确认

建图使用的是 NX 板上 Livox Mid-360 激光雷达的内置 IMU，不是控制器 IMU，也不是额外外接 IMU：

```text
Mid-360 雷达驱动
  ├── /front_lidar       PointCloud2
  └── /front_lidar/imu   sensor_msgs/msg/Imu
                         ↓
                    FAST-LIO-SAM
```

当前 SLAM 配置使用 `/front_lidar/imu`，IMU 初始化默认参数为：

```yaml
imu_init:
  sample_count: 600
  max_acc_variance: 0.5
  max_gyro_variance: 0.05
```

按 200Hz 采样约需静止 3 秒。IMU 初始化用于估计加速度计/陀螺仪偏置、重力方向、初始姿态，并保证 LiDAR 去畸变和 EKF 位姿估计稳定。

`/odom/mc_odom` 不作为建图 IMU，也不作为建图的主位姿来源。

## 1. 总体方案

在地图管理页面新增“建图类型”选择，并根据类型进入不同流程：

- 室内建图：不要求 RTK 原点，直接使用 `local_only + NDT` 建图流程。
- 室外建图：必须执行 RTK 锁原点、严格 ENU 坐标和航向复核流程。

室外建图流程为：

```text
准备传感器 → 点击锁定并静止采样 10 秒 → 锁定原点 → 启动 FAST-LIO-SAM（不建图）→ 人工原地转动复核 → 点击确认并开始建图
```

默认采用严格 ENU 坐标：

- `map.x = East`
- `map.y = North`
- `map.z = Up`
- RTK 天线锁定的经纬高作为地图 ENU 原点
- 使用现有标定的 GNSS/机体外参处理天线与机器人基座之间的偏移
- 不在源代码中硬编码具体经纬度，而是为每次地图会话生成不可变的 `gnss_origin.yaml`

室内建图不启动原点锁定状态机，不要求 `/fix` 或 `/rtk_pvh`，地图标记为 `local_only`，定位模式固定为 `ndt`。

室外建图必须使用 RTK 固定原点地图；若保留现有 `transition` 场景类型，`transition` 按室外规则处理，同样必须先锁定原点。

地图包和后端元数据必须保存 `scene_scope`，确保后续地图激活和任务绑定时不能绕过这个选择。

### 建图类型决策表

| 建图类型 | 原点锁定 | 坐标模式 | 定位模式 | 允许场景 |
|---|---:|---|---|---|
| 室内建图 | 否 | `local_only` | `ndt` | `indoor` |
| 室外建图 | 是 | `rtk_fixed` | `rtk_ndt` 或 `ndt` | `outdoor` |
| 室内外过渡 | 是 | `rtk_fixed` | `rtk_ndt` 或 `ndt` | `transition` |

## 2. 主要实现

### 1. Edge Agent 增加原点锁定状态机

原点锁定只对 `outdoor` 和 `transition` 建图会话启用。`indoor` 会话不得误触发原点锁定门控。

新增独立的 `OriginLockMonitor`，复用当前 ROS/Edge Agent 进程，订阅：

- `/fix`
- `/rtk_pvh`
- `/rtk/ntrip_status`
- 可选 `/front_lidar/imu` 与运动状态话题，用于状态提示

状态机：

```text
idle
  → preparing_sensors
  → waiting_position_fix
  → collecting_stable_window
  → locked
  → cancelled / failed
```

默认判定条件：

- `/fix` 为有效 RTK 固定解；同时 `/rtk/ntrip_status` 报告 `rtk_fixed`
- 双天线航向满足现有系统标准：
  - `sol_status == 0`
  - `heading_type > 0`
  - 基线不少于 `0.20 m`
  - 航向标准差不超过 `5°`
  - 数据年龄不超过 `1.5 s`
- 位置 FIX、双天线航向基线 FIX、经纬度对应水平散布小于 `0.02 m` 三项连续有效至少 10 秒
- 使用第一条合格样本建立局部 ENU 参考
- 将经纬度转换为 ENU 后，整个窗口最大水平散布不超过 `0.02 m`
- 任意 RTK 固定解、双天线航向或时间新鲜度失效时，稳定计时重置

实时状态至少包含：

```json
{
  "state": "collecting_stable_window",
  "elapsed_seconds": 7.3,
  "remaining_seconds": 2.7,
  "sample_count": 8,
  "position_fix": true,
  "heading_fix": true,
  "position_spread_m": 0.013,
  "latest": {
    "latitude": 39.0,
    "longitude": 116.0,
    "altitude": 45.2,
    "horizontal_std_m": 0.018,
    "heading_deg": 223.1,
    "heading_std_deg": 1.8,
    "baseline_m": 1.42
  },
  "checks": {
    "position_fixed": true,
    "dual_heading_fixed": true,
    "position_stable": true
  }
}
```

锁定成功后生成：

```yaml
schema_version: 1
origin_type: rtk_anchor
origin_latitude: ...
origin_longitude: ...
origin_altitude: ...
enu_axis: x=east,y=north,z=up
enu_to_map_yaw: 0.0
map_offset_x: 0.0
map_offset_y: 0.0
map_offset_z: 0.0
alignment_locked: 1
alignment_source: prelocked_anchor
lock_duration_seconds: 60
position_spread_m: ...
sample_count: ...
anchor_heading_deg: ...
heading_std_deg: ...
heading_baseline_m: ...
locked_at: ...
```

文件先保存到原点锁定会话目录，启动建图时复制到对应地图会话，最终随地图包上传。

### 2. 新增机器人命令与 REST API

新增命令类型：

- `mapping.origin_start`
- `mapping.origin_cancel`
- `mapping.slam_start`
- `mapping.begin`

新增接口：

```text
GET  /robots/{robot_id}/mapping/origin/status/
POST /robots/{robot_id}/mapping/origin/start/
POST /robots/{robot_id}/mapping/origin/cancel/

POST /robots/{robot_id}/mapping/slam/start/
POST /robots/{robot_id}/mapping/begin/
```

`mapping.start`、`mapping.slam_start` 和 `mapping.begin` 的请求均携带：

```json
{
  "mapping_type": "indoor",
  "scene_scope": "indoor",
  "map_name": "园区室内地图"
}
```

`mapping_type` 只允许 `indoor` 或 `outdoor`：

- `indoor` 自动固定为 `scene_scope=indoor`，不需要 `origin_lock_session_id`。
- `outdoor` 自动固定为 `scene_scope=outdoor`，必须携带有效的 `origin_lock_session_id`。
- 若系统继续支持 `transition`，只能由后端内部映射为室外强制规则，不允许前端以室内流程创建。

行为定义：

- `origin_start`
  - 确认当前没有 SLAM 进程
  - 启动雷达、IMU、双天线 RTK/NTRIP
  - 不启动 FAST-LIO-SAM
  - 进入原点待锁定状态；操作员点击“锁定 ENU 原点”后开始 10 秒三项质量采样
- `origin_cancel`
  - 停止原点采样
  - 保留诊断日志，不产生有效原点
- `slam_start`
  - 室内会话：直接启动 FAST-LIO-SAM，不需要原点锁定文件
  - 室外会话：必须携带有效且未过期的 `origin_lock_session_id`
  - 启动 FAST-LIO-SAM
  - 不调用 `/slam/start_mapping`
  - 等待 IMU 初始化和 SLAM 进程健康
- `begin`
  - 记录人工确认“已完成原地小范围转动复核”
  - 调用现有 `/slam/start_mapping`
  - 进入正式建图状态

现有 `mapping.start` 保留兼容调用，但新地图管理页面必须显式传递 `mapping_type`；对室外场景必须拒绝没有原点锁定会话的请求，返回：

```text
MAPPING_ORIGIN_REQUIRED
MAPPING_ORIGIN_EXPIRED
MAPPING_SLAM_ALREADY_ACTIVE
MAPPING_ORIGIN_LOCK_FAILED
MAPPING_TYPE_REQUIRED
MAPPING_OUTDOOR_ORIGIN_REQUIRED
```

### 3. FAST-LIO-SAM 支持预锁定 ENU 原点

在现有 `MappingAlg` 中增加 `mapping_type`、`gnss_fusion.origin_file` 和严格 ENU 模式：

- SLAM 启动前读取 `gnss_origin.yaml`
- 初始化 `gnss_origin_initialized_`
- 初始化 `gnss_alignment_locked_`
- 将 `enu_to_map_yaw` 固定为 `0`
- 禁止启动后再次通过轨迹拟合或临时航向重新改变地图坐标轴
- 使用锁定时双天线航向初始化机器人在 ENU 中的初始 yaw
- 使用现有 GNSS lever arm 计算天线与 base/lidar 的位置关系
- 后续 RTK 只能作为已固定 ENU 坐标中的融合观测，不能漂移地图原点

室内模式行为：

- 不读取或要求 `gnss_origin.yaml`
- 不等待 RTK 固定解或双天线航向
- 使用现有 SLAM 局部坐标启动
- 保存时生成 `coordinate_mode: local_only`、`origin_status: local_only`、`localization_mode: ndt`
- 即使现场存在 RTK，也不能把室内地图自动标记为 `rtk_fixed`

正式保存时必须保留预锁定的：

- 原点经纬高
- ENU 轴定义
- 锁定时间和质量统计
- 初始航向
- `alignment_source: prelocked_anchor`

现有旧地图仍按 `enu_to_map_yaw` 兼容加载。

### 4. 地图管理页面改造

在 `MapsPage.vue` 的建图区域先增加“建图类型”选择：

- `室内建图`
- `室外建图`

选择室内建图时显示四步操作卡：

1. 选择机器人和室内建图区域
2. 启动雷达和 IMU
3. 启动 FAST-LIO-SAM 并等待 IMU/SLAM 预热
4. 确认可以开始建图

选择室外建图时显示七步操作卡：

1. 选择机器人和开阔锚点说明
2. 启动雷达、IMU、双天线 RTK
3. 静止采样进度
4. 原点质量检查并锁定 ENU 原点
5. 启动 FAST-LIO-SAM，但不开始正式建图
6. 原地小范围转动并观察双天线航向
7. 人工确认航向稳定，开始正式建图

页面实时显示：

- `/fix` 状态及 RTK 解状态
- 经纬度、海拔、水平精度
- 双天线航向、航向精度、基线
- 10 秒原点三项质量进度
- RTK ENU X/Y/Yaw、解状态和数据年龄实时状态条
- 当前最大位置散布
- 通过/失败原因
- 原点锁定经纬度和时间
- FAST-LIO-SAM 进程状态
- NX Mid-360 内置 IMU 话题 `/front_lidar/imu` 状态、样本数和初始化进度
- IMU 初始化状态和首个有效 SLAM 位姿状态
- 当前航向实时数值

按钮设计：

保留现有四个按钮，并新增两个明确的安全门控按钮：

| 按钮 | 适用阶段 | 动作 |
|---|---|---|
| `启动并检查` | 室内初始阶段 | 启动雷达/IMU、FAST-LIO-SAM 预热，等待 IMU 初始化和有效 SLAM 位姿 |
| `启动传感器并检查` | 室外初始阶段 | 启动雷达、IMU、双天线 RTK，显示实时状态并停在原点待锁定步骤；不启动 FAST-LIO-SAM |
| `锁定 ENU 原点`（新增） | 室外原点待锁定阶段 | 启动三项连续 10 秒检查，合格后固化原点并生成 `gnss_origin.yaml` 和 `origin_lock_session_id` |
| `启动 SLAM 并检查航向` | 室外原点锁定后 | 启动 FAST-LIO-SAM，加载原点，进入 IMU/SLAM 预热和航向复核；不开始正式关键帧采集 |
| `确认航向稳定，开始建图`（新增） | 航向复核阶段 | 记录人工确认，开启正式关键帧、轨迹、预积分和指纹库采集 |
| `停止并保存地图` | 正式建图阶段 | 停止正式采集，执行保存、优化、导出、打包和上传 |
| `取消建图` | 原点采样、SLAM 预热、航向复核、正式建图阶段 | 取消当前会话并停止相关进程；原点采样未锁定时不产生有效原点 |
| `刷新状态` | 全流程 | 只读刷新，不改变状态 |

`启动并检查` 是动态按钮：室内显示“启动并检查”；室外在原点锁定前显示“启动传感器并锁原点”，原点锁定后显示“启动 SLAM 并检查航向”。这样只新增两个按钮，同时保留现有页面操作习惯。

按钮门控规则：

- 原点检查运行期间“锁定 ENU 原点”禁用并显示 10 秒进度；失败后可在失败步骤重试。
- 室外未完成原点锁定时，不能启动 FAST-LIO-SAM。
- “确认航向稳定，开始建图”在 SLAM/IMU 预热完成并进入航向复核步骤后激活；实时航向指标供操作员人工判断，不再设置倒计时门控。
- “停止并保存地图”只在正式 `MAPPING` 状态或发散救援状态启用。
- “取消建图”在原点采样、SLAM 预热、航向复核和正式建图阶段均可用。
- 航向转动检查采用实时数据展示和人工确认，不由页面自动判定“已稳定”。
- 所有按钮只发起请求，最终门控必须由后端和 Edge Agent 再次校验，不能依赖前端禁用状态。

地图列表和详情增加：

- `RTK 固定原点 / local_only / legacy_incomplete`
- 原点纬度、经度、海拔
- 原点锁定时间
- 最大位置散布
- 原点锁定质量摘要
- ENU 轴方向

地图包中的 `gnss_origin.yaml` 作为权威数据，数据库的地图元数据只作为展示缓存，避免数据库与地图文件产生第二套真相。

## 3. 测试与验收

### Edge Agent 单元测试

覆盖：

- `/fix` 非 FIX 时不能开始计时
- 双天线航向无效时不能开始计时
- 基线不足、航向标准差过大、数据过期时失败
- 10 秒窗口内最大散布超过 2cm 时重置
- 三项条件连续 10 秒满足后成功锁定
- 锁定结果中的经纬高、航向、统计值和时间戳完整
- 锁定后不能被后续噪声样本覆盖
- SLAM 活跃时拒绝启动原点锁定
- 没有有效原点锁定会话时拒绝启动严格 ENU SLAM

### FAST-LIO-SAM 测试

覆盖：

- 正确读取预锁定原点文件
- `map.x/map.y` 与 East/North 方向一致
- `enu_to_map_yaw=0` 时不再发生启动后坐标轴漂移
- lever arm 转换结果正确
- 预锁定模式不会触发旧的 15m 轨迹对齐逻辑
- 无原点文件时严格模式拒绝启动
- 旧地图仍能使用已有 `enu_to_map_yaw`

### 后端和前端测试

覆盖：

- 原点锁定 API 状态转换
- 命令状态和 Edge 实时状态正确合并
- 未锁原点时 UI 禁用 SLAM/建图按钮
- 10 秒三项质量进度、实时 RTK X/Y/Yaw/解状态/数据年龄和失败原因正确展示
- 地图详情正确显示原点质量摘要
- `rtk_fixed` 地图可用于室外/过渡区
- `local_only` 地图仍只能用于室内 NDT
- 室内建图流程不因 RTK 离线而失败
- 室外建图流程不能通过修改前端请求绕过原点锁定
- 旧地图不因缺少新原点字段而无法加载

### 现场可落地验收清单

#### 室内建图

1. 在地图管理页面选择“室内建图”。
2. 选择机器人和室内区域，确认不需要 RTK 原点。
3. 启动雷达和 IMU，确认传感器在线；不要求 RTK 在线。
4. 启动 FAST-LIO-SAM，确认可以直接进入建图状态。
5. 开始移动建图并保存地图。
6. 检查地图包标记为 `local_only` / `ndt`，且不能绑定到室外或过渡区任务。

#### 室外建图

1. 机器人到预先选定的开阔锚点并保持静止。
2. 在地图管理页面选择“室外建图”。
3. 点击“启动传感器并锁原点”。
4. 确认雷达、IMU、RTK 进程均在线，且无 SLAM 进程。
5. 点击“锁定 ENU 原点”并静止等待三项条件连续满足 10 秒；若 3 秒内没有位置 FIX，当前步骤失败并停止。
6. 确认页面持续显示：
   - 位置 FIX
   - 双天线航向 FIX
   - 经纬度对应 ENU 最大散布不超过 2cm
7. 点击“锁定 ENU 原点”，检查生成的原点经纬高。
8. 点击“启动 FAST-LIO-SAM”，确认仅启动 SLAM，不开始记录地图。
9. 原地小范围转动机器人，观察双天线航向是否连续、无明显跳变。
10. 人工确认航向稳定后点击“开始建图”。
11. 移动建图、保存地图并检查地图包包含 `gnss_origin.yaml` 和 `map_manifest.json`。
12. 重新加载地图，验证 ENU 原点、场景限制和 RTK+NDT 能力仍然存在。

## 4. 实施边界与默认假设

- 不修改机器人运动控制；开到锚点和原地转动由现场操作员完成。
- 建图类型是每次地图会话的必填项；室内默认 `local_only`，室外默认 `rtk_fixed`。
- 原点连续窗口、水平散布、双天线基线和位置 FIX 首次等待作为室外建图默认配置项，默认值分别为 `10s`、`0.02m`、`0.20m`、`3s`。
- 原点锁定会话只允许一个机器人同时存在，取消或过期后必须重新采样。
- 原点经纬高来自 RTK 天线参考点，不在代码中写死具体场地坐标。
- 室内 `local_only` 地图不执行严格 ENU 原点 SOP；室外地图必须执行完整锁原点 SOP。
- 实施顺序为：Edge 状态机与协议 → FAST-LIO-SAM 预锁定支持 → 后端 API → 地图管理 UI → 自动化测试与现场验收。

## 5. 分阶段实施任务与交付物

### 阶段 A：统一协议和状态模型

目标：让页面、后端和 Edge 对“室内/室外”和“原点是否锁定”使用同一套字段。

任务：

1. 固定建图请求字段：

   ```json
   {
     "mapping_type": "outdoor",
     "scene_scope": "outdoor",
     "map_name": "园区室外地图",
     "anchor_name": "东门开阔点"
   }
   ```

2. 固定原点会话字段：

   ```json
   {
     "origin_lock_session_id": "origin-20260822-001",
     "mapping_type": "outdoor",
     "state": "locked",
     "locked_at": "2026-08-22T10:00:00+08:00",
     "origin_file": "/.../gnss_origin.yaml"
   }
   ```

3. 后端不得只相信前端传入的 `scene_scope`。服务端必须重新校验：

   - `mapping_type=indoor` 只能得到 `scene_scope=indoor` 和 `coordinate_mode=local_only`。
   - `mapping_type=outdoor` 必须得到 `scene_scope=outdoor` 和有效的 `origin_lock_session_id`。
   - `transition` 如继续保留，按室外规则强制要求 RTK 固定原点。
   - `local_only` 地图禁止绑定室外、过渡区或 RTK 航点。

交付物：

- 协议字段定义和错误码。
- 后端请求校验。
- Edge Agent 会话结构和状态持久化。
- 向后兼容的旧 `mapping.start` 行为说明。

### 阶段 B：Edge 原点锁定状态机

原点质量采样集中在独立的 `origin_lock.py`，不把连续窗口逻辑散落在 `mapping_adapter.py` 的启动和保存函数中。

状态转移必须满足：

```text
IDLE
  → PREPARING_SENSORS
  → WAITING_POSITION_FIX
  → COLLECTING_STABLE_WINDOW
  → LOCKED

任意状态 → CANCELLED
任意采样状态 → FAILED
LOCKED → EXPIRED
```

具体行为：

1. `PREPARING_SENSORS`：确认雷达、IMU、双天线 RTK/NTRIP 已启动；确认 FAST-LIO-SAM 未启动。
2. `WAITING_POSITION_FIX`：等待位置 FIX、双天线航向 FIX 和数据新鲜度全部满足。
3. `COLLECTING_STABLE_WINDOW`：开始 10 秒连续计时，实时更新三项条件、样本、散布和质量状态。
4. 任意一个必需条件失效，清空当前窗口并回到 `WAITING_POSITION_FIX`，不能带着不连续样本继续累计。
5. `LOCKED`：生成不可变原点文件和质量摘要，向页面返回可启动 SLAM 的状态。
6. 原点锁定会话默认只允许被一次室外建图会话消费；取消、超时或地图启动失败后不得自动复用旧会话。

建议的配置项集中放在 Edge Agent 配置中：

```yaml
origin_lock:
  enabled: true
  stable_duration_seconds: 10
  max_horizontal_spread_m: 0.02
  max_fix_age_seconds: 1.5
  min_heading_baseline_m: 0.20
  max_heading_std_deg: 5.0
  session_ttl_seconds: 1800
```

质量判定注意事项：

- 不能直接用经度小数位差判断 2cm；必须根据当前纬度换算为 ENU East/North 后计算水平散布。
- “位置 FIX”与“航向 FIX”必须分别判定，不允许用单天线位置 FIX 代替双天线航向 FIX。
- 所有状态消息要带 `sample_timestamp` 和 `data_age_seconds`，便于排查延迟数据。
- 锁定结果必须保存首个合格样本、均值、最大散布和最终样本，不能只保存一个未经说明的经纬度。

### 阶段 C：SLAM 启动分层

室内流程：

```text
选择室内 → 启动雷达/IMU → 启动 FAST-LIO-SAM
→ SLAM 预热 → 等待 IMU 初始化 → 有效 SLAM 位姿确认
→ `READY_TO_MAP` 确认可以开始建图 → 正式采集关键帧
→ 保存 local_only 地图
```

室外流程：

```text
选择室外 → 启动雷达/IMU/双天线 RTK → 点击锁定并完成 10 秒三项检查
→ 写入 gnss_origin.yaml → 启动 FAST-LIO-SAM
→ SLAM 预热 → 等待 IMU 初始化 → 有效 SLAM 位姿确认
→ 原地小范围转动 → 人工确认双天线航向稳定
→ `READY_TO_MAP` 确认可以开始建图 → 正式采集关键帧
```

FAST-LIO-SAM 的启动接口需要明确区分“启动节点”和“开始记录地图”：

- `slam_start` 只负责启动节点、加载原点、运行传感器/SLAM 预热并等待健康状态。
- `begin` 才负责开启正式关键帧、轨迹、IMU 预积分和 Scan-Context 数据记录。
- Edge Agent 不能在 `slam_start` 阶段直接调用当前的 `start_data=3`/`ACTIVE` 建图命令。
- `begin` 执行时才调用正式建图服务，并设置 `mapping_capture_enabled=true`。
- SLAM 节点增加 `SLAM_WARMUP` 或等价状态：继续处理 `/front_lidar` 和 `/front_lidar/imu`，但禁止 `recordKeyframe()` 将数据写入正式地图目录。
- 室外没有 `LOCKED` 会话时，`slam_start` 和 `begin` 都必须拒绝。
- 室内不读取原点锁定文件，不因 RTK 离线阻塞启动。
- 原点文件读取失败、字段缺失、坐标系声明不一致时，必须停止室外流程并给出明确错误。

### 预热数据与正式建图数据边界

SLAM 启动后可以接收传感器数据并运行状态估计，但进入 `READY_TO_MAP` 前的数据不能直接作为正式地图关键帧。必须区分：

| 数据类型 | `READY_TO_MAP` 前 | `MAPPING_ACTIVE` 后 |
|---|---|---|
| 雷达/IMU 实时输入 | 接收 | 接收 |
| SLAM EKF/去畸变计算 | 运行 | 运行 |
| rosbag 诊断录制 | 可录制 | 可录制 |
| 正式 `keyframes.csv` | 不写入或只写临时预览 | 正式写入 |
| 正式 `scan_XXXXX.pcd` | 不写入正式目录 | 写入 |
| 正式轨迹 | 不计入正式地图 | 开始计入 |
| IMU 预积分 | 可做预热计算，但不得混入正式地图 | 建立正式关键帧区间 |
| Scan-Context 指纹 | 不进入正式库 | 正式生成 |

“确认航向稳定，开始建图”并进入 `READY_TO_MAP`/`MAPPING_ACTIVE` 时必须：

1. 清理或隔离预热阶段的临时关键帧和轨迹。
2. 重置正式关键帧编号和写盘队列。
3. 重置正式轨迹起点和 IMU 预积分区间。
4. 设置 `mapping_capture_enabled=true`。
5. 进入 `MAPPING` 状态后才调用正式关键帧记录逻辑。

不能仅依靠页面把按钮设为灰色；Edge Agent 和 SLAM 节点都必须在服务端再次检查正式采集门控。

当前 SLAM 的 IMU 初始化和首帧状态是“健康检查”信号，不应直接等同于正式建图状态：

- IMU 初始化：确认 NX Mid-360 内置 IMU 的偏置、重力和初始姿态满足要求。
- 有效 SLAM 位姿确认：确认雷达、IMU、时间同步和状态估计链路正常。
- 正式建图：必须等到 `READY_TO_MAP` 的人工确认后才开启关键帧采集。

### 页面现有 13 步与完整状态机的关系

地图管理页面现有 13 步是外层进度展示：

```text
1 空闲
2 已创建
3 已下发
4 Edge确认
5 启动中
6 IMU初始化
7 首帧确认
8 建图中
9 保存中
10 打包中
11 上传中
12 退出建图
13 已退出建图
```

它不是完整的现场操作状态机。完整状态应在现有步骤内部增加子状态：

| 页面步骤 | 新增/对应子状态 |
|---|---|
| 启动中 | `PRECHECK`、`PREPARING_SENSORS`、室外 `ORIGIN_LOCKING`、`SLAM_STARTING` |
| IMU 初始化 | `SLAM_WARMUP`、`IMU_INITIALIZING` |
| 首帧确认 | `SLAM_POSE_READY`，不代表正式关键帧已经进入最终地图 |
| 建图中 | `HEADING_CHECK`、`READY_TO_MAP`、`MAPPING_ACTIVE` |
| 保存中 | `SAVE_REQUESTED`、`FLUSHING_KEYFRAMES`、`GLOBAL_OPTIMIZATION`、`EXPORTING_MAP` |
| 打包中 | `FINALIZING_PACKAGE` |

室内流程跳过 `ORIGIN_LOCKING` 和 `HEADING_CHECK`，但仍需经过 `SLAM_WARMUP`、`IMU_INITIALIZING`、`SLAM_POSE_READY` 和 `READY_TO_MAP`。

室外流程必须经过：

```text
ORIGIN_LOCKING
→ ORIGIN_LOCKED
→ SLAM_STARTING
→ SLAM_WARMUP
→ IMU_INITIALIZING
→ SLAM_POSE_READY
→ HEADING_CHECK
→ READY_TO_MAP
→ MAPPING_ACTIVE
```

注意：当前页面展示的第 13 个外层状态是“已退出建图”，不是“可以开始建图”。`READY_TO_MAP` 是新增的现场操作子状态，不能与现有 13 个外层状态序号混用。只有进入 `MAPPING_ACTIVE` 后才允许写入正式关键帧和指纹库。

### 阶段 D：原点文件和地图 manifest

`gnss_origin.yaml` 建议采用以下完整结构：

```yaml
schema_version: 1
origin_type: rtk_anchor
origin_latitude: 39.0000000000
origin_longitude: 116.0000000000
origin_altitude: 45.2000
enu_axis: x=east,y=north,z=up
frame_id: map
enu_to_map_yaw: 0.0
map_offset_x: 0.0
map_offset_y: 0.0
map_offset_z: 0.0
alignment_locked: 1
alignment_source: prelocked_anchor
lock_duration_seconds: 60
position_spread_m: 0.013
sample_count: 60
anchor_heading_deg: 223.1
heading_std_deg: 1.8
heading_baseline_m: 1.42
locked_at: '2026-08-22T10:00:00+08:00'
```

室外地图的 `map_manifest.json` 至少必须满足：

```json
{
  "coordinate_mode": "rtk_fixed",
  "scene_scope": "outdoor",
  "origin_status": "fixed",
  "rtk_origin_required": true,
  "localization_mode": "rtk_ndt",
  "frame_id": "map",
  "origin_lock_session_id": "origin-20260822-001"
}
```

室内地图必须明确标记：

```json
{
  "coordinate_mode": "local_only",
  "scene_scope": "indoor",
  "origin_status": "local_only",
  "rtk_origin_required": false,
  "localization_mode": "ndt"
}
```

地图激活、路线编辑、任务创建和任务执行都以地图包中的 manifest 为最终约束来源。数据库缓存字段与地图包不一致时，必须拒绝激活并提示重新同步，不能静默采用较宽松的那一份配置。

### 阶段 E：指纹库和回环联动补齐

原点锁定与回环不是同一个门控条件：原点锁定决定地图是否具备稳定的全球 ENU 基准；回环用于减少长距离建图累计误差。两者应解耦，但最终地图包必须能同时说明二者状态。

当前链路已补齐以下两项：

1. **Scan-Context 到 C++ GTSAM 的约束转换**

   当前 Edge 生成 `scan_context/loop_candidates.csv`；调用 `/slam/global_optimize` 时 C++ 自动完成转换：

   ```text
   loop_candidates.csv
   → 仅保留 accepted=true 且 geometric_verified=true
   → 计算 from/to 相对 Pose3、平移/旋转协方差和 score
   → 写入 loop_closures.csv
   → C++ GTSAM 加入 NDT/IMU/RTK 同一因子图
   ```

   转换失败时保留 `trajectory_raw.csv` 和 `map_raw.pcd`，并在 manifest 中记录失败状态。

2. **优化轨迹后的点云地图重建**

   成功回环后使用 GTSAM 优化关键帧位姿重建 `map.pcd`，同时保留未变换的 `map_raw.pcd`：

   ```text
   raw keyframe clouds + optimized poses
   → transform each keyframe cloud
   → voxel/filter/grid export
   → map.pcd
   ```

   验收时不能只检查轨迹文件存在，还要确认：

   - `map_raw.pcd` 保持原始输出。
   - `map.pcd` 与 `map_raw.pcd` 的内容或点坐标确实反映优化结果。
   - 优化失败时 `map.pcd` 回退为 raw，不能产生半优化地图。

指纹库的上传策略：

- 当前默认上传轻量元数据：`index.json`、`loop_candidates.csv`、轨迹和 manifest。
- 若云端需要跨地图检索、地图合并或算法升级后重算，追加上传 `descriptors.bin`、`ring_keys.bin`、`sector_keys.bin`。
- 若云端需要重新做几何验证，还必须上传对应关键帧点云或可访问的关键帧对象存储地址。
- manifest 增加 `scan_context_storage: local_full | cloud_light | cloud_full`，明确云端是否具备重算条件。

## 6. 失败处理、回滚和安全边界

| 场景 | 必须动作 | 禁止动作 |
|---|---|---|
| RTK 未 FIX | 不开始 10 秒计时；首次等待超过 3 秒则失败并停止 | 不使用单点或历史坐标锁原点 |
| 双天线航向无效 | 重置稳定窗口 | 不使用 SLAM 当前 yaw 冒充 RTK 航向 |
| 位置散布超过 2cm | 清空窗口重新采样 | 不取中位数后强行判定通过 |
| SLAM 启动前原点文件损坏 | 拒绝室外 SLAM | 不自动降级为室内并继续室外任务 |
| 原地转动航向跳变 | 不能开始正式建图，允许重新检查 | 不静默记录为“已确认” |
| 回环优化失败 | 保留 raw 轨迹和 raw 点云 | 不覆盖可用 raw 地图 |
| 地图保存中断 | 标记会话可恢复或失败 | 不生成看似 complete 的 manifest |
| 地图激活时约束不一致 | 拒绝激活并提示修复 | 不忽略 `scene_scope` 或 `coordinate_mode` |

室外原点锁定失败后，允许操作员明确重新选择“室内建图”并新建会话；不得把原室外会话自动改写成室内会话，避免产生坐标语义不明的地图。

## 7. 完整验收矩阵

### 自动化验收

- Edge Agent：原点状态机、计时重置、过期、取消、重复启动、异常数据。
- Edge Agent：室内不依赖 RTK；室外无锁定会话不可启动。
- SLAM：预锁定 ENU 原点读取、坐标轴方向、lever arm、yaw 固定和失败退出。
- 后端：权限、状态转换、错误码、重复请求幂等性和会话消费。
- 前端：按钮门控、原点 10 秒质量进度、航向人工确认、实时状态、失败原因和地图详情。
- 地图包：manifest 字段、原点文件、raw/optimized 轨迹、回环状态、完整性计数，以及 `diagnostics/rosbag/` 下的实际 rosbag 文件。
- 回环：正例接受、几何验证拒绝、优化跳变拒绝、raw 回退和优化点云重建。
- 上传包：轻量包和完整指纹包两种模式的文件清单与 manifest 声明。

### 现场验收记录模板

每次室外验收必须保存以下证据：

```text
机器人编号：
地图名称：
原点锁定会话 ID：
锚点描述：
开始时间：
锁定时间：
位置 FIX：通过 / 失败
双天线航向 FIX：通过 / 失败
稳定采样时长：
样本数：
最大 ENU 水平散布：
平均水平精度：
双天线基线：
航向标准差：
原点文件校验值：
地图 manifest 校验值：
原地转动人工确认：通过 / 失败
回环状态：accepted / no_valid_loop / optimization_rejected
是否生成 map_raw.pcd：
是否生成 trajectory_raw.csv：
是否生成 trajectory_optimized.csv：
map.pcd 是否基于优化位姿重建：
地图重新加载结果：通过 / 失败
```

### 最终放行标准

只有同时满足以下条件，室外地图才允许进入可用地图列表：

1. 原点锁定会话状态为 `locked`，且三项条件连续 10 秒的质量证据完整。
2. `gnss_origin.yaml` 和 `map_manifest.json` 的坐标模式、场景和会话 ID 一致。
3. FAST-LIO-SAM 在正式建图前已加载原点，且原地航向检查通过。
4. 地图保存过程中没有未处理的 SLAM 或数据完整性错误。
5. raw 轨迹和 raw 点云存在；若使用优化结果，优化轨迹和优化点云都存在且来源明确。
6. 回环状态和指纹库上传级别在 manifest 中可追溯。
7. 地图重新加载后，室外/过渡区限制、RTK 初始化和坐标轴方向仍然正确。

## 8. 推荐实施顺序

```text
1. 固定 mapping_type / scene_scope / origin session 数据契约
2. 实现 Edge OriginLockMonitor 和质量采样
3. 实现 origin status/start/cancel 接口
4. 实现 slam_start 与 begin 的双阶段门控
5. 实现 FAST-LIO-SAM 预锁定原点读取和严格 ENU 模式
6. 地图管理页面增加室内/室外分支和实时质量面板
7. 接入地图 manifest、原点文件和地图激活约束
8. 打通 Scan-Context accepted 候选到 loop_closures.csv（已完成）
9. 实现优化位姿后的点云重建和 raw 回退（已完成）
10. 完成自动化测试、现场室内验收、现场室外验收
11. 根据云端用途决定上传轻量指纹还是完整指纹库
```

任何阶段未通过验收，都只能保留在开发/测试状态，不得仅因为生成了地图文件就标记为“室外 ENU 建图完成”。

## 9. 2026-08-22 实施结果

本计划的软件部分已落地，实际代码入口如下：

- Edge：`origin_lock.py` 实现 `/fix`、`/rtk_pvh`、`/rtk/ntrip_status` 三源质量采样，10 秒三项连续窗口、3 秒位置 FIX 超时、2 cm 最大水平散布、失败清零、原点原子落盘和 TTL 恢复。
- Edge：建图命令拆分为 `mapping.origin_start`、`mapping.origin_cancel`、`mapping.slam_start`、`mapping.begin`，并保留旧 `mapping.start` 的室内兼容行为。
- SLAM：新增 `WARMUP` 状态。预热期正常完成雷达内置 IMU 初始化、去畸变、EKF 和有效位姿建立，但 `mapping_capture_enabled=false`，不记录正式关键帧；人工确认后才打开采集门。
- SLAM：室外预热从会话 `gnss_origin.yaml` 加载固定 ENU 原点；室内预热显式关闭 GNSS 原点与融合，防止复用历史室外原点。
- 后端：增加原点状态/启动/取消、SLAM 预热、正式开始五个 REST 入口，并把新命令加入中心与 Edge 的协议白名单。
- 前端：增加室内/室外模式卡、原点三项检查卡、10 秒进度、RTK ENU X/Y/Yaw/解状态/数据年龄状态条、状态驱动按钮门控、动态完整状态机和响应式操作布局；航向复核提示原地小范围转动，确认按钮在预热完成后激活。
- 地图包：同步诊断录制开启时，将 rosbag 的 `metadata.yaml`、`.db3` 等实际文件写入 ZIP 的 `diagnostics/rosbag/`，并计入诊断数据指标后上传云端。
- 恢复：`mapping_workflow.json` 持久化工作流。Edge 重启后只恢复 TTL 内的锁定原点，航向稳定状态必须重新实时采样，不继承旧确认。

软件验证结果（2026-08-24）：Edge 全量测试 158 项通过；前端 21 项测试和生产构建通过；原点三项连续窗口、人工航向确认、实时 RTK ENU 状态与 rosbag ZIP 文件均有自动化覆盖。`robot_slam` ROS2 包和后端仍沿用此前已通过的编译、系统检查与协议/REST 验证结果。

尚未由软件测试替代的工作只有现场验收：真实双天线 RTK 三项条件连续 10 秒锁定、原地转动并人工确认航向、室内一次完整建图、室外一次完整建图、诊断 rosbag 随地图 ZIP 上传，以及保存后重新加载地图。现场证据必须填写第 7 节模板。
