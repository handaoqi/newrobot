# IMU 漂移诊断与整改方案

诊断日期：2026-08-25
诊断方式：**实机在线采集**（NX Orin NX，ROS 2 Humble，服务运行中）+ 全仓源码勘察
状态：诊断完成。P0-1 / P0-2 与 §2.5 方案 A **已于 2026-08-25 落成代码并编译通过**；P0-3 已完成定性（结论见 §2.3.1，与原判断相反）。
**所有改动均未重启服务，实机尚未生效，主验收指标（§5.2 静止漂移 A/B）尚未取得。**

---

## 0. 结论摘要

「IMU 漂移严重」的主因不是 IMU 硬件差，而是**定位节点把估好的陀螺零偏丢弃了**。

- 实测陀螺零偏 **0.740 °/s（2662 °/小时）**，其中偏航轴 z 占 **-0.703 °/s**。
- 定位侧 UKF 的零偏状态**恒为 0 且被过程噪声 `1e-6` 冻结**，永远估不出来，也没人把静止标定的结果注入进去。
- 建图侧（FAST-LIO2）在 ESKF 里正常估计零偏，所以**建图还行、跑巡检就漂**——与现场现象一致。

另有三个放大因子：aarch64 上 SLAM 并行被完全关闭、LiDAR 端到端时延 150 ms 未补偿、IMU 协方差全零且足式里程计完全退出定位链路。

**传感器选型（§2.5 补测）**：3588 板上的 BMI088 实测零偏 **0.285 °/s**，偏航轴 **-0.137 °/s**，比雷达内置 IMU 分别好 **2.6 倍**和 **5.1 倍**；3588 SPI0 上还有一对经转台六位置/六速率/温度标定的 ST LSM6DS 芯片（`gyro_range: Dps250`），精度潜力更高但有三个落地障碍。**不过换 IMU 修不了主因**——零偏被丢弃是软件缺陷，换更好的 IMU 后 UKF 依然当它是 0。先修 P0-1。

---

## 1. 实测基线

采集环境：机器人静止，`roamerx-edge-agent` / `localization` / `livox_lidar_publisher` 等服务正常运行。

### 1.1 陀螺零偏与噪声（静止 20 s，4693 帧）

```bash
source /opt/ros/humble/setup.bash
timeout 25 ros2 topic echo /front_lidar/imu --field angular_velocity > /tmp/imu_gyro.txt
```

| 轴 | 均值 (rad/s) | 标准差 (rad/s) | 最小 | 最大 |
| --- | ---: | ---: | ---: | ---: |
| x | +0.002137 | 0.006042 | -0.05084 | +0.04823 |
| y | -0.003380 | 0.001442 | -0.00892 | +0.00387 |
| **z** | **-0.012273** | 0.003797 | -0.03120 | +0.00395 |

- 合成零偏 **\|b\| = 0.012908 rad/s = 0.740 °/s = 2662 °/小时**
- 偏航轴单独 **-0.012273 rad/s = -0.703 °/s = -2532 °/小时**
- 各轴标准差远小于均值 → 采集期间**确实静止**，均值即真实零偏，不是运动残留。

**物理含义**：若该零偏不被补偿，纯惯性推算的航向每分钟偏约 **42°**，每 8.5 秒偏 6°。NDT 每次校正后，预测步立刻重新按这个速率累积误差。

### 1.2 加速度单位与协方差

| 项 | 实测 |
| --- | --- |
| 加速度均值 | x=+0.013926, y=+0.013900, z=+0.997051 |
| **合成模长** | **0.99725** |
| `orientation` | 全 0（x=y=z=0, w=1，即未提供姿态） |
| `orientation_covariance` | 全 0 |
| `angular_velocity_covariance` | 全 0 |
| `linear_acceleration_covariance` | 全 0 |

**结论 1**：模长 ≈ 1.0 而非 9.8 → `/front_lidar/imu` 的加速度单位是 **g**。
→ `robot/src/localization/localization/config/config.yaml:46` 的 `imu_acc_scale: 9.81` **正确**。该行的注释「若 `/front_lidar/imu` 已是 m/s^2，实测后改为 1.0」可以删除，此 TODO 就此关闭。

**结论 2**：Livox 驱动不填任何协方差。按 REP-145，全零协方差对下游滤波器语义不明（可理解为"完全确定"或"未知"），是后续 P1-1 的直接依据。

### 1.3 话题频率与时延

```bash
ros2 topic hz /front_lidar/imu /front_lidar /odom/lidar_odom /odom/localization_odom /odom/mc_odom
ros2 topic echo /sensor_health --once
```

| 话题 | 实测频率 | 备注 |
| --- | ---: | --- |
| `/front_lidar/imu` | 200.2 Hz | 正常 |
| `/front_lidar` | 10.06 Hz | 正常 |
| **`/odom/lidar_odom`** | **7.3 Hz** | **低于雷达 10 Hz → LIO 掉帧** |
| `/odom/localization_odom` | 14.3 Hz（std dev 0.069 s） | 抖动大 |
| `/odom/mc_odom` | 99.6 Hz | 在发布，但订阅数为 0 |

`/sensor_health`（`sensor_health_monitor.cpp` 每秒发布）：

| 传感器 | `measurement_time_offset_ms` | `measurement_time_valid` |
| --- | ---: | --- |
| **LiDAR** | **150.628** | **false** |
| IMU | 2.880 | true |
| odometry | 24.728 | true |
| RTK | 83.075 | true |

阈值来自 `robot/src/navigation/src/robot_navigo/src/sensor_health_monitor.cpp:100`：`std::fabs(offset_ms) <= 100.0`。
注意该指标的定义是 `system_now - msg.header.stamp`（同文件 `:93-95`），即**端到端时延**，既可能是链路/处理延迟，也可能是时钟偏移——两者修法完全不同，见 P0-3。

### 1.4 定位当前状态

```
/localization/decision:
  active_source: "unavailable"    preferred_source: "ndt"    phase: "moving"
  ndt_healthy: false              ndt_score: 8.047
  absolute_stable: false          rtk_quality: "invalid"
  rtk_blocked_reason: "gnss_origin_not_loaded"
  odom_time_source: "not_used"

/localization_info:  status: 0    pos: (0,0,0)
```

`ndt_score` 8.047 远超 `localization/config/config.yaml:105` 的 `ndt_max_fitness_score: 0.50`，定位处于不可用状态。RTK 因室内无解且未加载 ENU 原点而不可用（符合预期）。

### 1.5 计算资源

```
load average: 10.79, 9.99, 9.09   （8 核）
localization_node   125% CPU
内存 9.9 / 15 GiB 已用，可用 5.1 GiB
Jetson Orin NX，MAXN 模式，tj-thermal 64.0 °C（未降频）
```

> ⚠️ **采集时后台有 `colcon build --packages-select robot_slam` 在跑**（`cc1plus` 占 96.7% CPU），另有 `bot-version-test` 推理进程占 69.8%。
> 因此 **load average 与 `/odom/lidar_odom` 的 7.3 Hz 都被污染**，必须在空载状态复测后才能确认 LIO 掉帧的真实程度。
> 本节其余数据（零偏、单位、协方差、时延、NDT 分数）**不受编译影响**，可直接采信。

---

## 2. 根因分析

### 2.1 主因（P0）：定位侧 UKF 的陀螺零偏估而不用

`robot/src/localization/` 派生自 hdl_localization，使用 16 状态 UKF：
`pos(3) + vel(3) + quat(4) + acc_bias(3) + gyro_bias(3)`。

问题链条四步，全部经 grep 核实：

**第一步 — 零偏确实算出来了。**
`src/localization/static_imu_init.cpp:70-83` 在静止窗口内正确计算并保存：
```cpp
init_bias_gyro_ = mean_gyro_;
init_bias_acce_ = mean_acce_;
init_success_   = true;
```
参数见 `config/config.yaml:84-88`：`imu_init_time: 3.0`、`imu_init_queue_size: 600`、`imu_init_max_gyro_var: 0.05`（实测方差 3.6e-5，轻松通过）。

**第二步 — 结果从未被读取。**
```bash
$ grep -rn "GetInitBg\|GetInitBa\|GetGravity\|GetCovGyro" robot/src/localization/
include/localization/static_imu_init.hpp:53:  inline Eigen::Vector3d GetCovGyro() const
include/localization/static_imu_init.hpp:71:  inline Eigen::Vector3d GetInitBg() const
include/localization/static_imu_init.hpp:80:  inline Eigen::Vector3d GetInitBa() const
include/localization/static_imu_init.hpp:89:  inline Eigen::Vector3d GetGravity() const
```
**只有头文件里的四处声明，零个调用点。**

**第三步 — 注入接口存在但从未被调用。**
```bash
$ grep -rn "set_initial_biases" robot/src/localization/
include/localization/pose_estimator.hpp:135:  void set_initial_biases(...);
src/localization/pose_estimator.cpp:109:void PoseEstimator::set_initial_biases(...){
```
**只有声明和定义，零个调用点。**

`PoseEstimator` 构造函数支持零偏参数（`include/localization/pose_estimator.hpp:51-53`）：
```cpp
PoseEstimator(registration, stamp, pos, quat, cool_time_duration,
              Eigen::Vector3d bias_acc  = Eigen::Vector3d(0.0, 0.0, 0.0),
              Eigen::Vector3d bias_gyro = Eigen::Vector3d(0.0, 0.0, 0.0));
```
但 `apps/localization_nodelet.cpp` 的 **6 处构造点（`:453, :969, :1211, :1223, :1556, :2554`）全部只传 5 个参数**，零偏走默认零值。抽查 `:969`：
```cpp
pose_estimator.reset(new localization::PoseEstimator(
  registration, get_clock()->now(), last_init_pos_, last_init_quat_, cool_time_duration));
```

**第四步 — 在线也估不出来。**
`src/localization/pose_estimator.cpp:33-34`：
```cpp
process_noise.middleRows(10, 3) *= 1e-6;   // 加速度计零偏
process_noise.middleRows(13, 3) *= 1e-6;   // 陀螺零偏
```
`1e-6` 的过程噪声等于告诉滤波器"零偏绝不会变"，UKF 从初值 0 出发后基本冻结在 0。

**综合**：UKF 全程认为陀螺零偏 = 0，实际是 z 轴 -0.703 °/s。
`static_imu_init_` 在系统里退化成一个纯粹的**"静止就绪门闩"**（`localization_nodelet.cpp:1099-1100` 喂数据，`:1130-1134` 未成功就打印 `"Radar CallBack Waiting for IMU Initial !!!"` 并走外推），它算出的零偏被完整丢弃。

**为什么建图不明显、巡检明显**：建图侧 `robot/src/slam/`（FAST-LIO2 派生）在 IEKF 里正常估计并更新零偏（`src/process/imu_process.cpp`），且按 `mean_acc.norm()` 做重力尺度归一化，对单位不敏感。两条路径对零偏的处理完全不同——这解释了"地图能建出来但一跑巡检就漂/丢"。

#### 2.1.1 表述修正（2026-08-25 复核）

上面"第三步"的说法需要收紧：**构造函数内部早就把 `bias_gyro` 接到了状态量上**，不是没接线，而是没人传实参。

```cpp
// src/localization/pose_estimator.cpp
mean.middleRows(10, 3).setZero();                 // bias_acc 形参被忽略（见红线 2）
mean.middleRows(13, 3) = bias_gyro.cast<float>(); // bias_gyro 已正确写入
```

零偏参与姿态递推也已确认，这是整个整改方案的支点：

```cpp
// include/localization/pose_system.hpp:78
Vector3t gyro = raw_gyro - gyro_bias;
```

所以 P0-1 的实质改动只有两处：6 个构造点补实参，以及在标定完成后用 `set_initial_biases()` 补写那些早于标定建立的估计器。后者必须放在 `points_callback` 里——它持有 `pose_estimator_mutex`，而 `imu_callback` 不持有，写在 IMU 回调里是数据竞争。

#### 2.1.2 三条安全红线（务必先读，尤其第一条）

**红线 1：绝不把 `GetInitBg()` 的兄弟 `GetInitBa()` 传给 `bias_acc`。**

`static_imu_init.cpp` 里 `init_bias_acce_ = mean_acce_`，而同文件的重力估计那一行是被注释掉的（`// gravity_ = -mean_acce_/mean_acce_.norm()*9.81;`），`GetGravity()` 恒返回零向量。因此 `GetInitBa()` 返回的是**含重力的原始加计均值，模长 ≈ 9.81 m/s²**，是真实加计零偏的 100~1000 倍。而 `pose_system.hpp:71-74` 已经在世界系扣过重力：

```cpp
Vector3t g(0.0f, 0.0f, 9.80665f);
next_state.middleRows(3, 3) = vt + (acc - g) * dt;
```

传进去 = 重力扣两次 = **滤波器秒级发散**。要用加计零偏，必须先修 `static_imu_init.cpp` 的重力估计。

**红线 2：绝不"顺手修" `pose_estimator.cpp` 里 `mean.middleRows(10, 3).setZero()`。**
这行忽略 `bias_acc` 形参本身确实是 bug，但在红线 1 未解除之前它是**保护性的**——修它等于给炸弹接引线。已在代码里留注释说明原因，留待与重力修复一起做。

**红线 3：加计零偏过程噪声 `process_noise.middleRows(10, 3)` 保持 `1e-6`（继续冻结）。**
加计零偏在本系统可观测性很差：bridge 模式下加速度输入被置零、`imu_data_filter_num_ = 5` 只用 1/5 样本、`pose_system.hpp` 自己的注释就写着 acceleration 因噪声大而贡献有限。放开只会让滤波器拿加计零偏去换重力和俯仰角。

#### 2.1.3 调参入口（2026-08-25 落地）

陀螺零偏的两个关键量已提为 ROS 参数，目的是**免重编译扫参**：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `gyro_bias_process_noise` | `1e-4` | 零偏过程噪声。`1e-6` 等于冻结在线估计（改动前的行为），`1e-4` 对齐建图侧的 `b_gyr_cov` |
| `gyro_bias_initial_cov` | `1e-4` | 零偏初始协方差。原先跟随全 16 维的 `0.01`，即 σ=0.1 rad/s=5.7 °/s，比它要描述的零偏本身还大一个数量级 |

归因验证分两档，靠 `--ros-args -p` 覆盖切换：档 A `gyro_bias_process_noise:=1e-6` 只注入初值，漂移下降全部归因于零偏注入；档 B 用默认值，相对档 A 的增量归因于放开在线估计。两个改动一起上就没法归因。

### 2.2 次因 A（P0）：aarch64 上 SLAM 并行被完全关闭

`robot/src/slam/src/CMakeLists.txt:24-42`：
```cmake
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)     # ← Jetson Orin NX（8 核 aarch64）走这里
endif()
```

架构判断只匹配 x86 系列，**整个 Jetson 平台落到 `else` 分支**，`MP_EN` 未定义、`MP_PROC_NUM=1`，ikd-Tree 与残差计算的 OpenMP 并行段全程单线程。

实机编译命令行已确认（采集时正在编译）：
```
cc1plus ... -D MP_PROC_NUM=1 ... CMakeFiles/mapping.dir/src/mapping_alg.cpp.o
```

这是**上游 FAST-LIO 就有的历史缺陷**（原版假设 x86 桌面），移植到 Jetson 时没有同步修改。

### 2.3 次因 B（P0/P1）：LiDAR 150 ms 时延未补偿

`/sensor_health` 报 LiDAR `measurement_time_offset_ms: 150.628`，`measurement_time_valid: false`；同一时刻 IMU 只有 2.88 ms。

而 `robot/src/slam/src/config/config.yaml:14-15`：
```yaml
time_sync_en: false             # ONLY turn on when external time synchronization is really not possible
time_offset_lidar_to_imu: 0.0
```
即**不做任何时间补偿**。

若 150 ms 是真实的 LiDAR-IMU 相对时间错位：
- 0.7 m/s 直行 → 每帧约 **10.5 cm** 系统性平移误差
- 0.5 rad/s 转向 → 每帧约 **4.3°** 航向误差

这个量级足以让 NDT 匹配持续劣化，与实测 `ndt_score: 8.047` 相符。

**但必须先定性**：该指标是 `system_now - header.stamp`，包含驱动处理、Zenoh 传输、订阅回调排队等全部环节。如果只是链路时延而 LiDAR 与 IMU 的**相对**时间基准一致，则不需要 `time_offset` 补偿，而应该去解决 CPU 争抢。定性方法见 P0-3。

#### 2.3.1 定性结论（2026-08-25 实测）：不是时间错位，是帧积累（本节前提已推翻）

上面"若 150 ms 是真实的 LiDAR-IMU 相对时间错位"这个假设**经实测不成立**，据此推出的 10.5 cm / 4.3° 误差量级也不成立。

在线采样 12 s，逐条比较 `header.stamp` 与主机时钟：

| 话题 | 样本数 | 中位滞后 | min | max | stdev |
| --- | --- | --- | --- | --- | --- |
| `/front_lidar` | 120 | **107.5 ms** | 105.3 | 138.1 | 4.1 |
| `/front_lidar/imu` | 2395 | **1.4 ms** | 0.6 | 24.7 | 2.4 |

两个话题来自**同一台 MID360、同一条 UDP 通路、同一个 SDK 实例**。设备时钟若相对主机偏移，两者会同步偏移；实测 IMU 只滞后 1.4 ms，说明**雷达设备时钟与主机时钟本就对齐在毫秒级，不存在百毫秒级的时钟偏差**。

那 107 ms 的来源是帧积累：驱动 `lidar.launch.py` 设 `publish_freq = 10.0`，即累积 100 ms 的点云再发布一帧，而 `header.stamp` 取帧首时刻。`107 ms ≈ 100 ms 积累窗口 + 约 7 ms 驱动/传输处理`，stdev 仅 4.1 ms 也印证这是结构性常量而非排队抖动。

**由此推翻两个原有判断：**

1. **`time_offset_lidar_to_imu` 不是正确的解法。** 它补偿的是两个传感器之间的相对时间基准差，而这里两者基准是一致的。填进去等于人为制造一个本不存在的错位。
2. **给雷达开 PPS / PTP 硬件同步也消不掉这 107 ms。** 同步能修的是时钟偏差，而这里的时延来自帧积累窗口的定义方式。

**真正可用的手段**（本轮未做，需实测权衡）：
- 提高 `publish_freq` 到 20 Hz —— 积累窗口降到 50 ms，代价是单帧点数减半，NDT 匹配的点数支撑会变弱，且回调频率翻倍会加重已经饱和的 CPU（见 2.3.2）。
- 下游按扫描中点补偿约半个窗口（50 ms）—— 不动驱动，但需要确认建图与定位两侧的取值口径一致。

**未确认项**：MID360 驱动是厂商预编译件（`/opt/robot-driver/install/livox_driver/`，无源码，二进制中查不到 PTP/timesync 符号），`MID360_config.json` 也没有任何同步配置块。该型号究竟支持哪种硬件同步方式需向厂商核实，本文不据记忆下结论。

#### 2.3.2 顺带确认：主机侧时间同步已经健康，不是问题点

`/etc/chrony/chrony.conf`：
```
refclock PPS /dev/pps0 prefer refid PPS poll 2 precision 5e-4
server 192.168.234.1 iburst minpoll 1 maxpoll 1
```

实测 `chronyc tracking` / `sources`：

```
Reference ID : 50505300 (PPS)     Stratum : 1
RMS offset   : 0.000001812 s      Skew    : 0.455 ppm
#* PPS          0  2  377   4  +1296ns[+1892ns] +/- 500us
^- firefly      1  1  377   1  -6565ns[-6565ns] +/- 588us
```

`/dev/pps0` 为 `pps_from_gpio`。**NX 主机时钟已被 RTK 的 PPS 锁定，RMS 偏差 1.8 µs，reach 377 满格；3588（`firefly`）同为 stratum 1，与 NX 相差 6.6 µs。** 跨两台机器的时间基准不需要整改。

另有一项与本节相关的负载观测：定位节点点云回调耗时中位 **103 ms**（n=718，min 93，max 1661），而雷达周期是 100 ms —— 回调已经打满，与 2.5 节的 CPU 争抢判断一致。这才是 P0-2（aarch64 并行开关）要解决的对象。

### 2.4 次因 C（P1）：协方差全零 + 足式里程计完全退出

**协方差**：Livox 三个协方差数组实测全零。仓库里已经有补齐工具：
- `robot/script/robot/imu_covariance_sanitizer.py:12-22` → `/front_lidar/imu_sanitized`（补 yaw 0.03、yaw-rate 0.02）
- `robot/script/robot/ndt_odom_sanitizer.py:32-41` → `/odom/ndt_odom_sanitized`（按 confidence 动态设 xy/yaw 方差，z/roll/pitch 设 1000）
- `robot/script/robot/odom_stamp_sanitizer.py:15-37` → `/odom/mc_odom_sanitized`（改写时间戳为接收时刻、去重复位姿、补协方差）

但这三个节点**只服务于默认关闭的影子通道**：`robot/script/robot/official_ukf_shadow.yaml`（`robot_localization` 的 `ukf_node`，`publish_tf: false`），开关 `USE_OFFICIAL_UKF` 在 `start_navigation_real.sh:16` 默认为 `false`。生产链路吃的仍是原始零协方差数据。

**足式里程计**：`/odom/mc_odom` 以 99.6 Hz 正常发布，但订阅数为 0，被两处显式关闭：
```yaml
# localization/config/config.yaml:12-14
robot_odom_topic: "/odom/mc_odom"
enable_robot_odometry_prediction: false

# slam/config/config.yaml:68-70
odom_guard:
    # Disabled for live mapping until the lower-controller timestamp is fixed.
    enable: false
```
关闭理由（`docs/NAVIGATION_SENSOR_DESCRIPTION.md:32`）是控制器时间戳来自开机单调时钟且航向会漂。**而 `odom_stamp_sanitizer.py` 正是为解决这个问题写的，却没接进生产。**

后果：LiDAR 单点失效时没有任何冗余，只能靠 `imu_odom_bridge` 硬撑（上限 10 m / 20 s，`localization/config/config.yaml:75-76`）——而这个 bridge 依赖的正是零偏错误的 IMU 推算。

### 2.5 传感器选型：3588 的 IMU 精度明显优于雷达内置 IMU（2026-08-25 补测）

#### 硬件拓扑（实机勘察）

**NX 板**：无 `/dev/spidev*`，无本地 IMU。定位与建图唯一的惯性输入就是 `/front_lidar/imu`（Livox Mid-360 内置，200 Hz）。

**3588 板**：有 ROS 2 Humble + eCAL，**两条 SPI 总线上挂着两套独立 IMU**：

```
feb30000.spi (SPI3)  ── spi3.0  bmi088_gyro   驱动 bmg160_spi     → iio:device2
                      └─ spi3.1  bmi088-accel  驱动 bmi088_accel_spi → iio:device0
                         ↑ Bosch BMI088，内核 IIO 驱动，可直接从 sysfs 读

feb00000.spi (SPI0)  ── spi0.0  spidev（无内核驱动）
                      └─ spi0.1  spidev（无内核驱动）
                         ↑ 双芯片 MIMU，由用户态 imu_driver 读取
```

SPI0 上那对芯片是 **ST LSM6DS 系列**——依据是 `imu_driver` 二进制里的寄存器符号 `CTRL1_XL_ODR_MASK` / `CTRL2_G_ODR_MASK`（`XL` 指加速度计是 ST 的命名习惯，BMI088 用的是 `ACC_CONF` / `GYRO_BANDWIDTH`），以及配置里的 ODR 833 Hz 正是 LSM6DS 的标准档位。

#### 实测零偏对比（同一台机器人，静止状态）

| | **雷达内置 IMU** `/front_lidar/imu` | **3588 BMI088** IIO 原始读数 |
| --- | ---: | ---: |
| 采样 | 4693 帧 @ 200 Hz，20 s | 800 组，sysfs 轮询 |
| bias x | +0.002137 rad/s (+0.122 °/s) | +0.004139 rad/s (+0.237 °/s) |
| bias y | -0.003380 rad/s (-0.194 °/s) | -0.001414 rad/s (-0.081 °/s) |
| **bias z（偏航）** | **-0.012273 rad/s (-0.703 °/s)** | **-0.002382 rad/s (-0.137 °/s)** |
| **\|bias\| 合成** | **0.012908 rad/s = 0.740 °/s = 2662 °/h** | **0.004981 rad/s = 0.285 °/s = 1027 °/h** |
| 噪声 std x/y/z | 0.006042 / 0.001442 / 0.003797 | 0.001084 / 0.001532 / 0.001653 |
| 量程 | 未知（驱动闭源） | ±500 dps（scale 0.000266 rad/s/LSB） |
| 出厂标定 | **无**（协方差全零、无姿态输出） | 见下 |

**结论：3588 的 BMI088 在合成零偏上优 2.6 倍，在最关键的偏航轴上优 5.1 倍，噪声也更低。**

> 测量方法差异说明：BMI088 通过 sysfs 逐通道 `cat` 读取，三轴不同步且采样率低于 200 Hz。因此**噪声 std 只能作定性参考**；但静止状态下的均值（即零偏）不受采样方式影响，可直接采信。两次采集时间不同（雷达 IMU 约 17:00，BMI088 约 18:15，后者处于 `cooling_standby`），温度条件不完全一致，BMI088 采集时片上温度约 29 °C。

#### SPI0 那对 ST 芯片才是真正的高精度选项

`imu_driver` 二进制里的标定符号显示这是一套**经过转台标定的惯性模块**，远超普通消费级 MEMS：

```
accel_6pos_calib_param        gyro_6rate_calib_param
accel_temp_6pos_calib_param   gyro_temp_6pos_calib_param
turntable_pos_calib_flag      turntable_rate_calib_flag
mimu_poscalib_float           mimu_tempcalib_float
CALIB_TIME                    IMU_FAULTCODE_IN_TEMPCALIB
forsense/imucalib_param.c
```

即：**六位置标定 + 六速率标定 + 温度标定，用转台完成**，标定参数存在 flash 里，运行时经 `imu::Compensation::compensate()` 补偿输出 `ImuCalibrated`。这正好补上雷达 IMU 最缺的两块——零偏标定和温度补偿。

配置（`/opt/robot-driver/install/imu_driver/share/imu_driver/config/imu_params.yaml`）：
```yaml
topic_name: "/imu_driver/imu_central"    publish_frequency: 200.0
frame_id: "imu_link"                     fetch_frequency: 400.0
imu0_device: /dev/spidev0.0   imu1_device: /dev/spidev0.1   flash_device: /dev/spidev0.2
accel_range: G8   gyro_range: Dps250   odr: Hz833   # 头文件注明"保持默认，不允许配置"
```

`gyro_range: Dps250` 是明确的精度导向选择——量程越窄，同样位数下的分辨率越高。

#### 三个必须先解决的落地障碍

1. **`/dev/spidev0.2` 不存在。** 3588 上只有 `spidev0.0` 和 `spidev0.1`，而配置里标定参数 flash 指向 `spidev0.2`。**标定参数当前读不出来**，`FetchCalibratedData()` / `ReadCompensated()` 会失败。需要确认是设备树少使能了一路片选，还是 flash 挂在别处。**这是最关键的阻塞项**——读不到标定参数，这颗 IMU 的核心优势就不存在。
2. **`imu_driver` 没有部署到 3588。** 该 ROS 包位于 NX 的 `/opt/robot-driver/install/imu_driver/`，但 NX 没有 SPI 设备跑不了；3588 有 SPI 也有 ROS 2 Humble，却没有 `/opt/robot-driver` 目录。全仓 grep `imu_central` 零命中，从未被任何 launch 或脚本启动过。
3. **3588→NX 的时间戳通病。** 现有 3588 数据（`/odom/mc_odom`）的时间戳来自控制器开机单调时钟，这正是 `odom_stamp_sanitizer.py` 要修的问题，也是 `odom_guard.enable: false` 的关闭理由。3588 IMU 走同一条通路，**必然继承同样的时间戳问题**——而 IMU 对时间对齐远比里程计敏感。任何接入方案都必须先解决时间同步，否则引入的误差会抵消精度优势。

#### 现有的 3588 惯性数据通路

`/highlevel_robotstate`（`robots_dog_msgs`）里已经有 `acc`（m/s²）和 `gyro`（rad/s）字段，由 3588 的 `ecal2ros` egg 桥接过来——**3588 的惯性数据其实已经到 NX 了**，只是没人用于定位。另有 `robots_dog_msgs/msg/IMU.msg` 定义了 `quaternion / gyroscope / accelerometer / rpy / temperature`（**带温度字段**），但全仓无使用者。

相关 egg 由 `robot-launch server` 管理，见 `edge-agent/config.example.yaml:195` 的 `controller_runtime_eggs`，其中包含 `imu_daemon` 和 `ecal2ros`。

#### 建议的接入路径（按投入递增）

| 方案 | 做法 | 前提 | 评价 |
| --- | --- | --- | --- |
| **A. 只做交叉校验** ✅ **2026-08-25 已实现，见 §2.5.1** | 订阅 `/highlevel_robotstate` 的 `gyro`，与 `/front_lidar/imu` 比对，差异超阈值时告警 | 无（数据已在） | **最低成本**，能立刻发现雷达 IMU 异常，不改定位链路，零回归风险。建议先做这个 |
| **B. 作为零偏参考** | 用 3588 IMU 的静止零偏作为雷达 IMU 标定的交叉验证，或直接为 §4 的标定 SOP 提供基准 | 无 | 低成本，服务于 P0-1 |
| **C. 双 IMU 融合** | 把 3588 IMU 接入 UKF 作为第二惯性源，或用于姿态（roll/pitch）约束 | 必须先解决时间同步 | 中等收益，中等风险 |
| **D. 换为主 IMU** | 部署 `imu_driver` 到 3588，启用 `/imu_driver/imu_central`，替代雷达 IMU | 需解决 spidev0.2、部署、时间同步三项 | 收益最大，但改动最深 |

**重要判断：不建议先做 D。** 主因 §2.1（零偏估而不用）是**软件缺陷**，换一颗更好的 IMU 并不能修复它——UKF 依然会把零偏当成 0。**先修 P0-1，再评估是否需要换 IMU。** 修完之后雷达 IMU 的 0.74 °/s 零偏会被在线估计吸收，届时两颗 IMU 的实际差距会远小于原始零偏之比。

#### 2.5.1 方案 A 实现（2026-08-25）

| 文件 | 作用 |
| --- | --- |
| `edge-agent/roamerx_edge/imu_cross_check.py` | 比对逻辑本体。**不 import 任何 ROS**，因此可脱离整机跑测试 |
| `edge-agent/tests/test_imu_cross_check.py` | 10 个测试，全绿 |
| `edge-agent/roamerx_edge/ros_adapter.py` | 订阅 `/highlevel_robotstate` 与 `/front_lidar/imu`，喂给比对器 |
| `edge-agent/roamerx_edge/telemetry_collector.py` | `on_imu_cross_check()` → 状态快照 `localization.imu_cross_check` |
| `edge-agent/roamerx_edge/app.py` | `imu_cross_check_mismatch` 告警（`medium`），走 §2.2 建立的同一条通道 |
| `edge-agent/roamerx_edge/config.py` | `ImuCrossCheckConfig`，**带默认值**，既有 `EdgeConfig` 构造不受影响 |

**只比模长，不做外参对齐。** 方案原文给了"先用 `base_link→livox_frame` 静态外参对齐"和"只比对模长"两条路，实现选了后者：同一个刚体转动在任何坐标系下 `‖ω‖` 相同，比对因此完全不依赖外参标定是否准确——而外参恰好是 §2.6 里"未现场标定"的待确认项之一，让监测依赖它等于把一个未知量塞进告警逻辑。

**静止与运动分两套阈值。** 运动时比 `‖ω‖` 之差（默认 3.0 °/s），静止时（两路都 < 2.0 °/s）改用更紧的静态零偏差阈值。原因就是实测那组数：0.740 vs 0.285 °/s 的差距能轻松通过 3 °/s 的运动阈值，而它**正是偏航漂移的来源**。`test_resting_biases_are_compared_against_the_tighter_static_limit` 直接用这两个实测值断言。

**连续 3 个窗口超差才报**（`mismatch_samples`），单个坏窗口按噪声处理；一个正常窗口即清零连击。

**状态取值**：`ok` / `mismatch` / `insufficient_samples` / `body_stream_absent`。
**今天的真实状态是 `body_stream_absent`** —— `/ecal2ros2` 节点在跑但不发布 `/highlevel_robotstate`。这被刻意实现为"缺数据"而不是"不一致"：**没有数据不构成任何关于一致性的结论**，它既不产告警也不清除已有告警。要实测需要单独拉起 3588 桥接。

**实现注记**：回调最初只在超差时触发，而 app 侧的去重闩锁需要一个健康报告才能重新武装——那样告警一旦发出就永远不会解除。改为**每一份报告都送到 app**，由 app 按 `ok` / `mismatch` / 其他分支处理。

### 2.6 待确认项（本轮未下结论）

| 项 | 现象 | 位置 |
| --- | --- | --- |
| `heading_offset_deg` 两侧不一致 | 建图 `180.0` vs 定位 `0.0` | `slam/config/config.yaml:64` / `localization/config/config.yaml:64` |
| GTSAM IMU 因子关闭 | `use_imu_between_factor: false`，注释自承「preintegration does not remove gravity and stacked outdoor/indoor maps in Z」 | `slam/config/config.yaml:120` |
| 回环从未成功 | 最新 manifest `loop_closure_count: 0`, `loop_status: "no_valid_loop"`, `trajectory_source: "raw"` | `runtime/nx-edge/data/jszr/map/map_manifest.json` |
| IMU↔LiDAR 外参未现场标定 | 取自 Mid-360 手册标称值（平移 11.0/23.29/-44.12 mm，旋转为单位阵），`extrinsic_est_en: false` | `slam/config/config.yaml:28-32` |
| 加速度计静止方差门限被注释 | 只有陀螺方差门限生效 | `static_imu_init.cpp:76-79` |
| 3588 标定 flash 片选缺失 | 配置要求 `flash_device: /dev/spidev0.2`，实机只有 `spidev0.0` / `spidev0.1`，转台标定参数读不出 | 见 §2.5 |

---

## 3. 整改方案

按优先级排列。每项给出改动点、预期效果、回归风险、验证手段。

### P0-1　把静止标定的零偏注入 UKF，并放开在线估计

> **2026-08-25 已实现并编译通过（`colcon build --packages-select localization`，exit 0）。尚未重启定位服务，实机未生效。**
>
> 落地形态与下面的原始改动点有出入，以实现为准：
>
> - **6 个构造点收敛成一个工厂** `createPoseEstimator(pos, quat)`（`localization_nodelet.cpp:423`），6 处调用点全部改走它。原方案是"6 处各自补实参"，那样零偏和两个调参会在 6 份代码里各自漂移。
> - **绝不传 `GetInitBa()`**，加计零偏恒为零。理由见 §2.1.2 的第一条红线（`GetInitBa()` 含重力，而 `PoseSystem::f()` 已在世界系扣过一次）。原改动点里"传入 `GetInitBg()` / `GetInitBa()`"的后半句**是错的**。
> - **"沿用上一次成功标定的零偏"没有实现**。标定未完成时构造的估计器零偏为 0（优雅降级为改前行为），改为由 `seedImuBiasesOnce()` 在标定首次成功后一次性补写。原建议需要跨重启持久化零偏，而温度变化会让上一次的零偏在下一次开机时未必更接近真值——退化为 0 再补写是更保守的选择。
> - **补写点在 `points_callback` 里**（`:1266`，IMU 就绪闸门之后），因为它持有 `pose_estimator_mutex` 而 `imu_callback` 不持有。
> - **过程噪声没有硬编码**，提成两个 ROS 参数 `gyro_bias_process_noise`（默认 1e-4，对齐 FAST-LIO 的 `b_gyr_cov`）和 `gyro_bias_initial_cov`（默认 1e-4）。初始协方差必须单独收紧：原来跟着全 16 维的 0.01 走，σ=0.1 rad/s=5.7 °/s，比它要描述的零偏本身还大一个数量级。
> - **加计零偏过程噪声保持 1e-6**（继续冻结），`pose_estimator.cpp:44` 的 `setZero()` **不修**，只补注释。见 §2.1.2 的第二、三条红线。
>
> **验收状态**：§5.2 静止漂移 A/B **尚未取得**。本次会话期间定位从未收敛（1432 次 NDT 拒绝，最高分 5.682，阈值 0.50），没有可读的 yaw；随后建图占用整机，定位栈已停。§5.3 的离线回放 A/B 不依赖实时收敛，素材（59 个包）现成，是目前唯一可行的归因路径。

**改动点**
1. `robot/src/localization/localization/apps/localization_nodelet.cpp` 的 6 处构造点（`:453, :969, :1211, :1223, :1556, :2554`），改为传入 `static_imu_init_.GetInitBg()` / `GetInitBa()`；或在构造后调用已存在的 `set_initial_biases()`。
   - 注意 `:453` 等构造点可能发生在 `static_imu_init_` 完成之前（重定位路径），需要处理"尚未标定"的情形——建议此时沿用上一次成功标定的零偏，而不是回退到 0。
2. `robot/src/localization/localization/src/localization/pose_estimator.cpp:33-34` 的零偏过程噪声从 `1e-6` 放开。

**关于过程噪声取值**：不建议直接拍一个数。`slam/config/config.yaml:24-25` 里 FAST-LIO 侧用的是 `b_gyr_cov: 0.0001` / `b_acc_cov: 0.0001`，可作为量级参照起点。应该用 P0 验证方法里的离线回放做参数扫描，在"能跟上零偏温漂"与"不被 NDT 噪声带偏"之间取值。同时建议把这两个值**提升为 yaml 参数**而非硬编码，便于现场调参。

**预期效果**：直接消除 0.703 °/s 的偏航累积误差源。这是本方案中投入产出比最高的一项。

**回归风险**：中。放开 bias 过程噪声后，若 NDT 观测本身有偏（例如受 2.3 的时延影响），滤波器可能把观测误差错误吸收进 bias 状态。因此**建议 P0-1 与 P0-3 一起验证**，不要单独上线。

**验证**：见 §4。

### P0-2　修复 aarch64 的并行编译开关

> **2026-08-25 已实现。** `robot/src/slam/src/CMakeLists.txt` 去掉了 `CMAKE_SYSTEM_PROCESSOR` 的 x86 限定；实测编译产物已带 `-DMP_EN -DMP_PROC_NUM=3`（见 `build/robot_slam/CMakeFiles/*.dir/flags.make`）。
> 按本节"必要时改为可配置"的提醒，加了缓存变量 `ROAMERX_MP_PROC_NUM`：留空 = 按核数自动，`0` = 关闭，其他值 = 直接指定，**免改代码即可扫参**。
> **未验证**：`/odom/lidar_odom` 改前/改后频率对比需要拉起 LIO 栈且必须空载，尚未执行。

**改动点**：`robot/src/slam/src/CMakeLists.txt:24-42`，去掉 `CMAKE_SYSTEM_PROCESSOR` 的 x86 限定，统一按 `ProcessorCount(N)` 决策。Jetson Orin NX 为 8 核，将进入 `N GREATER 4` 分支得到 `MP_PROC_NUM=3` + `MP_EN`。

**预期效果**：恢复 ikd-Tree 与残差计算的并行，缓解 LIO 掉帧。

**回归风险**：低，但需注意：机器上同时跑着 `localization_node`（125% CPU）、`bot-version-test` 推理（69.8%）等，8 核已相当紧张。`MP_PROC_NUM=3` 是否合适需实测；必要时改为可配置，并考虑给关键节点设 CPU 亲和性或调度优先级。

**验证**：改前/改后对比 `/odom/lidar_odom` 频率，目标是稳定达到 10 Hz。**必须在空载状态下对比**。

### P0-3　LiDAR 150 ms 时延定性

> **2026-08-25 更新：本项的定性已经完成，结论见 2.3.1 —— 不是时间错位，是 10 Hz 帧积累窗口。**
> 因此下面第 3 步（`time_offset_lidar_to_imu` 补偿）**不应执行**，`time_sync_en` 同样不适用，给雷达配 PPS/PTP 也无效。
> 剩余待办改为二选一并实测权衡：把 `publish_freq` 提到 20 Hz（积累窗口减半，代价是单帧点数减半 + 回调频率翻倍，而回调已经饱和），或在下游按扫描中点补偿约 50 ms（需保证建图与定位两侧口径一致）。
> 第 1 步（离线回放看是否掉帧）仍然有效，因为它定性的是 CPU 争抢而非时间基准，与 P0-2 直接相关。

**方法**：用仓库现成的离线工具，不需要新写代码。
1. `robot/script/robot/replay_mapping_rosbag.sh`（隔离 `ROS_DOMAIN_ID=77`）回放已录的包，观察离线复算时 `/odom/lidar_odom` 是否仍掉帧——若离线正常，说明是**在线 CPU 争抢**导致的链路时延，应走 P0-2 和资源调度，而非时间补偿。
2. 直接从 bag 里比对 `/front_lidar` 与 `/front_lidar/imu` 的 `header.stamp` 相对关系（`recording_manifest.py` 已有读 bag 的现成代码），判断是否存在固定的相对偏移。
3. 若确认存在固定相对偏移，再用 `slam/config/config.yaml:15` 的 `time_offset_lidar_to_imu` 补偿；同时评估 `localization` 侧是否需要对应参数（当前该侧无此参数）。

**注意**：`time_sync_en: true` 是 FAST-LIO 的软件时间同步，上游注释明确说"ONLY turn on when external time synchronization is really not possible"，不应作为首选。

### P1-1　把协方差补齐节点接入生产链路

**改动点**：让生产链路消费 `imu_covariance_sanitizer.py` / `ndt_odom_sanitizer.py` 的输出，而不是只服务于默认关闭的 `official_ukf_shadow.yaml`。

需要先确认自研 UKF 是否真的读取 IMU 消息里的协方差字段——`pose_estimator.cpp` 的过程噪声是硬编码矩阵，可能根本不读。若不读，则本项对自研链路无效，重点应转为 P1-3（评估切换到官方 `robot_localization`）。**实施前必须先确认这一点。**

### P1-2　让足式里程计重新进入定位链路

**改动点**：
1. 用 `odom_stamp_sanitizer.py` 修正 `/odom/mc_odom` 时间戳（该节点就是为此写的）。
2. 重新评估打开 `slam/config/config.yaml:68` 的 `odom_guard.enable` 和 `localization/config/config.yaml:13` 的 `enable_robot_odometry_prediction`。
3. `pose_estimator.cpp:133-159` 已有一套 7 状态 `OdomSystem` UKF，当前配置下未启用，可直接复用。

**预期效果**：为 LiDAR 失效提供冗余，减少 `imu_odom_bridge` 的依赖（而 bridge 本身正受零偏问题拖累）。

**回归风险**：中。这两个开关是**被有意关闭**的，配置注释和 `NAVIGATION_SENSOR_DESCRIPTION.md:32` 都写明了理由（时间戳 + 航向漂移）。重新打开前必须验证 sanitizer 确实解决了时间戳问题，且足式里程计的航向漂移在 `odom_guard` 的约束范围内。

### P1-3　评估切换到官方 robot_localization

仓库已备好完整的影子通道：`official_ukf_shadow.yaml`（local + global 双 UKF）、`start_official_ukf_shadow.sh`、`switch_official_ukf.sh`（含 `rollback`）、`navigation_bringup.launch.py:95` 的 `use_official_ukf` 开关。

官方 `robot_localization` 原生支持 IMU 零偏估计和按传感器协方差加权，能一次性解决 2.1 和 2.4。但这是**重大架构变更**，应在 P0 全部完成、有了可靠的 A/B 基线之后再评估，不在本轮范围。

### P2　待确认项排查

按 §2.6 表格逐项排查。其中 `heading_offset_deg` 不一致和回环从未成功（`loop_closure_count: 0`）优先级较高——后者意味着建图的位姿图优化实际上一直在退化为原始 LIO 轨迹输出。

---

## 4. IMU 零偏标定 SOP（建议新增）

当前每次启动都靠 `imu_init_time: 3.0` 现场重新标定，而标定结果又被丢弃。建议改为：

1. **开机静止标定**：机器人上电后保持完全静止 ≥ 10 s（四足机器人趴伏状态最稳，注意站立状态的伺服微振会污染标定）。
2. **落盘**：写入 `runtime/nx-edge/conf/imu_bias.yaml`，记录 `bias_gyro[3]`、`bias_acc[3]`、`temperature`、`calibrated_at_unix`、以及标定时的方差（用于事后审计）。
3. **启动加载**：`localization` 与 `slam` 启动时读取该文件作为零偏初值，并在滤波器中继续在线跟踪温漂。
4. **失效判据**：文件超过 N 天、或当前温度与标定温度差超过阈值时，提示重新标定。
5. **可观测性**：把当前零偏估计值加入遥测上报（`telemetry_collector.py` 的 `localization.quality` 子对象），使零偏漂移在平台侧可见。

MEMS 陀螺零偏对温度敏感，Livox Mid-360 内置的 BMI088 也不例外。当前 `robots_dog_msgs/msg/IMU.msg:5` 定义了 `int8 temperature` 字段但无任何使用者，厂商 `imu_driver` 有 `ReadCompensated()` 温补接口但驱动未启用——若温漂被证实显著，这是现成的抓手。

---

## 5. 验证方法

全部基于仓库已有工具，不新造轮子。

### 5.1 前置：空载复测（必做）

本次采集受后台编译污染，以下两项必须在**无编译、无额外负载**时复测，作为真实基线：

```bash
uptime                                  # load average 应显著低于 10.79
ros2 topic hz /odom/lidar_odom          # 确认是否真的掉到 7.3 Hz
ros2 topic echo /sensor_health --once   # 确认 LiDAR 时延是否仍为 150 ms
```

若空载下 `/odom/lidar_odom` 恢复到 10 Hz 且时延回落到 100 ms 以内，则次因 A/B 的优先级下调，主因 2.1 依然成立且优先级不变。

### 5.2 静止漂移测试（直接验证 P0-1）

机器人静止 10 分钟，记录 `/odom/localization_odom` 的 yaw：

```bash
ros2 topic echo /odom/localization_odom --field pose.pose.orientation
```

- **改前预期**：yaw 以约 0.7 °/s 的速率单调累积（NDT 校正会周期性拉回，表现为锯齿状但整体有偏）。
- **改后预期**：累积速率显著下降，锯齿幅度减小。

这是最直接、最容易复现的验收指标。

### 5.3 离线 A/B 回放（验证 P0-1 / P0-2 / P0-3）

用同一个 bag 跑改前/改后：

```bash
robot/script/robot/replay_mapping_rosbag.sh    # 隔离 ROS_DOMAIN_ID=77，可调 rate/acc_cov/gyr_cov
robot/script/robot/replay_navigation_inputs.py # 按 live 时序重发 /front_lidar、/front_lidar/imu、/odom/mc_odom
```

现有素材充足：`runtime/nx-edge/data/rosbags/mapping/` 下已有 **60 个包**，含 9 个必要话题。

对比指标：
- 轨迹闭合误差（起点终点重合场景）
- `ndt_score` 分布（目标：稳定低于 `ndt_max_fitness_score: 0.50`）
- `active_source` 为 `unavailable` 的时长占比
- `/odom/lidar_odom` 输出帧数 / 输入点云帧数（掉帧率）

`replay_mapping_rosbag.sh` 已支持通过参数调 `acc_cov` / `gyr_cov`，可直接用于 P0-1 的过程噪声参数扫描。

### 5.4 现场巡检回归

在真实巡检路线上跑完整任务，统计：
- `task.pausing` 且 `reason_code == "LOCALIZATION_LOST"` 的次数（改前基线 vs 改后）
- 每次丢失的恢复耗时
- 前端 `TaskExecutionPage.vue` 地图上定位丢失点标记的分布（`taskMapState.js:36-60` 已实现该功能）

---

## 6. 相关文档

- [导航传感器说明](NAVIGATION_SENSOR_DESCRIPTION.md) —— LiDAR/IMU/RTK/TF 权威说明
- [定位丢失恢复与自愈方案](LOCALIZATION_SELF_HEALING_PLAN.md) —— 本文降低丢失**频率**，该文处理丢失**之后**的处置
- [地图管线 mcap、Foxglove 与路线预演方案](MAP_PIPELINE_MCAP_FOXGLOVE_AND_ROUTE_PREVIEW_PLAN.md) —— 离线回放基础设施，是本文验证方法的载体
- [FAST-LIO-SLAM ENU/GPS/IMU 统一计划](FAST_LIO_SLAM_ENU_GPS_IMU_UNIFIED_PLAN.md) —— 提出的 `CombinedImuFactor` 等**尚未实现**，是计划而非现状
- [工作区改动梳理](WORKING_TREE_TRIAGE_20260825.md) —— 本文整改点与未提交的 C1 簇位于同一批文件，建议先固化 C1
