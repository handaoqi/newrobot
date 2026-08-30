# FastGICP/ICP、NDT/VGICP 应用情况与性能分析

> 分析日期：2026-08-30
>
> 适用平台：RoamerX / ROS 2 Humble / Livox Mid-360 / 8 核 Cortex-A78AE NX 计算平台
> 文档性质：技术选型结论、现状评估与后续基准规范

## 1. 结论摘要

ICP、FastGICP、NDT 和 FastVGICP 不是简单的四选一关系。它们擅长的配准层级不同：

| 任务 | 推荐算法 | 结论 |
|---|---|---|
| 连续帧间运动估计 | FastGICP | 在已有较好运动先验和较高帧间重叠时，兼顾几何精度与 CPU 实时性 |
| 地图绝对定位、周期漂移校正 | NDT_OMP | 适合在全图或较大局部地图上进行粗配准，收敛域通常比纯最近邻 ICP 更实用 |
| 已有可靠粗位姿后的局部精化 | FastVGICP | 对墙、地面等局部平面结构有优势，但不应在每帧或 NDT 已足够好时无条件执行 |
| 回环/重定位候选几何验证 | FastGICP | 候选检索先提供位置与 yaw 种子，再用真实三维几何验证；不能把它当作无初值的全局搜索 |
| 兼容回退、交叉诊断 | 普通 ICP | 实现成熟、易解释，但速度、收敛域和退化鲁棒性通常不适合作为现网主判据 |

RoamerX 当前采用的分层组合是合理的：

```text
Mid-360 点云 + IMU
  ├─ FAST-LIO：约 10 Hz 连续相对位姿
  ├─ FastGICP：必要时提供帧间 LiDAR 运动先验
  └─ 约 2 Hz 绝对匹配调度
       ├─ NDT_OMP：全图粗配准
       └─ FastVGICP：仅在 NDT 不够好时，对 18 m 局部地图精化
             ↓
       质量门、连续性门、漂移门、3 帧稳定确认
             ↓
       对 FAST-LIO 锚点做限速平滑校正
```

现有静态真机 A/B 表明，性能瓶颈不是 NDT 本身，而是高质量 NDT 之后仍无条件执行的 FastVGICP 长尾。加入质量门后，定位回调加权均值从 `31.0 ms` 降至 `1.81 ms`，最大值从 `438.5 ms` 降至 `27.9 ms`，定位 CPU 从约 `159%` 降至约 `6.5%–11%`。因此当前不建议把定位链路改成“VGICP 全面替换 NDT”，也不建议恢复每次 NDT 后都运行 VGICP。

需要保留一个重要限制：上述 A/B 是静止机器人上的性能证据，尚未证明运动、转向和弱几何场景下的校正时延与精度。当前冷却门还使静态实际 NDT 频率约为 `0.49 Hz`，低于配置目标 `2 Hz`；必须通过运动 rosbag 或获授权后的短路线测试补齐验证。

## 2. 算法本质与工程差异

### 2.1 ICP

本文中的 ICP 指 PCL 常规点到点 ICP。其基本过程是反复寻找最近邻对应点、求解刚体变换、重新建立对应关系，直到变换量或误差满足终止条件。

优点：

- 算法成熟、依赖少、结果容易解释；
- 点云规模适中且初值准确时，可得到稳定的局部对齐结果；
- 适合作为兼容回退、离线检查和另一种几何估计的交叉诊断。

限制：

- 强依赖初值和点云重叠率，错误初值容易收敛到局部极小值；
- 最近邻搜索和逐轮对应更新开销明显；
- 走廊、单平面、稀疏点云及大量动态物体会产生退化或错误对应；
- `hasConverged()` 只表示优化满足终止条件，不等价于位置正确。

因此 ICP 适合“已有可靠种子后的局部求精”，不适合承担无位置、无航向先验的全局重定位。

### 2.2 FastGICP

GICP 将 ICP 与点到平面思想统一到概率模型中，为源点和目标点估计局部协方差，形成近似“平面对平面”的误差。FastGICP 保留这一目标函数，通过并行化、邻域计算复用和更高效的优化实现降低耗时。

相对普通 ICP，FastGICP 通常具有以下特点：

- 对局部平面结构的利用更充分；
- 对不完全准确的对应关系通常更鲁棒；
- 多线程版本明显快于 PCL GICP；
- 仍然是局部优化算法，初值、重叠率和最大对应距离仍决定成败；
- 协方差估计本身有预处理成本，目标点云能够复用时收益更明显。

RoamerX 中 FastGICP 的适用位置是：

1. 当前已用的帧间点云里程计：前后帧间隔约 100 ms，重叠高，并有 IMU 增量作为初值；
2. 后续目标中的回环或重定位候选验证：Scan Context、RTK 或人工位姿先提供候选，FastGICP 再验证真实三维几何。当前全局恢复验证器仍是两遍 PCL ICP，尚未切换为 FastGICP。

### 2.3 NDT

NDT 将目标点云划分为体素，并用每个有效体素内点的位置拟合高斯分布。源点经过候选位姿变换后，在这些概率分布上优化似然，不需要逐点建立显式对应关系。

工程优势：

- 目标地图体素化后可重复用于多帧定位；
- 在合理分辨率和较好运动先验下，适合 scan-to-map 绝对定位；
- 对点密度变化的敏感性通常低于纯点到点 ICP；
- 能在较大地图上承担粗锁定，再把结果交给局部精配准。

工程限制：

- 体素分辨率是关键参数：过粗会抹平墙角等结构，过细则导致体素样本不足、目标函数不连续且计算增加；
- 仍是局部优化，不具备真正的全地图位置和 360° yaw 搜索能力；
- 长直走廊、大平面和重复结构中，沿弱约束方向可能产生高不确定度；
- 动态点、地图与实时点云预处理不一致会显著恶化分数；
- NDT fitness 与 ICP/GICP fitness 的定义和有效点集合不同，数值不能直接横比。

Autoware 将 NDT 用作点云地图定位，并同时提供初始位姿估计、动态地图加载、执行时间和匹配状态诊断，说明 NDT 仍是成熟的生产级 scan-to-map 路线之一。[Autoware NDT Scan Matcher](https://autowarefoundation.github.io/autoware_core/latest/localization/autoware_ndt_scan_matcher/)

### 2.4 VGICP / FastVGICP

VGICP 同样使用体素加速，但它不是简单的 NDT 替代品。NDT 从体素内点位置直接估计分布；VGICP 将各点的局部协方差聚合到体素中，在保留 GICP 局部表面模型的同时减少昂贵的最近邻查询，并利于 CPU/GPU 并行。

其典型优势是：

- 精度可接近 GICP，而速度显著提高；
- 局部墙面、地面和物体表面结构利用充分；
- 对重复使用的目标体素地图较友好；
- 已有可靠粗位姿时，适合做局部精化和几何验证。

主要代价和风险是：

- 建立体素分布和点协方差需要时间与内存；
- 分辨率、邻域搜索方式和协方差正则化影响明显；
- 低重叠、错误初值和退化结构下仍可能得到错误的局部解；
- 在小型 ARM CPU 上，无条件精化可能形成数百毫秒长尾；
- VGICP 得到更小 fitness，不自动代表绝对位姿更正确，必须检查与粗配准的几何一致性。

VGICP 论文报告其 CPU 可运行约 30 Hz、GPU 约 120 Hz，且精度与 GICP 相当；这些结果证明算法设计的潜力，但不能直接换算为 RoamerX NX 上的耗时。[Voxelized GICP for Fast and Accurate 3D Point Cloud Registration](https://easychair.org/publications/preprint/ftvV/download)

## 3. 性能对比

### 3.1 定性性能矩阵

| 维度 | ICP | FastGICP | NDT_OMP | FastVGICP |
|---|---|---|---|---|
| 基本匹配对象 | 点—点 | 局部协方差—局部协方差 | 点—目标体素分布 | 点协方差—目标体素分布 |
| 最适合任务 | 简单局部精配准、诊断 | 帧间里程计、局部几何验证 | 大地图粗定位、绝对校正 | 局部地图精化、高速 GICP |
| 初值依赖 | 高 | 高 | 中到高 | 高 |
| 大初值误差容忍 | 较弱 | 较弱到中等 | 通常优于纯 ICP，但仍非全局方法 | 中等，取决于体素和对应距离 |
| 平面结构利用 | 普通 | 强 | 由体素高斯表达 | 强 |
| 最近邻搜索成本 | 高 | 并行优化后中等 | 无显式逐点最近邻 | 体素邻域搜索，通常较低 |
| 目标预处理复用收益 | 中 | 高 | 高 | 高 |
| 参数敏感项 | 对应距离、迭代数 | 对应距离、邻域点数、线程 | 体素分辨率、步长、邻域方式 | 体素分辨率、邻域、正则化、线程 |
| 走廊/单平面退化 | 明显 | 明显，Hessian 可诊断 | 明显 | 明显，Hessian 可诊断 |
| CPU 实时潜力 | 低到中 | 高 | 高 | 高，但需控制目标范围和触发率 |
| 无初值全局重定位 | 不适合 | 不适合 | 不适合 | 不适合 |

“NDT 初值依赖低、GICP 精度高”只能作为一般趋势，不能作为固定排序。实际结果会随 LiDAR 视场、点数、地图密度、预处理、初值误差和参数产生反转。

### 3.2 公开实现 benchmark

FastGICP 官方仓库在 Core i9-9900K、约 1.7 万点的示例上报告：

| 实现 | 单次示例耗时 | 目标数据复用后的 100 次平均量级 |
|---|---:|---:|
| PCL GICP | 127.5 ms | 125.5 ms |
| PCL NDT | 53.6 ms | 54.7 ms |
| FastGICP 多线程 | 20.2 ms | 10.2 ms |
| FastVGICP 多线程 | 18.1 ms | 8.1 ms |
| FastVGICP CUDA RBF | 5.9 ms | 2.3 ms |

仓库给出的概括值为 FastGICP 约 40 FPS、FastVGICP 约 70 FPS、FastVGICP CUDA 约 120 FPS。[FastGICP 官方仓库](https://github.com/koide3/fast_gicp)

`ndt_omp` 官方示例在 Core i7-6700K 上显示，8 线程 DIRECT7 的单次 NDT 约 `63.1 ms`，重复 10 次约 `34.3 ms/次`；DIRECT1 更快但稳定性较弱，官方推荐 DIRECT7 作为速度与稳定性的折中。[ndt_omp 官方仓库](https://github.com/koide3/ndt_omp)

这些数字只能用于说明实现之间的相对加速，不能直接用于 RoamerX 容量规划，原因包括：

- 官方硬件是高频桌面 x86 CPU/独立 GPU，RoamerX 是 8 核 Cortex-A78AE；
- 点数、地图大小、体素大小和初值不同；
- 首次调用包含目标建树或协方差计算，复用调用不包含相同成本；
- 当前 RoamerX `BUILD_VGICP_CUDA=OFF`，不能引用 CUDA FPS 作为现网能力；
- 端到端回调还包含 ROS→PCL、裁剪、地面过滤、体素化、局部地图裁剪、质量门和消息发布。

### 3.3 新实现趋势

FastGICP 作者已经发布 `small_gicp`。其官方 benchmark 称单线程 GICP 比 PCL GICP 快约 2.4 倍、比 FastGICP GICP 快约 1.9 倍，并改善了多线程扩展性。[small_gicp 官方仓库](https://github.com/koide3/small_gicp)

这使 `small_gicp` 值得进入后续替换候选，但当前不建议直接迁移：RoamerX 已围绕 PCL Registration 接口、现有 FastGICP 类型、质量门和状态消息完成集成，迁移收益必须用同一批 ARM/NX rosbag 验证，且不能只比较孤立内核耗时。

## 4. 典型应用情况

### 4.1 激光里程计

帧间或 scan-to-local-map 配准具有小位姿增量和高重叠，FastGICP、VGICP 和经过良好工程化的 ICP 都可用于此任务。KISS-ICP 也表明，配合自适应阈值、运动补偿和局部体素地图后，点到点 ICP 仍能实现有竞争力的 LiDAR 里程计。[KISS-ICP 官方仓库](https://github.com/PRBonn/kiss-icp)

RoamerX 已由 FAST-LIO 承担连续主里程计，因此 FastGICP 不应成为第二个并行连续主源。它更适合作为 FAST-LIO 不新鲜或初始化阶段的独立短时运动先验，避免重复融合导致双重计量。

### 4.2 点云地图定位

NDT 广泛用于车辆或机器人在预建 PCD 地图中的 scan-to-map 定位。典型链路是 EKF、IMU、轮速或 LIO 提供初值，NDT 输出地图绝对观测，再由滤波器处理连续性和协方差。

RoamerX 的 NDT 角色与此一致：FAST-LIO 提供连续初值，NDT 以较低频率检测并校正累计漂移，而不是每帧替代 FAST-LIO。

### 4.3 局部精配准

FastVGICP 适合在粗配准已进入正确吸引域后，对裁剪的局部地图进一步贴合墙、地面等结构。对整栋建筑 PCD 直接运行 VGICP 会扩大目标预处理和邻域搜索成本，也增加重复结构中的错误对应机会。

RoamerX 当前使用 18 m XY、4 m Z 的局部地图，并要求至少 400 点，是正确的计算边界。进一步的关键是按 NDT 质量选择性触发，而非只要启用 refine 就每次执行。

### 4.4 回环闭合与图优化约束

ICP/GICP 常用于验证地点识别算法给出的回环候选。LIO-SAM 的示例回环即使用 ICP，但其官方说明也指出 ICP 较慢，回环实现是概念验证。[LIO-SAM 官方仓库](https://github.com/TixiaoShan/LIO-SAM)

正确顺序应为：

```text
地点识别/里程/RTK 产生候选
  → 局部子图构造
  → FastGICP 几何验证
  → 重叠、内点、残差、位姿差、退化检查
  → 生成带协方差的图约束
  → 鲁棒核与图优化
```

单独的 `hasConverged()` 或 fitness 不能赋予回环接受资格。重复走廊可能得到数值上收敛但地点错误的结果。

### 4.5 全局重定位

ICP、FastGICP、NDT 和 FastVGICP 都不是完整的全局检索算法。若机器人与最后可信位姿相距较远或 yaw 偏差很大，应先用下列方法之一产生 Top-K 候选：

- RTK/已知地图坐标；
- 操作员提供的初始位姿；
- Scan Context 或其他地点描述子；
- 离散位置与 yaw 搜索；
- 全局特征匹配或其他全局配准方法。

候选再交给 FastGICP/NDT/VGICP 做局部几何验证和精化。重复从同一个错误种子运行 ICP，不会扩大搜索范围。

## 5. RoamerX 当前实现评估

### 5.1 当前配置

| 环节 | 当前实现与参数 | 作用 |
|---|---|---|
| 连续主源 | FAST-LIO，点云约 10 Hz | 连续相对位姿 |
| 帧间 LiDAR 预测 | FastGICP，0.40 m 体素、1.0 m 对应距离、20 次迭代、2 线程 | FAST-LIO 不可用或需要独立运动先验时启用 |
| 粗绝对匹配 | NDT_OMP，DIRECT7，0.15 m 分辨率，全图目标 | 地图锁定和绝对校正候选 |
| 局部精化 | FastVGICP，18 m × 4 m 局部裁剪、0.50 m 体素、1.0 m 对应距离、20 次迭代、2 线程 | NDT 不够好时尝试改善 |
| 稳态调度 | 移动/静止 stride=5，上限 2 Hz | 控制重型匹配频率 |
| 初始化调度 | stride=2，上限 5 Hz | 加快首次锁定 |
| VGICP 跳过门 | 稳定 FAST-LIO 且 NDT score ≤ 0.30 | 避免无收益精化 |
| 精化接受门 | 至少改善 5%，与 NDT 差异 ≤0.30 m/5° | 防止较低分数但几何错误的 VGICP 覆盖 NDT |
| 最终绝对门 | fitness、内点、连续性、3 帧稳定、漂移阈值、最大跳变 | 决定是否校正 LIO 锚点 |

当前预处理使用 `0.5–10 m` 距离、前方 `240°` FOV、地面过滤和 `0.15 m` 体素，与建图点云口径保持一致。这一点对匹配质量往往比更换优化器更重要：如果地图删除了地面或后向点，而定位输入保留这些点，正确位姿也可能得到较差分数。

### 5.2 已有静态 A/B

已有记录覆盖 41 个 10 秒窗口、4124 次点云输入：518 次重处理、3606 次轻处理、1 次旧帧丢弃。结果如下：

| 版本 | 回调/重处理 | 加权均值 | 窗口 P99 | 单次最大 | VGICP | 定位 CPU |
|---|---:|---:|---:|---:|---:|---:|
| 原始运行版本 | 约 9.8 Hz / 约 2.0 Hz | 31.0 ms | 中位 174.2 ms，最大 405.3 ms | 438.5 ms | P90 最高 235.2 ms | 约 159% |
| 调度优化后、无质量门 | 约 9.8 Hz / 约 2.0 Hz | 31.0 ms | 仍有 174–405 ms 长尾 | 438.5 ms | 稳态仍执行 | 44.1% |
| NDT 0.30 跳过门 | 约 10 Hz / 1.26 Hz | 1.81 ms | 中位 14.5 ms，最大 22.4 ms | 27.9 ms | 稳态跳过，P90=0 | 约 6.5%–11% |

在质量门之前，254 次被选的 VGICP 结果中有 199 次 fitness 劣于 NDT。这说明：

1. 当前静态位置的 NDT 已经稳定落入正确解；
2. VGICP 不是必然改善器，目标裁剪、体素和局部结构会使它得到不同目标函数下的结果；
3. 仅凭“VGICP 理论更精确”而无条件调用，会显著增加 CPU 和长尾；
4. 质量门比简单降低 VGICP 迭代次数更有效，因为它直接消除了无收益工作。

### 5.3 当前方案的保留项

- 保留 FAST-LIO 作为唯一连续主定位源；
- 保留 FastGICP 帧间预测，但仅在调度器判定需要时执行；
- 保留 NDT_OMP/DIRECT7 作为全图粗绝对匹配；
- 保留 FastVGICP 的初始化、恢复和低质量 NDT 精化能力；
- 保留 NDT/VGICP 几何差异和改善比例门；
- 保留 10 Hz 轻量位姿发布、约 2 Hz 重型绝对观测的分层调度；
- 当前工作树已加入容量为 1 的后台全局恢复任务和 generation ID 过期结果保护；应完成构建、单元测试和 rosbag 验证后再视为阶段 4 正式完成。后台验证器目前仍是两遍 PCL ICP，Scan Context 也默认关闭。

### 5.4 不建议的改法

- 不建议将 FastVGICP 改为每帧全图匹配；
- 不建议用普通 ICP 直接替换 NDT 进行全图绝对定位；
- 不建议因官方 GPU benchmark 较高就直接启用 CUDA 路径，当前构建和目标硬件上的收益、显存与调度影响尚未验证；
- 不建议比较 NDT 与 VGICP 的 fitness 绝对值后直接选最小者；
- 不建议把一次收敛结果立即注入 UKF，必须保留稳定确认、跳变门和限速校正；
- 不建议把 ICP/GICP 称为“全局 ICP”，除非前面确实有覆盖位置与 yaw 的全局候选搜索。

## 6. 后续可复现 rosbag 基准方案

本节定义基准，不在本报告中声称已经执行。

### 6.1 数据集

| 数据 | 时长/规模 | 可用内容 | 用途 |
|---|---:|---|---|
| `ndt_initial_loss_20260817_025133` | 149.18 s，约 775 MiB | 1493 帧点云、IMU、17 次初始位姿、匹配状态与输出 | 初始定位、错误种子、连续失败与恢复 |
| `20260808_002915_task_0f7411c4` | 317.07 s，约 1.7 GiB | 3169 帧点云、IMU、控制与定位输出 | 正常导航、直行、转弯、端到端负载 |
| `nav_sway_20260826_1555` | 167.79 s，约 30 MiB | 位姿、控制、状态，无原始点云 | 仅用于轨迹和控制结果诊断，不能重跑配准 |

回放必须固定地图版本、TF、预处理配置、线程数和 CPU 功耗模式。算法之间使用完全相同的源点云、目标范围和初值样本。

### 6.2 两层对比

第一层是算法内核对比：

- ICP；
- FastGICP；
- NDT_OMP/DIRECT7；
- FastVGICP/DIRECT7/PLANE；
- 可选候选项 small_gicp GICP/VGICP。

使用相同的 scan-to-local-map 目标和初值，分别记录目标预处理首次成本与目标复用后的稳定成本。该层回答“优化器本身在同一输入上有多快、收敛域多大”。

第二层是生产策略对比：

- NDT-only；
- FastVGICP-only；
- 每次 NDT→FastVGICP；
- 当前质量门控 NDT→FastVGICP；
- FAST-LIO + 低频 NDT/VGICP 完整链路。

该层保留各算法在生产中的合理目标范围，例如 NDT 使用较大地图粗配准、VGICP 使用局部裁剪，回答“哪条链路满足定位质量和实时预算”。

### 6.3 初值扰动与场景

每个选定关键帧从参考种子构造以下扰动：

| 级别 | 平移扰动 | yaw 扰动 | 目的 |
|---|---:|---:|---|
| 正常跟踪 | 0.05–0.10 m | 1–2° | 连续定位精度和稳定耗时 |
| 漂移触发 | 0.30 m | 5° | 对齐当前 LIO 漂移校正门 |
| 局部恢复 | 0.50–1.00 m | 10–15° | 测量局部算法有效收敛域 |
| 收敛域边界 | 1.50–2.00 m | 20–30° | 识别错误接受和失败方式 |
| 全局失配 | 不同地点或 ≥45° | ≥45° | 证明局部配准不能代替候选检索 |

扰动方向应覆盖正负 X/Y 和正负 yaw，场景应覆盖静止、直行、原地转向、墙角、长走廊、开阔区、低重叠、动态人员遮挡及点云短时缺失。

### 6.4 指标

性能指标：

- 目标预处理、源预处理、优化和总耗时的 P50/P90/P99/max；
- 每秒成功处理帧数、超过 80 ms 的慢回调数、消息年龄和旧帧丢弃数；
- 进程 CPU、峰值 RSS、线程数和是否使用 GPU；
- 首次设置目标与复用目标的耗时分别统计。

定位质量指标：

- 收敛率与通过质量门的接受率；
- 错误接受率，必须与正常不收敛分开；
- 平移/旋转误差、相对位姿误差、闭环残差和重复到达锚点的一致性；
- fitness、内点率、重叠率、NDT/VGICP 位姿差；
- Hessian 特征值或条件数，用于识别走廊和单平面退化；
- 校正触发延迟、校正幅度、平滑完成时间及是否引起 Nav2 位姿跳变。

室内数据没有独立真值时，FAST-LIO、NDT 或现有输出只能作为参考轨迹，不能被写成真实精度。报告应把闭环误差、重复定位一致性和人工锚点误差标为代理指标。正式绝对精度需使用测量控制点、全站仪/动捕，或在室外使用质量合格的固定 RTK。

### 6.5 结果解释规则

1. 不跨算法直接比较原始 fitness；只在同一算法、同一预处理和相同评分实现内部比较。
2. `hasConverged()`、低 fitness、较多迭代均不能单独判定成功。
3. 失败率和错误接受率必须同时报告；单纯提高收敛率可能增加错误位置接受。
4. 平均耗时不能替代 P99/max，定位系统首先受长尾和消息积压影响。
5. 内核 benchmark 不能替代端到端回放；ROS 转换、地图裁剪和输出可能占据主要成本。
6. 静止测试不能替代运动测试，尤其不能证明转向时的航向校正和走廊方向约束。

## 7. 最终技术选型

### 7.1 当前生产方案

继续采用以下组合：

```text
连续定位：FAST-LIO
短时独立运动先验：FastGICP
绝对粗校正：NDT_OMP / DIRECT7
条件式局部精化：FastVGICP / DIRECT7 / PLANE
当前全局恢复：可选候选检索 → 两遍 PCL ICP → NDT/VGICP 连续确认
目标全局恢复：候选检索 → FastGICP 权威验证/ICP 诊断回退 → NDT/VGICP 连续确认
```

其中 NDT 与 FastVGICP 的关系应定义为“粗配准 + 条件精化”，而不是并行竞争的两个连续位姿源。NDT 已达到高质量门时直接采用 NDT；只有在初始化、恢复或 NDT 质量不足时才支付 VGICP 成本。VGICP 若没有足够改善，或与 NDT 相差超过 `0.30 m/5°`，必须拒绝覆盖。

### 7.2 后续优先级

1. 用运动 rosbag 验证 `2 Hz` 绝对校正预算和当前冷却门，优先确认漂移达到 `0.30 m/5°` 后的触发时延；
2. 验证当前工作树中的容量为 1 后台恢复任务和 generation ID 保护，再将阶段 4 标记为完成；
3. 将两遍 PCL ICP 验证升级为 FastGICP 权威结果、ICP 诊断回退，并增加重叠率、内点率和退化条件数；
4. 在相同 ARM/NX 数据上比较 FastGICP 与 small_gicp，再决定是否迁移；
5. 只有 CPU 路径仍不能满足 P99 预算时，才评估 FastVGICP CUDA，并同时测试端到端调度与内存成本。

## 8. 参考资料

- Peter Biber, Wolfgang Straßer, *The Normal Distributions Transform: A New Approach to Laser Scan Matching*, IROS 2003. [论文页面](https://doi.org/10.1109/IROS.2003.1249285)
- Aleksandr Segal, Dirk Hähnel, Sebastian Thrun, *Generalized-ICP*, RSS 2009. [论文 PDF](https://www.robots.ox.ac.uk/~avsegal/resources/papers/Generalized_ICP.pdf)
- Kenji Koide et al., *Voxelized GICP for Fast and Accurate 3D Point Cloud Registration*, ICRA 2021. [论文 PDF](https://easychair.org/publications/preprint/ftvV/download)
- [FastGICP 官方仓库与 benchmark](https://github.com/koide3/fast_gicp)
- [ndt_omp 官方仓库与 benchmark](https://github.com/koide3/ndt_omp)
- [small_gicp 官方仓库](https://github.com/koide3/small_gicp)
- [Autoware NDT Scan Matcher](https://autowarefoundation.github.io/autoware_core/latest/localization/autoware_ndt_scan_matcher/)
- [LIO-SAM 官方仓库](https://github.com/TixiaoShan/LIO-SAM)
- [KISS-ICP 官方仓库](https://github.com/PRBonn/kiss-icp)

项目内依据：

- `robot/src/localization/localization/config/config.yaml`
- `robot/src/localization/localization/apps/localization_nodelet.cpp`
- `robot/src/localization/localization/src/localization/pose_estimator.cpp`
- `docs/LOCALIZATION_POINT_CLOUD_CALLBACK_PERFORMANCE_OPTIMIZATION_PLAN.md`
- `docs/GICP_GEOMETRIC_VALIDATION_AND_POST_SAVE_LOCALIZATION_SELF_CHECK_PLAN.md`
