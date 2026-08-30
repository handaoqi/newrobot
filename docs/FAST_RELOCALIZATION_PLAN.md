# 快速重定位方案（Scan-Context 位置识别接入定位）

实现进展（2026-08-30）：Edge Agent 的有界候选搜索现在会原子写入
`relocalization_search_state.json`，记录搜索来源、种子、候选总数、每个候选的
位姿/拒绝原因以及最终状态（running/committing_best/accepted/failed/superseded/
handoff_failed）。该文件仅用于诊断和断电后
恢复现场，不改变默认定位策略，也不发送运动指令。

同日补充修复：人工 `nav.initial_pose` 和自动重定位已使用定位操作代次互斥；新人工
请求会使旧搜索立即失效。Edge 订阅 `/localization/scan_match_pose`，把每个候选的绝对
NDT 位姿与 score/inlier 关联起来；三帧验证通过后提交最优位姿并停止换候选，只等待
FAST-LIO 接管。初始化匹配增加 1.50 m / 30° 种子偏差门，避免错误局部极值成为可信
位姿。定位节点改为双线程执行器，FAST-LIO 接收使用独立回调组，避免 NDT/VGICP
重计算阻塞造成 0.30 s freshness 假超时。

状态：**已落码编译，默认关闭，未重启生效**。离线留一法评估（§6.1）已跑完并给出结论，回放（§6.3）与实机（§6.4）尚未进行——两者都需要另行征得拉起回放栈 / 重启定位栈的同意。`config.yaml` 的 `relocalization.use_scan_context` 默认 `false`，因此**在显式打开之前，现网行为逐字节不变**。

本文档从 [定位丢失恢复与自愈方案](LOCALIZATION_SELF_HEALING_PLAN.md) 中拆出，只处理"丢失之后怎么快速找回来"，不重复自愈分级和告警链路。建图侧 Scan-Context 的产生过程见 [SLAM 采集与世界位姿计划](SLAM_DATA_CAPTURE_AND_WORLD_POSE_PLAN.md)，本方案是它的**运行时消费方**。

---

## 1. 背景：现在的"全局重定位"不是全局的

`robot/src/localization/localization/src/localization/global_localization.cpp:19` 的 `performGlobalLocalization()` 全文只有一件事：

```cpp
pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
icp.setMaximumIterations(30);
// ... align 两遍，起点是调用方传进来的 initial_trans
```

没有 BBS、没有偏航扫描、没有多假设、没有位置识别。`initial_trans` 来自 `localization_nodelet.cpp:2535` 的包装函数，取的是 `last_init_pos_/last_init_quat_`——**最后一个可信位姿**。

于是重定位的成败**完全由种子位姿决定**，而种子恰好是被漂移侵蚀的那个量：

| 环节 | 参数出处 | 时间 | 0.703 °/s 偏航零偏累积 |
| --- | --- | --- | --- |
| 判定丢失（迟滞 3 帧，行进中 NDT 2 Hz） | `ndt_failure_hysteresis_frames: 3` | ~1.5 s | 1.05° |
| 开环到 armed 一次 ICP | `runtime_relocalization_failure_threshold: 10` | ~1 s | 0.7° |
| 每一轮重试 | `runtime_relocalization_retry_seconds: 5.0` | 5 s | **3.5°** |
| 4 轮重试后 | — | 20 s | **14°** |

一个 30 迭代、无偏航搜索的 ICP 在 14° 偏航误差下没有任何机会。这就是"一旦丢了就再也回不来"的机理——**每次重试的起点都比上一次更差，是一场必输的赛跑**。

同时，**绑架 / 真正的全局失位在当前架构下不可恢复**：没有任何机制能在不依赖先验位姿的情况下确定机器人在哪。这不是调参问题，是能力缺失。

### 1.1 零偏注入解决不了这个

[IMU 方案](IMU_DRIFT_DIAGNOSIS_AND_REMEDIATION_PLAN.md) 的 P0-1 把种子的**衰减速率**降到约 1/14（残差 ~0.05 °/s），20 秒重试只累积 1°，种子留在 ICP 盆地内。那是必要的，但它只是让赛跑变慢，没有改变"只有一个假设、且这个假设必须一开始就基本正确"的结构。绑架场景仍然归零。

### 1.2 当前唯一的绝对观测：RTK 双天线

`localization_nodelet.cpp:1514` 的 `seedPositionFromGnss(stamp, reason, require_fixed, require_heading)` 已经实现了从 RTK 固定解 + 双天线航向直接播种位置和偏航，并且已经挂在重定位路径上（`:1893`，`performGlobalLocalization` 之前）。

**室外 RTK 固定解可用时，快速重定位这个问题已经被解决了。** 本方案的全部价值在于**室内 / RTK 拒止 / 遮挡**场景。

---

## 2. 关键发现：地图里已经有位置识别描述子，运行时从来没用过

建图流程已经在每个地图会话里产出了 Scan-Context 资产（`edge-agent/roamerx_edge/map_loop_closure.py`），但**只用于建图后的回环检测，定位节点完全不知道它们的存在**。

```
runtime/nx-edge/data/jszr/map/<session>/
├── keyframes/
│   ├── keyframes.csv          # 每帧 world_* / lidar_* 位姿、scan_context_index、RTK、逐帧陀螺零偏
│   └── scan_000NN.pcd         # 关键帧点云（世界系，已实测确认）
└── scan_context/
    ├── descriptors.bin        # float32, N × 20 × 60
    ├── ring_keys.bin          # float32, N × 20
    ├── sector_keys.bin        # float32, N × 60
    ├── index.json             # rings=20 sectors=60 max_radius_m=80 descriptor=max-height frame=lidar
    └── loop_candidates.csv    # 建图回环候选，对重定位无用（见 §3.3）
```

实测：95 关键帧的地图，`descriptors.bin` = 456000 B = 95 × 20 × 60 × 4，与 `index.json` 完全自洽。

### 2.1 现有地图资产盘点（146 个会话实测）

| 类别 | 数量 | 可用性 |
| --- | --- | --- |
| 有关键帧点云 + 有描述子 | 40 | 完全可用 |
| 有关键帧点云、无描述子（建图早于该特性） | 63 | **可在载入时于 C++ 侧现算** |
| 两者都无 | 43 | 退回现状（Tier 2） |

**所以"在 C++ 侧从关键帧点云现算描述子"比"解析 `descriptors.bin`"覆盖面大 1.6 倍（103 vs 40），且查询与底库由同一份代码产生，不存在 Python/C++ 漂移。** 这是本方案的选型依据。

### 2.2 上传包不含描述子和关键帧点云

`edge-agent/roamerx_edge/mapping_adapter.py` 的 `upload_files` 清单包含 `keyframes/keyframes.csv`、`scan_context/index.json`、`scan_context/loop_candidates.csv`，但**不含** `descriptors.bin`、`ring_keys.bin` 和 `keyframes/scan_*.pcd`。

- 对本机建图的地图无影响（本方案的常规场景）。
- 但从平台下发到**另一台**机器人的地图将两者皆无，Tier 1 不可用。
- 建议顺带把 `scan_context/descriptors.bin` 和 `ring_keys.bin`（合计 464 KB）加入上传清单，让跨机地图复用保住重定位能力。这是本方案唯一涉及 edge-agent 的改动，独立可回滚。

---

## 3. 方案：三层重定位，只新增中间一层

| 层 | 触发条件 | 手段 | 状态 |
| --- | --- | --- | --- |
| Tier 0 | RTK 固定解 + 双天线航向有效 | `seedPositionFromGnss(require_fixed=true, require_heading=true)` | **已存在**，`:1893` |
| Tier 1 | Tier 0 不可用，且地图有 Scan-Context 底库 | 位置识别 → top-K (位置, 偏航) 种子 | **本方案新增** |
| Tier 2 | 以上都不可用 | 从最后可信位姿做 ICP | **已存在**，退化兜底 |

### 3.1 Tier 1 数据流

```
raw_points_ptr_ (0.15 m 体素降采样 + 4 cm 外参平移，仍等价于雷达系)
  → 构建 20×60 max-height 描述子
  → ring key (20 维) L1 距离粗筛，取 top-(3×K)
  → 每个候选做 60 次循环移位，取最小 L1 距离 → (最佳候选, 粗偏航, 距离)
  → top-K 输出为种子：
       位置 = 该关键帧的 world_x/y/z
       偏航 = 关键帧偏航 + shift·6°   （符号已由实测定标，见 §6.1）
       横滚/俯仰 = 关键帧姿态（种子只需落进 ICP 盆地，精化由 NDT 完成）
```

**关于"雷达系"的一处订正。** 本方案定稿时认为 `raw_points_ptr_` 是 `fromROSMsg` 直出的原始点云。实际读码（`localization_nodelet.cpp:1957`–`:1958`）是先 `downsample()`（`points_voxel_filter_size: 0.15`）再 `TransformPoints()`。后者施加的 `gravity_transform_` 变量名容易误导——它由 `config.yaml:182` 的 `init_T` 装载，而 `init_T` 是**雷达到 IMU 的外参**：旋转为单位阵，平移仅 (−0.011, −0.02329, 0.04412) m。

也就是说**它并不做任何重力对齐**，只平移 4 cm。相对 4 m 的环宽（80 m / 20 环）和 6° 的扇区宽可以忽略，所以"查询侧不需要额外变换"的结论成立，但成立的理由与原文不同。同时这也意味着 §5.2 的俯仰/横滚担忧**依然完全有效**——这条链路上没有任何一步做过重力对齐。

### 3.2 成本：约 3 ms，可在 `points_callback` 内联，不需要工作线程

这是本方案最重要的工程结论，直接决定了改动规模。

| 步骤 | 计算量 | 估计 |
| --- | --- | --- |
| 查询描述子 | ~2×10⁴ 点 × (hypot + atan2) | ~0.4 ms |
| ring key 粗筛 | 95 × 20 | 可忽略 |
| 移位搜索 | 15 候选 × 60 移位 × 1200 元素 ≈ 1.1×10⁶ 次绝对差 | ~1–3 ms |

定位节点现使用双线程执行器，FAST-LIO 接收位于独立 callback group；NDT/VGICP 与
LIO 接收不会再互相阻塞。Scan-Context 查询本身仍应保持约 3 ms 的预算，避免挤占
点云处理链路。

对比：现有 `performGlobalLocalization` 要对 12 MB 全局地图跑两遍 30 迭代 ICP，那才是真正阻塞回调的部分。**本方案不新增任何重量级在线计算，反而把昂贵的 ICP 从"唯一手段"降级为"验证手段"。**

### 3.3 与建图侧 Scan-Context 的语义差异（必须注意）

`map_loop_closure.py:567` 的 `_retrieve_candidates()` 有 `MIN_KEYFRAME_GAP = 30`：跳过索引相近的关键帧，因为**回环检测**要的是"回到很久以前去过的地方"。

**重定位的语义完全相反——任何关键帧都是合法答案。** 因此：

- 不能复用 `loop_candidates.csv`（它是带 gap 排除的回环候选，对重定位无意义）。
- 必须用原始描述子重新检索，检索时**不设任何索引间隔排除**。

### 3.4 验收复用现有初始化路径

Tier 1 **只负责提出种子，不负责判定成功**。拿到最佳种子后：

```
last_init_pos_ / last_init_quat_ = 种子
has_set_init_pose_ = true
is_init_success_   = false
```

随后交给 `localization_nodelet.cpp:1922` 的既有初始化闸门验证：

```cpp
if (init_result.is_converged_ && init_result.fitness_score_ < init_match_score_threshold_) {
    init_match_count_++;   // 需连续通过 init_match_count_threshold_ (=2) 帧
```

**新增的验收逻辑为零**，NDT 精配准和收敛判据全部沿用。这把本方案的风险面压到最小：Tier 1 猜错了，就是多花一帧 NDT 然后被拒，行为回到现状。

多假设的处理：按描述子距离从优到劣依次尝试，每个种子给一帧 NDT 验证机会，全部失败则落回 Tier 2。

---

## 4. 改动清单（已实施）

| 文件 | 改动 |
| --- | --- |
| `include/localization/scan_context_db.hpp` | **新增**。`ScanContextParams` / `ScanContextCandidate` / `ScanContextDatabase`。刻意不含 `rclcpp`，离线校验工具可直接链接 |
| `src/localization/scan_context_db.cpp` | **新增**。CSV / PCD / index.json 载入、世界系→雷达系还原、max-height 描述子、ring key、60 次移位检索、种子位姿合成。参数与 `map_loop_closure.py:19-24` 对齐（RINGS 20 / SECTORS 60 / MAX_RADIUS_M 80 / 空格填 0.0） |
| `tools/scan_context_check.cpp` | **新增**。§6.1 的离线留一法评估工具，不依赖 ROS 图、地图服务和机器人 |
| `apps/localization_nodelet.cpp` — `loadScanContextForMap()` + `LoadMapCallBack` | 载入地图后建库。沿用 `loadGnssOriginForMap` 的 `parent_path()` 先例定位 `keyframes/`。载入失败只记 warning |
| `apps/localization_nodelet.cpp` — `Reset()` | 换图 / 冷启动时游标归零，底库随 `LoadMapCallBack` 重建 |
| `apps/localization_nodelet.cpp` — `tryScanContextRelocalization()` | Tier 1 主体：检索 → 取游标处候选 → 交 ICP 验证 → 游标顺延 |
| `apps/localization_nodelet.cpp` — 重定位闸门 | 在 `seedPositionFromGnss` 之后、`performGlobalLocalization` 之前插入 Tier 1；失败则原样回落 Tier 2 |
| `apps/localization_nodelet.cpp` — `performGlobalLocalization()` | 新增可选 `const Eigen::Matrix4d* seed_override`，`nullptr` 时行为与改动前逐字节一致 |
| `config/config.yaml` | 新增 `relocalization` 段：`use_scan_context: false`、`top_k: 5`、`max_seeds_per_attempt: 1`、`max_descriptor_distance: 0.0` |
| `CMakeLists.txt` | `scan_context_db.cpp` 加入 `localization_node`；新增 `localization_scan_context_check` 可执行目标 |

不改动：`global_localization.cpp`（Tier 2 保持原样）、`pose_estimator`、UKF、行为树、任何 3588 资产。

### 4.1 与原方案的三处偏离（均有据）

1. **不加 `/localization_info` 遥测字段。** 原文假设该话题带自由文本 JSON，实际 `robots_dog_msgs/msg/Localization.msg` 是固定字段消息（header/type/status/coord_type/pos/rpy/vel/acc/gyro/speed），加字段等于改动 Edge 与平台共用的接口包并触发全部下游重编。这超出本方案范围，未做。§6.4 的实机观察改用 `tryScanContextRelocalization()` 打的 `RCLCPP_INFO`——每次尝试都会打印候选序号、关键帧号、描述子距离、偏航和种子坐标，足够定位现场行为。

2. **不做 `verify_local_map_radius_m` 局部地图裁剪。** 原为"可选优化"。既然每次闸门触发只验 1 个种子，单帧 ICP 次数已与改动前持平，这项优化对延迟预算没有必要，先不引入额外分支。

3. **`max_descriptor_distance` 默认 0（不设限）而非 0.35。** 见 §6.1 的实测：正确与错误检索的描述子距离分布重叠，卡阈值只会误杀。ICP 验证才是有区分力的那道门。

### 4.2 未做：`mapping_adapter.py` 上传清单

§2.2 提出的把 `descriptors.bin` / `ring_keys.bin` 加入上传清单**未实施**。原因是本方案最终选择在 C++ 侧从 `keyframes/scan_*.pcd` 现算描述子，而上传清单同样不含关键帧点云——只补两个 `.bin` 并不能让跨机地图用上 Tier 1，除非同时上传关键帧点云（数十 MB 量级）或改为解析 `.bin`。这是一个独立的取舍，留待跨机地图复用真正成为需求时单独决策。

**结论不变但理由更强：本机建图的地图不受影响，跨机下发的地图 Tier 1 不可用，行为退回 Tier 2。**

---

## 5. 明确的能力边界（不要过度承诺）

Scan-Context 是 **2.5D max-height 描述子**，它的失效模式和 NDT 的失效模式**高度重叠**：

1. **几何简并场所**（长走廊、空旷大厅、对称厂房）——描述子近乎相同，检索给出的 top-K 可能全是错的。而这恰恰是定位最容易丢的地方。**本方案不解决几何简并。**
2. **横滚/俯仰敏感**。描述子在雷达系构建、未做重力对齐；四足步态和地形会引入俯仰扰动，降低可重复性。用 UKF 姿态做重力对齐能改善，但会与已生成的 40 个地图的 v2 描述子不兼容——本方案**保持 v2 兼容，不做重力对齐**，把它列为后续可选项。
3. **6° 偏航分辨率**（`yaw_search_steps: 60`），只能当种子，必须由 NDT 精化。
4. **地图与现场不符**（物体移动、施工）同样使描述子失配。
5. 43 个既有地图两类资产皆无，行为完全不变。

**能解决的**：绑架 / 冷启动位姿未知 / 种子已经漂到 ICP 盆地之外的深度失位——即当前架构下**完全不可恢复**的那一类。这是从 0 到 1，不是从 1 到 1.2。

---

## 6. 验证

### 6.1 离线留一法检索评估（**已完成**）

工具：`localization_scan_context_check`。对每个地图建库，再把每个关键帧当查询、其余当底库。

```bash
cd /home/dogrobot/robot
./install/localization/lib/localization/localization_scan_context_check \
  --min-gap 30 --top-k 5 --radius 2.0 \
  $(ls -d /home/dogrobot/runtime/nx-edge/data/jszr/map/*/keyframes/keyframes.csv \
    | xargs -n1 dirname | xargs -n1 dirname)
```

只读地图目录，不写任何文件，不连 ROS 图，可随时执行。

**关键的方法学修正：分母必须是"可答查询"。** `--min-gap N` 会把查询前后 N 个关键帧一起从底库里拿掉，用来强迫检索去匹配另一趟经过的同一地点——但若该地点本来只走过一次，拿掉邻居后底库里**根本不存在正确答案**，把这种查询记成检索失败会低估方法本身。因此工具先扫一遍：只有当排除之后底库里仍有关键帧落在真值 `--radius` 内时，该查询才计入分母。

`--min-gap 30`、`--top-k 5`、`--radius 2.0`，118 个带 `keyframes.csv` 的地图会话：

| 指标 | 结果 |
| --- | --- |
| 成功建库 | **103 / 118** |
| 可答查询 | 7248 / 24583 |
| **top-1 命中率** | **95.8 %** |
| **top-5 命中率** | **97.9 %** |
| top-1 位置误差中位数 | 0.01 m |
| 偏航误差中位数 / p90 | **0.11° / 3.17°** |

未建库的 15 个会话经逐个核对，全部是 `keyframes.csv` 只有表头、目录下没有任何 `scan_*.pcd` 的**中断建图会话**，不是载入器缺陷。其中 `20260717_171836_872` 用的还是无 `world_*` 列的旧表结构，载入器的回退分支能正确处理，只是同样没有数据行。

单个代表性地图 `20260825_222530_461`（95 关键帧，`loop_status: no_valid_loop`，几乎没有重访）的难度梯度：

| `--min-gap` | 可答 / 总数 | top-1 | top-5 | 偏航误差中位数 / p90 |
| --- | --- | --- | --- | --- |
| 1（只排除自己） | 95 / 95 | 100.0 % | 100.0 % | 0.52° / 1.97° |
| 10 | 76 / 95 | 81.6 % | 96.1 % | 1.32° / 10.80° |
| 30 | 62 / 95 | 80.6 % | 95.2 % | 1.24° / 7.75° |

逐地图分布（51–52 个存在可答查询的会话）：top-1 中位数 97.8 %，top-5 中位数 100 %，top-5 的 p10 为 93.9 %，**只有 2 个会话的 top-5 低于 80 %**。

**三条可直接用于设计的结论：**

1. **`top_k: 5` 是对的。** 单图 min-gap 30 下 top-1 只有 80.6 % 而 top-5 有 95.2 %，正确答案通常在前 5 名里但不总在第 1 名。配合"每次闸门只验 1 个、失败后游标顺延"的设计，5 次重挂（约 25 s）就能把候选走完。
2. **偏航符号已定死。** `yaw_query = yaw_match + yaw_offset`。此前用 21 个地图 8577 对共位关键帧做过一次独立定标：该式误差中位数 3.66°（正好是 6° 扇区量化的一半），取反则是 36.95°。本次留一法给出的 0.11°–1.32° 中位偏航误差是同一结论的第二次独立确认。
3. **描述子距离不能当门限。** 正确检索的距离 p90 为 0.33，错误检索的距离中位数为 0.26，两个分布重叠。因此 `max_descriptor_distance` 默认设为 0（不设限），由 ICP 验证充当唯一的判别门。原方案里 0.35 的建议值会在几乎不拦截错误候选的同时误杀正确候选。

### 6.2 尚未回答：查询点云与底库点云的密度差

底库描述子由关键帧 PCD 现算（实测 `scan_00050.pcd` 仅 3643 点，SLAM 侧已重度降采样），而在线查询用的是 `points_voxel_filter_size: 0.15` 降采样后的实时帧，密度明显更高。max-height 是取每格最大值的统计量，密度更高会使查询侧各格系统性偏高，给所有候选带来一个方向一致的正偏置。

**留一法量不到这一项**——它两侧用的是同一批点云。构造合成扰动只能得到一个弱代理，真正能回答它的是 §6.3 的回放：那里的查询是真实在线帧，底库是真实地图。**在回放跑通之前，本条是本方案唯一未闭环的已知风险。**

失效模式是温和的：密度失配只会降低检索命中率、增加重挂轮次，不会让错误位姿通过——ICP 验证和既有的 2 帧 `init_match_count_threshold` 仍在后面把关。

### 6.3 回放 A/B

用 `robot/script/robot/replay_mapping_rosbag.sh`（隔离 `ROS_DOMAIN_ID=77`）：人为把初始位姿设成偏离真值 30°/5 m，对比改前改后能否收敛、收敛耗时。素材现成。

### 6.4 实机（需另行征得重启同意）

遮挡雷达制造丢失 → 看节点日志里的 `Scan context candidate .../...` 与 `Scan context seed verified against the map`，量恢复耗时；搬动机器人制造绑架 → 现状必然不可恢复，改后应能恢复。（不使用 `/localization_info`，理由见 §4.1.1。）

### 6.5 编译（**已完成**）

```bash
colcon build --packages-select localization    # Finished，无新增 warning
```

该包仍无 CMake 单元测试目标。本轮引入的 `localization_scan_context_check` 是**离线评估工具而不是单元测试**——它需要真实地图数据，不能在 CI 里无条件跑，因此没有挂进 `ament` 测试。这是一个有意的取舍，不是遗漏：在这个包里，一个能对 103 个真实地图量化召回率的工具，比一组自造点云的断言更有价值。

---

## 7. 执行顺序与进度

| # | 步骤 | 状态 |
| --- | --- | --- |
| 1 | §6.1 离线评估（门槛项） | ✅ 通过：top-5 97.9 % |
| 2 | `scan_context_db` + 离线评估工具 | ✅ 已落码编译 |
| 3 | 接入 `LoadMapCallBack` / `Reset` / 重定位闸门 | ✅ 已落码编译，默认关闭 |
| 4 | 打开 `use_scan_context` 跑 §6.3 回放 A/B | ⏸ **待授权拉起回放栈**；同时闭环 §6.2 的密度差 |
| 5 | §6.4 实机 | ⏸ **待授权重启定位栈** |
| 6 | `mapping_adapter.py` 上传清单 | ❌ 不做，理由见 §4.2 |

第 4 步之前，`relocalization.use_scan_context` 保持 `false`，现网行为不变。

---

## 8. 与其他方案的关系

- [IMU 漂移方案](IMU_DRIFT_DIAGNOSIS_AND_REMEDIATION_PLAN.md)：零偏注入减缓种子衰减，与本方案**互补而非替代**。两者都做，Tier 2 的成功率也会提高。
- [定位自愈方案](LOCALIZATION_SELF_HEALING_PLAN.md)：本方案是其 L2「主动重定位」的实际能力来源。自愈负责"何时触发、失败了怎么办、怎么上报"，本方案负责"触发之后靠什么找回来"。
- [SLAM 采集与世界位姿计划](SLAM_DATA_CAPTURE_AND_WORLD_POSE_PLAN.md)：本方案消费其 P3 产出的 Scan-Context 资产，不改建图侧。
- **推算桥接当前是死代码**：`localization_nodelet.cpp:1203` 的 `startBridge()` 首个条件即 `!enable_robot_odometry_prediction`，而 `config.yaml` 中该项为 `false`，`/odom/mc_odom` 也未发布。因此 `bridge_max_distance_m` / `bridge_max_seconds` / `max_yaw_sigma_deg` 这套安全限幅从未生效。这不属于本方案范围，但在评估丢失行为时必须知道——**丢失后是无限幅的纯 IMU 外推，不是受控推算**。已记入自愈方案待办。
