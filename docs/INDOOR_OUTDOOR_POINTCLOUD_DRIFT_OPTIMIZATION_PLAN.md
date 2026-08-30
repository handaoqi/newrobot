# 室内外点云与漂移参数场景化优化计划

## 摘要

基于地图 `scene_scope` 建立 `indoor / transition / outdoor` 三套定位参数。室内保持现有精度基线；室外扩大裁剪范围、适度粗化体素；VGICP 继续作为条件式精化器。将共享漂移门限拆分为 NDT/VGICP 与 RTK 两类，避免放宽室外 NDT 时同步降低固定 RTK 精度。

## 实现变更

- 启动导航时读取活动地图 `map_manifest.json.scene_scope`，同时传给 FAST-LIO 和 localization；非法或缺失值回退为 `indoor`。
- localization 增加场景 profile 参数，启动时选择一次，不在运行中热切换：
  - `indoor`：扫描 10 m/240°，地图与扫描体素 0.15 m，NDT 0.15 m，VGICP 0.50 m，局部地图 18×4 m。
  - `transition`：扫描 20 m/240°，地图与扫描体素 0.20 m，NDT 0.25 m，VGICP 0.60 m，局部地图 25×6 m。
  - `outdoor`：扫描 25 m/240°，地图体素 0.25 m、扫描体素 0.20 m，NDT 0.30 m，VGICP 0.75 m，局部地图 30×8 m。
- FAST-LIO 建图体素按 profile 设置为：
  - 室内 0.15/0.15 m；
  - 过渡区 0.20/0.20 m；
  - 室外 0.20/0.25 m（surface/map）。
- FastGICP 短期里程计体素使用室内 0.40 m、过渡区/室外 0.50 m。
- 保持 VGICP 条件式运行：初始化、恢复或 NDT 质量不足时才执行；各 profile 分别统计健康 NDT 分数后再调整 `refine_skip_ndt_score`，首版室内继续使用 0.30，过渡区和室外暂不基于未经验证的固定分数跳过。
- 将漂移策略接口由一套共享门限改为按校正源选择：
  - 室内 NDT/VGICP：0.30 m/5°，rearm 0.20 m，稳定容差 0.10 m/2°，3 帧。
  - 过渡区 NDT/VGICP：0.40 m/6°，rearm 0.25 m，稳定容差 0.15 m/2.5°，3 帧。
  - 室外 NDT/VGICP 回退：0.45 m/7°，rearm 0.30 m，稳定容差 0.20 m/3°，4 帧。
  - 室外固定 RTK：0.30 m/5°，rearm 0.20 m，稳定容差 0.10 m/2°，3 帧。
- 校正平滑速率使用室内 0.25 m/s、8°/s，过渡区 0.30 m/s、8°/s，室外 0.40 m/s、10°/s；最大校正跳变仍保持 1.50 m/30°，超过门限进入重定位，不直接注入 UKF。
- 保持定位 NDT/VGICP 为 240°以匹配建图点云；导航 FAST-LIO 继续使用 360°在线点云。

## 接口与兼容性

- localization 启动入口新增 `scene_scope` 参数，枚举为 `indoor|transition|outdoor`。
- 配置增加 `profiles.<scene_scope>` 和 `lio_primary.correction_sources.{scan_match,rtk}`；未提供 profile 时完整回退到现有室内参数，兼容旧地图和旧启动命令。
- 状态日志增加当前 profile、有效裁剪/体素参数、校正源及该源实际漂移门限，便于回放验收。
- 不改变地图格式；继续使用现有 manifest 的 `scene_scope` 字段。

## 测试与验收

- 单元测试覆盖 profile 选择、缺失/非法场景回退、NDT 与 RTK 门限隔离、rearm 和连续帧逻辑。
- 启动测试验证室内/过渡区/室外分别向 FAST-LIO 与 localization 传入 10/20/25 m 范围，且 NDT 保持 240°、FAST-LIO 保持 360°。
- 室内回放要求定位成功率和轨迹误差不低于当前基线，NDT 延迟无明显回退，稳定阶段 VGICP 仍主要被跳过。
- 室外回放分别覆盖固定 RTK、RTK 短时丢失、恢复固定、稀疏道路和植被场景；确认 NDT 回退不会采用 RTK 门限，RTK 恢复不会等待 0.45 m 漂移。
- 统计每个 profile 的 NDT/VGICP fitness、重叠率、校正残差、误触发次数和 P50/P95/P99 耗时；仅在至少一条独立验证路线通过后调整 fitness 阈值。
- 实机仅进行低速、有人监护的验证；先影子记录候选校正，不直接改变导航输出，回放通过后再启用。

## 假设

- 当前室内参数和地图作为精度基线，不重新生成现有室内地图。
- 过渡区采用室外建图流程，但使用独立的中间尺度定位 profile。
- 首轮优化目标为精度与 NX 算力平衡，不把扫描距离扩展到 25 m 以上。
- 大尺度室外路线若全局 NDT 目标增长导致延迟超限，后续采用分块局部目标地图，不继续通过增大体素解决。
