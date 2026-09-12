# 路径规划、定位与导航直线防摆实施计划

## 1. 界面与调试包

- “仿真和回放检查”“回放调试台”归入“巡检任务”子菜单，保留原 URL。
- 路线选择下方提示缩小字号、禁止换行；“上次执行时间”内容和格式保持不变。
- 保存路线按钮上方增加“录制导航调试包”，默认关闭并保存到路线 `record_rosbag`。
- 路线执行未显式传参时继承路线配置，单次执行参数可覆盖；巡检任务模板原有开关保持独立。

## 2. 室外/过渡场景定位

下发地图、初始化定位、主动重定位统一优先 RTK fixed。连续 3 个新鲜样本满足 fixed、双天线航向有效且 RTK/FAST-LIO 偏差小于 0.30 m 后立即完成；限定时间内失败则有界回退到建图原点、周边、手选点/路线航点和全局匹配，不无限等待。室内和 `local_only` 保持 NDT 渐进定位。

## 3. MPPI 参数与差异

当前为 `time_steps=56`、`model_dt=0.05`、`batch_size=1000`、`motion_model="DiffDrive"`。调整为：

```yaml
motion_model: "DiffDrive"
time_steps: 40
model_dt: 0.05
batch_size: 1200
```

预测时域由 2.8 s 缩短为 2.0 s，采样轨迹数量增加 20%，总时域样本由 56000 降至 48000，预计降低内存和总体计算量，同时提高单周期采样多样性。`DiffDrive` 保持原大小写和值不变。

直线 Profile：`wz_std=0.04`、`gamma=0.03`、`wz_max=0.28`、PathFollow 权重 14、PathAngle 权重 3、最大角度 0.30，直线无遮挡时关闭 PathAlignCritic。

## 4. RPP 参数与实现

普通直线巡航使用：

```yaml
desired_linear_vel: 0.22
lookahead_dist: 1.2
min_lookahead_dist: 0.6
max_lookahead_dist: 1.8
use_velocity_scaled_lookahead_dist: true
lookahead_time: 2.5
use_regulated_linear_velocity_scaling: true
max_angular_vel: 0.30
rotate_to_heading_threshold: 0.52
rotate_to_heading_angular_vel: 0.22
angular_deadband: 0.03
```

自研 RPP 增加上述参数声明和动态更新。前视距离按 `clamp(max(lookahead_dist, abs(vx)*lookahead_time), min, max)` 计算；曲率调速映射现有调速逻辑。终点接近和精确航向阶段继续使用短前视低速参数。

## 5. 共享速度平滑与验收

- 角速度死区 `0.03 rad/s`。
- 角速度最大加速度/减速度调整为 `0.8/-0.8 rad/s²`。
- 测试长直线、轻微折线、转弯、直线遇障碍和终点航向，记录 `wz` 正负切换、峰值、横向误差、到点率、避障干预和 MPPI 周期性能。
- 验收目标：直线段 `wz` 正负切换下降至少 50%，横向误差不超过 0.15 m，到点率、避障和安全停车能力不下降。
