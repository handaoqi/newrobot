# 导航任务 7–10 实施与完成跟踪

- 计划建立日期：2026-09-12
- 当前范围：只实施任务 7、8、9、10
- 明确不在本次范围：任务 6（MPPI/RPP `wz` 振荡观察与调参）
- 当前阶段：软件开发、构建与回归验证完成；低速实机验收待部署后执行

## 完成状态

| 任务 | 实施内容 | 状态 | 验收证据 |
| --- | --- | --- | --- |
| 7 | 自研 NaviGo ABI 的 Smac Hybrid A* 全局规划器、iLQR 局部控制器；保留 ThetaStar+MPPI 默认并支持路线/航点选择 | 开发完成（实机待验） | 插件包编译、导出符号、Edge 映射/协议、后端与前端构建通过 |
| 8 | SmootherServer 注册 Savitzky-Golay 平滑器；默认启用，失败时依次回退 Simple 与原路径 | 开发完成（实机待验） | Smoother/BT Navigator 编译、BT XML lint 和选择链路测试通过 |
| 9 | 卡死进度检查恢复；BT 调用 BehaviorServer 清图、旋转、后退、左右横移；恢复动作受碰撞检查约束 | 开发完成（实机待验） | ProgressChecker、BehaviorTree、BehaviorServer 编译；左右横移树结构测试通过 |
| 10 | Collision Monitor 保持独立速度链路；补充左右横移停止区，并按速度方向启用安全区 | 开发完成（实机待验） | Collision Monitor/robot_navigo 编译；独立输入输出与方向安全区配置测试通过 |

## 实施约束

1. 当前默认算法保持 `ThetaStar + MPPI`，新插件仅在显式选择时使用。
2. 本次不修改 MPPI/RPP 的 `wz` 抑振参数，避免与任务 6 混合。
3. 不在开发机执行真实运动命令；横移与碰撞监控需部署后做低速实机验收。
4. 构建或测试失败时不得把对应任务标记为完成，需在本表记录阻塞原因。

## 验收清单

- [x] 三个全局规划插件和三个局部控制插件均已注册、映射并可选择。
- [x] Savitzky-Golay 插件已注册，默认 BT 在 FollowPath 前执行平滑，并具有安全回退。
- [x] 进度检查器可将卡死转为控制失败，BT 已包含后退/左右横移恢复分支。
- [x] 横移恢复的 `vy` 软件链路已贯通 BehaviorServer、速度平滑器和 Collision Monitor。
- [x] Collision Monitor 继续作为独立节点输出最终 `/cmd_vel`，左右安全区仅匹配相应方向。
- [x] ROS 定向构建、Edge 测试、后端测试和前端构建通过。
- [ ] 部署后完成低速实机的规划、跟踪、横移恢复和四向碰撞停止验收。

## 完成记录

- ROS：9 个相关包构建通过；BT Navigator 的 6 个 lint/XML 测试目标通过；robot_navigo 的 2 个 CTest 目标、9 个 GTest 用例通过。
- Edge：229 个定向 pytest 用例通过，覆盖插件映射、协议、任务状态机、控制器参数及边界限速。
- 后端：54 个 Django 协议、路线序列化与单点导航用例通过。
- 前端：`npm run build` 通过。
- 配置：YAML/XML 可解析，C++ 变更通过 `ament_uncrustify`，`git diff --check` 通过。
- 剩余项：未部署，未向机器人下发运动；需要部署后在安全场地低速完成实机验收。
