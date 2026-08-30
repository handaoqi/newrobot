# 导航容器 SIGSEGV / core 调试推进计划

## 目标与当前事实

目标是将“航点 3 成功后 `navigo_container` 崩溃”定位到具体组件、源码文件和线程，并完成修复验证。

已确认：

- 航点 1、2、3 均曾报告成功；
- 点 3 完成后 `component_container_isolated` 收到 `SIGSEGV`，退出码 `-6`；
- 日志出现 `Magick: abort due to signal 11`；
- 当前没有可用 core；
- 定位节点仍保持 `status=3`，不是定位节点先崩溃；
- 已加入 `MAP_DEBUG`、`WAYPOINT_DEBUG` 日志；
- 已加入 core 配置和 Debug 构建；
- `navigo_map_server` 已改为 GraphicsMagick 进程内只初始化一次，但该修复尚未通过复现验证。

## 阶段一：固定调试运行环境

1. 保留 `LimitCORE=infinity`、`ulimit -c unlimited` 和固定 core 路径 `/tmp/roamerx-core/core.<程序>.<pid>.<时间>`。
2. 对 core 目录设置容量监控：单个 core 最大 2 GiB，总目录超过 6 GiB 时停止复现并转存旧 core。
3. 导航相关包使用 `Debug` 或 `RelWithDebInfo` 构建，并确认运行时加载的是 overlay 安装目录而不是 `/opt/ros` 同名二进制。

## 阶段二：补齐崩溃前后日志

### Map Server

记录 configure、YAML 加载、GraphicsMagick 初始化、图片尺寸、activate/deactivate/cleanup/shutdown，以及 `msg_` 清理前后的状态。

### Waypoint Follower

记录 action goal 接收、航点序号和坐标、controller goal、航点任务回调、最后一个航点收尾、`succeeded_current()` 调用前后和 action handle 清理。

### Controller / Optimizer / Costmap

记录 optimizer reset 来源、controller 完成后的资源释放、costmap/plugin cleanup、collision monitor/velocity optimizer shutdown，以及 component container 的退出边界。

## 阶段三：安全复现

1. 先取消云端残留任务，确认数据库不再是 `running`。
2. 停止残留导航 launch，确保只有一个 `component_container_isolated`。
3. 确认机器人静止、急停可用且速度指令为零。
4. 使用 Debug 二进制启动导航栈，先确认所有 lifecycle 节点为 `active`，并确认地图只加载一次。
5. 使用地图 149、路线 92 复现一次 1→2→3：点 1 `(0.3081, 0.0612)`、点 2 `(5.0082, -0.4888)`、点 3 `(9.3082, -0.4888)`。
6. 每次只执行一次路线，保存 navigation/ROS/edge-agent 日志、动作状态、速度指令和 core 文件。

## 阶段四：core 分析

```bash
file /tmp/roamerx-core/core.*
gdb /opt/ros/humble/lib/rclcpp_components/component_container_isolated \
    /tmp/roamerx-core/core.*
```

在 gdb 中执行：

```gdb
set pagination off
thread apply all bt full
info registers
info sharedlibrary
thread info
```

若栈落在 map server/GraphicsMagick，检查图片对象、全局初始化和 cleanup；若落在 waypoint follower，检查最后航点 action 生命周期；若落在 controller/optimizer、costmap 或 collision monitor，检查 reset、插件卸载和线程退出顺序。若只有 libc/libstdc++，使用 ASan/UBSan Debug 构建复现。

## 阶段五：针对性修复

按 core 栈只处理一个嫌疑组件：GraphicsMagick 生命周期 → waypoint follower action 收尾 → controller optimizer reset/析构 → costmap/collision monitor unload → component container 多线程退出顺序。

每个修复单独提交、Debug 编译、运行对应测试，并重复一次 1→2→3 验证。

## 阶段六：验收与恢复生产

修复通过条件：连续 5 次 1→2→3 无 SIGSEGV、无新 core、FollowWaypoints 正常完成、Nav2 lifecycle 持续 active、点 3 后无异常 abort、定位保持 `status=3`，且日志能明确显示组件完成和清理顺序。

通过后恢复生产构建，保留低频调试日志和有限额 core 配置，并将根因、修复提交和复现结果补入故障记录。

## 阶段七：Edge Agent 导航栈看门狗与断点恢复

为避免导航容器崩溃后任务长期停留在 `running`，由 Edge Agent 增加受控看门狗：

1. 监测 `component_container_isolated`、`map_server`、`controller_server`、`planner_server`、`bt_navigator`、`waypoint_follower` 进程/节点，以及 `/follow_waypoints` action server 和 lifecycle 状态。
2. 检测异常时先保持/发布零速度，暂停任务并记录任务 ID、地图版本、当前航点、已完成航点和重启次数。
3. 清理残留 launch 后只启动一个导航栈实例，等待所有 lifecycle 节点进入 `active`。
4. 重新确认定位 `status=3`、TF 连通、地图 ID/版本一致后，只恢复当前未完成航点，不重发整条路线。
5. 每个任务自动恢复最多 2 次，采用递增退避；超过次数进入 `NAVIGATION_RECOVERY_FAILED`，等待人工处理。
6. 定位服务异常、地图版本不一致、TF 缺失或速度安全检查失败时禁止自动恢复。
7. 所有恢复开始、健康检查、航点重发、成功/失败和最终停车事件写入任务生命周期日志。

### Debug/Release 构建约束

- 调试阶段固定使用 `Debug` 或 `RelWithDebInfo` 导航二进制；普通增量编译不得将导航包切回 Release。
- 每次启动前记录可执行文件路径、Build-ID、是否 stripped 和调试段存在性，确认运行实例加载的是 Debug overlay。
- 看门狗调试期间不得自动更新到 Release 镜像或清理 Debug 符号。
- 只有 core 根因修复并满足连续 5 次 1→2→3 验收后，才执行一次显式 Release 构建、部署和回归；Release 恢复必须单独提交并保留 Debug 构建产物用于复盘。

## 明确不做

- 不删除地图、任务数据库或 MQTT 凭据；
- 不在残留 `running` 任务上叠加第二个导航实例；
- 不把 `Magick` 日志直接当作根因；
- 没有 core 栈或可重复证据时不宣布最终修复；
- 不修改与本次崩溃无关的并行任务文件。
