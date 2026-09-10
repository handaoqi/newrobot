# 四种导航算法组合 — 现场读回检查表

适用设备：本 NX（Nav2 已注册 `ThetaStar/NavFn` + `FollowPath/RPP`）。

## 一键脚本（推荐）

导航栈在跑、且无巡检任务时：

```bash
sudo bash /home/dogrobot/scripts/verify_navigation_algorithm_combos.sh
```

脚本会：核对插件已注册 → 四种组合写参并读回 `use_astar` → 发布 `/planner_selector` 与 `/controller_selector` → 确认硬停仍开启 → 恢复常用 `ThetaStar+RPP`。

## 人工巡检读回（每组合跑一条短路线后）

在路径规划页改全局/局部后发车，然后：

```bash
sudo bash -lc 'source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=24
ros2 topic echo /planner_selector --once
ros2 topic echo /controller_selector --once
ros2 param get /planner_server NavFn.use_astar
'
```

| 页面全局 | 页面局部 | 期望 planner_selector | 期望 controller_selector | 期望 NavFn.use_astar |
| --- | --- | --- | --- | --- |
| theta_star | mppi | ThetaStar | FollowPath | （无关） |
| theta_star | rpp | ThetaStar | RPP | （无关） |
| navfn | mppi | NavFn | FollowPath | True |
| navfn | rpp | NavFn | RPP | True |

Edge 侧成功时日志应出现：`navigation profile applied generation=... global=... local=...`。
