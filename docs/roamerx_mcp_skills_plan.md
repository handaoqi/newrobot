# RoamerX MCP 技能清单与建设方案

## 首期发布技能清单

所有控制均走正式链路：`MCP -> 云平台 API -> MQTT -> Edge Agent -> ROS/SDK`。
用户询问“你有哪些技能”时，MCP 调用 `robot_remote_control_capabilities`，仅返回下列已纳入首期发布包的能力；规划中的能力不会混入此清单。

| 类别 | MCP 工具 | 能力 |
| --- | --- | --- |
| 基础遥控 | `robot_direction` | 前后左右、左右转、速度向量和停止 |
| 速度档位 | `robot_speed` | 微速、低速、中速、高速 |
| 姿态与运控 | `robot_action` | 起立、匍匐、阻尼、启动/停止运控 |
| 组合动作 | `robot_skill_run` / `robot_skill_status` / `robot_skill_cancel` | 执行、查询、取消预设或受限步骤组合 |
| 人员识别 | `robot_person_detection` / `robot_person_detection_status` | 开关人员检测并读取可选 `track_id` |
| 持续人员跟随 | `robot_person_follow` / `robot_person_follow_status` / `robot_person_follow_stop` | 启动、查询和停止指定人员的本地视觉跟随 |
| 技能清单 | `robot_remote_control_capabilities` | 只读返回上述已上线能力与安全提示 |

所有控制均须由用户明确提出对应操作命令，无需现场通道安全的二次确认。组合动作和人员跟随仍不得根据含糊表述猜测预设、步骤或目标。

> 本次完成的是源代码、迁移、文档和自动化验证；未重启机器人服务、未推送云端发布，也未触发实体运动。只有 MCP、平台迁移、Edge Agent 和检测服务作为同一发布包部署完成后，人员跟随相关工具才应对实际用户宣称为可用。

## 首期实现

- MCP 技能注册表是面向用户的唯一已上线清单；所有工具均保留清晰的安全约束与云端审计链路。
- 持续人员跟随不再在 MCP 或云端轮询闭环。`roamerx-bike-bot` 对每帧人员轨迹原子写入 `/run/roamerx/person_detections.json`，Edge Agent 仅读取此本机快照并以 150ms 周期控制速度。
- Edge `PersonFollowController` 一次只允许一个 `track_id` 会话；MCP 可使用明确 `track_id`，或在用户明确要求时由平台从最新人员框中选择最接近画面中心的人员。人员丢失、检测帧过期、前方障碍、停止命令、任务接管、阻尼、趴下或 Edge 关闭时，均先发布零速度后终止跟随。
- 平台增加 `teleop.person_follow_start`、`teleop.person_follow_stop`、`teleop.person_follow_status` 三类 RemoteCommand；MCP 接口通过其创建、审计并等待 Edge 返回结果。
- 跟随速度限制为前进不超过 `0.16m/s`、后退不超过 `0.08m/s`、转向不超过 `0.28rad/s`；前方障碍距离不大于 `0.8m` 时停车。检测快照超过 `1s` 未更新时不可启动或自动终止。

## 基础遥控 Skill 包

- 已创建可安装的个人 Codex 插件 `roamerx-basic-teleop`，包含本机 `http://127.0.0.1:8095/mcp` 的 MCP 配置、基础遥控 Skill 和中文自然语言命令参考。插件不保存平台令牌或机器人 ID。
- Skill 被交互式 Codex 和配置了共享 Codex Home 的 `roamerx-dev-agent` 共同加载；远程开发助手因此遵循与交互式会话一致的 MCP、明确命令与能力问答规则。
- MCP 的 `robot_id` 可以省略。部署人员在 MCP 服务环境设置正数 `ROAMERX_DEFAULT_ROBOT_ID` 即可绑定默认机器狗；显式 `robot_id` 仍可用于多机器人场景并优先于默认值。
- 用户询问技能或预设时先读取 `robot_remote_control_capabilities`，仅返回自然语言指南；不会因问答触发控制。所有实际控制在用户明确请求后直接执行，组合动作和人员跟随仍须提供明确的预设、步骤或目标。
- Dev Agent 的本地转写增加统一唤醒格式“小太阳，<操作命令>”。匹配时移除唤醒名并自动创建 `voice` 会话任务，使共享 Codex Home 中所有匹配的已安装 Skill 都可执行该命令，并为每次接受的命令发布 `dev/voice/wake` 审计事件。

## 后续阶段

1. 高级控制：导航状态/启停/恢复、初始位姿与重定位、充电、传感器、音量和受控组合动作。
2. 问答：机器人状态、告警、任务、地图和技能的只读问答；不得由功能咨询触发运动。
3. AI 开发：接入现有 Dev Agent，提供受审计的开发任务提交、结果查询与只读计划；任何代码执行或写入均需明确确认。
4. 监测、巡检与建图：遥测与告警、巡检任务生命周期、地图开始/保存/取消/激活，均复用已存在的 Edge/平台命令。
5. 循环时序：提供有限次数、固定间隔的已授权技能编排；首版不自动循环高风险运动动作。

## 验证与发布

- 单元测试覆盖协议白名单、MCP API 命令映射、跟随启动/停止、检测过期和障碍停车。
- 自动化验证不触发实体运动。部署前先验证云端 RemoteCommand 的创建、MQTT ACK/结果和 Edge 状态。
- 真机跟随验收使用云端正式路径执行；开始前需给出明确的人员目标，但不要求现场安全二次确认。
- 部署时启用并健康检查 `roamerx-bike-bot.service`；检测服务未运行或本地快照不可用时，人员跟随应拒绝启动而非降级到云端控制。
