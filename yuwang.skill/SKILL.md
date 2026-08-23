---
name: yuwang
description: 在 /home/dogrobot/DogRobot Monorepo 中进行代码修改、测试、评审和 Git 操作时，遵循模块边界、真实机器人安全约束、Vue 前端规范和可回滚的多人协作流程。
metadata:
  short-description: DogRobot 项目开发与 Git 安全规范
---

# Yuwang 开发指导

本技能适用于 `/home/dogrobot` 项目中的开发、重构、调试、测试、提交和代码评审。它是项目约束，不替代用户对具体任务的授权；没有明确要求时，不部署、不连接生产服务、不触发机器人运动。

## 开始工作前

1. 先阅读根目录 `AGENT.md`、目标模块的 README，以及 `robot/AGENTS.md`（涉及机器人、边缘代理、部署或运行时则必须读）。
2. 在仓库根目录执行 `git status --short --branch`、`git branch -vv`，确认当前分支、未提交修改和远端同步状态。
3. `/home/dogrobot` 同时是用户主目录。不要使用 `git add .` 或 `git add -A`；只按目标文件显式暂存。忽略并保护 `.ssh`、`.codex`、`.npm`、`.bash_history` 等主目录文件。
4. 先定位模块边界、调用方、配置来源、接口定义和现有测试，再修改代码。避免无关重排、全局格式化和跨模块顺手重构。

## 模块边界

- `robot/`：ROS 2 Humble 的消息、定位、建图、导航和真实机器人脚本。
- `edge-agent/`：MQTT、ROS 适配、任务执行、设备控制和运行时代理。
- `dev-agent/`：远程开发代理及其本地语音/会话能力。
- `platform/backend/`：Django API、模型、迁移、worker、scheduler 和持久化。
- `platform/frontend/`：Vue 操作界面、路由、API service 和页面状态。
- `deploy/`：稳定部署入口；`runtime/`：配置模板、安装清单、说明和运行时布局。

真实配置、密钥、数据库、地图、rosbag、日志、模型缓存和 Codex 会话属于运行数据，不应提交。代码必须继续使用 `/home/dogrobot` 源码路径和 `runtime/{platform,nx-edge,3588-motion}` 运行时边界，不要恢复旧的 `/home/robot` 或已废弃入口。

## 前端代码位置与规范

网页前端位于 `/home/dogrobot/platform/frontend`，技术栈是 Vue 3 + Vite。优化页面前先按以下边界定位代码：

- `src/views/`：路由页面和页面级状态；路由映射、页面标题与鉴权在 `src/router/index.js`。
- `src/components/`：可复用的展示或交互组件；跨页面的 UI 不要复制粘贴到多个 view。
- `src/services/api.js`：统一 HTTP 请求、鉴权头、401 会话失效处理和 API 函数；页面不要自行拼接另一套 fetch 封装。
- `src/services/*State.js`：可独立验证的纯状态计算、归一化和分页逻辑；复杂且无副作用的逻辑优先抽到这里。
- `src/composables/`：跨页面复用的 Vue 状态和行为，例如主题、Toast。
- `src/style.css`：全局 CSS 变量、主题和公共布局；组件私有样式优先使用 Vue `<style scoped>`。
- `public/`：页面运行时需要的图片、音频等静态资源；不要提交构建产物 `dist/` 或依赖目录 `node_modules/`。

线上页面 `https://39.107.250.69/dashboard/` 与本地前端路由按下表对应。线上地址是验收参照，不是代码目录；修改页面时使用表中的本地文件，联调时再打开对应线上路径核对视觉和交互。

| 线上路径 | 页面/用途 | 本地入口 |
| --- | --- | --- |
| `/` | 重定向到实时监测中心 | `src/router/index.js` → `/dashboard/overview` |
| `/login` | 登录页 | `src/views/LoginPage.vue` |
| `/dashboard/` | 工作台外壳（无默认子页内容） | `src/views/DashboardLayout.vue` + `src/router/index.js` |
| `/dashboard/overview` | 实时监测中心 | `src/views/DashboardOverview.vue`；视频复用 `src/components/LiveVideoPlayer.vue`，提示复用 `src/components/AppToast.vue` |
| `/dashboard/guard-duty` | 保安值守 | `src/views/GuardDutyPage.vue` |
| `/dashboard/remote-control` | 远程控制 | `src/views/RemoteControlPage.vue` |
| `/dashboard/remote-development` | 远程 AI 开发 | `src/views/RemoteDevelopmentPage.vue` |
| `/dashboard/analytics` | 统计分析中心 | `src/views/AnalyticsPage.vue` + `src/components/TrendLineChart.vue` |
| `/dashboard/events` | 事件中心 | `src/views/EventsPage.vue` |
| `/dashboard/robots` | 机器人管理 | `src/views/RobotsPage.vue` |
| `/dashboard/tasks` | 巡检任务列表 | `src/views/TasksPage.vue` |
| `/dashboard/tasks/calendar` | 巡检日历 | `src/views/PatrolCalendarPage.vue` |
| `/dashboard/task-executions/:executionId` | 任务执行详情 | `src/views/TaskExecutionPage.vue` |
| `/dashboard/tasks/maps` | 地图管理 | `src/views/MapsPage.vue` + `src/services/taskMapState.js` |
| `/dashboard/tasks/routes` | 路径规划 | `src/views/RoutePlannerPage.vue` + `src/services/routePlannerState.js` |
| `/dashboard/tasks/zones` | 禁区管理 | `src/views/ZoneManagerPage.vue` |
| `/dashboard/tasks/tracks` | 轨迹回放 | `src/views/TrackPlaybackPage.vue` |

所有 `/dashboard/*` 页面共同使用 `src/views/DashboardLayout.vue` 的侧栏、顶部栏、主题切换、用户信息和退出登录逻辑。线上菜单文字和路由若与本表不一致，先核对远端构建版本、`src/router/index.js` 与 `platform/docs/website-features.md`，不要直接通过复制线上产物修复本地代码。

编写和优化时遵循当前工程风格：

- 使用 Vue 3 `<script setup>`、ES Module、两空格缩进和无分号风格；不要在没有项目需求时引入 TypeScript、CSS-in-JS、Prettier 或新的 lint 规则。
- 页面级数据加载、loading、空数据和错误状态留在对应 view；重复 UI、媒体播放器和提示能力抽成 component 或 composable。
- API 调用统一经过 `services/api.js`，保留现有鉴权、错误对象和 401 跳转行为；修改 HTTP 接口时同步核对后端实现、文档和测试。
- 颜色、主题、背景和通用尺寸优先复用 `style.css` 的 CSS 变量；避免把页面专属选择器继续堆入全局样式。
- 页面改动至少检查浅色/深色主题、宽屏/窄屏布局、长文本、加载中、空状态、失败状态和按钮禁用状态。
- `setInterval`、DOM/window 监听、`EventSource`、媒体流和播放器实例必须在 `onBeforeUnmount` 中清理；涉及实时视频、SSE、录音或轮询时同时检查重复连接和失败恢复。
- 远程控制、导航、接管、急停和机器人命令属于真实设备副作用。UI 优化不得改变命令 payload、状态机、互斥关系、安全停止或超时语义；没有明确授权不得用真实设备做验证。

## 前端优化工作流

1. 先阅读 `platform/docs/website-features.md`、相关 API 文档，并从 `router/index.js` 找到目标页面，再追踪其组件、service、状态逻辑和资源。
2. 先做单页面、单行为的最小修改；不要在同一变更中混入无关格式化、重命名、全局样式重排或跨模块重构。
3. 若出现重复逻辑，优先提取到已有的 component/composable/state service，并为纯函数补充 `node:test` 测试；不要为一次性模板抽象出难以复用的层。
4. 修改后在 `/home/dogrobot/platform/frontend` 执行 `npm test` 和 `npm run build`，在仓库根目录执行 `git diff --check`。没有后端、媒体流或浏览器环境时，明确记录未执行的手工验收及原因。
5. 手工验收至少覆盖受影响路由、主要交互、主题切换和响应式断点；若改动实时功能，还要检查卸载页面后定时器、监听器、SSE 和播放器是否停止。

## 编码要求

- 保持所在文件的既有风格；Python 使用四空格和清晰类型注解，C++ 遵循所在 ROS 包风格，Shell 使用安全的严格模式并引用变量，Vue/JS 保持现有组件和缩进风格。
- 公共接口、协议、状态机和外部副作用要有明确错误处理；SSH、ROS、MQTT、文件系统和网络调用应可注入或 mock，单元测试不能依赖真实设备。
- MQTT 协议、ROS 消息、HTTP API、数据库模型和配置键都是跨模块接口。修改时同步生产方、消费方、兼容处理、文档和测试，特别检查 cloud → MQTT → edge-agent → ROS/Nav2 链路。
- 不要在同一变更中混入无关格式化、重命名或大规模重构；功能、重构、文档和生成/资源更新分开提交。
- 未经用户明确授权，绝不发送运动指令、重启真实服务、修改生产数据库或覆盖设备配置。

## 测试与验证

按受影响范围执行最小充分验证：

```bash
# 通用
git diff --check

# Python agents
cd /home/dogrobot/dev-agent && python3 -m pytest -q
cd /home/dogrobot/edge-agent && python3 -m pytest -q

# MCP
cd /home/dogrobot/mcp_server && python3 -m pytest -q

# Django backend
cd /home/dogrobot/platform/backend && python3 manage.py test monitoring

# Vue frontend
cd /home/dogrobot/platform/frontend && npm test && npm run build

# ROS workspace（涉及 robot 源码、消息、launch 或配置时）
cd /home/dogrobot/robot && ./build.sh all
```

测试失败时区分“本次新增失败”和“已有环境/基线失败”，不得通过删测试、关闭插件、访问真实设备或掩盖异常来让结果变绿。修改服务、导航、建图或协议后，还要验证受影响的启动入口、状态转换、超时、重复消息和失败恢复路径；物理验收需单独确认场地安全。

## Git 协作流程

多人协作时从同步后的 `main` 创建单目标分支：

```bash
git switch main
git pull --ff-only
git switch -c feat/<module>-<topic>
```

分支前缀使用 `feat/`、`fix/`、`refactor/`、`test/`、`docs/`。不要直接提交 `main`，不要强推共享分支。提交信息沿用仓库约定：

```text
feat(edge): validate map activation metadata
fix(platform): preserve task state after reconnect
test(robot): cover localization recovery
docs(deploy): clarify NX rollback procedure
```

提交前只暂存明确路径，并检查：

```bash
git add <明确的文件路径>
git diff --cached --name-status
git diff --cached
git diff --cached --check
git status --short
```

前端单目标分支可使用 `feat/frontend-<topic>`、`fix/frontend-<topic>` 或 `refactor/frontend-<topic>`；提交信息沿用 Conventional Commit 风格，例如：

```text
feat(frontend): improve dashboard responsive layout
fix(frontend): release live stream on route change
refactor(frontend): extract route planner state helpers
```

网页变更提交前特别确认只包含目标页面、组件、service、测试或文档文件；不要使用 `git add .` 或 `git add -A`，不要把 `.env`、`dist/`、`node_modules/`、媒体运行数据或主目录隐藏文件带入提交。

PR 描述必须包含：影响模块、接口/迁移变化、执行过的测试、未执行测试及原因、部署或回滚注意事项。默认通过评审后合并到 `main`；不把运行数据、密钥、构建产物或主目录隐藏文件带入提交。

## 交付判断

只有在目标模块测试通过、跨模块接口已检查、工作区没有意外文件、差异范围与任务一致时，才报告完成。若缺少依赖、设备、权限或真实环境，应明确报告阻塞点和已完成的静态/单元验证，不擅自扩大权限或改变测试目标。

后续用户补充规则时，优先更新本文件对应章节；新规则必须说明适用范围，避免把某一次故障或单个模块特例扩展成全项目约束。
