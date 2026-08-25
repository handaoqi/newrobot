# Vue 3 + Vite

This template should help get you started developing with Vue 3 in Vite. The template uses Vue 3 `<script setup>` SFCs, check out the [script setup docs](https://v3.vuejs.org/api/sfc-script-setup.html#sfc-script-setup) to learn more.

## Recommended IDE Setup

- [VS Code](https://code.visualstudio.com/) + [Volar](https://marketplace.visualstudio.com/items?itemName=Vue.volar) (and disable Vetur) + [TypeScript Vue Plugin (Volar)](https://marketplace.visualstudio.com/items?itemName=Vue.vscode-typescript-vue-plugin).

## 平板响应式视觉检查

项目在 `641–1024px` 的竖屏视口，以及 `1200–2048px` 宽、`900–1280px` 高的横屏视口使用平板布局。顶部应用栏使用圆形菜单按钮配合抽屉导航，主体内容取消桌面整体缩放并按可用宽度重排，主要按钮、菜单和表单控件保持至少 `44px` 触控尺寸。`2000×1200` 是横屏大平板的固定验收规格；桌面布局与手机断点保持独立，适配不依赖浏览器是否报告 `pointer: coarse`。

平板菜单按钮点击后展开左侧导航抽屉，支持遮罩点击、Esc、路由跳转关闭和焦点恢复；当前路由所属的巡检任务子菜单会自动展开。该变化只影响显示和交互层，不改变路由、鉴权、API 或机器人命令。

这套检查使用 Playwright 的浏览器视口模拟，不需要连接实体平板。首次运行先安装 Chromium 和 WebKit：

```sh
npx playwright install chromium webkit
```

以下命令会自动启动 Vite，并使用固定模拟数据检查登录、监测中心、保安值守和远程控制页面：

```sh
npm run tablet:capture       # 生成供 AI 检查的全页截图和布局报告
npm run test:tablet          # 执行布局断言和视觉基线对比
npm run test:tablet:update   # 确认视觉修改后更新基线
```

测试矩阵包括：

- `768×1024` WebKit：iPad 类竖屏视口；
- `800×1280` Chromium：Android 平板类竖屏视口；
- `834×1194` WebKit：11 英寸 iPad 类竖屏视口；
- `2000×1200` Chromium：横屏大平板视口。

`npm run tablet:capture` 会生成每个页面的首屏截图、全页截图和布局 JSON 报告，供 AI 检查留白、遮挡、断行及组件层级。报告还会检查页面是否出现非预期横向滚动，以及主要交互控件是否小于 `44×44px`。

`npm run test:tablet` 会执行同样的布局断言，并将首屏截图与已审核基线比较。只有确认修改后的截图符合设计后，才运行 `npm run test:tablet:update` 更新 `tests/tablet/__screenshots__/` 中的基线；不要用更新基线来掩盖布局断言失败。

本地运行产物位于 `test-results/tablet/`，HTML 报告位于 `playwright-report/tablet/`。GitHub Actions 工作流 [frontend-tablet-visual.yml](../../.github/workflows/frontend-tablet-visual.yml) 会在前端变更时执行单测、构建和三档平板视觉检查，并上传失败截图与报告。
