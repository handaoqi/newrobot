# 智能机器人巡检监测平台

基于 `Django + Vue` 的前后端分离演示工程，包含：

- 登录页面
- 实时监测中心
- 事件中心
- 机器人管理
- 巡检任务页面
- 机器人板端上报数据标准文档

## 目录结构

- `backend/`：Django 后端
- `frontend/`：Vue 前端
- `bot-version/`：机器人板端识别、上报与推流程序
- `docs/robot-payload-standard.md`：机器人上报协议说明
- `docs/project-docs-index.md`：项目详细文档入口
- `platform-demo.html`：参考 demo 页面

## 项目文档

详细文档请从 [docs/project-docs-index.md](docs/project-docs-index.md) 开始阅读，包含项目整体架构、环境配置、前后端接口、数据传输格式、数据存储、网站功能、板端识别与视频链路、运维排障等内容。

## 后端启动

```powershell
cd backend
..\.venv\Scripts\python manage.py migrate
..\.venv\Scripts\python manage.py runserver
```

默认接口地址：`http://127.0.0.1:8000/api/`

## 前端启动

```powershell
cd frontend
npm install
npm run dev
```

默认访问地址：`http://127.0.0.1:5173`

## 云端部署

从机器人 NX 本机部署当前平台版本：

```bash
scripts/deploy_cloud_platform.sh
```

仅更新页面可用 `--frontend-only`；仅更新 Django 后端可用 `--backend-only`。脚本在本机构建 Vue，再同步到云端并检查服务状态；不会重启机器狗端 Edge Agent。

## 演示账号

- 用户名：`operator`
- 密码：`admin123456`

首次登录时，后端会自动补充演示用户与基础演示数据。

## 模拟告警事件

后端和前端启动后，可用脚本模拟机器狗上报告警，用于验证前端 SSE toast 与“历史事件识别”实时更新：

```powershell
python scripts/simulate_robot_event.py
```

如需同时上传抓拍图：

```powershell
python scripts/simulate_robot_event.py --image 违规1.jpg
```
