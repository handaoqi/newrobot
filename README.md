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
- `docs/robot-payload-standard.md`：机器人上报协议说明
- `platform-demo.html`：参考 demo 页面

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

## 演示账号

- 用户名：`operator`
- 密码：`admin123456`

首次登录时，后端会自动补充演示用户与基础演示数据。
