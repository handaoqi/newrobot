# 智能机器人巡检监测平台文档中心

本文档中心面向当前仓库代码整理，覆盖平台架构、环境启动、接口协议、数据存储、前端功能、板端识别与视频链路、运维排障等内容。

## 文档导航

| 文档                                                 | 内容                                                           |
| ---------------------------------------------------- | -------------------------------------------------------------- |
| [项目整体架构](./project-architecture.md)               | 系统模块、调用链路、目录职责、核心运行流程                     |
| [环境配置与启动](./environment-setup.md)                | Python、Node、Django、Vue、ZLMediaKit、板端程序启动方式        |
| [前后端接口与数据传输格式](./api-and-payloads.md)       | REST API、鉴权、请求参数、响应结构、板端 JSON 与媒体上传       |
| [数据存储说明](./data-storage.md)                       | SQLite 数据表、字段含义、文件存储、数据写入路径                |
| [网站功能说明](./website-features.md)                   | 登录、监测中心、统计分析、事件中心、机器人管理、巡检任务       |
| [板端识别与视频链路](./edge-and-video-pipeline.md)      | `bot-version` 模块、YOLO 检测、抓拍、遥测、RTSP/RTMP/FLV/HLS |
| [bot-version 项目结构与实现分析](./bot_version_eval.md) | `bot-version` 目录结构、功能介绍、核心实现、风险与改进建议     |
| [运维、测试与排障](./operations-and-troubleshooting.md) | 健康检查、常见问题、验证步骤、生产化建议                       |

## 项目一句话说明

本项目是一个智能机器人巡检监测平台演示工程，由 Vue 前端、Django REST 后端、机器人板端检测程序和 ZLMediaKit 视频服务组成。平台支持实时视频查看、机器人状态展示、AI 事件识别、事件复核归档、巡检任务展示、统计分析和板端遥测上报。

## 当前代码模块

```text
Playground/
  backend/       Django + Django REST Framework 后端服务
  frontend/      Vue 3 + Vite 前端工作台
  bot-version/   机器人板端检测、上报、推流程序
  docs/          项目文档与方案记录
```
