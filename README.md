# 机器狗公园巡检项目

这个仓库用于协作开发机器狗公园巡检系统。当前代码重点覆盖远程连接、ROS2 状态读取、建图、地图查看、单点导航、运动测试和视频预览，下一步要扩展为固定路线自主巡检。

## 当前目录

- `dog_mvp_platform/`：浏览器巡检控制台和 Python 后端。
- `dog_mobile_web_proxy.py`：本地控制台的轻量访问代理，可加 Basic Auth。
- `dog_orin_reverse_tunnel_setup.sh`：部署到 Orin 的反向 SSH 隧道脚本模板。
- `start_dog_remote_control.cmd`：Windows 本地启动远程控制台的脚本。
- `docs/`：协作流程、巡检任务规划和机器人端代码同步建议。

## 本地运行

```powershell
cd "D:\Users\talent\Documents\New project\dog_mvp_platform"
python server.py
```

浏览器访问：

```text
http://127.0.0.1:8765
```

需要远程连接机器狗时，先配置本机环境变量或复制 `.env.example` 为本地私有配置文件。不要把真实密码、密钥、服务器地址访问文档提交到仓库。

## 协作原则

1. GitHub 私密仓库作为代码源头。
2. 每个人在自己的分支开发，通过 Pull Request 合并。
3. 机器狗上的代码不直接手改成唯一版本，改动要回传到仓库。
4. 机器人运行数据、地图大文件、日志、密钥和密码不入库。
5. 厂商安装目录只记录依赖和启动方式，不整包提交。

详细建议见 `docs/COLLABORATION.md`。
