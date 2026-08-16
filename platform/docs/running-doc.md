**1. 启动 ZLMediaKit**

在项目根目录：

```powershell
docker compose -f docker-compose.zlmediakit.yml up -d
```

检查是否启动：

```powershell
docker ps --filter name=playground-zlmediakit
```

**2. 启动后端 Django**

```powershell
.venv\Scripts\python.exe manage.py runserver 127.0.0.1:8000
```

检查后端：

```powershell
curl http://127.0.0.1:8000/api/health/
```

**3. 启动前端 Vue**

新开一个终端：

```powershell
cd frontend
npm run dev
```

访问：

```text
http://127.0.0.1:5173/
```

登录账号：

```text
operator
admin123456
```

**4. 启动视频推流**

新开一个终端：

```powershell
cd bot-version
.\venv\Scripts\python.exe run_stream.py --config config.yaml
```

这一步会把：

```text
rtsp://192.168.234.1:8554/test
```

推到 ZLMediaKit：

```text
rtmp://127.0.0.1/live/dog_ZSL-1A-07_front
```

前端播放地址是：

```text
http://127.0.0.1:8080/live/dog_ZSL-1A-07_front.live.flv
```

**5. 启动识别与上报**

如果你要跑完整“识别 + 抓拍上传 + telemetry 上报”，再开一个终端：

```powershell
cd C:\Users\lzc\Documents\Playground\bot-version
.\venv\Scripts\python.exe run_edge.py --config config.yaml
```

注意：`run_edge.py` 里现在也会启动推流线程。如果你已经单独运行了 `run_stream.py`，就不要再同时让 `run_edge.py` 推同一个流，否则会遇到 `Already publishing`。更简单的完整闭环方式是只运行：

```powershell
.\venv\Scripts\python.exe run_edge.py --config config.yaml
```

**推荐启动组合**

只看视频 MVP：

```text
1. ZLMediaKit
2. Django
3. Vue
4. run_stream.py
```

完整识别闭环：

```text
1. ZLMediaKit
2. Django
3. Vue
4. run_edge.py
```

如果 RTSP 源报错，先单独测：

```powershell
ffmpeg -rtsp_transport tcp -i rtsp://192.168.234.1:8554/test -t 5 -f null -
```