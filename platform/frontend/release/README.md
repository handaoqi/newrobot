# RoamerX 前端 v1.0.0 离线发布

镜像：`roamerx/platform-frontend:v1.0.0-amd64`，容器仅监听 `127.0.0.1:13000`。

```sh
./frontendctl modules list
./frontendctl up
./frontendctl verify
./frontendctl modules enable maps --ignore-recommended
```

`modules.json` 由 `frontendctl` 原子替换；`scene-map-config.json` 可通过 `ROAMERX_SCENE_MAP_CONFIG` 指向部署环境配置，以启用高德卫星图。公网切换前先完成本机 `verify`，备份并检查两份 Nginx 配置后执行 `nginx -t && systemctl reload nginx`。API、媒体、直播和 TLS 位置不由前端容器接管。
