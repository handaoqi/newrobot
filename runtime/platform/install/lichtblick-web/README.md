# Lichtblick Web runtime asset

将锁定的 Lichtblick Web 1.28.1 `dist/` 放到本目录。平台前端以只读方式挂载并通过
`/foxglove/` 提供服务。可复用仓库脚本下载到临时目录后复制校验通过的 `dist`：

```bash
INSTALL_ROOT=/opt/roamerx/runtime/platform/install/lichtblick-web \
  /opt/roamerx/source/robot/script/robot/fetch_lichtblick_web.sh
```

运行时无需 Foxglove/Lichtblick 账户，也不会访问云端布局服务。
