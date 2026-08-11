# P0 MQTT Broker

中心服务器运行 Mosquitto，设备只需访问中心服务器的 `8883/TCP`（MQTT/TLS）。
`8084/TCP` 是可选 WSS 传输端口，不提供给业务浏览器订阅设备 Topic。

部署前：

1. 将 CA、服务器证书和私钥放到 `certs/ca.crt`、`certs/server.crt`、`certs/server.key`。
2. 用 `mosquitto_passwd -c passwords platform-worker` 创建中心 worker 账号。
3. 每台设备以 `robot_id` 为用户名创建独立账号，例如 `mosquitto_passwd passwords rx-001`。
4. 保护 `passwords` 和私钥，仅允许 Broker 服务账户读取。
5. 防火墙只暴露 TLS 端口；禁止匿名连接和明文 1883。

ACL 将设备限制在自己的 `robots/{robot_id}/...` Topic。中心 worker 可读取上行并写入命令、
同步请求和轨迹应用层 ACK。
