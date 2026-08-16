#!/bin/bash
# 启动地图管理 API 服务
# 用法: bash start_map_api.sh [端口号，默认8088]

PORT=${1:-8088}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo "  机器狗地图管理 API 服务"
echo "  http://0.0.0.0:${PORT}"
echo "=========================================="

python3 "${SCRIPT_DIR}/map_api_server.py" --port ${PORT} --host 0.0.0.0
