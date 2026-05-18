#!/usr/bin/env bash
set -euo pipefail

export DOG_JUMP_HOST=127.0.0.1
export DOG_JUMP_PORT=60021
export DOG_ORIN_HOST=127.0.0.1
export DOG_ORIN_PORT=60022
export DOG_MVP_PORT="${DOG_MVP_PORT:-8765}"

cd "$(dirname "$0")"
exec python3 server.py
