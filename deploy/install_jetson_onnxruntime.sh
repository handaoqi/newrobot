#!/usr/bin/env bash
set -euo pipefail

readonly ORT_VERSION="1.23.0"
readonly ORT_INDEX="https://pypi.jetson-ai-lab.io/jp6/cu126"

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "This installer is only for Jetson aarch64 devices." >&2
  exit 1
fi

python3 -m pip install \
  --index-url "${ORT_INDEX}" \
  --force-reinstall \
  --no-deps \
  "onnxruntime-gpu==${ORT_VERSION}"

python3 - <<'PY'
import onnxruntime as ort

providers = ort.get_available_providers()
print(f"onnxruntime={ort.__version__} providers={providers}")
if "CUDAExecutionProvider" not in providers:
    raise SystemExit("CUDAExecutionProvider is unavailable")
PY
