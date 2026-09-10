#!/usr/bin/env bash
set -euo pipefail

readonly ORT_VERSION="1.23.0"
readonly ORT_INDEX="https://pypi.jetson-ai-lab.io/jp6/cu126"
readonly NUMPY_VERSION="1.26.4"
readonly VISION_VENV="${ROAMERX_VISION_VENV:-/home/dogrobot/runtime/nx-edge/data/vision/venv}"

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "This installer is only for Jetson aarch64 devices." >&2
  exit 1
fi

python3 -m venv --system-site-packages "${VISION_VENV}"

"${VISION_VENV}/bin/python" -m pip install --ignore-installed \
  "numpy==${NUMPY_VERSION}"

"${VISION_VENV}/bin/python" -m pip install \
  --index-url "${ORT_INDEX}" \
  --force-reinstall \
  --no-deps \
  "onnxruntime-gpu==${ORT_VERSION}"

PYTHONNOUSERSITE=1 "${VISION_VENV}/bin/python" - <<'PY'
import cv2
import onnxruntime as ort
from pathlib import Path

providers = ort.get_available_providers()
print(f"onnxruntime={ort.__version__} path={Path(ort.__file__).resolve()} providers={providers}")
if "CUDAExecutionProvider" not in providers:
    raise SystemExit("CUDAExecutionProvider is unavailable")
if "GStreamer:                   YES" not in cv2.getBuildInformation():
    raise SystemExit("OpenCV GStreamer support is unavailable")
print(f"opencv={cv2.__version__} path={Path(cv2.__file__).resolve()} gstreamer=yes")
PY

echo "Vision runtime ready: ${VISION_VENV}"
