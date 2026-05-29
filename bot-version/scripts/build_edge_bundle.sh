#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_VENV="${ROOT_DIR}/.build-venv"
DIST_DIR="${ROOT_DIR}/dist"
APP_NAME="bike-bot-edge"
BUNDLE_DIR="${DIST_DIR}/${APP_NAME}-bundle"
ARCH="$(uname -m)"
PACKAGE_PATH="${DIST_DIR}/${APP_NAME}-${ARCH}.tar.gz"

cd "${ROOT_DIR}"

if [ ! -f "models/bike.onnx" ]; then
  echo "models/bike.onnx is required. Export the model before packaging." >&2
  exit 1
fi

python3 -m venv "${BUILD_VENV}"
"${BUILD_VENV}/bin/python" -m pip install --upgrade pip
"${BUILD_VENV}/bin/python" -m pip install -r requirements-packaging.txt

rm -rf "${DIST_DIR:?}/${APP_NAME}" "${BUNDLE_DIR}" "${PACKAGE_PATH}"

"${BUILD_VENV}/bin/pyinstaller" \
  --clean \
  --noconfirm \
  --name "${APP_NAME}" \
  --paths "${ROOT_DIR}/src" \
  "${ROOT_DIR}/run_edge.py"

mkdir -p "${BUNDLE_DIR}"
cp -a "${DIST_DIR}/${APP_NAME}/." "${BUNDLE_DIR}/"
mkdir -p "${BUNDLE_DIR}/models" "${BUNDLE_DIR}/snapshots" "${BUNDLE_DIR}/data/telemetry" "${BUNDLE_DIR}/bin"
cp "models/bike.onnx" "${BUNDLE_DIR}/models/bike.onnx"
cp "config.yaml" "${BUNDLE_DIR}/config.yaml"

if [ -f "packaging/bin/ffmpeg" ]; then
  cp "packaging/bin/ffmpeg" "${BUNDLE_DIR}/bin/ffmpeg"
  chmod +x "${BUNDLE_DIR}/bin/ffmpeg"
fi

cat > "${BUNDLE_DIR}/start.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${APP_DIR}"
exec "${APP_DIR}/bike-bot-edge" --config "${APP_DIR}/config.yaml"
EOF
chmod +x "${BUNDLE_DIR}/start.sh"

tar -czf "${PACKAGE_PATH}" -C "${DIST_DIR}" "$(basename "${BUNDLE_DIR}")"

echo "Built ${PACKAGE_PATH}"
