#!/usr/bin/env bash
set -euo pipefail

# Packages required by NX runtime and task diagnostics. Keep this list
# declarative so a new robot does not rely on packages installed by hand.
readonly REQUIRED_APT_PACKAGES=(
  sysstat # provides pidstat
)

missing_packages=()
for package in "${REQUIRED_APT_PACKAGES[@]}"; do
  if ! dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -q '^install ok installed$'; then
    missing_packages+=("$package")
  fi
done

if (( ${#missing_packages[@]} > 0 )); then
  echo "Installing NX system dependencies: ${missing_packages[*]}"
  sudo apt-get update
  sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    "${missing_packages[@]}"
else
  echo "NX system dependencies already installed: ${REQUIRED_APT_PACKAGES[*]}"
fi

if ! command -v pidstat >/dev/null 2>&1; then
  echo "ERROR: pidstat is unavailable after installing sysstat" >&2
  exit 1
fi

if command -v tegrastats >/dev/null 2>&1; then
  echo "Verified task diagnostics: pidstat=$(command -v pidstat), tegrastats=$(command -v tegrastats)"
else
  echo "WARNING: tegrastats is unavailable; install it through the approved NVIDIA JetPack/L4T base image. pidstat metrics remain available." >&2
fi
