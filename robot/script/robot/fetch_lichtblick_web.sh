#!/usr/bin/env bash
#
# Vendor the Lichtblick web bundle into the NX runtime install tree.
#
#   fetch_lichtblick_web.sh            fetch + verify + unpack (skips if current)
#   fetch_lichtblick_web.sh --force    re-fetch even if the checksum already matches
#   fetch_lichtblick_web.sh --verify   check what is on disk, download nothing
#
# Lichtblick is the MPL-2.0 community continuation of Foxglove Studio. Upstream's
# own container image is nothing but `caddy file-server` over this exact tarball's
# contents, so the bundle is pure static files: no Docker, no Node, no build step,
# no account. See runtime/nx-edge/install/lichtblick-web/README.md.
set -euo pipefail

SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
INSTALL_ROOT="${INSTALL_ROOT:-/home/dogrobot/runtime/nx-edge/install/lichtblick-web}"
LOCK_FILE="${INSTALL_ROOT}/lichtblick-web.lock"

log() { printf '[lichtblick-fetch] %s\n' "$*" >&2; }
die() { printf '[lichtblick-fetch] ERROR: %s\n' "$*" >&2; exit 1; }

read_lock() {
  [ -f "${LOCK_FILE}" ] || die "lock file not found: ${LOCK_FILE}"
  # Deliberately not sourced: the lock is data, and sourcing it would let a
  # stray line in a vendored file execute as this script.
  VERSION="$(awk -F= '$1=="version"{print $2}' "${LOCK_FILE}")"
  URL="$(awk -F= '$1=="url"{print $2}' "${LOCK_FILE}")"
  SHA256="$(awk -F= '$1=="sha256"{print $2}' "${LOCK_FILE}")"
  [ -n "${VERSION}" ] && [ -n "${URL}" ] && [ -n "${SHA256}" ] || \
    die "lock file is missing version/url/sha256: ${LOCK_FILE}"
  ARCHIVE="${INSTALL_ROOT}/lichtblick-web-${VERSION}.tar.gz"
  DIST="${INSTALL_ROOT}/dist"
}

archive_matches() {
  [ -f "${ARCHIVE}" ] || return 1
  local actual
  actual="$(sha256sum "${ARCHIVE}" | cut -d' ' -f1)"
  [ "${actual}" = "${SHA256}" ]
}

# A bundle that unpacked halfway is worse than no bundle: the page half-loads and
# the failure looks like a layout bug. index.html plus at least one wasm module is
# the cheapest signal that the whole tree landed.
verify_dist() {
  [ -d "${DIST}" ] || { log "no bundle unpacked at ${DIST}"; return 1; }
  [ -f "${DIST}/index.html" ] || { log "bundle at ${DIST} has no index.html"; return 1; }
  local count
  count="$(find "${DIST}" -maxdepth 1 -name '*.wasm' | wc -l)"
  [ "${count}" -gt 0 ] || { log "bundle at ${DIST} has no .wasm modules"; return 1; }
  grep -q 'LICHTBLICK_SUITE_DEFAULT_LAYOUT_PLACEHOLDER' "${DIST}/index.html" || \
    log "warning: index.html has no default-layout placeholder; --default-layout injection will not work"
  log "bundle OK: ${DIST} ($(find "${DIST}" -type f | wc -l) files, ${count} wasm modules)"
  return 0
}

main() {
  local force=0 verify_only=0
  case "${1:-}" in
    --force)  force=1 ;;
    --verify) verify_only=1 ;;
    "")       ;;
    -h|--help) sed -n '3,8p' "$(readlink -f "${BASH_SOURCE[0]}")" | sed 's/^# \{0,1\}//'; return 0 ;;
    *) die "unknown option: $1" ;;
  esac

  read_lock
  log "lichtblick-web ${VERSION}"

  if [ "${verify_only}" -eq 1 ]; then
    verify_dist || die "bundle is missing or incomplete; run ${BASH_SOURCE[0]} without --verify"
    archive_matches && log "archive checksum matches the lock" || log "note: archive absent (fine once unpacked)"
    return 0
  fi

  if [ "${force}" -eq 0 ] && verify_dist >/dev/null 2>&1 && archive_matches; then
    verify_dist
    log "already current; nothing to do"
    return 0
  fi

  mkdir -p "${INSTALL_ROOT}"

  if [ "${force}" -eq 1 ] || ! archive_matches; then
    command -v curl >/dev/null 2>&1 || die "curl is required"
    log "downloading ${URL}"
    # Staged next to the target so the rename is atomic on the same filesystem,
    # and so a failed download never leaves a truncated file that the checksum
    # check would have to catch on the next run.
    local staging_archive="${ARCHIVE}.partial-$$"
    curl -fsSL --retry 3 --retry-delay 2 -o "${staging_archive}" "${URL}" || {
      rm -f "${staging_archive}"
      die "download failed: ${URL}"
    }
    local actual
    actual="$(sha256sum "${staging_archive}" | cut -d' ' -f1)"
    if [ "${actual}" != "${SHA256}" ]; then
      mv "${staging_archive}" "${ARCHIVE}.rejected"
      die "checksum mismatch, refusing to unpack
  expected ${SHA256}
  actual   ${actual}
The rejected file is kept at ${ARCHIVE}.rejected for inspection."
    fi
    mv "${staging_archive}" "${ARCHIVE}"
    log "checksum verified: ${actual}"
  else
    log "archive already present and matching; skipping download"
  fi

  local staging="${INSTALL_ROOT}/.unpack-$$"
  rm -rf "${staging}"
  mkdir -p "${staging}"
  log "unpacking"
  tar xzf "${ARCHIVE}" -C "${staging}"
  [ -f "${staging}/index.html" ] || {
    rm -rf "${staging}"
    die "archive does not contain index.html at its root"
  }
  rm -rf "${DIST}.old"
  [ -d "${DIST}" ] && mv "${DIST}" "${DIST}.old"
  mv "${staging}" "${DIST}"
  rm -rf "${DIST}.old"

  verify_dist || die "unpacked bundle failed verification"
  log "installed to ${DIST}"
  log "serve it with: ${SCRIPT_DIR}/foxglove_demo.sh web"
}

main "$@"
