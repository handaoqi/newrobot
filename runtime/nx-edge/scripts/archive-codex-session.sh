#!/usr/bin/env bash
set -euo pipefail

session_id="${1:-${CODEX_THREAD_ID:-}}"
codex_home="${CODEX_HOME:-${HOME}/.codex}"
runtime_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
session_root="${codex_home}/sessions"
archive_root="${runtime_root}/data/codex/sessions"

if [[ -z "${session_id}" ]]; then
  echo "usage: $0 <codex-session-id>" >&2
  exit 2
fi

mapfile -t matches < <(
  find "${session_root}" \( -type f -o -type l \) \
    -name "*${session_id}.jsonl" -print 2>/dev/null
)

if [[ "${#matches[@]}" -ne 1 ]]; then
  echo "expected one Codex session for ${session_id}, found ${#matches[@]}" >&2
  exit 1
fi

source_path="${matches[0]}"
relative_path="${source_path#${session_root}/}"
target_path="${archive_root}/${relative_path}"

if [[ -L "${source_path}" ]]; then
  if [[ "$(readlink -f "${source_path}")" == "${target_path}" ]]; then
    echo "already archived: ${target_path}"
    exit 0
  fi
  echo "refusing to replace unrelated symlink: ${source_path}" >&2
  exit 1
fi

install -d -m 2770 "$(dirname "${target_path}")"
mv "${source_path}" "${target_path}"
chmod 0660 "${target_path}"
ln -s "${target_path}" "${source_path}"

echo "archived=${target_path}"
echo "compatibility_link=${source_path}"
