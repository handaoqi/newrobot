#!/usr/bin/env bash
set -euo pipefail

session_id="${1:-}"
codex_home="${CODEX_HOME:-${HOME}/.codex}"
runtime_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
archive_root="${runtime_root}/data/codex/sessions"
session_root="${codex_home}/sessions"

if [[ -z "${session_id}" ]]; then
  echo "usage: $0 <codex-session-id>" >&2
  exit 2
fi

mapfile -t matches < <(
  find "${archive_root}" -type f -name "*${session_id}.jsonl" -print 2>/dev/null
)

if [[ "${#matches[@]}" -ne 1 ]]; then
  echo "expected one archived session for ${session_id}, found ${#matches[@]}" >&2
  exit 1
fi

archive_path="${matches[0]}"
relative_path="${archive_path#${archive_root}/}"
session_path="${session_root}/${relative_path}"

install -d -m 0700 "$(dirname "${session_path}")"
if [[ -e "${session_path}" || -L "${session_path}" ]]; then
  if [[ -L "${session_path}" ]] && \
      [[ "$(readlink -f "${session_path}")" == "${archive_path}" ]]; then
    echo "already restored: ${session_path}"
    exit 0
  fi
  echo "refusing to overwrite existing session: ${session_path}" >&2
  exit 1
fi

ln -s "${archive_path}" "${session_path}"
echo "restored=${session_path}"
echo "resume with: codex resume ${session_id}"
