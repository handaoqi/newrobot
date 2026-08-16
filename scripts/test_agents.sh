#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$REPO_ROOT/edge-agent:$REPO_ROOT/dev-agent${PYTHONPATH:+:$PYTHONPATH}"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
exec python3 -m pytest --import-mode=importlib -q \
  "$REPO_ROOT/edge-agent/tests" \
  "$REPO_ROOT/dev-agent/tests"
