#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
PYTHON_BIN="${PYTHON_BIN:-.venv/bin/python}"
case "${1:-}" in
  build) shift; exec "$PYTHON_BIN" build.py "$@" ;;
  validate) shift; exec "$PYTHON_BIN" validate.py "$@" ;;
esac
exec "$PYTHON_BIN" run.py "$@"
