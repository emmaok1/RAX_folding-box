#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
case "${1:-}" in
  rax-teacher) shift; exec .venv/bin/python -m rax_integration.teacher.run "$@" ;;
  rax-fold) shift; exec .venv/bin/python -m rax_integration.fold "$@" ;;
  rax-check) shift; exec .venv/bin/python -m rax_integration.launch check "$@" ;;
  rax-prepare) shift; exec .venv/bin/python -m rax_integration.launch prepare "$@" ;;
  rax-fixture) shift; exec .venv/bin/python -m rax_integration.launch fixture "$@" ;;
  rax-smoke) shift; exec .venv/bin/python -m rax_integration.launch smoke "$@" ;;
esac
exec .venv/bin/python run.py "$@"
