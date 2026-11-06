#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../../.."
export LD_LIBRARY_PATH="$PWD/validation/planner_comparison/trajopt/runtime${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export MUJOCO_GL=egl
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
exec .venv/bin/python3.11 validation/planner_comparison/trajopt/smoke.py
