#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "$0")/.."
test -x .venv/bin/python
.venv/bin/python -m pip install -r requirements-rax.txt
for tool in ffmpeg ffprobe bwrap claude; do
  command -v "$tool" >/dev/null || { printf 'Missing system tool: %s\n' "$tool" >&2; exit 1; }
done
if ! command -v socat >/dev/null && [[ ! -x rax_integration/tools/usr/bin/socat ]]; then
  # Task-local extraction: no sudo or changes to the system package database.
  command -v apt-get >/dev/null
  command -v dpkg-deb >/dev/null
  mkdir -p rax_integration/tools/packages
  (
    cd rax_integration/tools/packages
    apt-get download socat
    packages=(socat_*.deb)
    [[ ${#packages[@]} == 1 ]]
    dpkg-deb -x "${packages[0]}" ..
  )
fi
printf 'Task-local RAX dependencies ready. No model requests sent.\n'
