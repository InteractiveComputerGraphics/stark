#!/usr/bin/env bash
set -euo pipefail

WHEEL="${1:-}"
if [[ -z "$WHEEL" ]]; then
  echo "Usage: $0 path/to/stark_sim-*.whl"
  exit 1
fi

TMPENV="/tmp/stark-wheel-test-$(date +%s)"
python -m venv "$TMPENV"
source "$TMPENV/bin/activate"
python -m pip install --upgrade pip
python -m pip install "$WHEEL"
python - <<'PY'
import pystark
print("pystark import OK")
print("module:", pystark.__file__)
PY
