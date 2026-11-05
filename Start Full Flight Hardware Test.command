#!/bin/zsh
set -eu
TASK_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
exec "$TASK_DIR/.venv/bin/python" "$TASK_DIR/full_flight_hil.py" "$@"
