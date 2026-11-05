#!/bin/zsh
set -eu
cd -- "${0:A:h}"
if [[ ! -x .venv/bin/python ]]; then
    print -u2 -- 'Run Setup.command in this folder before starting the live view.'
    exit 1
fi
# The camera tunnel must already be available at Mac port 18082.
exec .venv/bin/python launch.py --mode live --serve --config live_native_config.json
