#!/bin/zsh
set -eu
cd -- "${0:A:h}"
if [[ ! -x .venv/bin/python ]]; then ./Setup.command; fi
open 'http://127.0.0.1:18183/'
exec .venv/bin/python launch.py --mode live --serve
