#!/bin/zsh
set -eu
cd -- "${0:A:h}"
python3 -c 'import sys; assert sys.version_info >= (3,9), "Python 3.9 or newer required"'
if [[ ! -x .venv/bin/python ]]; then python3 -m venv .venv; fi
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m unittest discover -s tests -v
