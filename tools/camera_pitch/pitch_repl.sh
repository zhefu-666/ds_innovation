#!/bin/sh
exec python3 "$(dirname "$0")/pitch_ctl.py" repl "$@"
