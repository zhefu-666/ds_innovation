#!/bin/sh
exec python3 "$(dirname "$0")/frame_ctl.py" status "$@"
