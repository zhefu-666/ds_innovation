#!/bin/sh
exec python3 "$(dirname "$0")/gripper_ctl.py" open "$@"
