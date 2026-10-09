#!/usr/bin/env bash
set -euo pipefail
base="$(cd -- "$(dirname -- "$0")" && pwd)"
exec bash "$base/runtime/tools/demo/move_forward.sh" "$@"
