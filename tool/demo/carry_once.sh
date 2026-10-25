#!/usr/bin/env bash
set -euo pipefail
base="$(cd -- "$(dirname -- "$0")" && pwd)"
exec bash "$base/../../tools/demo/carry_once.sh" "$@"
