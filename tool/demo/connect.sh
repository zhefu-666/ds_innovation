#!/usr/bin/env bash
set -euo pipefail
exec ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 -o ServerAliveCountMax=3 -o HostKeyAlias=192.168.161.7 -L 18080:127.0.0.1:18080 -L 18765:127.0.0.1:18765 cat@192.168.34.7
