#!/usr/bin/env bash
set -euo pipefail

[[ "$#" -eq 1 && "${1:-}" == "--arm" ]] || {
  echo "Usage: $0 --arm" >&2
  exit 2
}

exec python3 -u /root/r680_nav/scripts/r680_web_chassis_control.py \
  --arm \
  --advertise-host "${R680_WEB_HOST:-192.0.2.30}"
