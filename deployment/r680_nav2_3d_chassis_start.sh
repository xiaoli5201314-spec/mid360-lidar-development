#!/usr/bin/env bash
set -euo pipefail
[[ "$#" -eq 1 && "${1:-}" == "--arm" ]] ||
  { echo "Usage: $0 --arm" >&2; exit 2; }
exec /root/r680_nav/scripts/r680_nav2_chassis_start.sh --mode 3d --arm
