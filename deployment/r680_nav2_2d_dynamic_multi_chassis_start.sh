#!/usr/bin/env bash
set -euo pipefail
[[ "$#" -eq 1 && "${1:-}" == "--arm" ]] ||
  { echo "Usage: $0 --arm" >&2; exit 2; }
export SCAN_MIN_HEIGHT=-0.35
export SCAN_MAX_HEIGHT=1.20
export SCAN_RANGE_MIN=0.10
export RESTAMP_SCAN_NOW=1
exec /root/r680_nav/scripts/r680_nav2_chassis_start.sh --mode 2d --arm --multi --dynamic-obstacle
