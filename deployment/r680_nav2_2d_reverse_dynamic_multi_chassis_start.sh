#!/usr/bin/env bash
set -euo pipefail

[[ "$#" -eq 1 && "${1:-}" == "--arm" ]] || {
  echo "Usage: $0 --arm" >&2
  exit 2
}

# Dynamic profile only: retain low chair legs/near-field returns and refresh
# delayed scan stamps before Nav2's obstacle layer consumes them.  Ordinary
# single/multi-point profiles keep their previously verified scan settings.
export SCAN_MIN_HEIGHT=-0.35
export SCAN_MAX_HEIGHT=1.20
export SCAN_RANGE_MIN=0.10
export RESTAMP_SCAN_NOW=1

echo "Starting isolated reverse/dynamic multi-point profile."
echo "Profile: /root/r680_nav/config/r680_nav2_reverse_dynamic_params.yaml"
echo "Planner: REEDS_SHEPP, minimum turning radius: 1.40 m, reverse enabled."
exec /root/r680_nav/scripts/r680_nav2_chassis_start.sh \
  --mode 2d --arm --multi --dynamic-obstacle --reverse
