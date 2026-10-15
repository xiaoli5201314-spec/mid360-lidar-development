#!/usr/bin/env bash
set -euo pipefail
MAP_ARG="${1:-}"
mapping_started=0
startup_success=0

cleanup_failed_start() {
  local rc=$?
  trap - EXIT INT TERM
  if (( startup_success == 0 && mapping_started == 1 )); then
    /root/r680_nav/scripts/r680_mid360_3d_mapping_stop.sh || rc=1
  fi
  exit "${rc}"
}
trap cleanup_failed_start EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

/root/r680_nav/scripts/r680_mid360_3d_mapping_start.sh
mapping_started=1

PARAMS="/root/r680_nav/config/r680_nav2_3d_params.yaml"
if [[ -n "${MAP_ARG}" ]]; then
  /root/r680_nav/scripts/r680_nav2_dryrun_start.sh "${MAP_ARG}" "${PARAMS}"
else
  /root/r680_nav/scripts/r680_nav2_dryrun_start.sh "" "${PARAMS}"
fi
startup_success=1
