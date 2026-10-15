#!/usr/bin/env bash
set -u

failures=0

check_file() {
  local path="$1"
  if [[ -f "${path}" ]]; then
    echo "[OK] file: ${path}"
  else
    echo "[FAIL] missing file: ${path}" >&2
    failures=$((failures + 1))
  fi
}

check_exec() {
  local path="$1"
  if [[ -x "${path}" ]]; then
    echo "[OK] executable: ${path}"
  else
    echo "[FAIL] missing executable: ${path}" >&2
    failures=$((failures + 1))
  fi
}

check_ros_pkg() {
  local pkg="$1"
  set +u
  source /opt/ros/humble/setup.bash >/dev/null 2>&1
  local source_rc=$?
  set -u
  if [[ "${source_rc}" -eq 0 ]] && ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
    echo "[OK] ROS package: ${pkg}"
  else
    echo "[FAIL] missing ROS package: ${pkg}" >&2
    failures=$((failures + 1))
  fi
}

check_yaml_map() {
  local yaml="$1"
  if [[ ! -f "${yaml}" ]]; then
    echo "[FAIL] map yaml missing: ${yaml}" >&2
    failures=$((failures + 1))
    return
  fi

  local image
  image="$(awk -F': *' '$1 == "image" {print $2; exit}' "${yaml}")"
  if [[ -z "${image}" ]]; then
    echo "[FAIL] map yaml has no image entry: ${yaml}" >&2
    failures=$((failures + 1))
    return
  fi

  local image_path
  if [[ "${image}" = /* ]]; then
    image_path="${image}"
  else
    image_path="$(dirname "${yaml}")/${image}"
  fi

  if [[ -f "${image_path}" ]]; then
    echo "[OK] map yaml+image: ${yaml} -> ${image_path}"
  else
    echo "[FAIL] map image missing: ${image_path}" >&2
    failures=$((failures + 1))
  fi
}

nav_map="/root/r680_nav/maps/mid360_slam2d_latest.yaml"

echo "=== R680 Nav2 dry-run asset check ==="
date '+%F %T %Z'

check_file /root/r680_nav/config/r680_nav2_params.yaml
check_file /root/r680_nav/config/r680_nav2_3d_params.yaml
check_yaml_map "${nav_map}"

check_exec /root/r680_nav/scripts/r680_process_stop.sh
check_exec /root/r680_nav/scripts/r680_mid360_slam2d_stop.sh
check_exec /root/r680_nav/scripts/r680_mid360_pointcloud_to_scan.sh
check_exec /root/r680_nav/scripts/r680_mid360_pointcloud_to_scan_stop.sh
check_exec /root/r680_nav/scripts/r680_nav_map_server.sh
check_exec /root/r680_nav/scripts/r680_nav_nav2_stack.sh
check_exec /root/r680_nav/scripts/r680_nav2_2d_dryrun_start.sh
check_exec /root/r680_nav/scripts/r680_nav2_3d_dryrun_start.sh
check_exec /root/r680_nav/scripts/r680_nav2_dryrun_start.sh
check_exec /root/r680_nav/scripts/r680_nav2_dryrun_stop.sh
check_exec /root/r680_nav/scripts/r680_nav2_dryrun_status.sh
check_exec /root/r680_nav/scripts/r680_nav2_cmd_vel_watch.sh

check_ros_pkg nav2_bringup
check_ros_pkg nav2_map_server
check_ros_pkg nav2_controller
check_ros_pkg nav2_planner
check_ros_pkg nav2_bt_navigator
check_ros_pkg nav2_lifecycle_manager
check_ros_pkg pointcloud_to_laserscan
check_ros_pkg slam_toolbox
check_ros_pkg topic_tools
check_ros_pkg octomap_server

if [[ "${failures}" -gt 0 ]]; then
  echo "=== FAIL: ${failures} problem(s) found ===" >&2
  exit 1
fi

echo "=== PASS: Nav2 dry-run assets look ready ==="
