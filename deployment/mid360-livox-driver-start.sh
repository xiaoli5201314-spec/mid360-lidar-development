#!/usr/bin/env bash
set -eo pipefail

LIDAR_IF="${MID360_LIDAR_IF:-eth0}"
HOST_IP="${MID360_HOST_IP:-192.0.2.10}"
LIDAR_IP="${MID360_LIDAR_IP:-192.0.2.20}"
WAIT_TIMEOUT="${MID360_WAIT_TIMEOUT:-60}"
STARTUP_TOPIC_TIMEOUT="${MID360_STARTUP_TOPIC_TIMEOUT:-25}"
TOPIC_CHECK_PERIOD="${MID360_TOPIC_CHECK_PERIOD:-10}"
TOPIC_MISS_LIMIT="${MID360_TOPIC_MISS_LIMIT:-3}"

log() {
  printf '[mid360-livox-start] %s %s\n' "$(date '+%F %T')" "$*"
}

deadline=$((SECONDS + WAIT_TIMEOUT))

until ip -4 addr show dev "${LIDAR_IF}" 2>/dev/null | grep -q "${HOST_IP}/"; do
  if (( SECONDS >= deadline )); then
    log "ERROR: ${LIDAR_IF} did not get ${HOST_IP}/${WAIT_TIMEOUT}s"
    exit 1
  fi
  log "waiting for ${LIDAR_IF} to have ${HOST_IP}"
  sleep 2
done

until ping -c 1 -W 1 "${LIDAR_IP}" >/dev/null 2>&1; do
  if (( SECONDS >= deadline )); then
    log "ERROR: lidar ${LIDAR_IP} is not reachable after ${WAIT_TIMEOUT}s"
    exit 1
  fi
  log "waiting for lidar ${LIDAR_IP}"
  sleep 2
done

source /opt/ros/humble/setup.bash
source /userdata/livox/ros2_ws/install/setup.bash

log "starting livox_ros_driver2 for MID-360S"
ros2 launch livox_ros_driver2 mid360s_pointcloud_driver_launch.py &
launch_pid=$!

cleanup() {
  if kill -0 "${launch_pid}" >/dev/null 2>&1; then
    kill "${launch_pid}" >/dev/null 2>&1 || true
    wait "${launch_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup INT TERM

topic_deadline=$((SECONDS + STARTUP_TOPIC_TIMEOUT))
until timeout 2 ros2 topic list 2>/dev/null | grep -Fxq '/livox/lidar'; do
  if ! kill -0 "${launch_pid}" >/dev/null 2>&1; then
    wait "${launch_pid}" || true
    log "ERROR: livox launch exited before /livox/lidar appeared"
    exit 1
  fi
  if (( SECONDS >= topic_deadline )); then
    log "ERROR: /livox/lidar did not appear after ${STARTUP_TOPIC_TIMEOUT}s; restarting service"
    cleanup
    exit 1
  fi
  sleep 1
done

log "/livox/lidar is publishing; entering watchdog"
miss_count=0
while kill -0 "${launch_pid}" >/dev/null 2>&1; do
  sleep "${TOPIC_CHECK_PERIOD}"
  if timeout 3 ros2 topic list 2>/dev/null | grep -Fxq '/livox/lidar'; then
    miss_count=0
  else
    miss_count=$((miss_count + 1))
    log "WARN: /livox/lidar missing (${miss_count}/${TOPIC_MISS_LIMIT})"
    if (( miss_count >= TOPIC_MISS_LIMIT )); then
      log "ERROR: /livox/lidar stayed missing; restarting service"
      cleanup
      exit 1
    fi
  fi
done

wait "${launch_pid}"
