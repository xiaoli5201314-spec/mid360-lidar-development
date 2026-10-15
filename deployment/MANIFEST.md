# R680 MID-360S deployment manifest

This directory is the release staging area. Copy files by target role; do not
run any start script during deployment verification.

## RK3588 scripts

Install executable scripts and Python files into `/root/r680_nav/scripts/`.
The active closure includes:

- `r680_process_stop.sh`
- `r680_check_3588_time_sane.sh`
- `r680_mid360_pointcloud_to_scan.sh` and `_stop.sh`
- `r680_scan_qos_relay.py` (Best Effort sensor scan -> Reliable `/scan` bridge)
- `r680_mid360_slam_toolbox_mapping.sh` and `_stop.sh`
- `r680_mid360_slam2d_start.sh` and `_stop.sh`
- `r680_nav_map_server.sh`, `r680_nav_nav2_stack.sh`, `r680_nav_nav2_stop.sh`
- `r680_nav2_2d_dryrun_start.sh`, `r680_nav2_3d_dryrun_start.sh`,
  `r680_nav2_3d_dryrun_stop.sh`, `r680_nav2_dryrun_start.sh`,
  `r680_nav2_dryrun_stop.sh`, `r680_nav2_dryrun_status.sh`
- `r680_nav2_cmd_vel_watch.sh`, `r680_nav2_cmd_vel_watch_compact.sh`
- `r680_mid360_3d_mapping_start.sh`, `_stop.sh`
- `r680_display_accumulated_cloud.py`, `_start.sh`, `_stop.sh`
- `r680_octomap3d_start.sh`, `_stop.sh`, `_status.sh`, `_save.sh`
- `r680_nav2_2d_chassis_start.sh`, `r680_nav2_3d_chassis_start.sh`,
  `r680_nav2_2d_multi_chassis_start.sh`, `r680_nav2_3d_multi_chassis_start.sh`,
  `r680_nav2_2d_dynamic_multi_chassis_start.sh` (isolated 2D multi-point
  dynamic-obstacle mode),
  `r680_nav2_2d_reverse_dynamic_multi_chassis_start.sh` (isolated 2D
  Reeds-Shepp reverse/dynamic-obstacle multi-point mode),
  `r680_nav2_chassis_start.sh`, `r680_nav2_chassis_stop.sh`,
  `r680_nav2_chassis_status.sh`, `r680_nav_realtime_preflight.py`
- `r680_nav2_chassis_bridge_start.sh`, `r680_nav2_chassis_bridge_stop.sh`,
  `r680_ackermann_bridge.py` (never auto-started; `--arm` remains mandatory)
- `r680_multi_goal_navigator.py`, `r680_multi_goal_navigator.sh` (action-only
  multi-point/cyclic route client; never publishes `/cmd_vel` and never opens
  the chassis serial device)
- `r680_rviz_multi_goal.py`, `r680_rviz_multi_goal.sh` (RViz `2D Goal Pose`
  queue compatible with the official `send_mark.py` workflow; consumes the
  private `/r680_multi_goal_pose` topic, publishes `/waypoints`, submits one
  `NavigateToPose` action at a time, and never owns the chassis interface)
- `r680_rviz_multi_goal_start.sh`, `r680_rviz_multi_goal_stop.sh` (managed by
  the real chassis start/stop transaction only in `--multi` mode; the default
  chassis command remains the tested single-goal `/goal_pose` workflow)
- `r680_patrol_route.example.yaml` (offline route template; replace map-frame
  coordinates before use)
- `r680_web_chassis_control.sh`, `r680_web_chassis_control.py` (manual web
  dead-man controller; never auto-started; requires `--arm`)

Install configuration files into `/root/r680_nav/config/`:

- `r680_mid360_slam_toolbox_params.yaml`
- `r680_nav2_params.yaml`
- `r680_nav2_3d_params.yaml`
- `r680_nav2_dynamic_obstacle_params.yaml`
- `r680_nav2_reverse_dynamic_params.yaml`
- the two baseline Ackermann behavior-tree XML files
- `r680_ackermann_dynamic_nav_to_pose.xml` and
  `r680_ackermann_dynamic_navigate_through_poses.xml`

The dynamic/reverse wrappers use an isolated scan projection profile: the
pointcloud-to-scan height/near-field window is widened for low chair legs and
close obstacles, and the reliable scan relay can restamp delayed FAST-LIO
messages at receive time.  Ordinary single-goal and ordinary multi-goal
profiles keep their previous scan projection settings.

The FAST-LIO ROS 2 source-side latency patch is kept as a reproducible
unified diff and applied inside `/userdata/fastlio_ws/src/FAST_LIO_ROS2` before
rebuilding the board workspace:

- `fastlio_ros2_bounded_lidar_queue.patch`

Install `r680_fastlio_mid360_realtime.yaml` to both FAST-LIO configuration
locations so the source and the service-loaded install copy stay identical:

- `/userdata/fastlio_ws/src/FAST_LIO_ROS2/config/mid360.yaml`
- `/userdata/fastlio_ws/install/fast_lio/share/fast_lio/config/mid360.yaml`

The live map pair is not shipped from this directory. It is generated on the
RK3588 and must remain together in `/root/r680_nav/maps/`:

- `mid360_slam2d_latest.yaml`
- `mid360_slam2d_latest.pgm`

## VM files

Install RViz launchers/configuration and the vehicle model into the `YOUR_USER`
user's home directory. This includes `r680_3588_sync_time_prepare.sh`, the 2D
mapping and navigation launchers, `r680_nav2_dynamic_multi_view_rviz.sh`, 3D
common/navigation views, raw cloud and FAST-LIO view pairs.

## Runtime dependencies (not copied by this manifest)

- ROS 2 Humble under `/opt/ros/humble`
- Livox workspace `/userdata/livox/ros2_ws`
- FAST-LIO workspace `/userdata/fastlio_ws`
- 3D workspace `/userdata/nav2_3d_ws`
- ROS packages: `pointcloud_to_laserscan`, `slam_toolbox`, Nav2,
  `topic_tools`, `octomap_server`
- systemd units: Livox driver, static TF, vehicle TF and FAST-LIO

## Offline verification

```bash
bash -n /root/r680_nav/scripts/*.sh
test -s /root/r680_nav/config/r680_nav2_params.yaml
test -s /root/r680_nav/config/r680_nav2_3d_params.yaml
test -s /root/r680_nav/config/r680_nav2_dynamic_obstacle_params.yaml
test -s /root/r680_nav/config/r680_nav2_reverse_dynamic_params.yaml
test -s /root/r680_nav/maps/mid360_slam2d_latest.yaml
test -s /root/r680_nav/maps/mid360_slam2d_latest.pgm
```

These checks do not prove hardware operation. Real motion is a separate,
supervised acceptance test.
