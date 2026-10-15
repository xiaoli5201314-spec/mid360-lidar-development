# 部署布局与配置模板

这是集成源码的移植说明，首次检出后不要直接运行真实底盘启动脚本。

## 运行依赖

历史环境为 ROS 2 Humble，外部依赖包括 Nav2、`pointcloud_to_laserscan`、`slam_toolbox`、`topic_tools`、`octomap_server`、Livox ROS Driver 2、FAST-LIO ROS 2。Python 节点按用途使用 ROS 消息包、NumPy、PyYAML 或 pyserial；这些库应在目标 ROS 环境安装。

历史目录约定：

| 公开文件 | 目标布局 |
| --- | --- |
| `deployment/*.py`、`*.sh` | `/root/r680_nav/scripts/` |
| Nav2 / SLAM YAML、行为树 XML | `/root/r680_nav/config/` |
| 本地生成地图 YAML + PGM | `/root/r680_nav/maps/`，成对保留 |
| systemd 示例 | 根据本机账户与工作空间审阅后安装 |
| RViz 视图与启动器 | 使用者自己的 ROS 桌面环境 |

Livox 启动包装器是安装例外：`deployment/mid360-livox-driver-start.sh`
对应服务中的 `/usr/local/bin/mid360-livox-driver-start`，安装时需去掉 `.sh`
后缀并设置执行权限。不要仅把它复制到 `/root/r680_nav/scripts/` 后就启动服务。

`deployment/MANIFEST.md` 是保留的历史角色清单，提到的虚拟机时间准备器或个别视图不一定在此源码子集中；以当前仓库实际文件和本说明为准，不将清单当自动安装脚本。

## 必须本地设置的内容

- `deployment/mid360-livox-driver-start.sh`、相应 `.service`、底盘启动预检中的示例网络端点。
- `.service` 中的 `User=robot`、设备接口和 `/userdata` 工作空间位置。
- 桥接、诊断、网页控制与启停入口中的 `/dev/serial/by-id/YOUR_CHASSIS_SERIAL`。
- SSH 示例的 `~/.ssh/YOUR_DEPLOY_KEY`；仓库不含私钥文件。
- 车辆 URDF、雷达外参、转弯半径、车体尺寸、速度范围、地图 origin 和巡航坐标。

运行前检查所有文件引用是否存在，并对每个 shell 文件逐一执行 `bash -n <file>`。`bash -n scripts/*.sh` 只会将第一个匹配文件作为脚本解析，不能代替逐个检查。

## 验证顺序

1. 运行 `tests` 离线套件、逐个 shell 语法检查和 Python 语法检查。
2. 在底盘写入关闭状态启动传感器/TF，检查消息时戳、频率、坐标系、队列延迟和地图。
3. 从 `r680_nav2_2d_dryrun_start.sh` 观察规划与 `/cmd_vel`，保持实际桥接未解锁。
4. 只有在协议、电源、硬件急停、底盘架空和低速测试就绪后，才另行进行受控 `--arm` 验收。

网页手动控制、多目标客户端和 Nav2 都有不同的控制职责；保持底盘串口只有一个写入者。三维显示不意味着能够进行三维全局路径规划。此发布没有完成或替代这些现场验收。
