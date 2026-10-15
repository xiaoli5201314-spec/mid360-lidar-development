# MID-360S 雷达建图与 ROS 2 导航集成开发

面向 RK3588 / ROS 2 Humble 的 MID-360S 开发记录与集成源码：点云转二维地图、二维 SLAM、Nav2 阿克曼约束、串口底盘桥接、多目标导航、实时性预检，以及三维点云 / OctoMap 可视化。

这里的“三维”是二维全局规划配合三维障碍和显示，不是三维全局导航。`r680` 是历史开发脚本前缀；本仓库没有将这些参数当作所有车辆的通用标定。

## 内容

| 路径 | 内容 |
| --- | --- |
| `pcd_to_2d_map.py` | ASCII PCD 到 ROS PGM/YAML 地图，保留未知区域 |
| `deployment/r680_ackermann_bridge.py` | `/cmd_vel` 到串口控制帧、阿克曼约束、超时归零和打开串口控制 |
| `deployment/r680_*nav*`、行为树 XML / YAML | Nav2 启停事务、干运行、固定模式、多目标与动态障碍配置 |
| `deployment/r680_nav_realtime_preflight.py` | 导航相关数据流实时性预检 |
| `deployment/r680_web_chassis_control.py` | 显式解锁的手动网页控制与会话/超时处理 |
| `deployment/*pointcloud*`、`*slam*`、`*octomap*` | 扫描投影、SLAM、三维显示与地图输出 |
| `deployment/fastlio_ros2_bounded_lidar_queue.patch` | 对历史 FAST-LIO ROS 2 基线的有界队列补丁 |
| 根 `r680_*.rviz` / launcher | 配套 RViz 视图 |
| `tests` | 不依赖旧个人目录或厂商 STM32 工程的 6 个离线测试模块 |

## 本地离线验证

不需要 ROS 或连接底盘即可运行所选测试：

```bash
python -m pip install -r requirements-dev.txt
python -B -m pytest -p no:cacheprovider -q tests
```

首次公开整理在 2026-10-15 运行上述命令得到 **76 passed**。测试覆盖地图生成、控制帧/速度约束、多目标队列与控制入口等软件行为；不构成雷达实测、导航精度、串口硬件或实车动态避障验收。

旧工作区包含更多依赖厂家 STM32 工程、父目录工具或原始文件字节哈希的测试，本仓库没有宣称那套完整历史测试可以直接重跑。

## ROS 部署前提

运行环境需另装 ROS 2 Humble、Nav2、Livox 驱动及 FAST-LIO ROS 2，并准备车辆模型、标定、网络和地图。来源与历史版本见 [第三方依赖](THIRD_PARTY_NOTICES.md)，目录映射及参数模板见 [部署说明](docs/DEPLOYMENT.md)。

公开文件中使用的 `192.0.2.10/20/30` 是文档示例端点，`YOUR_CHASSIS_SERIAL`、`YOUR_DEPLOY_KEY` 和服务账户 `robot` 都必须按实际机器设置。保留 `/root/r680_nav`、`/userdata/...` 是为了说明原部署布局，它们不是在任意电脑上可直接执行的安装命令。

建议从 `dryrun` 和离线检查开始。打开底盘写入、`--arm` 或允许非零速度属于另一步受控实车试验，须先核对协议、电平、急停、时间同步和标定。发布过程没有启动导航或发送运动指令。

## 当前边界

仓库保留的是实际集成工作，不包含可独立构建的完整 FAST-LIO 算法源码、厂商 STM32 固件、Livox SDK 包、地图数据和 ROS 系统镜像。历史板端 ROS 2 记录不等于本次重新连接硬件验收；实车路线、漂移、动态障碍与失联停车仍需现场核验。

真实网络地址、串口唯一标识、SSH 私钥文件名和运行账户已模板化。不含个人简历、其他 Holiday 项目、日志、点云地图或截图。

自有部分本次未新增许可证；第三方部分遵循各自原始条款，不统一标为 MIT。
