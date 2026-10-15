# 外部依赖与历史基线

本仓库主要发布自有集成脚本、参数和补丁，不复制完整上游代码树。

| 依赖 | 历史来源 / 基线 | 范围 |
| --- | --- | --- |
| FAST-LIO ROS 2 | `https://github.com/Ericsii/FAST_LIO_ROS2.git`，`2fffc570a25d0df172720bac034fbdb6a13d2162` | 历史板端记录；本次未连接板端确认 |
| Livox ROS Driver 2 | `https://github.com/Livox-SDK/livox_ros_driver2.git`，`13eb05e4e6dd7a765b934d0c5fd6236676a57b49` | 历史配套记录；MID-360S 还需核对具体驱动版本与设备协议 |
| FAST-LIO ROS 1 参考库 | `https://github.com/hku-mars/FAST_LIO.git`，`7cc4175de6f8ba2edf34bab02a42195b141027e9` | 只作过参考，未复制到本仓库；不可替代 ROS 2 基线 |
| ROS 2 Humble / Nav2 / SLAM Toolbox / OctoMap | 各项目官方发行包 | 运行时外部依赖 |
| 厂商 STM32 底盘工程 | 用户原有板卡资料 | 未再分发；控制帧必须与实际固件匹配 |

`deployment/fastlio_ros2_bounded_lidar_queue.patch` 含对上游代码的差异上下文，应连同相应上游源码和许可使用，不能据此给算法重新授予许可证。应用前在自己的完整 ROS 2 工作树运行 `git apply --check` 并核对基线，再进行编译和回归。

本地原工作区的 ROS 2 快照只有少量源文件，缺构建元数据，本仓库没有将它们拼成“完整可编译源码”。ROS 1 参考库也曾出现 LICENSE 与包元数据许可声明不一致，因此没有复制该树或擅自修订它的许可。

本次未为自有部分选择开源许可证。需要复用或再发布时，请先向仓库所有者确认。
