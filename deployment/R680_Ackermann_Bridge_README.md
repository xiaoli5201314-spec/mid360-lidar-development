# R680 阿克曼底盘串口桥接

> 历史阶段记录，包含旧固件和旧导航配置的排查背景。当前公开范围与部署前提以仓库根 `README.md`、`docs/DEPLOYMENT.md` 和实际脚本参数为准；下文“当前”指记录当时。

## 当前状态

`r680_ackermann_bridge.py` 是 ROS 2 `/cmd_vel` 到 WHEELTEC C50X `Akm_Car` 串口协议的安全桥接骨架。

### 2026-09-03 串口自检触发问题

历史低速测试中，桥接打开串口后约 10 秒出现 `vx≈0.55 m/s`、`wz≈0.35~0.42 rad/s`，随后自动回零。对照 `Akm_Car` 固件可确认：上电约 10 秒会执行厂家自检，期间调用 `Drive_Motor(0.2f,0,0)`。旧版 pyserial 打开 USB 串口时默认 `DTR=True/RTS=True`，可能使 STM32 复位，从而重新触发这段自检。

桥接现已在打开串口前后保持 `DTR=False/RTS=False`，启用串口独占（可用时），并清空输入/输出缓存。该改动只改变串口初始化和占用方式，不改变 STM32 协议，也不会自动发送非零速度。电机重新上电前仍必须完成架空低速测试。

已根据 C50X 工程源码核对：

- 端口：STM32 USART3，115200，8N1；
- 控制帧：11 字节，帧头 `0x7B`，帧尾 `0x7D`；
- 字节 3..8：`Vx、Vy、Vz`，有符号大端 int16，单位为 mm/s 和 mrad/s；
- BCC：字节 0..8 按位异或，放在字节 9；
- STM32 固件内部已经执行 `Akm_Vz_to_Angle(Vx, Vz)`，RK3588 不重复计算舵角；
- 阿克曼车不支持横移，也不支持 `Vx=0` 时的原地旋转。

## 安全模式

默认不打开串口写入：

```bash
python3 /root/r680_nav/scripts/r680_ackermann_bridge.py
```

此模式只订阅 `/cmd_vel` 并打印待发送帧，不向任何设备写数据。

打开串口但仍然只允许零速帧：

```bash
python3 /root/r680_nav/scripts/r680_ackermann_bridge.py \
  --write \
  --device /dev/serial/by-id/YOUR_CHASSIS_SERIAL
```

只有在车轮架空、急停可用、确认设备节点正确后，才允许非零帧。此开关暂不作为日常启动命令：

```bash
python3 /root/r680_nav/scripts/r680_ackermann_bridge.py \
  --write --allow-nonzero \
  --device /dev/serial/by-id/YOUR_CHASSIS_SERIAL \
  --max-linear-x 0.05 --max-angular-z 0.19
```

桥接节点还会以 20 Hz 重发最新指令；超过 0.5 秒未收到 `/cmd_vel` 时自动发送零速帧；退出时发送三次零速帧。

串口写入时应确保系统中只有一个控制程序打开 `/dev/serial/by-id/YOUR_CHASSIS_SERIAL`。串口独占会拒绝第二个写端，避免控制帧交错。

接线后、开启电机电源前，可运行零速诊断：

```bash
python3 /root/r680_nav/scripts/r680_ackermann_serial_diag.py
```

诊断只发送 `7B 00 00 00 00 00 00 00 00 7B 7D`，不会发送运动指令。`RESULT=ZERO_LINK_OK` 表示串口打开参数正确、反馈帧可解析且反馈速度处于静止阈值内；`RESULT=CHECK_NONZERO_FEEDBACK` 或 `RESULT=NO_VALID_FEEDBACK` 时不要开启电机电源，应先排查串口和固件状态。

注意：上面的反馈帧判断适用于仍保留官方遥测发送的固件。当前
`R680_NAV_MINIMAL_V2` 只接收 11 字节控制帧，暂不发送 24 字节反馈帧；在该
固件上运行此诊断得到 `RESULT=NO_VALID_FEEDBACK` 是预期现象，不能据此判断
控制帧未到达。V2 首次验证应以 OLED、串口零帧写入计数、架空 PWM 为零和
失联停车测试为准。

## 从虚拟机部署

当前文件需要从项目目录复制到 RK3588 的 `/root/r680_nav/scripts/`。部署时使用虚拟机中的 SSH 密钥和网络，不使用 Windows 侧密码登录：

```bash
scp -i ~/.ssh/YOUR_DEPLOY_KEY \
  r680_ackermann_bridge.py \
  root@192.0.2.30:/root/r680_nav/scripts/r680_ackermann_bridge.py

ssh -i ~/.ssh/YOUR_DEPLOY_KEY root@192.0.2.30 \
  'chmod 755 /root/r680_nav/scripts/r680_ackermann_bridge.py'
```

部署前先确认 STM32 设备：

```bash
ls -l /dev/serial/by-id/
ls -l /dev/ttyACM0
```

## 尚未完成的部分

- 尚未建立 systemd 开机自启动；
- 尚未把底盘 24 字节反馈积分成正式 `/odom`；
- 尚未进行任何非零速度实车测试；
- 当前 Nav2 DWB 仍可能发布原地旋转命令，需要后续配置阿克曼约束或在桥接层做相应处理。
