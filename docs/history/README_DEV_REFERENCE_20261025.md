# 开发参考（2026-09-25 至 2026-10-04 历史摘录）

> 本文件是 2026-10-25 从 README 迁出的旧阶段段落，保留原始事实。以下描述的接口、脚本和默认值属于对应日期；
> 当前入口以 [README](../../README.md)、[运行与调试](../运行与调试.md) 和 [架构](../ARCHITECTURE.md) 为准。

主入口与模块职责见 [当前架构](../ARCHITECTURE.md)，运行参数见 [运行配置](../RUNTIME_CONFIGURATION.md)。`src/controller.cpp` 是未加入当前构建的历史控制器；`config/rescue.yaml` 是历史参考，当前运行配置使用显式 `--config` JSON。

重构前 README 中的早期状态名、模型尺寸、门控和入口说明已保存在 [历史阶段摘录](README_BEFORE_APP_REFACTOR.md)，不作为当前部署指南。

## 2026-09-25 电控接口更新

此节保留该日期的接口演进记录；当前主程序的硬件运行方式以本文前面的功能边界和应用架构为准。

上位机发送 `56 + float32 vx(m/s) + float32 wz(rad/s) + int8 gripper_offset(+5开/-5关) + uint8 gripper_action_id + int16 camera_pitch_offset(-30..+30°) + int16 imu_yaw_cdeg(0.01°，-32768表示IMU不可用) + CRC16/Modbus`，固定17字节、小端，CRC覆盖字节0..14；下位机反馈 `A6 + gripper_done + gripper_action_id + int16 camera_pitch_cdeg(读回) + CRC16/Modbus`，固定8字节（末尾0x0A换行），CRC覆盖字节0..4；上位机用 `gripperActionResult()` 按编号确认夹爪完成，用 `cameraPitchResult()` 按读回角确认相机到位。ToF字段保留注释，IMU状态独立。

调用 `UARTController::sendMotion(command)` 发送，使用 `latestActuatorFeedback()` 读取反馈。旧无参数 `execute()` 不再发送旧双电机/双舵机协议。详情见 [速度协议](../../VELOCITY_PROTOCOL.md) 和 [反馈协议](../../SENSOR_PROTOCOL.md)。本次接口改动不开放主程序实车模式，也不构成执行器动作闭环。


### 终端手动控制下位机（不启动主程序）

夹爪开关可直接用脚本，自动读取当前动作编号并加1，重复发送直到下位机报完成（默认3s超时）：

```bash
tools/gripper/gripper_status.sh   # 查看当前动作编号/完成标志/pitch
tools/gripper/gripper_open.sh     # 张开
tools/gripper/gripper_close.sh    # 合上
```

可加 `--dry-run` 只打印将发送的包、`--port`/`--baud`/`--timeout` 修改参数。需要同时控制速度或相机时用下面的通用脚本。

相机pitch调整用 `tools/camera_pitch/`，夹爪保持张开。脚本先读A6反馈拿到当前动作编号，用“编号+1”发一次张开并等done=1，之后每包都沿用该编号和 `gripper_open=1`，调pitch不会再触发夹爪动作。角度单位为度，0平视、正值向下、负值向上，超出舵机限位时截断（下位机已放宽到±35°，脚本仍按±25°截断）：

```bash
tools/camera_pitch/pitch_status.sh     # 查看动作编号/夹爪开关状态/pitch读回（不发包）
tools/camera_pitch/pitch_set.sh 80     # 张开夹爪，相机向下20°；直接输入真实角度
tools/camera_pitch/pitch_set.sh -5     # 相机向上5°；新协议限位±30°
tools/camera_pitch/pitch_repl.sh       # 交互模式：终端输入角度实时调整
```

交互模式下直接输入角度（如 `10`、`-5`、`0`）即可转动，`s` 查看读回角，`x` 打印当前包，`q` 或 Ctrl+C 退出。后台以20Hz持续发当前目标，速度固定为0；退出后下位机200ms超时停车，相机保持当前角度。

`pitch_set.sh` 在读回角进入容差（默认3°，`--tol` 修改）后再保持发送0.5s，`--timeout`（默认3s）内未到位则返回非0。同样支持 `--dry-run`、`--port`、`--baud`。读回精度约1~3°，不要把读回角当作精确外参使用，见 [速度协议](../../VELOCITY_PROTOCOL.md)“相机pitch”。

先断开网页串口助手、`rescue_upper_host --hardware` 等占用 `/dev/ttyACM0` 的程序（主程序独占该串口）。参数依次为：vx(m/s)、wz(rad/s)、夹爪(0关/1开)、动作编号(1..255)、pitch(0.01°，正值向下，舵机限位±2500)。脚本只发一包并打印下位机回包；速度非0时车只动一下，200ms后下位机超时停车。

```bash
python3 - 0 0 1 1 0 <<'EOF'
import os, sys, struct, termios, tty, time
vx, wz = float(sys.argv[1]), float(sys.argv[2])
g, gid, pitch = int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])
pitch = max(-2500, min(2500, pitch))  # 舵机限位±25°
def crc(b):
    c = 0xFFFF
    for x in b:
        c ^= x
        for _ in range(8): c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c
b = struct.pack('<BffBBh', 0x56, vx, wz, g, gid, pitch)
pkt = b + struct.pack('<H', crc(b))
print('TX', pkt.hex(' ').upper())
fd = os.open('/dev/ttyACM0', os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
tty.setraw(fd); a = termios.tcgetattr(fd); a[4] = a[5] = termios.B115200; termios.tcsetattr(fd, termios.TCSANOW, a)
os.write(fd, pkt); time.sleep(0.3)
try: print('RX', os.read(fd, 64).hex(' ').upper())
except BlockingIOError: print('RX 无数据')
os.close(fd)
EOF
```

只需改第一行参数。夹爪每次换状态动作编号都要加1，编号不变下位机按重复包忽略。

| 第一行参数 | 效果 | 对应字节 |
|---|---|---|
| `0 0 1 1 0` | 夹爪张开，编号1 | `56 00 00 00 00 00 00 00 00 01 01 00 00 9A 41` |
| `0 0 0 2 0` | 夹爪合上，编号2 | `56 00 00 00 00 00 00 00 00 00 02 00 00 6B BD` |
| `0 0 0 2 2000` | 夹爪保持合上，相机向下20° | — |
| `0 0 0 2 -2000` | 相机向上20° | — |

回包 `A6 <done> <编号> <pitch低> <pitch高> <CRC低> <CRC高> 0A`，done=1且编号等于最新编号才表示夹爪动作完成。

### 终端发送速度脚本

`tools/send_velocity.py` 默认使用当前 17 字节运动协议，预览模式不会打开串口：

```bash
python3 tools/send_velocity.py --port /dev/ttyACM0 --vx 0.10 --wz 0
```

确认端口和机械状态后，添加 `--send` 才会实际发送。默认 25 Hz 发送 1 秒，结束时补发零速度停车帧：

```bash
python3 tools/send_velocity.py \
  --port /dev/ttyACM0 --baud 115200 \
  --vx 0 --wz 10 --gripper closed --action-id 0 \
  --duration 1 --send
```

默认退出时补发零速度停车帧；需要自行控制是否停车时可显式选择：

```bash
python3 tools/send_velocity.py --no-stop-on-exit --vx 0 --wz 10 --send
python3 tools/send_velocity.py --stop-on-exit --vx 0 --wz 0 --send
```

持续发送直到 `Ctrl+C`：

```bash
python3 tools/send_velocity.py --port /dev/ttyACM0 \
  --vx 0 --wz 0.5 --duration 0 --send
```

当前协议中 `--gripper open` 为 `1`、`--gripper closed` 为 `0`；夹爪状态变化时使用新的 `--action-id`（1..255），`0` 表示不触发新动作。`--pitch` 使用内部偏移真实度数，限幅 `±30°`；A6 回传为绝对角 `90..150°`，`120°` 为零点。发送模式会监听现有 A6 执行器反馈并显示夹爪状态、动作编号和 pitch；下位机不回传速度帧，因此终端不能显示实测 `vx/wz`，只能显示发送命令。使用前应停止占用 `/dev/ttyACM0` 的其他程序。

旧六字节协议可用 `--protocol legacy6` 显式选择；不要将两种协议混用。

## HiPNUC IMU 接收（2026-09-26）

已接入 HI91 接收、CRC 校验、SI 单位转换、新鲜度检查及网页/Foxglove 数据显示。详细包格式、数据接口和故障边界见 [IMU 接入](../IMU接入.md)。

```bash
# 只看IMU，无需相机/模型，不发送控制字节
./build-arm64-telemetry/rescue_imu_monitor /dev/ttyUSB0 115200 30

# 在真实相机预览中接收IMU，网页增加IMU数据卡片
python3 tools/remote_camera_telemetry/manage_project.py start --imu --imu-port /dev/ttyUSB0 --imu-baud 115200
```

两个命令二选一，不能同时读取同一个IMU串口。已运行预览时，先执行管理脚本的 `stop` 再带上述参数启动。默认不启用IMU，避免未接设备时影响原有预览。`--dry-run --imu` 只允许IMU接收，不启用下位机运动输出。


### 查看速度脚本收到的下位机原始数据

在原有发送命令末尾增加 `--show-rx`：

```bash
python3 tools/send_velocity.py --vx 0 --wz 0 --duration 2 --send --show-rx
```

将 vx/wz 换成已验证的测试值即可。默认退出补发零速度帧。
`RX RAW` 是每次串口实际读取的字节块，可能包含半帧或多帧；
`RX A6 [CRC OK]` 是校验通过的完整8字节反馈，随后显示夹爪状态、动作编号与pitch。
结束时汇总收到的字节数和有效帧数；不加参数保持原有状态变化显示。
当前A6不包含实际线速度或角速度；此参数只显示已有回包，不能让固件自动增加速度反馈。


## 2026-10-04：相机+IMU受控空场

用户确认的仅己方安全区、前后范围清空、无人/对手进入的封闭测试场景，使用
`bash tools/match/start_controlled_empty_field.sh --check` 进行不接触硬件的配置检查。
具体预览、等待开始、停止命令及边界见 [CONTROLLED_FIELD_20261004.md](CONTROLLED_FIELD_20261004.md)。
每趟最多两个物资；0.32米为车体旋转半径，投放包络单独定义。此模式不做全局定位，
不把人工清场说成自主避障；按用户要求不将MCU独立断联停车列为本次待办。


### 受控空场开局与找物更新

`start_controlled_empty_field.sh` 现在先累计下发40秒、0.2米/秒的前进指令，搜索及投放后回转为1弧度/秒。
没有普通物资时，`--search-cues` 可用伤员/蓝色/核心模型物块作方向线索，
闭爪低速短推后停车重新搜索，最多两次；不授予首趟非普通物资搬运许可。
参数、停止条件和指令距离与实测距离的区别见 `CONTROLLED_FIELD_20261004.md`。
