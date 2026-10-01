# 下位机 → 上位机：执行器反馈

2026-09-27 更新：`SensorPacket` 为固定 **8 字节**（夹爪完成标志+编号、相机pitch读回角）、小端、1 字节对齐，末尾固定换行符0x0A，便于串口助手按行显示。ToF 尚未安装，四路字段继续保留注释，不在线上传输。IMU 独立接上位机，此帧不包含姿态。

| 偏移 | 类型/字段 | 内容 |
|---|---|---|
| 0 | uint8 start_of_frame | 固定 0xA6 |
| 1 | uint8 gripper_done | 夹爪动作是否执行完成：0未完成，1已完成 |
| 2 | uint8 gripper_action_id | 该标志对应的夹爪动作编号；0=上电后未收到动作 |
| 3–4 | int16 camera_pitch_cdeg | 相机舵机读回的实际pitch，0.01°，0平视、正值向下，低字节在前；正常范围±3500（舵机限位±35°）；`00 80`(-32768)表示读回无效 |
| 5–6 | uint16 crc16 | CRC16/Modbus，低字节在前 |
| 7 | uint8 newline | 固定0x0A（`\n`），不参与CRC |

CRC 覆盖字节 **0..4**（不含帧尾），初值 0xFFFF，多项式 0xA001，无最终异或。旧4字节 `gripper_done` 帧和5字节无相机帧已废弃。

已知校验样例：

- 夹爪未完成(1)、pitch 0°：`A6 00 01 00 00 7D D9 0A`
- 夹爪已完成(1)、向下30°(3000)：`A6 01 01 B8 0B 4F E2 0A`
- 夹爪已完成(2)、向上15°(-1500)：`A6 01 02 24 FA 17 66 0A`

解析器支持拆包、粘包、坏CRC恢复和100ms字节间隔超时；载荷中的A6不视为新帧头；CRC正确但帧尾不是0x0A时整帧丢弃并重新同步。注意CRC或载荷中也可能出现0x0A，不能按换行切帧，必须按帧头+定长+CRC解析。每帧同时回传夹爪完成标志与相机读回角，两者互不覆盖。旧9字节姿态模板不再支持。

有效完整帧发布到独立 `ActuatorFeedback`，通过 `latestActuatorFeedback()` 获取。`valid` 只表示CRC有效且距完整收帧未超过200ms；无帧、过期或关闭串口时为false。过期快照保留原始返回值与时间戳，调用方必须同时检查有效性。若固件仅在动作完成时单次上报，应由动作控制层消费新事件，不应把缓存作为持续心跳。

解析器保留uint8原值；夹爪完成标志只有1视为完成，其他值均按未完成处理，判断见 VELOCITY_PROTOCOL.md“动作编号”。相机到位由 `cameraPitchResult(tolerance_cdeg)` 按读回角判断，见 VELOCITY_PROTOCOL.md“相机pitch”。下位机应以≥20Hz周期上报（相机读回角需持续刷新），200ms未更新即判NoFeedback。

“完成”依赖下位机的判断方式（限位开关、舵机电流或标定延时），需与电控确认；若仅为延时估计，Done不代表机械上一定到位。

`SensorState` 继续为独立IMU和未来ToF提供业务字段，A6解析器不更新该结构，不会将执行器反馈当成IMU有效数据。ToF缺失不代表前后道路安全。

`rescue_sensor_monitor [port] [baud] [seconds]` 现在仅监视A6执行器返回值、时间戳和有效性，不发送运动命令，不是HiPNUC驱动。


独立HiPNUC接收现已实现：见 [HIPNUC_IMU.md](HIPNUC_IMU.md)。使用 `rescue_imu_monitor` 读取 `/dev/ttyUSB0`；本文件描述的 `rescue_sensor_monitor` 仍只用于 `/dev/ttyACM0` 上的A6执行器反馈，两者不能混用。
