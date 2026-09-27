# HiPNUC HI91 接收与项目使用

适配依据：本板 `/dev/ttyUSB0` 的真实采样，115200、8N1、100Hz；官方 `hipnuc/products` 提交 `2676ffb1214fc5eb80e1410c5b217adda1569035` 中 `c/hipnuc/hipnuc_dec.c/.h` 的HI91布局与CRC规则。只支持已经验证的0x91载荷；不把0x81/0x83或A6执行器反馈误解析成姿态。

## 线上数据包

6字节头 + 76字节载荷，整帧82字节，小端：

| 整帧偏移 | 类型 | 内容 |
|---|---|---|
| 0..1 | 2字节 | 5A A5 |
| 2..3 | uint16 | 载荷长度76，即4C 00 |
| 4..5 | uint16 | CRC16/XMODEM，低字节在前 |
| 6 | uint8 | 类型0x91 |
| 7..8 | uint16 | 状态原值 |
| 9 | int8 | 温度°C |
| 10..13 | float32 | 气压Pa |
| 14..17 | uint32 | 设备时间ms |
| 18..29 | float32[3] | 加速度g |
| 30..41 | float32[3] | 角速度°/s |
| 42..53 | float32[3] | 磁场µT |
| 54..65 | float32[3] | roll/pitch/yaw，° |
| 66..81 | float32[4] | 四元数w,x,y,z |

CRC初值0、多项式0x1021、无最终异或；覆盖整帧0..3和6..81，跳过CRC自身。`Hi91Parser` 按字节偏移读取，不直接强转有对齐要求的结构体。

## 接收数据结构

`include/rescue/hipnuc_imu.hpp` 定义 `ImuSample` 和 `ImuSnapshot`。`HipnucImu::snapshot()` 线程安全地返回最新一份测量，不等待串口读操作：

- `acceleration_mps2`：线上g乘9.8，包含重力相关比力，不能直接作为去重力线加速度积分。
- `angular_velocity_rps`、`rpy_rad`：°转rad；数组顺序分别x/y/z与roll/pitch/yaw。
- `quaternion_wxyz`、`magnetic_ut`、温度、气压、`status`：保留设备定义。
- `device_time_ms`：设备自己的时间；`received_us`：主机steady clock收到推进时间戳时的时间，两者不能直接相减。
- `sequence`、CRC/异常计数、`age_ms`、`fresh`：用于观测新鲜度及故障诊断。
- `measurements_valid`：只表示数值有限且四元数模长约为1，不代表姿态收敛、标定有效或状态位无告警。

## 安装变换（2026-09-27）

本车IMU倒装：设备坐标为FRD（X前、Y右、Z下），相对车体 `base_link`（FLU：X前、Y左、Z上）绕X轴旋转180°。依据 `telemetry_logs/imu-orient-20260927/motion3.jsonl` 实车手动转动采样：

| 动作 | 设备原始数据 | 修正后车体数据 |
|---|---|---|
| 水平静止 | acc_z≈-1.00g，roll≈179.5° | acc_z≈+1.00g，roll≈+1°、pitch≈+0.5° |
| 车头抬高约25° | acc_x≈+0.44g，roll从179.5°跳到-155° | pitch≈-25°（REP-103，车头下俯为正） |
| 原地左转约80° | gyro_z积分-80° | gyro_z积分+80°，yaw增大 |

`applyImuMounting()` 在解析时填写 `body_` 字段，公式：

```
acc_body  = (ax, -ay, -az)
gyro_body = (gx, -gy, -gz)
q_body    = q_device ⊗ (0,1,0,0) = (-qx, qw, qz, -qy)   // w,x,y,z
body_rpy  = q_body 的ZYX分解（roll/pitch/yaw）
```

设备原始字段保持不变，遥测里仍为 `frame_id=imu_device`；修正字段带 `body_frame_id=base_link`。设备自带的欧拉角在倒装时roll停在±180°附近，微小晃动即跳变，任何倾斜判断必须使用 `body_rpy_rad`，不要使用 `rpy_rad`。

尚未完成的验证：侧倾动作采样未成功（该段实际是前后倾），Y轴方向由右手系推导，未经独立侧倾实测；`body_rpy_rad[2]` 仅为相对航向。固件状态位含义仍未确认，因此遥测 `attitude_valid_for_control` 保持false，也不写入 `SensorState`。IMU若重新安装，必须重新采样并更新此变换。

## 数据如何被使用

1. 独立串口线程持续接收，100Hz更新最新快照；主线程推理不会阻塞串口接收。
2. `main.cpp` 开启 `--imu` 后读取快照，终端打印姿态、新鲜度、状态字和CRC错误。
3. IMU过期或数值无效时可否决已有 `safety_ok`；IMU正常绝不会把缺失的路径、区域、几何许可提升为有效。当前任务仍为预览，运动输出关闭。
4. 遥测快照添加 `imu` 段，网页 `/status` 和IMU卡片展示数据；Foxglove新增 `/imu/data`。桥接每次按主机单调时钟重新计算IMU年龄，不能用新相机画面掩盖已过期IMU。
5. 状态字0x2323按原值显示。当前官方SDK的bit5解释为ACC_SAT，但型号/固件未知，因此接收有效不能等同于姿态健康；控制接入需核实状态位、安装变换和动态响应。

## 使用

```bash
cd /home/cat/ds_innovation
cmake -S . -B build-arm64-telemetry -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DRESCUE_ENABLE_TELEMETRY=ON
cmake --build build-arm64-telemetry -j2
./build-arm64-telemetry/rescue_imu_monitor /dev/ttyUSB0 115200 30
```

`fresh=1 numeric_ok=1` 表示有新鲜且数值有效的消息；退出码0表示结束时满足此条件，2表示未收到/过期/数值无效，1表示参数或串口错误。监视器限频打印，接收仍为设备原始频率。Ctrl+C退出。

同时查看检测和IMU（二选一，不要与独立监视器同时占用USB0）：

```bash
python3 tools/remote_camera_telemetry/manage_project.py start --imu --imu-port /dev/ttyUSB0 --imu-baud 115200
python3 tools/remote_camera_telemetry/manage_project.py status
# 网页：http://192.168.1.123:8080/
# Foxglove：ws://192.168.1.123:8765，订阅 /imu/data
python3 tools/remote_camera_telemetry/manage_project.py stop
```

直接主程序入口：

```bash
./build-arm64-telemetry/rescue_upper_host --dry-run --no-show --telemetry --imu --imu-port /dev/ttyUSB0 --imu-baud 115200 --imu-timeout-ms 200
```

串口为8N1无流控、禁回显，只接收不写入指令，退出后保留115200原始模式。独占锁拒绝第二个本工具实例；启动前应退出其他串口阅读器。当前节点cat账号可访问，若权限变化使用合适串口组或sudo。

## 失效与验证边界

CRC失败、未知载荷、非法长度不刷新测量；半包超过100ms丢弃。重复设备时间不刷新，倒退时间拒绝；uint32自然回绕可接受。设备重启或UTC同步导致时间倒退时，需要重启接收进程建立新时间基准，不将旧数据自动当成新数据。

默认200ms未收到推进帧即过期；断开标记连接失效并退出接收线程，主程序可继续预览且IMU显示失效。当前不做自动重连，重插后重启接收程序。数值非法的新帧会覆盖旧快照并标记无效，不使用旧健康值掩盖故障。

测试：`rescue_imu_tests`使用真实采样金标准、噪声/CRC/拆包/粘包/半包超时、非法数值和伪终端验证时效、重复/倒退/回绕、拔出、无发送和端口独占。真实fixture来自2026-09-26采样，并经官方解码器校验。桥接测试验证IMU独立时效；遥测测试验证SI字段传递。

本模块不替代安装标定、姿态动态响应验证、固件状态确认、车体融合及运动闭环。
