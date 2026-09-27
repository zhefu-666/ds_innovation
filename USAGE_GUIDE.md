# 地面推行任务接入指南

## 运行方式

先按 README 构建。`--push-replay PATH` 读取 JSON 中的 `frames` 数组，逐帧执行推行任务并打印状态、批次、已交付数及速度指令；不打开摄像头和串口。`--detect-image PATH` 在板子上执行单图 RKNN 推理并打印类别与原图像素框；`--dry-run` 仅进行检测预览。当前实车运动适配器未实现，无上述模式的启动会报错退出。

之前的夹爪、抓球、释放和盲目前冲命令不再是主程序行为。不要用旧根目录或旧 build 中的二进制代替新构建。

## 观测约定

数据结构为 `rescue::PushObservation`。回放布尔值采用整数 0/1；缺失许可字段默认为 false。

| 字段 | 提供者及含义 |
|---|---|
| `now_us` | 单调时间戳，微秒，必须递增；重复、倒序或间隔大于 200 ms 使任务停止并等待 |
| `run`、`reset` | 启停与显式清空任务；run=false 停车且保留交付统计 |
| `safety_ok` | 独立监督模块确认传感器新鲜、通信、倾斜、障碍等检查通过 |
| `target_valid`、`target_id`、`label` | 当前帧有效目标和稳定身份，不得拿缓存目标冒充新观测 |
| `geometry_valid`、`distance_m`、`heading_error` | 经过标定的距离（米）和转向误差（弧度）；不能直接填像素坐标 |
| `path_safe` | 规划和障碍模块确认合法推行路径；不是“未检测到危险目标”就默认为真 |
| `available_count` | 可同批推行的同类目标数量上限；实际批次还需外部跟踪全部成员 |
| `in_push_region` | 目标进入经过标定的推板接触区域 |
| `zone_valid`、`zone_own`、`zone_class` | 有效己方安全区及物资半区 supply / 伤员半区 injured |
| `zone_aligned` | 目标、推板和合法分区入口已对准，不能仅用安全区框中心替代 |
| `fully_inside`、`off_fence`、`stable` | 整批目标完全位于正确半区、不压围栏或隔板、已经静止 |
| `delivered_count` | 外部实际确认的目标数量，必须和锁定批次数相等 |
| `separated` | 机器人/推板已与目标脱离；不是串口断开 |
| `retreat_safe` | 后方路径安全，才允许低速后退 |

目标类别统一为 `ordinary_supply/core_supply/injured_person/dangerous_object`；安全区颜色统一为 `red_safe_zone/blue_safe_zone`。现有模型的 core/wounded/red/dangerous/normal/blue 已显式映射；main 保留为 unmapped_main，禁止参与任务选取。class_id 保留原模型编号，model_label 保留原名称。

## 输出与规则边界

`PushOutput.motion` 使用米/秒和弧度/秒，带 100 ms 有效期。它不是旧 13 字节电机/舵机包，不能直接将浮点速度作为旧电机数值。正式底盘适配器应验证单位、下位机超时停车，并把安全监督放在最终发送前。

本阶段新入口不创建 Controller，不打开串口，不会调用夹爪动作，也不伪造传感器或分区识别结果。相机预览因此会保持等待状态，这是未接入真实规则观测的明确表现。

批次规则已可回放验证；实际多物体推行需要多目标成员管理。安全区入口、隔板、目标轮廓完整性和静止检测仍需感知模块实现。本状态机不能代替比赛全部规则、计时、独立急停或整机现场验收。

## 测试

```bash
ctest --test-dir build-arm64 --output-on-failure
./build-arm64/rescue_upper_host --push-replay tests/fixtures/push_delivery.json
```

测试涵盖初始危险/核心/伤员拒绝、首单一件、后续批次上限、错误分区、交付数量错误、不完整进入、各阶段目标丢失/替换、安全中断、重复/过期观测和超时。回放证据为合成输入，仅用于任务逻辑验证。

## 电控接口（2026-09-25）

发送入口为 `UARTController::sendMotion(const MotionCommand&)`，序列化为15字节小端浮点包（夹爪/相机档位及各自动作编号 + CRC16/Modbus覆盖字节0..12），需与下位机固件同步升级；禁止直接发送普通业务结构体的内存大小。速度单位保持m/s和rad/s，无乘1000转换。旧 `execute()` 返回false，不再发送历史双电机/双舵机包。

接收使用 `latestActuatorFeedback()`，返回 `timestamp_us/gripper_done/gripper_action_id/camera_pitch_cdeg/valid`；仅校验8字节A6执行器反馈（末尾0x0A）。夹爪是否完成用 `gripperActionResult()` 判断，相机是否到位用 `cameraPitchResult(容差)` 判断，不提供IMU数据。ToF仍未安装，字段保留注释。协议字段与CRC样例见 `SENSOR_PROTOCOL.md`、`VELOCITY_PROTOCOL.md`。

虚拟串口测试包含精确收发字节、CRC错误恢复、拆包/粘包与反馈过期，不会打开实际设备。实车主入口仍等待几何、安全和动作流程接入。
